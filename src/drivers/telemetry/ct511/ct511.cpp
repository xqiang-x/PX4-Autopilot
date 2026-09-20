/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ct511.cpp
 *
 * DX-CT511 4G modem driver - TCP transparent mode (透传).
 *
 * The modem attaches to a board UART (default TELEM1 = /dev/ttyS5 on
 * fmu-v6c, see CT511_DEV param) and opens ONE TCP connection to a relay
 * server (CT511_HOST:CT511_PRT) in transparent mode: everything written
 * to the modem UART goes to the socket and vice versa.  MAVLink bytes
 * are therefore piped raw (no hex encoding, no AT traffic in the data
 * path).
 *
 * This driver registers a virtual serial device /dev/ct511 that the
 * mavlink module attaches to (`mavlink start -d /dev/ct511`). MAVLink
 * bytes written into the virtual device are drained to the modem UART
 * (uplink), bytes received from the modem are pushed into the virtual
 * device (downlink).
 *
 * Link establishment (manual §3.1.2.1 "TCP 透传", §5.4):
 *   ATE0 -> AT+CIPMODE=1 -> AT+NETOPEN (async, polled) ->
 *   AT+CIPOPEN=0,"TCP",host,port -> "ATO\r\n" -> transparent
 * Once in transparent mode the driver sends the relay auth line
 * "CT511AUTH/1 DRONE <token> <IMEI>\n" as the first bytes on the
 * socket, where token = first 48 hex chars of
 * SHA-256(password || CT511_AUTH_SALT); the password is the numeric
 * CT511_PW parameter (default 666666).  The relay derives the same
 * token from its password and validates it before forwarding
 * anything to mavlink-router, so unauthenticated dialers never reach
 * the router.
 * Exit transparent mode: "+++" WITHOUT CR/LF terminator, needs ~1s of
 * UART silence before and after (§5.4.6).
 *
 * Supervision: transparent mode is a blind pipe - the driver cannot
 * query the modem while it is in transparent mode, so liveness can only
 * be observed from the downlink (bytes the module sends us, i.e. data
 * from the server).  When the downlink stays silent for
 * CT511_RELINK_SILENT_MS the link is considered lost (dead TCP leg or
 * simply nobody on the server side): the driver escapes (+++), closes
 * and re-opens the TCP connection.  When a GCS is connected, MAVLink
 * heartbeats flow down every ~1s so the link stays up; when idle the
 * reconnect cycle doubles as a keepalive and self-heals any silently
 * dead TCP leg within one period.  Cost of a cycle is a few bytes of
 * 4G traffic.
 *
 * Module resets (AT+RESET) are rate limited (CT511_RESET_MIN_MS) and
 * used only as the last escalation step.
 *
 * Default relay: hz (47.96.23.146), port 4002 (compiled in host, see
 * CT511_HOST / CT511_PORT_DEFAULT below; port overridable with the
 * CT511_PRT param).
 */

#include "ct511.h"
#include "ct511_sha256.h"
#include "ct511_vuart.h"

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/ct511_status.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>

#define CT511_HOST_DEFAULT      "47.96.23.146"
#define CT511_PORT_DEFAULT      4002

/* 鉴权密码为纯数字, 用 CT511_PW 参数配置 (relay / 手机 proxy 必须一致,
 * 默认 666666). 线上传输的 token 由 密码+盐 派生:
 * token = sha256(pw || salt)[0:48], 规则与 relay.c / ProxyService.java
 * 完全一致, 盐串三处必须相同。参数值 <=0 时回退到下面的默认值。 */
#define CT511_PW_DEFAULT        666666
#define CT511_AUTH_SALT         "CT511-4G-AUTH-V1"

#define CT511_UART_TELEM1       "/dev/ttyS5"   /* fmu-v6c TELEM1 */
#define CT511_UART_TELEM2       "/dev/ttyS3"   /* fmu-v6c TELEM2 */
#define CT511_UART_TELEM3       "/dev/ttyS1"   /* fmu-v6c TELEM3 */
#define CT511_UART_GPS2         "/dev/ttyS6"   /* fmu-v6c GPS2 */

#define CT511_RX_BUF_SIZE       1024           /* modem serial rx ring (lib) */
#define CT511_IO_BATCH          512            /* bytes moved per loop step */

#define CT511_PROBE_RETRY_MS    5000           /* AT probe backoff */
#define CT511_NETOPEN_TIMEOUT_MS 45000
#define CT511_ATO_SETTLE_MS     1500           /* swallow noise after ATO */
#define CT511_RELINK_SILENT_MS  60000          /* downlink silence -> relink */
#define CT511_RESET_MIN_MS      120000         /* min spacing between AT+RESET */

namespace
{

enum Ct511State : uint8_t {
	STATE_STARTING = 0,
	STATE_PROBING = 1,
	STATE_NETOPEN = 2,
	STATE_CONNECT = 3,
	STATE_TRANSPARENT = 4,
	STATE_ERROR = 5,
};

const char *const k_uarts[] = {
	CT511_UART_TELEM1,
	CT511_UART_TELEM2,
	CT511_UART_TELEM3,
	CT511_UART_GPS2,
};

} // namespace

class Ct511 : public ModuleBase
{
public:
	Ct511();
	~Ct511() override;

	static int task_spawn(int argc, char *argv[]);
	static Ct511 *instantiate(int argc, char *argv[]) { return new Ct511(); }
	static int run_trampoline(int argc, char *argv[]);
	static ModuleBase::Descriptor desc;

	int print_status() override;
	void run() override;
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/* one-shot AT diagnostics: needs the driver stopped (owns the UART) */
	int _debug_run();
	static int debug_dispatch();

private:
	static void modem_event_cb(ct511_t *dev, int evt, const void *data);
	static void hal_send(const uint8_t *data, uint16_t len);
	static uint16_t hal_recv(uint8_t *data, uint16_t max_len);
	static uint32_t hal_millis();
	static void hal_delay_ms(uint32_t ms);

	void _send(const uint8_t *data, uint16_t len);
	uint16_t _recv(uint8_t *data, uint16_t max_len);
	uint32_t _millis() { return (uint32_t)(hrt_absolute_time() / 1000); }
	void _wait_ms(uint32_t ms);

	bool _open_uart();
	void _close_uart();
	void _flush_uart_input();
	void _set_state(Ct511State state);

	/* link lifecycle */
	void _link_main();
	bool _ensure_at();
	bool _ensure_netopen();
	bool _open_tcp_and_enter_transparent();
	void _exit_transparent_to_at();
	bool _pump_transparent();
	void _reset_module();

	/* byte movers */
	void _drain_vuart_to_modem();
	void _drain_modem_to_vuart(bool discard);

	void _publish_uorb_status();

	static Ct511 *_instance;

	char _uart_dev[32] {};
	int _uart_fd{-1};

	ct511_t _modem {};
	uint8_t _modem_rx_buf[CT511_RX_BUF_SIZE];
	ct511_hal_t _hal {};

	Ct511Vuart *_vuart {nullptr};

	int32_t _param_uart {0};
	int32_t _param_port {CT511_PORT_DEFAULT};
	int32_t _param_pw {CT511_PW_DEFAULT};

	Ct511State _state {STATE_STARTING};
	bool _modem_config_done {false};   /* CIPMODE=1 already applied since boot */
	char _imei[24] {};                 /* queried once at first AT probe */
	int32_t _last_csq {-1};
	int32_t _last_cereg {-1};
	uint32_t _tx_bytes {0};
	uint32_t _rx_bytes {0};
	uint32_t _link_reconnects {0};
	int _link_fail_streak {0};
	uint64_t _last_modem_reset_ms {0};
	uint64_t _last_modem_rx_ms {0};
	uint64_t _discard_rx_until_ms {0};

	uint8_t _io_buf[CT511_IO_BATCH] {};

	uORB::Publication<ct511_status_s> _status_pub {ORB_ID(ct511_status)};
};

Ct511 *Ct511::_instance {nullptr};

ModuleBase::Descriptor Ct511::desc{task_spawn, custom_command, print_usage};

Ct511::Ct511()
{
	_instance = this;

	_hal.send = hal_send;
	_hal.millis = hal_millis;
	_hal.recv = hal_recv;
	_hal.delay_ms = hal_delay_ms;
}

Ct511::~Ct511()
{
	_close_uart();

	delete _vuart;
	_vuart = nullptr;

	_instance = nullptr;
}

int
Ct511::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("ct511",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  16384,
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -1;
	}

	// wait until the thread is up & running (max 1 second)
	if (wait_until_running(desc) < 0) {
		desc.task_id = -1;
		return -1;
	}

	return 0;
}

int
Ct511::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return Ct511::instantiate(ac, av);
	}, argc, argv);
}

void
Ct511::run()
{
	param_get(param_find("CT511_DEV"), &_param_uart);

	if (_param_uart < 0 || _param_uart > 3) {
		_param_uart = 0;
	}

	param_get(param_find("CT511_PRT"), &_param_port);

	if (_param_port <= 0 || _param_port > 65535) {
		_param_port = CT511_PORT_DEFAULT;
	}

	param_get(param_find("CT511_PW"), &_param_pw);

	if (_param_pw <= 0) {
		_param_pw = CT511_PW_DEFAULT;
	}

	_vuart = new Ct511Vuart();

	if (_vuart->init() != PX4_OK) {
		delete _vuart;
		_vuart = nullptr;
		PX4_ERR("ct511: failed to register /dev/ct511");
		return;
	}

	if (ct511_init(&_modem, &_hal, _modem_rx_buf, sizeof(_modem_rx_buf), modem_event_cb) != CT511_OK) {
		PX4_ERR("ct511: failed to init modem library");
		return;
	}

	PX4_INFO("ct511: starting, uart %s, relay %s:%d (TCP transparent)",
		 k_uarts[_param_uart], CT511_HOST_DEFAULT, (int)_param_port);

	while (!should_exit()) {
		if (!_open_uart()) {
			PX4_WARN("ct511: cannot open %s, retrying", k_uarts[_param_uart]);
			_set_state(STATE_ERROR);
			_wait_ms(5000);
			continue;
		}

		_link_main();

		_close_uart();
		_wait_ms(3000);
	}
}

int
Ct511::debug_dispatch()
{
	if (_instance) {
		PX4_WARN("ct511: driver is running, stop it first (`ct511 stop`), then `ct511 dbg`");
		return 1;
	}

	Ct511 *dbg = new Ct511();
	int rc = dbg->_debug_run();
	delete dbg;
	return rc;
}

int
Ct511::_debug_run()
{
	param_get(param_find("CT511_DEV"), &_param_uart);

	if (_param_uart < 0 || _param_uart > 3) {
		_param_uart = 0;
	}

	if (!_open_uart()) {
		PX4_ERR("ct511 dbg: cannot open %s", k_uarts[_param_uart]);
		return 1;
	}

	if (ct511_init(&_modem, &_hal, _modem_rx_buf, sizeof(_modem_rx_buf), modem_event_cb) != CT511_OK) {
		PX4_ERR("ct511 dbg: library init failed");
		_close_uart();
		return 1;
	}

	PX4_INFO("---- ct511 dbg on %s ----", k_uarts[_param_uart]);

	static const struct {
		const char *at;
		uint32_t timeout_ms;
	} tbl[] = {
		{"ATE0", 3000},
		{"AT", 3000},
		{"AT+CPIN?", 5000},
		{"AT+CFUN?", 3000},
		{"AT+CEREG?", 5000},
		{"AT+CREG?", 5000},
		{"AT+CSQ", 3000},
		{"AT+ICCID", 3000},
		{"AT+CIMI", 3000},
		{"AT+CNUM", 3000},
		{"AT+CGSN", 3000},
		{"AT+COPS?", 5000},
		{"AT+QICSGP?", 3000},
		{"AT+NETOPEN?", 5000},
	};

	for (const auto &e : tbl) {
		char rsp[192] {};

		ct511_result_t rc = ct511_cmd(&_modem, e.at, rsp, sizeof(rsp), e.timeout_ms, 400);
		PX4_INFO("AT> %-14s => %s%s%s", e.at, (rc == CT511_OK) ? rsp : "(no OK)",
			 (rc == CT511_OK) ? "" : " ", (rc == CT511_OK) ? "" : "TIMEOUT");
	}

	/* recovery attempt: re-init radio stack, full operator scan, watch attach */
	{
		char rsp[128] {};

		ct511_result_t rc = ct511_cmd(&_modem, "AT+CFUN=0", rsp, sizeof(rsp), 8000, 400);
		PX4_INFO("AT> AT+CFUN=0     => %s%s", (rc == CT511_OK) ? rsp : "(no OK)",
			 (rc == CT511_OK) ? "" : " TIMEOUT");
		_wait_ms(2000);

		rc = ct511_cmd(&_modem, "AT+CFUN=1", rsp, sizeof(rsp), 15000, 400);
		PX4_INFO("AT> AT+CFUN=1     => %s%s", (rc == CT511_OK) ? rsp : "(no OK)",
			 (rc == CT511_OK) ? "" : " TIMEOUT");
		_wait_ms(3000);

		char scan[512] {};
		rc = ct511_cmd(&_modem, "AT+COPS=?", scan, sizeof(scan), 210000, 1000);
		PX4_INFO("AT> AT+COPS=?     => %s%s", (rc == CT511_OK) ? scan : "(no OK)",
			 (rc == CT511_OK) ? "" : " TIMEOUT");

		rc = ct511_cmd(&_modem, "AT+COPS=0", rsp, sizeof(rsp), 8000, 1000);
		PX4_INFO("AT> AT+COPS=0     => %s%s", (rc == CT511_OK) ? rsp : "(no OK)",
			 (rc == CT511_OK) ? "" : " TIMEOUT");

		for (int i = 1; i <= 6; i++) {
			_wait_ms(10000);
			rsp[0] = 0;
			rc = ct511_cmd(&_modem, "AT+CEREG?", rsp, sizeof(rsp), 5000, 400);
			PX4_INFO("AT> +CEREG? (+%ds) => %s", i * 10, (rc == CT511_OK) ? rsp : "(no OK)");
		}
	}

	PX4_INFO("---- ct511 dbg done ----");
	_close_uart();
	return 0;
}

bool
Ct511::_open_uart()
{
	strncpy(_uart_dev, k_uarts[_param_uart], sizeof(_uart_dev) - 1);

	_uart_fd = ::open(_uart_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		return false;
	}

	struct termios config {};

	if (tcgetattr(_uart_fd, &config) < 0) {
		_close_uart();
		return false;
	}

	config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	config.c_oflag &= ~OPOST;
	config.c_lflag &= ~(ECHONL | ICANON | IEXTEN | ISIG | ECHO | ECHOE);
	config.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB);

	cfsetispeed(&config, B115200);
	cfsetospeed(&config, B115200);

	if (tcsetattr(_uart_fd, TCSANOW, &config) < 0) {
		_close_uart();
		return false;
	}

	tcflush(_uart_fd, TCIOFLUSH);

	return true;
}

void
Ct511::_close_uart()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

void
Ct511::_flush_uart_input()
{
	tcflush(_uart_fd, TCIFLUSH);

	/* drain whatever already entered the library ring */
	for (int i = 0; i < 4; i++) {
		ct511_poll(&_modem);
	}
}

void
Ct511::_send(const uint8_t *data, uint16_t len)
{
	size_t off = 0;

	while (off < len && !should_exit()) {
		ssize_t n = ::write(_uart_fd, data + off, len - off);

		if (n > 0) {
			off += (size_t)n;

		} else if (n < 0 && errno != EAGAIN) {
			return;

		} else {
			px4_usleep(1000);
		}
	}
}

uint16_t
Ct511::_recv(uint8_t *data, uint16_t max_len)
{
	ssize_t n = ::read(_uart_fd, data, max_len);

	if (n <= 0) {
		return 0;
	}

	return (uint16_t)n;
}

void
Ct511::_wait_ms(uint32_t ms)
{
	uint32_t start = _millis();

	while (((_millis() - start) < ms) && !should_exit()) {
		px4_usleep(5000);
	}
}

void
Ct511::_set_state(Ct511State state)
{
	_state = state;
	_publish_uorb_status();
}

/**
 * Full link lifecycle: probe -> network -> TCP -> transparent pump;
 * any stage failure escalates (netopen redo -> module reset) and retries.
 * Returns only on module exit.
 */
void
Ct511::_link_main()
{
	while (!should_exit()) {
		/* ------- stage 1: modem alive (AT) ------- */
		_set_state(STATE_PROBING);

		if (!_ensure_at()) {
			_wait_ms(CT511_PROBE_RETRY_MS);
			continue;
		}

		/* ------- stage 2: data network ------- */
		_set_state(STATE_NETOPEN);

		if (!_ensure_netopen()) {
			_link_fail_streak++;

			if (_link_fail_streak >= 3) {
				_reset_module();
			}

			_wait_ms(10000);
			continue;
		}

		/* ------- stage 3: TCP + transparent ------- */
		_set_state(STATE_CONNECT);

		if (_open_tcp_and_enter_transparent()) {
			_link_fail_streak = 0;
			_link_reconnects++;
			_set_state(STATE_TRANSPARENT);

			if (_pump_transparent()) {
				return; /* driver shutdown */
			}

			/* downlink went silent: escape and re-establish */
			_exit_transparent_to_at();
			_wait_ms(2000);
			continue;
		}

		/* TCP open failed: escalate */
		_link_fail_streak++;

		if (_link_fail_streak >= 3) {
			/* force a full network redo next round */
			_modem_config_done = false;
			ct511_netclose(&_modem);
		}

		if (_link_fail_streak >= 6) {
			_reset_module();
		}

		_wait_ms(3000);
	}
}

bool
Ct511::_ensure_at()
{
	/* inside _link_main only; escalate to module reset when stuck */
	for (int try_no = 0; try_no < 3 && !should_exit(); try_no++) {
		if (ct511_test(&_modem) == CT511_OK) {
			ct511_set_echo(&_modem, false);

			char info[64] {};

			if (ct511_info(&_modem, info, sizeof(info)) == CT511_OK && info[0]) {
				PX4_INFO("ct511: modem info: %s", info);
			}

			if (!_imei[0]) {
				char imei[24] {};

				if (ct511_get_imei(&_modem, imei, sizeof(imei)) == CT511_OK && imei[0]) {
					strncpy(_imei, imei, sizeof(_imei) - 1);
					PX4_INFO("ct511: IMEI %s", _imei);
				}
			}

			_last_csq = ct511_csq(&_modem);

			int reg = -1;

			if (ct511_cereg(&_modem, &reg) == CT511_OK) {
				_last_cereg = reg;

				if (reg != 1 && reg != 5) {
					PX4_WARN("ct511: not registered on network (stat %d)", reg);
				}
			}

			return true;
		}

		PX4_WARN("ct511: modem not responding, retrying");

		/* 模块可能卡在透传/数据就绪态: +++ 逃逸后再探测 (手册 5.4.6) */
		if (try_no == 1) {
			_exit_transparent_to_at();
		}

		_wait_ms(CT511_PROBE_RETRY_MS);
	}

	if (_millis() - (uint32_t)_last_modem_reset_ms > CT511_RESET_MIN_MS) {
		_reset_module();
	}

	return false;
}

bool
Ct511::_ensure_netopen()
{
	if (!_modem_config_done) {
		/* 手册 5.4.1: CIPMODE 须在开启数据网络之前设置;
		 * 数据网络已开时模块拒绝 CIPMODE, 先 NETCLOSE 再设置 */
		if (ct511_sock_set_mode(&_modem, true) != CT511_OK) {
			PX4_INFO("ct511: CIPMODE=1 needs closed data network, netclose first");
			ct511_netclose(&_modem); /* 数据网络未开时失败可忽略 */

			if (ct511_sock_set_mode(&_modem, true) != CT511_OK) {
				PX4_WARN("ct511: CIPMODE=1 failed");
				return false;
			}
		}

		ct511_set_apn(&_modem, nullptr, nullptr, nullptr);

		if (ct511_netopen(&_modem, CT511_NETOPEN_TIMEOUT_MS) != CT511_OK) {
			PX4_WARN("ct511: NETOPEN timeout");
			return false;
		}

		_modem_config_done = true;
		PX4_INFO("ct511: network open");
		return true;
	}

	/* already configured once since boot: only verify it is still open */
	if (ct511_netopen_status(&_modem)) {
		return true;
	}

	if (ct511_netopen(&_modem, CT511_NETOPEN_TIMEOUT_MS) != CT511_OK) {
		PX4_WARN("ct511: NETOPEN (again) timeout");
		return false;
	}

	return true;
}

bool
Ct511::_open_tcp_and_enter_transparent()
{
	/* close any leftover socket from a previous session, then open fresh */
	if (ct511_sock_close(&_modem, 0) != CT511_OK) {
		/* no previous socket: fine */
	}

	if (ct511_sock_open(&_modem, "TCP", CT511_HOST_DEFAULT, (uint16_t)_param_port, 0) != CT511_OK) {
		PX4_WARN("ct511: CIPOPEN %s:%d failed", CT511_HOST_DEFAULT, (int)_param_port);
		return false;
	}

	/*
	 * CIPMODE=1 下 CIPOPEN 成功后模块回 '>' 进入数据就绪状态, 可直接
	 * 收发数据, 无需 ATO(补发会作为数据泄给对端 5 字节); 仅当未见
	 * 提示符(旧固件回 OK/SUCCESS)时补发 ATO 进入透传。
	 * Discard the first CT511_ATO_SETTLE_MS of modem output so any
	 * entry noise (OK/CONNECT/'>') never reaches the mavlink side.
	 */
	if (!_modem.prompt_seen) {
		ct511_sock_enter_transparent(&_modem);
	}

	/*
	 * 鉴权握手: 进入透传后的第一批字节必须是 CT511AUTH/1 行(含 IMEI
	 * 终端信息), 服务器校验通过后才把本连接接入 mavlink-router;
	 * 校验失败会被断开, 由下行静默看门狗触发重连重试。
	 */
	char auth_line[96] {};
	char token[49] {};
	char pw[16] {};
	snprintf(pw, sizeof(pw), "%d", (int)_param_pw);
	derive_auth_token(token, pw, CT511_AUTH_SALT);
	const int alen = snprintf(auth_line, sizeof(auth_line), "CT511AUTH/1 DRONE %s %s\n",
				  token, _imei[0] ? _imei : "-");
	_send((const uint8_t *)auth_line, (uint16_t)alen);

	_discard_rx_until_ms = _millis() + CT511_ATO_SETTLE_MS;
	_last_modem_rx_ms = hrt_absolute_time(); /* avoid instant relink trigger */

	PX4_INFO("ct511: TCP transparent to %s:%d", CT511_HOST_DEFAULT, (int)_param_port);
	return true;
}

/**
 * Leave transparent mode: "+++" has no CR/LF terminator and needs ~1s of
 * UART silence before and after so the modem recognises it as an escape
 * rather than data (manual 5.4.6).  After this the modem is back in AT
 * command mode.
 */
void
Ct511::_exit_transparent_to_at()
{
	PX4_INFO("ct511: exiting transparent mode");

	px4_usleep(1100 * 1000); /* guard: keep UART quiet before +++ */
	ct511_sock_exit_transparent(&_modem);
	px4_usleep(1100 * 1000); /* guard after */

	/* 模块若本就不在数据模式, "+++" 会残留于其行缓冲, 空行清掉 */
	_send((const uint8_t *)"\r\n", 2);
	px4_usleep(200 * 1000);

	/* swallow anything the modem printed while exiting (OK/CLOSED/...) */
	_flush_uart_input();
}

bool
Ct511::_pump_transparent()
{
	uint64_t last_uorb = hrt_absolute_time();

	while (!should_exit()) {
		struct pollfd fds[1] = {};
		fds[0].fd = _uart_fd;
		fds[0].events = POLLIN;
		int pr = ::poll(fds, 1, 10);

		if (pr > 0 && (fds[0].revents & (POLLIN | POLLHUP))) {
			const bool discard = (_millis() < _discard_rx_until_ms);
			_drain_modem_to_vuart(discard);
		}

		_drain_vuart_to_modem();

		if (hrt_elapsed_time(&last_uorb) >= 1000000) {
			last_uorb = hrt_absolute_time();
			_publish_uorb_status();
		}

		/* downlink silence watchdog: nobody on the other end or TCP dead */
		if (hrt_elapsed_time(&_last_modem_rx_ms) >= CT511_RELINK_SILENT_MS * 1000) {
			PX4_WARN("ct511: downlink silent > %u s, relinking",
				 (unsigned)CT511_RELINK_SILENT_MS / 1000);
			return false;
		}
	}

	return true;
}

void
Ct511::_drain_vuart_to_modem()
{
	if (!_vuart) {
		return;
	}

	for (;;) {
		const size_t n = _vuart->tx_pull(_io_buf, sizeof(_io_buf));

		if (n == 0) {
			break;
		}

		_send(_io_buf, (uint16_t)n);
		_tx_bytes += (uint32_t)n;
	}
}

void
Ct511::_drain_modem_to_vuart(bool discard)
{
	if (!_vuart) {
		return;
	}

	uint16_t total = 0;

	for (;;) {
		const uint16_t n = _recv(_io_buf, sizeof(_io_buf));

		if (n == 0) {
			break;
		}

		total += n;

		if (!discard) {
			const int pushed = _vuart->rx_push(_io_buf, n);
			_rx_bytes += (uint32_t)pushed;

			if ((size_t)pushed < n) {
				PX4_WARN("ct511: downlink rx ring overflow");
			}
		}
	}

	if (total > 0) {
		_last_modem_rx_ms = hrt_absolute_time();
	}
}

void
Ct511::_reset_module()
{
	if (_millis() - (uint32_t)_last_modem_reset_ms < CT511_RESET_MIN_MS) {
		return;
	}

	PX4_WARN("ct511: resetting modem");
	ct511_reboot(&_modem);
	_last_modem_reset_ms = _millis();
	_modem_config_done = false;
	_link_fail_streak = 0;
}

void
Ct511::_publish_uorb_status()
{
	ct511_status_s status {};
	status.timestamp = hrt_absolute_time();
	status.state = (uint8_t)_state;
	status.link_up = (_state == STATE_TRANSPARENT);
	status.csq = _last_csq;
	status.tx_bytes = _tx_bytes;
	status.rx_bytes = _rx_bytes;
	status.reconnects = _link_reconnects;

	_status_pub.publish(status);
}

/* ==================== static HAL glue ==================== */

void
Ct511::modem_event_cb(ct511_t *dev, int evt, const void *data)
{
	/* no events matter in transparent mode; library is only used for AT phases */
}

void
Ct511::hal_send(const uint8_t *data, uint16_t len)
{
	if (_instance) {
		_instance->_send(data, len);
	}
}

uint16_t
Ct511::hal_recv(uint8_t *data, uint16_t max_len)
{
	return _instance ? _instance->_recv(data, max_len) : 0;
}

uint32_t
Ct511::hal_millis()
{
	return (uint32_t)(hrt_absolute_time() / 1000);
}

void
Ct511::hal_delay_ms(uint32_t ms)
{
	px4_usleep(ms * 1000);
}

int
Ct511::print_status()
{
	PX4_INFO("ct511: state %u, uart %s, link %s, csq %d",
		 (unsigned)_state, _uart_dev, _state == STATE_TRANSPARENT ? "transparent" : "down", (int)_last_csq);
	PX4_INFO("ct511: tx %u bytes, rx %u bytes, reconnects %u, fail streak %d",
		 (unsigned)_tx_bytes, (unsigned)_rx_bytes, (unsigned)_link_reconnects, _link_fail_streak);
	PX4_INFO("ct511: relay %s:%d (TCP transparent), auth password %d",
		 CT511_HOST_DEFAULT, (int)_param_port, (int)_param_pw);
	return 0;
}

int
Ct511::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
DX-CT511 4G modem driver (TCP transparent mode).

Creates a virtual serial port (/dev/ct511) that the mavlink module attaches to
(`mavlink start -d /dev/ct511 -b 115200 -r 40000`). MAVLink bytes are piped raw
over a single TCP transparent connection to the relay server
(host compiled in, port CT511_PRT).

Link is supervised by downlink silence: after 60s without bytes from the modem
the TCP connection is torn down and re-established. GCS heartbeats keep the
link busy when connected; idle links re-connect every ~60s, which also
self-heals silently dead TCP legs (4G NAT/carrier drops).

Default relay: 47.96.23.146:4002 (compiled in host).
Auth: relay password is the numeric CT511_PW parameter (default 666666); it is
hashed with a fixed salt into the 48-hex token sent in the CT511AUTH/1 line.
Keep the password in sync with the relay server and the phone proxy app.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ct511", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("dbg");
	PRINT_MODULE_USAGE_COMMAND("stop");
	return 0;
}

extern "C" __EXPORT int ct511_main(int argc, char *argv[])
{
	return ModuleBase::main(Ct511::desc, argc, argv);
}

int
Ct511::custom_command(int argc, char *argv[])
{
	const char *verb = nullptr;

	if (argc >= 2 && strcmp(argv[1], "dbg") == 0) {
		verb = argv[1];
	}

	if (argc >= 1 && strcmp(argv[0], "dbg") == 0) {
		verb = argv[0];
	}

	if (verb && strcmp(verb, "dbg") == 0) {
		return debug_dispatch();
	}

	return print_usage("unknown command");
}
