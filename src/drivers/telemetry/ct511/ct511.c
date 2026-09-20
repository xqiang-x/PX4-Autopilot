/**
 * @file ct511.c
 * @brief DX-CT511/CT511N 4G 模块 C 驱动库实现
 *
 * 行协议: 收到 '\n' 成行; '\r' 忽略; 命令期间收集响应到 rsp,
 * OK/ERROR/+CME ERROR 结束本次命令; 空闲时按事件分发。
 */
#include "ct511.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* ==================== 内部工具 ==================== */

static bool ct511_ring_pop(ct511_t *dev, uint8_t *b)
{
	if (dev->rx_tail == dev->rx_head) {
		return false;
	}

	*b = dev->rx_buf[dev->rx_tail];
	dev->rx_tail = (dev->rx_tail + 1) % dev->rx_size;
	return true;
}

/** 忙等/睡眠轮询一步: 有 delay_ms 时先睡 5ms 让出 CPU 再处理接收, 否则忙等 */
static void ct511_poll_yield(ct511_t *dev)
{
	if (dev->hal->delay_ms) {
		dev->hal->delay_ms(5);
	}

	ct511_poll(dev);
}

/** 等待 ms 毫秒, 期间持续处理接收 */
static void ct511_wait(ct511_t *dev, uint32_t ms)
{
	uint32_t start = dev->hal->millis();

	while ((dev->hal->millis() - start) < ms) {
		ct511_poll_yield(dev);
	}
}

static void ct511_dispatch(ct511_t *dev, int evt, const void *data)
{
	if (dev->event_cb) {
		dev->event_cb(dev, evt, data);
	}
}

/* 区分下行投递与 AT+MSUB 自身的应答:
 * 下行: +MSUB: "topic",N bytes,"payload"  应答: +MSUB:SUCCESS
 * 要求冒号后有引号包裹的 topic 且含 "bytes", 避免把应答误判为空消息 */
static bool ct511_is_msub_push(const char *line)
{
	const char *p;

	if (strncmp(line, "+MSUB:", 6) != 0) {
		return false;
	}

	p = line + 6;

	while (*p == ' ' || *p == '\t') {
		p++;
	}

	return *p == '"' && strstr(p, "bytes") != NULL;
}

static void ct511_emit_push(ct511_t *dev, const char *line)
{
	if (line[0] == '$') {
		ct511_dispatch(dev, CT511_EVT_NMEA, line);
		return;
	}

	if (ct511_is_msub_push(line)) {
		/* +MSUB: "topic",N bytes,"payload" */
		const char *p = strchr(line, ':');
		ct511_mqtt_msg_t *m = &dev->mqtt_msg;
		uint16_t i;
		p++;

		while (*p == ' ' || *p == '\t') { p++; }

		memset(m, 0, sizeof * m);

		if (*p == '"') {
			p++;

			for (i = 0; *p && *p != '"' && i < CT511_TOPIC_MAX - 1; i++) {
				m->topic[i] = *p++;
			}

			if (*p == '"') { p++; }
		}

		p = strstr(p, "bytes");

		if (p) {
			p += 5;

			while (*p == ' ' || *p == '\t') { p++; }

			if (*p == ',' || *p == ':') { p++; }

			while (*p == ' ' || *p == '\t') { p++; }

			i = 0;

			while (*p && *p != '\r' && *p != '\n' && i < CT511_PAYLOAD_MAX - 1) {
				m->payload[i++] = *p++;
			}

			/* 模块输出形如 "payload", 剥掉首尾包裹引号 */
			if (i >= 2 && m->payload[0] == '"'
			    && m->payload[i - 1] == '"') {
				memmove(m->payload, m->payload + 1, i - 2);
				i -= 2;
			}

			m->payload[i] = 0;
			m->payload_len = i;
		}

		dev->mqtt_new = true;
		ct511_dispatch(dev, CT511_EVT_MQTT_MSG, m);
		return;
	}

	ct511_dispatch(dev, CT511_EVT_PUSH, line);
}

static bool ct511_line_complete(ct511_t *dev, const char *line, uint16_t len)
{
	/* 去掉尾部 \r */
	while (len && line[len - 1] == '\r') { len--; }

	if (ct511_is_msub_push(line)) {
		/* 下行投递可能在命令在途时到达, 不能并入命令响应(否则丢失) */
		ct511_emit_push(dev, line);
		return false;
	}

	if (dev->collecting) {
		/* 回显行过滤 */
		if (dev->cmd_echo && (len == strlen(dev->cmd_echo))
		    && memcmp(line, dev->cmd_echo, len) == 0) {
			return false;        /* 仅为回显, 非结束 */
		}

		if (dev->rsp && dev->rsp_len < dev->rsp_cap) {
			if (dev->rsp_len) {
				if (dev->rsp_len + 1 < dev->rsp_cap) {
					dev->rsp[dev->rsp_len++] = '\n';
				}
			}

			uint16_t n = len;

			if (dev->rsp_len + n >= dev->rsp_cap) {
				n = dev->rsp_cap - dev->rsp_len - 1;
			}

			memcpy(dev->rsp + dev->rsp_len, line, n);
			dev->rsp_len += n;
			dev->rsp[dev->rsp_len] = 0;
		}

		if ((len == 2 && memcmp(line, "OK", 2) == 0)
		    || (len >= 5 && memcmp(line, "ERROR", 5) == 0)
		    || (len >= 10 && memcmp(line, "+CME ERROR", 10) == 0)) {
			dev->done = true;
			return true;
		}

		return false;
	}

	/* 空闲: 主动上报 */
	ct511_emit_push(dev, line);
	return false;
}

/* ==================== 底层协议 ==================== */

void ct511_rx_bytes(ct511_t *dev, const uint8_t *data, uint16_t len)
{
	uint16_t i, next;

	if (!dev->rx_buf || dev->rx_size == 0) {
		return;
	}

	for (i = 0; i < len; i++) {
		next = (dev->rx_head + 1) % dev->rx_size;

		if (next != dev->rx_tail) {
			dev->rx_buf[dev->rx_head] = data[i];
		}

		dev->rx_head = next; /* 满时覆盖最旧(丢弃策略简单化) */
	}
}

void ct511_poll(ct511_t *dev)
{
	uint8_t byte;

	if (dev->hal && dev->hal->recv) {
		uint8_t tmp[64];
		uint16_t n = dev->hal->recv(tmp, sizeof tmp);

		if (n) {
			ct511_rx_bytes(dev, tmp, n);
		}
	}

	while (ct511_ring_pop(dev, &byte)) {
		if (byte == '\n') {
			if (dev->line_overrun) {
				dev->line_overrun = false;

			} else if (dev->line_len == 1 && dev->line[0] == '>') {
				dev->prompt_seen = true; /* 数据模式提示符(带换行形态) */

			} else if (dev->line_len) {
				dev->line[dev->line_len] = 0;
				ct511_line_complete(dev, dev->line, dev->line_len);
			}

			dev->line_len = 0;

		} else if (byte == '\r') {
			/* 忽略回车 */
		} else {
			if (dev->line_len < CT511_LINE_MAX - 1) {
				dev->line[dev->line_len++] = (char)byte;

			} else {
				dev->line_overrun = true;
			}
		}
	}

	/* 数据模式提示符 '>': 无换行, 单独成段 */
	if (dev->line_len == 1 && dev->line[0] == '>') {
		dev->line_len = 0;
		dev->prompt_seen = true;
	}
}

ct511_result_t ct511_cmd(ct511_t *dev, const char *at,
			 char *rsp, uint16_t rsp_cap,
			 uint32_t timeout_ms, uint32_t linger_ms)
{
	uint32_t start;
	uint16_t at_len;

	if (!dev || !dev->hal || !at) {
		return CT511_ERR_ARG;
	}

	if (!dev->hal->send || !dev->hal->millis) {
		return CT511_ERR_ARG;
	}

	if (dev->in_cmd) {
		return CT511_ERR;        /* 防重入 */
	}

	dev->in_cmd = true;
	dev->collecting = true;
	dev->done = false;
	dev->prompt_seen = false;
	dev->rsp = rsp;
	dev->rsp_cap = rsp_cap;
	dev->rsp_len = 0;
	dev->cmd_echo = at;

	if (rsp && rsp_cap) {
		rsp[0] = 0;
	}

	at_len = (uint16_t)strlen(at);
	dev->hal->send((const uint8_t *)at, at_len);
	dev->hal->send((const uint8_t *)"\r\n", 2);

	start = dev->hal->millis();

	/* '>' 数据模式提示符(CIPMODE=1 下 CIPOPEN 成功)也结束等待 */
	while (!dev->done && !dev->prompt_seen
	       && ((dev->hal->millis() - start) < timeout_ms)) {
		ct511_poll_yield(dev);
	}

	/* 收尾窗口: 捕获 OK 后的结果行(如 +NETOPEN:SUCCESS) */
	ct511_wait(dev, linger_ms);

	dev->collecting = false;
	dev->cmd_echo = NULL;
	dev->in_cmd = false;
	return dev->done ? CT511_OK : CT511_ERR_TIMEOUT;
}

ct511_result_t ct511_send_prompt(ct511_t *dev, const char *at,
				 const uint8_t *data, uint16_t len,
				 const uint8_t *tail, uint16_t tail_len,
				 char *rsp, uint16_t rsp_cap,
				 uint32_t timeout_ms, uint32_t linger_ms)
{
	uint32_t start;

	if (!dev || !dev->hal || !at || !data) {
		return CT511_ERR_ARG;
	}

	if (dev->in_cmd) {
		return CT511_ERR;
	}

	dev->in_cmd = true;
	dev->collecting = true;
	dev->done = false;
	dev->prompt_seen = false;
	dev->rsp = rsp;
	dev->rsp_cap = rsp_cap;
	dev->rsp_len = 0;
	dev->cmd_echo = at;

	if (rsp && rsp_cap) {
		rsp[0] = 0;
	}

	dev->hal->send((const uint8_t *)at, (uint16_t)strlen(at));
	dev->hal->send((const uint8_t *)"\r\n", 2);

	start = dev->hal->millis();

	while (!dev->prompt_seen && !dev->done
	       && ((dev->hal->millis() - start) < timeout_ms)) {
		ct511_poll_yield(dev);
	}

	if (dev->prompt_seen) {
		dev->hal->send(data, len);

		if (tail && tail_len) {
			dev->hal->send(tail, tail_len);
		}

		start = dev->hal->millis();

		while (!dev->done && ((dev->hal->millis() - start) < timeout_ms)) {
			ct511_poll_yield(dev);
		}
	}

	ct511_wait(dev, linger_ms);

	dev->collecting = false;
	dev->cmd_echo = NULL;
	dev->in_cmd = false;

	if (!dev->prompt_seen) {
		return CT511_ERR_TIMEOUT;
	}

	return dev->done ? CT511_OK : CT511_ERR_TIMEOUT;
}

bool ct511_has(const char *rsp, const char *token)
{
	return rsp && token && strstr(rsp, token) != NULL;
}

const ct511_mqtt_msg_t *ct511_mqtt_wait_message(ct511_t *dev, uint32_t timeout_ms)
{
	uint32_t start = dev->hal->millis();

	while (!dev->mqtt_new && ((dev->hal->millis() - start) < timeout_ms)) {
		ct511_poll_yield(dev);
	}

	if (!dev->mqtt_new) {
		return NULL;
	}

	dev->mqtt_new = false;
	return &dev->mqtt_msg;
}

/* ==================== 初始化 ==================== */

ct511_result_t ct511_init(ct511_t *dev, const ct511_hal_t *hal,
			  uint8_t *rx_buf, uint16_t rx_buf_size,
			  ct511_event_cb_t event_cb)
{
	if (!dev || !hal || !hal->send || !hal->millis) {
		return CT511_ERR_ARG;
	}

	if (!rx_buf || rx_buf_size < 32) {
		return CT511_ERR_ARG;
	}

	memset(dev, 0, sizeof * dev);
	dev->hal = hal;
	dev->rx_buf = rx_buf;
	dev->rx_size = rx_buf_size;
	dev->event_cb = event_cb;
	return CT511_OK;
}

void ct511_set_event_cb(ct511_t *dev, ct511_event_cb_t cb)
{
	if (dev) {
		dev->event_cb = cb;
	}
}

/* ==================== 基础指令 ==================== */

ct511_result_t ct511_test(ct511_t *dev)
{
	char rsp[64];

	if (ct511_cmd(dev, "AT", rsp, sizeof rsp, 3000, 200) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_set_echo(ct511_t *dev, bool enable)
{
	char cmd[8];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "ATE%d", enable ? 1 : 0);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 3000, 200) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_info(ct511_t *dev, char *out, uint16_t cap)
{
	if (!out || !cap) {
		return CT511_ERR_ARG;
	}

	return ct511_cmd(dev, "ATI", out, cap, 5000, 300);
}

static int ct511_copy_after(const char *rsp, const char *token,
			    char *out, uint16_t cap)
{
	const char *p = strstr(rsp, token);
	uint16_t i = 0;

	if (!p) {
		return -1;
	}

	p += strlen(token);

	while (*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t') { p++; }

	if (*p == '"') { p++; } /* 可选引号 */

	while (*p && *p != '\r' && *p != '\n' && i < cap - 1) {
		out[i++] = *p++;
	}

	out[i] = 0;
	return (int)i;
}

ct511_result_t ct511_get_imei(ct511_t *dev, char *out, uint16_t cap)
{
	char rsp[80];

	if (ct511_cmd(dev, "AT+CGSN", rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	if (ct511_copy_after(rsp, "+CGSN:", out, cap) < 0) {
		/* 部分固件直接回显 IMEI 数字行 */
		const char *p = rsp;
		uint16_t i = 0;

		while (*p && i < cap - 1) {
			if (*p >= '0' && *p <= '9') {
				uint16_t j = 0;
				const char *q = p;

				while (*q >= '0' && *q <= '9' && j < 20) { q++; j++; }

				if (j == 15) {
					memcpy(out, p, 15);
					out[15] = 0;
					return CT511_OK;
				}
			}

			p++;
		}

		return CT511_ERR;
	}

	return CT511_OK;
}

ct511_result_t ct511_get_iccid(ct511_t *dev, char *out, uint16_t cap)
{
	char rsp[80];

	if (ct511_cmd(dev, "AT+ICCID", rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	if (ct511_copy_after(rsp, "+ICCID:", out, cap) < 0) {
		return CT511_ERR;
	}

	return CT511_OK;
}

/* ==================== 网络服务 ==================== */

int ct511_csq(ct511_t *dev)
{
	char rsp[64];
	int v = -1;
	const char *p;

	if (ct511_cmd(dev, "AT+CSQ", rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return -1;
	}

	p = strstr(rsp, "+CSQ:");

	if (p && sscanf(p, "+CSQ:%d", &v) == 1 && v >= 0) {
		return v;
	}

	return -1;
}

ct511_result_t ct511_cereg(ct511_t *dev, int *stat)
{
	char rsp[64];
	const char *p;
	int n = 0, s = -1;

	if (!stat) {
		return CT511_ERR_ARG;
	}

	if (ct511_cmd(dev, "AT+CEREG?", rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	p = strstr(rsp, "+CEREG:");

	if (!p) {
		return CT511_ERR;
	}

	if (sscanf(p, "+CEREG:%d,%d", &n, &s) != 2) {
		return CT511_ERR;
	}

	*stat = s;
	return CT511_OK;
}

ct511_result_t ct511_set_apn(ct511_t *dev, const char *apn,
			     const char *user, const char *pwd)
{
	char cmd[128];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "AT+QICSGP=1,1,\"%s\",\"%s\",\"%s\"",
		 apn ? apn : "", user ? user : "", pwd ? pwd : "");

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

bool ct511_netopen_status(ct511_t *dev)
{
	char rsp[64];

	if (ct511_cmd(dev, "AT+NETOPEN?", rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return false;
	}

	return ct511_has(rsp, "+NETOPEN:1");
}

ct511_result_t ct511_netopen(ct511_t *dev, uint32_t timeout_ms)
{
	char rsp[64];
	uint32_t start;
	ct511_cmd(dev, "AT+NETOPEN", rsp, sizeof rsp, 5000, 800); /* 触发开启 */
	start = dev->hal->millis();

	while ((dev->hal->millis() - start) < timeout_ms) {
		if (ct511_netopen_status(dev)) {
			return CT511_OK;
		}

		ct511_wait(dev, 100);
	}

	return CT511_ERR_TIMEOUT;
}

ct511_result_t ct511_netclose(ct511_t *dev)
{
	char rsp[64];

	if (ct511_cmd(dev, "AT+NETCLOSE", rsp, sizeof rsp, 6000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "SUCCESS") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_ntp_sync(ct511_t *dev, const char *server, uint16_t port,
			      char *time_str, uint16_t cap)
{
	char cmd[96];
	char rsp[64];
	const char *p;
	snprintf(cmd, sizeof cmd, "AT+QNTP=1,\"%s\",%u,1",
		 server ? server : "cn.pool.ntp.org", (unsigned)port);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 10000, 500) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	p = strstr(rsp, "+QNTP");

	if (!p) {
		return CT511_ERR;
	}

	p = strchr(p, '"');

	if (!p) {
		return CT511_ERR;
	}

	if (ct511_copy_after(p, "\"", time_str, cap) < 0) {
		return CT511_ERR;
	}

	return CT511_OK;
}

/* ==================== MQTT ==================== */

ct511_result_t ct511_mqtt_config(ct511_t *dev, const char *client_id,
				 const char *user, const char *pwd)
{
	char cmd[CT511_TOPIC_MAX + 96];
	char rsp[64];

	if (!client_id) {
		return CT511_ERR_ARG;
	}

	if (user && pwd)
		snprintf(cmd, sizeof cmd, "AT+MCONFIG=\"%s\",\"%s\",\"%s\"",
			 client_id, user, pwd);
	else {
		snprintf(cmd, sizeof cmd, "AT+MCONFIG=\"%s\"", client_id);
	}

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 8000, 500) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_mqtt_server(ct511_t *dev, const char *host,
				 uint16_t port, uint16_t version)
{
	char cmd[160];
	char rsp[96];

	if (!host) {
		return CT511_ERR_ARG;
	}

	if (version)
		snprintf(cmd, sizeof cmd, "AT+MIPSTART=\"%s\",%u,%u",
			 host, (unsigned)port, (unsigned)version);
	else {
		snprintf(cmd, sizeof cmd, "AT+MIPSTART=\"%s\",%u", host, (unsigned)port);
	}

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 15000, 1500) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "SUCCESS") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_mqtt_connect(ct511_t *dev, int clean_session,
				  uint16_t keepalive)
{
	char cmd[48];
	char rsp[96];
	snprintf(cmd, sizeof cmd, "AT+MCONNECT=%d,%u",
		 clean_session ? 1 : 0, (unsigned)keepalive);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 15000, 1500) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "SUCCESS") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_mqtt_open(ct511_t *dev, const char *client_id,
			       const char *host, uint16_t port,
			       const char *user, const char *pwd)
{
	ct511_mqtt_disconnect(dev); /* 清理残留会话, MCONFIG 才能成功 */
	ct511_mqtt_close(dev);

	if (ct511_mqtt_config(dev, client_id, user, pwd) != CT511_OK) {
		return CT511_ERR;
	}

	if (ct511_mqtt_server(dev, host, port, 0) != CT511_OK) {
		return CT511_ERR;
	}

	if (ct511_mqtt_connect(dev, 1, 60) != CT511_OK) {
		return CT511_ERR;
	}

	return CT511_OK;
}

bool ct511_mqtt_is_connected(ct511_t *dev)
{
	char rsp[64];

	if (ct511_cmd(dev, "AT+MQTTSTATU", rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return false;
	}

	return ct511_has(rsp, "+MQTTSTATU:1");
}

ct511_result_t ct511_mqtt_publish(ct511_t *dev, const char *topic,
				  const char *payload, int qos, int retain)
{
	uint16_t plen = payload ? (uint16_t)strlen(payload) : 0;
	char rsp[96];
	char at[CT511_MPUB_MAX + CT511_TOPIC_MAX + 32];
	char atex[CT511_TOPIC_MAX + 48];
	ct511_result_t r;

	if (!topic || !payload) {
		return CT511_ERR_ARG;
	}

	if (strchr(payload, '\r') || strchr(payload, '\n')) {
		return CT511_ERR_ARG;        /* 行协议限制 */
	}

	if (plen > CT511_MPUBEX_MAX) {
		return CT511_ERR_ARG;
	}

	if (plen <= CT511_MPUB_MAX) {
		snprintf(at, sizeof at, "AT+MPUB=\"%s\",%d,%d,\"%s\"",
			 topic, qos, retain ? 1 : 0, payload);
		r = ct511_cmd(dev, at, rsp, sizeof rsp, 10000, 2500);

	} else {
		snprintf(atex, sizeof atex, "AT+MPUBEX=\"%s\",%d,%d,%u",
			 topic, qos, retain ? 1 : 0, (unsigned)plen);
		r = ct511_send_prompt(dev, atex, (const uint8_t *)payload, plen,
				      NULL, 0, rsp, sizeof rsp, 10000, 2500);

		if (r != CT511_OK) {
			return r;
		}

		return ct511_has(rsp, "SUCCESS") ? CT511_OK : CT511_ERR;
	}

	if (r != CT511_OK) {
		return r;
	}

	return ct511_has(rsp, "SUCCESS") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_mqtt_subscribe(ct511_t *dev, const char *topic, int qos)
{
	char cmd[CT511_TOPIC_MAX + 32];
	char rsp[96];

	if (!topic) {
		return CT511_ERR_ARG;
	}

	snprintf(cmd, sizeof cmd, "AT+MSUB=\"%s\",%d", topic, qos);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 10000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "SUCCESS") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_mqtt_unsubscribe(ct511_t *dev, const char *topic)
{
	char cmd[CT511_TOPIC_MAX + 32];
	char rsp[96];

	if (!topic) {
		return CT511_ERR_ARG;
	}

	snprintf(cmd, sizeof cmd, "AT+MUNSUB=\"%s\"", topic);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 10000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "SUCCESS") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_mqtt_disconnect(ct511_t *dev)
{
	char rsp[64];
	ct511_cmd(dev, "AT+MDISCONNECT", rsp, sizeof rsp, 6000, 800);
	return CT511_OK; /* 未连接时可能 ERROR, 忽略 */
}

ct511_result_t ct511_mqtt_close(ct511_t *dev)
{
	char rsp[64];
	ct511_cmd(dev, "AT+MIPCLOSE", rsp, sizeof rsp, 6000, 800);
	return CT511_OK; /* 未配置时可能 ERROR, 忽略 */
}

/* ==================== GNSS ==================== */

ct511_result_t ct511_gps_set(ct511_t *dev, bool on, bool active_antenna_retry)
{
	char cmd[16];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "AT+MGPSC=%d", on ? 1 : 0);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 5000, 1000) == CT511_OK
	    && ct511_has(rsp, "OK")) {
		return CT511_OK;
	}

	if (!active_antenna_retry) {
		return CT511_ERR;
	}

	/* 有源天线序列后重试 */
	ct511_cmd(dev, "AT+CGDRT=12,1", rsp, sizeof rsp, 5000, 300);
	ct511_cmd(dev, "AT+CGSETV=12,1", rsp, sizeof rsp, 5000, 300);
	ct511_cmd(dev, "AT+CGGETV=12", rsp, sizeof rsp, 5000, 300);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 5000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_gps_mode(ct511_t *dev, uint8_t mode)
{
	char cmd[16];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "AT+GPSMODE=%u", (unsigned)mode);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 5000, 500) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_gps_nmea_output(ct511_t *dev, bool enable)
{
	char cmd[24];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "AT+MGPSGET=ALL,%d", enable ? 1 : 0);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_gps_location(ct511_t *dev, ct511_gps_t *loc)
{
	char rsp[160];
	const char *p;
	int fix = 0, cn = 0;
	double lon = 0, alt = 0, lat = 0;

	if (!loc) {
		return CT511_ERR_ARG;
	}

	if (ct511_cmd(dev, "AT+GPSST", rsp, sizeof rsp, 5000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	p = strstr(rsp, "+GPSST:");

	if (!p) {
		return CT511_ERR;
	}

	/* 手册示例: +GPSST: fix,cn,lon,alt,lat; ... */
	sscanf(p, "+GPSST:%d,%d,%lf,%lf,%lf", &fix, &cn, &lon, &alt, &lat);
	memset(loc, 0, sizeof * loc);
	loc->valid = (fix == 1);
	loc->fix = fix;

	if (fix == 1) {
		loc->lon = lon;
		loc->alt = alt;
		loc->lat = lat;
	}

	return CT511_OK;
}

ct511_result_t ct511_agps_download(ct511_t *dev, const char *server)
{
	char cmd[80];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "AT+AGNSSGET=\"%s\"",
		 server ? server : "pos.asrmicro.com");

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 30000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_agps_apply(ct511_t *dev)
{
	char rsp[64];

	if (ct511_cmd(dev, "AT+AGNSSSET", rsp, sizeof rsp, 15000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

/* ==================== Socket ==================== */

ct511_result_t ct511_sock_set_mode(ct511_t *dev, bool transparent)
{
	char cmd[16];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "AT+CIPMODE=%d", transparent ? 1 : 0);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_sock_mcipcfg(ct511_t *dev, uint16_t seconds)
{
	char cmd[24];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "AT+MCIPCFG=%u", (unsigned)seconds);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

void ct511_sock_enter_transparent(ct511_t *dev)
{
	if (dev && dev->hal && dev->hal->send) {
		/* 手册 5.4.5: 普通 AT 行结束符, 响应为空(静默进入) */
		dev->hal->send((const uint8_t *)"ATO\r\n", 5);
	}
}

void ct511_sock_exit_transparent(ct511_t *dev)
{
	if (dev && dev->hal && dev->hal->send) {
		/* 手册 5.4.6: 指令结尾无结束符(不要加 \r\n) */
		dev->hal->send((const uint8_t *)"+++", 3);
	}
}

ct511_result_t ct511_sock_open(ct511_t *dev, const char *proto,
			       const char *host, uint16_t port, uint8_t cid)
{
	char cmd[160];
	char rsp[96];

	if (!proto || !host) {
		return CT511_ERR_ARG;
	}

	snprintf(cmd, sizeof cmd, "AT+CIPOPEN=%u,\"%s\",\"%s\",%u",
		 (unsigned)cid, proto, host, (unsigned)port);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 15000, 1000) != CT511_OK) {
		/* CIPMODE=1 下 CIPOPEN 成功仅回 '>' 数据提示符(无 OK),
		 * 此时连接已建立, 模块可直接收发数据 */
		return dev->prompt_seen ? CT511_OK : CT511_ERR_TIMEOUT;
	}

	return (ct511_has(rsp, "SUCCESS") || ct511_has(rsp, "OK"))
	       ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_sock_send(ct511_t *dev, uint8_t cid,
			       const uint8_t *data, uint16_t len)
{
	char cmd[32];
	char rsp[96];
	const uint8_t hextail = 0x1A;

	if (!data) {
		return CT511_ERR_ARG;
	}

	snprintf(cmd, sizeof cmd, "AT+CIPSEND=%u", (unsigned)cid);

	if (ct511_send_prompt(dev, cmd, data, len, &hextail, 1,
			      rsp, sizeof rsp, 10000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "SUCCESS") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_sock_close(ct511_t *dev, uint8_t cid)
{
	char cmd[32];
	char rsp[64];
	snprintf(cmd, sizeof cmd, "AT+CIPCLOSE=%u", (unsigned)cid);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 6000, 1000) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return (ct511_has(rsp, "SUCCESS") || ct511_has(rsp, "OK"))
	       ? CT511_OK : CT511_ERR;
}

/* ==================== HTTP ==================== */

ct511_result_t ct511_http_get(ct511_t *dev, const char *url, uint16_t port,
			      char *rsp, uint16_t cap, uint32_t timeout_ms)
{
	char cmd[64 + 256];
	char rb[96];

	if (ct511_cmd(dev, "AT$HTTPOPEN", rb, sizeof rb, 8000, 500) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	snprintf(cmd, sizeof cmd, "AT$HTTPPARA=%s,%u", url, (unsigned)port);

	if (ct511_cmd(dev, cmd, rb, sizeof rb, 8000, 500) != CT511_OK) {
		ct511_cmd(dev, "AT$HTTPCLOSE", rb, sizeof rb, 5000, 300);
		return CT511_ERR_TIMEOUT;
	}

	if (ct511_cmd(dev, "AT$HTTPACTION=0", rsp, cap, timeout_ms, 2000)
	    != CT511_OK) {
		ct511_cmd(dev, "AT$HTTPCLOSE", rb, sizeof rb, 5000, 300);
		return CT511_ERR_TIMEOUT;
	}

	ct511_cmd(dev, "AT$HTTPCLOSE", rb, sizeof rb, 5000, 300);
	return CT511_OK;
}

/* ==================== 电源 ==================== */

ct511_result_t ct511_reboot(ct511_t *dev)
{
	char rsp[32];
	ct511_cmd(dev, "AT+RESET", rsp, sizeof rsp, 5000, 400);
	return CT511_OK;
}

ct511_result_t ct511_sleep(ct511_t *dev, bool enable)
{
	char cmd[24];
	char rsp[32];
	snprintf(cmd, sizeof cmd, "AT+SYSSLEEP=%d", enable ? 1 : 0);

	if (ct511_cmd(dev, cmd, rsp, sizeof rsp, 5000, 300) != CT511_OK) {
		return CT511_ERR_TIMEOUT;
	}

	return ct511_has(rsp, "OK") ? CT511_OK : CT511_ERR;
}

ct511_result_t ct511_poweroff(ct511_t *dev)
{
	char rsp[32];
	ct511_cmd(dev, "AT+POWEROFF", rsp, sizeof rsp, 5000, 300);
	return CT511_OK;
}

/* ==================== NMEA 解析 ==================== */

/** ddmm.mmmm / dddmm.mmmm -> 度 (lat: 2位度, lon: 3位度) */
static double ct511_dm2deg(const char *s, int deg_digits)
{
	char buf[4] = { 0, 0, 0, 0 };
	double deg = 0, min = 0;
	int i;

	if (!s) {
		return 0;
	}

	for (i = 0; i < deg_digits && i < 3 && s[i]; i++) {
		buf[i] = s[i];
	}

	deg = atof(buf);
	min = atof(s + deg_digits);
	return deg + min / 60.0;
}

bool ct511_parse_gpgga(const char *line, ct511_gps_t *out)
{
	const char *p = line;
	char f[16][24];
	int n = 0, i;

	memset(f, 0, sizeof f);

	while (n < 16 && p && *p) {
		i = 0;

		while (*p && *p != ',' && i < 23) {
			f[n][i++] = *p++;
		}

		if (*p == ',') {
			p++;
		}

		n++;
	}

	if (n < 14 || strcmp(f[0], "$GPGGA") != 0) {
		return false;
	}

	memset(out, 0, sizeof * out);
	out->fix = atoi(f[6]);

	if (out->fix == 0) {
		out->valid = false;
		return true;
	}

	strncpy(out->utc, f[1], sizeof(out->utc) - 1);
	out->lat = ct511_dm2deg(f[2], 2);

	if (f[3][0] == 'S') {
		out->lat = -out->lat;
	}

	out->lon = ct511_dm2deg(f[4], 3);

	if (f[5][0] == 'W') {
		out->lon = -out->lon;
	}

	out->satellites = (uint16_t)atoi(f[7]);
	out->alt = atof(f[9]);
	out->valid = true;
	return true;
}

bool ct511_parse_gprmc(const char *line, ct511_gps_t *out)
{
	const char *p = line;
	char f[14][24];
	int n = 0, i;

	memset(f, 0, sizeof f);

	while (n < 14 && p && *p) {
		i = 0;

		while (*p && *p != ',' && i < 23) {
			f[n][i++] = *p++;
		}

		if (*p == ',') {
			p++;
		}

		n++;
	}

	if (n < 12 || strcmp(f[0], "$GPRMC") != 0) {
		return false;
	}

	memset(out, 0, sizeof * out);

	if (f[2][0] != 'A') {
		out->valid = false;
		return true;
	}

	strncpy(out->utc, f[1], sizeof(out->utc) - 1);
	strncpy(out->date, f[9], sizeof(out->date) - 1);
	out->lat = ct511_dm2deg(f[3], 2);

	if (f[4][0] == 'S') {
		out->lat = -out->lat;
	}

	out->lon = ct511_dm2deg(f[5], 3);

	if (f[6][0] == 'W') {
		out->lon = -out->lon;
	}

	out->speed_kmh = atof(f[7]) * 1.852;
	out->course = atof(f[8]);
	out->fix = 1;
	out->valid = true;
	return true;
}
