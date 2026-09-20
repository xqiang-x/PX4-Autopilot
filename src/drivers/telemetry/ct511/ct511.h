/**
 * @file ct511.h
 * @brief DX-CT511/CT511N 4G 模块 C 驱动库 (MCU 移植版)
 *
 * AT 指令依据: 《DX-CT511&CT511N_串口UART_应用指导》V2.1
 *
 * 特性:
 *   - 无动态内存分配, 全部缓冲区由用户提供
 *   - 串口收发通过 HAL 函数指针接入, 支持两种接收方式(可同时):
 *       1) UART 中断 -> ct511_rx_bytes() 写入环形缓冲
 *       2) HAL 轮询接收 -> hal->recv 回调
 *   - 事件驱动: 模块主动上报(下行 MQTT 消息/NMEA/其他)通过事件回调分发
 *   - 命令 API 为阻塞式(内部轮询 HAL->millis 超时), 禁止在中断里调用
 *
 * 移植步骤:
 *   1. 实现 ct511_hal_t 三个函数 (send/millis/可选 recv), 参考 ct511_hal_template.c
 *   2. 提供接收环形缓冲数组 (建议 >= 512 字节)
 *   3. UART 接收中断里调用 ct511_rx_bytes()
 *   4. 主循环周期调用 ct511_poll() 处理主动上报, 需要时调用各命令 API
 *
 * 使用示例见 example_main.c, 主机侧逻辑测试见 tests/test_host.c
 */
#ifndef CT511_H
#define CT511_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 可配置参数(移植可覆盖) ==================== */

#ifndef CT511_LINE_MAX
#define CT511_LINE_MAX 512U /* 行缓冲区大小(含结尾0)。
                              MQTT 下行消息正文长度受限于此 */
#endif
#ifndef CT511_TOPIC_MAX
#define CT511_TOPIC_MAX 64U /* MQTT 主题最大长度 */
#endif
#ifndef CT511_PAYLOAD_MAX
#define CT511_PAYLOAD_MAX (CT511_LINE_MAX - 64U) /* MQTT 消息正文最大长度 */
#endif

#define CT511_MPUB_MAX   512U  /* AT+MPUB 单条消息上限 (手册 5.5.4) */
#define CT511_MPUBEX_MAX 4096U /* AT+MPUBEX 单条消息上限 */

#define CT511_VERSION "1.1.0"

/* ==================== 类型定义 ==================== */

struct ct511;

/** HAL: 平台适配层 */
typedef struct ct511_hal {
	/** 串口发送, 在命令上下文(主循环)中调用 */
	void (*send)(const uint8_t *data, uint16_t len);
	/** 毫秒时间戳, 用于超时 */
	uint32_t (*millis)(void);
	/** [可选] 轮询接收: 从串口读最多 max_len 字节, 返回实际读取数, 0 表示无数据 */
	uint16_t (*recv)(uint8_t *data, uint16_t max_len);
	/** [可选] 睡眠等待: 有 OS 的平台上提供, 让命令等待/轮询让出 CPU;
	 *  置 NULL 则退化为忙等轮询(无 OS 的 MCU 上行为不变) */
	void (*delay_ms)(uint32_t ms);
} ct511_hal_t;

/** 操作结果 */
typedef enum {
	CT511_OK = 0,
	CT511_ERR = -1,         /* 命令失败(引擎收到 ERROR) */
	CT511_ERR_TIMEOUT = -2, /* 等待超时 */
	CT511_ERR_ARG = -3,     /* 参数错误 */
} ct511_result_t;

/** 事件类型 */
typedef enum {
	CT511_EVT_PUSH = 0,  /* 其他主动上报, data=(const char*)行文本 */
	CT511_EVT_NMEA,      /* NMEA 语句, data=(const char*)行文本 */
	CT511_EVT_MQTT_MSG,  /* 收到 MQTT 消息, data=(ct511_mqtt_msg_t*) */
} ct511_evt_t;

/** MQTT 下行消息 */
typedef struct {
	char topic[CT511_TOPIC_MAX];
	char payload[CT511_PAYLOAD_MAX];
	uint16_t payload_len;
} ct511_mqtt_msg_t;

/** GNSS 定位信息 (WGS-84) */
typedef struct {
	bool valid;          /* 是否有效 */
	int fix;             /* 0=未定位 1=已定位 */
	double lat;          /* 纬度 */
	double lon;          /* 经度 */
	double alt;          /* 海拔(m) */
	double speed_kmh;    /* 速度(km/h, 仅 RMC) */
	double course;       /* 航向(度, 仅 RMC) */
	uint16_t satellites; /* 卫星数(仅 GGA) */
	char utc[16];        /* UTC 时间 hhmmss.ss */
	char date[16];       /* 日期 ddmmyy (仅 RMC) */
} ct511_gps_t;

/** 事件回调: 在 ct511_poll()/命令等待期间(主循环上下文)调用 */
typedef void (*ct511_event_cb_t)(struct ct511 *dev, int evt, const void *data);

/** 设备对象(用户声明/静态分配) */
typedef struct ct511 {
	const ct511_hal_t *hal;
	ct511_event_cb_t event_cb;

	/* 接收环形缓冲: 中断写 head, 主循环读 tail */
	uint8_t *rx_buf;
	uint16_t rx_size;
	volatile uint16_t rx_head;
	uint16_t rx_tail;

	/* 行装配 */
	char line[CT511_LINE_MAX];
	uint16_t line_len;
	bool line_overrun;

	/* 命令状态机 */
	bool in_cmd;
	bool collecting;
	volatile bool done;
	bool prompt_seen;
	const char *cmd_echo; /* 回显过滤: 与发送命令相同的整行忽略 */
	char *rsp;
	uint16_t rsp_cap;
	uint16_t rsp_len;

	/* 事件缓冲 */
	ct511_mqtt_msg_t mqtt_msg;
	bool mqtt_new;
} ct511_t;

/* ==================== 初始化 / 底层 ==================== */

/**
 * 初始化设备. rx_buf 为用户提供环形缓冲(建议 512 字节以上).
 * hal->send/millis 必须提供; recv 可选.
 */
ct511_result_t ct511_init(ct511_t *dev, const ct511_hal_t *hal,
			  uint8_t *rx_buf, uint16_t rx_buf_size,
			  ct511_event_cb_t event_cb);

void ct511_set_event_cb(ct511_t *dev, ct511_event_cb_t cb);

/** UART 接收中断/回调中调用: 把收到的字节喂给库(可多次调用) */
void ct511_rx_bytes(ct511_t *dev, const uint8_t *data, uint16_t len);

/** 在主循环周期调用: 处理接收数据并分发事件(空闲时也会触发) */
void ct511_poll(ct511_t *dev);

/**
 * 发送 AT 指令并等待 OK/ERROR, 返回时可为响应(行以'\n'分隔, 含OK/ERROR行).
 * linger_ms: 收到 OK 后继续收尾等待(用于 +XXX:SUCCESS 等后置结果行).
 * 返回 CT511_OK 仅代表得到 OK/ERROR; 具体结果需解析 rsp.
 */
ct511_result_t ct511_cmd(ct511_t *dev, const char *at,
			 char *rsp, uint16_t rsp_cap,
			 uint32_t timeout_ms, uint32_t linger_ms);

/**
 * 数据模式指令(AT+MPUBEX / AT+CIPSEND): 等待 '>' 提示符后发送数据,
 * 数据发送完可附加 tail(如 HEX 1A 结束符). 返回后续响应行.
 */
ct511_result_t ct511_send_prompt(ct511_t *dev, const char *at,
				 const uint8_t *data, uint16_t len,
				 const uint8_t *tail, uint16_t tail_len,
				 char *rsp, uint16_t rsp_cap,
				 uint32_t timeout_ms, uint32_t linger_ms);

/** rsp 中是否包含 token */
bool ct511_has(const char *rsp, const char *token);

/** 阻塞等待一条 MQTT 下行消息(同时会触发事件回调), 超时返回 NULL */
const ct511_mqtt_msg_t *ct511_mqtt_wait_message(ct511_t *dev, uint32_t timeout_ms);

/* ==================== 基础指令 ==================== */

ct511_result_t ct511_test(ct511_t *dev);                    /* AT */
ct511_result_t ct511_set_echo(ct511_t *dev, bool enable);   /* ATE0/1 */
/** ATI 原始文本拷入 out */
ct511_result_t ct511_info(ct511_t *dev, char *out, uint16_t cap);
ct511_result_t ct511_get_imei(ct511_t *dev, char *out, uint16_t cap);
ct511_result_t ct511_get_iccid(ct511_t *dev, char *out, uint16_t cap);

/* ==================== 网络服务 ==================== */

/** AT+CSQ -> RSSI(0-31), 失败返回 -1 */
int ct511_csq(ct511_t *dev);
/** AT+CEREG? -> *stat 0-5, 1/5 已注册 */
ct511_result_t ct511_cereg(ct511_t *dev, int *stat);

/** AT+QICSGP=1,1,"apn","user","pwd" (apn 传 NULL 表示空=模块默认) */
ct511_result_t ct511_set_apn(ct511_t *dev, const char *apn,
			     const char *user, const char *pwd);
/** 开启数据网络并轮询等待(<= timeout_ms) */
ct511_result_t ct511_netopen(ct511_t *dev, uint32_t timeout_ms);
ct511_result_t ct511_netclose(ct511_t *dev);
bool ct511_netopen_status(ct511_t *dev);

/** AT+QNTP 校时, 成功拷入 time_str */
ct511_result_t ct511_ntp_sync(ct511_t *dev, const char *server, uint16_t port,
			      char *time_str, uint16_t cap);

/* ==================== MQTT ==================== */

/** AT+MCONFIG (user/pwd 可传 NULL) */
ct511_result_t ct511_mqtt_config(ct511_t *dev, const char *client_id,
				 const char *user, const char *pwd);
/** AT+MIPSTART, version 为 0 时不指定(默认 3.1.1) */
ct511_result_t ct511_mqtt_server(ct511_t *dev, const char *host,
				 uint16_t port, uint16_t version);
/** AT+MCONNECT */
ct511_result_t ct511_mqtt_connect(ct511_t *dev, int clean_session,
				  uint16_t keepalive);
/** 一键连接: 清理旧会话 + 配置 + 连接 */
ct511_result_t ct511_mqtt_open(ct511_t *dev, const char *client_id,
			       const char *host, uint16_t port,
			       const char *user, const char *pwd);
bool ct511_mqtt_is_connected(ct511_t *dev);
/** 发布: <=512 字节走 AT+MPUB, 更长走 AT+MPUBEX(数据模式) */
ct511_result_t ct511_mqtt_publish(ct511_t *dev, const char *topic,
				  const char *payload, int qos, int retain);
ct511_result_t ct511_mqtt_subscribe(ct511_t *dev, const char *topic, int qos);
ct511_result_t ct511_mqtt_unsubscribe(ct511_t *dev, const char *topic);
ct511_result_t ct511_mqtt_disconnect(ct511_t *dev);
ct511_result_t ct511_mqtt_close(ct511_t *dev);

/* ==================== GNSS (仅 CT511N) ==================== */

/** AT+MGPSC 开关; 开启失败且 active_antenna_retry=1 时自动重试有源天线序列 */
ct511_result_t ct511_gps_set(ct511_t *dev, bool on, bool active_antenna_retry);
/** AT+GPSMODE: 1热启动 2温启动 3冷启动 */
ct511_result_t ct511_gps_mode(ct511_t *dev, uint8_t mode);
ct511_result_t ct511_gps_nmea_output(ct511_t *dev, bool enable);
/** AT+GPSST -> 定位信息 */
ct511_result_t ct511_gps_location(ct511_t *dev, ct511_gps_t *loc);
ct511_result_t ct511_agps_download(ct511_t *dev, const char *server);
ct511_result_t ct511_agps_apply(ct511_t *dev);

/* ==================== TCP/UDP Socket ==================== */

/** AT+CIPMODE=0/1 设置传输模式(透传模式须在 NETOPEN/连接之前设置, 且连接号必须为 0) */
ct511_result_t ct511_sock_set_mode(ct511_t *dev, bool transparent);
/** AT+MCIPCFG=<sec> 设置 TCP/UDP 心跳间隔(0-7200 秒, 0=关闭); 须在建立连接前 */
ct511_result_t ct511_sock_mcipcfg(ct511_t *dev, uint16_t seconds);
/** AT+CIPOPEN=cid,"TCP|UDP",host,port */
ct511_result_t ct511_sock_open(ct511_t *dev, const char *proto,
			       const char *host, uint16_t port, uint8_t cid);
/** AT+CIPSEND=cid 数据模式发送(附件 HEX 1A 结束) */
ct511_result_t ct511_sock_send(ct511_t *dev, uint8_t cid,
			       const uint8_t *data, uint16_t len);
ct511_result_t ct511_sock_close(ct511_t *dev, uint8_t cid);

/**
 * 透传模式原始指令(手册 5.4.5/5.4.6, 只发送原始字节, 不等待任何响应;
 * 由调用方负责: 进入前 UART 已空闲, 退出 '+++' 前后各留 >=1s 静默, 发送后自行切状态)
 */
/** 进入透传: 发送 "ATO\r\n" (仅在 CIPMODE=1 且连接建立后有效) */
void ct511_sock_enter_transparent(ct511_t *dev);
/** 退出透传: 发送 "+++", 注意无回车换行结束符 */
void ct511_sock_exit_transparent(ct511_t *dev);

/* ==================== HTTP ==================== */

/** GET: 按手册流程 open->para->action->close, 响应拷入 rsp */
ct511_result_t ct511_http_get(ct511_t *dev, const char *url, uint16_t port,
			      char *rsp, uint16_t cap, uint32_t timeout_ms);

/* ==================== 电源 ==================== */

ct511_result_t ct511_reboot(ct511_t *dev);              /* AT+RESET */
ct511_result_t ct511_sleep(ct511_t *dev, bool enable);  /* AT+SYSSLEEP */
ct511_result_t ct511_poweroff(ct511_t *dev);            /* AT+POWEROFF */

/* ==================== NMEA 解析(独立工具) ==================== */

bool ct511_parse_gpgga(const char *line, ct511_gps_t *out);
bool ct511_parse_gprmc(const char *line, ct511_gps_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CT511_H */
