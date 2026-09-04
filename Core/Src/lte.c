/*
 * Air780E AT driver for bare-metal STM32 HAL on USART2.
 * Flow: power on module, register network, connect MQTT, publish binary
 * rs485_data_t frames, then release the module for low-power sleep.
 * USART1 printf logging is available through LTE_DBG, but disabled by default.
 */

#include "lte.h"
#include "usart.h"
#include "main.h"
#include "app_config.h"
#include <string.h>
#include <stdio.h>

/* ===================================================================== */

#define LTE_DBG               0  /* LTE 调试打印开关，1 打开 USART1 printf 日志，0 关闭。 */

#if LTE_DBG
#define LTE_LOG(...)   printf(__VA_ARGS__)  /* LTE 调试日志宏，关闭调试时编译为空操作。 */
#else
#define LTE_LOG(...)   ((void)0)  /* LTE 调试日志宏，关闭调试时编译为空操作。 */
#endif

/* ===================================================================== */

/* ===================================================================== */

extern UART_HandleTypeDef huart2;

/* ===================================================================== */

/* ===================================================================== */

static volatile uint8_t  s_rx_buf[LTE_RX_BUF_SIZE];  /* LTE USART2 接收缓冲区，保存 AT 返回文本。 */
static volatile uint16_t s_rx_len = 0;  /* LTE 接收缓冲区当前有效长度。 */
#if LTE_DBG
static uint16_t s_dump_pos = 0;  /* LTE 调试打印已输出到的位置。 */
#endif

/**
 * @brief lte_rx_clear
 */
static void lte_rx_clear(void)
{
    __disable_irq();
    s_rx_len = 0;
    memset((void *)s_rx_buf, 0, sizeof(s_rx_buf));
#if LTE_DBG
    s_dump_pos = 0;
#endif
    __enable_irq();
}

/**
 * @brief lte_uart_rx_byte
 * @param byte 串口收到的单字节。
 */
void lte_uart_rx_byte(uint8_t byte)
{
    if (s_rx_len < LTE_RX_BUF_SIZE - 1)
    {
        s_rx_buf[s_rx_len++] = byte;
        s_rx_buf[s_rx_len] = 0;
    }
}

/**
 * @brief lte_rx_contains
 * @param needle 要在接收缓冲区查找的关键字。
 */
static int lte_rx_contains(const char *needle)
{
    if (s_rx_len == 0) return 0;
    return strstr((const char *)s_rx_buf, needle) != NULL;
}

/* ===================================================================== */
/*  GPIO                                                                  */
/* ===================================================================== */

/**
 * @brief lte_rst_low
 */
static void lte_rst_low(void)  { HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_RESET); }

/**
 * @brief lte_rst_high
 */
static void lte_rst_high(void) { HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_SET);   }

/**
 * @brief lte_dtr_low
 */
static void lte_dtr_low(void)  { HAL_GPIO_WritePin(LTE_DTR_GPIO_Port, LTE_DTR_Pin, GPIO_PIN_RESET); }

/**
 * @brief lte_dtr_high
 */
static void lte_dtr_high(void) { HAL_GPIO_WritePin(LTE_DTR_GPIO_Port, LTE_DTR_Pin, GPIO_PIN_SET);   }

/**
 * @brief lte_rdy_is_high
 */
static int  lte_rdy_is_high(void)
{
    return HAL_GPIO_ReadPin(LTE_RDY_GPIO_Port, LTE_RDY_Pin) == GPIO_PIN_SET;
}

/**
 * @brief lte_dtr_sleep
 */
static void lte_dtr_sleep(void) { lte_dtr_high(); }

/**
 * @brief lte_dtr_wake
 */
static void lte_dtr_wake(void)  { lte_dtr_low();  }

/* ===================================================================== */

/* ===================================================================== */

/**
 * @brief lte_uart_send
 * @param data 待发送或待处理数据。
 * @param len 数据长度。
 */
static void lte_uart_send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 1000);
}

/**
 * @brief lte_send_cmd
 * @param cmd 命令字符串。
 */
static void lte_send_cmd(const char *cmd)
{
    lte_rx_clear();
    if (cmd && cmd[0])
    {
        LTE_LOG("[LTE] >> %s\r\n", cmd);
        lte_uart_send((const uint8_t *)cmd, (uint16_t)strlen(cmd));
    }
    lte_uart_send((const uint8_t *)"\r\n", 2);
}

/* ===================================================================== */

/* ===================================================================== */

typedef enum {
    LTE_S_IDLE = 0,

    LTE_S_WAKEUP,
    LTE_S_PROBE,
    LTE_S_RST,
    LTE_S_RST_WAIT,

    LTE_S_AT,
    LTE_S_ATE0,
    LTE_S_CGREG,
    LTE_S_CGATT,

    LTE_S_MQTTMSGSET,
    LTE_S_MCONFIG,
    LTE_S_MIPSTART,
    LTE_S_MCONNECT,

    LTE_S_CONNECTED,

    LTE_S_PUB_HEADER,       /* AT+MPUBEX=topic,0,0,len ? ">" */
    LTE_S_PUB_PAYLOAD,

    LTE_S_DISCONNECT,
    LTE_S_IPCLOSE,
    LTE_S_SLEEP,

    LTE_S_FAIL_WAIT,
} lte_state_t;

typedef struct {
    lte_state_t state;
    uint32_t    timer_ms;
    uint32_t    timeout_ms;
    uint8_t     retry;
    uint8_t     entered;
    uint8_t     net_ready;

    uint8_t     want_connect;
    uint8_t     want_prepare_sleep;
    uint8_t     want_disconnect;
    uint8_t     failed;
    uint8_t     at_ready;

    uint8_t     pending_send;
    rs485_data_t pending_frame;
} lte_ctx_t;

static lte_ctx_t L;  /* LTE 状态机上下文，保存状态、定时、重试和待发帧。 */

static const char *lte_state_name(lte_state_t s)
{
    switch (s)
    {
    case LTE_S_IDLE:        return "IDLE";
    case LTE_S_WAKEUP:      return "WAKEUP";
    case LTE_S_PROBE:       return "PROBE";
    case LTE_S_RST:         return "RST";
    case LTE_S_RST_WAIT:    return "RST_WAIT";
    case LTE_S_AT:          return "AT";
    case LTE_S_ATE0:        return "ATE0";
    case LTE_S_CGREG:       return "CGREG";
    case LTE_S_CGATT:       return "CGATT";
    case LTE_S_MQTTMSGSET:  return "MQTTMSGSET";
    case LTE_S_MCONFIG:     return "MCONFIG";
    case LTE_S_MIPSTART:    return "MIPSTART";
    case LTE_S_MCONNECT:    return "MCONNECT";
    case LTE_S_CONNECTED:   return "CONNECTED";
    case LTE_S_PUB_HEADER:  return "PUB_HEADER";
    case LTE_S_PUB_PAYLOAD: return "PUB_PAYLOAD";
    case LTE_S_DISCONNECT:  return "DISCONNECT";
    case LTE_S_IPCLOSE:     return "IPCLOSE";
    case LTE_S_SLEEP:       return "SLEEP";
    case LTE_S_FAIL_WAIT:   return "FAIL_WAIT";
    default:                return "?";
    }
}

/**
 * @brief lte_goto
 * @param s 状态枚举或字符串指针。
 * @param timeout_ms 超时时间，单位 ms。
 */
static void lte_goto(lte_state_t s, uint32_t timeout_ms)
{
    LTE_LOG("[LTE] %s -> %s (to=%lums)\r\n",
            lte_state_name(L.state), lte_state_name(s), (unsigned long)timeout_ms);
    L.state = s;
    L.timer_ms = 0;
    L.timeout_ms = timeout_ms;
    L.entered = 0;
    lte_rx_clear();
}

/* ===================================================================== */

/* ===================================================================== */

/**
 * @brief lte_tick
 * @param ms 经过的毫秒数。
 */
void lte_tick(uint32_t ms)
{
    L.timer_ms += ms;
}

/* ===================================================================== */

/* ===================================================================== */

/**
 * @brief lte_connect
 */
void lte_connect(void)
{
    if (L.state == LTE_S_IDLE)
    {
        L.want_connect = 1;
        L.want_prepare_sleep = 0;
        L.failed = 0;
    }
}

/**
 * @brief lte_prepare_sleep
 */
void lte_prepare_sleep(void)
{
    if (L.state == LTE_S_IDLE)
    {
        L.want_prepare_sleep = 1;
        L.want_connect = 0;
        L.failed = 0;
    }
}

/**
 * @brief lte_disconnect
 */
void lte_disconnect(void)
{
    L.want_disconnect = 1;
}

/**
 * @brief lte_sleep_now
 */
void lte_sleep_now(void)
{
    lte_dtr_sleep();
    L.state = LTE_S_IDLE;
    L.pending_send = 0;
    L.want_connect = 0;
    L.want_prepare_sleep = 0;
    L.want_disconnect = 0;
    L.failed = 0;
}

/**
 * @brief lte_abort
 */
void lte_abort(void)
{
    lte_send_cmd("AT+MIPCLOSE");
    lte_dtr_sleep();
    L.net_ready = 0;
    L.state = LTE_S_IDLE;
    L.pending_send = 0;
    L.want_connect = 0;
    L.want_prepare_sleep = 0;
    L.want_disconnect = 0;
    L.failed = 1;
}

/**
 * @brief lte_power_reset_state
 */
void lte_power_reset_state(void)
{
    memset(&L, 0, sizeof(L));
    L.state = LTE_S_IDLE;
    lte_rx_clear();
}

/**
 * @brief lte_request_send
 * @param frame 待发送的数据帧指针。
 */
void lte_request_send(const rs485_data_t *frame)
{
    if (frame == NULL) return;
    L.pending_frame = *frame;
    L.pending_send = 1;
}

/**
 * @brief lte_is_idle
 */
uint8_t lte_is_idle(void)
{
    return (L.state == LTE_S_IDLE && L.pending_send == 0 &&
            L.want_connect == 0 && L.want_prepare_sleep == 0) ? 1 : 0;
}

/**
 * @brief lte_is_connected
 */
uint8_t lte_is_connected(void)
{
    return (L.state == LTE_S_CONNECTED && L.pending_send == 0) ? 1 : 0;
}

/**
 * @brief lte_is_failed
 */
uint8_t lte_is_failed(void)
{
    return L.failed;
}

/**
 * @brief lte_is_at_ready
 */
uint8_t lte_is_at_ready(void)
{
    return L.at_ready;
}

/**
 * @brief lte_is_net_ready
 */
uint8_t lte_is_net_ready(void)
{
    return L.net_ready;
}

/**
 * @brief lte_get_rdy_level
 */
uint8_t lte_get_rdy_level(void)
{
    return (uint8_t)lte_rdy_is_high();
}

/**
 * @brief lte_get_rx_len
 */
uint16_t lte_get_rx_len(void)
{
    return s_rx_len;  /* LTE 接收缓冲区当前有效长度。 */
}

const char *lte_get_state_string(void)
{
    return lte_state_name(L.state);
}

/* ===================================================================== */

/* ===================================================================== */

/**
 * @brief lte_init
 */
void lte_init(void)
{

    lte_dtr_wake();
    lte_rst_low();

    memset(&L, 0, sizeof(L));
    L.state = LTE_S_IDLE;

    LTE_LOG("\r\n[LTE] init done, awake\r\n");
}

/* ===================================================================== */

/* ===================================================================== */

/**
 * @brief lte_do_publish_header
 */
static void lte_do_publish_header(void)
{
    const app_config_t *cfg = AppConfig_Get();
    char cmd[160];
    int len = (int)sizeof(rs485_data_t);

    snprintf(cmd, sizeof(cmd),
             "AT+MPUBEX=\"%s\",0,0,%d",
             cfg->mqtt_pub_topic, len);
    lte_send_cmd(cmd);
}

/**
 * @brief lte_do_publish_payload
 */
static void lte_do_publish_payload(void)
{
    lte_rx_clear();
    /* Send binary rs485_data_t payload without CRLF. */
    lte_uart_send((const uint8_t *)&L.pending_frame, (uint16_t)sizeof(rs485_data_t));
}

#if LTE_DBG

/**
 * @brief lte_dump_rx
 */
static void lte_dump_rx(void)
{
    if (s_rx_len < s_dump_pos) s_dump_pos = 0;
    while (s_dump_pos < s_rx_len)
    {
        uint8_t c = s_rx_buf[s_dump_pos++];
        if (c == '\r' || c == '\n' || (c >= 0x20 && c < 0x7F))
            putchar(c);
        else
            printf("\\x%02X", c);
    }
}
#else
#define lte_dump_rx() ((void)0)
#endif

/**
 * @brief lte_log_fail
 * @param reason 失败原因字符串。
 */
static void lte_log_fail(const char *reason)
{
#if LTE_DBG
    printf("\r\n[LTE] FAIL: %s, state=%s, retry=%u, timer=%lums, rx_len=%u\r\n",
           reason,
           lte_state_name(L.state),
           (unsigned)L.retry,
           (unsigned long)L.timer_ms,
           (unsigned)s_rx_len);
    if (s_rx_len > 0)
    {
        printf("[LTE] RXBUF: ");
        for (uint16_t i = 0; i < s_rx_len; i++)
        {
            uint8_t c = s_rx_buf[i];
            if (c == '\r' || c == '\n' || (c >= 0x20 && c < 0x7F))
                putchar(c);
            else
                printf("\\x%02X", c);
        }
        printf("\r\n");
    }
#endif
}

/**
 * @brief lte_task
 */
void lte_task(void)
{
    lte_dump_rx();

    switch (L.state)
    {

    case LTE_S_IDLE:
        if (L.want_connect || L.want_prepare_sleep)
        {
            L.want_disconnect = 0;
            lte_dtr_wake();
            lte_goto(LTE_S_WAKEUP, 5000);
            L.retry = 0;
        }
        break;

    case LTE_S_WAKEUP:
        if (lte_rdy_is_high() || L.timer_ms >= L.timeout_ms)
        {
            if (L.net_ready)
                lte_goto(LTE_S_PROBE, 2000);
            else
                lte_goto(LTE_S_AT, 2000);
        }
        break;

    case LTE_S_PROBE:
        if (!L.entered || L.timer_ms >= L.timeout_ms)
        {
            if (L.retry >= 6)
            {

                L.net_ready = 0;
                lte_goto(LTE_S_RST, 50);
                break;
            }
            lte_send_cmd("AT");
            L.retry++;
            L.entered = 1;
            L.timer_ms = 0;
            L.timeout_ms = 800;
        }
        if (lte_rx_contains("OK"))
        {
            L.retry = 0;
            lte_goto(LTE_S_MQTTMSGSET, 2000);
        }
        break;

    case LTE_S_RST:
        lte_rst_high();
        if (L.timer_ms >= 200)        /* 200ms ? */
        {
            lte_rst_low();
            lte_goto(LTE_S_RST_WAIT, 8000);
        }
        break;

    case LTE_S_RST_WAIT:
        if (lte_rdy_is_high() && L.timer_ms >= 2000)
        {
            lte_goto(LTE_S_AT, 2000);
        }
        else if (L.timer_ms >= L.timeout_ms)
        {
            lte_goto(LTE_S_AT, 2000);
        }
        break;

    /* ---- AT ---- */
    case LTE_S_AT:
        if (!L.entered || L.timer_ms >= L.timeout_ms)
        {
            if (L.retry >= 5) { lte_log_fail("AT no response"); lte_goto(LTE_S_FAIL_WAIT, 3000); break; }
            lte_send_cmd("AT");
            L.retry++;
            L.entered = 1;
            L.timer_ms = 0;
        }
        if (lte_rx_contains("OK")) { L.retry = 0; lte_goto(LTE_S_ATE0, 2000); }
        break;

    /* ---- ATE0 ---- */
    case LTE_S_ATE0:
        if (!L.entered || L.timer_ms >= L.timeout_ms)
        {
            if (L.retry >= 3) { lte_log_fail("ATE0 no response"); lte_goto(LTE_S_FAIL_WAIT, 3000); break; }
            lte_send_cmd("ATE0");
            L.retry++;
            L.entered = 1;
            L.timer_ms = 0;
        }
        if (lte_rx_contains("OK"))
        {
            L.retry = 0;
            L.at_ready = 1;
            lte_goto(LTE_S_CGREG, 3000);
        }
        break;

    case LTE_S_CGREG:
        if (!L.entered || L.timer_ms >= L.timeout_ms)
        {
            if (L.retry >= 30) { lte_log_fail("CGREG not registered"); lte_goto(LTE_S_FAIL_WAIT, 3000); break; }
            lte_send_cmd("AT+CGREG?");
            L.retry++;
            L.entered = 1;
            L.timer_ms = 0;
            L.timeout_ms = 2000;
        }
        if (lte_rx_contains("+CGREG: 0,1") || lte_rx_contains("+CGREG: 0,5"))
        {
            L.retry = 0;
            lte_goto(LTE_S_CGATT, 3000);
        }
        break;

    case LTE_S_CGATT:
        if (!L.entered || L.timer_ms >= L.timeout_ms)
        {
            if (L.retry >= 30) { lte_log_fail("CGATT not attached"); lte_goto(LTE_S_FAIL_WAIT, 3000); break; }
            lte_send_cmd("AT+CGATT?");
            L.retry++;
            L.entered = 1;
            L.timer_ms = 0;
            L.timeout_ms = 2000;
        }
        if (lte_rx_contains("+CGATT: 1"))
        {
            L.retry = 0;
            L.net_ready = 1;
            lte_goto(LTE_S_MQTTMSGSET, 2000);
        }
        break;

    case LTE_S_MQTTMSGSET:
        if (!L.entered)
        {
            lte_send_cmd("AT+MQTTMSGSET=0");
            L.entered = 1;
        }
        if (lte_rx_contains("OK"))      lte_goto(LTE_S_MCONFIG, 3000);
        else if (lte_rx_contains("ERROR")) { lte_log_fail("MQTTMSGSET ERROR"); lte_goto(LTE_S_FAIL_WAIT, 3000); }
        else if (L.timer_ms >= L.timeout_ms) { lte_log_fail("MQTTMSGSET timeout"); lte_goto(LTE_S_FAIL_WAIT, 3000); }
        break;

    case LTE_S_MCONFIG:
        if (!L.entered)
        {
            const app_config_t *cfg = AppConfig_Get();
            char cmd[220];
            snprintf(cmd, sizeof(cmd),
                     "AT+MCONFIG=\"%s\",\"%s\",\"%s\"",
                     cfg->mqtt_client_id, cfg->mqtt_user, cfg->mqtt_pass);
            lte_send_cmd(cmd);
            L.entered = 1;
        }
        if (lte_rx_contains("OK"))         lte_goto(LTE_S_MIPSTART, 5000);
        else if (lte_rx_contains("ERROR")) { lte_log_fail("MCONFIG ERROR"); lte_goto(LTE_S_FAIL_WAIT, 3000); }
        else if (L.timer_ms >= L.timeout_ms) { lte_log_fail("MCONFIG timeout"); lte_goto(LTE_S_FAIL_WAIT, 3000); }
        break;

    case LTE_S_MIPSTART:
        if (!L.entered)
        {
            const app_config_t *cfg = AppConfig_Get();
            char cmd[128];
            snprintf(cmd, sizeof(cmd),
                     "AT+MIPSTART=\"%s\",\"%s\"",
                     cfg->mqtt_host, cfg->mqtt_port);
            lte_send_cmd(cmd);
            L.entered = 1;
        }
        if (lte_rx_contains("CONNECT OK")) lte_goto(LTE_S_MCONNECT, 5000);
        else if (lte_rx_contains("ERROR")) { lte_log_fail("MIPSTART ERROR"); lte_goto(LTE_S_FAIL_WAIT, 3000); }
        else if (L.timer_ms >= L.timeout_ms) { lte_log_fail("MIPSTART timeout"); lte_goto(LTE_S_FAIL_WAIT, 3000); }
        break;

    /* ---- MQTT CONNECT ---- */
    case LTE_S_MCONNECT:
        if (!L.entered)
        {
            lte_send_cmd("AT+MCONNECT=1,60");
            L.entered = 1;
        }
        if (lte_rx_contains("CONNACK OK")) lte_goto(LTE_S_CONNECTED, 0);
        else if (lte_rx_contains("ERROR")) { lte_log_fail("MCONNECT ERROR"); lte_goto(LTE_S_FAIL_WAIT, 3000); }
        else if (L.timer_ms >= L.timeout_ms) { lte_log_fail("MCONNECT timeout"); lte_goto(LTE_S_FAIL_WAIT, 3000); }
        break;

    case LTE_S_CONNECTED:
        if (L.pending_send)
        {
            lte_goto(LTE_S_PUB_HEADER, 3000);
        }
        else if (L.want_disconnect)
        {
            L.want_disconnect = 0;
            lte_goto(LTE_S_DISCONNECT, 3000);
        }
        break;

    /* ---- AT+MPUBEX, ? '>' ---- */
    case LTE_S_PUB_HEADER:
        if (!L.entered)
        {
            lte_do_publish_header();
            L.entered = 1;
        }
        if (lte_rx_contains(">"))
        {
            lte_goto(LTE_S_PUB_PAYLOAD, 3000);
        }
        else if (lte_rx_contains("ERROR"))
        {

            L.pending_send = 0;
            lte_goto(LTE_S_CONNECTED, 0);
        }
        else if (L.timer_ms >= L.timeout_ms)
        {
            L.pending_send = 0;
            lte_goto(LTE_S_CONNECTED, 0);
        }
        break;

    /* ---- ? payload, ? OK ---- */
    case LTE_S_PUB_PAYLOAD:
        if (!L.entered)
        {
            lte_do_publish_payload();
            L.entered = 1;
        }
        if (lte_rx_contains("OK"))
        {
            L.pending_send = 0;
            LTE_LOG("[LTE] *** PUBLISH OK ***\r\n");
            lte_goto(LTE_S_CONNECTED, 0);
        }
        else if (lte_rx_contains("ERROR"))
        {

            L.pending_send = 0;
            lte_goto(LTE_S_CONNECTED, 0);
        }
        else if (L.timer_ms >= L.timeout_ms)
        {
            L.pending_send = 0;
            lte_goto(LTE_S_CONNECTED, 0);
        }
        break;

    /* ---- ? MQTT ---- */
    case LTE_S_DISCONNECT:
        if (!L.entered)
        {
            lte_send_cmd("AT+MDISCONNECT");
            L.entered = 1;
        }
        if (lte_rx_contains("OK") || lte_rx_contains("DISCONNECT") ||
            L.timer_ms >= L.timeout_ms)
        {
            lte_goto(LTE_S_IPCLOSE, 3000);
        }
        break;

    /* ---- ? TCP ---- */
    case LTE_S_IPCLOSE:
        if (!L.entered)
        {
            lte_send_cmd("AT+MIPCLOSE");
            L.entered = 1;
        }
        if (lte_rx_contains("OK") || lte_rx_contains("CLOSE") ||
            L.timer_ms >= L.timeout_ms)
        {
            lte_goto(LTE_S_SLEEP, 1000);
        }
        break;

    case LTE_S_SLEEP:
        if (!L.entered)
        {
            lte_dtr_sleep();
            L.entered = 1;
        }
        if (L.timer_ms >= L.timeout_ms)
        {
            L.state = LTE_S_IDLE;
            L.pending_send = 0;
        }
        break;

    case LTE_S_FAIL_WAIT:
        L.net_ready = 0;
        L.state = LTE_S_IDLE;
        L.pending_send = 0;
        L.want_connect = 0;
        L.want_disconnect = 0;
        L.failed = 1;
        break;

    default:
        L.state = LTE_S_IDLE;
        break;
    }
}

