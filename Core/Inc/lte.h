#ifndef __LTE_H
#define __LTE_H

#include "main.h"
#include "ICM42688P.h"
#include <stdint.h>

/*
 * Air780E AT 固件驱动 (裸机版, USART2)
 * 当前硬件由应用层通过 PB12 控制 4G 模块物理上/断电:
 *   1. 应用层 PB12 拉高, 等模块启动
 *   2. AT / ATE0
 *   3. 等待网络注册和附着
 *   4. AT+MQTTMSGSET=0
 *   5. AT+MCONFIG  (client_id, user, pass)
 *   6. AT+MIPSTART (host, port)
 *   7. AT+MCONNECT=1,60
 *   8. AT+MPUBEX 发送二进制数据
 *   9. 应用层 PB12 拉低, 物理断电并清状态
 *
 * 状态机非阻塞, 主循环里调 lte_task() 即可.
 * 1ms 计数靠主循环里调用 lte_tick(ms_elapsed).
 */

#define LTE_RST_GPIO_Port   GPIOB
#define LTE_RST_Pin         GPIO_PIN_14
#define LTE_DTR_GPIO_Port   GPIOB
#define LTE_DTR_Pin         GPIO_PIN_13
#define LTE_RDY_GPIO_Port   GPIOB
#define LTE_RDY_Pin         GPIO_PIN_15

#define LTE_RX_BUF_SIZE     1024

/* ========== 公开 API ========== */
void lte_init(void);
void lte_tick(uint32_t ms);
void lte_task(void);

/* 连接管理 (突发发送模式):
 *   lte_connect()           - 建立 MQTT 连接, 完成后停在 CONNECTED 待命
 *   lte_request_send()      - 在 CONNECTED 下发一帧, 发完回到 CONNECTED
 *   lte_power_reset_state() - 4G 物理断电后清状态机
 */
void lte_connect(void);
void lte_power_reset_state(void);
void lte_prepare_sleep(void);
void lte_disconnect(void);
void lte_sleep_now(void);
void lte_abort(void);

/* 应用层用: 排队一帧数据 (需已 CONNECTED). 状态机发完回 CONNECTED. */
void lte_request_send(const rs485_data_t *frame);

/* 状态机是否空闲 (1=在 IDLE 且无待发; 0=忙) */
uint8_t lte_is_idle(void);

/* 已建立 MQTT 连接且当前没有待发的一帧 (1=可以发下一帧) */
uint8_t lte_is_connected(void);

/* 连接过程是否失败 (FAIL_WAIT 兜底已触发, 本轮放弃) */
uint8_t lte_is_failed(void);
uint8_t lte_is_at_ready(void);
uint8_t lte_is_net_ready(void);
uint8_t lte_get_rdy_level(void);
uint16_t lte_get_rx_len(void);
const char *lte_get_state_string(void);

/* 串口接收回调入口 (在 HAL_UART_RxCpltCallback 里调用) */
void lte_uart_rx_byte(uint8_t byte);

#endif
