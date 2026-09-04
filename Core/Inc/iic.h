#ifndef __IIC_H
#define __IIC_H

#include "main.h"
#include <stdint.h>

#define IIC_OK      1U
#define IIC_ERROR   0U

void IIC1_Init(void);  /* 初始化 I2C1，用于 BQ27220 通信。 */
uint8_t IIC1_IsDeviceReady(uint8_t dev_addr_7bit);  /* 用默认参数检测指定 7 位地址设备是否在线。 */
uint8_t IIC1_IsDeviceReadyTimeout(uint8_t dev_addr_7bit, uint32_t trials, uint32_t timeout_ms);  /* 用指定重试次数和超时时间检测 I2C 设备是否在线。 */
uint8_t IIC1_ReadRegU16(uint8_t dev_addr_7bit, uint8_t reg, uint16_t *value);  /* 按 BQ27220 小端格式读取 16 位寄存器。 */
uint8_t IIC1_GetLastHalStatus(void);  /* 获取最近一次 HAL I2C 状态。 */
uint32_t IIC1_GetLastHalError(void);  /* 获取最近一次 HAL I2C 错误码。 */

#endif
