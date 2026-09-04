#ifndef __BQ27220_H
#define __BQ27220_H

#include "main.h"
#include <stdint.h>

#define BQ27220_BATTERY_FULL_CAPACITY_MAH   6000.0f
#define BQ27220_INVALID_PERCENT             (-1.0f)
#define BQ27220_I2C_ADDR_7BIT               0x55U

#define BQ27220_OK      1U
#define BQ27220_ERROR   0U

void BQ27220_Init(void);  /* 初始化 BQ27220 使用的 I2C 底层接口。 */
uint8_t BQ27220_IsReady(void);  /* 检测 BQ27220 在 I2C 总线上是否应答。 */
uint8_t BQ27220_ReadVoltage(uint16_t *voltage_mv);  /* 读取 BQ27220 电压寄存器，单位 mV。 */
uint8_t BQ27220_ReadRemainingCapacity(uint16_t *capacity_mah);  /* 读取 BQ27220 剩余容量寄存器，当前未配置化学参数时仅作参考。 */
uint8_t BQ27220_ReadStateOfCharge(uint16_t *soc_percent);  /* 读取 BQ27220 SOC 寄存器，当前未配置化学参数时仅作参考。 */
uint8_t BQ27220_ReadBatteryPercent(float *percent);  /* 读取电压并估算电量百分比，失败时输出无效值。 */
float BQ27220_GetBatteryPercent(void);  /* 简化接口：直接返回电量百分比或无效值。 */
float BQ27220_VoltageToBatteryPercent(uint16_t voltage_mv);  /* 把电池电压转换为 0 到 100 范围内的电量百分比。 */
uint8_t BQ27220_GetLastI2cStatus(void);  /* 返回最近一次 I2C HAL 调用状态，供调试判断通信失败原因。 */
uint32_t BQ27220_GetLastI2cError(void);  /* 返回最近一次 I2C HAL 错误码。 */

#endif
