#include "bq27220.h"
#include "iic.h"

#define BQ27220_CMD_VOLTAGE          0x08U  /* BQ27220 电压寄存器命令地址。 */
#define BQ27220_CMD_REMAIN_CAP       0x10U  /* BQ27220 剩余容量寄存器命令地址。 */
#define BQ27220_CMD_STATE_OF_CHARGE  0x1CU  /* BQ27220 SOC 寄存器命令地址。 */

/**
 * @brief 把浮点电量百分比四舍五入到两位小数。
 * @param value 配置值字符串或寄存器值。
 */
static float BQ27220_Round2(float value)
{
    if (value >= 0.0f)
    {
        return (float)((int32_t)(value * 100.0f + 0.5f)) / 100.0f;
    }
    return (float)((int32_t)(value * 100.0f - 0.5f)) / 100.0f;
}

/* 磷酸铁锂(LiFePO4)单节 电压->电量 估算表 (近似, 常温, 由高到低).
 * 说明: 本芯片未用 bqStudio 配置化学 ID / 设计容量, 因此
 * RemainingCapacity(0x10) 与 StateOfCharge(0x1C) 都不可信;
 * 这里只用可信的 Voltage(0x08) 按放电曲线查表估算电量, 仅作粗略指示.
 * 磷酸铁锂放电曲线极平(3.2~3.3V 覆盖很大电量区间), 中段误差较大. */
typedef struct
{
    uint16_t mv;       /* 电池电压 (mV) */
    float    percent;  /* 对应电量 (%) */
} bq_ocv_point_t;

static const bq_ocv_point_t BQ27220_OCV_TABLE[] =
{
    { 3650U, 100.0f },
    { 3400U,  95.0f },
    { 3350U,  90.0f },
    { 3300U,  80.0f },
    { 3280U,  65.0f },
    { 3260U,  50.0f },
    { 3240U,  35.0f },
    { 3220U,  25.0f },
    { 3200U,  20.0f },
    { 3150U,  10.0f },
    { 3000U,   5.0f },
    { 2800U,   0.0f },
};

/**
 * @brief 按磷酸铁锂开路电压表估算电量百分比。
 * @param mv 电池电压，单位 mV。
 */
static float BQ27220_VoltageToPercent(uint16_t mv)
{
    const uint32_t n = sizeof(BQ27220_OCV_TABLE) / sizeof(BQ27220_OCV_TABLE[0]);
    uint32_t i;

    if (mv >= BQ27220_OCV_TABLE[0].mv)     return BQ27220_OCV_TABLE[0].percent;
    if (mv <= BQ27220_OCV_TABLE[n - 1].mv) return BQ27220_OCV_TABLE[n - 1].percent;

    for (i = 0; i < n - 1; i++)
    {
        uint16_t v_hi = BQ27220_OCV_TABLE[i].mv;
        uint16_t v_lo = BQ27220_OCV_TABLE[i + 1].mv;

        if (mv <= v_hi && mv >= v_lo)
        {
            float p_hi = BQ27220_OCV_TABLE[i].percent;
            float p_lo = BQ27220_OCV_TABLE[i + 1].percent;
            /* 在相邻两点间线性插值 */
            return p_lo + (p_hi - p_lo) * ((float)(mv - v_lo) / (float)(v_hi - v_lo));
        }
    }
    return BQ27220_OCV_TABLE[n - 1].percent;
}

/**
 * @brief 把电池电压转换为 0 到 100 范围内的电量百分比。
 * @param voltage_mv 电池电压，单位 mV。
 */
float BQ27220_VoltageToBatteryPercent(uint16_t voltage_mv)
{
    float value;

    value = BQ27220_VoltageToPercent(voltage_mv);  /* 按磷酸铁锂开路电压表估算电量百分比。 */
    if (value < 0.0f) value = 0.0f;
    if (value > 100.0f) value = 100.0f;

    return BQ27220_Round2(value);  /* 把浮点电量百分比四舍五入到两位小数。 */
}

/**
 * @brief 初始化 BQ27220 使用的 I2C 底层接口。
 */
void BQ27220_Init(void)
{
    IIC1_Init();  /* 初始化 I2C1，用于 BQ27220 通信。 */
}

/**
 * @brief 检测 BQ27220 在 I2C 总线上是否应答。
 */
uint8_t BQ27220_IsReady(void)
{
    return IIC1_IsDeviceReady(BQ27220_I2C_ADDR_7BIT);  /* 用默认参数检测指定 7 位地址设备是否在线。 */
}

/**
 * @brief 读取 BQ27220 电压寄存器，单位 mV。
 * @param voltage_mv 电池电压，单位 mV。
 */
uint8_t BQ27220_ReadVoltage(uint16_t *voltage_mv)
{
    return IIC1_ReadRegU16(BQ27220_I2C_ADDR_7BIT, BQ27220_CMD_VOLTAGE, voltage_mv);  /* 按 BQ27220 小端格式读取 16 位寄存器。 */
}

/**
 * @brief 读取 BQ27220 剩余容量寄存器，当前未配置化学参数时仅作参考。
 * @param capacity_mah 剩余容量，单位 mAh。
 */
uint8_t BQ27220_ReadRemainingCapacity(uint16_t *capacity_mah)
{
    return IIC1_ReadRegU16(BQ27220_I2C_ADDR_7BIT, BQ27220_CMD_REMAIN_CAP, capacity_mah);  /* 按 BQ27220 小端格式读取 16 位寄存器。 */
}

/**
 * @brief 读取 BQ27220 SOC 寄存器，当前未配置化学参数时仅作参考。
 * @param soc_percent SOC 百分比。
 */
uint8_t BQ27220_ReadStateOfCharge(uint16_t *soc_percent)
{
    return IIC1_ReadRegU16(BQ27220_I2C_ADDR_7BIT, BQ27220_CMD_STATE_OF_CHARGE, soc_percent);  /* 按 BQ27220 小端格式读取 16 位寄存器。 */
}

/**
 * @brief 读取电压并估算电量百分比，失败时输出无效值。
 * @param percent 输出参数，电量百分比。
 */
uint8_t BQ27220_ReadBatteryPercent(float *percent)
{
    uint16_t voltage_mv;
    float value;

    if (percent == 0) return BQ27220_ERROR;

    /* 芯片未经 bqStudio 配置, 只用可信的 Voltage 按放电曲线查表估算电量 */
    if (BQ27220_ReadVoltage(&voltage_mv) != BQ27220_OK)
    {
        *percent = BQ27220_INVALID_PERCENT;
        return BQ27220_ERROR;
    }

    value = BQ27220_VoltageToBatteryPercent(voltage_mv);  /* 把电池电压转换为 0 到 100 范围内的电量百分比。 */
    *percent = value;
    return BQ27220_OK;
}

/**
 * @brief 简化接口：直接返回电量百分比或无效值。
 */
float BQ27220_GetBatteryPercent(void)
{
    float percent;

    if (BQ27220_ReadBatteryPercent(&percent) != BQ27220_OK)
    {
        return BQ27220_INVALID_PERCENT;
    }

    return percent;
}

/**
 * @brief 返回最近一次 I2C HAL 调用状态，供调试判断通信失败原因。
 */
uint8_t BQ27220_GetLastI2cStatus(void)
{
    return IIC1_GetLastHalStatus();  /* 获取最近一次 HAL I2C 状态。 */
}

/**
 * @brief 返回最近一次 I2C HAL 错误码。
 */
uint32_t BQ27220_GetLastI2cError(void)
{
    return IIC1_GetLastHalError();  /* 获取最近一次 HAL I2C 错误码。 */
}
