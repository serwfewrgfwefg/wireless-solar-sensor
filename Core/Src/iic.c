#include "iic.h"
#include "i2c.h"

#define IIC1_HAL_TIMEOUT_MS  100U

static uint8_t iic1_last_hal_status = HAL_OK;
static uint32_t iic1_last_hal_error = HAL_I2C_ERROR_NONE;

/**
 * @brief 保存最近一次 HAL I2C 调用状态和错误码。
 * @param status 函数输入参数。
 */
static void IIC1_SaveHalResult(HAL_StatusTypeDef status)
{
    iic1_last_hal_status = (uint8_t)status;
    iic1_last_hal_error = HAL_I2C_GetError(&hi2c1);
}

/**
 * @brief 初始化 I2C1，用于 BQ27220 通信。
 */
void IIC1_Init(void)
{
    if (hi2c1.Instance != I2C1)
    {
        MX_I2C1_Init();  /* 初始化 I2C1，用于 BQ27220 电量计。 */
    }
}

/**
 * @brief 用默认参数检测指定 7 位地址设备是否在线。
 * @param dev_addr_7bit I2C 7 位设备地址。
 */
uint8_t IIC1_IsDeviceReady(uint8_t dev_addr_7bit)
{
    return IIC1_IsDeviceReadyTimeout(dev_addr_7bit, 3U, IIC1_HAL_TIMEOUT_MS);  /* 用指定重试次数和超时时间检测 I2C 设备是否在线。 */
}

/**
 * @brief 用指定重试次数和超时时间检测 I2C 设备是否在线。
 * @param dev_addr_7bit I2C 7 位设备地址。
 * @param trials I2C 检测重试次数。
 * @param timeout_ms 超时时间，单位 ms。
 */
uint8_t IIC1_IsDeviceReadyTimeout(uint8_t dev_addr_7bit, uint32_t trials, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(&hi2c1,
                                   (uint16_t)(dev_addr_7bit << 1),
                                   trials,
                                   timeout_ms);
    IIC1_SaveHalResult(status);  /* 保存最近一次 HAL I2C 调用状态和错误码。 */

    return (status == HAL_OK) ? IIC_OK : IIC_ERROR;
}

/**
 * @brief 按 BQ27220 小端格式读取 16 位寄存器。
 * @param dev_addr_7bit I2C 7 位设备地址。
 * @param reg 寄存器地址。
 * @param value 配置值字符串或寄存器值。
 */
uint8_t IIC1_ReadRegU16(uint8_t dev_addr_7bit, uint8_t reg, uint16_t *value)
{
    uint8_t buffer[2];
    HAL_StatusTypeDef status;

    if (value == 0)
    {
        return IIC_ERROR;
    }

    status = HAL_I2C_Mem_Read(&hi2c1,
                              (uint16_t)(dev_addr_7bit << 1),
                              (uint16_t)reg,
                              I2C_MEMADD_SIZE_8BIT,
                              buffer,
                              sizeof(buffer),
                              IIC1_HAL_TIMEOUT_MS);
    IIC1_SaveHalResult(status);  /* 保存最近一次 HAL I2C 调用状态和错误码。 */

    if (status != HAL_OK)
    {
        return IIC_ERROR;
    }

    *value = ((uint16_t)buffer[1] << 8) | buffer[0];
    return IIC_OK;
}

/**
 * @brief 获取最近一次 HAL I2C 状态。
 */
uint8_t IIC1_GetLastHalStatus(void)
{
    return iic1_last_hal_status;
}

/**
 * @brief 获取最近一次 HAL I2C 错误码。
 */
uint32_t IIC1_GetLastHalError(void)
{
    return iic1_last_hal_error;
}
