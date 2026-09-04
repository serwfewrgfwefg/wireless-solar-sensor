#include "spi.h"
#include "ICM42688P.h"
#include <math.h>

#define IMU_DELTA_T     0.005f  /* 动态姿态融合采样周期，单位秒。 */
#define IMU_M_PI        3.1415926535f  /* 圆周率常量，用于弧度和角度转换。 */
#define IMU_NEW_WEIGHT  0.35f  /* 加速度低通滤波中新数据权重。 */
#define IMU_OLD_WEIGHT  0.65f  /* 加速度低通滤波中历史数据权重。 */

#define ICM42688_SPI_CMD_LSB          0U  /* 兼容旧 HXY 驱动的 LSB 命令格式标识。 */
#define ICM42688_SPI_CMD_MSB          1U  /* ICM42688-P 正式 SPI 命令格式标识。 */
#define ICM42688_SPI_LSB_READ_CMD(reg)   (uint8_t)((((reg) & 0x7FU) << 1) | 0x01U)
#define ICM42688_SPI_LSB_WRITE_CMD(reg)  (uint8_t)((((reg) & 0x7FU) << 1) & 0xFEU)
#define ICM42688_SPI_MSB_READ_CMD(reg)   (uint8_t)((reg) | 0x80U)
#define ICM42688_SPI_MSB_WRITE_CMD(reg)  (uint8_t)((reg) & 0x7FU)

float angle_x_correction;  /* roll/pitch 修正量之一，保留旧项目角度补偿接口。 */
float angle_y_correction;  /* roll/pitch 修正量之一，保留旧项目角度补偿接口。 */
quater_info_t g_q_info = {1.0f, 0.0f, 0.0f, 0.0f};  /* 动态姿态融合使用的全局四元数状态。 */
float g_param_kp;  /* 动态姿态融合比例增益。 */
float g_param_ki;  /* 动态姿态融合积分增益。 */
uint64_t g_imu_timestamp = 0x00000019B9A18A20ULL;  /* 保留旧协议时间戳变量，当前发送帧时间戳仍置 0。 */

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};  /* 陀螺仪零偏数组，当前默认全 0。 */
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};  /* 加速度零偏数组，静态角度逻辑默认不扣零偏。 */
static uint8_t icm42688_spi_cmd_mode = ICM42688_SPI_CMD_MSB;  /* 当前 ICM42688 SPI 命令格式，初始化时自动识别。 */

/**
 * @brief 根据当前 SPI 命令格式生成读寄存器命令字节。
 * @param reg 寄存器地址。
 */
static uint8_t ICM42688_ReadCmd(uint8_t reg)
{
    if (icm42688_spi_cmd_mode == ICM42688_SPI_CMD_MSB)
    {
        return ICM42688_SPI_MSB_READ_CMD(reg);
    }

    return ICM42688_SPI_LSB_READ_CMD(reg);
}

/**
 * @brief 根据当前 SPI 命令格式生成写寄存器命令字节。
 * @param reg 寄存器地址。
 */
static uint8_t ICM42688_WriteCmd(uint8_t reg)
{
    if (icm42688_spi_cmd_mode == ICM42688_SPI_CMD_MSB)
    {
        return ICM42688_SPI_MSB_WRITE_CMD(reg);
    }

    return ICM42688_SPI_LSB_WRITE_CMD(reg);
}

/**
 * @brief 用指定命令格式读取单个寄存器，主要用于初始化时探测命令格式。
 * @param reg 寄存器地址。
 * @param cmd 命令字符串。
 */
static uint8_t ICM42688_ReadRegWithCmd(uint8_t reg, uint8_t cmd)
{
    uint8_t value;

    CS_Low();  /* 拉低 ICM42688 片选，开始一次 SPI 访问。 */
    SPI_TransmitReceive(cmd);  /* SPI1 同步收发 1 字节，给 ICM42688 驱动使用。 */
    value = SPI_TransmitReceive(0xFF);  /* SPI1 同步收发 1 字节，给 ICM42688 驱动使用。 */
    CS_High();  /* 拉高 ICM42688 片选，结束一次 SPI 访问。 */

    return value;
}

/**
 * @brief 切换 ICM42688 的寄存器 Bank。
 * @param bank ICM42688 寄存器 Bank 号。
 */
static void ICM42688_SelectBank(uint8_t bank)
{
    ICM42688_WriteReg(ICM42688_REG_REG_BANK_SEL, bank & 0x07U);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
}

/**
 * @brief 配置 ICM42688 的 SPI 接口相关寄存器，禁用未使用接口/功能。
 */
static void ICM42688_ConfigureSpiInterface(void)
{
    uint8_t intf_config4;

    ICM42688_SelectBank(ICM42688_BANK0);  /* 切换 ICM42688 的寄存器 Bank。 */
    ICM42688_WriteReg(ICM42688_REG_DRIVE_CONFIG, ICM42688_DRIVE_CONFIG_SPI);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    ICM42688_WriteReg(ICM42688_REG_INTF_CONFIG0, ICM42688_INTF_CONFIG0_SPI_ONLY);  /* 通过 SPI 写 ICM42688 单个寄存器。 */

    ICM42688_SelectBank(ICM42688_BANK1);  /* 切换 ICM42688 的寄存器 Bank。 */
    intf_config4 = ICM42688_ReadReg(ICM42688_REG_INTF_CONFIG4);  /* 通过 SPI 读 ICM42688 单个寄存器。 */
    intf_config4 &= (uint8_t)~0x40U;
    intf_config4 |= 0x02U;
    ICM42688_WriteReg(ICM42688_REG_INTF_CONFIG4, intf_config4);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    ICM42688_WriteReg(ICM42688_REG_INTF_CONFIG5, ICM42688_INTF_CONFIG5_FSYNC);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    ICM42688_WriteReg(ICM42688_REG_INTF_CONFIG6, ICM42688_INTF_CONFIG6_SPI);  /* 通过 SPI 写 ICM42688 单个寄存器。 */

    ICM42688_SelectBank(ICM42688_BANK0);  /* 切换 ICM42688 的寄存器 Bank。 */
}

/**
 * @brief 通过 SPI 写 ICM42688 单个寄存器。
 * @param reg 寄存器地址。
 * @param value 配置值字符串或寄存器值。
 */
void ICM42688_WriteReg(uint8_t reg, uint8_t value)
{
    CS_Low();  /* 拉低 ICM42688 片选，开始一次 SPI 访问。 */
    SPI_TransmitReceive(ICM42688_WriteCmd(reg));  /* SPI1 同步收发 1 字节，给 ICM42688 驱动使用。 */
    SPI_TransmitReceive(value);  /* SPI1 同步收发 1 字节，给 ICM42688 驱动使用。 */
    CS_High();  /* 拉高 ICM42688 片选，结束一次 SPI 访问。 */
    HAL_Delay(1);
}

/**
 * @brief 通过 SPI 读 ICM42688 单个寄存器。
 * @param reg 寄存器地址。
 */
uint8_t ICM42688_ReadReg(uint8_t reg)
{
    uint8_t value;

    CS_Low();  /* 拉低 ICM42688 片选，开始一次 SPI 访问。 */
    SPI_TransmitReceive(ICM42688_ReadCmd(reg));  /* SPI1 同步收发 1 字节，给 ICM42688 驱动使用。 */
    value = SPI_TransmitReceive(0xFF);  /* SPI1 同步收发 1 字节，给 ICM42688 驱动使用。 */
    CS_High();  /* 拉高 ICM42688 片选，结束一次 SPI 访问。 */

    return value;
}

/**
 * @brief 通过 SPI 从指定寄存器开始连续读取多个字节。
 * @param reg 寄存器地址。
 * @param buffer 读写数据缓冲区。
 * @param len 数据长度。
 */
void ICM42688_ReadBurst(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    uint8_t i;

    CS_Low();  /* 拉低 ICM42688 片选，开始一次 SPI 访问。 */
    SPI_TransmitReceive(ICM42688_ReadCmd(reg));  /* SPI1 同步收发 1 字节，给 ICM42688 驱动使用。 */
    for (i = 0; i < len; i++)
    {
        buffer[i] = SPI_TransmitReceive(0xFF);
    }
    CS_High();  /* 拉高 ICM42688 片选，结束一次 SPI 访问。 */
}

/**
 * @brief 读取 WHO_AM_I，用于确认 ICM42688-P 通信正常。
 */
uint8_t ICM42688_ReadWhoAmI(void)
{
    return ICM42688_ReadReg(ICM42688_REG_WHO_AM_I);  /* 通过 SPI 读 ICM42688 单个寄存器。 */
}

/**
 * @brief 用 LSB 命令格式读取 WHO_AM_I，用于兼容早期 HXY 驱动差异。
 */
uint8_t ICM42688_ReadWhoAmI_LsbCmd(void)
{
    return ICM42688_ReadRegWithCmd(ICM42688_REG_WHO_AM_I,
                                  ICM42688_SPI_LSB_READ_CMD(ICM42688_REG_WHO_AM_I));
}

/**
 * @brief 用 MSB 命令格式读取 WHO_AM_I，用于 ICM42688-P 正式 SPI 格式。
 */
uint8_t ICM42688_ReadWhoAmI_MsbCmd(void)
{
    return ICM42688_ReadRegWithCmd(ICM42688_REG_WHO_AM_I,
                                  ICM42688_SPI_MSB_READ_CMD(ICM42688_REG_WHO_AM_I));
}

/**
 * @brief 返回当前识别出的 SPI 命令格式。
 */
uint8_t ICM42688_GetSpiCmdMode(void)
{
    return icm42688_spi_cmd_mode;  /* 当前 ICM42688 SPI 命令格式，初始化时自动识别。 */
}

/**
 * @brief 初始化 ICM42688-P：识别通信格式、软复位、配置接口和量程，最后关闭采样。
 */
void ICM42688_Init(void)
{
    uint8_t who_lsb;
    uint8_t who_msb;

    icm42688_spi_cmd_mode = ICM42688_SPI_CMD_MSB;
    CS_High();  /* 拉高 ICM42688 片选，结束一次 SPI 访问。 */
    HAL_Delay(10);

    ICM42688_SelectBank(ICM42688_BANK0);  /* 切换 ICM42688 的寄存器 Bank。 */
    who_lsb = ICM42688_ReadWhoAmI_LsbCmd();  /* 用 LSB 命令格式读取 WHO_AM_I，用于兼容早期 HXY 驱动差异。 */
    who_msb = ICM42688_ReadWhoAmI_MsbCmd();  /* 用 MSB 命令格式读取 WHO_AM_I，用于 ICM42688-P 正式 SPI 格式。 */
    if ((who_lsb == ICM42688_WHO_AM_I_VALUE) && (who_msb != ICM42688_WHO_AM_I_VALUE))
    {
        icm42688_spi_cmd_mode = ICM42688_SPI_CMD_LSB;
    }
    else
    {
        icm42688_spi_cmd_mode = ICM42688_SPI_CMD_MSB;
    }

    ICM42688_WriteReg(ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET_CMD);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    HAL_Delay(10);

    ICM42688_SelectBank(ICM42688_BANK0);  /* 切换 ICM42688 的寄存器 Bank。 */
    (void)ICM42688_ReadWhoAmI();  /* 读取 WHO_AM_I，用于确认 ICM42688-P 通信正常。 */
    ICM42688_ConfigureSpiInterface();  /* 配置 ICM42688 的 SPI 接口相关寄存器，禁用未使用接口/功能。 */

    ICM42688_WriteReg(ICM42688_REG_FIFO_CONFIG, ICM42688_FIFO_BYPASS);  /* 通过 SPI 写 ICM42688 单个寄存器。 */

    ICM42688_WriteReg(ICM42688_REG_ACC_CONF, ICM42688_ACC_CONF_2G_100HZ);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    ICM42688_WriteReg(ICM42688_REG_GYR_CONF, ICM42688_GYR_CONF_125DPS_100HZ);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    ICM42688_ACC_Suspend();  /* 关闭 ICM42688 加速度计和陀螺仪，使 IMU 进入待机低功耗状态。 */
}

/**
 * @brief 关闭 ICM42688 加速度计和陀螺仪，使 IMU 进入待机低功耗状态。
 */
void ICM42688_ACC_Suspend(void)
{
    ICM42688_SelectBank(ICM42688_BANK0);  /* 切换 ICM42688 的寄存器 Bank。 */
    ICM42688_WriteReg(ICM42688_REG_PWR_CTRL, ICM42688_PWR_OFF);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    HAL_Delay(5);
}

/**
 * @brief 发送数据前打开加速度计低功耗模式，陀螺仪保持关闭。
 */
void ICM42688_ACC_Wakeup(void)
{
    ICM42688_SelectBank(ICM42688_BANK0);  /* 切换 ICM42688 的寄存器 Bank。 */
    ICM42688_WriteReg(ICM42688_REG_PWR_CTRL, ICM42688_PWR_TEMP_ACC_ON);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    HAL_Delay(20);
}

/**
 * @brief 打开陀螺仪和加速度计，主要保留给动态姿态融合或调试使用。
 */
void ICM42688_GYR_Enable(void)
{
    ICM42688_SelectBank(ICM42688_BANK0);  /* 切换 ICM42688 的寄存器 Bank。 */
    ICM42688_WriteReg(ICM42688_REG_PWR_CTRL, ICM42688_PWR_ALL_ON);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    HAL_Delay(250);
}

/**
 * @brief 关闭陀螺仪，仅保留温度/加速度计工作状态。
 */
void ICM42688_GYR_Disable(void)
{
    ICM42688_SelectBank(ICM42688_BANK0);  /* 切换 ICM42688 的寄存器 Bank。 */
    ICM42688_WriteReg(ICM42688_REG_PWR_CTRL, ICM42688_PWR_TEMP_ACC_ON);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
    HAL_Delay(2);
}

/**
 * @brief 读取 ICM42688 三轴加速度、三轴角速度和温度，并转换成物理量。
 * @param data 待发送或待处理数据。
 */
void ICM42688_ReadData(IMU_Data_t *data)
{
    uint8_t buffer[12];
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;

    ICM42688_ReadBurst(ICM42688_REG_ACC_XH, buffer, sizeof(buffer));  /* 通过 SPI 从指定寄存器开始连续读取多个字节。 */

    raw_accel_x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    raw_accel_y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    raw_accel_z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
    raw_gyro_x  = (int16_t)(((uint16_t)buffer[6] << 8) | buffer[7]);
    raw_gyro_y  = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    raw_gyro_z  = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);

    data->accel_x = ((float)raw_accel_x) / ACCEL_SENSITIVITY - accel_bias[0];
    data->accel_y = ((float)raw_accel_y) / ACCEL_SENSITIVITY - accel_bias[1];
    data->accel_z = ((float)raw_accel_z) / ACCEL_SENSITIVITY - accel_bias[2];

    data->gyro_x = ((float)raw_gyro_x) / GYRO_SENSITIVITY - gyro_bias[0];
    data->gyro_y = ((float)raw_gyro_y) / GYRO_SENSITIVITY - gyro_bias[1];
    data->gyro_z = ((float)raw_gyro_z) / GYRO_SENSITIVITY - gyro_bias[2];
    data->temperature = ICM42688_ReadTemperatureC();
}

/**
 * @brief 读取 ICM42688 内置温度传感器并转换为摄氏度。
 */
float ICM42688_ReadTemperatureC(void)
{
    uint8_t buffer[2];
    int16_t raw_temp;

    ICM42688_ReadBurst(ICM42688_REG_TEMP_H, buffer, sizeof(buffer));  /* 通过 SPI 从指定寄存器开始连续读取多个字节。 */
    raw_temp = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);

    return ((float)raw_temp) / 132.48f + 25.0f;
}

/**
 * @brief 快速计算 1/sqrt(x)，供姿态融合归一化使用。
 * @param x 退出码。
 */
static float imu_inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 对加速度数据做一阶低通滤波，降低抖动。
 * @param ax X 轴加速度。
 * @param ay Y 轴加速度。
 * @param az Z 轴加速度。
 */
static void imu_data_transform(float *ax, float *ay, float *az)
{
    static double lastax = 0.0;
    static double lastay = 0.0;
    static double lastaz = 0.0;

    *ax = *ax * IMU_NEW_WEIGHT + (float)lastax * IMU_OLD_WEIGHT;
    *ay = *ay * IMU_NEW_WEIGHT + (float)lastay * IMU_OLD_WEIGHT;
    *az = *az * IMU_NEW_WEIGHT + (float)lastaz * IMU_OLD_WEIGHT;

    lastax = *ax;
    lastay = *ay;
    lastaz = *az;
}

/**
 * @brief 无磁力计姿态融合核心，根据陀螺仪和加速度修正四元数。
 * @param gx X 轴角速度。
 * @param gy Y 轴角速度。
 * @param gz Z 轴角速度。
 * @param ax X 轴加速度。
 * @param ay Y 轴加速度。
 * @param az Z 轴加速度。
 * @param kp 姿态融合比例增益。
 * @param ki 姿态融合积分增益。
 */
static void imu_ahrsupdate_nomagnetic(float gx, float gy, float gz, float ax, float ay, float az, float kp, float ki)
{
    static float i_ex;
    static float i_ey;
    static float i_ez;
    float half_t = 0.25f * IMU_DELTA_T;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float q0 = g_q_info.q0;
    float q1 = g_q_info.q1;
    float q2 = g_q_info.q2;
    float q3 = g_q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float norm;
    float delta_2;

    norm = ax * ax + ay * ay + az * az;
    if (norm <= 0.0f)
    {
        return;
    }

    norm = imu_inv_sqrt(norm);  /* 快速计算 1/sqrt(x)，供姿态融合归一化使用。 */
    ax *= norm;
    ay *= norm;
    az *= norm;

    vx = 2.0f * (q1 * q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    i_ex += IMU_DELTA_T * ex;
    i_ey += IMU_DELTA_T * ey;
    i_ez += IMU_DELTA_T * ez;

    gx = gx + kp * ex + ki * i_ex;
    gy = gy + kp * ey + ki * i_ey;
    gz = gz + kp * ez + ki * i_ez;

    delta_2 = (2.0f * half_t * gx) * (2.0f * half_t * gx)
            + (2.0f * half_t * gy) * (2.0f * half_t * gy)
            + (2.0f * half_t * gz) * (2.0f * half_t * gz);

    q0 = (1.0f - delta_2 / 8.0f) * q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = (1.0f - delta_2 / 8.0f) * q1 + ( q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = (1.0f - delta_2 / 8.0f) * q2 + ( q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = (1.0f - delta_2 / 8.0f) * q3 + ( q0 * gz + q1 * gy - q2 * gx) * half_t;

    norm = imu_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);  /* 快速计算 1/sqrt(x)，供姿态融合归一化使用。 */
    g_q_info.q0 = q0 * norm;
    g_q_info.q1 = q1 * norm;
    g_q_info.q2 = q2 * norm;
    g_q_info.q3 = q3 * norm;
}

/**
 * @brief 执行动态姿态融合并输出欧拉角。
 * @param gx X 轴角速度。
 * @param gy Y 轴角速度。
 * @param gz Z 轴角速度。
 * @param ax X 轴加速度。
 * @param ay Y 轴加速度。
 * @param az Z 轴加速度。
 * @param kp 姿态融合比例增益。
 * @param ki 姿态融合积分增益。
 */
eulerian_angles_t imu_get_eulerian_angles(float gx, float gy, float gz, float ax, float ay, float az, float kp, float ki)
{
    eulerian_angles_t eulerangle;
    float q0;  /* 四元数 q0。 */
    float q1;  /* 四元数 q1。 */
    float q2;  /* 四元数 q2。 */
    float q3;  /* 四元数 q3。 */

    imu_data_transform(&ax, &ay, &az);  /* 对加速度数据做一阶低通滤波，降低抖动。 */
    imu_ahrsupdate_nomagnetic(gx, gy, gz, ax, ay, az, kp, ki);  /* 无磁力计姿态融合核心，根据陀螺仪和加速度修正四元数。 */

    q0 = g_q_info.q0;
    q1 = g_q_info.q1;
    q2 = g_q_info.q2;
    q3 = g_q_info.q3;

    eulerangle.pitch = -asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 180.0f / IMU_M_PI;
    eulerangle.roll = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 180.0f / IMU_M_PI;
    eulerangle.yaw = -atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / IMU_M_PI;

    if (eulerangle.roll > 90.0f || eulerangle.roll < -90.0f)
    {
        if (eulerangle.pitch > 0.0f)
        {
            eulerangle.pitch = 180.0f - eulerangle.pitch;
        }
        if (eulerangle.pitch < 0.0f)
        {
            eulerangle.pitch = -(180.0f + eulerangle.pitch);
        }
    }

    if (eulerangle.yaw > 180.0f)
    {
        eulerangle.yaw -= 360.0f;
    }
    else if (eulerangle.yaw < -180.0f)
    {
        eulerangle.yaw += 360.0f;
    }

    return eulerangle;
}

/**
 * @brief 只根据重力加速度分量计算静态 roll/pitch。
 * @param ax X 轴加速度。
 * @param ay Y 轴加速度。
 * @param az Z 轴加速度。
 */
eulerian_angles_t ICM42688_GetStaticAngles(float ax, float ay, float az)
{
    eulerian_angles_t eulerangle = {0.0f, 0.0f, 0.0f};
    float horizontal_g = sqrtf(ay * ay + az * az);
    float norm_sq = ax * ax + ay * ay + az * az;

    if (norm_sq < 1e-6f)
    {
        return eulerangle;
    }

    eulerangle.pitch = atan2f(-ax, horizontal_g) * 180.0f / IMU_M_PI;
    eulerangle.roll = atan2f(ay, az) * 180.0f / IMU_M_PI;
    eulerangle.yaw = 0.0f;

    return eulerangle;
}

/**
 * @brief 初始化四元数姿态状态。
 */
void imu_init(void)
{
    g_q_info.q0 = 1.0f;
    g_q_info.q1 = 0.0f;
    g_q_info.q2 = 0.0f;
    g_q_info.q3 = 0.0f;
}

/**
 * @brief 反转 float 字节序，匹配原 V3.2 上传协议。
 * @param value 配置值字符串或寄存器值。
 */
float reverse_float_endian(float value)
{
    uint8_t *src = (uint8_t *)&value;
    float result;
    uint8_t *dst = (uint8_t *)&result;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];

    return result;
}

/**
 * @brief 把 IMU、角度、温度、电量和设备 ID 打包为原 RS485/MQTT 二进制协议帧。
 * @param data 待发送或待处理数据。
 * @param angles 静态角度结构体指针。
 * @param temperature 温度值，单位摄氏度。
 * @param battery_percent 电量百分比。
 * @param device_id 设备 ID。
 * @param rs485buf 输出协议帧缓冲区。
 */
void ICM42688_Pack(IMU_Data_t *data, eulerian_angles_t *angles, float temperature, float battery_percent, uint16_t device_id, rs485_data_t *rs485buf)
{
    static uint16_t frame_id = 1;
    uint16_t current_frame_id = frame_id++;

    (void)temperature;

    rs485buf->header = 0x68;
    rs485buf->data_length = 0x3C00;
    rs485buf->frame_id = BSWAP16(current_frame_id);
    if (frame_id == 0U)
    {
        frame_id = 1;
    }
    rs485buf->encrypt = 0x00;
    rs485buf->function_code = 0x02;
    rs485buf->device_id = BSWAP16(device_id);
    rs485buf->timestamp = 0;
    rs485buf->channel_count = 0x0700;

    rs485buf->channel_id1 = 0x0100;
    rs485buf->accel_x = reverse_float_endian(data->accel_x);
    rs485buf->channel_id2 = 0x0200;
    rs485buf->accel_y = reverse_float_endian(data->accel_y);
    rs485buf->channel_id3 = 0x0300;
    rs485buf->accel_z = reverse_float_endian(data->accel_z);
    rs485buf->channel_id4 = 0x0400;
    rs485buf->roll = reverse_float_endian(angles->pitch);
    rs485buf->channel_id5 = 0x0500;
    rs485buf->pitch = reverse_float_endian(angles->roll);
    rs485buf->channel_id6 = 0x0600;
    rs485buf->temperature = reverse_float_endian(data->temperature);
    rs485buf->channel_id7 = 0x0700;
    rs485buf->battery_percent = reverse_float_endian(battery_percent);
    rs485buf->check_digit = 0xFFFF;
}

/**
 * @brief 兼容旧代码的打包封装，使用默认设备 ID 和默认电量。
 * @param data 待发送或待处理数据。
 * @param rs485buf 输出协议帧缓冲区。
 */
void ICM42688Pack(IMU_Data_t *data, rs485_data_t *rs485buf)
{
    eulerian_angles_t angles = ICM42688_GetStaticAngles(data->accel_x, data->accel_y, data->accel_z);  /* 只根据重力加速度分量计算静态 roll/pitch。 */
    ICM42688_Pack(data, &angles, data->temperature, 0.0f, 3000U, rs485buf);  /* 把 IMU、角度、温度、电量和设备 ID 打包为原 RS485/MQTT 二进制协议帧。 */
}

/**
 * @brief 给陀螺仪角速度增加补偿量，保留旧动态姿态接口。
 * @param data 待发送或待处理数据。
 * @param angle_x X 方向补偿值。
 * @param angle_y Y 方向补偿值。
 */
void ICM42688_angle_correction(IMU_Data_t *data, float angle_x, float angle_y)
{
    data->gyro_x = data->gyro_x + angle_x;
    data->gyro_y = data->gyro_y + angle_y;
}

