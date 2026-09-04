#ifndef __ICM42688P_H
#define __ICM42688P_H

#include "main.h"

#define ICM42688_REG_DEVICE_CONFIG      0x11  /* ICM42688 设备配置寄存器地址。 */
#define ICM42688_REG_DRIVE_CONFIG       0x13  /* ICM42688 SPI/I2C 驱动能力配置寄存器地址。 */
#define ICM42688_REG_FIFO_CONFIG        0x16  /* ICM42688 FIFO 配置寄存器地址。 */
#define ICM42688_REG_TEMP_H             0x1D  /* ICM42688 温度高字节寄存器地址。 */
#define ICM42688_REG_ACC_XH             0x1F  /* ICM42688 加速度数据起始寄存器地址。 */
#define ICM42688_REG_GYR_XH             0x25  /* ICM42688 陀螺仪数据起始寄存器地址。 */
#define ICM42688_REG_INTF_CONFIG0       0x4C  /* ICM42688 接口配置 0 寄存器地址。 */
#define ICM42688_REG_INTF_CONFIG1       0x4D  /* ICM42688 接口配置 1 寄存器地址。 */
#define ICM42688_REG_PWR_CTRL           0x4E  /* ICM42688 电源管理寄存器地址。 */
#define ICM42688_REG_GYR_CONF           0x4F  /* ICM42688 陀螺仪量程和 ODR 配置寄存器地址。 */
#define ICM42688_REG_ACC_CONF           0x50  /* ICM42688 加速度计量程和 ODR 配置寄存器地址。 */
#define ICM42688_REG_REG_BANK_SEL       0x76  /* ICM42688 寄存器 Bank 选择寄存器地址。 */
#define ICM42688_REG_INTF_CONFIG4       0x7A  /* ICM42688 接口配置 4 寄存器地址。 */
#define ICM42688_REG_INTF_CONFIG5       0x7B  /* ICM42688 接口配置 5 寄存器地址。 */
#define ICM42688_REG_INTF_CONFIG6       0x7C  /* ICM42688 接口配置 6 寄存器地址。 */
#define ICM42688_REG_WHO_AM_I           0x75  /* ICM42688 芯片 ID 寄存器地址。 */

#define ICM42688_WHO_AM_I_VALUE         0x47  /* ICM42688-P 正常 WHO_AM_I 返回值。 */
#define ICM42688_SOFT_RESET_CMD         0x01  /* ICM42688 软件复位命令。 */
#define ICM42688_BANK0                  0x00  /* ICM42688 用户寄存器 Bank0。 */
#define ICM42688_BANK1                  0x01  /* ICM42688 用户寄存器 Bank1。 */

#define ICM42688_PWR_OFF                0x20  /* 关闭加速度计和陀螺仪后的待机电源配置值。 */
#define ICM42688_PWR_ACC_LP             0x02  /* 只打开加速度计低功耗模式的电源配置值。 */
#define ICM42688_PWR_ACC_LN             0x03  /* 打开加速度计低噪声模式的电源配置值。 */
#define ICM42688_PWR_GYR_LN             0x0C  /* 打开陀螺仪低噪声模式的电源配置位。 */
#define ICM42688_PWR_ALL_ON             (ICM42688_PWR_ACC_LN | ICM42688_PWR_GYR_LN)  /* 加速度计和陀螺仪全开配置值。 */
#define ICM42688_PWR_ACC_ON             ICM42688_PWR_ACC_LP  /* 当前业务使用的加速度计工作模式。 */
#define ICM42688_PWR_TEMP_ACC_ON        ICM42688_PWR_ACC_ON  /* 读取温度和加速度时使用的工作模式。 */

#define ICM42688_DRIVE_CONFIG_SPI       0x05  /* SPI 引脚驱动能力配置值。 */
#define ICM42688_INTF_CONFIG0_SPI_ONLY  0x33  /* 禁用 I2C 并配置 FIFO 字节序等接口选项。 */
#define ICM42688_INTF_CONFIG5_FSYNC     0x01  /* 关闭/固定未使用的 FSYNC 相关输入配置。 */
#define ICM42688_INTF_CONFIG6_SPI       0x13  /* 选择 ICM42688 SPI 接口相关模式。 */
#define ICM42688_FIFO_BYPASS            0x00  /* FIFO 旁路模式，本工程直接读寄存器不用 FIFO。 */
#define ICM42688_ACC_CONF_2G_100HZ      0x68  /* 加速度计配置为 +/-2g、100Hz ODR。 */
#define ICM42688_GYR_CONF_125DPS_100HZ  0x88  /* 陀螺仪配置为 +/-125dps、100Hz ODR，待机时仍关闭。 */

#define ACCEL_SENSITIVITY               16384.0f  /* +/-2g 量程下加速度原始值到 g 的比例。 */
#define GYRO_SENSITIVITY                262.0f  /* +/-125dps 量程下陀螺仪原始值到 dps 的比例。 */
#define IMU_TIMESTAMP_STEP_MS           60000ULL  /* 保留旧协议时间戳步进值。 */

#define BSWAP16(x) ((uint16_t)(((x) >> 8) | ((x) << 8)))
#define BSWAP64(x) ((uint64_t)( \
    ((x) >> 56)                 | \
    (((x) >> 40) & 0xFF00ULL)   | \
    (((x) >> 24) & 0xFF0000ULL) | \
    (((x) >> 8)  & 0xFF000000ULL) | \
    (((x) << 8)  & 0xFF00000000ULL) | \
    (((x) << 24) & 0xFF0000000000ULL) | \
    (((x) << 40) & 0xFF000000000000ULL) | \
    ((x) << 56) ))

#pragma pack(push, 1)

typedef struct
{
    float accel_x;  /* X 轴加速度，单位 g。 */
    float accel_y;  /* Y 轴加速度，单位 g。 */
    float accel_z;  /* Z 轴加速度，单位 g。 */
    float gyro_x;  /* X 轴角速度，单位 dps。 */
    float gyro_y;  /* Y 轴角速度，单位 dps。 */
    float gyro_z;  /* Z 轴角速度，单位 dps。 */
    float temperature;  /* IMU 内部温度，单位摄氏度。 */
} IMU_Data_t;

typedef struct
{
    uint8_t header;  /* 协议帧头。 */
    uint16_t data_length;  /* 协议数据长度。 */
    uint16_t frame_id;  /* 协议帧序号。 */
    uint8_t encrypt;  /* 加密标志，沿用旧协议。 */
    uint8_t function_code;  /* 功能码，沿用旧协议。 */
    uint16_t device_id;  /* 上传协议帧中的设备 ID。 */
    uint64_t timestamp;  /* 时间戳字段，当前沿用旧协议置 0。 */
    uint16_t channel_count;  /* 通道数量。 */
    uint16_t channel_id1;  /* 通道 1 ID。 */
    float accel_x;  /* X 轴加速度，单位 g。 */
    uint16_t channel_id2;  /* 通道 2 ID。 */
    float accel_y;  /* Y 轴加速度，单位 g。 */
    uint16_t channel_id3;  /* 通道 3 ID。 */
    float accel_z;  /* Z 轴加速度，单位 g。 */
    uint16_t channel_id4;  /* 通道 4 ID。 */
    float roll;  /* 静态横滚角，单位度。 */
    uint16_t channel_id5;  /* 通道 5 ID。 */
    float pitch;  /* 静态俯仰角，单位度。 */
    uint16_t channel_id6;  /* 通道 6 ID。 */
    float temperature;  /* IMU 内部温度，单位摄氏度。 */
    uint16_t channel_id7;  /* 通道 7 ID。 */
    float battery_percent;  /* 电量百分比。 */
    uint16_t check_digit;  /* 协议校验字。 */
} rs485_data_t;

typedef struct
{
    float q0;  /* 四元数 q0。 */
    float q1;  /* 四元数 q1。 */
    float q2;  /* 四元数 q2。 */
    float q3;  /* 四元数 q3。 */
} quater_info_t;

typedef struct
{
    float pitch;  /* 静态俯仰角，单位度。 */
    float roll;  /* 静态横滚角，单位度。 */
    float yaw;  /* 航向角，单位度。 */
} eulerian_angles_t;

extern float g_param_kp;  /* 动态姿态融合比例增益。 */
extern float g_param_ki;  /* 动态姿态融合积分增益。 */
extern float angle_x_correction;  /* roll/pitch 修正量之一，保留旧项目角度补偿接口。 */
extern float angle_y_correction;  /* roll/pitch 修正量之一，保留旧项目角度补偿接口。 */
extern uint64_t g_imu_timestamp;  /* 保留旧协议时间戳变量，当前发送帧时间戳仍置 0。 */

#pragma pack(pop)

void ICM42688_WriteReg(uint8_t reg, uint8_t value);  /* 通过 SPI 写 ICM42688 单个寄存器。 */
uint8_t ICM42688_ReadReg(uint8_t reg);  /* 通过 SPI 读 ICM42688 单个寄存器。 */
void ICM42688_ReadBurst(uint8_t reg, uint8_t *buffer, uint8_t len);  /* 通过 SPI 从指定寄存器开始连续读取多个字节。 */
uint8_t ICM42688_ReadWhoAmI(void);  /* 读取 WHO_AM_I，用于确认 ICM42688-P 通信正常。 */
uint8_t ICM42688_ReadWhoAmI_LsbCmd(void);  /* 用 LSB 命令格式读取 WHO_AM_I，用于兼容早期 HXY 驱动差异。 */
uint8_t ICM42688_ReadWhoAmI_MsbCmd(void);  /* 用 MSB 命令格式读取 WHO_AM_I，用于 ICM42688-P 正式 SPI 格式。 */
uint8_t ICM42688_GetSpiCmdMode(void);  /* 返回当前识别出的 SPI 命令格式。 */
void ICM42688_Init(void);  /* 初始化 ICM42688-P：识别通信格式、软复位、配置接口和量程，最后关闭采样。 */
void ICM42688_ACC_Suspend(void);  /* 关闭 ICM42688 加速度计和陀螺仪，使 IMU 进入待机低功耗状态。 */
void ICM42688_ACC_Wakeup(void);  /* 发送数据前打开加速度计低功耗模式，陀螺仪保持关闭。 */
void ICM42688_GYR_Enable(void);  /* 打开陀螺仪和加速度计，主要保留给动态姿态融合或调试使用。 */
void ICM42688_GYR_Disable(void);  /* 关闭陀螺仪，仅保留温度/加速度计工作状态。 */
void ICM42688_ReadData(IMU_Data_t *data);  /* 读取 ICM42688 三轴加速度、三轴角速度和温度，并转换成物理量。 */
float ICM42688_ReadTemperatureC(void);  /* 读取 ICM42688 内置温度传感器并转换为摄氏度。 */
void imu_init(void);  /* 初始化四元数姿态状态。 */
eulerian_angles_t imu_get_eulerian_angles(float gx, float gy, float gz, float ax, float ay, float az, float g_param_kp, float g_param_ki);  /* 执行动态姿态融合并输出欧拉角。 */
eulerian_angles_t ICM42688_GetStaticAngles(float ax, float ay, float az);  /* 只根据重力加速度分量计算静态 roll/pitch。 */
float reverse_float_endian(float value);  /* 反转 float 字节序，匹配原 V3.2 上传协议。 */
void ICM42688_Pack(IMU_Data_t *data, eulerian_angles_t *angles, float temperature, float battery_percent, uint16_t device_id, rs485_data_t *rs485buf);  /* 把 IMU、角度、温度、电量和设备 ID 打包为原 RS485/MQTT 二进制协议帧。 */
void ICM42688Pack(IMU_Data_t *data, rs485_data_t *rs485buf);  /* 兼容旧代码的打包封装，使用默认设备 ID 和默认电量。 */
void ICM42688_angle_correction(IMU_Data_t *data, float angle_x, float angle_y);  /* 给陀螺仪角速度增加补偿量，保留旧动态姿态接口。 */

#endif

