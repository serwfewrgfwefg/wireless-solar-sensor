#include "app.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"
#include "ICM42688P.h"
#include "bq27220.h"
#include "lte.h"
#include "app_config.h"
#include "config_cli.h"

#define APP_BOOT_DOWNLOAD_WINDOW_MS   10000UL  /* 上电后保留给 ST-LINK/Keil 下载的等待窗口，结束后才允许进入低功耗业务。 */
#define APP_CONFIG_MODE_TIMEOUT_MS    120000UL  /* 进入 Type-C 参数配置模式后的无操作超时时间。 */
#define APP_RTC_WAKEUP_MAX_SEC        65536UL  /* RTC WakeUp Timer 单次可设置的最大秒数，长等待会被拆段。 */
#define APP_RTC_DAY_SECONDS           86400UL  /* 一天的秒数，用于 RTC 秒数跨午夜回绕计算。 */

typedef enum
{
  APP_WAKE_NONE = 0,
  APP_WAKE_RTC,
  APP_WAKE_CONFIG
} app_wake_reason_t;

static IMU_Data_t s_imu_data;  /* 最近一次读取到的 IMU 原始物理量。 */
static eulerian_angles_t s_static_angles;  /* 由三轴加速度计算出的静态倾角。 */
static rs485_data_t s_sensor_frame;  /* 即将通过 MQTT 发送的二进制协议帧。 */
static float s_frame_battery_percent = BQ27220_INVALID_PERCENT;  /* 本次上报周期内缓存的电量百分比。 */
static uint32_t s_lte_last_tick;  /* LTE 状态机上一次推进时的 HAL tick。 */
static volatile app_wake_reason_t s_wake_reason = APP_WAKE_NONE;  /* STOP2 醒来的原因，由中断回调写入。 */
static volatile uint8_t s_config_wake_pending = 0U;  /* Type-C 配置唤醒待处理标志。 */
static uint8_t s_usb_config_armed = 1U;  /* USB 插入电平轮询的消抖/重入保护标志。 */

extern void SystemClock_Config(void);  /* 配置系统时钟：使用 HSE+PLL 作为主频，同时打开 LSE 供 RTC 低功耗计时。 */

static void App_InitModules(void);  /* 初始化应用依赖的参数、IMU、电量计、RTC 唤醒和姿态算法参数。 */
static void App_RunBootUpload(void);  /* 执行上电首发流程：唤醒传感器和 4G，连接 MQTT，快速发送若干帧。 */
static void App_RunPeriodicUpload(void);  /* 执行周期上报流程：唤醒传感器和 4G，连接 MQTT，在突发窗口内发送数据。 */
static void App_RtcWakeupInit(void);  /* 初始化 RTC WakeUp Timer 的中断通道，真正的唤醒时间在进入 STOP2 前设置。 */
static void App_SleepWithConfig(uint32_t seconds, uint8_t normal_interval_phase);  /* 带配置入口的休眠等待：既能被 RTC 到点唤醒，也能被 USB 配置中断提前唤醒。 */
static app_wake_reason_t App_EnterStopUntilEvent(uint32_t seconds, uint32_t *elapsed_sec);  /* 设置 RTC 唤醒时间并进入 STOP2，醒来后恢复系统时钟和通信外设。 */
static uint8_t App_ConfigRequested(void);  /* 检查是否需要进入参数配置模式，包括外部中断标志和 USB 插入电平轮询。 */
static uint8_t App_RunConfigMode(void);  /* 运行 LPUART2 参数配置窗口，处理 GET/SET/SAVE/DEFAULT/EXIT 命令。 */
static uint32_t App_RtcSecondsOfDay(void);  /* 读取 RTC 当前时分秒，并转换为当天内的秒数。 */
static uint32_t App_RtcElapsedSeconds(uint32_t start_sec, uint32_t end_sec);  /* 计算两个 RTC 秒数之间的差值，兼容跨午夜回绕。 */
static void App_ReduceRemaining(uint32_t *remaining_sec, uint32_t elapsed_sec);  /* 从剩余等待时间中扣除已经过去的时间，防止无符号下溢。 */
static void App_LtePowerOn(void);  /* 打开 4G 模块供电，等待模块启动，并初始化 LTE 状态机。 */
static void App_LtePowerOff(void);  /* 关闭 4G 模块相关控制脚并清空 LTE 状态机。 */
static void App_PrepareStopPeripherals(void);  /* 进入 STOP2 前关闭高功耗外设，并把空闲 GPIO 配到低漏电状态。 */
static void App_RunLteEx(uint8_t reset_tick);  /* 按经过的毫秒数推进 LTE 状态机，可选择重置 LTE 计时基准。 */
static void App_RunLte(void);  /* 推进一次 LTE 状态机，不重置 LTE 计时基准。 */
static uint8_t App_ConnectBlocking(void);  /* 阻塞式等待 LTE 状态机完成 MQTT 连接，成功返回 1，失败或超时返回 0。 */
static uint8_t App_WaitLteReady(uint32_t timeout_ms);  /* 等待 LTE 保持 CONNECTED 状态，常用于连续发送两帧之间。 */
static void App_LteDrain(uint32_t duration_ms);  /* 在指定时间内继续运行 LTE 状态机，用于发送后收尾和串口残留处理。 */
static void App_QueueSensorFrame(void);  /* 读取 IMU 和电量数据，计算静态角度，打包为 rs485_data_t 并交给 LTE 发送。 */
static void App_BurstSend(uint32_t duration_ms);  /* 在突发窗口内按配置周期持续排队发送传感器帧。 */
static float App_ReadBatteryPercent(void);  /* 读取当前电量百分比，底层由 bq27220 驱动完成。 */
static float App_ClampAngle(float angle);  /* 限制角度范围到 -90 到 90 度，避免异常值进入报文。 */

/**
 * @brief 应用层主入口：完成模块初始化、上电首发、周期休眠和周期突发发送。
 */
void App_Run(void)
{
  App_InitModules();  /* 初始化应用依赖的参数、IMU、电量计、RTC 唤醒和姿态算法参数。 */

  App_LtePowerOff();  /* 关闭 4G 模块相关控制脚并清空 LTE 状态机。 */
  ICM42688_ACC_Suspend();  /* 关闭 ICM42688 加速度计和陀螺仪，使 IMU 进入待机低功耗状态。 */
  HAL_Delay(APP_BOOT_DOWNLOAD_WINDOW_MS);

  App_SleepWithConfig(AppConfig_Get()->initial_send_delay_sec, 0U);  /* 带配置入口的休眠等待：既能被 RTC 到点唤醒，也能被 USB 配置中断提前唤醒。 */
  App_RunBootUpload();  /* 执行上电首发流程：唤醒传感器和 4G，连接 MQTT，快速发送若干帧。 */

  while (1)
  {
    App_SleepWithConfig(AppConfig_Get()->normal_send_interval_sec, 1U);  /* 带配置入口的休眠等待：既能被 RTC 到点唤醒，也能被 USB 配置中断提前唤醒。 */
    App_RunPeriodicUpload();  /* 执行周期上报流程：唤醒传感器和 4G，连接 MQTT，在突发窗口内发送数据。 */
  }
}

/**
 * @brief 应用层 RTC 唤醒回调：记录本次 STOP2 是由 RTC 定时唤醒。
 * @param rtc RTC 句柄，HAL 回调传入。
 */
void App_RtcWakeupCallback(RTC_HandleTypeDef *rtc)
{
  if ((rtc != NULL) && (rtc->Instance == RTC) && (s_wake_reason == APP_WAKE_NONE))
  {
    s_wake_reason = APP_WAKE_RTC;
  }
}

/**
 * @brief 应用层配置唤醒回调：记录 Type-C/USB 外部中断请求进入参数配置模式。
 * @param GPIO_Pin 触发外部中断的 GPIO 引脚号。
 */
void App_ConfigWakeupCallback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_5)
  {
    s_wake_reason = APP_WAKE_CONFIG;
    s_config_wake_pending = 1U;
    s_usb_config_armed = 0U;
  }
}

/**
 * @brief 初始化应用依赖的参数、IMU、电量计、RTC 唤醒和姿态算法参数。
 */
static void App_InitModules(void)
{
#if APP_CONFIG_FORCE_FLASH_DEFAULTS
  AppConfig_ForceWriteFlashDefaults();  /* 测试/量产辅助函数：上电时强制把写死参数覆盖到 Flash。 */
#endif
  AppConfig_Init();  /* 初始化运行参数：Flash 校验通过则读取 Flash，否则使用默认参数。 */
  ICM42688_Init();  /* 初始化 ICM42688-P：识别通信格式、软复位、配置接口和量程，最后关闭采样。 */
  imu_init();  /* 初始化四元数姿态状态。 */
  BQ27220_Init();  /* 初始化 BQ27220 使用的 I2C 底层接口。 */
  App_RtcWakeupInit();  /* 初始化 RTC WakeUp Timer 的中断通道，真正的唤醒时间在进入 STOP2 前设置。 */

  g_param_kp = 640.0f;
  g_param_ki = 0.3f;
  angle_x_correction = -0.0f;
  angle_y_correction = -0.0f;
}

/**
 * @brief 执行上电首发流程：唤醒传感器和 4G，连接 MQTT，快速发送若干帧。
 */
static void App_RunBootUpload(void)
{
  ICM42688_ACC_Wakeup();  /* 发送数据前打开加速度计低功耗模式，陀螺仪保持关闭。 */
  BQ27220_Init();  /* 初始化 BQ27220 使用的 I2C 底层接口。 */
  s_frame_battery_percent = App_ReadBatteryPercent();  /* 读取当前电量百分比，底层由 bq27220 驱动完成。 */

  App_LtePowerOn();  /* 打开 4G 模块供电，等待模块启动，并初始化 LTE 状态机。 */
  if (App_ConnectBlocking())
  {
    for (uint8_t i = 0; i < AppConfig_Get()->fast_boot_send_count; i++)
    {
      App_QueueSensorFrame();  /* 读取 IMU 和电量数据，计算静态角度，打包为 rs485_data_t 并交给 LTE 发送。 */
      if (App_WaitLteReady(5000UL) == 0U)
      {
        break;
      }
      App_LteDrain(AppConfig_Get()->fast_boot_gap_ms);  /* 在指定时间内继续运行 LTE 状态机，用于发送后收尾和串口残留处理。 */
    }
    App_LteDrain(AppConfig_Get()->lte_drain_ms);  /* 在指定时间内继续运行 LTE 状态机，用于发送后收尾和串口残留处理。 */
  }

  App_LtePowerOff();  /* 关闭 4G 模块相关控制脚并清空 LTE 状态机。 */
  ICM42688_ACC_Suspend();  /* 关闭 ICM42688 加速度计和陀螺仪，使 IMU 进入待机低功耗状态。 */
}

/**
 * @brief 执行周期上报流程：唤醒传感器和 4G，连接 MQTT，在突发窗口内发送数据。
 */
static void App_RunPeriodicUpload(void)
{
  ICM42688_ACC_Wakeup();  /* 发送数据前打开加速度计低功耗模式，陀螺仪保持关闭。 */
  BQ27220_Init();  /* 初始化 BQ27220 使用的 I2C 底层接口。 */
  s_frame_battery_percent = App_ReadBatteryPercent();  /* 读取当前电量百分比，底层由 bq27220 驱动完成。 */

  App_LtePowerOn();  /* 打开 4G 模块供电，等待模块启动，并初始化 LTE 状态机。 */
  if (App_ConnectBlocking())
  {
    App_BurstSend(AppConfig_Get()->burst_duration_ms);  /* 在突发窗口内按配置周期持续排队发送传感器帧。 */
    App_LteDrain(AppConfig_Get()->lte_drain_ms);  /* 在指定时间内继续运行 LTE 状态机，用于发送后收尾和串口残留处理。 */
  }

  App_LtePowerOff();  /* 关闭 4G 模块相关控制脚并清空 LTE 状态机。 */
  ICM42688_ACC_Suspend();  /* 关闭 ICM42688 加速度计和陀螺仪，使 IMU 进入待机低功耗状态。 */
}

/**
 * @brief 初始化 RTC WakeUp Timer 的中断通道，真正的唤醒时间在进入 STOP2 前设置。
 */
static void App_RtcWakeupInit(void)
{
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
  HAL_NVIC_SetPriority(RTC_TAMP_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(RTC_TAMP_IRQn);
}

/**
 * @brief 带配置入口的休眠等待：既能被 RTC 到点唤醒，也能被 USB 配置中断提前唤醒。
 * @param seconds 需要等待或设置 RTC 唤醒的秒数。
 * @param normal_interval_phase 是否处于正常周期发送等待阶段，1 表示是，0 表示上电首发等待。
 */
static void App_SleepWithConfig(uint32_t seconds, uint8_t normal_interval_phase)
{
  uint32_t remaining_sec = seconds;

  while (remaining_sec > 0U)
  {
    uint32_t elapsed_sec = 0U;
    app_wake_reason_t reason;

    if (App_ConfigRequested())
    {
      uint32_t cfg_start_sec = App_RtcSecondsOfDay();  /* 读取 RTC 当前时分秒，并转换为当天内的秒数。 */
      uint8_t events = App_RunConfigMode();  /* 运行 LPUART2 参数配置窗口，处理 GET/SET/SAVE/DEFAULT/EXIT 命令。 */
      uint32_t cfg_elapsed_sec = App_RtcElapsedSeconds(cfg_start_sec, App_RtcSecondsOfDay());  /* 计算两个 RTC 秒数之间的差值，兼容跨午夜回绕。 */

      if ((events & CONFIG_CLI_EVENT_TIMING_SAVED) != 0U)
      {
        remaining_sec = normal_interval_phase ? AppConfig_Get()->normal_send_interval_sec
                                              : AppConfig_Get()->initial_send_delay_sec;
      }
      else
      {
        App_ReduceRemaining(&remaining_sec, cfg_elapsed_sec);  /* 从剩余等待时间中扣除已经过去的时间，防止无符号下溢。 */
      }
      continue;
    }

    uint32_t sleep_sec = remaining_sec;
    if (sleep_sec > APP_RTC_WAKEUP_MAX_SEC)
    {
      sleep_sec = APP_RTC_WAKEUP_MAX_SEC;
    }

    reason = App_EnterStopUntilEvent(sleep_sec, &elapsed_sec);  /* 设置 RTC 唤醒时间并进入 STOP2，醒来后恢复系统时钟和通信外设。 */
    if (reason == APP_WAKE_CONFIG)
    {
      App_ReduceRemaining(&remaining_sec, elapsed_sec);  /* 从剩余等待时间中扣除已经过去的时间，防止无符号下溢。 */
      continue;
    }

    if (elapsed_sec == 0U)
    {
      elapsed_sec = sleep_sec;
    }
    App_ReduceRemaining(&remaining_sec, elapsed_sec);  /* 从剩余等待时间中扣除已经过去的时间，防止无符号下溢。 */
  }
}

/**
 * @brief 设置 RTC 唤醒时间并进入 STOP2，醒来后恢复系统时钟和通信外设。
 * @param seconds 需要等待或设置 RTC 唤醒的秒数。
 * @param elapsed_sec 输出参数，返回本次 STOP2 实际睡眠秒数。
 */
static app_wake_reason_t App_EnterStopUntilEvent(uint32_t seconds, uint32_t *elapsed_sec)
{
  uint32_t start_sec;
  uint32_t end_sec;
  app_wake_reason_t reason;

  if (elapsed_sec != NULL)
  {
    *elapsed_sec = 0U;
  }

  if (seconds == 0U)
  {
    return APP_WAKE_RTC;
  }

  if (seconds > APP_RTC_WAKEUP_MAX_SEC)
  {
    seconds = APP_RTC_WAKEUP_MAX_SEC;
  }

  HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);

  start_sec = App_RtcSecondsOfDay();  /* 读取 RTC 当前时分秒，并转换为当天内的秒数。 */
  s_wake_reason = APP_WAKE_NONE;

  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
  __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_5);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
                                  seconds - 1U,
                                  RTC_WAKEUPCLOCK_CK_SPRE_16BITS,
                                  0U) != HAL_OK)
  {
    return APP_WAKE_RTC;
  }

  App_PrepareStopPeripherals();  /* 进入 STOP2 前关闭高功耗外设，并把空闲 GPIO 配到低漏电状态。 */
  HAL_SuspendTick();
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

  SystemClock_Config();  /* 配置系统时钟：使用 HSE+PLL 作为主频，同时打开 LSE 供 RTC 低功耗计时。 */
  HAL_ResumeTick();

  MX_GPIO_Init();  /* 初始化全部普通 GPIO，包括 LED、4G 控制脚、IMU 片选和 USB 唤醒脚。 */
  MX_I2C1_Init();  /* 初始化 I2C1，用于 BQ27220 电量计。 */
  MX_SPI1_Init();  /* 初始化 SPI1 为主机模式，用于 ICM42688。 */
  MX_USART2_UART_Init();  /* 初始化 USART2 为 4G 模块 AT 通信串口，并启动接收中断。 */

  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

  end_sec = App_RtcSecondsOfDay();  /* 读取 RTC 当前时分秒，并转换为当天内的秒数。 */
  if (elapsed_sec != NULL)
  {
    uint32_t elapsed = App_RtcElapsedSeconds(start_sec, end_sec);  /* 计算两个 RTC 秒数之间的差值，兼容跨午夜回绕。 */
    *elapsed_sec = (elapsed > seconds) ? seconds : elapsed;
  }

  reason = s_wake_reason;
  if (reason == APP_WAKE_CONFIG)
  {
    s_config_wake_pending = 1U;
  }

  return reason;
}

/**
 * @brief 检查是否需要进入参数配置模式，包括外部中断标志和 USB 插入电平轮询。
 */
static uint8_t App_ConfigRequested(void)
{
  uint8_t requested = 0U;

  __disable_irq();
  if (s_config_wake_pending != 0U)
  {
    s_config_wake_pending = 0U;
    requested = 1U;
  }
  __enable_irq();

  if (requested != 0U)
  {
    return 1U;
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
  {
    if (s_usb_config_armed != 0U)
    {
      s_usb_config_armed = 0U;
      return 1U;
    }
  }
  else
  {
    s_usb_config_armed = 1U;
  }

  return 0U;
}

/**
 * @brief 运行 LPUART2 参数配置窗口，处理 GET/SET/SAVE/DEFAULT/EXIT 命令。
 */
static uint8_t App_RunConfigMode(void)
{
  uint32_t last_activity_tick = HAL_GetTick();
  uint8_t all_events = CONFIG_CLI_EVENT_NONE;

  MX_LPUART2_UART_Init();  /* 初始化 LPUART2 为参数配置串口，默认 115200 8N1。 */
  ConfigCli_Init();  /* 进入配置模式时初始化 CLI 工作副本和 LPUART2 接收。 */

  while ((HAL_GetTick() - last_activity_tick) < APP_CONFIG_MODE_TIMEOUT_MS)
  {
    uint8_t events = ConfigCli_Task();  /* 配置 CLI 轮询任务，处理已接收完整命令并返回事件。 */

    if ((events & CONFIG_CLI_EVENT_ACTIVITY) != 0U)
    {
      last_activity_tick = HAL_GetTick();
    }

    all_events |= events;
    if ((events & CONFIG_CLI_EVENT_EXIT) != 0U)
    {
      break;
    }

    __WFI();
  }

  ConfigCli_DeInit();  /* 退出配置模式时清理 CLI 接收状态。 */
  HAL_UART_DeInit(&hlpuart2);
  return all_events;
}

/**
 * @brief 读取 RTC 当前时分秒，并转换为当天内的秒数。
 */
static uint32_t App_RtcSecondsOfDay(void)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};

  (void)HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
  (void)HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

  return ((uint32_t)time.Hours * 3600UL) +
         ((uint32_t)time.Minutes * 60UL) +
         (uint32_t)time.Seconds;
}

/**
 * @brief 计算两个 RTC 秒数之间的差值，兼容跨午夜回绕。
 * @param start_sec 起始秒数。
 * @param end_sec 结束秒数。
 */
static uint32_t App_RtcElapsedSeconds(uint32_t start_sec, uint32_t end_sec)
{
  if (end_sec >= start_sec)
  {
    return end_sec - start_sec;
  }

  return (APP_RTC_DAY_SECONDS - start_sec) + end_sec;
}

/**
 * @brief 从剩余等待时间中扣除已经过去的时间，防止无符号下溢。
 * @param remaining_sec 剩余等待秒数指针。
 * @param elapsed_sec 输出参数，返回本次 STOP2 实际睡眠秒数。
 */
static void App_ReduceRemaining(uint32_t *remaining_sec, uint32_t elapsed_sec)
{
  if (remaining_sec == NULL)
  {
    return;
  }

  if (elapsed_sec >= *remaining_sec)
  {
    *remaining_sec = 0U;
  }
  else
  {
    *remaining_sec -= elapsed_sec;
  }
}

/**
 * @brief 打开 4G 模块供电，等待模块启动，并初始化 LTE 状态机。
 */
static void App_LtePowerOn(void)
{
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(AppConfig_Get()->lte_power_on_delay_ms);
  lte_init();
  s_lte_last_tick = HAL_GetTick();
}

/**
 * @brief 关闭 4G 模块相关控制脚并清空 LTE 状态机。
 */
static void App_LtePowerOff(void)
{
  HAL_GPIO_WritePin(DTR_GPIO_Port, DTR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
  lte_power_reset_state();
}

/**
 * @brief 进入 STOP2 前关闭高功耗外设，并把空闲 GPIO 配到低漏电状态。
 */
static void App_PrepareStopPeripherals(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_UART_DeInit(&huart2);
  HAL_I2C_DeInit(&hi2c1);
  HAL_SPI_DeInit(&hspi1);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_7, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = SPI1_CS_Pin | GPIO_PIN_1 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_6 | GPIO_PIN_7 |
                        GPIO_PIN_8 | GPIO_PIN_9 | GPOUT_Pin | RDY_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
 * @brief 按经过的毫秒数推进 LTE 状态机，可选择重置 LTE 计时基准。
 * @param reset_tick 是否重置 LTE 状态机计时基准。
 */
static void App_RunLteEx(uint8_t reset_tick)
{
  uint32_t now = HAL_GetTick();

  if (reset_tick)
  {
    s_lte_last_tick = now;
  }

  lte_tick(now - s_lte_last_tick);
  s_lte_last_tick = now;
  lte_task();
}

/**
 * @brief 推进一次 LTE 状态机，不重置 LTE 计时基准。
 */
static void App_RunLte(void)
{
  App_RunLteEx(0U);  /* 按经过的毫秒数推进 LTE 状态机，可选择重置 LTE 计时基准。 */
}

/**
 * @brief 阻塞式等待 LTE 状态机完成 MQTT 连接，成功返回 1，失败或超时返回 0。
 */
static uint8_t App_ConnectBlocking(void)
{
  uint32_t start_tick;

  App_RunLteEx(1U);  /* 按经过的毫秒数推进 LTE 状态机，可选择重置 LTE 计时基准。 */
  lte_connect();
  start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < AppConfig_Get()->burst_connect_timeout_ms)
  {
    App_RunLte();  /* 推进一次 LTE 状态机，不重置 LTE 计时基准。 */
    if (lte_is_connected())
    {
      return 1U;
    }
    if (lte_is_failed())
    {
      return 0U;
    }
    __WFI();
  }

  lte_abort();
  App_LteDrain(1000UL);  /* 在指定时间内继续运行 LTE 状态机，用于发送后收尾和串口残留处理。 */
  return 0U;
}

/**
 * @brief 等待 LTE 保持 CONNECTED 状态，常用于连续发送两帧之间。
 * @param timeout_ms 超时时间，单位 ms。
 */
static uint8_t App_WaitLteReady(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    App_RunLte();  /* 推进一次 LTE 状态机，不重置 LTE 计时基准。 */
    if (lte_is_connected())
    {
      return 1U;
    }
    if (lte_is_failed())
    {
      return 0U;
    }
    __WFI();
  }

  return lte_is_connected();
}

/**
 * @brief 在指定时间内继续运行 LTE 状态机，用于发送后收尾和串口残留处理。
 * @param duration_ms 持续运行时间，单位 ms。
 */
static void App_LteDrain(uint32_t duration_ms)
{
  uint32_t start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < duration_ms)
  {
    App_RunLte();  /* 推进一次 LTE 状态机，不重置 LTE 计时基准。 */
    __WFI();
  }
}

/**
 * @brief 读取 IMU 和电量数据，计算静态角度，打包为 rs485_data_t 并交给 LTE 发送。
 */
static void App_QueueSensorFrame(void)
{
  ICM42688_ReadData(&s_imu_data);  /* 读取 ICM42688 三轴加速度、三轴角速度和温度，并转换成物理量。 */
  s_static_angles = ICM42688_GetStaticAngles(s_imu_data.accel_x,
                                             s_imu_data.accel_y,
                                             s_imu_data.accel_z);
  s_static_angles.roll = App_ClampAngle(s_static_angles.roll + angle_x_correction);
  s_static_angles.pitch = App_ClampAngle(s_static_angles.pitch + angle_y_correction);

  ICM42688_Pack(&s_imu_data,
                &s_static_angles,
                s_imu_data.temperature,
                s_frame_battery_percent,
                (uint16_t)AppConfig_Get()->device_id,
                &s_sensor_frame);
  lte_request_send(&s_sensor_frame);
}

/**
 * @brief 在突发窗口内按配置周期持续排队发送传感器帧。
 * @param duration_ms 持续运行时间，单位 ms。
 */
static void App_BurstSend(uint32_t duration_ms)
{
  uint32_t start_tick = HAL_GetTick();
  uint32_t next_send_tick = start_tick;

  while ((HAL_GetTick() - start_tick) < duration_ms)
  {
    App_RunLte();  /* 推进一次 LTE 状态机，不重置 LTE 计时基准。 */

    if (lte_is_failed())
    {
      return;
    }

    if (lte_is_connected() && ((int32_t)(HAL_GetTick() - next_send_tick) >= 0))
    {
      App_QueueSensorFrame();  /* 读取 IMU 和电量数据，计算静态角度，打包为 rs485_data_t 并交给 LTE 发送。 */
      next_send_tick += AppConfig_Get()->burst_period_ms;
    }

    __WFI();
  }
}

/**
 * @brief 读取当前电量百分比，底层由 bq27220 驱动完成。
 */
static float App_ReadBatteryPercent(void)
{
  return BQ27220_GetBatteryPercent();  /* 简化接口：直接返回电量百分比或无效值。 */
}

/**
 * @brief 限制角度范围到 -90 到 90 度，避免异常值进入报文。
 * @param angle 待限制的角度值。
 */
static float App_ClampAngle(float angle)
{
  if (angle > 90.0f)
  {
    return 90.0f;
  }
  if (angle < -90.0f)
  {
    return -90.0f;
  }
  return angle;
}
