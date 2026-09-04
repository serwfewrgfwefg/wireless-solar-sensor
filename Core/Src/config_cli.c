#include "config_cli.h"
#include "app_config.h"
#include "usart.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#define CONFIG_CLI_LINE_LEN  192U  /* LPUART2 配置命令单行最大长度。 */

static uint8_t s_rx_byte;  /* LPUART2 当前接收到的单字节缓存。 */
static volatile uint8_t s_line_ready;  /* CLI 已收到完整一行命令的标志。 */
static volatile uint8_t s_line_overflow;  /* CLI 接收行超长标志。 */
static volatile uint16_t s_rx_pos;  /* CLI 当前接收行写入位置。 */
static char s_rx_line[CONFIG_CLI_LINE_LEN];  /* CLI 中断中组装的命令行缓冲区。 */
static char s_process_line[CONFIG_CLI_LINE_LEN];  /* CLI 主循环中处理命令用的安全副本。 */
static app_config_t s_work_config;  /* CLI 配置工作副本，SAVE 前不影响 Flash。 */

/**
 * @brief 重新启动 LPUART2 单字节中断接收。
 */
static void config_cli_restart_rx(void)
{
  if (hlpuart2.Instance == LPUART2)
  {
    (void)HAL_UART_Receive_IT(&hlpuart2, &s_rx_byte, 1U);
  }
}

/**
 * @brief 通过 LPUART2 发送一段普通字符串给上位机。
 * @param text 输入字符串。
 */
static void config_cli_send(const char *text)
{
  if (text != NULL)
  {
    (void)HAL_UART_Transmit(&hlpuart2,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            1000U);
  }
}

/**
 * @brief 格式化字符串后通过 LPUART2 发送给上位机。
 * @param fmt printf 风格格式字符串。
 */
static void config_cli_sendf(const char *fmt, ...)
{
  char buf[160];
  va_list ap;

  va_start(ap, fmt);
  (void)vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  config_cli_send(buf);  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
}

static char *config_cli_trim(char *s)
{
  char *end;

  while ((*s != '\0') && isspace((unsigned char)*s))
  {
    s++;
  }

  end = s + strlen(s);
  while ((end > s) && isspace((unsigned char)end[-1]))
  {
    *--end = '\0';
  }

  return s;
}

/**
 * @brief 忽略大小写比较命令是否完全相同。
 * @param text 输入字符串。
 * @param cmd 命令字符串。
 */
static uint8_t config_cli_cmd_equals(const char *text, const char *cmd)
{
  while ((*cmd != '\0') && (*text != '\0'))
  {
    if (toupper((unsigned char)*text) != toupper((unsigned char)*cmd))
    {
      return 0U;
    }
    text++;
    cmd++;
  }

  return ((*cmd == '\0') && (*text == '\0')) ? 1U : 0U;
}

/**
 * @brief 忽略大小写判断命令是否以前缀开头。
 * @param text 输入字符串。
 * @param cmd 命令字符串。
 */
static uint8_t config_cli_cmd_starts_with(const char *text, const char *cmd)
{
  while (*cmd != '\0')
  {
    if ((*text == '\0') ||
        (toupper((unsigned char)*text) != toupper((unsigned char)*cmd)))
    {
      return 0U;
    }
    text++;
    cmd++;
  }

  return 1U;
}

/**
 * @brief 把配置键名转换为小写，统一后续匹配逻辑。
 * @param s 状态枚举或字符串指针。
 */
static void config_cli_key_lower(char *s)
{
  while (*s != '\0')
  {
    *s = (char)tolower((unsigned char)*s);
    s++;
  }
}

/**
 * @brief 把字符串解析为指定范围内的无符号整数。
 * @param text 输入字符串。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 * @param out 解析结果输出指针。
 */
static uint8_t config_cli_parse_u32(const char *text,
                                    uint32_t min_value,
                                    uint32_t max_value,
                                    uint32_t *out)
{
  char *end = NULL;
  unsigned long value;

  if ((text == NULL) || (out == NULL) || (*text == '\0'))
  {
    return 0U;
  }

  value = strtoul(text, &end, 10);
  if ((end == text) || (*config_cli_trim(end) != '\0') ||
      (value < min_value) || (value > max_value))
  {
    return 0U;
  }

  *out = (uint32_t)value;
  return 1U;
}

/**
 * @brief 校验并写入字符串类配置项。
 * @param dst 目标缓冲区指针。
 * @param dst_len 目标缓冲区长度。
 * @param value 配置值字符串或寄存器值。
 */
static uint8_t config_cli_set_string(char *dst,
                                     uint32_t dst_len,
                                     const char *value)
{
  size_t len;

  if ((dst == NULL) || (value == NULL) || (dst_len == 0U))
  {
    return 0U;
  }

  len = strlen(value);
  if ((len == 0U) || (len >= dst_len))
  {
    return 0U;
  }

  (void)strncpy(dst, value, dst_len - 1U);
  dst[dst_len - 1U] = '\0';
  return 1U;
}

/**
 * @brief 把当前工作配置按 key=value 形式输出给上位机。
 */
static void config_cli_print_config(void)
{
  config_cli_send("OK\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_sendf("initial_delay_sec=%lu\r\n", (unsigned long)s_work_config.initial_send_delay_sec);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("interval_sec=%lu\r\n", (unsigned long)s_work_config.normal_send_interval_sec);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("burst_ms=%lu\r\n", (unsigned long)s_work_config.burst_duration_ms);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("burst_period_ms=%lu\r\n", (unsigned long)s_work_config.burst_period_ms);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("lte_wait_ms=%lu\r\n", (unsigned long)s_work_config.lte_power_on_delay_ms);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("fast_count=%lu\r\n", (unsigned long)s_work_config.fast_boot_send_count);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("fast_gap_ms=%lu\r\n", (unsigned long)s_work_config.fast_boot_gap_ms);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("drain_ms=%lu\r\n", (unsigned long)s_work_config.lte_drain_ms);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("connect_timeout_ms=%lu\r\n", (unsigned long)s_work_config.burst_connect_timeout_ms);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("device_id=%lu\r\n", (unsigned long)s_work_config.device_id);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("mqtt_host=%s\r\n", s_work_config.mqtt_host);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("mqtt_port=%s\r\n", s_work_config.mqtt_port);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("mqtt_user=%s\r\n", s_work_config.mqtt_user);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("mqtt_pass=%s\r\n", s_work_config.mqtt_pass);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("mqtt_client_id=%s\r\n", s_work_config.mqtt_client_id);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_sendf("mqtt_topic=%s\r\n", s_work_config.mqtt_pub_topic);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
  config_cli_send("END\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
}

/**
 * @brief 输出 LPUART2 参数配置命令帮助。
 */
static void config_cli_print_help(void)
{
  config_cli_send("OK\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_send("GET\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_send("SET key=value\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_send("SAVE\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_send("DEFAULT\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_send("EXIT\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_send("keys: initial_delay_sec interval_sec burst_ms burst_period_ms lte_wait_ms fast_count fast_gap_ms drain_ms connect_timeout_ms device_id mqtt_host mqtt_port mqtt_user mqtt_pass mqtt_client_id mqtt_topic\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_send("END\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
}

/**
 * @brief 解析并修改单个配置项，修改的是工作副本而不是立即写 Flash。
 * @param key 配置键名。
 * @param value 配置值字符串或寄存器值。
 */
static uint8_t config_cli_set_value(char *key, char *value)
{
  uint32_t number;

  config_cli_key_lower(key);  /* 把配置键名转换为小写，统一后续匹配逻辑。 */

  if ((strcmp(key, "initial_delay_sec") == 0) || (strcmp(key, "first_delay_sec") == 0))
  {
    if (!config_cli_parse_u32(value, 1UL, 86400UL, &number)) return 0U;
    s_work_config.initial_send_delay_sec = number;
  }
  else if ((strcmp(key, "interval_sec") == 0) || (strcmp(key, "send_interval_sec") == 0))
  {
    if (!config_cli_parse_u32(value, 1UL, 86400UL, &number)) return 0U;
    s_work_config.normal_send_interval_sec = number;
  }
  else if (strcmp(key, "burst_ms") == 0)
  {
    if (!config_cli_parse_u32(value, 100UL, 600000UL, &number)) return 0U;
    s_work_config.burst_duration_ms = number;
  }
  else if (strcmp(key, "burst_period_ms") == 0)
  {
    if (!config_cli_parse_u32(value, 10UL, 60000UL, &number)) return 0U;
    s_work_config.burst_period_ms = number;
  }
  else if (strcmp(key, "lte_wait_ms") == 0)
  {
    if (!config_cli_parse_u32(value, 1000UL, 120000UL, &number)) return 0U;
    s_work_config.lte_power_on_delay_ms = number;
  }
  else if (strcmp(key, "fast_count") == 0)
  {
    if (!config_cli_parse_u32(value, 0UL, 20UL, &number)) return 0U;
    s_work_config.fast_boot_send_count = number;
  }
  else if (strcmp(key, "fast_gap_ms") == 0)
  {
    if (!config_cli_parse_u32(value, 0UL, 600000UL, &number)) return 0U;
    s_work_config.fast_boot_gap_ms = number;
  }
  else if (strcmp(key, "drain_ms") == 0)
  {
    if (!config_cli_parse_u32(value, 0UL, 600000UL, &number)) return 0U;
    s_work_config.lte_drain_ms = number;
  }
  else if (strcmp(key, "connect_timeout_ms") == 0)
  {
    if (!config_cli_parse_u32(value, 5000UL, 300000UL, &number)) return 0U;
    s_work_config.burst_connect_timeout_ms = number;
  }
  else if (strcmp(key, "device_id") == 0)
  {
    if (!config_cli_parse_u32(value, 1UL, 65535UL, &number)) return 0U;
    s_work_config.device_id = number;
  }
  else if ((strcmp(key, "mqtt_host") == 0) || (strcmp(key, "host") == 0))
  {
    if (!config_cli_set_string(s_work_config.mqtt_host, APP_CONFIG_MQTT_HOST_LEN, value)) return 0U;
  }
  else if ((strcmp(key, "mqtt_port") == 0) || (strcmp(key, "port") == 0))
  {
    if (!config_cli_set_string(s_work_config.mqtt_port, APP_CONFIG_MQTT_PORT_LEN, value)) return 0U;
  }
  else if ((strcmp(key, "mqtt_user") == 0) || (strcmp(key, "user") == 0))
  {
    if (!config_cli_set_string(s_work_config.mqtt_user, APP_CONFIG_MQTT_USER_LEN, value)) return 0U;
  }
  else if ((strcmp(key, "mqtt_pass") == 0) || (strcmp(key, "pass") == 0))
  {
    if (!config_cli_set_string(s_work_config.mqtt_pass, APP_CONFIG_MQTT_PASS_LEN, value)) return 0U;
  }
  else if ((strcmp(key, "mqtt_client_id") == 0) || (strcmp(key, "client_id") == 0))
  {
    if (!config_cli_set_string(s_work_config.mqtt_client_id, APP_CONFIG_MQTT_CLIENT_ID_LEN, value)) return 0U;
  }
  else if ((strcmp(key, "mqtt_topic") == 0) || (strcmp(key, "topic") == 0))
  {
    if (!config_cli_set_string(s_work_config.mqtt_pub_topic, APP_CONFIG_MQTT_TOPIC_LEN, value)) return 0U;
  }
  else
  {
    return 0U;
  }

  return 1U;
}

/**
 * @brief 解析一整行配置命令，并返回本次命令产生的事件标志。
 * @param line 断言失败所在行号。
 */
static uint8_t config_cli_process_line(char *line)
{
  char *cmd = config_cli_trim(line);
  char *arg;
  uint8_t events = CONFIG_CLI_EVENT_ACTIVITY;

  if (*cmd == '\0')
  {
    return events;
  }

  if (config_cli_cmd_equals(cmd, "GET"))
  {
    config_cli_print_config();  /* 把当前工作配置按 key=value 形式输出给上位机。 */
  }
  else if (config_cli_cmd_equals(cmd, "HELP"))
  {
    config_cli_print_help();  /* 输出 LPUART2 参数配置命令帮助。 */
  }
  else if (config_cli_cmd_equals(cmd, "SAVE"))
  {
    const app_config_t *old_cfg = AppConfig_Get();
    uint8_t timing_changed =
      ((old_cfg->initial_send_delay_sec != s_work_config.initial_send_delay_sec) ||
       (old_cfg->normal_send_interval_sec != s_work_config.normal_send_interval_sec)) ? 1U : 0U;

    AppConfig_Set(&s_work_config);  /* 用外部传入的配置覆盖当前 RAM 配置。 */
    if (AppConfig_Save())
    {
      config_cli_send("OK SAVE\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
      if (timing_changed)
      {
        events |= CONFIG_CLI_EVENT_TIMING_SAVED;
      }
    }
    else
    {
      config_cli_send("ERR SAVE\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
    }
  }
  else if (config_cli_cmd_equals(cmd, "DEFAULT"))
  {
    AppConfig_SetDefaults(&s_work_config);  /* 填充一套出厂默认参数，仅写入 RAM 中传入的结构体。 */
    config_cli_send("OK DEFAULT RAM_ONLY\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  }
  else if (config_cli_cmd_equals(cmd, "EXIT"))
  {
    config_cli_send("OK EXIT\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
    events |= CONFIG_CLI_EVENT_EXIT;
  }
  else if (config_cli_cmd_starts_with(cmd, "SET "))
  {
    arg = config_cli_trim(cmd + 4U);  /* 去掉字符串首尾空白字符，便于解析命令。 */
    char *eq = strchr(arg, '=');
    if (eq == NULL)
    {
      config_cli_send("ERR SET FORMAT\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
    }
    else
    {
      *eq = '\0';
      char *key = config_cli_trim(arg);
      char *value = config_cli_trim(eq + 1U);

      if (config_cli_set_value(key, value))
      {
        config_cli_sendf("OK SET %s\r\n", key);  /* 格式化字符串后通过 LPUART2 发送给上位机。 */
      }
      
      else
      {
        config_cli_send("ERR SET VALUE\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
      }
    }
  }
  else
  {
    config_cli_send("ERR UNKNOWN\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  }

  return events;
}

/**
 * @brief 进入配置模式时初始化 CLI 工作副本和 LPUART2 接收。
 */
void ConfigCli_Init(void)
{
  s_work_config = *AppConfig_Get();

  __disable_irq();
  s_line_ready = 0U;
  s_line_overflow = 0U;
  s_rx_pos = 0U;
  memset(s_rx_line, 0, sizeof(s_rx_line));
  __enable_irq();

  config_cli_send("\r\nCFG READY 115200\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_send("HELP for commands\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
  config_cli_restart_rx();  /* 重新启动 LPUART2 单字节中断接收。 */
}

/**
 * @brief 退出配置模式时清理 CLI 接收状态。
 */
void ConfigCli_DeInit(void)
{
  __disable_irq();
  s_line_ready = 0U;
  s_line_overflow = 0U;
  s_rx_pos = 0U;
  __enable_irq();
}

/**
 * @brief LPUART2 单字节接收完成回调，组装一行命令。
 */
void ConfigCli_UartRxCpltCallback(void)
{
  uint8_t c = s_rx_byte;

  if ((c == '\r') || (c == '\n'))
  {
    if ((s_rx_pos > 0U) && (s_line_ready == 0U))
    {
      s_rx_line[s_rx_pos] = '\0';
      s_line_ready = 1U;
    }
    s_rx_pos = 0U;
  }
  else if ((c == 0x08U) || (c == 0x7FU))
  {
    if (s_rx_pos > 0U)
    {
      s_rx_pos--;
    }
  }
  else if (s_line_ready == 0U)
  {
    if (s_rx_pos < (CONFIG_CLI_LINE_LEN - 1U))
    {
      s_rx_line[s_rx_pos++] = (char)c;
    }
    else
    {
      s_rx_pos = 0U;
      s_line_overflow = 1U;
    }
  }

  config_cli_restart_rx();  /* 重新启动 LPUART2 单字节中断接收。 */
}

/**
 * @brief LPUART2 接收错误回调，重新启动接收以恢复通信。
 */
void ConfigCli_UartErrorCallback(void)
{
  config_cli_restart_rx();  /* 重新启动 LPUART2 单字节中断接收。 */
}

/**
 * @brief 配置 CLI 轮询任务，处理已接收完整命令并返回事件。
 */
uint8_t ConfigCli_Task(void)
{
  uint8_t events = CONFIG_CLI_EVENT_NONE;

  if (s_line_overflow)
  {
    __disable_irq();
    s_line_overflow = 0U;
    __enable_irq();
    config_cli_send("ERR LINE TOO LONG\r\n");  /* 通过 LPUART2 发送一段普通字符串给上位机。 */
    events |= CONFIG_CLI_EVENT_ACTIVITY;
  }

  if (s_line_ready)
  {
    __disable_irq();
    (void)strncpy(s_process_line, s_rx_line, sizeof(s_process_line) - 1U);
    s_process_line[sizeof(s_process_line) - 1U] = '\0';
    s_line_ready = 0U;
    __enable_irq();

    events |= config_cli_process_line(s_process_line);
  }

  return events;
}
