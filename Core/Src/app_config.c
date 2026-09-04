#include "app_config.h"
#include <string.h>

#define APP_CONFIG_MAGIC       0x33474643UL  /* Flash 参数记录魔数，用于判断最后一页是不是本工程写入的配置。 */
#define APP_CONFIG_VERSION     1UL  /* Flash 参数结构版本，结构体变化时改版本可让旧配置失效。 */
#define APP_CONFIG_FLASH_ADDR  (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)  /* 保存参数的 Flash 地址，当前使用片内 Flash 最后一页。 */

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t size;
  uint32_t crc;
  app_config_t cfg;
} app_config_record_t;

static app_config_t g_app_config;  /* 当前 RAM 中正在使用的全局配置。 */

/**
 * @brief 计算配置结构体的 CRC32，用于判断 Flash 里的参数是否可靠。
 * @param data 待发送或待处理数据。
 * @param len 数据长度。
 */
static uint32_t app_config_crc32(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;

  while (len--)
  {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8U; i++)
    {
      if (crc & 1UL)
      {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

/**
 * @brief 安全复制配置字符串，保证目标缓冲区以 0 结尾。
 * @param dst 目标缓冲区指针。
 * @param dst_len 目标缓冲区长度。
 * @param src 源字符串或源数据指针。
 */
static void app_config_copy_string(char *dst, uint32_t dst_len, const char *src)
{
  if ((dst == NULL) || (dst_len == 0U))
  {
    return;
  }

  if (src == NULL)
  {
    dst[0] = '\0';
    return;
  }

  (void)strncpy(dst, src, dst_len - 1U);
  dst[dst_len - 1U] = '\0';
}

/**
 * @brief 填充一套出厂默认参数，仅写入 RAM 中传入的结构体。
 * @param cfg 应用配置结构体指针。
 */
void AppConfig_SetDefaults(app_config_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  memset(cfg, 0, sizeof(*cfg));
  cfg->initial_send_delay_sec = 300UL;
  cfg->normal_send_interval_sec = 3600UL;
  cfg->burst_duration_ms = 15000UL;
  cfg->burst_period_ms = 50UL;
  cfg->lte_power_on_delay_ms = 15000UL;
  cfg->fast_boot_send_count = 2UL;
  cfg->fast_boot_gap_ms = 1000UL;
  cfg->lte_drain_ms = 5000UL;
  cfg->burst_connect_timeout_ms = 90000UL;
  cfg->device_id = 3000UL;
  app_config_copy_string(cfg->mqtt_host, APP_CONFIG_MQTT_HOST_LEN, "121.41.191.104");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg->mqtt_port, APP_CONFIG_MQTT_PORT_LEN, "1883");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg->mqtt_user, APP_CONFIG_MQTT_USER_LEN, "sersor");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg->mqtt_pass, APP_CONFIG_MQTT_PASS_LEN, "M4jYAf6TmF0R9T9d");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg->mqtt_client_id, APP_CONFIG_MQTT_CLIENT_ID_LEN, "ICM42688-4GV3.3.2-3000");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg->mqtt_pub_topic, APP_CONFIG_MQTT_TOPIC_LEN, "/device/dai/3000/thing/up_row");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
}

#if APP_CONFIG_FORCE_FLASH_DEFAULTS

/**
 * @brief 测试/量产辅助函数：上电时强制把写死参数覆盖到 Flash。
 */
void AppConfig_ForceWriteFlashDefaults(void)
{
  app_config_t cfg;

  memset(&cfg, 0, sizeof(cfg));

  cfg.initial_send_delay_sec = 30UL;
  cfg.normal_send_interval_sec = 60UL;
  cfg.burst_duration_ms = 15000UL;
  cfg.burst_period_ms = 50UL;
  cfg.lte_power_on_delay_ms = 15000UL;
  cfg.fast_boot_send_count = 2UL;
  cfg.fast_boot_gap_ms = 1000UL;
  cfg.lte_drain_ms = 5000UL;
  cfg.burst_connect_timeout_ms = 90000UL;
  cfg.device_id = 3000UL;
  app_config_copy_string(cfg.mqtt_host, APP_CONFIG_MQTT_HOST_LEN, "121.41.191.104");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg.mqtt_port, APP_CONFIG_MQTT_PORT_LEN, "1883");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg.mqtt_user, APP_CONFIG_MQTT_USER_LEN, "sersor");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg.mqtt_pass, APP_CONFIG_MQTT_PASS_LEN, "M4jYAf6TmF0R9T9d");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg.mqtt_client_id, APP_CONFIG_MQTT_CLIENT_ID_LEN, "ICM42688-4GV3.3.2-3000");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */
  app_config_copy_string(cfg.mqtt_pub_topic, APP_CONFIG_MQTT_TOPIC_LEN, "/device/dai/3000/thing/up_row");  /* 安全复制配置字符串，保证目标缓冲区以 0 结尾。 */

  AppConfig_Set(&cfg);  /* 用外部传入的配置覆盖当前 RAM 配置。 */
  (void)AppConfig_Save();  /* 擦除最后一页 Flash，并把当前 RAM 配置带校验头写入 Flash。 */
}
#endif

/**
 * @brief 校验 Flash 参数记录的魔数、版本、长度和 CRC。
 * @param record Flash 参数记录指针。
 */
static uint8_t app_config_record_valid(const app_config_record_t *record)
{
  uint32_t crc;

  if (record == NULL)
  {
    return 0U;
  }

  if ((record->magic != APP_CONFIG_MAGIC) ||
      (record->version != APP_CONFIG_VERSION) ||
      (record->size != sizeof(app_config_t)))
  {
    return 0U;
  }

  crc = app_config_crc32((const uint8_t *)&record->cfg, sizeof(record->cfg));  /* 计算配置结构体的 CRC32，用于判断 Flash 里的参数是否可靠。 */
  if (crc != record->crc)
  {
    return 0U;
  }

  return 1U;
}

/**
 * @brief 初始化运行参数：Flash 校验通过则读取 Flash，否则使用默认参数。
 */
void AppConfig_Init(void)
{
  const app_config_record_t *record = (const app_config_record_t *)APP_CONFIG_FLASH_ADDR;

  if (app_config_record_valid(record))
  {
    g_app_config = record->cfg;
  }
  else
  {
    AppConfig_SetDefaults(&g_app_config);  /* 填充一套出厂默认参数，仅写入 RAM 中传入的结构体。 */
  }
}

const app_config_t *AppConfig_Get(void)
{
  return &g_app_config;
}

/**
 * @brief 把当前 RAM 配置恢复为默认值，不主动写 Flash。
 */
void AppConfig_ResetRuntimeToDefaults(void)
{
  AppConfig_SetDefaults(&g_app_config);  /* 填充一套出厂默认参数，仅写入 RAM 中传入的结构体。 */
}

/**
 * @brief 用外部传入的配置覆盖当前 RAM 配置。
 * @param cfg 应用配置结构体指针。
 */
void AppConfig_Set(const app_config_t *cfg)
{
  if (cfg != NULL)
  {
    g_app_config = *cfg;
  }
}

/**
 * @brief 擦除最后一页 Flash，并把当前 RAM 配置带校验头写入 Flash。
 */
uint8_t AppConfig_Save(void)
{
  app_config_record_t record;
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0U;
  uint32_t page = (APP_CONFIG_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
  const uint8_t *src;
  uint32_t addr;
  uint32_t total;
  HAL_StatusTypeDef status;

  memset(&record, 0xFF, sizeof(record));
  record.magic = APP_CONFIG_MAGIC;
  record.version = APP_CONFIG_VERSION;
  record.size = sizeof(app_config_t);
  record.cfg = g_app_config;
  record.crc = app_config_crc32((const uint8_t *)&record.cfg, sizeof(record.cfg));

  status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
  {
    return 0U;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_PGSERR | FLASH_FLAG_MISERR | FLASH_FLAG_FASTERR | FLASH_FLAG_OPTVERR);

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = page;
  erase.NbPages = 1U;

  status = HAL_FLASHEx_Erase(&erase, &page_error);
  if (status == HAL_OK)
  {
    src = (const uint8_t *)&record;
    addr = APP_CONFIG_FLASH_ADDR;
    total = (sizeof(record) + 7U) & ~7UL;

    for (uint32_t offset = 0U; offset < total; offset += 8U)
    {
      uint64_t data = 0xFFFFFFFFFFFFFFFFULL;
      uint32_t copy_len = sizeof(record) - offset;

      if (copy_len > 8U)
      {
        copy_len = 8U;
      }

      if (offset < sizeof(record))
      {
        memcpy(&data, &src[offset], copy_len);
      }

      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + offset, data);
      if (status != HAL_OK)
      {
        break;
      }
    }
  }

  (void)HAL_FLASH_Lock();
  return (status == HAL_OK) ? 1U : 0U;
}
