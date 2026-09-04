#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CONFIG_MQTT_HOST_LEN       64U  /* MQTT 服务器地址字符串最大长度。 */
#define APP_CONFIG_MQTT_PORT_LEN       8U  /* MQTT 端口字符串最大长度。 */
#define APP_CONFIG_MQTT_USER_LEN       32U  /* MQTT 用户名字符串最大长度。 */
#define APP_CONFIG_MQTT_PASS_LEN       64U  /* MQTT 密码字符串最大长度。 */
#define APP_CONFIG_MQTT_CLIENT_ID_LEN  48U  /* MQTT ClientID 字符串最大长度。 */
#define APP_CONFIG_MQTT_TOPIC_LEN      96U  /* MQTT 发布主题字符串最大长度。 */

#define APP_CONFIG_FORCE_FLASH_DEFAULTS 0U  /* 强制覆盖 Flash 默认参数的编译开关，1 打开，0 关闭。 */

typedef struct
{
  uint32_t initial_send_delay_sec;  /* 上电后首次发送前等待时间，单位秒。 */
  uint32_t normal_send_interval_sec;  /* 正常周期发送间隔，单位秒。 */
  uint32_t burst_duration_ms;  /* 每次周期突发发送持续时间，单位毫秒。 */
  uint32_t burst_period_ms;  /* 突发发送窗口内每帧间隔，单位毫秒。 */
  uint32_t lte_power_on_delay_ms;  /* 4G 模块上电后等待启动时间，单位毫秒。 */
  uint32_t fast_boot_send_count;  /* 上电首发阶段快速发送帧数。 */
  uint32_t fast_boot_gap_ms;  /* 上电首发阶段两帧之间的间隔，单位毫秒。 */
  uint32_t lte_drain_ms;  /* 4G 发送结束后的收尾等待时间，单位毫秒。 */
  uint32_t burst_connect_timeout_ms;  /* 4G/MQTT 连接最大等待时间，单位毫秒。 */
  uint32_t device_id;  /* 上传协议帧中的设备 ID。 */
  char mqtt_host[APP_CONFIG_MQTT_HOST_LEN];  /* MQTT 服务器地址。 */
  char mqtt_port[APP_CONFIG_MQTT_PORT_LEN];  /* MQTT 服务器端口。 */
  char mqtt_user[APP_CONFIG_MQTT_USER_LEN];  /* MQTT 用户名。 */
  char mqtt_pass[APP_CONFIG_MQTT_PASS_LEN];  /* MQTT 密码。 */
  char mqtt_client_id[APP_CONFIG_MQTT_CLIENT_ID_LEN];  /* MQTT ClientID。 */
  char mqtt_pub_topic[APP_CONFIG_MQTT_TOPIC_LEN];  /* MQTT 数据发布主题。 */
} app_config_t;

void AppConfig_Init(void);  /* 初始化运行参数：Flash 校验通过则读取 Flash，否则使用默认参数。 */
#if APP_CONFIG_FORCE_FLASH_DEFAULTS
void AppConfig_ForceWriteFlashDefaults(void);  /* 测试/量产辅助函数：上电时强制把写死参数覆盖到 Flash。 */
#endif
const app_config_t *AppConfig_Get(void);  /* 获取当前运行中的全局配置，只读返回给业务和驱动使用。 */
void AppConfig_SetDefaults(app_config_t *cfg);  /* 填充一套出厂默认参数，仅写入 RAM 中传入的结构体。 */
void AppConfig_ResetRuntimeToDefaults(void);  /* 把当前 RAM 配置恢复为默认值，不主动写 Flash。 */
void AppConfig_Set(const app_config_t *cfg);  /* 用外部传入的配置覆盖当前 RAM 配置。 */
uint8_t AppConfig_Save(void);  /* 擦除最后一页 Flash，并把当前 RAM 配置带校验头写入 Flash。 */

#ifdef __cplusplus
}
#endif

#endif
