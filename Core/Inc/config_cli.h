#ifndef __CONFIG_CLI_H
#define __CONFIG_CLI_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_CLI_EVENT_NONE           0x00U
#define CONFIG_CLI_EVENT_ACTIVITY       0x01U
#define CONFIG_CLI_EVENT_TIMING_SAVED   0x02U
#define CONFIG_CLI_EVENT_EXIT           0x04U

void ConfigCli_Init(void);  /* 进入配置模式时初始化 CLI 工作副本和 LPUART2 接收。 */
void ConfigCli_DeInit(void);  /* 退出配置模式时清理 CLI 接收状态。 */
void ConfigCli_UartRxCpltCallback(void);  /* LPUART2 单字节接收完成回调，组装一行命令。 */
void ConfigCli_UartErrorCallback(void);  /* LPUART2 接收错误回调，重新启动接收以恢复通信。 */
uint8_t ConfigCli_Task(void);  /* 配置 CLI 轮询任务，处理已接收完整命令并返回事件。 */

#ifdef __cplusplus
}
#endif

#endif
