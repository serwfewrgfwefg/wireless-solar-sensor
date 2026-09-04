#ifndef __APP_H
#define __APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void App_Run(void);  /* 应用层主入口：完成模块初始化、上电首发、周期休眠和周期突发发送。 */
void App_RtcWakeupCallback(RTC_HandleTypeDef *rtc);  /* 应用层 RTC 唤醒回调：记录本次 STOP2 是由 RTC 定时唤醒。 */
void App_ConfigWakeupCallback(uint16_t GPIO_Pin);  /* 应用层配置唤醒回调：记录 Type-C/USB 外部中断请求进入参数配置模式。 */

#ifdef __cplusplus
}
#endif

#endif
