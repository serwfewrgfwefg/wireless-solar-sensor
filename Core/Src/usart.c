/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "lte.h"
#include "config_cli.h"

static uint8_t g_uart2_rx_byte;  /* USART2 单字节接收缓存，供 4G AT 返回中断使用。 */

/**
 * @brief 重新启动 USART2 单字节中断接收，用于 4G AT 返回数据。
 */
static void USART2_RestartRxIT(void)
{
    if (huart2.Instance == USART2)
    {
        (void)HAL_UART_Receive_IT(&huart2, &g_uart2_rx_byte, 1);
    }
}

#if 1
#if (__ARMCC_VERSION >= 6010050)
__asm(".global __use_no_semihosting\n\t");
__asm(".global __ARM_use_no_argv \n\t");
#else
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};
#endif

/**
 * @brief Keil 非半主机运行时输出单字符钩子。
 * @param ch 单个输出字符。
 */
int _ttywrch(int ch)
{
    return ch;
}

/**
 * @brief Keil 非半主机退出钩子，避免 printf 依赖半主机。
 * @param x 退出码。
 */
void _sys_exit(int x)
{
    (void)x;
}

char *_sys_command_string(char *cmd, int len)
{
    (void)cmd;
    (void)len;
    return NULL;
}

FILE __stdout;

/**
 * @brief 把 printf 输出重定向到 USART1 调试串口。
 * @param ch 单个输出字符。
 * @param f 标准库 FILE 指针。
 */
int fputc(int ch, FILE *f)
{
    uint8_t data = (uint8_t)ch;
    (void)f;
    HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
    return ch;
}
#endif
/* USER CODE END 0 */

UART_HandleTypeDef hlpuart2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/**
 * @brief 初始化 LPUART2 为参数配置串口，默认 115200 8N1。
 */
void MX_LPUART2_UART_Init(void)
{
  hlpuart2.Instance = LPUART2;
  hlpuart2.Init.BaudRate = 115200;
  hlpuart2.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart2.Init.StopBits = UART_STOPBITS_1;
  hlpuart2.Init.Parity = UART_PARITY_NONE;
  hlpuart2.Init.Mode = UART_MODE_TX_RX;
  hlpuart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart2.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart2) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart2) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
}

/**
 * @brief 初始化 USART1 调试串口，供 printf 使用。
 */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
}

/**
 * @brief 初始化 USART2 为 4G 模块 AT 通信串口，并启动接收中断。
 */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
  }

  HAL_NVIC_SetPriority(USART2_LPUART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_LPUART2_IRQn);
  USART2_RestartRxIT();  /* 重新启动 USART2 单字节中断接收，用于 4G AT 返回数据。 */
}

/**
 * @brief UART 底层 MSP 初始化：根据实例配置对应时钟和 GPIO。
 * @param uartHandle UART 句柄。
 */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if(uartHandle->Instance==LPUART2)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART2;
    PeriphClkInit.Lpuart2ClockSelection = RCC_LPUART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
    }

    __HAL_RCC_LPUART2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_LPUART2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(uartHandle->Instance==USART1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(uartHandle->Instance==USART2)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */
    }

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/**
 * @brief UART 底层 MSP 反初始化：关闭时钟并释放 GPIO/中断。
 * @param uartHandle UART 句柄。
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==LPUART2)
  {
    __HAL_RCC_LPUART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
  }
  else if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(USART2_LPUART2_IRQn);
  }
}

/**
 * @brief HAL 串口接收完成回调，把字节分发给 LTE 或配置 CLI。
 * @param huart UART 句柄。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        lte_uart_rx_byte(g_uart2_rx_byte);
        USART2_RestartRxIT();  /* 重新启动 USART2 单字节中断接收，用于 4G AT 返回数据。 */
    }
    else if (huart->Instance == LPUART2)
    {
        ConfigCli_UartRxCpltCallback();  /* LPUART2 单字节接收完成回调，组装一行命令。 */
    }
}

/**
 * @brief HAL 串口错误回调，重新启动对应串口接收。
 * @param huart UART 句柄。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        USART2_RestartRxIT();  /* 重新启动 USART2 单字节中断接收，用于 4G AT 返回数据。 */
    }
    else if (huart->Instance == LPUART2)
    {
        ConfigCli_UartErrorCallback();  /* LPUART2 接收错误回调，重新启动接收以恢复通信。 */
    }
}
