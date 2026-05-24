/*
 * lora.c
 *
 *  Created on: May 23, 2026
 *      Author: J2005
 */
#include "lora.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;  // 需要在 CubeMX 中配置并生成好 USART3 句柄

/**
 * @brief  初始化 LoRa 模块
 */
void LoRa_Init(void)
{
    HAL_Delay(200);   // 等待模块上电稳定
}

/**
 * @brief  定向发送固定帧字符串
 * @param  str 发送的 ASCII 字符串
 */
void LoRa_SendFixed(char *str)
{
    static uint8_t tx_buf[128];
    uint16_t len;

    /* 上一次发送未完成则跳过，避免覆盖 DMA/IT 缓冲区导致数据错乱 */
    if (huart3.gState != HAL_UART_STATE_READY) return;

    len = strlen(str);
    tx_buf[0] = TARGET_ADDR_H;
    tx_buf[1] = TARGET_ADDR_L;
    tx_buf[2] = TARGET_CHANNEL;
    if (len > sizeof(tx_buf) - 3) len = sizeof(tx_buf) - 3;
    memcpy(tx_buf + 3, str, len);

    /* 非阻塞中断发送：不冻结主循环，小车循迹不受影响 */
    HAL_UART_Transmit_IT(&huart3, tx_buf, 3 + len);
}

