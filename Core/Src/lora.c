#include "lora.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

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
    uint8_t header[3] = {
        TARGET_ADDR_H,
        TARGET_ADDR_L,
        TARGET_CHANNEL
    };

    // 1. 发送定向传输所需的 3 字节帧头
    HAL_UART_Transmit(&huart3, header, 3, 1000);

    // 2. 发送真实的业务报文数据
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 1000);
}
