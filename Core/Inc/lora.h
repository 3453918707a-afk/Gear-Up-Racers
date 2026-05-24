/*
 * lora.h
 *
 *  Created on: May 23, 2026
 *      Author: J2005
 */

#ifndef __LORA_H
#define __LORA_H

#include "main.h"

// 目标定向传输的配置信息 [cite: 1]
#define TARGET_ADDR_H    0x00
#define TARGET_ADDR_L    0x02
#define TARGET_CHANNEL   0x14   // 信道 20 (0x14)

void LoRa_Init(void);
void LoRa_SendFixed(char *str);   // 发送定向固定帧数据

#endif /* __LORA_H */
