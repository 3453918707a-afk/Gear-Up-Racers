/*
 * us100.h
 *
 *  Created on: May 17, 2026
 *      Author: J2005
 */
#ifndef __US100_H
#define __US100_H

#include "main.h"

// 外部可调用的时间全局变量（由内部 TIM3 累计更新）
extern volatile uint8_t  hour;
extern volatile uint8_t  minute;
extern volatile uint8_t  second;
extern volatile uint16_t elapsed_min;
extern volatile uint8_t  elapsed_sec;

// 超声波最终测量结果 (单位: cm)
extern volatile float    distance_cm;
extern volatile uint32_t hcsr04_sample_seq;

/* 外部公共调用接口 */
void HCSR04_Init(void);
void HCSR04_Task(void);
void HCSR04_TIM_Callback(TIM_HandleTypeDef *htim);
void HCSR04_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* __US100_H */
