/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    track.h
  * @brief   This file contains all the function prototypes for
  * the track.c file (PID and Motor control)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TRACK_H__
#define __TRACK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* PID结构体 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Error;
    float LastError;
    float Integral;
    float Output;
} PID_TypeDef;

/* 公共函数接口 */
void Track_Init(void);
void Track_Control(void);
void Motor_SetBoth(int left, int right);
void Motor_Stop(void);

int Track_GetLastError(void);
int Track_IsSpecificTRoad(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRACK_H__ */
