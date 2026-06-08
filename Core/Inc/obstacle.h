/*
 * obstacle.h
 *
 *  Created on: May 23, 2026
 *      Author: J2005
 */

#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 系统工作状态枚举 */
typedef enum {
    CAR_MODE_NORMAL_TRACKING = 0,
    CAR_MODE_RADAR_ON_TRACKING,
    CAR_MODE_AVOIDING
} CarState_t;

extern volatile CarState_t g_car_state;
extern volatile uint8_t g_avoidance_done;

void Obstacle_Init(void);
void Obstacle_ActivateRadar(void);
void Obstacle_ManagerLoop(void);
int8_t Obstacle_CompareAvgDist(void);
int8_t Obstacle_GetPendingAvoidDir(void);

#ifdef __cplusplus
}
#endif

#endif /* __OBSTACLE_H__ */
