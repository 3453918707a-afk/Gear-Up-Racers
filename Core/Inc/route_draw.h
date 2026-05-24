/*
 * route_draw.h
 *
 *  Created on: May 23, 2026
 *      Author: J2005
 */

#ifndef __ROUTE_DRAW_H__
#define __ROUTE_DRAW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 路线绘制模块初始化
void Route_Draw_Init(void);

// 路线绘制任务（在while循环中调用）
void Route_Draw_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_DRAW_H__ */
