/*
 * route_draw.c
 *
 *  Created on: May 23, 2026
 *      Author: J2005
 */

#include "route_draw.h"
#include "lcd.h"
#include "track.h"

static int current_x = 160;     // 起点X：屏幕水平正中间 (320宽)
static int current_y = 230;     // 起点Y：屏幕最底部上方一点 (240高)
static uint32_t last_draw_tick = 0;

void Route_Draw_Init(void)
{
    current_x = 160;
    current_y = 230;

    // 初始化时清空屏幕为黑色
    LCD_FillRect(0, 0, 320, 240, BLACK);

    // 打印当前的模式提示
    LCD_ShowString(10, 10, "Mode: Normal Tracking", GREEN, BLACK);
    LCD_ShowString(10, 26, "Drawing Route...", CYAN, BLACK);
}

void Route_Draw_Task(void)
{
    // 控制绘图的刷新率，每 50ms 取样并画一次线，防止画得太快
    if (HAL_GetTick() - last_draw_tick < 50) {
        return;
    }
    last_draw_tick = HAL_GetTick();

    // 1. 读取循迹传感器的当前误差
    int error = Track_GetLastError();

    // 2. 根据误差计算下一个点的坐标
    // 误差 error 的范围大概在 -9 到 +9 之间，乘以一个系数(比如2)来放大转弯的视觉效果
    int next_x = current_x + (error * 2);
    // 小车默认是一直向前开的，所以Y坐标不断减少（向上画）
    int next_y = current_y - 2;

    // 3. 边界检测：如果画到了屏幕最上方或者左右边缘，重置画布
    if (next_x <= 0 || next_x >= 319 || next_y <= 45)
    {
        Route_Draw_Init();
        return;
    }

    // 4. 在屏幕上将上一个点到这一个点连线 (为了醒目，用红色，并画两根线加粗)
    LCD_DrawLine(current_x, current_y, next_x, next_y, RED);
    LCD_DrawLine(current_x + 1, current_y, next_x + 1, next_y, RED);

    // 5. 更新当前坐标
    current_x = next_x;
    current_y = next_y;
}
