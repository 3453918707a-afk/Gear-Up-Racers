/*
 * obstacle.c
 *
 * Created on: May 23, 2026
 * Author: J2005
 */

#include "obstacle.h"
#include "hlk_ld2410s.h"
#include "track.h"

/* 全局工作状态变量 */
volatile CarState_t g_car_state = CAR_MODE_NORMAL_TRACKING;
static uint32_t avoidance_start_tick = 0;
static uint32_t junction_cooldown_tick = 0;
/* 避障核心参数宏定义 */
#define AVOIDING_DURATION_MS    1200  // 转向持续时间（毫秒），转弯90度后自动切回循迹
#define RADAR_DIST_THRESHOLD_CM 40    // 常规近场刹车避障阈值（保持原有安全机制）
#define BASE_AVOID_SPEED        450   // 避障转向时的差速电机速度

// 记录转向方向：1 为向右转，-1 为向左转
static int g_avoid_dir = 0;

void Obstacle_Init(void)
{
    g_car_state = CAR_MODE_NORMAL_TRACKING;
    avoidance_start_tick = 0;
    g_avoid_dir = 0;
}

void Obstacle_ActivateRadar(void)
{
    if (g_car_state == CAR_MODE_NORMAL_TRACKING)
    {
        g_car_state = CAR_MODE_RADAR_ON_TRACKING;
    }
}

void Obstacle_ManagerLoop(void)
{
    static float avg_left_dist  = 200.0f;
    static float avg_right_dist = 200.0f;

    float cur_left  = (float)radar1.data.target_dist_cm;
    float cur_right = (float)radar2.data.target_dist_cm;

    /* 雷达超量程时返回 0，映射为大安全值，防止 EMA 被错误拉向 0 而误触发避障 */
    if (cur_left  == 0.0f) cur_left  = 200.0f;
    if (cur_right == 0.0f) cur_right = 200.0f;

    /* 避障旋转期间暂停 EMA 更新，防止车体旋转时雷达对墙/对空交替污染滤波器 */
    if (g_car_state != CAR_MODE_AVOIDING) {
        avg_left_dist  = avg_left_dist * 0.8f + cur_left * 0.2f;
        avg_right_dist = avg_right_dist * 0.8f + cur_right * 0.2f;
    }

    switch (g_car_state)
    {
        case CAR_MODE_NORMAL_TRACKING:
            Track_Control();
            break;

        case CAR_MODE_RADAR_ON_TRACKING:
            if (Track_IsSpecificTRoad() && (HAL_GetTick() - junction_cooldown_tick > 2000))
            {
                g_car_state = CAR_MODE_AVOIDING;
                avoidance_start_tick = HAL_GetTick();

                if (avg_left_dist > avg_right_dist) {
                    g_avoid_dir = -1;
                } else {
                    g_avoid_dir = 1;
                }
            }
            else if ((radar1.data.target_dist_cm > 0 && radar1.data.target_dist_cm <= RADAR_DIST_THRESHOLD_CM) ||
                     (radar2.data.target_dist_cm > 0 && radar2.data.target_dist_cm <= RADAR_DIST_THRESHOLD_CM))
            {
                g_car_state = CAR_MODE_AVOIDING;
                avoidance_start_tick = HAL_GetTick();
                /* 方向判断改用 EMA 滤波值，消除瞬时毛刺导致的方向误判 */
                g_avoid_dir = (avg_left_dist <= avg_right_dist) ? 1 : -1;
            }
            else
            {
                Track_Control();
            }
            break;

        case CAR_MODE_AVOIDING:
            if (HAL_GetTick() - avoidance_start_tick >= AVOIDING_DURATION_MS)
            {
                g_car_state = CAR_MODE_RADAR_ON_TRACKING;
                g_avoid_dir = 0;
                junction_cooldown_tick = HAL_GetTick();
            }
            else
            {
                if (g_avoid_dir == 1) {
                    Motor_SetBoth(BASE_AVOID_SPEED, -BASE_AVOID_SPEED);
                } else if (g_avoid_dir == -1) {
                    Motor_SetBoth(-BASE_AVOID_SPEED, BASE_AVOID_SPEED);
                }

                /* 旋转 400ms 后开始检测黑线是否已恢复，提前结束多余旋转 */
                if (HAL_GetTick() - avoidance_start_tick > 400) {
                    if (Track_IsLineRecovered()) {
                        g_car_state = CAR_MODE_RADAR_ON_TRACKING;
                        g_avoid_dir = 0;
                        junction_cooldown_tick = HAL_GetTick();
                    }
                }
            }
            break;
    }
}
