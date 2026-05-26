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
    // -----------------------------------------------------------------
    // 算法核心：实时计算左右雷达的指数移动平均距离（EMA滤波），消除雷达数据抖动
    // -----------------------------------------------------------------
    static float avg_left_dist = 200.0f;
    static float avg_right_dist = 200.0f;

    // 当雷达测距返回0时，代表前方极为开阔（超量程），我们赋予它一个大安全基准值（如500cm）
    float cur_left  = (float)radar1.data.target_dist_cm;
    float cur_right = (float)radar2.data.target_dist_cm;

    // 经典 EMA 滤波公式（80%权重保留历史，20%听从当前，滤掉瞬间噪声）
    avg_left_dist  = avg_left_dist * 0.8f + cur_left * 0.2f;
    avg_right_dist = avg_right_dist * 0.8f + cur_right * 0.2f;
    // -----------------------------------------------------------------

    switch (g_car_state)
    {
        case CAR_MODE_NORMAL_TRACKING:
            Track_Control();
            break;

        case CAR_MODE_RADAR_ON_TRACKING:
            // 优先拦截检测：看是否正好走到了 特异性丁字路口
            if (Track_IsSpecificTRoad()&& (HAL_GetTick() - junction_cooldown_tick > 2000))
            {
                // 满足条件，立刻接管底盘，切入强制转向避障状态
                g_car_state = CAR_MODE_AVOIDING;
                avoidance_start_tick = HAL_GetTick();

                // 核心决策：向平均距离较大的方向（即阻碍物更远、空间更开阔的方向）转弯
                if (avg_left_dist > avg_right_dist)
                {
                    g_avoid_dir = -1; // 左边更空旷，决定向左转
                }
                else
                {
                    g_avoid_dir = 1;  // 右边更空旷，决定向右转
                }
            }
            // 如果不是特殊路口，检测是否有突发的近场障碍物（保留原有的近场刹车保护）
            else if ((radar1.data.target_dist_cm > 0 && radar1.data.target_dist_cm <= RADAR_DIST_THRESHOLD_CM) ||
                     (radar2.data.target_dist_cm > 0 && radar2.data.target_dist_cm <= RADAR_DIST_THRESHOLD_CM))
            {
                g_car_state = CAR_MODE_AVOIDING;
                avoidance_start_tick = HAL_GetTick();
                // 突发近场避障时，谁近就往相反方向闪避
                g_avoid_dir = (radar1.data.target_dist_cm <= radar2.data.target_dist_cm) ? 1 : -1;
            }
            else
            {
                // 没有任何异常路口和障碍物，常规执行循迹
                Track_Control();
            }
            break;

        case CAR_MODE_AVOIDING:
                    // 执行固定时长的差速原地旋转
                    if (HAL_GetTick() - avoidance_start_tick < AVOIDING_DURATION_MS)
                    {
                        if (g_avoid_dir == 1)
                        {
                            // 🔄 【已修正极性】：原本是向右急转弯
                            // 严格匹配你底盘“负数为前、正数为后”的特殊极性
                            Motor_SetBoth(BASE_AVOID_SPEED, -BASE_AVOID_SPEED);
                        }
                        else if (g_avoid_dir == -1)
                        {
                            // 🔄 【已修正极性】：原本是向左急转弯
                            Motor_SetBoth(-BASE_AVOID_SPEED, BASE_AVOID_SPEED);
                        }
                    }
                    else
                    {
                        // 转向时间到，完成转弯，重新回退到带雷达的常规循迹状态
                        g_car_state = CAR_MODE_RADAR_ON_TRACKING;
                        g_avoid_dir = 0;
                        junction_cooldown_tick = HAL_GetTick();
                    }
                    break;
    }
}
