#include "track.h"
#include "i2c.h"
#include "tim.h"
#include "obstacle.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define SENSOR_I2C_ADDR        (0x5D << 1)
#define SENSOR_DATA_REG        0x05

#define PWM_MAX                999
#define BASE_SPEED             272
#define TURN_SPEED             355
#define LOST_TURN_SPEED        314

#define LEFT_MOTOR_REVERSE     0
#define RIGHT_MOTOR_REVERSE    0

#define FRONT_MASK             0x18
#define LEFT_BRANCH_MASK       0x07
#define RIGHT_BRANCH_MASK      0xE0

#define RIGHT_TURN_MAX_TIME    900    //900
#define RIGHT_BRANCH_EXTRA_TIME   350
#define TURN_SEARCH_SPEED      263
#define TURN_COOLDOWN_TIME     650

#define SOFT_KP_DURATION_MS      1000
#define SOFT_KP_VALUE             10.0f
#define SOFT_KD_VALUE             6.0f
#define NORMAL_KP_VALUE           20.0f
#define NORMAL_KD_VALUE           22.0f
#define SOFT_BASE_SPEED           244
#define NORMAL_BASE_SPEED         272

#define STRAIGHT_BLIND_DURATION_MS  300
#define STRAIGHT_BLIND_SPEED        263
#define RADAR_TURN_INTERVAL_MS      15000U
#define STUCK_DETECT_MS             2000U
#define STUCK_BOOST_MS              55U
#define STUCK_BOOST_SPEED           780
#define STUCK_BOOST_COOLDOWN_MS     1500U

/* 非阻塞右转状态机枚举 ------------------------------------------------------*/
typedef enum {
    TURN_IDLE = 0,
    TURN_SPINNING,
    TURN_STRAIGHT_BLIND,
    TURN_POST_STOP
} TurnState_t;

typedef enum {
    RADAR_WAIT_30S = 0,
    RADAR_WAIT_10S_STRAIGHT,
    RADAR_WAIT_2_TURNS,
    RADAR_READY
} RadarActState_t;

static RadarActState_t g_radar_act_state = RADAR_WAIT_30S;
static uint32_t g_straight_start_tick = 0;
static uint8_t g_hard_turn_count = 0;
static uint8_t g_was_spinning = 0;
static uint32_t g_last_hard_turn_tick = 0;

/* 私有静态变量 --------------------------------------------------------------*/
/* P微降抑制过冲，D适度增大增强阻尼 */
static PID_TypeDef LinePID = {
    20.0f, 0.0f, 27.0f,
    0.0f, 0.0f, 0.0f, 0.0f
};
//30 0 27
static int g_last_error = 0;
static int g_last_turn_dir = 1;
static uint32_t g_turn_cooldown_until = 0;
static uint32_t g_straight_blind_cooldown = 0;
static uint8_t g_cached_raw = 0;
static uint32_t g_turn_deadline_tick = 0;

static TurnState_t g_turn_state = TURN_IDLE;
static uint32_t g_turn_start_tick = 0;
static uint8_t g_turn_left_old_line = 0;
static uint8_t g_turn_center_count = 0;
static int g_turn_spin_dir = -1;
static uint32_t g_soft_kp_until = 0;
static uint8_t  g_last_sensor_stuck = 0xFF;
static uint32_t g_sensor_change_tick = 0;
static uint8_t g_last_rescue_raw = 0xFF;
static uint32_t g_rescue_change_tick = 0;
static uint32_t g_rescue_boost_until = 0;
static uint32_t g_rescue_cooldown_until = 0;
/* 甩尾过滤：滑动窗口绝对误差和 + 过零率检测 */
#define HISTORY_LEN         10
#define STABILITY_THRESHOLD 15.0f
#define ZC_WINDOW           10
#define MAX_ALLOWED_ZC      2
#define ZC_DEADBAND         0.5f

static float   g_error_history[HISTORY_LEN] = {0};
static int8_t  g_sign_history[ZC_WINDOW]    = {0};

/* 私有函数实现 --------------------------------------------------------------*/
static int Count_Bits(uint8_t value)
{
    int count = 0;
    while (value) {
        if (value & 0x01) count++;
        value >>= 1;
    }
    return count;
}

static int Has_Front_Line(uint8_t raw) {
    return (raw & FRONT_MASK) != 0;
}

static int Has_Left_Branch(uint8_t raw) {
    return ((raw & 0x01) != 0) && (Count_Bits(raw & LEFT_BRANCH_MASK) >= 2);
}

static int Has_Right_Branch(uint8_t raw) {
    return ((raw & 0x80) != 0) && (Count_Bits(raw & RIGHT_BRANCH_MASK) >= 2);
}

static void PID_Reset(PID_TypeDef *pid)
{
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->Integral = 0.0f;
    pid->Output = 0.0f;
}

/* ---------- 甩尾过滤：滑动窗口绝对误差和 ---------- */
static void Update_Error_History(float current_error)
{
    for (int i = 0; i < HISTORY_LEN - 1; i++) {
        g_error_history[i] = g_error_history[i + 1];
    }
    g_error_history[HISTORY_LEN - 1] = (current_error > 0.0f) ? current_error : -current_error;
}

static uint8_t Is_Car_Stable(void)
{
    float sum = 0.0f;
    for (int i = 0; i < HISTORY_LEN; i++) {
        sum += g_error_history[i];
    }
    return (sum < STABILITY_THRESHOLD) ? 1 : 0;
}

/* ---------- 甩尾过滤：过零率检测 ---------- */
static void Update_Sign_History(float current_error)
{
    for (int i = 0; i < ZC_WINDOW - 1; i++) {
        g_sign_history[i] = g_sign_history[i + 1];
    }
    if (current_error > ZC_DEADBAND) {
        g_sign_history[ZC_WINDOW - 1] = 1;
    } else if (current_error < -ZC_DEADBAND) {
        g_sign_history[ZC_WINDOW - 1] = -1;
    } else {
        g_sign_history[ZC_WINDOW - 1] = 0;
    }
}

static uint8_t Check_Zero_Crossing_Stable(void)
{
    uint8_t zc_count = 0;
    for (int i = 0; i < ZC_WINDOW - 1; i++) {
        if (g_sign_history[i] * g_sign_history[i + 1] < 0) {
            zc_count++;
        }
    }
    return (zc_count <= MAX_ALLOWED_ZC) ? 1 : 0;
}

static int Limit_Int(int value, int min, int max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static void Motor_SetLeft(int speed)
{
#if LEFT_MOTOR_REVERSE
    speed = -speed;
#endif
    if (speed >= 0) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
        speed = -speed;
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Limit_Int(speed, 0, PWM_MAX));
}

static void Motor_SetRight(int speed)
{
#if RIGHT_MOTOR_REVERSE
    speed = -speed;
#endif
    if (speed >= 0) {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
        speed = -speed;
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Limit_Int(speed, 0, PWM_MAX));
}

/* 公有函数实现 --------------------------------------------------------------*/
void Motor_SetBoth(int left, int right)
{
    Motor_SetLeft(left);
    Motor_SetRight(right);
}

void Motor_Stop(void)
{
    Motor_SetBoth(0, 0);
}

static int Track_UpdateStuckRescue(uint8_t sensor_raw, uint8_t allow_zero)
{
    uint32_t now = HAL_GetTick();

    if (sensor_raw != g_last_rescue_raw) {
        g_last_rescue_raw = sensor_raw;
        g_rescue_change_tick = now;
        return 0;
    }

    if ((!allow_zero && sensor_raw == 0x00) || now < g_rescue_cooldown_until) {
        return 0;
    }

    if (now - g_rescue_change_tick < STUCK_DETECT_MS) {
        return 0;
    }

    g_rescue_boost_until = now + STUCK_BOOST_MS;
    g_rescue_cooldown_until = now + STUCK_BOOST_COOLDOWN_MS;
    g_rescue_change_tick = now;
    Motor_SetBoth(STUCK_BOOST_SPEED, STUCK_BOOST_SPEED);
    return 1;
}

static int Get_Line_Error(uint8_t raw)
{
	int weights[8] = {7, 5, 3, 1, -1, -3, -5, -7};
    int sum = 0;
    int count = 0;
    int i;
    int error;

    if (raw == 0x00) {
        if (g_last_error > 0) return 9;
        if (g_last_error < 0) return -9;
        return 0;
    }

    for (i = 0; i < 8; i++) {
        if (raw & (1 << i)) {
            sum += weights[i];
            count++;
        }
    }

    if (count == 0) {
        return g_last_error;
    }

    error = sum / count;
    g_last_error = error;

    if (error > 0) g_last_turn_dir = 1;
    if (error < 0) g_last_turn_dir = -1;

    return error;
}

static float PID_Compute(PID_TypeDef *pid, int current_err)
{
    pid->Error = (float)current_err;
    pid->Integral += pid->Error;

    if (pid->Integral > 80.0f) pid->Integral = 80.0f;
    if (pid->Integral < -80.0f) pid->Integral = -80.0f;

    pid->Output = pid->Kp * pid->Error
                + pid->Ki * pid->Integral
                + pid->Kd * (pid->Error - pid->LastError);

    pid->LastError = pid->Error;
    return pid->Output;
}

void Track_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    g_rescue_change_tick = HAL_GetTick();
    g_rescue_boost_until = 0;
    g_rescue_cooldown_until = 0;
    Motor_Stop();
}

void Track_ResetControlState(void)
{
    uint32_t now = HAL_GetTick();

    PID_Reset(&LinePID);
    LinePID.Kp = NORMAL_KP_VALUE;
    LinePID.Kd = NORMAL_KD_VALUE;

    g_last_error = 0;
    g_last_turn_dir = 1;
    g_turn_cooldown_until = now + TURN_COOLDOWN_TIME;
    g_straight_blind_cooldown = now + 2000U;
    g_cached_raw = 0;
    g_turn_deadline_tick = 0;
    g_turn_state = TURN_IDLE;
    g_turn_start_tick = 0;
    g_turn_left_old_line = 0;
    g_turn_center_count = 0;
    g_turn_spin_dir = -1;
    g_soft_kp_until = 0;
    g_last_rescue_raw = 0xFF;
    g_rescue_change_tick = now;
    g_rescue_boost_until = 0;
    g_rescue_cooldown_until = now + STUCK_BOOST_COOLDOWN_MS;

    for (int i = 0; i < HISTORY_LEN; i++) {
        g_error_history[i] = 0.0f;
    }

    for (int i = 0; i < ZC_WINDOW; i++) {
        g_sign_history[i] = 0;
    }
}

void Track_Control(void)
{
    Track_UpdateRadarActivation();

    /* 只要避障在停车观察，循迹系统彻底休眠（拉死手刹），防止滑出黑线后抢跑 */
    if (g_rescue_boost_until && HAL_GetTick() < g_rescue_boost_until) {
        Motor_SetBoth(STUCK_BOOST_SPEED, STUCK_BOOST_SPEED);
        return;
    }
    if (g_rescue_boost_until && HAL_GetTick() >= g_rescue_boost_until) {
        g_rescue_boost_until = 0;
    }

    /* 转弯完成1s后恢复Kp/Kd */
    if (g_soft_kp_until && HAL_GetTick() >= g_soft_kp_until) {
        LinePID.Kp = NORMAL_KP_VALUE;
        LinePID.Kd = NORMAL_KD_VALUE;
        g_soft_kp_until = 0;
    }

    uint8_t sensor_raw = 0;

    /* 8灯全亮直行  --- */
    if (g_turn_state == TURN_STRAIGHT_BLIND) {
        Motor_SetBoth(STRAIGHT_BLIND_SPEED, STRAIGHT_BLIND_SPEED);

        if ((HAL_GetTick() - g_turn_start_tick) >= STRAIGHT_BLIND_DURATION_MS) {
            uint8_t blind_raw = 0;
            if (HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG,
                                 I2C_MEMADD_SIZE_8BIT, &blind_raw, 1, 10) == HAL_OK
                && (blind_raw & FRONT_MASK) != 0) {
                /* 还有中间线 → 十字路口，退出直行 */
                g_turn_cooldown_until = HAL_GetTick() + TURN_COOLDOWN_TIME;
                g_turn_state = TURN_IDLE;
            } else {
                /* 中间没线 → T字路口，右转 */
                g_turn_spin_dir = -1;
                g_last_turn_dir = -1;
                g_turn_state = TURN_SPINNING;
                g_turn_start_tick = HAL_GetTick();
                g_turn_deadline_tick = g_turn_start_tick + RIGHT_TURN_MAX_TIME;
                g_turn_left_old_line = 0;
                g_turn_center_count = 0;
            }
        }
        return;
    }

    /* --- */
    if (g_turn_state == TURN_SPINNING) {
        if (HAL_GetTick() >= g_turn_deadline_tick) {
            g_last_turn_dir = g_turn_spin_dir;
            g_turn_cooldown_until = HAL_GetTick() + TURN_COOLDOWN_TIME;
            g_straight_blind_cooldown = HAL_GetTick() + 2000;
            g_soft_kp_until = HAL_GetTick() + SOFT_KP_DURATION_MS;
            LinePID.Kp = SOFT_KP_VALUE;
            LinePID.Kd = SOFT_KD_VALUE;
            g_turn_state = TURN_IDLE;
            return;
        }

        Motor_SetBoth(TURN_SEARCH_SPEED * g_turn_spin_dir,
                     -TURN_SEARCH_SPEED * g_turn_spin_dir);

        if (HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG,
                             I2C_MEMADD_SIZE_8BIT, &sensor_raw, 1, 10) == HAL_OK) {
            int front = Has_Front_Line(sensor_raw);
            int left  = Has_Left_Branch(sensor_raw);
            int right = Has_Right_Branch(sensor_raw);

            /* 右侧仍有分支：延长旋转截止时间，防止提前退出 */
            if (right && !left) {
                uint32_t extended_until = HAL_GetTick() + RIGHT_BRANCH_EXTRA_TIME;
                g_turn_spin_dir = -1;
                if (extended_until > g_turn_deadline_tick) {
                    g_turn_deadline_tick = extended_until;
                }
            }

            if (!front) {
                g_turn_left_old_line = 1;
                g_turn_center_count = 0;
            }

            if (g_turn_left_old_line && front) {
                g_turn_center_count++;
                if (g_turn_center_count >= 2) {
                    int exit_err = Get_Line_Error(sensor_raw);
                    LinePID.Error     = (float)exit_err;
                    LinePID.LastError = (float)exit_err;
                    g_turn_cooldown_until = HAL_GetTick() + TURN_COOLDOWN_TIME;
                    g_straight_blind_cooldown = HAL_GetTick() + 2000;
                    g_soft_kp_until = HAL_GetTick() + SOFT_KP_DURATION_MS;
                    LinePID.Kp = SOFT_KP_VALUE;
                    LinePID.Kd = SOFT_KD_VALUE;
                    g_turn_state = TURN_POST_STOP;
                    g_turn_start_tick = HAL_GetTick();
                    return;
                }
            } else if (g_turn_left_old_line) {
                g_turn_center_count = 0;
            }
        }
        return;
    }

    /* 正常循迹模式 */
    if (HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG,
                         I2C_MEMADD_SIZE_8BIT, &sensor_raw, 1, 50) == HAL_OK)
    {
        extern volatile CarState_t g_car_state;

        g_cached_raw = sensor_raw;
        uint8_t rescue_allow_zero = (g_radar_act_state == RADAR_WAIT_2_TURNS) ? 1U : 0U;
        if (g_car_state != CAR_MODE_RADAR_ON_TRACKING
            && Track_UpdateStuckRescue(sensor_raw, rescue_allow_zero)) {
            return;
        }

        int error = Get_Line_Error(sensor_raw);

        /* 每轮更新甩尾过滤历史特征 */
        Update_Error_History((float)error);
        Update_Sign_History((float)error);

        /* 路口检测（冷却期外）：严格互斥多分支，不依赖if-else隐式顺序 */
                if (HAL_GetTick() >= g_turn_cooldown_until) {
                    int front = Has_Front_Line(sensor_raw);
                    int left  = Has_Left_Branch(sensor_raw);
                    int right = Has_Right_Branch(sensor_raw);

                    if (Count_Bits(sensor_raw) >= 7 && HAL_GetTick() >= g_straight_blind_cooldown) {
                        /* 7灯全亮：强制直行0.3s */
                        PID_Reset(&LinePID);
                        g_turn_state = TURN_STRAIGHT_BLIND;
                        g_turn_start_tick = HAL_GetTick();
                    }
                    else if (front && !left && right) {
                        /* 右侧直角分支 (前黑+左白+右黑) */
                        if (Is_Car_Stable() && Check_Zero_Crossing_Stable()) {
                            PID_Reset(&LinePID);
                            g_turn_spin_dir = -1;
                            g_last_turn_dir = -1;
                            g_turn_state = TURN_SPINNING;
                            g_turn_start_tick = HAL_GetTick();
                            g_turn_deadline_tick = g_turn_start_tick + RIGHT_TURN_MAX_TIME;
                            g_turn_left_old_line = 0;
                            g_turn_center_count = 0;
                        }
                    }
                    else if (!front) {
                        if (0 && g_car_state == CAR_MODE_RADAR_ON_TRACKING) {
                            if (left && right) {
                                /* 雷达模式下踩到T字路口：立即强制停车并return */
                                Motor_Stop();
                                return;
                            }
                        }
                        else {
                            /* 非雷达模式：T字和死角路口右转/左转逻辑 */
                            if (left && right) {
                                /* 死胡同T字 (前白+左右黑) → 原地右转 */
                                PID_Reset(&LinePID);
                                g_turn_spin_dir = -1;
                                g_last_turn_dir = -1;
                                g_turn_state = TURN_SPINNING;
                                g_turn_start_tick = HAL_GetTick();
                                g_turn_deadline_tick = g_turn_start_tick + RIGHT_TURN_MAX_TIME;
                                g_turn_left_old_line = 0;
                                g_turn_center_count = 0;
                            }
                            else if (!left && right) {
                                /* 纯右侧死角 (前白+左白+右黑) → 右转 */
                                PID_Reset(&LinePID);
                                g_turn_spin_dir = -1;
                                g_last_turn_dir = -1;
                                g_turn_state = TURN_SPINNING;
                                g_turn_start_tick = HAL_GetTick();
                                g_turn_deadline_tick = g_turn_start_tick + RIGHT_TURN_MAX_TIME;
                                g_turn_left_old_line = 0;
                                g_turn_center_count = 0;
                            }
                            else if (left && !right) {
                                /* 纯左侧死角 (前白+左黑+右白) → 左转 */
                                PID_Reset(&LinePID);
                                g_turn_spin_dir = 1;
                                g_last_turn_dir = 1;
                                g_turn_state = TURN_SPINNING;
                                g_turn_start_tick = HAL_GetTick();
                                g_turn_deadline_tick = g_turn_start_tick + RIGHT_TURN_MAX_TIME;
                                g_turn_left_old_line = 0;
                                g_turn_center_count = 0;
                            }
                        }
                    }
                }

        if (g_turn_state == TURN_POST_STOP) {
            Motor_Stop();
            if (HAL_GetTick() - g_turn_start_tick >= 1000) {
                g_turn_state = TURN_IDLE;
            }
        }
        else if (g_turn_state != TURN_IDLE) {
            /* 已进入状态机，本轮不再执行普通PID */
        }
        else if (sensor_raw == 0x00) {
            LinePID.Integral  = 0.0f;
            LinePID.LastError = (float)error;
            Motor_SetBoth(LOST_TURN_SPEED * g_last_turn_dir,
                         -LOST_TURN_SPEED * g_last_turn_dir);
        }
        else if (error >= 6 || error <= -6) {
            int turn_dir = (error > 0) ? 1 : -1;
            LinePID.Integral  = 0.0f;
            LinePID.LastError = (float)error;
            g_last_turn_dir = turn_dir;
            Motor_SetBoth(TURN_SPEED * turn_dir,
                         -TURN_SPEED * turn_dir);
        }
        else {
            float adjust = PID_Compute(&LinePID, error);
            adjust += 1.5f;
            if (adjust > 45.0f)  adjust = 45.0f;
            if (adjust < -45.0f) adjust = -45.0f;
            int base = g_soft_kp_until ? SOFT_BASE_SPEED : NORMAL_BASE_SPEED;
            int left_pwm  = base + (int)adjust;
            int right_pwm = base - (int)adjust;
            Motor_SetBoth(left_pwm, right_pwm);
        }
    }
    else {
        Motor_Stop();
    }
}

int Track_GetLastError(void)
{
    return g_last_error;
}

int Track_IsSpecificTRoad(void)
{
    int front = Has_Front_Line(g_cached_raw);
    int left  = Has_Left_Branch(g_cached_raw);
    int right = Has_Right_Branch(g_cached_raw);

    if (!front && left && right) return 1;
    return 0;
}

int Track_IsLineRecovered(void)
{
    uint8_t raw = 0;

    if (HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG,
                         I2C_MEMADD_SIZE_8BIT, &raw, 1, 10) == HAL_OK)
    {
        if (raw == 0x00) return 0;
        int err = Get_Line_Error(raw);
        if (err > -4 && err < 4) return 1;
    }
    return 0;
}

void Track_UpdateRadarActivation(void)
{
    uint32_t now = HAL_GetTick();

    switch (g_radar_act_state) {
        case RADAR_WAIT_30S:
            if (now >= 90000U) {
                g_radar_act_state = RADAR_WAIT_10S_STRAIGHT;
                g_straight_start_tick = now;
            }
            break;

        case RADAR_WAIT_10S_STRAIGHT:
            /* 传感器数据变化检测：图案不变 = 车没动 */
            /*if (g_cached_raw != g_last_sensor_stuck) {
                g_last_sensor_stuck = g_cached_raw;
                g_sensor_change_tick = now;
            }*/
            if (now >= 990000U) {
                g_radar_act_state = RADAR_WAIT_2_TURNS;
                g_hard_turn_count = 0;
                g_last_hard_turn_tick = 0;
                g_was_spinning = (g_turn_state == TURN_SPINNING);
            } else if (g_turn_state == TURN_SPINNING || g_turn_state == TURN_STRAIGHT_BLIND) {
                g_straight_start_tick = now;
            } else if (now - g_straight_start_tick >= 8000U) {
                /* 传感器图案冻结 > 800ms → 车已卡住，重置直行计时 */
                g_radar_act_state = RADAR_WAIT_2_TURNS;
                g_hard_turn_count = 0;
                g_last_hard_turn_tick = 0;
                g_was_spinning = (g_turn_state == TURN_SPINNING);
            }
            break;

        case RADAR_WAIT_2_TURNS:
            if (g_turn_state == TURN_SPINNING && !g_was_spinning && g_turn_spin_dir > 0) {
                if (g_hard_turn_count == 0U
                    || (g_hard_turn_count == 1U && now - g_last_hard_turn_tick >= RADAR_TURN_INTERVAL_MS))
                {
                    g_hard_turn_count++;
                    g_last_hard_turn_tick = now;

                    if (g_hard_turn_count >= 2U || (g_hard_turn_count == 1U && now - g_last_hard_turn_tick >= RADAR_TURN_INTERVAL_MS)) {
                        g_radar_act_state = RADAR_READY;
                    }
                }
            }

            g_was_spinning = (g_turn_state == TURN_SPINNING);
            break;

        case RADAR_READY:
        default:
            break;
    }
}

uint8_t Track_GetRadarStage(void)
{
    return (uint8_t)g_radar_act_state;
}

int Track_ShouldActivateRadar(void)
{
    return g_radar_act_state == RADAR_READY;
}
