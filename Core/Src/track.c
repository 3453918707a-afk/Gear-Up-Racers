#include "track.h"
#include "i2c.h"
#include "tim.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define SENSOR_I2C_ADDR        (0x5D << 1)
#define SENSOR_DATA_REG        0x05

#define PWM_MAX                999
#define BASE_SPEED             330
#define TURN_SPEED             430
#define LOST_TURN_SPEED        390

#define LEFT_MOTOR_REVERSE     0
#define RIGHT_MOTOR_REVERSE    0

#define FRONT_MASK             0x18
#define LEFT_BRANCH_MASK       0x07
#define RIGHT_BRANCH_MASK      0xE0

/* 私有静态变量 --------------------------------------------------------------*/
static PID_TypeDef LinePID = {
    32.0f, 0.0f, 18.0f,
    0.0f, 0.0f, 0.0f, 0.0f
};

static int g_last_error = 0;
static int g_last_turn_dir = 1;
static uint8_t g_cached_raw = 0;

/* 私有函数声明 --------------------------------------------------------------*/
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

static int Need_Right_Turn(uint8_t raw)
{
    int front = Has_Front_Line(raw);
    int left = Has_Left_Branch(raw);
    int right = Has_Right_Branch(raw);

    if (front && right && !left) return 1;
    if (!front && left && right) return 1;
    return 0;
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

static int Get_Line_Error(uint8_t raw)
{
    int weights[8] = {7, 5, 3, 1, -1, -3, -5, -7};
    int sum = 0, count = 0, i, error;

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

    if (count == 0) return g_last_error;

    error = sum / count;
    g_last_error = error;
    if (error > 0) g_last_turn_dir = 1;
    if (error < 0) g_last_turn_dir = -1;

    /* EMA 低通滤波：抑制传感器瞬时噪声，0.5 系数兼顾响应速度与平滑度 */
    static int filtered = 0;
    static uint8_t filter_ready = 0;
    if (!filter_ready) {
        filtered = error;
        filter_ready = 1;
    } else {
        filtered = (int)(0.5f * (float)filtered + 0.5f * (float)error);
    }

    return filtered;
}

static float PID_Compute(PID_TypeDef *pid, int current_err)
{
    /*
     * 时间感知 PID：测量两次调用之间的实际时间间隔 dt，按 dt 缩放积分和微分项。
     * 标称周期 5ms (840,000 个 168MHz 周期)，dt_norm = 实际 / 标称。
     * 屏幕绘图拉长周期时，I/D 项自动补偿 —— 无需把 I2C 放进 ISR 冒险。
     */
    static uint32_t last_cycles = 0;
    static uint8_t  first_call = 1;
    float dt_norm;

    if (first_call) {
        last_cycles = DWT->CYCCNT;
        first_call = 0;
        dt_norm = 1.0f;
    } else {
        uint32_t now = DWT->CYCCNT;
        uint32_t elapsed = now - last_cycles;
        last_cycles = now;
        if (elapsed > 8400000) elapsed = 8400000;   /* 上限 50ms，异常保护 */
        if (elapsed < 168000)  elapsed = 168000;    /* 下限 1ms  */
        dt_norm = (float)elapsed / 840000.0f;        /* 5ms = 1.0 */
    }

    pid->Error = (float)current_err;

    /* 积分项：误差 × 归一化时间，周期长则积分多（物理上误差作用更久） */
    pid->Integral += pid->Error * dt_norm;
    if (pid->Integral > 80.0f) pid->Integral = 80.0f;
    if (pid->Integral < -80.0f) pid->Integral = -80.0f;

    /* 微分项：误差变化率 / 归一化时间，周期长则变化率低（物体移动更慢） */
    pid->Output = pid->Kp * pid->Error
                + pid->Ki * pid->Integral
                + pid->Kd * (pid->Error - pid->LastError) / dt_norm;

    pid->LastError = pid->Error;
    return pid->Output;
}

void Track_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    Motor_Stop();
}

void Track_Control(void)
{
    uint8_t sensor_raw = 0;
    static uint8_t pid_was_active = 1;

    if (HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG, I2C_MEMADD_SIZE_8BIT, &sensor_raw, 1, 50) == HAL_OK)
    {
        g_cached_raw = sensor_raw;
        int error = Get_Line_Error(sensor_raw);

        if (Need_Right_Turn(sensor_raw)) {
            g_last_turn_dir = -1;
            Motor_SetBoth(-TURN_SPEED, TURN_SPEED);
            pid_was_active = 0;
        }
        else if (sensor_raw == 0x00) {
            Motor_SetBoth(LOST_TURN_SPEED * g_last_turn_dir, -LOST_TURN_SPEED * g_last_turn_dir);
            pid_was_active = 0;
        }
        else if (error >= 6 || error <= -6) {
            int turn_dir = (error > 0) ? 1 : -1;
            g_last_turn_dir = turn_dir;
            Motor_SetBoth(TURN_SPEED * turn_dir, -TURN_SPEED * turn_dir);
            pid_was_active = 0;
        }
        else {
            /* 硬转向后首次切回 PID：令 LastError = 当前误差，消除微分冲击 */
            if (!pid_was_active) {
                LinePID.LastError = (float)error;
                pid_was_active = 1;
            }
            float adjust = PID_Compute(&LinePID, error);
            int left_pwm  = BASE_SPEED + (int)adjust;
            int right_pwm = BASE_SPEED - (int)adjust;

            /* 输出变化率限制：单次 PWM 跳变不超过 SLEW_MAX，抑制过冲震荡 */
            static int prev_left  = BASE_SPEED;
            static int prev_right = BASE_SPEED;
            #define SLEW_MAX  80
            if (left_pwm - prev_left > SLEW_MAX)   left_pwm = prev_left + SLEW_MAX;
            if (prev_left - left_pwm > SLEW_MAX)   left_pwm = prev_left - SLEW_MAX;
            if (right_pwm - prev_right > SLEW_MAX)  right_pwm = prev_right + SLEW_MAX;
            if (prev_right - right_pwm > SLEW_MAX)  right_pwm = prev_right - SLEW_MAX;
            prev_left  = left_pwm;
            prev_right = right_pwm;

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
    /* 复用 Track_Control 最近一次读到的传感器原始值，避免重复 I2C 读取 */
    int front = Has_Front_Line(g_cached_raw);
    int left  = Has_Left_Branch(g_cached_raw);
    int right = Has_Right_Branch(g_cached_raw);

    if (!front && left && right) return 1;
    return 0;
}

int Track_IsLineRecovered(void)
{
    uint8_t raw = 0;

    if (HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG, I2C_MEMADD_SIZE_8BIT, &raw, 1, 10) == HAL_OK)
    {
        if (raw == 0x00) return 0;
        int err = Get_Line_Error(raw);
        if (err > -4 && err < 4) return 1;
    }
    return 0;
}
