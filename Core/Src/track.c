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

static void PID_Reset(PID_TypeDef *pid)
{
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->Integral = 0.0f;
    pid->Output = 0.0f;
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
    Motor_Stop();
}

void Track_Control(void)
{
    uint8_t sensor_raw = 0;

    if (HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG, I2C_MEMADD_SIZE_8BIT, &sensor_raw, 1, 50) == HAL_OK)
    {
        int error = Get_Line_Error(sensor_raw);

        if (Need_Right_Turn(sensor_raw)) {
            g_last_turn_dir = -1;
            Motor_SetBoth(-TURN_SPEED, TURN_SPEED);
        }
        else if (sensor_raw == 0x00) {
            PID_Reset(&LinePID);
            Motor_SetBoth(LOST_TURN_SPEED * g_last_turn_dir, -LOST_TURN_SPEED * g_last_turn_dir);
        }
        else if (error >= 6 || error <= -6) {
            int turn_dir = (error > 0) ? 1 : -1;
            PID_Reset(&LinePID);
            g_last_turn_dir = turn_dir;
            Motor_SetBoth(TURN_SPEED * turn_dir, -TURN_SPEED * turn_dir);
        }
        else {
            float adjust = PID_Compute(&LinePID, error);
            int left_pwm = BASE_SPEED + (int)adjust;
            int right_pwm = BASE_SPEED - (int)adjust;
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
    uint8_t sensor_raw = 0;
    // 从底层寄存器直接读取当前红外传感器的状态
    if (HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG, I2C_MEMADD_SIZE_8BIT, &sensor_raw, 1, 50) == HAL_OK)
    {
        int front = Has_Front_Line(sensor_raw);  // 前方是否有线
        int left  = Has_Left_Branch(sensor_raw); // 左边是否有分支
        int right = Has_Right_Branch(sensor_raw);// 右边是否有分支

        // 严格匹配用户需求：左右有，前没有
        if (!front && left && right)
        {
            return 1;
        }
    }
    return 0;
}
