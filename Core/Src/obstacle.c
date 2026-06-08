#include "obstacle.h"
#include "hlk_ld2410s.h"
#include "i2c.h"
#include "track.h"

volatile CarState_t g_car_state = CAR_MODE_NORMAL_TRACKING;
volatile uint8_t g_avoidance_done = 0;

#define SENSOR_I2C_ADDR             (0x5D << 1)
#define SENSOR_DATA_REG             0x05

#define FRONT_MASK                  0x18
#define LEFT_BRANCH_MASK            0x07
#define RIGHT_BRANCH_MASK           0xE0

#define RADAR_ARM_DELAY_MS          4000U
#define POST_RADAR_TURN_DELAY_MS    15000U
#define JUNCTION_FORWARD_MS         300U
#define JUNCTION_FORWARD_SPEED      220
#define TURN_SPEED                  360
#define TURN_MIN_MS                 320U
#define TURN_TIMEOUT_MS             1300U

typedef enum {
    RADAR_FLOW_IDLE = 0,
    RADAR_FLOW_WAIT_T,
    RADAR_FLOW_T_FORWARD,
    RADAR_FLOW_T_TURN,
    RADAR_FLOW_WAIT_15S,
    RADAR_FLOW_WAIT_HALF_T,
    RADAR_FLOW_HALF_T_TURN,
    RADAR_FLOW_DONE
} RadarFlowState_t;

static RadarFlowState_t g_radar_flow_state = RADAR_FLOW_IDLE;
static uint32_t g_radar_enter_tick = 0;
static uint32_t g_flow_tick = 0;
static uint32_t g_forward_until = 0;
static int g_radar_turn_dir = 0;
static uint8_t g_turn_center_count = 0;
static uint8_t g_turn_saw_no_front = 0;

static float g_avg_left_dist = 200.0f;
static float g_avg_right_dist = 200.0f;

static int Count_Bits(uint8_t value)
{
    int count = 0;
    while (value) {
        count += (value & 0x01) ? 1 : 0;
        value >>= 1;
    }
    return count;
}

static int Has_Front_Line(uint8_t raw)
{
    return (raw & FRONT_MASK) != 0;
}

static int Has_Left_Branch(uint8_t raw)
{
    return Count_Bits(raw & LEFT_BRANCH_MASK) >= 2;
}

static int Has_Right_Branch(uint8_t raw)
{
    return Count_Bits(raw & RIGHT_BRANCH_MASK) >= 2;
}

static int Read_Line_Sensor(uint8_t *raw)
{
    return HAL_I2C_Mem_Read(&hi2c2, SENSOR_I2C_ADDR, SENSOR_DATA_REG,
                            I2C_MEMADD_SIZE_8BIT, raw, 1, 10) == HAL_OK;
}

static int Is_T_Junction_Now(void)
{
    uint8_t raw = 0;

    if (!Read_Line_Sensor(&raw)) return 0;

    return Count_Bits(raw) >= 7;
}

static int Is_Half_T_Junction_Now(void)
{
    uint8_t raw = 0;
    int front;
    int left;
    int right;

    if (!Read_Line_Sensor(&raw)) return 0;

    front = Has_Front_Line(raw);
    left = Has_Left_Branch(raw);
    right = Has_Right_Branch(raw);

    return front && (left || right) && !(left && right);
}

static int Obstacle_GetRadarAvoidDir(void)
{
    return (g_avg_left_dist > g_avg_right_dist) ? -1 : 1;
}

int8_t Obstacle_GetPendingAvoidDir(void)
{
    if (g_radar_turn_dir != 0) {
        return (int8_t)g_radar_turn_dir;
    }

    if (g_avg_left_dist > g_avg_right_dist) return -1;
    if (g_avg_left_dist < g_avg_right_dist) return 1;
    return 0;
}

static void Set_Turn_Spin(int dir)
{
    if (dir < 0) {
        Motor_SetBoth(TURN_SPEED, -TURN_SPEED);
    } else if (dir > 0) {
        Motor_SetBoth(-TURN_SPEED, TURN_SPEED);
    } else {
        Motor_Stop();
    }
}

static void Start_Turn(RadarFlowState_t next_state)
{
    g_radar_flow_state = next_state;
    g_turn_center_count = 0;
    g_turn_saw_no_front = 0;
    g_flow_tick = HAL_GetTick();
}

static int Turn_Finished(void)
{
    uint8_t raw = 0;
    uint32_t elapsed = HAL_GetTick() - g_flow_tick;

    Set_Turn_Spin(g_radar_turn_dir);

    if (elapsed >= TURN_TIMEOUT_MS) {
        return 1;
    }

    if (elapsed < TURN_MIN_MS) {
        return 0;
    }

    if (!Read_Line_Sensor(&raw)) {
        return 0;
    }

    if (!Has_Front_Line(raw)) {
        g_turn_saw_no_front = 1;
        g_turn_center_count = 0;
        return 0;
    }

    if (g_turn_saw_no_front && Has_Front_Line(raw)) {
        g_turn_center_count++;
        if (g_turn_center_count >= 2U) {
            return 1;
        }
    } else {
        g_turn_center_count = 0;
    }

    return 0;
}

static void Finish_First_Radar_Turn(void)
{
    Motor_Stop();
    Track_ResetControlState();
    g_car_state = CAR_MODE_NORMAL_TRACKING;
    g_avoidance_done = 1;
    g_radar_flow_state = RADAR_FLOW_WAIT_15S;
    g_flow_tick = HAL_GetTick();
}

void Obstacle_Init(void)
{
    g_car_state = CAR_MODE_NORMAL_TRACKING;
    g_avoidance_done = 0;
    g_radar_enter_tick = 0;
    g_flow_tick = 0;
    g_forward_until = 0;
    g_radar_turn_dir = 0;
    g_turn_center_count = 0;
    g_turn_saw_no_front = 0;
    g_radar_flow_state = RADAR_FLOW_IDLE;
}

void Obstacle_ActivateRadar(void)
{
    if (g_car_state == CAR_MODE_NORMAL_TRACKING && g_radar_flow_state == RADAR_FLOW_IDLE) {
        g_car_state = CAR_MODE_RADAR_ON_TRACKING;
        g_radar_enter_tick = HAL_GetTick();
        g_radar_flow_state = RADAR_FLOW_WAIT_T;
    }
}

void Obstacle_ManagerLoop(void)
{
    float cur_left = (float)radar1.data.target_dist_cm;
    float cur_right = (float)radar2.data.target_dist_cm;
    uint32_t now = HAL_GetTick();

    if (cur_left <= 0.20f) cur_left = 200.0f;
    if (cur_right <= 0.20f) cur_right = 200.0f;

    if (g_car_state != CAR_MODE_AVOIDING) {
        g_avg_left_dist = g_avg_left_dist * 0.2f + cur_left * 0.8f;
        g_avg_right_dist = g_avg_right_dist * 0.2f + cur_right * 0.8f;
    }

    switch (g_radar_flow_state) {
        case RADAR_FLOW_WAIT_T:
            if (g_car_state != CAR_MODE_RADAR_ON_TRACKING) {
                break;
            }

            if (now - g_radar_enter_tick < RADAR_ARM_DELAY_MS) {
                break;
            }

            if (Is_T_Junction_Now()) {
                g_radar_turn_dir = Obstacle_GetRadarAvoidDir();
                g_forward_until = now + JUNCTION_FORWARD_MS;
                g_car_state = CAR_MODE_AVOIDING;
                g_radar_flow_state = RADAR_FLOW_T_FORWARD;
            }
            break;

        case RADAR_FLOW_T_FORWARD:
            if (now < g_forward_until) {
                Motor_SetBoth(JUNCTION_FORWARD_SPEED, JUNCTION_FORWARD_SPEED);
            } else {
                Start_Turn(RADAR_FLOW_T_TURN);
            }
            break;

        case RADAR_FLOW_T_TURN:
            if (Turn_Finished()) {
                Finish_First_Radar_Turn();
            }
            break;

        case RADAR_FLOW_WAIT_15S:
            if (now - g_flow_tick >= POST_RADAR_TURN_DELAY_MS) {
                g_radar_flow_state = RADAR_FLOW_WAIT_HALF_T;
            }
            break;

        case RADAR_FLOW_WAIT_HALF_T:
            if (Is_Half_T_Junction_Now()) {
                g_car_state = CAR_MODE_AVOIDING;
                Start_Turn(RADAR_FLOW_HALF_T_TURN);
            }
            break;

        case RADAR_FLOW_HALF_T_TURN:
            if (Turn_Finished()) {
                Motor_Stop();
                Track_ResetControlState();
                g_car_state = CAR_MODE_NORMAL_TRACKING;
                g_radar_flow_state = RADAR_FLOW_DONE;
            }
            break;

        case RADAR_FLOW_IDLE:
        case RADAR_FLOW_DONE:
        default:
            break;
    }
}

int8_t Obstacle_CompareAvgDist(void)
{
    if (g_avg_left_dist > g_avg_right_dist) return 1;
    if (g_avg_left_dist < g_avg_right_dist) return -1;
    return 0;
}
