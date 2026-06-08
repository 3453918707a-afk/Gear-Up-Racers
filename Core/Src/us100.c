#include "us100.h"
#include "lora.h"
#include "main.h"  // 必须包含以获取 SystemCoreClock 和 HAL 库支持
#include <stdio.h>

#define TRIG_PIN          GPIO_PIN_10
#define TRIG_PORT         GPIOC
#define ECHO_PIN          GPIO_PIN_11
#define ECHO_PORT         GPIOC

#define TRIG_PULSE_US     15
#define MEASURE_TIMEOUT   100
#define TRIGGER_DIST_CM   50.0f
#define TX_LOCKOUT_MS     7000

volatile uint8_t  hour        = 9;
volatile uint8_t  minute      = 07;
volatile uint8_t  second      = 17;
volatile uint16_t elapsed_min = 0;
volatile uint8_t  elapsed_sec = 0;

volatile uint32_t echo_start   = 0;
volatile uint32_t echo_width   = 0;
volatile uint8_t  echo_ready   = 0;
volatile float    distance_cm  = 0.0f;
volatile uint32_t hcsr04_sample_seq = 0;

static uint32_t   last_tx_tick = 0;

static void DWT_Init(void)
{
    CoreDebug->DEMCR  |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t DWT_Get(void)
{
    return DWT->CYCCNT;
}

static void DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT_Get();
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT_Get() - start) < ticks);
}

static void HCSR04_Trigger(void)
{
    echo_ready = 0;
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    DWT_DelayUs(TRIG_PULSE_US);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

void HCSR04_Init(void)
{
    DWT_Init();
}

void HCSR04_Task(void)
{
    static uint32_t last_measure_tick = 0;

    // 1. 每 60ms 到了之后，先处理【上一次】已经捕获完成的雷达/超声波数据
    if (HAL_GetTick() - last_measure_tick >= 60)
    {
        last_measure_tick = HAL_GetTick();

        // 异步检查：如果 EXTI 中断已经成功捕获到了边沿，echo_ready 会被置 1
        if (echo_ready)
        {
            echo_ready = 0; // 清除就绪标志

            // 只有合法的距离才触发无线发送逻辑
            if (distance_cm > 0.0f && distance_cm < TRIGGER_DIST_CM)
            {
                if (HAL_GetTick() - last_tx_tick >= TX_LOCKOUT_MS)
                {
                    last_tx_tick = HAL_GetTick();

                    char msg[120];
                    snprintf(msg, sizeof(msg),
                                                 "T:%02d:%02d:%02d,N:02,TEAM:Gear Up Racers,ET:%02d:%02d\r\n",
                                                 hour, minute, second,
                                                 elapsed_min, elapsed_sec);
                    LoRa_SendFixed(msg);
                }
            }
        }

        // 2. 处理完旧数据后，立刻发出【下一次】测距的触发脉冲
        // 发完后函数直接退出，主循环继续跑 PID 循迹，等待 EXTI 中断在后台默默捕获
        HCSR04_Trigger();
    }
}

void HCSR04_TIM_Callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        elapsed_sec++;
        if (elapsed_sec >= 60)
        {
            elapsed_sec = 0;
            elapsed_min++;
        }
        second++;
        if (second >= 60)
        {
            second = 0;
            minute++;
            if (minute >= 60)
            {
                minute = 0;
                hour++;
                if (hour >= 24) hour = 0;
            }
        }
    }
}

void HCSR04_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != ECHO_PIN) return;

    if (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        echo_start = DWT_Get();
    }
    else
    {
        uint32_t end = DWT_Get();
        if (end > echo_start)
        {
            echo_width = end - echo_start;
            distance_cm = (float)echo_width / (168.0f * 58.0f);
        }
        else
        {
            distance_cm = 0.0f;
        }
        echo_ready = 1;
        hcsr04_sample_seq++;
    }
}
