/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "hlk_ld2410s.h"
#include "track.h"
#include "obstacle.h"
#include "lora.h"
#include "us100.h"
#include "route_draw.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
uint8_t rx_byte1;
uint8_t rx_byte2;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define FINISH_TRIGGER_DIST_CM   50.0f
#define FINISH_STRAIGHT_MS       3000U
#define FINISH_STRAIGHT_SPEED    260
#define FINISH_SENSOR_I2C_ADDR   (0x5D << 1)
#define FINISH_SENSOR_DATA_REG   0x05
/* USER CODE END PD */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
int Track_IsFinishStraightActive(void);
void Track_FinishControlTask(void);
static int Finish_UltrasonicTask(void);
static void Finish_ShowEndScreen(void);
static void DebugLed_Update(uint8_t radar_stage, uint8_t blink_state);
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  LCD_Init();
  LD2410S_Init();

  huart2.hdmarx = NULL;
  huart5.hdmarx = NULL;
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart5);
  HAL_UART_Receive_IT(&huart2, &rx_byte1, 1);
  HAL_UART_Receive_IT(&huart5, &rx_byte2, 1);

  HCSR04_Init();
  LoRa_Init();
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_Delay(200);
  Track_Init();
  Obstacle_Init();
  //Obstacle_ActivateRadar();
  HAL_Delay(200);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  static CarState_t last_state = (CarState_t)-1;
  static uint8_t radar_activated = 0;
  static uint32_t last_blink_tick = 0;
  static uint8_t blink_state = 0;

  while (1)
  {
      uint8_t radar_stage = Track_GetRadarStage();
      HCSR04_Task();

      if (!radar_activated && Track_ShouldActivateRadar()) {
          Obstacle_ActivateRadar();
          radar_activated = 1;
      }

      if (HAL_GetTick() - last_blink_tick >= 200U) {
          last_blink_tick = HAL_GetTick();
          blink_state = !blink_state;
      }

      if (Track_IsFinished()) {
          Motor_Stop();
          continue;
      }

      if (Track_IsFinishStraightActive()) {
          Track_FinishControlTask();
          DebugLed_Update(radar_stage, blink_state);
          Route_Draw_Task();
          HCSR04_Task();
          HAL_Delay(10);
          continue;
      }

      Obstacle_ManagerLoop();
      DebugLed_Update(radar_stage, blink_state);

      if (last_state != g_car_state) {
          LCD_FillRect(0, 0, 320, 240, BLACK);
          if (g_car_state == CAR_MODE_NORMAL_TRACKING) {
              Route_Draw_Init();
          }
          last_state = g_car_state;
      }

      if (Finish_UltrasonicTask()) {
          Route_Draw_Task();
          HAL_Delay(10);
          continue;
      }

      if (g_car_state != CAR_MODE_AVOIDING) {
          Track_Control();
      }

      if (g_car_state == CAR_MODE_NORMAL_TRACKING) {
          Route_Draw_Task();
      } else {
          Draw_Energy_Spectrum();
      }

      HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
typedef enum {
    FINISH_WAIT_TRIGGER = 0,
    FINISH_DRIVE_STRAIGHT,
    FINISH_STOPPED
} FinishState_t;

static FinishState_t g_finish_state = FINISH_WAIT_TRIGGER;
static uint32_t g_finish_start_tick = 0;
static uint32_t g_finish_trigger_seq = 0;
static uint8_t g_finish_display_done = 0;

static int Finish_ReadLineRaw(uint8_t *raw)
{
    return HAL_I2C_Mem_Read(&hi2c2, FINISH_SENSOR_I2C_ADDR, FINISH_SENSOR_DATA_REG,
                            I2C_MEMADD_SIZE_8BIT, raw, 1, 10) == HAL_OK;
}

static int Finish_LineIsLost(void)
{
    uint8_t raw = 0;

    if (!Finish_ReadLineRaw(&raw)) {
        return 0;
    }

    return raw == 0x00;
}

static int Finish_UltrasonicTask(void)
{
    if (g_finish_state == FINISH_WAIT_TRIGGER) {
        if (!g_avoidance_done) {
            g_finish_trigger_seq = hcsr04_sample_seq;
            return 0;
        }

        if (hcsr04_sample_seq != g_finish_trigger_seq
            && distance_cm > 0.0f
            && distance_cm < FINISH_TRIGGER_DIST_CM
            && Finish_LineIsLost())
        {
            g_finish_start_tick = HAL_GetTick();
            g_finish_state = FINISH_DRIVE_STRAIGHT;
            Motor_SetBoth(FINISH_STRAIGHT_SPEED, FINISH_STRAIGHT_SPEED);
            return 1;
        }
        return 0;
    }

    return g_finish_state == FINISH_DRIVE_STRAIGHT;
}

static void Finish_DrawBlockLetter(uint16_t x, uint16_t y, uint8_t scale,
                                   const char rows[7], uint16_t color)
{
    for (uint8_t row = 0; row < 7; row++) {
        for (uint8_t col = 0; col < 5; col++) {
            if (rows[row * 5 + col] == '1') {
                LCD_FillRect(x + col * scale, y + row * scale, scale - 1, scale - 1, color);
            }
        }
    }
}

static void Finish_ShowEndScreen(void)
{
    static const char win_w[] =
        "10001"
        "10001"
        "10001"
        "10101"
        "10101"
        "11011"
        "10001";
    static const char win_i[] =
        "11111"
        "00100"
        "00100"
        "00100"
        "00100"
        "00100"
        "11111";
    static const char win_n[] =
        "10001"
        "11001"
        "10101"
        "10011"
        "10001"
        "10001"
        "10001";
    static const uint16_t name_zhou[] = {0x5468, 0x8F69, 0x5B87};
    static const uint16_t name_cui[]  = {0x5D14, 0x5BB8, 0x8D6B};
    static const uint16_t name_wang[] = {0x738B, 0x5609, 0x6210};
    static const uint16_t name_shen[] = {0x6C88, 0x9633};
    static const uint16_t name_yi[]   = {0x738B, 0x9038, 0x9633};

    LCD_FillRect(0, 0, 107, 240, RED);
    LCD_FillRect(107, 0, 107, 240, GREEN);
    LCD_FillRect(214, 0, 106, 240, BLUE);
    Finish_DrawBlockLetter(18, 64, 13, win_w, WHITE);
    Finish_DrawBlockLetter(126, 64, 13, win_i, BLACK);
    Finish_DrawBlockLetter(232, 64, 13, win_n, YELLOW);
    HAL_Delay(1500);

    LCD_FillRect(0, 0, 320, 80, RED);
    LCD_FillRect(0, 80, 320, 80, GREEN);
    LCD_FillRect(0, 160, 320, 80, BLUE);
    LCD_ShowStringScaled(112, 16, "Gear", 3, WHITE, RED);
    LCD_ShowStringScaled(136, 96, "Up", 3, BLACK, GREEN);
    LCD_ShowStringScaled(88, 176, "Racers", 3, YELLOW, BLUE);
    HAL_Delay(1500);

    LCD_Clear(BLACK);
    LCD_ShowHz32Line(106, 20, name_zhou, 3, WHITE, BLACK);
    LCD_ShowHz32Line(106, 62, name_cui, 3, CYAN, BLACK);
    LCD_ShowHz32Line(106, 104, name_wang, 3, YELLOW, BLACK);
    LCD_ShowHz32Line(124, 146, name_shen, 2, GREEN, BLACK);
    LCD_ShowHz32Line(106, 188, name_yi, 3, MAGENTA, BLACK);
}

static void DebugLed_Update(uint8_t radar_stage, uint8_t blink_state)
{
    if (g_car_state == CAR_MODE_AVOIDING) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, blink_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, blink_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        return;
    }

    if (radar_stage == 0U) {
        GPIO_PinState pin_val = blink_state ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, pin_val);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, pin_val);
    } else if (radar_stage == 1U) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, blink_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, blink_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

int Track_IsFinished(void)
{
    if (g_finish_state == FINISH_STOPPED) {
        return 1;
    }

    if (g_finish_state == FINISH_DRIVE_STRAIGHT) {
        if (HAL_GetTick() - g_finish_start_tick >= FINISH_STRAIGHT_MS) {
            g_finish_state = FINISH_STOPPED;
            Motor_Stop();
            if (!g_finish_display_done) {
                g_finish_display_done = 1;
                Finish_ShowEndScreen();
            }
            return 1;
        }
        return 0;
    }

    return 0;
}

int Track_IsFinishStraightActive(void)
{
    return g_finish_state == FINISH_DRIVE_STRAIGHT;
}

void Track_FinishControlTask(void)
{
    if (g_finish_state == FINISH_DRIVE_STRAIGHT) {
        Motor_SetBoth(FINISH_STRAIGHT_SPEED, FINISH_STRAIGHT_SPEED);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HCSR04_TIM_Callback(htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HCSR04_EXTI_Callback(GPIO_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        LD2410S_ParseByte(&radar2, rx_byte1);
        HAL_UART_Receive_IT(&huart2, &rx_byte1, 1);
    } else if (huart->Instance == UART5) {
        LD2410S_ParseByte(&radar1, rx_byte2);
        HAL_UART_Receive_IT(&huart5, &rx_byte2, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    HAL_UART_AbortReceive(huart);

    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, &rx_byte1, 1);
    } else if (huart->Instance == UART5) {
        HAL_UART_Receive_IT(&huart5, &rx_byte2, 1);
    }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
