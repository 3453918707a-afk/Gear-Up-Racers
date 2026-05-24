/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body与新版路线绘制动态切换整合
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "hlk_ld2410s.h"
#include "track.h"       // 智能循迹模块
#include "obstacle.h"    // 状态机避障管理模块
#include "lora.h"        // 无线通信模块
#include "us100.h"       // 超声波整合模块
#include "route_draw.h"  // 新增引入的路线绘制模块
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rx_byte1;
uint8_t rx_byte2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_UART4_Init();               // 保留：超声波/通信相关外设
  MX_TIM3_Init();                // 保留：系统时间基准定时器
  MX_USART3_UART_Init();         // 保留：LoRa 串口

  /* USER CODE BEGIN 2 */
  LCD_Init();
  LD2410S_Init();
  Track_Init();
  Obstacle_Init();
  //Obstacle_ActivateRadar();      // 上电直接激活雷达状态机

  HCSR04_Init();                 // 保留：高精度延时组件初始化
  LoRa_Init();                   // 保留：无线射频模块软启动延迟

  HAL_TIM_Base_Start_IT(&htim3); // 保留：以中断方式开启 TIM3 计时器

  HAL_Delay(30);

  // 开启毫米波双雷达串口非阻塞中断接收
  HAL_UART_Receive_IT(&huart2, &rx_byte1, 1);
  HAL_UART_Receive_IT(&huart5, &rx_byte2, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // 状态切换检测所需的辅助静态变量
  static CarState_t last_state = CAR_MODE_NORMAL_TRACKING;
  Route_Draw_Init(); // 初始画布化

  while (1)
  {
      // 1. 指示灯反转，提供肉眼烧录与正常运行诊断
      HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);

      // 2. 状态切换检测：如果工作模式变了，需要清屏以防雷达图和路线图重叠
      if (last_state != g_car_state) {
          LCD_FillRect(0, 0, 320, 240, BLACK); // 模式切换，全局清屏
          if (g_car_state == CAR_MODE_NORMAL_TRACKING) {
              Route_Draw_Init(); // 若切回了普通循迹，重新初始化画布并打印提示
          }
          last_state = g_car_state;
      }

      // 3. 根据当前模式决定屏幕显示内容
      if (g_car_state == CAR_MODE_NORMAL_TRACKING) {
          Route_Draw_Task();          // 模式A：屏幕绘制小车行驶路线
      } else {
          Draw_Energy_Spectrum();     // 模式B：近场雷达触发，绘制雷达能量图谱
      }

      // 4. 多路状态机实时接管：小车基本循迹与近场雷达刹车避障
      Obstacle_ManagerLoop();

      // 5. 保留原有超声波任务：测距并在<50cm时通过LoRa自动无线上传环境报文
      HCSR04_Task();

      //HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
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
/**
 * @brief  统一管理整个硬件系统的外设中断回调（绝不能删，否则超声波和LoRa失效）
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 路由分发给超声波/时间计数文件累计系统运行时间
    HCSR04_TIM_Callback(htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 捕获超声波Echo引脚的双边沿宽度
    HCSR04_EXTI_Callback(GPIO_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 以前：USART2 连接 radar1。现在：直接让 USART2 的底层硬件数据流进 radar2（左侧）
    if(huart->Instance == USART2)
    {
        LD2410S_ParseByte(&radar2, rx_byte1); // <--- 修改为 &radar2
        HAL_UART_Receive_IT(&huart2, &rx_byte1, 1);
    }
    // 以前：UART5 连接 radar2。现在：直接让 UART5 的底层硬件数据流进 radar1（右侧）
    else if(huart->Instance == UART5)
    {
        LD2410S_ParseByte(&radar1, rx_byte2); // <--- 修改为 &radar1
        HAL_UART_Receive_IT(&huart5, &rx_byte2, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        HAL_UART_Receive_IT(&huart2, &rx_byte1, 1);
    }
    else if(huart->Instance == UART5)
    {
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
