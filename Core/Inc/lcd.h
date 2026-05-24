#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"

// --- 引脚控制宏定义 (新配置) ---
#define LCD_CS_CLR()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define LCD_CS_SET()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

#define LCD_DC_CLR()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)
#define LCD_DC_SET()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)

#define LCD_RST_CLR()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define LCD_RST_SET()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)

// --- LCD 颜色代码 (RGB565 格式) ---
#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE             0x001F
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40
#define BRRED            0XFC07
#define GRAY             0X8430
#define DARKBLUE         0X01CF
#define LIGHTBLUE        0X7D7C
#define GRAYBLUE         0X5458
#define LIGHTGREEN       0X841F
#define LGRAY            0XC618
#define LGRAYBLUE        0XA651
#define LBBLUE           0X2B12

// --- 基础显示控制函数声明 ---
void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_Init(void);
void LCD_Clear(uint16_t color);

// --- 图形与文本函数声明 ---
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_ShowChar(uint16_t x, uint16_t y, char num, uint16_t color, uint16_t bg_color);
void LCD_ShowString(uint16_t x, uint16_t y, char *p, uint16_t color, uint16_t bg_color);

#endif /* INC_LCD_H_ */
