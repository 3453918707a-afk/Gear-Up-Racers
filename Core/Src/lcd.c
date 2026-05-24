/*
 * lcd.c
 *
 * Created on: May 7, 2026
 * Author: J2005
 */
#include "lcd.h"
#include "font.h"

extern SPI_HandleTypeDef hspi1;

void LCD_WriteCmd(uint8_t cmd) {
    LCD_DC_CLR();
    LCD_CS_CLR();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    LCD_CS_SET();
}

void LCD_WriteData(uint8_t data) {
    LCD_DC_SET();
    LCD_CS_CLR();
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    LCD_CS_SET();
}

void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    LCD_WriteCmd(0x2A);
    LCD_WriteData(x0 >> 8); LCD_WriteData(x0 & 0xFF);
    LCD_WriteData(x1 >> 8); LCD_WriteData(x1 & 0xFF);

    LCD_WriteCmd(0x2B);
    LCD_WriteData(y0 >> 8); LCD_WriteData(y0 & 0xFF);
    LCD_WriteData(y1 >> 8); LCD_WriteData(y1 & 0xFF);

    LCD_WriteCmd(0x2C);
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if(w == 0 || h == 0) return;
    LCD_SetWindow(x, y, x + w - 1, y + h - 1);

    LCD_DC_SET();
    LCD_CS_CLR();

    /* 按批次缓冲发送，32 像素 = 64 字节一批，大幅减少 SPI 事务数 */
    uint32_t total = (uint32_t)w * h;
    uint8_t  batch_buf[64];
    uint8_t  color_hi = color >> 8;
    uint8_t  color_lo = color & 0xFF;

    while (total > 0) {
        uint32_t batch = (total > 32) ? 32 : total;
        uint8_t *p = batch_buf;
        for (uint32_t i = 0; i < batch; i++) {
            *p++ = color_hi;
            *p++ = color_lo;
        }
        HAL_SPI_Transmit(&hspi1, batch_buf, batch * 2, HAL_MAX_DELAY);
        total -= batch;
    }
    LCD_CS_SET();
}

void LCD_Init(void) {
    LCD_RST_SET(); HAL_Delay(10);
    LCD_RST_CLR(); HAL_Delay(20);
    LCD_RST_SET(); HAL_Delay(120);

    LCD_WriteCmd(0x11); HAL_Delay(120);
    LCD_WriteCmd(0x36); LCD_WriteData(0x28);
    LCD_WriteCmd(0x3A); LCD_WriteData(0x55);
    LCD_WriteCmd(0x29); HAL_Delay(20);

    LCD_FillRect(0, 0, 320, 240, BLACK); // 初始全屏清黑
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color) {
    LCD_SetWindow(x, y, x, y);
    LCD_DC_SET();
    LCD_CS_CLR();
    uint8_t data[2] = {color >> 8, color & 0xFF};
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    LCD_CS_SET();
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    int xerr = 0, yerr = 0, delta_x = x2 - x1, delta_y = y2 - y1;
    int incx = (delta_x > 0) ? 1 : ((delta_x == 0) ? 0 : -1);
    int incy = (delta_y > 0) ? 1 : ((delta_y == 0) ? 0 : -1);
    delta_x = (delta_x > 0) ? delta_x : -delta_x;
    delta_y = (delta_y > 0) ? delta_y : -delta_y;
    int distance = (delta_x > delta_y) ? delta_x : delta_y;
    int uRow = x1, uCol = y1;

    for (int t = 0; t <= distance + 1; t++) {
        LCD_DrawPoint(uRow, uCol, color);
        xerr += delta_x; yerr += delta_y;
        if (xerr > distance) { xerr -= distance; uRow += incx; }
        if (yerr > distance) { yerr -= distance; uCol += incy; }
    }
}

// 采用同时填充前景色与背景色的方式，彻底消除文字刷新闪烁
void LCD_ShowChar(uint16_t x, uint16_t y, char num, uint16_t color, uint16_t bg_color) {
    uint8_t temp, t, t1;
    LCD_SetWindow(x, y, x + 7, y + 15);
    LCD_DC_SET();
    LCD_CS_CLR();

    num = num - ' ';
    /* 按行缓冲：将 128 次 SPI 单像素传输 → 16 次行传输，速度提升约 8 倍 */
    uint8_t row_buf[16];

    for(t = 0; t < 16; t++) {
        temp = asc2_1608[(uint8_t)num][t];
        uint8_t *p = row_buf;
        for(t1 = 0; t1 < 8; t1++) {
            if(temp & 0x80) {
                *p++ = color >> 8;
                *p++ = color & 0xFF;
            } else {
                *p++ = bg_color >> 8;
                *p++ = bg_color & 0xFF;
            }
            temp <<= 1;
        }
        HAL_SPI_Transmit(&hspi1, row_buf, 16, HAL_MAX_DELAY);
    }
    LCD_CS_SET();
}

void LCD_ShowString(uint16_t x, uint16_t y, char *p, uint16_t color, uint16_t bg_color) {
    while (*p != '\0') {
        if (x > 320 - 8) { x = 0; y += 16; }
        if (y > 240 - 16) { y = x = 0; }
        LCD_ShowChar(x, y, *p, color, bg_color);
        x += 8;
        p++;
    }
}
