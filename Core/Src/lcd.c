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

    uint8_t data[2] = {color >> 8, color & 0xFF};
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
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

void LCD_Clear(uint16_t color) {
    LCD_FillRect(0, 0, 320, 240, color);
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
    for(t = 0; t < 16; t++) {
        temp = asc2_1608[(uint8_t)num][t];
        for(t1 = 0; t1 < 8; t1++) {
            if(temp & 0x80) {
                uint8_t d[2] = {color >> 8, color & 0xFF};
                HAL_SPI_Transmit(&hspi1, d, 2, HAL_MAX_DELAY);
            } else {
                uint8_t d[2] = {bg_color >> 8, bg_color & 0xFF};
                HAL_SPI_Transmit(&hspi1, d, 2, HAL_MAX_DELAY);
            }
            temp <<= 1;
        }
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

void LCD_ShowCharScaled(uint16_t x, uint16_t y, char num, uint8_t scale,
                        uint16_t color, uint16_t bg_color) {
    if (scale == 0U) {
        return;
    }

    uint8_t index = (uint8_t)(num - ' ');
    if (index >= 95U) {
        index = 0U;
    }

    for (uint8_t row = 0; row < 16; row++) {
        uint8_t bits = asc2_1608[index][row];
        for (uint8_t col = 0; col < 8; col++) {
            uint16_t out = (bits & (0x80 >> col)) ? color : bg_color;
            LCD_FillRect(x + col * scale, y + row * scale, scale, scale, out);
        }
    }
}

void LCD_ShowStringScaled(uint16_t x, uint16_t y, char *p, uint8_t scale,
                          uint16_t color, uint16_t bg_color) {
    while (*p != '\0') {
        LCD_ShowCharScaled(x, y, *p, scale, color, bg_color);
        x += 8U * scale;
        p++;
    }
}

void LCD_ShowHz16(uint16_t x, uint16_t y, uint16_t unicode, uint16_t color, uint16_t bg_color) {
    const unsigned char *glyph = NULL;

    for (uint32_t i = 0; i < HZ16_TABLE_SIZE; i++) {
        if (hz16_table[i].unicode == unicode) {
            glyph = hz16_table[i].data;
            break;
        }
    }

    LCD_SetWindow(x, y, x + 15, y + 15);
    LCD_DC_SET();
    LCD_CS_CLR();

    uint8_t row_buf[32];
    for (uint8_t row = 0; row < 16; row++) {
        uint8_t byte0 = glyph ? glyph[row * 2] : ((row == 0 || row == 15) ? 0xFF : 0x80);
        uint8_t byte1 = glyph ? glyph[row * 2 + 1] : ((row == 0 || row == 15) ? 0xFF : 0x01);
        uint8_t *p = row_buf;

        for (uint8_t col = 0; col < 16; col++) {
            uint8_t mask = (col < 8) ? (0x80 >> col) : (0x80 >> (col - 8));
            uint8_t on = (col < 8) ? (byte0 & mask) : (byte1 & mask);
            uint16_t out = on ? color : bg_color;
            *p++ = out >> 8;
            *p++ = out & 0xFF;
        }
        HAL_SPI_Transmit(&hspi1, row_buf, 32, HAL_MAX_DELAY);
    }
    LCD_CS_SET();
}

void LCD_ShowHz16Line(uint16_t x, uint16_t y, const uint16_t *text, uint8_t len,
                      uint16_t color, uint16_t bg_color) {
    for (uint8_t i = 0; i < len; i++) {
        LCD_ShowHz16(x + i * 18, y, text[i], color, bg_color);
    }
}

void LCD_ShowHz32(uint16_t x, uint16_t y, uint16_t unicode, uint16_t color, uint16_t bg_color) {
    const unsigned char *glyph = NULL;

    for (uint32_t i = 0; i < HZ16_TABLE_SIZE; i++) {
        if (hz16_table[i].unicode == unicode) {
            glyph = hz16_table[i].data;
            break;
        }
    }

    for (uint8_t row = 0; row < 16; row++) {
        uint8_t byte0 = glyph ? glyph[row * 2] : ((row == 0 || row == 15) ? 0xFF : 0x80);
        uint8_t byte1 = glyph ? glyph[row * 2 + 1] : ((row == 0 || row == 15) ? 0xFF : 0x01);

        for (uint8_t col = 0; col < 16; col++) {
            uint8_t mask = (col < 8) ? (0x80 >> col) : (0x80 >> (col - 8));
            uint8_t on = (col < 8) ? (byte0 & mask) : (byte1 & mask);
            LCD_FillRect(x + col * 2U, y + row * 2U, 2, 2, on ? color : bg_color);
        }
    }
}

void LCD_ShowHz32Line(uint16_t x, uint16_t y, const uint16_t *text, uint8_t len,
                      uint16_t color, uint16_t bg_color) {
    for (uint8_t i = 0; i < len; i++) {
        LCD_ShowHz32(x + i * 36U, y, text[i], color, bg_color);
    }
}
