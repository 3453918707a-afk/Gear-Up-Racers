#include "hlk_ld2410s.h"
#include "usart.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;

// 实例化两个雷达对象
LD2410S_Context_t radar1 = {0};
LD2410S_Context_t radar2 = {0};

// 局部显存 (高180, 宽160)
static uint16_t graph_buf[180][160];

// 💡 调节此系数可以控制波形在屏幕上的高度（如果曲线老是顶格，请调小此值，如 0.8f）
#define ENERGY_SCALE_FACTOR   0.01f

// 在内存中画线算法
static void DrawLineToBuf(int x1, int y1, int x2, int y2, uint16_t color) {
    uint16_t swap_color = (color >> 8) | (color << 8);
    int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int err = dx + dy, e2;

    for (;;) {
        if (x1 >= 0 && x1 < 160 && y1 >= 0 && y1 < 180) {
            graph_buf[y1][x1] = swap_color;
        }
        if (x1 == x2 && y1 == y2) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
}

// 多实例解析器：传入对应的雷达上下文指针
void LD2410S_ParseByte(LD2410S_Context_t *ctx, uint8_t byte) {
    switch (ctx->state) {
        case STATE_IDLE:
            if (byte == 0xF4) {
                ctx->head_reg = byte; ctx->state = STATE_HEAD; ctx->count = 1;
            }
            break;
        case STATE_HEAD:
            ctx->head_reg |= ((uint32_t)byte << (8 * ctx->count));
            ctx->count++;
            if (ctx->count == 4) {
                if (ctx->head_reg == 0xF1F2F3F4) { ctx->state = STATE_LEN; ctx->count = 0; }
                else { ctx->state = STATE_IDLE; }
            }
            break;
        case STATE_LEN:
            ctx->buf[ctx->count++] = byte;
            if (ctx->count == 2) {
                ctx->data_len = ctx->buf[0] | (ctx->buf[1] << 8);
                ctx->state = STATE_DATA; ctx->count = 0;
            }
            break;
        case STATE_DATA:
            ctx->buf[ctx->count++] = byte;
            if (ctx->count >= ctx->data_len || ctx->count >= 128) {
                ctx->state = STATE_TAIL; ctx->count = 0; ctx->tail_reg = 0;
            }
            break;
        case STATE_TAIL:
            ctx->tail_reg |= ((uint32_t)byte << (8 * ctx->count));
            ctx->count++;
            if (ctx->count == 4) {
                if (ctx->tail_reg == 0xF5F6F7F8) {
                    if ((ctx->buf[0] == 0x01 || ctx->buf[0] == 0x02) && ctx->data_len >= 6) {
                        ctx->data.target_status = ctx->buf[1];
                        ctx->data.target_dist_cm = ctx->buf[2] | ((uint16_t)ctx->buf[3] << 8);

                        if (ctx->data_len >= 70) {
                            for (int g = 0; g < 16; g++) {
                                int base_idx = 6 + (g * 4);
                                ctx->data.gate_energy[g] = (uint32_t)ctx->buf[base_idx]         |
                                                           ((uint32_t)ctx->buf[base_idx + 1] << 8)  |
                                                           ((uint32_t)ctx->buf[base_idx + 2] << 16) |
                                                           ((uint32_t)ctx->buf[base_idx + 3] << 24);
                            }
                        }
                        ctx->data.is_new_data = 1;
                    }
                }
                ctx->state = STATE_IDLE;
            }
            break;
    }
}

// 发送命令
void LD2410S_SendCmd(UART_HandleTypeDef *huart, uint16_t cmd, uint8_t *params, uint16_t param_len) {
    uint8_t head[] = {0xFD, 0xFC, 0xFB, 0xFA};
    uint8_t tail[] = {0x04, 0x03, 0x02, 0x01};
    uint16_t frame_len = param_len + 2;

    HAL_UART_Transmit(huart, head, 4, 100);
    HAL_UART_Transmit(huart, (uint8_t*)&frame_len, 2, 100);
    HAL_UART_Transmit(huart, (uint8_t*)&cmd, 2, 100);
    if (params && param_len > 0) { HAL_UART_Transmit(huart, params, param_len, 100); }
    HAL_UART_Transmit(huart, tail, 4, 100);
}

// Catmull-Rom 三次样条插值
static float CatmullRomInterpolate(float p0, float p1, float p2, float p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    return 0.5f * ((2.0f * p1) +
                   (-p0 + p2) * t +
                   (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

/* 雷达数据快照：画图前冻结当前帧，避免SPI传输期间ISR更新导致帧撕裂 */
typedef struct {
    uint16_t target_dist_cm;
    uint8_t  target_status;
    uint32_t gate_energy[16];
} RadarSnapshot_t;

/* ==================== 优化后的雷达图刷屏算法（分时分批+强行延时歇息） ==================== */

static void DrawRadarGraph(RadarSnapshot_t *snap, uint16_t x_offset, uint16_t y_offset, uint16_t text_x, const char* name, uint16_t *last_dist) {
    // 1. 刷新文本（无目标时显示---）
    {
        uint16_t display_val = (snap->target_status != 0) ? snap->target_dist_cm : 0xFFFF;
        if (display_val != *last_dist) {
            char dist_str[32];
            if (snap->target_status != 0) {
                sprintf(dist_str, "%s:%03dcm ", name, snap->target_dist_cm);
            } else {
                sprintf(dist_str, "%s:---   ", name);
            }
            LCD_ShowString(text_x, 10, dist_str, WHITE, BLACK);
            *last_dist = display_val;
        }
    }

    // 2. 清空绘制缓冲区
    memset(graph_buf, 0, sizeof(graph_buf));

    // 3. 绘制平滑雷达能量谱曲线
    int last_x = 0;
    int last_y = (int)(snap->gate_energy[0] * ENERGY_SCALE_FACTOR);
    if (last_y > 179) last_y = 179;
    if (last_y < 0) last_y = 0;

    for (int px = 1; px < 160; px++) {
        float gate_index = (float)px * 15.0f / 159.0f;
        int i = (int)gate_index;
        float t = gate_index - i;

        if (i >= 15) { i = 14; t = 1.0f; }

        float p0 = (i - 1 >= 0) ? (float)snap->gate_energy[i - 1] : (float)snap->gate_energy[0];
        float p1 = (float)snap->gate_energy[i];
        float p2 = (i + 1 <= 15) ? (float)snap->gate_energy[i + 1] : (float)snap->gate_energy[15];
        float p3 = (i + 2 <= 15) ? (float)snap->gate_energy[i + 2] : (float)snap->gate_energy[15];

        float interpolated_y = CatmullRomInterpolate(p0, p1, p2, p3, t);

        int current_y = (int)(interpolated_y * ENERGY_SCALE_FACTOR);
        if (current_y > 179) current_y = 179;
        if (current_y < 0) current_y = 0;

        DrawLineToBuf(last_x, 179 - last_y, px, 179 - current_y, GREEN);

        last_x = px;
        last_y = current_y;
    }

    // 4. 分块SPI传输，每块后短暂延时避免供电轨塌陷
    uint8_t *ptr = (uint8_t*)graph_buf;
    uint32_t remaining = sizeof(graph_buf);

    LCD_SetWindow(x_offset, y_offset, x_offset + 159, y_offset + 179);
    LCD_DC_SET();

    while (remaining > 0) {
        uint32_t chunk = (remaining > 2880) ? 2880 : remaining;

        LCD_CS_CLR();
        HAL_SPI_Transmit(&hspi1, ptr, chunk, HAL_MAX_DELAY);
        LCD_CS_SET();

        ptr += chunk;
        remaining -= chunk;

        HAL_Delay(1);
    }
}

// 无阻塞刷新：快照冻结数据后绘制，消除帧撕裂
void Draw_Energy_Spectrum(void) {
    static uint16_t last_dist_1 = 0xFFFF;
    static uint16_t last_dist_2 = 0xFFFF;

    /* 两个雷达都没有新数据 → 跳过整屏重绘，主循环从 ~5Hz 恢复到 ~100Hz */
    if (!radar1.data.is_new_data && !radar2.data.is_new_data) return;

    RadarSnapshot_t snap1, snap2;

    __disable_irq();
    snap1.target_dist_cm = radar1.data.target_dist_cm;
    snap1.target_status   = radar1.data.target_status;
    memcpy(snap1.gate_energy, radar1.data.gate_energy, sizeof(snap1.gate_energy));
    uint8_t r1_new = radar1.data.is_new_data;
    radar1.data.is_new_data = 0;
    snap2.target_dist_cm = radar2.data.target_dist_cm;
    snap2.target_status   = radar2.data.target_status;
    memcpy(snap2.gate_energy, radar2.data.gate_energy, sizeof(snap2.gate_energy));
    uint8_t r2_new = radar2.data.is_new_data;
    radar2.data.is_new_data = 0;
    __enable_irq();

    if (r1_new) DrawRadarGraph(&snap1, 0,   40, 10,  "R1", &last_dist_1);
    if (r2_new) DrawRadarGraph(&snap2, 160, 40, 170, "R2", &last_dist_2);
}

void LD2410S_Init(void) {
    uint8_t enable_val[] = {0x01, 0x00};
    uint8_t mode_val[] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
    // 0x0060: 动检最大门=3, 静检最大门=3, 无人持续时间=0s
    uint8_t dist_cfg[] = {
        0x00, 0x00,               // word: 动检最大距离门
        0x03, 0x00, 0x00, 0x00,   // value: 3 (~2.25m)
        0x01, 0x00,               // word: 静检最大距离门
        0x03, 0x00, 0x00, 0x00,   // value: 3
        0x02, 0x00,               // word: 无人持续时间
        0x00, 0x00, 0x00, 0x00    // value: 0s
    };

    HAL_Delay(1000);

    LD2410S_SendCmd(&huart2, 0x00FF, enable_val, 2);
    HAL_Delay(80);
    LD2410S_SendCmd(&huart5, 0x00FF, enable_val, 2);
    HAL_Delay(80);

    LD2410S_SendCmd(&huart2, 0x007A, mode_val, 6);
    HAL_Delay(80);
    LD2410S_SendCmd(&huart5, 0x007A, mode_val, 6);
    HAL_Delay(80);

    LD2410S_SendCmd(&huart2, 0x0060, dist_cfg, 18);
    HAL_Delay(80);
    LD2410S_SendCmd(&huart5, 0x0060, dist_cfg, 18);
    HAL_Delay(80);

    LD2410S_SendCmd(&huart2, 0x00FE, NULL, 0);
    HAL_Delay(80);
    LD2410S_SendCmd(&huart5, 0x00FE, NULL, 0);
    HAL_Delay(80);
}
