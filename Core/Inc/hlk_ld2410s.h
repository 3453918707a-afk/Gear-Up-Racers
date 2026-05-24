#ifndef __HLK_LD2410S_H
#define __HLK_LD2410S_H

#include "stm32f4xx.h"
#include "lcd.h"

// 帧定义
#define LD_FRAME_HEAD 0xF1F2F3F4
#define LD_FRAME_TAIL 0xF5F6F7F8

// 解析状态机枚举
typedef enum {
    STATE_IDLE,
    STATE_HEAD,
    STATE_LEN,
    STATE_DATA,
    STATE_TAIL
} ParseState_t;

// 传感器数据结构
typedef struct {
    uint8_t  target_status;
    uint16_t target_dist_cm;
    uint32_t gate_energy[16];
    uint8_t  is_new_data;
} LD2410S_Data_t;

// 雷达上下文结构体 (包含状态机和数据)
typedef struct {
    ParseState_t state;
    uint8_t      buf[128];
    uint16_t     data_len;
    uint16_t     count;
    uint32_t     head_reg;
    uint32_t     tail_reg;
    LD2410S_Data_t data;       // 解析后的最终数据
} LD2410S_Context_t;

// 暴露两个雷达对象供外部使用
extern LD2410S_Context_t radar1;
extern LD2410S_Context_t radar2;

// 函数声明
void LD2410S_Init(void);
void LD2410S_ParseByte(LD2410S_Context_t *ctx, uint8_t byte);
void LD2410S_SendCmd(UART_HandleTypeDef *huart, uint16_t cmd, uint8_t *params, uint16_t param_len);
void Draw_Energy_Spectrum(void);

#endif
