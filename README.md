# Gear Up Racers

基于 STM32F407 的智能循迹避障小车，集成双毫米波雷达、超声波测距、LoRa 无线通信与 TFT 实时可视化。

> **作者**：Shen Yang、Zhou Xuanyu、Cui Chenhe、Wang Yiyang、Wang Jiacheng

## 硬件配置

| 模块 | 型号 | 接口 | 用途 |
|------|------|------|------|
| MCU | STM32F407ZGTx | — | 主控 @ 168MHz |
| 循迹传感器 | 8 路红外 | I2C2 (0x5D) | 黑线检测与 PID 循迹 |
| 毫米波雷达 ×2 | HLK-LD2410S | USART2 / UART5 | 障碍物距离 + 能量谱 |
| 超声波 | US-100 | GPIO + EXTI | 近距离测距 |
| 无线通信 | LoRa | USART3 | 定向报文发送 |
| 显示屏 | 1.8" TFT (ST7735) | SPI1 | 路线绘制 / 雷达图谱 |
| 电机驱动 | L298N | TIM4 PWM | 左右轮差速控制 |

## 软件架构

```
main.c
├── Obstacle_ManagerLoop()     状态机调度器
│   ├── CAR_MODE_NORMAL_TRACKING   → Track_Control()
│   ├── CAR_MODE_RADAR_ON_TRACKING → 丁字路口检测 + 近场避障 + Track_Control()
│   └── CAR_MODE_AVOIDING          → 差速旋转 + 黑线恢复检测
├── HCSR04_Task()               超声波测距 + LoRa 上报
├── Route_Draw_Task()           实时路线绘制 (50ms 刷新)
└── Draw_Energy_Spectrum()      双雷达能量谱 (100ms 交替)
```

## 核心特性

- **时间感知 PID**：利用 DWT 周期计数器测量实际控制间隔，按 dt 自动缩放积分和微分项，消除屏幕绘图造成的控制频率抖动
- **平滑重入**：硬转向切回 PID 时强制 `LastError = Error`，消除微分冲击（Derivative Kick）
- **EMA 滤波**：循迹误差与雷达距离均经指数移动平均滤波，抑制瞬时噪声
- **Slew Rate 限幅**：PWM 输出单次跳变上限 80，防止电机过冲震荡
- **雷达 0 值修正**：LD2410S 超量程返回 0 时映射为安全大值，避免误触发避障
- **避障闭环退出**：旋转 400ms 后实时检测黑线恢复，提前结束多余旋转
- **非阻塞 LoRa**：UART 中断发送，不冻结主循环
- **SPI 缓冲批量传输**：LCD 字符 128→16 次 SPI/字，刷屏 57KB 拆为 512B 小块

## 快速开始

1. 用 STM32CubeIDE 打开项目根目录（`.cproject` / `.ioc`）
2. `Project → Build All` 编译
3. 通过 ST-Link 烧录到 STM32F407 开发板
4. 上电后小车默认进入**普通循迹模式**，可通过 `Obstacle_ActivateRadar()` 切换到雷达避障模式

## 目录结构

```
├── Core/
│   ├── Inc/         头文件 (track.h, obstacle.h, lcd.h, ...)
│   └── Src/         源文件 (main.c, track.c, obstacle.c, ...)
├── Drivers/         HAL 库 + CMSIS
├── .cproject        CubeIDE 工程配置
├── .ioc             CubeMX 引脚配置
└── LICENSE
```

## 开源协议

MIT License © 2026 Shen Yang, Zhou Xuanyu, Cui Chenhe, Wang Yiyang, Wang Jiacheng
