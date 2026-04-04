# STM32 Motor Control Journey

本项目记录了从底层基础外设到多轴电机联动控制的完整研发过程。
核心目标：搭建一套高精度的医疗级 XY 扫描云台控制系统，涵盖下位机驱动算法与上位机并发通信架构。

## 目录结构 (Directory Structure)

### 01_Basic_Peripherals
基础底层外设驱动实战，包含 GPIO (Blink)、定时器 (TIM)、PWM 波形生成以及 USART 基础收发。

### 02_DC_Motor_PID
直流有刷减速电机的闭环控制实战，包含增量式/位置式 PID 算法调参验证，以及梯形轨迹规划消除极低速稳态抖动。

### 03_Stepper_Basic
步进电机基础脉冲驱动，主从定时器级联与基础开环控制。

### 04_Stepper_Interpolation_Lab (当前核心验证区)
步进电机多轴插补与滑台控制沙盘。
- Firmware_STM32: 抛弃 HAL 库死锁缺陷，使用寄存器直写实现全双工收发；手写轻量级解析器，彻底消除内存溢出隐患；基于时间戳的非阻塞状态机流转。
- Software_Python: 基于 CustomTkinter 与 Matplotlib 搭建的现代工业风上位机。采用 Threading 多线程分离 UI 与串口读写，实现防阻塞并发通信与实时轨迹雷达渲染。

### 05_Servo_Control_Lab (Pre-Research)
交流伺服电机与无刷电机 (BLDC) 技术预研。规划包含脉冲/通信控制模式验证及 FOC 算法基础。

### 06_Medical_XY_Platform_Final
最终大满贯工程：医疗级精密扫描云台。规划包含自研 PCB 驱动底板、极简高效的双轴插补固件及联动视觉交互的最终版上位机。

## 技术栈 (Tech Stack)
- 微控制器: STM32F1/F4 Series
- 固件开发: C (Keil MDK, STM32 HAL/LL)
- 上位机开发: Python 3.10 (CustomTkinter, pyserial, matplotlib)
- 架构亮点: 非阻塞轮询状态机、双端缓冲防溢出、全双工无锁串口通信
