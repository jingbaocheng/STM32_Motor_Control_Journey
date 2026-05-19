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

### 04_Stepper_Interpolation_Lab (当前核心验证区 - V5.0 稳定版封板)

步进电机多轴插补与医疗级滑台控制沙盘。本项目已跨越极具挑战的软硬件联调期，成功建立工业级底层物理与逻辑防线。

*   **【硬件防线重构】**：彻底排查并避开商用开发板隐藏的 I2C 硬件上拉电阻“死亡陷阱”，重构绝对纯净的数字 GPIO 脉冲矩阵 (PB6/PB7, PE5/PE6)。引入闭环驱动器 FOC 峰值电流硬性降额 (400mA~800mA) 保护机制，彻底杜绝电机与驱动板热失控。
*   **【工业级解析引擎】**：抛弃庞大且脆弱的 `sscanf`，手撕基于 `strstr` + `atoi` 的轻量级、非阻塞字符串解析器。完美解决无符号整数下溢出 (`Unsigned Underflow`) 坐标跳变 Bug；引入“位置继承 (Hold Position)”防御逻辑，彻底消灭单轴指令缺省导致的自动撞原点致命 Bug。
*   **【三段式防抖归零 (Homing)】**：废弃“一碰就停”的业余逻辑，构建基于系统滴答定时器 (SysTick) 的非阻塞三段式状态机 (Fast-Approach -> Back-Off -> Creep-Approach)。内置 5ms 软件低通滤波器，完美免疫电机运行时的 EMI 高频电磁干扰假触发，实现医疗级绝对零点锁定。
*   **Software_Python (上位机)**：基于 CustomTkinter 与 Matplotlib 搭建现代工业风上位机。采用 Threading 多线程分离 UI 与串口读写，实现防阻塞并发通信与实时轨迹雷达渲染 *(注：目前配合 V5.0 单片机内核暂停联调，等待 V6.0 CNC 插补内核完成后进行流式下发适配)*。

> **🚀 Next Milestone (进行中)**：开辟 `feature/v6-ring-buffer` 分支，全面进军 CNC 运动控制内核。抛弃即时阻塞执行，引入 **Ring Buffer (环形缓冲区)** 解耦串口解析与定时器插补；引入 **Trapezoidal Profile (梯形加减速)** 彻底消除高频启动时的机械顿挫与闭环过载对抗。


### 05_Servo_Control_Lab (Pre-Research)
交流伺服电机与无刷电机 (BLDC) 技术预研。规划包含脉冲/通信控制模式验证及 FOC 算法基础。

### 06_Medical_XY_Platform_Final
最终大满贯工程：医疗级精密扫描云台。规划包含自研 PCB 驱动底板、极简高效的双轴插补固件及联动视觉交互的最终版上位机。

## 技术栈 (Tech Stack)
- 微控制器: STM32F1/F4 Series
- 固件开发: C (Keil MDK, STM32 HAL/LL)
- 上位机开发: Python 3.10 (CustomTkinter, pyserial, matplotlib)
- 架构亮点: 非阻塞轮询状态机、双端缓冲防溢出、全双工无锁串口通信
