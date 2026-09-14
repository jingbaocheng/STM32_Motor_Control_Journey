# V5.5 最终收尾版

这版不再扩架构，只做最后两项 Home 收尾修改。

## 1. PG6 / PG7 Home 软件消抖：30 ms

- 原始 GPIO 仍在每个 ControlTask 周期读取；
- 同一电平连续稳定 30 ms 才允许 Home 状态机承认 ACTIVE / RELEASED；
- Home 启动时也先等首个稳定电平，再允许第一笔运动；
- 目标是压掉“还没到真正限位就误触发，导致 FAST -> BACK -> SLOW 来回”的现象。

## 2. Home chunk 间隔：200 ms -> 50 ms

不改任何 Home 步长、速度和方向，只去掉每个小 chunk 之间过长的人为停顿。

## 保留

- 单 ControlTask：Motion_X -> Motion_Y -> System_Task -> osDelay(10)
- HSE 8 MHz / 168 MHz
- CAN 500 kbit/s，SJW=3
- Transmit FIFO Priority = Enable
- DLC 检查、HAL_BUSY 重试
- F4 sticky counters / pre-TX transaction baseline
- encoder freshness
- SetZero 新鲜度检查
- Home 专用 completion policy
- CameraReady 严格 encoder + stillness
- CameraReady 最终位置联锁

## 不再改

- FAST 512
- BACK 512
- SLOW 64
- RELEASE 2048
- Home 速度与方向
- CAMERA_READY_X = 96304
- CAMERA_READY_Y = 36864
- Python / OpenCV

## Build marker

    g_build_id = 0x26091256

## 项目截止线

只做 3 次完整回归：

    上电/Reset -> WAIT_HOME_KEY -> KEY0
    -> X Home -> Y Home -> CAMERA_READY -> PB0

- 3/3 正常：立即冻结 STM32 固件，转项目总结/面试材料，并开始第二个项目。
- 仍有偶发异常：不再做新的 RTOS/CAN/状态机重构，记录为已知问题；
  后续若要工程化，只查 Home 传感器物理信号、机械余程和 CAN 物理层。

## Watch

正常测试只留：

    system_state
    g_home_x_stage
    Motion_X.error_code
    ((GPIOG->IDR >> 6) & 1U)

需要观察 Y 时再临时换 Y。
