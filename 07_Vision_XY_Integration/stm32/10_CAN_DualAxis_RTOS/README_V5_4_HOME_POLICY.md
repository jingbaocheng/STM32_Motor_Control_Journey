# Core V5.4 — Home completion policy separation

基线：你当前最新 `Core.zip`。

## 这版为什么改

V5.3 对 CAMERA_READY / 后续视觉定位的 Motion 完成判据是严格的：

    fresh encoder
    + 位置进入 tolerance
    + 位置基本停稳
    + 连续 150 ms
    -> DONE

这个策略适合精密定位，但之前也被套到了 Home 的每一个小 chunk：

    FAST    512
    BACK    512
    SLOW     64
    RELEASE 2048

所以 Home 会显得更碎、更慢，并且每个小段都多了一层普通定位语义。

V5.4 把两种语义明确拆开。

## 普通 / CAMERA_READY / 未来视觉纠偏

完全保留 V5.3：

    encoder到位 + stillness 150ms -> DONE

以下全部不回退：

- HSE 8 MHz / SYSCLK 168 MHz
- CAN 500 kbit/s
- SJW = 3 TQ
- Transmit FIFO Priority = Enable
- CAN DLC 检查
- ACK sticky counters
- pre-TX transaction baseline
- encoder freshness
- absolute residual 在 fresh PREPARE 中计算
- stall / timeout
- CameraReady 最终位置联锁
- 单 ControlTask

## Home

FAST / BACK / SLOW / RELEASE 使用 Home 专用 completion policy：

    当前 F4 transaction FAIL
        -> ERROR

    当前 F4 transaction STOPPED
        FAST/SLOW -> MOTION_RESULT_STOPPED
        BACK/RELEASE -> ERROR

    当前 F4 transaction COMPLETE
        -> 本 Home chunk DONE

然后 Home 状态机自己用 PG6/PG7 决定：

    FAST 搜索
    -> BACK 离开
    -> SLOW 再逼近
    -> 稳定 500 ms
    -> SetZero
    -> RELEASE
    -> 确认传感器释放

也就是说：

    Home 的权威 = 传感器边界
    CameraReady 的权威 = 编码器精密到位

## 重要说明

Home 这里虽然重新使用 F4 COMPLETE 做 chunk 完成，但不是旧版那个
`motor->move_ack == 0x02` 的单字节方案。

V5.3 已经把 F4 状态改成 sticky counters，并且每笔命令在 TX 前锁存
baseline，所以已消费的旧 COMPLETE 不会直接完成下一笔 Home chunk。

## 顺手修正

`AxisHome_Init()` 现在每次明确清零：

    back_retry_count
    release_retry_count

## 不改参数

    HOME_FAST_STEP = 512
    HOME_BACK_STEP = 512
    HOME_SLOW_STEP = 64
    HOME_RELEASE_STEP = 2048

    CAMERA_READY_X = 96304
    CAMERA_READY_Y = 36864

## Build marker

    g_build_id = 0x26091255

## 第一轮 Watch

先只保留 4 项：

    system_state
    g_home_x_stage
    Motion_X.error_code
    ((GPIOG->IDR >> 6) & 1U)

X Home 通过后，如果需要观察 Y，再换成 Y 对应项。

第一轮测试只跑：

    WAIT_HOME_KEY
    -> KEY0
    -> X Home
    -> Y Home
    -> CAMERA_READY

先不要同时改 Python、Home 参数、CAN 参数。
