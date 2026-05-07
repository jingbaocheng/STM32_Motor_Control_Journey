#include "homing_limit.h"

// ==========================================
// 私有状态变量
// ==========================================
HomingState_t Current_Homing_State = HOMING_IDLE;
HomingAxis_t  Current_Homing_Axis  = HOMING_AXIS_X;
static uint32_t Last_EXTI_Time     = 0;   // 限位开关 50ms 软件防抖时间戳

// ==========================================
// 动作 1：启动归零的“点火开关”
// ==========================================
void Homing_Init(void) {
    // 1. 强行拉下全局状态机的闸门
    CNC_State = SYS_HOMING;
    Current_Homing_Axis = HOMING_AXIS_X;

    // 【架构师的新补丁：开局物理安检！】
    // 先睁眼看看，X 轴限位是不是已经被挡住了？
    if (HAL_GPIO_ReadPin(X_Limit_GPIO_Port, X_Limit_Pin) == GPIO_PIN_SET) {
        // 如果已经被挡住了，千万别撞！直接挂倒挡退出来！
        Current_Homing_State = HOMING_BACKOFF;
    } else {
        // 如果没挡住，正常快撞寻找限位
        Current_Homing_State = HOMING_FAST_SEEK;
    }
}
// ==========================================
// 动作 2：归零特权模式下的无情发脉冲机器 (TIM3 中调用)
// ==========================================
void Homing_Step_Handler(void) {
    if (Current_Homing_State == HOMING_DONE || Current_Homing_State == HOMING_IDLE) {
        return;
    }

     // ==========================================
    // 状态机十字路口：根据当前状态，挂上不同的挡位和方向！
    // ==========================================
    if (Current_Homing_State == HOMING_FAST_SEEK) {
        // 快速负方向冲撞
        if (Current_Homing_Axis == HOMING_AXIS_X) {
            HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
        }
        TIM3->ARR = 2000;       // 较快速度（500 Hz）
    }
    else if (Current_Homing_State == HOMING_BACKOFF) {
        // 离开限位，正方向慢退
        if (Current_Homing_Axis == HOMING_AXIS_X) {
            HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_RESET);
        }
        TIM3->ARR = 8000;     // 中速（125 Hz）
    }
    else if (Current_Homing_State == HOMING_SLOW_CRAWL) {
        // 极慢二次贴近，提高重复定位精度
        if (Current_Homing_Axis == HOMING_AXIS_X) {
            HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
        }
        TIM3->ARR = 15000;    // 极慢（约 67 Hz）
    }

       
  // ==========================================
    // 暴力发脉冲：根据当前正在归零的轴，翻转对应的引脚！
    // ==========================================
   if (Current_Homing_Axis == HOMING_AXIS_X) {
        HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
        Delay_us(2);
        HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
        // 同步更新物理账本（虽然归零结束会清零，但中间状态保持一致）
        Physical_Current_X += (Current_Homing_State == HOMING_BACKOFF) ? 1 : -1;
    } else {
        HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_SET);
        Delay_us(2);
        HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_RESET);
        Physical_Current_Y += (Current_Homing_State == HOMING_BACKOFF) ? 1 : -1;
    }
}
   
// ==========================================
// EXTI 外部中断回调（限位开关被触发时切换状态）
// 【工业防弹版】：双边沿触发 + 严格读取物理电平验证！
//
// 【V6.1 关键修复】：原版本花括号嵌套错乱，Y 轴 else if 被误嵌进 X 轴
// if 块内部，导致 Y_Limit 中断永久不可达！本版本已彻底拉平结构。
// ==========================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick();

    // 50ms 软件防抖
    if (current_time - Last_EXTI_Time < 50) {
        return;
    }
    Last_EXTI_Time = current_time;

    // ==========================================
    // ----- X 轴限位处理 -----
    // ==========================================
    if (GPIO_Pin == X_Limit_Pin && Current_Homing_Axis == HOMING_AXIS_X)
    {
        // 【核心补丁】：读取真实的物理电平！
        // NPN+常闭：撞击(遮挡) = SET(1)， 离开(通透) = RESET(0)
        GPIO_PinState pin_state = HAL_GPIO_ReadPin(X_Limit_GPIO_Port, X_Limit_Pin);

        if (Current_Homing_State == HOMING_FAST_SEEK && pin_state == GPIO_PIN_SET) {
            // 撞上了！开始慢退
            Current_Homing_State = HOMING_BACKOFF;
        }
        else if (Current_Homing_State == HOMING_BACKOFF && pin_state == GPIO_PIN_RESET) {
            // 退出来了！开始极慢爬行二次定位
            Current_Homing_State = HOMING_SLOW_CRAWL;
        }
        else if (Current_Homing_State == HOMING_SLOW_CRAWL && pin_state == GPIO_PIN_SET) {
            // X 轴零点绝对确立！
            Physical_Current_X = 0;
            Planner_Current_X  = 0;

            // 完美接力交棒给 Y 轴
            Current_Homing_Axis = HOMING_AXIS_Y;

            // 【架构师的新补丁：Y 轴起步前的物理安检！】
            if (HAL_GPIO_ReadPin(Y_Limit_GPIO_Port, Y_Limit_Pin) == GPIO_PIN_SET) {
                Current_Homing_State = HOMING_BACKOFF;     // Y 轴已经在雷上，先退
            } else {
                Current_Homing_State = HOMING_FAST_SEEK;   // 正常找限位
            }
        }
    }

  // ==========================================
    // ----- Y 轴限位处理（独立平级 if，绝不嵌套！） -----
    // ==========================================
    else if (GPIO_Pin == Y_Limit_Pin && Current_Homing_Axis == HOMING_AXIS_Y)
    {
        // 同理，读取 Y 轴真实电平
        GPIO_PinState pin_state = HAL_GPIO_ReadPin(Y_Limit_GPIO_Port, Y_Limit_Pin);

        if (Current_Homing_State == HOMING_FAST_SEEK && pin_state == GPIO_PIN_SET) {
            Current_Homing_State = HOMING_BACKOFF;
        }
        else if (Current_Homing_State == HOMING_BACKOFF && pin_state == GPIO_PIN_RESET) {
            Current_Homing_State = HOMING_SLOW_CRAWL;
        }
        else if (Current_Homing_State == HOMING_SLOW_CRAWL && pin_state == GPIO_PIN_SET) {
            // Y 轴搞定，固化零点
            Physical_Current_Y = 0;
            Planner_Current_Y  = 0;

            // 整个系统归零完成！大功告成！
            Current_Homing_State = HOMING_DONE;
            CNC_State            = SYS_READY;
        }
    }
}