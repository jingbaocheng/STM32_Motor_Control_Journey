#include "vofa_plus.h"
#include "usart.h"
#include "foc_app.h"
/* 在 Core/Src/vofa_plus.c 文件的末尾补充 */
void VOFA_SendMotorData(foc_control_t *motor) {
    float debug_buf[4];
    
    // 将该电机的关键闭环变量抽稀打包
    debug_buf[0] = motor->target_velocity;   // 通道 0：目标速度 (rad/s)
    debug_buf[1] = motor->filtered_velocity; // 通道 1：实际滤波速度 (rad/s)
    debug_buf[2] = motor->Iq;                // 通道 2：反馈力矩电流 (A)
    debug_buf[3] = motor->Uq;                // 通道 3：输出控制电压 (V)
    
    // 调用现有的 JustFloat 协议流发送
    VOFA_SendData(debug_buf, 4);
}
