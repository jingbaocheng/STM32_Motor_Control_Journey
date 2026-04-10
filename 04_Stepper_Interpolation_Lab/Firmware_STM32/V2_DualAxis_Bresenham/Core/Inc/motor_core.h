#ifndef __MOTOR_CORE_H__
#define __MOTOR_CORE_H__

#include "main.h"  // 包含底层的引脚定义

// 状态机枚举
typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_ACCEL,
    MOTOR_RUN,
    MOTOR_DECEL,
    MOTOR_STOP
} MotorState_t;

// 系统核心结构体
typedef struct {
    MotorState_t State;         
    uint32_t Target_X;
    uint32_t Target_Y;
    uint32_t Current_X;
    uint32_t Current_Y;
    uint32_t dx_total;          
    uint32_t dy_total;          
    uint8_t  is_X_Boss;         
    int32_t  Error_Bucket;      
    
    uint32_t Main_Steps_Total;  
    uint32_t Main_Steps_Current;
    uint32_t Current_ARR;       
} DualAxisSystem_t;

// 暴露全局变量和函数，让别的文件也能用
extern DualAxisSystem_t XY_Sys;

void Motor_Load_Command(int32_t target_x, int32_t target_y); // 装填弹药函数
void Motor_TIM_Interrupt_Handler(void);                      // 中断处理函数

#endif
