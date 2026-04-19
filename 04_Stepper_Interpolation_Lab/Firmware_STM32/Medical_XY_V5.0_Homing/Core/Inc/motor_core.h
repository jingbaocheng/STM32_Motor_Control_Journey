#ifndef __MOTOR_CORE_H__
#define __MOTOR_CORE_H__

#include "main.h"  // 包含底层的引脚定义

// 状态机枚举
typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_ACCEL,
    MOTOR_RUN,
    MOTOR_DECEL,
    MOTOR_STOP,
     // --- 新增：三段式归零专属状态 ---
    MOTOR_HOMING_FAST,   // 快撞
    MOTOR_HOMING_BACK,   // 慢退
    MOTOR_HOMING_CREEP   // 极速爬行
} MotorState_t;
// 2. 新增一个枚举，记录当前在归零哪个轴
typedef enum {
    HOME_NONE = 0,
    HOME_X_AXIS,
    HOME_Y_AXIS,
      HOME_ALL 
} HomingTarget_t;
// 系统核心结构体
typedef struct {
    MotorState_t State; 
    int8_t   dir_x;             // X轴方向：1为正向，-1为反向
    int8_t   dir_y;             // Y轴方向：1为正向，-1为反向
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
 HomingTarget_t Homing_Axis;  // 正在归零的轴    
    uint8_t Homing_Seq_Flag;    // 新增：0=单轴归零，1=正在执行双轴顺序归零
} DualAxisSystem_t;

// 暴露全局变量和函数，让别的文件也能用
extern volatile DualAxisSystem_t XY_Sys; 

void Motor_Load_Command(int32_t target_x, int32_t target_y); // 装填弹药函数
void Motor_TIM_Interrupt_Handler(void);                      // 中断处理函数
void Motor_Emergency_Stop(void);                             //急停拔插头函数
void Motor_Homing_Handler(void);                             //三段式状态机函数
void Motor_Start_Homing(HomingTarget_t axis);
#endif
