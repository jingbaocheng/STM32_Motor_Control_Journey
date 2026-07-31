#ifndef __MKS_MOTOR_H
#define __MKS_MOTOR_H

#include "main.h"

// 面向对象：用和说明书绝对对齐的中文具名属性定义电机控制卡
typedef struct {
    uint32_t tx_id;              // 主控发送给该电机的标准 ID (X轴=0x01, Y轴=0x02)
    uint32_t rx_id;              // 电机回传给主控的反馈 ID (X轴=0x01, Y轴=0x02)
    
    uint8_t  is_enabled;         // 本地影子状态：电机当前的使能状态 (1:锁死, 0:释放)
 volatile int64_t  actual_encoder_val; // 核心数据：严格对齐说明书 31H 协议的 48位有符号多圈编码器值
} MKS_Motor_t;

// 声明全局双轴电机房间，方便 main.c 和硬件中断随时提调
extern MKS_Motor_t Motor_X;
extern MKS_Motor_t Motor_Y;
extern CAN_HandleTypeDef hcan1;

// 纯净分布式总线驱动 API 声明
void MKS_Motor_System_Init(void);
void STM32_bxCAN_Filter_Config(void);
void MKS_Set_Enable_State(MKS_Motor_t *motor, uint8_t enable_state);
void MKS_Parse_Feedback_Payload(uint32_t std_id, uint8_t *rx_data);
void MKS_Read_Absolute_Position(MKS_Motor_t *motor);
void MKS_Set_Work_Mode(MKS_Motor_t *motor, uint8_t mode);
void MKS_Move_Relative_Axis(MKS_Motor_t *motor, uint16_t speed, uint8_t acc, int32_t rel_axis);
#endif

