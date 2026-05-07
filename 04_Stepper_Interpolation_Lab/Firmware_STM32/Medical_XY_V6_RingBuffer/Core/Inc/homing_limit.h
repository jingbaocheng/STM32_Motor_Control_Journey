#ifndef HOMING_LIMIT_H
#define HOMING_LIMIT_H
#include "main.h"       // 必须包含，为了认识 GPIO_PIN 等硬件名字
#include "motor_core.h" // 必须包含，为了认识全局挂挡杆 CNC_State
// ==========================================
// 1. 归零子状态机枚举 
// ==========================================
typedef enum {
    HOMING_IDLE       = 0, // 待命
    HOMING_FAST_SEEK,      // 快撞限位（高速负方向找零点）
    HOMING_BACKOFF,        // 离开限位（低速正方向退几毫米）
    HOMING_SLOW_CRAWL,     // 缓慢爬行（极慢二次贴近，提精度）
    HOMING_DONE            // 完成
} HomingState_t;

typedef enum {
    HOMING_AXIS_X = 0,
    HOMING_AXIS_Y
    // HOMING_AXIS_Z   // 【Z 轴预留】未来扩展时取消注释
} HomingAxis_t;

extern HomingAxis_t Current_Homing_Axis; // 记录当前正在归零谁
extern HomingState_t Current_Homing_State;
void Homing_Init(void);
void Homing_Step_Handler(void);
void Delay_us(uint32_t us);
#endif