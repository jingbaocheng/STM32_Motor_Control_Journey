#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include <stdint.h>
#include "mks_motor.h"

typedef enum
{
    MOTION_IDLE = 0,     // 空闲，当前没有运动任务
    MOTION_PREPARE,      // 准备阶段，先连续读几次当前位置
    MOTION_MOVING,       // 运动中，持续读位置并计算误差
    MOTION_DONE,         // 已到位
    MOTION_ERROR         // 异常状态
} MotionState_t;

typedef struct
{
    MKS_Motor_t *motor;          // 绑定的 MKS 电机对象

    MotionState_t state;         // 当前运动状态

    int64_t start_pos;           // 开始运动前的位置
    int64_t target_pos;          // 目标位置 = start_pos + move_axis
    int64_t actual_pos;          // 当前实际位置
    int64_t last_pos;            // 上一次读取到的位置
    int64_t pos_error;           // 位置误差 = target_pos - actual_pos
    int64_t pos_delta;           // 本次位置变化量 = actual_pos - last_pos

    int32_t move_axis;           // 本次相对运动位移
    uint16_t speed;              // 本次运动速度
    uint8_t acc;                 // 本次运动加速度档位

    int64_t tolerance;           // 到位误差容忍范围
    int64_t still_threshold;     // 静止判断阈值，位置变化小于它认为基本停稳

    uint32_t last_read_tick;     // 上一次读取位置的时间戳
    uint8_t prepare_count;       // 准备阶段读取次数计数
    uint8_t motion_done;         // 到位完成标志，1 表示完成

} MotionController_t;


void Motion_Init(MotionController_t *motion, MKS_Motor_t *motor);
void Motion_StartRelative(MotionController_t *motion, int32_t move_axis, uint16_t speed, uint8_t acc);
void Motion_Task(MotionController_t *motion);

#endif
