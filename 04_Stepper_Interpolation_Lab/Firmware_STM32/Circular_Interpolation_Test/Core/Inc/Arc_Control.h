#ifndef __ARC_CONTROL_H
#define __ARC_CONTROL_H

#include "main.h"

// 定义圆弧方向
typedef enum {
    DIR_CW  = 0, // 顺时针
    DIR_CCW = 1  // 逆时针
} Arc_Dir_t;

// 圆弧插补参数结构体
typedef struct {
    int32_t startX, startY; // 起点绝对坐标
    int32_t endX, endY;     // 终点绝对坐标
    int32_t I, J;           // 圆心相对起点的偏移量
    Arc_Dir_t dir;          // 顺逆时针
    uint32_t stepDelay;     // 脉冲间隔(控制速度)
} Arc_Task_t;

// 函数声明：使用指针传递结构体
void Arc_Interpolate_Process(Arc_Task_t *pTask);

#endif
