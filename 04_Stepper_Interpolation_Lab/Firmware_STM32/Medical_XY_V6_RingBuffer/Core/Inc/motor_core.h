#ifndef __MOTOR_CORE_H__
#define __MOTOR_CORE_H__

#include "main.h"  // 包含底层的引脚定义

#define MOTION_BUFFER_SIZE  16  // 缓冲区大小

typedef enum {
    SYS_UNHOMED = 0, // 刚上电，瞎子状态 (P挡，锁死厨房)
    SYS_HOMING  = 1, // 正在执行归零动作 (特殊物理接管状态)
    SYS_READY   = 2, // 归零完毕，集装箱为空，随时待命 (N挡)
    SYS_RUNNING = 3, // 正在极速吃便当，执行插补与加减速 (D挡)
    SYS_ERROR   = 4  // 发生物理撞击或急停报警 (锁死报警)
} SystemState_t;

// ==========================================
// 单条运动指令块（便当盒）
// 注意：Z 轴字段已经预留，现在 Delta_Z = 0、Dir_Z = 0 表示不动
//       未来扩展时只需在 Planner_Add_Block 里赋值即可
// ==========================================
typedef struct {
    int32_t  Target_X;
    int32_t  Target_Y;
    int32_t  Target_Z;       // 【Z 轴预留】当前不使用，保持为 0
    
   uint32_t Delta_X;
    uint32_t Delta_Y;
    uint32_t Delta_Z;        // 【Z 轴预留】
    
   int8_t   Dir_X;          // 1: 正, -1: 反, 0: 不动
    int8_t   Dir_Y;
    int8_t   Dir_Z;          // 【Z 轴预留】
    
    uint32_t Total_Steps; // 主位移 (X和Y里脉冲数最大的那个)
       // --- V6.0 新增：梯形加减速核心参数 ---
    uint32_t Target_Speed;  // 巡航时的目标最高速度 (Hz)
    uint32_t Accel_Steps;   // 加速段需要经历的步数
    uint32_t Decel_Steps;   // 减速段需要经历的步数
} MotionBlock_t;

// 2. 集装箱：环形缓冲区
typedef struct {
    MotionBlock_t Queue[MOTION_BUFFER_SIZE];
    volatile uint8_t Head; // 写入指针
    volatile uint8_t Tail; // 读取指针
} MotionRingBuffer_t;

// 全局变量声明
extern MotionRingBuffer_t CNC_Motion_Buffer;
extern int32_t Planner_Current_X;
extern int32_t Planner_Current_Y;
extern volatile int32_t        Physical_Current_X;
extern volatile int32_t        Physical_Current_Y;
// 声明全局系统状态变量 (让其他文件也能看到)
extern volatile SystemState_t CNC_State; 
// 函数声明
void RingBuffer_Init(MotionRingBuffer_t *buf);
int8_t RingBuffer_Push(MotionRingBuffer_t *buf, MotionBlock_t *block);
int8_t RingBuffer_Pop(MotionRingBuffer_t *buf, MotionBlock_t *block);
void Planner_Add_Block(int32_t target_x, int32_t target_y);
void   CNC_TIM_Interrupt_Handler(void);   // 放在 TIM3 中断回调里调用
void CNC_Debug_Print(void);
void   Fast_itoa(int32_t num, char *str);
void   Delay_us(uint32_t us);

#endif