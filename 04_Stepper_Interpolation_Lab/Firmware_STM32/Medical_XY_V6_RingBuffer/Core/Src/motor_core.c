#include "motor_core.h"
#include "homing_limit.h"     // 调用 Homing_Step_Handler
#include <math.h>
#include <stdio.h>
#include <string.h>
// ==========================================
// 全局变量实体定义
// ==========================================
MotionRingBuffer_t CNC_Motion_Buffer;
MotionRingBuffer_t       CNC_Motion_Buffer;
int32_t                  Planner_Current_X = 0;     // 虚拟账本 X
int32_t                  Planner_Current_Y = 0;     // 虚拟账本 Y

volatile int32_t         Physical_Current_X = 0;    // 绝对物理账本 X
volatile int32_t         Physical_Current_Y = 0;    // 绝对物理账本 Y

volatile SystemState_t   CNC_State = SYS_UNHOMED;   // 上电默认未归零！

// ----- 速度调度器内部状态 -----
static uint32_t Current_Speed_Hz       = 500;       // 起步最低速度
static uint32_t Last_Speed_Update_Time = 0;         // 上次更新速度的时间戳 (ms)
static uint32_t Accel_Rate             = 10;        // 每毫秒速度增加 10 Hz
// ----- TIM3 中断里的私有吃货变量 -----
static MotionBlock_t  Current_Block;
static uint32_t       Steps_Executed   = 0;
static int32_t        Bresenham_Error  = 0;

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef  htim3;

// ==========================================
// 【架构师底层引擎】极速整数转字符串 (替代 sprintf)
// 内存开销：几乎为0 | 执行速度：微秒级
// ==========================================
void Fast_itoa(int32_t num, char* str) {
    int i = 0;
    uint8_t isNegative = 0;

    // 1. 拦截特殊情况：数字就是 0
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0'; // 字符串结束标志
        return;
    }

    // 2. 负数处理：打个标记，然后变成正数去剥洋葱
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // 3. 剥洋葱核心逻辑 (从末尾提取数字转ASCII)
    while (num != 0) {
        int rem = num % 10;          // 取出最后一位
        str[i++] = rem + '0';        // 加上 '0' (0x30) 变成字符，存进数组
        num = num / 10;              // 砍掉最后一位
    }

    // 4. 如果是负数，在最后补上负号
    if (isNegative) {
        str[i++] = '-';
    }

    str[i] = '\0'; // 封口：加上字符串结束标志

    // 5. 逆转乾坤：把存反的字符串首尾对调
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}
// 微秒级延迟函数 (发脉冲用)
void Delay_us(uint32_t us) {
    uint32_t delay = (SystemCoreClock / 1000000) * us / 4;
    while(delay--) { __NOP(); }
}

// ----------------------------------------------------
// 【仓库管理员模块】
// ----------------------------------------------------
void RingBuffer_Init(MotionRingBuffer_t *buf)
{
    buf->Head = 0;
    buf->Tail = 0;
}

int8_t RingBuffer_Push(MotionRingBuffer_t *buf, MotionBlock_t *block)
{
    uint8_t next_head = (buf->Head + 1) % MOTION_BUFFER_SIZE;
    if (next_head == buf->Tail) return -1;     // 满
    buf->Queue[buf->Head] = *block;
    buf->Head = next_head;
    return 0;
}
int8_t RingBuffer_Pop(MotionRingBuffer_t *buf, MotionBlock_t *block)
{
    if (buf->Head == buf->Tail) return -1;     // 空
    *block = buf->Queue[buf->Tail];
    buf->Tail = (buf->Tail + 1) % MOTION_BUFFER_SIZE;
    return 0;
}

// ==========================================
// Planner 中央厨房：把目标坐标拆成 MotionBlock 入队
// ==========================================
void Planner_Add_Block(int32_t target_x, int32_t target_y) {
    
    
     MotionBlock_t block = {0};   // 全字段清零，Z 轴字段自动为 0
     
     // 【上帝安检门】：如果没有归零，或者系统正在报警，绝对拒接任何订单！
    if (CNC_State != SYS_READY && CNC_State != SYS_RUNNING) {
        return; // 直接把订单撕了，扔进垃圾桶！
    }
    
    // 1. 算 X 轴差值与方向
    int32_t dx = target_x - Planner_Current_X;
    if      (dx > 0) { block.Dir_X =  1; block.Delta_X =  dx; }
    else if (dx < 0) { block.Dir_X = -1; block.Delta_X = -dx; }
    else             { block.Dir_X =  0; block.Delta_X =   0; }

    // 2. 算 Y 轴差值与方向
    int32_t dy = target_y - Planner_Current_Y;
    if      (dy > 0) { block.Dir_Y =  1; block.Delta_Y =  dy; }
    else if (dy < 0) { block.Dir_Y = -1; block.Delta_Y = -dy; }
    else             { block.Dir_Y =  0; block.Delta_Y =   0; }
    
    // 3. Z 轴预留位（现在永远不动）
    block.Target_Z = 0;
    block.Delta_Z  = 0;
    block.Dir_Z    = 0;
    
      // 4. 主位移 = 三轴里步数最多的那个（现在 Z=0，等于 max(X,Y)）
    block.Total_Steps = block.Delta_X;
    if (block.Delta_Y > block.Total_Steps) block.Total_Steps = block.Delta_Y;
    if (block.Delta_Z > block.Total_Steps) block.Total_Steps = block.Delta_Z;

    if (block.Total_Steps == 0) return;   // 没动，丢弃

    // ----------------------------------------------------
    // 5. 【V6.0 核心装填】梯形加减速与三角形降维算法！
    // ----------------------------------------------------
// 5. 梯形加减速参数（短距离自动降维成三角形）
    block.Target_Speed = 10000;           // 目标 10 kHz
    if (block.Total_Steps >= (2000 + 2000)) {
        block.Accel_Steps = 2000;
        block.Decel_Steps = 2000;
    } else {
        block.Accel_Steps = block.Total_Steps / 2;
        block.Decel_Steps = block.Total_Steps - block.Accel_Steps;
    }

    // 6. 入队（非阻塞：满了直接丢弃 + 串口告警）
    //    旧代码用 while(...) 死等，会卡死主循环 —— 改成丢弃更工业级
    if (RingBuffer_Push(&CNC_Motion_Buffer, &block) == -1) {
        const char *warn = "[WARN] Buffer full, command dropped\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t *)warn, strlen(warn), 10);
        return;
    }

    // 7. 入队成功才更新虚拟账本
    Planner_Current_X = target_x;
    Planner_Current_Y = target_y;
}
// ----------------------------------------------------
// 【无情吞噬者模块 - 必须放入 TIM3 的回调函数中】
// ----------------------------------------------------
void CNC_TIM_Interrupt_Handler(void) {
      // ----- 状态机分流 -----
    if (CNC_State == SYS_HOMING) {
        // 归零特权模式：交给归零状态机独占控制
        Homing_Step_Handler();         // 【已修复】之前这一行被注释掉了
        return;
    }

    if (CNC_State != SYS_RUNNING && CNC_State != SYS_READY) {
        return;   // ERROR / UNHOMED 直接退出，谁都不让动
    }
   
    // ====================================================
    // 1. 当前 Block 是不是吃完了？吃完就拿下一个
    // ====================================================
    if (Steps_Executed >= Current_Block.Total_Steps) {
        if (RingBuffer_Pop(&CNC_Motion_Buffer, &Current_Block) == 0) {
            Steps_Executed  = 0;

            // 拨动物理 DIR 引脚（X / Y）
            if (Current_Block.Dir_X ==  1) HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
            else if (Current_Block.Dir_X == -1) HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);

            if (Current_Block.Dir_Y ==  1) HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_RESET);
            else if (Current_Block.Dir_Y == -1) HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);

            // 【Z 轴预留】未来加 Z 时取消注释
            // if (Current_Block.Dir_Z ==  1) HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_RESET);
            // else if (Current_Block.Dir_Z == -1) HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_SET);

            // Bresenham 误差桶初始化（漏水法标准起点）
            Bresenham_Error = (int32_t)Current_Block.Total_Steps / 2;

            // 速度打回低速起步档
            Current_Speed_Hz = 500;
            CNC_State = SYS_RUNNING;
        } else {
            // 仓库空了，回到 READY 待命
            CNC_State = SYS_READY;
            return;
        }
    }


    // ====================================================
    // 2. Chief Engineer 的绝对防御：时间戳加减速状态机！
    // ====================================================
   uint32_t current_time = HAL_GetTick();
    if (current_time - Last_Speed_Update_Time >= 1) {
        Last_Speed_Update_Time = current_time;

        if (Steps_Executed < Current_Block.Accel_Steps) {
            // 加速段
            Current_Speed_Hz += Accel_Rate;
            if (Current_Speed_Hz > Current_Block.Target_Speed)
                Current_Speed_Hz = Current_Block.Target_Speed;
        } else if (Steps_Executed < (Current_Block.Total_Steps - Current_Block.Decel_Steps)) {
            // 巡航段
            Current_Speed_Hz = Current_Block.Target_Speed;
        } else {
            // 减速段
            if (Current_Speed_Hz > 500 + Accel_Rate)
                Current_Speed_Hz -= Accel_Rate;
            else
                Current_Speed_Hz = 500;
        }

        // ARR = 1MHz / 速度Hz（TIM3 配置成 1MHz 输入时钟）
        TIM3->ARR = 1000000U / Current_Speed_Hz;
    }
    // ====================================================
    // 3. Bresenham 漏水算法 + 发脉冲（XY 二维插补）
    //    Z 轴预留：未来扩展为 3D Bresenham 时再改
    // ====================================================
    uint8_t step_x = 0;
    uint8_t step_y = 0;
    
   if (Current_Block.Delta_X >= Current_Block.Delta_Y) {
        // X 是主轴
        if (Current_Block.Dir_X != 0) step_x = 1;
        Bresenham_Error -= (int32_t)Current_Block.Delta_Y;
        if (Bresenham_Error < 0) {
            if (Current_Block.Dir_Y != 0) step_y = 1;
            Bresenham_Error += (int32_t)Current_Block.Delta_X;
        }
    } else {
        // Y 是主轴
        if (Current_Block.Dir_Y != 0) step_y = 1;
        Bresenham_Error -= (int32_t)Current_Block.Delta_X;
        if (Bresenham_Error < 0) {
            if (Current_Block.Dir_X != 0) step_x = 1;
            Bresenham_Error += (int32_t)Current_Block.Delta_Y;
        }
    }
    
        // =========================================================
     // ----- 拉高脉冲 + 同步更新物理账本 -----
    // =========================================================
      if (step_x) {
        HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
        if (Current_Block.Dir_X ==  1) Physical_Current_X++;
        else                           Physical_Current_X--;
    }
    if (step_y) {
        HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_SET);
        if (Current_Block.Dir_Y ==  1) Physical_Current_Y++;
        else                           Physical_Current_Y--;
    }

    Delay_us(2);   // 脉冲高电平宽度 2 μs（驱动器一般要求 ≥1.5 μs）

    if (step_x) HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
    if (step_y) HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_RESET);

    Steps_Executed++;
}

// ==========================================
// CNC 仪表盘：双货车并行（Python 雷达 + 人类可读）
// ==========================================

// 工业级防溢出仪表盘 (100% 隔离 sprintf)
void CNC_Debug_Print(void) {
     // 关中断捞快照（防止打印过程中数据被改变）
    __disable_irq();
    uint32_t current_total = Current_Block.Total_Steps;
    uint32_t executed      = Steps_Executed;
    int32_t  err           = Bresenham_Error;
    int32_t  phys_x        = Physical_Current_X;
    int32_t  phys_y        = Physical_Current_Y;
    SystemState_t state    = CNC_State;
    __enable_irq();

    char tx_buffer[150] = {0}; // 最终发送的物理货车
    char pos_buffer[64] = {0}; // 【新增】雷达专用的货车
    char temp_num[15] = {0};   // 临时数字转换小盆子 (最多10位数字+符号)
   // ===============================================
    // 第一辆货车：专供 Python 2D 雷达画图用的 POS 数据
    // 格式：POS,15000,30000\n
    // ===============================================
    strcpy(pos_buffer, "POS,");
    Fast_itoa(phys_x, temp_num);
    strcat(pos_buffer, temp_num);
    strcat(pos_buffer, ",");
    Fast_itoa(phys_y, temp_num);
    strcat(pos_buffer, temp_num);
    strcat(pos_buffer, "\r\n");
    
    // 把雷达数据发射出去！(就算空闲也发，让雷达知道当前在哪)
    HAL_UART_Transmit(&huart1, (uint8_t *)pos_buffer, strlen(pos_buffer), 10);
    
      // ===============================================
    // 第二辆货车：原来的人类观看仪表盘 (保持你现在的完美逻辑)
    // ===============================================
     if (state == SYS_HOMING) {
        strcpy(tx_buffer, "[CNC Homing] Seeking limit switches...\r\n");
    } else if (state == SYS_ERROR) {
        strcpy(tx_buffer, "[CNC E-STOP] SYSTEM LOCKED! Reset MCU Required!\r\n");
    } else if (state == SYS_UNHOMED) {
        strcpy(tx_buffer, "[CNC Unhomed] Send 'HOME' to start homing.\r\n");
    } else if (current_total > 0 && executed < current_total) {
        strcpy(tx_buffer, "[CNC Run] Tgt:");
        Fast_itoa(current_total, temp_num); strcat(tx_buffer, temp_num);
        strcat(tx_buffer, " Exe:");
        Fast_itoa(executed, temp_num); strcat(tx_buffer, temp_num);
        strcat(tx_buffer, " Err:");
        Fast_itoa(err, temp_num); strcat(tx_buffer, temp_num);
        strcat(tx_buffer, "\r\n");
    } else {
        strcpy(tx_buffer, "[CNC Ready] Buffer empty, motor idle.\r\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
}
