#include "motor_core.h"
#include "tim.h"   // 需要用到 htim3
#include "usart.h" // 需要用到 printf
#include <stdlib.h> // 需要用到 abs()
#include <stdio.h>
#include "protocol.h"
DualAxisSystem_t XY_Sys = {0}; // 实例化全局变量

// ==========================================
// 函数1：装填弹药与扣动扳机 (串口解析完后调用这个！)
// ==========================================
void Motor_Load_Command(int32_t target_x, int32_t target_y) {
    if(XY_Sys.State != MOTOR_IDLE) return; // 没跑完不接新活

    XY_Sys.Target_X = target_x;
    XY_Sys.Target_Y = target_y;

    int32_t diff_x = XY_Sys.Target_X - XY_Sys.Current_X;
    int32_t diff_y = XY_Sys.Target_Y - XY_Sys.Current_Y;

    // 判方向
    if (diff_x > 0) HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
    else            HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
    
    if (diff_y > 0) HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
    else            HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_RESET);

    XY_Sys.dx_total = abs(diff_x);
    XY_Sys.dy_total = abs(diff_y);

    // 找老大，初始化水桶
    if (XY_Sys.dx_total >= XY_Sys.dy_total) {
        XY_Sys.is_X_Boss = 1;
        XY_Sys.Main_Steps_Total = XY_Sys.dx_total;
    } else {
        XY_Sys.is_X_Boss = 0;
        XY_Sys.Main_Steps_Total = XY_Sys.dy_total;
    }
    XY_Sys.Error_Bucket = 0;

    // 起步配置
    XY_Sys.Main_Steps_Current = 0;
    XY_Sys.Current_ARR = 1500;   // 起步慢一点，1500us
    XY_Sys.State = MOTOR_ACCEL;

    // 强行塞进 TIM3 并开启中断心跳！
    __HAL_TIM_SET_AUTORELOAD(&htim3, XY_Sys.Current_ARR); 
    HAL_TIM_Base_Start_IT(&htim3);
}

// ==========================================
// 函数2：放在 TIM3 中断回调里执行的心脏跳动
// ==========================================
void Motor_TIM_Interrupt_Handler(void) {
    if (XY_Sys.State == MOTOR_IDLE) return;

    // 1. 瞬间空间插补 (Bresenham)
    if (XY_Sys.is_X_Boss == 1) {
        HAL_GPIO_TogglePin(X_STEP_GPIO_Port, X_STEP_Pin);
        XY_Sys.Error_Bucket += 2 * XY_Sys.dy_total;
        if (XY_Sys.Error_Bucket >= XY_Sys.dx_total) {
            HAL_GPIO_TogglePin(Y_STEP_GPIO_Port, Y_STEP_Pin);
            XY_Sys.Error_Bucket -= 2 * XY_Sys.dx_total;
        }
    } else {
        HAL_GPIO_TogglePin(Y_STEP_GPIO_Port, Y_STEP_Pin);
        XY_Sys.Error_Bucket += 2 * XY_Sys.dx_total;
        if (XY_Sys.Error_Bucket >= XY_Sys.dy_total) {
            HAL_GPIO_TogglePin(X_STEP_GPIO_Port, X_STEP_Pin);
            XY_Sys.Error_Bucket -= 2 * XY_Sys.dy_total;
        }
    }

    // 2. 状态机流转 (梯形加减速)
    static uint8_t toggle_flag = 0;
    toggle_flag++;
    if (toggle_flag >= 2) {
        toggle_flag = 0;
        XY_Sys.Main_Steps_Current++; 

        // 【状态机就在这里！！！】根据已走步数，调整 ARR 油门
        switch(XY_Sys.State) {
            case MOTOR_ACCEL:
                if (XY_Sys.Current_ARR > 400) {  // 最高速限制 (400us)
                    XY_Sys.Current_ARR -= 2;     // 每次加速减小 ARR
                } else {
                    XY_Sys.State = MOTOR_RUN;
                }
                // 判断是否需要提前进入减速 (总步数太短的情况)
                if (XY_Sys.Main_Steps_Total - XY_Sys.Main_Steps_Current <= XY_Sys.Main_Steps_Current) {
                    XY_Sys.State = MOTOR_DECEL;
                }
                break;
                
            case MOTOR_RUN:
                // 距离终点还有多少步开始减速？(这里简单设为开始加速时的步数)
                if (XY_Sys.Main_Steps_Total - XY_Sys.Main_Steps_Current <= 500) { 
                    XY_Sys.State = MOTOR_DECEL;
                }
                break;

            case MOTOR_DECEL:
                if (XY_Sys.Current_ARR < 1500) {
                    XY_Sys.Current_ARR += 2; // 踩刹车
                }
                break;
                
            default:
                break;
        }

        // 终点判断
        if (XY_Sys.Main_Steps_Current >= XY_Sys.Main_Steps_Total) {
            XY_Sys.State = MOTOR_STOP;
            XY_Sys.Current_X = XY_Sys.Target_X; 
            XY_Sys.Current_Y = XY_Sys.Target_Y;
            HAL_TIM_Base_Stop_IT(&htim3); // 停跳
             // 【替换这行】
            UART_SendString("[CMD OK] Target Reached!\r\n"); 
            XY_Sys.State = MOTOR_IDLE;
        } else {
            // 更新油门
            __HAL_TIM_SET_AUTORELOAD(&htim3, XY_Sys.Current_ARR);
        }
    }
}
