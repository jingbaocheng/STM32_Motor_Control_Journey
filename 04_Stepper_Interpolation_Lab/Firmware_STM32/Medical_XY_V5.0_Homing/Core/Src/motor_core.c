#include "motor_core.h"
#include "tim.h"   // 需要用到 htim3
#include "usart.h" // 需要用到 printf
#include <stdlib.h> // 需要用到 abs()
#include <stdio.h>
#include "protocol.h"
volatile DualAxisSystem_t XY_Sys = {0};
 // 实例化全局变量

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
    if (diff_x > 0) 
    {HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
                    XY_Sys.dir_x = 1; }
    else            {
        HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
         XY_Sys.dir_x = -1; 
    }
    if (diff_y > 0) 
    {HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_RESET);
         XY_Sys.dir_y = 1;
    }
    else           { 
        HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
         XY_Sys.dir_y = -1;
    }

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
      // 【终极吐真剂 2：看看算出来的绝对位移是不是 0？】
    char dbg2[64];
    sprintf(dbg2, "[MOTOR] Start! dx:%d dy:%d\r\n", XY_Sys.dx_total, XY_Sys.dy_total);
    UART_SendString(dbg2);


    // 强行塞进 TIM3 并开启中断心跳！
    __HAL_TIM_SET_AUTORELOAD(&htim3, XY_Sys.Current_ARR); 
    HAL_TIM_Base_Start_IT(&htim3);
}

// ==========================================
// 函数2：放在 TIM3 中断回调里执行的心脏跳动
// ==========================================
void Motor_TIM_Interrupt_Handler(void) {
    if (XY_Sys.State == MOTOR_IDLE) return;
    
    static uint8_t toggle_flag = 0;
    static uint8_t sub_step = 0;   
      // ======== 【新增：归零劫持点】 ========
    // 如果状态大于等于归零状态，说明在找零点，直接跳入 Homing 逻辑，不执行后面的插补！
    if (XY_Sys.State >= MOTOR_HOMING_FAST) {
        Motor_Homing_Handler();
        return; 
    }
    // ===================================

    //空间插补 (Bresenham 瞬间爆发)
   
    if (XY_Sys.is_X_Boss == 1) {
        HAL_GPIO_TogglePin(X_STEP_GPIO_Port, X_STEP_Pin);
        
        // 【新增】：当完整走完一步时更新物理坐标
        if (toggle_flag == 0) {
          XY_Sys.Error_Bucket += 2 * XY_Sys.dy_total;

            if (XY_Sys.Error_Bucket >= XY_Sys.dx_total) 
                {    sub_step = 1; // 在备忘录里打个勾：小弟这步要跟上！
                HAL_GPIO_TogglePin(Y_STEP_GPIO_Port, Y_STEP_Pin);
                    XY_Sys.Error_Bucket -= 2 * XY_Sys.dx_total;
                     
                }else{ sub_step = 0; // 不用跟
                        }
         }else {
                // 【后半拍：统一更新物理坐标，并补齐小弟的脉冲】
                XY_Sys.Current_X += XY_Sys.dir_x; // 老大坐标更新
                if (sub_step == 1) {
                    HAL_GPIO_TogglePin(Y_STEP_GPIO_Port, Y_STEP_Pin); // 小弟后半拍翻转补齐波形！
                    XY_Sys.Current_Y += XY_Sys.dir_y;                 // 小弟坐标更新！
                            }     
                    } 
    }else {
        // Y 是老大 (逻辑反过来)
        HAL_GPIO_TogglePin(Y_STEP_GPIO_Port, Y_STEP_Pin);

        if (toggle_flag == 0) {
            XY_Sys.Error_Bucket += 2 * XY_Sys.dx_total;
            if (XY_Sys.Error_Bucket >= XY_Sys.dy_total) {
                sub_step = 1;
                HAL_GPIO_TogglePin(X_STEP_GPIO_Port, X_STEP_Pin);
                XY_Sys.Error_Bucket -= 2 * XY_Sys.dy_total;
            } else {
                sub_step = 0;
            }
        } else {
            XY_Sys.Current_Y += XY_Sys.dir_y;
            if (sub_step == 1) {
                HAL_GPIO_TogglePin(X_STEP_GPIO_Port, X_STEP_Pin);
                XY_Sys.Current_X += XY_Sys.dir_x;
            }
        }
    }
    // 2. 状态机流转 (梯形加减速)
 
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
// ==========================================
// 绝对防线：紧急停止
// ==========================================
void Motor_Emergency_Stop(void) {
    HAL_TIM_Base_Stop_IT(&htim3); // 瞬间掐断脉冲起搏器！
    XY_Sys.State = MOTOR_IDLE;    // 强行把状态机打回空闲状态
    XY_Sys.Error_Bucket = 0;      // 清空水桶
}
// ==========================================
// 【核心】三段式非阻塞归零状态机 (在 TIM3 中断内运行)
// ==========================================
void Motor_Homing_Handler(void) {
    static uint8_t toggle_h = 0; // 专属的脉冲翻转标志
    
    // 1. 动态映射当前归零的引脚 (是指针的艺术)
    GPIO_TypeDef* step_port = (XY_Sys.Homing_Axis == HOME_X_AXIS) ? X_STEP_GPIO_Port : Y_STEP_GPIO_Port;
    uint16_t      step_pin  = (XY_Sys.Homing_Axis == HOME_X_AXIS) ? X_STEP_Pin       : Y_STEP_Pin;
    GPIO_TypeDef* dir_port  = (XY_Sys.Homing_Axis == HOME_X_AXIS) ? X_DIR_GPIO_Port  : Y_DIR_GPIO_Port;
    uint16_t      dir_pin   = (XY_Sys.Homing_Axis == HOME_X_AXIS) ? X_DIR_Pin        : Y_DIR_Pin;
    
    // 2. 动态读取当前轴的传感器状态 (刚刚物理测试打通的防线！)
    uint8_t sensor_val = 0;
    if (XY_Sys.Homing_Axis == HOME_X_AXIS) sensor_val = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6); // X限位
    else                                   sensor_val = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7); // Y限位

    // 3. 核心状态机跃迁逻辑
    switch (XY_Sys.State) {
        
        case MOTOR_HOMING_FAST:
            if (sensor_val == 1) { // 撞到了！(根据你的测试，遮挡=1)
                XY_Sys.State = MOTOR_HOMING_BACK; // 切换到慢退模式
                HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET); // 假设 SET 是反向退走，请根据实际调整
                __HAL_TIM_SET_AUTORELOAD(&htim3, 2000); // 降低频率 (慢退)
                return; // 暂不发脉冲，等下一拍
            }
            break;

        case MOTOR_HOMING_BACK:
            if (sensor_val == 0) { // 刚刚退出来，光耦灯灭了！
                XY_Sys.State = MOTOR_HOMING_CREEP; // 切换到爬行模式
                HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET); // 再次正向逼近
                __HAL_TIM_SET_AUTORELOAD(&htim3, 4000); // 极度降频 (龟速爬行，确保精度)
                return;
            }
            break;

         case MOTOR_HOMING_CREEP:
            if (sensor_val == 1) { // 纳秒级瞬间撞击锁定！
                HAL_TIM_Base_Stop_IT(&htim3); // 瞬间掐断脉冲
                
                // 1. 抹平世界，定义绝对零点
                if (XY_Sys.Homing_Axis == HOME_X_AXIS) {
                    XY_Sys.Current_X = 0;
                    UART_SendString("[SYS] X_Axis ZERO Locked.\r\n");
                } else {
                    XY_Sys.Current_Y = 0;
                    UART_SendString("[SYS] Y_Axis ZERO Locked.\r\n");
                }

                // 2. 核心联动逻辑：接力棒交接！
                if (XY_Sys.Homing_Seq_Flag == 1 && XY_Sys.Homing_Axis == HOME_X_AXIS) {
                    // X轴刚跑完第一棒，立刻准备启动Y轴第二棒
                    XY_Sys.State = MOTOR_IDLE; 
                    UART_SendString("[SYS] Seq Homing: Starting Y_Axis...\r\n");
                    Motor_Start_Homing(HOME_Y_AXIS); // 递归自己，启动Y轴
                    return;
                }

                // 3. 全部跑完，彻底收工
                XY_Sys.State = MOTOR_IDLE;    
                XY_Sys.Homing_Axis = HOME_NONE;
                XY_Sys.Homing_Seq_Flag = 0;
                UART_SendString("[SYS] ALL Homing Complete! System Ready.\r\n");
                return;
            }
            break;
            
        default:
            return; // 异常保护
    }

    // 4. 如果没有触发状态切换，就老老实实发归零脉冲
    if (toggle_h == 0) {
        HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
        toggle_h = 1;
    } else {
        HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
        toggle_h = 0;
    }
}
// 启动指定轴的归零流程
void Motor_Start_Homing(HomingTarget_t axis) {
//    if (XY_Sys.State != MOTOR_IDLE) return; // 正在运动则拒接接客
    
      // 智能任务分配
    if (axis == HOME_ALL) {
        XY_Sys.Homing_Seq_Flag = 1;       // 竖起双轴联动大旗
        XY_Sys.Homing_Axis = HOME_X_AXIS; // 强制第一棒先跑 X 轴
    } else {
        XY_Sys.Homing_Seq_Flag = 0;       // 单轴模式
        XY_Sys.Homing_Axis = axis;
    }

    XY_Sys.State = MOTOR_HOMING_FAST; // 切入快撞模式

    // 动态分配方向 (根据昨晚测试，SET 是朝着限位开关走)
    if (XY_Sys.Homing_Axis == HOME_X_AXIS) {
        HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
    } else if (XY_Sys.Homing_Axis == HOME_Y_AXIS) {
        HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
    }

    __HAL_TIM_SET_AUTORELOAD(&htim3, 2000); // 设定安全的起步油门
    UART_SendString("[SYS] Homing Started...\r\n");
    HAL_TIM_Base_Start_IT(&htim3); // 点火！
}