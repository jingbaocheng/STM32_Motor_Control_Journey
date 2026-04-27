#include "protocol.h"
#include "usart.h"
#include "motor_core.h"
#include <stdio.h>
#include "tim.h" 
#include <string.h>
#include <stdlib.h>
uint8_t Rx_Data;
uint8_t Rx_Buffer[50];
uint8_t Rx_Index = 0;

// ==========================================
// 1. 【新增】绝对不死机的纯手工发送函数
// ==========================================
void UART_SendString(char *str) {
    while (*str) {
        // 死等硬件发送通道空闲，然后直接把字符塞进底层数据寄存器 (DR)
        while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
        huart1.Instance->DR = (uint8_t)(*str);
        str++;
    }
}

// ==========================================
// 2. 纯手工解析器：提取 "X:1000 Y:500" 
// ==========================================
void Protocol_Parse_Command(void) {
     // ========================================================
    // 【核心修复 1：坐标默认值必须是当前绝对坐标，绝不能是 0！】
    // ========================================================
    int32_t target_x = XY_Sys.Current_X; 
    int32_t target_y = XY_Sys.Current_Y;
    uint8_t valid_cmd = 0; // 标记是否提取到了有效数字


     // 【新增】：最高优先级拦截急停指令！
     if (Rx_Buffer[0] == 'E') {
        Motor_Emergency_Stop();
        UART_SendString("[WARN] EMERGENCY STOP EXECUTED!!!\r\n");
        return; 
    }

    // 2. 主动重置防死锁：强制停止并解开一切锁定
    if (strncmp((char*)Rx_Buffer, "RESET", 5) == 0) {
        HAL_TIM_Base_Stop_IT(&htim3);
        XY_Sys.State = MOTOR_IDLE;
        UART_SendString("[SYS] System State Reset to IDLE.\r\n");
        return;
    }

    // 3. 高鲁棒性归零解析指令 (不论带不带换行符)
    if (strncmp((char*)Rx_Buffer, "XH", 2) == 0) {
        UART_SendString("[CMD] X_Axis Homing Initiated...\r\n");
        Motor_Start_Homing(HOME_X_AXIS); 
        return; 
    }
    if (strncmp((char*)Rx_Buffer, "YH", 2) == 0) {
        UART_SendString("[CMD] Y_Axis Homing Initiated...\r\n");
        Motor_Start_Homing(HOME_Y_AXIS); 
        return; 
    }
    // 注意：单字符 'H' 放在后面判断，防止与 'XH' 冲突
    if (Rx_Buffer[0] == 'H') {
        UART_SendString("[CMD] Auto Sequence Homing Initiated...\r\n");
        Motor_Start_Homing(HOME_ALL); 
        return; 
    }
    
   
    // ========================================================
    // 【核心修复 2：引入轻量级 strstr + atoi，完美支持负号 '-'】
    // ========================================================
    // 提取 X 坐标
    char* ptr_x = strstr((char*)Rx_Buffer, "X:");
    if (ptr_x != NULL) {
        target_x = atoi(ptr_x + 2); // 跨过 "X:"，atoi会自动处理负号
        valid_cmd = 1;
    }

    // 提取 Y 坐标
    char* ptr_y = strstr((char*)Rx_Buffer, "Y:");
    if (ptr_y != NULL) {
        target_y = atoi(ptr_y + 2);
        valid_cmd = 1;
    }
     // ========================================================
    // 【核心修复 3：终极拦截与放行】
    // ========================================================
    if (valid_cmd == 1) {
        char dbgl[64];
        // 打印吐真剂，看看到底提取了什么
        sprintf(dbgl, "[CMD OK] Parse X:%d Y:%d\r\n", target_x, target_y);
        UART_SendString(dbgl);
        
        Motor_Load_Command(target_x, target_y);
    } else {
        // 如果发了乱码，也不至于崩溃
        // UART_SendString("[ERR] Invalid Command Format.\r\n");
    }
}

// ==========================================
// 3. 串口接收中断回调函数
// ==========================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        if(Rx_Data == '\n' || Rx_Data == '\r') {
            Rx_Buffer[Rx_Index] = '\0';
            if(Rx_Index > 0) {
                Protocol_Parse_Command();
            }
            Rx_Index = 0;
        } else {
            Rx_Buffer[Rx_Index++] = Rx_Data;
            if(Rx_Index >= 50) Rx_Index = 0; 
        }
        HAL_UART_Receive_IT(&huart1, &Rx_Data, 1);
    }
}

// ==========================================
// 4. 通信层初始化
// ==========================================
void Protocol_Init(void) {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_Receive_IT(&huart1, &Rx_Data, 1);
}
