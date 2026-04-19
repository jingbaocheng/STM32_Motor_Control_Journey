#include "protocol.h"
#include "usart.h"
#include "motor_core.h"
#include <stdio.h>

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
    int target_x = 0;
    int target_y = 0;
    int i = 0;

     // 【新增】：最高优先级拦截急停指令！
    if (Rx_Buffer[0] == 'E') {
        Motor_Emergency_Stop();
        UART_SendString("[WARN] EMERGENCY STOP EXECUTED!!!\r\n");
        return; // 直接退出，绝不执行后面的解析
    }
    
       
    // ==========================================

    // 提取 X 坐标
    while(Rx_Buffer[i] != '\0' && Rx_Buffer[i] != 'X') i++;
    if(Rx_Buffer[i] == 'X') {
        i += 2; // 跳过 'X' 和 ':'
        while(Rx_Buffer[i] >= '0' && Rx_Buffer[i] <= '9') {
            target_x = target_x * 10 + (Rx_Buffer[i] - '0');
            i++;
        }
    }

    // 提取 Y 坐标
    i = 0;
    while(Rx_Buffer[i] != '\0' && Rx_Buffer[i] != 'Y') i++;
    if(Rx_Buffer[i] == 'Y') {
        i += 2; // 跳过 'Y' 和 ':'
        while(Rx_Buffer[i] >= '0' && Rx_Buffer[i] <= '9') {
            target_y = target_y * 10 + (Rx_Buffer[i] - '0');
            i++;
        }
    }

    // 【终极吐真剂 1：看看你到底提取出了什么数字？】
    char dbg1[64];
    sprintf(dbg1, "[CMD OK] Parse X:%d Y:%d\r\n", target_x, target_y);
    UART_SendString(dbg1);
    
    Motor_Load_Command(target_x, target_y); 
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
