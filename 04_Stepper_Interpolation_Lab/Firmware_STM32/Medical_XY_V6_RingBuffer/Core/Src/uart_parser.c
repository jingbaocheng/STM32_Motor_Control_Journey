#include "uart_parser.h"
#include "motor_core.h"
#include "homing_limit.h"
#include <string.h>
#include <stdlib.h> // 为了使用 atoi() 把字符串转成整数

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef  htim3;
extern uint8_t RX_Temp_Char;   // 在 main.c 里定义

// 串口接收缓冲区 (最多存 64 个字符)

char    RX_Buffer[64];
uint8_t RX_Index       = 0;
volatile uint8_t Command_Ready = 0;

//==========================================
// 动作 1：主循环里的“翻译官”
// (这个函数要放在 main.c 的 while(1) 里面疯狂调用)
// ==========================================
void UART_Parse_Command(void) {
     if (Command_Ready != 1) return;

    // 场景 A：归零指令 "HOME"
    if (strncmp(RX_Buffer, "HOME", 4) == 0) {
        Homing_Init();
    }
         // 场景 B：急停指令 "E"（最高优先级，秒级响应）
    else if (RX_Buffer[0] == 'E') {
        // 1. 全局挂挡杆瞬间掰到 ERROR
        CNC_State = SYS_ERROR;

        // 2. 强行清空缓冲区（丢弃所有未执行订单）
        CNC_Motion_Buffer.Head = 0;
        CNC_Motion_Buffer.Tail = 0;

        // 3. 关闭 TIM3 中断（剥夺电机最后一次发脉冲的权利）
        HAL_TIM_Base_Stop_IT(&htim3);
    }
         // 场景 C：运动指令，例如 "X15000 Y20000"
    else if (RX_Buffer[0] == 'X') {
        int32_t target_x = 0;
        int32_t target_y = 0;
        char *ptr_x = strchr(RX_Buffer, 'X');
        char *ptr_y = strchr(RX_Buffer, 'Y');
        if (ptr_x != NULL) target_x = atoi(ptr_x + 1);
        if (ptr_y != NULL) target_y = atoi(ptr_y + 1);
        Planner_Add_Block(target_x, target_y);
    }
    // 【可选扩展】场景 D：复位 ERROR 状态（"R" 或 "RESET"）
    else if (RX_Buffer[0] == 'R') {
        // 软件层面把状态拨回 UNHOMED，让用户重新归零
        CNC_State = SYS_UNHOMED;
        HAL_TIM_Base_Start_IT(&htim3);
    }

    // 擦黑板，准备接收下一条
    memset(RX_Buffer, 0, sizeof(RX_Buffer));
    RX_Index      = 0;
    Command_Ready = 0;
}


// ==========================================
// 附赠动作 2：串口接收中断 (每收到一个字母就触发一次)
// (这个不用你写，直接复制去用，它是翻译官的苦力)
// ==========================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (RX_Temp_Char == '\n' || RX_Temp_Char == '\r') {
            if (RX_Index > 0) {              // 防止空回车把 Command_Ready 拉错
                Command_Ready = 1;
            }
        } else {
            if (RX_Index < (sizeof(RX_Buffer) - 1)) {
                RX_Buffer[RX_Index++] = RX_Temp_Char;
            }
        }
        // 重新启动接收（让中断永远活着）
        HAL_UART_Receive_IT(huart, &RX_Temp_Char, 1);
    }
}

