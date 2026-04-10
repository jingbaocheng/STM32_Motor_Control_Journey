#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "main.h"

void Protocol_Init(void);
void UART_SendString(char *str); // 【新增】我们自己的纯手工发送函数

#endif
