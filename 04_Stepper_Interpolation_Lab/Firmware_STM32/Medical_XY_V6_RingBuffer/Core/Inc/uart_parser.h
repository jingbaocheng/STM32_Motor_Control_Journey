#ifndef UART_PARSER_H
#define UART_PARSER_H

#include "main.h"

void UART_Parse_Command(void);

// 全局变量声明（实体在 uart_parser.c 里）
extern char    RX_Buffer[64];
extern uint8_t RX_Index;
extern volatile uint8_t Command_Ready;

#endif
