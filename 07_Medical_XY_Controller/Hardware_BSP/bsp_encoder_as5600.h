#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H

#include "main.h" // 这样才能使用 HAL 库的定义

// 1. AS5600 的设备地址 (7位地址是 0x36，左移一位变成 0x6C 供 HAL 使用)
#define AS5600_ADDR        0x6C 

// 2. 寄存器地址：我们要读的是 0x0C(高8位) 和 0x0D(低8位)
#define AS5600_RAW_ANG_REG 0x0C 

// 3. 函数声明
float AS5600_Get_Angle(void);  // 获取弧度值 (0 ~ 6.28)
uint16_t AS5600_Read_Raw(void); // 获取原始值 (0 ~ 4095)

#endif