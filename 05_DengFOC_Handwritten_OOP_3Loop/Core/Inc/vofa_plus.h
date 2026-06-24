#ifndef __VOFA_PLUS_H
#define __VOFA_PLUS_H

#include "foc_app.h" // 引入指针依赖

void VOFA_SendData(float *data, int num);
void VOFA_SendMotorData(foc_control_t *motor); // ?? 补齐该声明

#endif
