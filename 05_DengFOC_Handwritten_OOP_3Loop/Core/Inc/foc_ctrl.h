#ifndef __FOC_CTRL_H
#define __FOC_CTRL_H

#include "main.h" 
#include "foc_app.h"

// 🚀 核心重构：所有控制函数完美解耦，支持任意多路电机克隆调用
void setPhaseVoltage(foc_control_t *motor, float Uq, float Ud, float angle_el);
float _normalizeAngle(float angle);
float get_electrical_angle(float shaft_angle, int pole_pairs, float zero_offset);
void set_pwm(foc_control_t *motor, float Ua, float Ub, float Uc);   

void Forward_Clarke(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);
void Forward_Park(float Ialpha, float Ibeta, float angle_el, float *Id, float *Iq);

#endif
