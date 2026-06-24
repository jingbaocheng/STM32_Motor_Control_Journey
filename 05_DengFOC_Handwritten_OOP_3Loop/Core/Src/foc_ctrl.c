
#include <stdio.h>
#include <string.h>
#include "foc_ctrl.h"
#include "tim.h"
#include "foc_app.h"
#include <math.h>
// --- 全局变量声明 ---
extern TIM_HandleTypeDef htim1;


// 基础宏定义
#ifndef constrain
#define constrain(x,min,mx) ((x)<(min)?(min) :((x)>(mx)?(mx):(x)))
#endif
#define _SQRT3 1.73205081f
#define _2PI 6.283185307f
#define _SQRT3_2 0.86602540378f  // 根号3除以2
#define SQRT3_DIV3 0.577350269f  // 1 / sqrt(3)

// 2. 角度归一化函数 (直接写在驱动文件里供内部调用)
float _normalizeAngle(float angle) {
    float a = fmod(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

// 2. 改进后的电角度计算
float get_electrical_angle(float shaft_angle, int pole_pairs, float zero_offset) {
    // 机械角度转电角度
    float el_angle = (shaft_angle * (float)pole_pairs) - zero_offset;
    // 必须归一化，让它永远在 0~2PI 之间
    return _normalizeAngle(el_angle);
}

// 🚀 完美修正：彻底抹除了全局变量 "foc"，全部改为通过指针写入对应 motor 内存空间
void setPhaseVoltage(foc_control_t *motor, float Uq, float Ud, float angle_el) {
    float s = sinf(angle_el);
    float c = cosf(angle_el);
    
    float Ualpha = Ud * c - Uq * s;
    float Ubeta  = Ud * s + Uq * c;

    motor->Ua = Ualpha;
    motor->Ub = (-0.5f * Ualpha) + (_SQRT3_2 * Ubeta);
    motor->Uc = (-0.5f * Ualpha) - (_SQRT3_2 * Ubeta);
    
    // 传递当前电机的指针下去
    set_pwm(motor, motor->Ua, motor->Ub, motor->Uc);
}

// 接收 motor 指针，自适应发波
void set_pwm(foc_control_t *motor, float Ua, float Ub, float Uc) {
    float v_bus = (motor->voltage_power_supply > 0.1f) ? motor->voltage_power_supply : 12.0f;

    float dc_a = constrain((Ua / v_bus) + 0.5f, 0.0f, 1.0f);
    float dc_b = constrain((Ub / v_bus) + 0.5f, 0.0f, 1.0f);
    float dc_c = constrain((Uc / v_bus) + 0.5f, 0.0f, 1.0f);

    // 🚀 核心硬件映射切换：通过指针获取当前电机的定时器 ARR 
    float timer_arr = (float)motor->htim->Instance->ARR;
    motor->ccr1 = (uint32_t)(dc_a * timer_arr);
    motor->ccr2 = (uint32_t)(dc_b * timer_arr);
    motor->ccr3 = (uint32_t)(dc_c * timer_arr);

    // 往对应的定时器里灌入占空比
    __HAL_TIM_SET_COMPARE(motor->htim, TIM_CHANNEL_1, motor->ccr1);
    __HAL_TIM_SET_COMPARE(motor->htim, TIM_CHANNEL_2, motor->ccr2);
    __HAL_TIM_SET_COMPARE(motor->htim, TIM_CHANNEL_3, motor->ccr3);
}
/**
 * @brief 前向克拉克变换 (Forward Clarke)
 * @note  将物理世界采集的三相交流电 Ia,Ib,Ic 拍扁降维到静止直角坐标系
 */
void Forward_Clarke(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta) {
    // 工业级精简算法，结合 Ia + Ib + Ic = 0 物理特性，只需 Ia 和 Ib 即可快速求解
    *Ialpha = Ia;
    *Ibeta = SQRT3_DIV3 * (Ia + 2.0f * Ib);
}

/**
 * @brief 前向帕克变换 (Forward Park)
 * @note  结合编码器实时电角度，将静止轴电流旋转投影到跟随转子同步转动的直轴(d)和交轴(q)
 */
void Forward_Park(float Ialpha, float Ibeta, float angle_el, float *Id, float *Iq) {
    float _sin = sinf(angle_el);
    float _cos = cosf(angle_el);
    
    *Id =  Ialpha * _cos + Ibeta * _sin;
    *Iq = -Ialpha * _sin + Ibeta * _cos;
}


