
#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include "main.h" // 包含基础数据类型

// 定义 FOC 控制结构体
typedef struct {
    // --- 1. 电机物理参数 ---
    uint8_t pole_pairs;      // 极对数 (PP)
    
    // --- 2. 反馈原值 (从 BSP 层获取) ---
    float mechanical_angle;  // 机械角度 (0~2*PI, 从编码器直接读取)
    float electrical_angle;  // 电角度 (mechanical_angle * pole_pairs)
    
    // --- 3. 电流采样反馈 (ABC -> alpha/beta -> d/q) ---
    float i_a, i_b, i_c;
    float alpha, beta;
    float d, q;
    
    // --- 4. 目标设定值 ---
    float target_d, target_q;
    
    // --- 5. 数学中间变量 ---
    float sin_theta;
    float cos_theta;
    
    // --- 6. 输出占空比/电压 ---
    float v_a, v_b, v_c; 
} FOC_Controller_t;

typedef struct {
    float voltage_power_supply; // 电源电压 (V)
    float voltage_limit;        // 输出限幅 (V)
    float pwm_period_max;       // 定时器 ARR 的值 (比如 168MHz 下 10kHz PWM 对应 16800)
} FOC_Config_t;

#endif