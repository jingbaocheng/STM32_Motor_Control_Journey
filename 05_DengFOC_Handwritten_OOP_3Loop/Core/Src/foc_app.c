#include "foc_app.h"
#include "i2c.h"
#include <math.h>
#include "as5600.h"
#include "pid.h"
#include "vofa_plus.h"
#include "foc_ctrl.h"
#include "adc.h"

extern TIM_HandleTypeDef htim1; 
// 实例化所有控制单元实体
// 1. 实例化两个完全隔离的独立电机对象
foc_control_t motor1;
foc_control_t motor2;

#define _PI 3.1415926535f
#define _2PI 6.283185307f

#ifndef constrain
#define constrain(x,min,mx) ((x)<(min)?(min) :((x)>(mx)?(mx):(x)))
#endif

void angle_trans_vel(foc_control_t *motor) {
    float Ts = 0.001f;
    float d_angle = motor->shaft_angle - motor->last_shaft_angle;
    motor->current_velocity = d_angle / Ts;
    motor->filtered_velocity = LPF_Compute(&motor->SpeedFilter, motor->current_velocity);
    motor->last_shaft_angle = motor->shaft_angle; // 记忆带圈数的历史
}

/**
 * @brief 医疗级相电流静态零点开机大数校准器
 */
void FOC_Current_Calibration(foc_control_t *motor) {
    uint32_t sum_ia = 0, sum_ib = 0, sum_ic = 0;
    set_pwm(motor, 0.0f, 0.0f, 0.0f);
    HAL_Delay(50);
    
    for (int i = 0; i < 1000; i++) {
        HAL_ADCEx_InjectedStart(motor->hadc); // 开启对应的 ADC
        if (HAL_ADC_PollForConversion(motor->hadc, 10) == HAL_OK) {
            // ?? 绝妙之处：通过绑定的 rank 座位号精准打捞数据
            sum_ia += HAL_ADCEx_InjectedGetValue(motor->hadc, motor->adc_rank_ia);
            sum_ib += HAL_ADCEx_InjectedGetValue(motor->hadc, motor->adc_rank_ib);
            sum_ic += HAL_ADCEx_InjectedGetValue(motor->hadc, motor->adc_rank_ic);
        }
    }
    motor->adc_offset_ia = (float)sum_ia / 1000.0f;
    motor->adc_offset_ib = (float)sum_ib / 1000.0f;
    motor->adc_offset_ic = (float)sum_ic / 1000.0f;
}


// 工业级初始化：参数灌入与静态校准
void FOC_Init(foc_control_t *motor) {
    motor->pole_pairs = 7;
    motor->voltage_power_supply = 12.6f; 
    motor->zero_electric_angle = 0.0f;
    
    // 关卡 1：位置外环限幅设定
    motor->PosPID.Kp = 1.5f;   motor->PosPID.Ki = 0.0f;   motor->PosPID.Kd = 0.0f;
    motor->PosPID.OutputLimit = 15.0f;     motor->PosPID.IntegralLimit = 0.0f; 

    // 关卡 2：速度内环限幅设定
    motor->SpeedPID.Kp = 0.08f; motor->SpeedPID.Ki = 0.5f; motor->SpeedPID.Kd = 0.0f;
    motor->SpeedPID.OutputLimit = 3.0f;    motor->SpeedPID.IntegralLimit = 0.8f;

    // 关卡 3：电流内环双子星设定
    motor->Iq_PID.Kp = 0.15f;  motor->Iq_PID.Ki = 18.0f;  motor->Iq_PID.Kd = 0.0f;
    motor->Iq_PID.OutputLimit = 6.0f;  motor->Iq_PID.IntegralLimit = 1.5f;

    motor->Id_PID.Kp = 0.15f;  motor->Id_PID.Ki = 18.0f;  motor->Id_PID.Kd = 0.0f;
    motor->Id_PID.OutputLimit = 6.0f;  motor->Id_PID.IntegralLimit = 1.5f;
    
    motor->SpeedFilter.Tf = 0.01f;   
    motor->Id_Filter.Tf = 0.004f;
    motor->Iq_Filter.Tf = 0.004f;

    // 动态执行当前电机的静态校准
    FOC_Current_Calibration(motor);
}
// ?? 核心重构：完全体三闭环串级 FOC 控制内核 (无全局变量污染，完全支持并发重入)
void FOC_Core_Loop(foc_control_t *motor) {
    // ==================== 1. 反馈采集层 ====================
    update_shaft_angle(motor); // 传入电机的专属句柄
    angle_trans_vel(motor); 
    motor->electric_angle = get_electrical_angle(motor->shaft_angle, motor->pole_pairs, motor->zero_electric_angle);

    // 硬件真实注入采样提取
    HAL_ADCEx_InjectedStart(motor->hadc);
    float adc_to_amps = (3.3f / 4095.0f) * 2.0f; 
    
    motor->Ia = ((float)HAL_ADCEx_InjectedGetValue(motor->hadc, motor->adc_rank_ia) - motor->adc_offset_ia) * adc_to_amps;
    motor->Ib = ((float)HAL_ADCEx_InjectedGetValue(motor->hadc, motor->adc_rank_ib) - motor->adc_offset_ib) * adc_to_amps;
    motor->Ic = ((float)HAL_ADCEx_InjectedGetValue(motor->hadc, motor->adc_rank_ic) - motor->adc_offset_ic) * adc_to_amps;

    // 前向降维变换
    Forward_Clarke(motor->Ia, motor->Ib, motor->Ic, &motor->Ialpha, &motor->Ibeta);
    Forward_Park(motor->Ialpha, motor->Ibeta, motor->electric_angle, &motor->Id, &motor->Iq);
    
    motor->Id_filtered = LPF_Compute(&motor->Id_Filter, motor->Id);
    motor->Iq_filtered = LPF_Compute(&motor->Iq_Filter, motor->Iq);

    // ==================== 2. 串级套娃控制层 ====================
    // A. 位置环计算 -> 输出目标速度
    float pos_error = motor->target_pos - motor->shaft_angle;
    motor->target_velocity = PIDController(&motor->PosPID, pos_error);

    // B. 速度环计算 -> 输出目标 Iq 电流
    float vel_error = motor->target_velocity - motor->filtered_velocity;
    motor->Iq_target = PIDController(&motor->SpeedPID, vel_error);
    motor->Id_target = 0.0f; 

    // C. 电流环计算 -> 输出所需的控制基准电压 Ud, Uq
    float Iq_error = motor->Iq_target - motor->Iq_filtered;
    float Id_error = motor->Id_target - motor->Id_filtered;
    
    motor->Uq = PIDController(&motor->Iq_PID, Iq_error);
    motor->Ud = PIDController(&motor->Id_PID, Id_error);

    // ==================== 3. 执行驱动层 ====================
    setPhaseVoltage(motor, motor->Uq, motor->Ud, motor->electric_angle);
}
// 这是 1ms 中断调用的总指挥部
// ?? 1ms 硬件定时中断总指挥
void FOC_Task_1ms(void) {
    // 轮流把两路电机丢进同一个数学计算内核，绝不重复拷贝面条代码！
    FOC_Core_Loop(&motor1);
    FOC_Core_Loop(&motor2);
 // 2. 推进双轴核心控制流 (坐标变换、三环套娃、SVPWM 发波)
    FOC_Core_Loop(&motor1); // 算 1 号轴 (TIM1)
    FOC_Core_Loop(&motor2); // 算 2 号轴 (TIM8)
    
    // 3. ?? 【补齐核心调用】VOFA 数据上报分频器 (每 10ms 发送一次，防止串口堵死)
    static uint8_t vofa_tick = 0;
    vofa_tick++;
    if (vofa_tick >= 10) { 
        vofa_tick = 0;
        
        // 我们可以通过一个开关或者分时发送来观察不同轴
        VOFA_SendMotorData(&motor1); 
        // 如果想看电机 2，可以改为 VOFA_SendMotorData(&motor2);
    }
    
}
float LPF_Compute(LPF_t *filter, float in) {
    float Ts = 0.001f; // 1ms 运行周期
    float alpha = Ts / (filter->Tf + Ts);
    float out = alpha * in + (1.0f - alpha) * filter->out_last;
    filter->out_last = out;
    return out;
}

