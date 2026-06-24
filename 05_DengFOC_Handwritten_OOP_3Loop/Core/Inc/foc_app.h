#ifndef __FOC_APP_H
#define __FOC_APP_H

#include "main.h"
#include "pid.h"

// 通用低通滤波器结构体
typedef struct {
    float out_last;
    float Tf;       
} LPF_t;

// FOC 航空母舰级核心控制对象 (完全面向对象封装)
typedef struct foc_control_t {
    // ================== 1. 绑定该电机的专属硬件资源 ==================
    TIM_HandleTypeDef *htim;       // 绑定的高级发波定时器 (&htim1 或 &htim8)
    ADC_HandleTypeDef *hadc;       // 绑定的电流采样ADC (&hadc1 或 &hadc2)
    I2C_HandleTypeDef *hi2c;       // 绑定的位置编码器I2C句柄
    uint32_t adc_rank_ia;          // A相在注入组中的Rank位置 (如 ADC_INJECTED_RANK_1)
    uint32_t adc_rank_ib;          // B相在注入组中的Rank位置
    uint32_t adc_rank_ic;          // C相在注入组中的Rank位置

    // ================== 2. 物理层传感器原始反馈值 ==================
    uint16_t raw_angle;            // ?? 补齐：AS5600读出的 0-4095 原始值
    float last_raw_angle;          // ?? 补齐：上一次的单圈弧度值
    int32_t count_loops;           // ?? 补齐：多圈累加圈数

    // ================== 3. 运动学解算中间态变量 ==================
    int pole_pairs;             
    float voltage_power_supply; 
    float zero_electric_angle;  
    float shaft_angle;             // 带圈数的绝对机械角度 (rad)
    float electric_angle;          // 定子归一化电角度 (rad)
    
    // ================== 4. 串级控制链条状态空间 ==================
    float target_pos;              // 最外环：目标位置 (rad)
    float target_velocity;         // 中间环：目标速度 (rad/s)
    float current_velocity;        // 差分原始速度
    float filtered_velocity;       // 滤波后速度反馈
    float last_shaft_angle;        // 历史机械角度
    
    // 电流环反馈
    float adc_offset_ia; float adc_offset_ib; float adc_offset_ic; 
    float Ia; float Ib; float Ic;  // 还原后的物理真实相电流 (A)
    float Ialpha; float Ibeta;    // Clarke变换结果
    float Id; float Iq;           // Park变换结果
    float Id_filtered; float Iq_filtered; 

    // 各环控制输出
    float Id_target; float Iq_target; 
    float Ud; float Uq;            // 电流环PID算出的控制电压
    float Ua; float Ub; float Uc;  // 逆变换求出的定子相电压
    
    uint32_t ccr1;                 // ?? 补齐：通道1物理寄存器比较值
    uint32_t ccr2;                 // ?? 补齐：通道2物理寄存器比较值
    uint32_t ccr3;                 // ?? 补齐：通道3物理寄存器比较值

    // ================== 5. 专属回路闭星控制器与滤波器 ==================
    PID_TypeDef PosPID;
    PID_TypeDef SpeedPID;
    PID_TypeDef Id_PID;
    PID_TypeDef Iq_PID;

    LPF_t SpeedFilter;
    LPF_t Id_Filter;
    LPF_t Iq_Filter;
} foc_control_t;

// 声明双路全局实例
extern foc_control_t motor1;
extern foc_control_t motor2;

// 全局核心API业务接口 (全部带指针，彻底实现可重入)
void FOC_Init(foc_control_t *motor);
void FOC_Core_Loop(foc_control_t *motor);
void FOC_Current_Calibration(foc_control_t *motor);
void FOC_Task_1ms(void);

float LPF_Compute(LPF_t *filter, float in);
void angle_trans_vel(foc_control_t *motor);

#endif
