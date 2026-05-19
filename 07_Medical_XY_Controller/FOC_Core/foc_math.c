
#include "foc_math.h"
#include <math.h> // 暂时用标准数学库
void FOC_Set_Phase_Voltage(FOC_Controller_t *foc,FOC_Config_t *config, float uq, float angle_elec) {

  // 1. 电压限幅（防止过调制）
    if (uq > config->voltage_limit) uq = config->voltage_limit;
    if (uq < -config->voltage_limit) uq = -config->voltage_limit;

    // 2. 角度处理
   foc->electrical_angle = angle_elec;
    float s = sinf(angle_elec);
    float c = cosf(angle_elec);

 // 3. 逆 Park 变换 (D/Q 坐标系 -> Alpha/Beta 坐标系)
    // 假设 Ud = 0
    foc->alpha = -uq * s;
    foc->beta  =  uq * c;
    // 4. 逆 Clarke 变换 (Alpha/Beta -> A/B/C 三相电压)
    //  (专业系数：0.8660254f 即 sqrt(3)/2)
    // 这里我们先算出相对于中心点的偏移
    float va_temp = foc->alpha;
    float vb_temp = -0.5f *  foc->alpha + 0.8660254f *  foc->beta;
    float vc_temp = -0.5f *  foc->alpha - 0.8660254f *  foc->beta;

// 5. 归一化并映射到 PWM 占空比（核心专业逻辑）
    // 将 -Vcc~Vcc 的电压 映射到 0~1.0 的占空比
    // 这样做的好处是：以后换了不同电压的电机，这套算法一个字都不用改！
    float v_offset = config->voltage_power_supply / 2.0f;
   foc->v_a = (va_temp + v_offset) / config->voltage_power_supply;
    foc->v_b = (vb_temp + v_offset) / config->voltage_power_supply;
    foc->v_c = (vc_temp + v_offset) / config->voltage_power_supply;

}

