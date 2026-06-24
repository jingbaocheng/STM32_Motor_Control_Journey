
#include "foc_app.h"
#include "main.h"
#include "pid.h"




float PIDController(PID_TypeDef *pid, float error)
    {

 float Ts = 0.001f; // 固定的 1ms 中断周期

    pid->Error = error;
    
    // 1. 积分累加 (I)
  // 1. 积分累加 (I)
    pid->Integral += pid->Error * Ts;
        
 // 3. 积分限幅      
    if (pid->Integral > pid->IntegralLimit) 
        {
    pid->Integral = pid->IntegralLimit;
    }   else if (pid->Integral < -pid->IntegralLimit) 
    {
        pid->Integral = -pid->IntegralLimit;
    }


    pid->Out = (pid->Kp * pid->Error) + \
               (pid->Ki * pid->Integral) + \
               (pid->Kd * (pid->Error - pid->LastError) / Ts);
    
// 4. 限制输出上限 (防止 PID 算出超过电源能承受的电压)
if (pid->Out > pid->OutputLimit) {
    pid->Out = pid->OutputLimit;
} else if (pid->Out < -pid->OutputLimit) {
    pid->Out = -pid->OutputLimit;
}

    // 4. 存下本次误差，下次用
    pid->LastError = pid->Error;

    return pid->Out;
}

