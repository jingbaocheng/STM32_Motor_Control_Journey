#ifndef __PID_H
#define __PID_H

typedef struct{
float Kp;
float Ki;
 float Kd;
float Target;
float Actual;
 float Error;
 float LastError;
 float Integral;
 float Out;
    float OutputLimit; // 必须加：输出限幅（比如不能超过 12V）
    float IntegralLimit; // 必须加：积分限幅（防止积分饱和导致的电机发疯） 
}PID_TypeDef;
extern PID_TypeDef PosPID ;
extern PID_TypeDef SpeedPID ;
float PIDController(PID_TypeDef *pid, float error);

#endif

