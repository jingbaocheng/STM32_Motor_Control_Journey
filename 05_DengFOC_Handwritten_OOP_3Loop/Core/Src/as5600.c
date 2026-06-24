#include "foc_app.h"
#include "as5600.h"
#include "i2c.h"


// 定义常数
#define _PI 3.1415926535f
#define _2PI 6.283185307f



//uint16_t AS5600_Read_Raw(void)        
//    
//  {
//    uint8_t buf[2]; // 存高低两个字节
//    

//    /* 
//     * 核心函数：HAL_I2C_Mem_Read 
//     * 参数含义：I2C句柄, 设备地址, 寄存器地址, 寄存器地址长度, 数据缓冲区, 读取长度, 超时时间
//     */
// if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_RAW_ANG_REG, 
//                         I2C_MEMADD_SIZE_8BIT, buf, 2, 10) == HAL_OK) 
// {
//        return ((uint16_t)buf[0] << 8) | buf[1];
//    }
//    return foc.raw_angle; // 失败则返回上次的值，防止跳变
//}
uint16_t AS5600_Read_Raw(foc_control_t *motor) {
    uint8_t buf[2];
    // 🚀 妙处：通过 motor->hi2c 动态决定读哪一路 I2C 接口
    if (HAL_I2C_Mem_Read(motor->hi2c, AS5600_ADDR, AS5600_RAW_ANG_REG, 
                         I2C_MEMADD_SIZE_8BIT, buf, 2, 10) == HAL_OK) {
        return ((uint16_t)buf[0] << 8) | buf[1];
    }
    return motor->raw_angle; 
}

//void update_shaft_angle(void) {
//    
//   // 1. 读取原始值
//    foc.raw_angle = AS5600_Read_Raw();
//    
//    // 2. 将原始值转为当前的弧度 (0 ~ 6.28)
//    float current_angle = (float)foc.raw_angle / 4096.0f * _2PI;
//    
//    // 3. 处理圈数（这是最关键的一步！）
//    // 我们需要判断：电机是从 6.28 跨向 0 了（加一圈），还是从 0 跨向 6.28 了（减一圈）
//    
//    // 计算这次读取的角度和上次读取的角度之间的差值
//    float d_angle = current_angle - foc.last_raw_angle;
//    
//    // 如果差值特别大（比如超过了 PI），说明它肯定跨越了 0 点
//    if (d_angle < -_PI) {
//        foc.count_loops++; // 顺时针转过 0 点，圈数加 1
//    } 
//    else if (d_angle > _PI) {
//        foc.count_loops--; // 逆时针转过 0 点，圈数减 1
//    }
//    
//    // 4. 更新总的机械角度 (带圈数的)
//    foc.shaft_angle = (float)foc.count_loops * _2PI + current_angle;
//    
//    // 5. 保存当前角度，为下一次计算做准备
//    foc.last_raw_angle = current_angle;
//}
void update_shaft_angle(foc_control_t *motor) {
    motor->raw_angle = AS5600_Read_Raw(motor);
    float current_angle = (float)motor->raw_angle / 4096.0f * _2PI;
    
    float d_angle = current_angle - motor->last_raw_angle;
    if (d_angle < -_PI) {
        motor->count_loops++;
    } else if (d_angle > _PI) {
        motor->count_loops--;
    }
    
    motor->shaft_angle = (float)motor->count_loops * _2PI + current_angle;
    motor->last_raw_angle = current_angle;
}