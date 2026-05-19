#include "bsp_encoder_as5600.c"
// 假设你在 CubeMX 里配置的是 I2C1
extern I2C_HandleTypeDef hi2c1; 
uint16_t AS5600_Read_Raw(void) {
    uint8_t buf[2]; // 存高低两个字节
    uint16_t raw_data;

    /* 
     * 核心函数：HAL_I2C_Mem_Read 
     * 参数含义：I2C句柄, 设备地址, 寄存器地址, 寄存器地址长度, 数据缓冲区, 读取长度, 超时时间
     */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_RAW_ANG_REG, 
                                               I2C_MEMADD_SIZE_8BIT, buf, 2, 100);

    if (status != HAL_OK) {
        return 0; // 如果读取失败，返回0 (实际工程中需要错误处理)
    }

    // 数据拼接：高字节在 buf[0]，低字节在 buf[1]
    // 注意：AS5600 的 0x0C 寄存器只有低 4 位有效
    raw_data = ((uint16_t)buf[0] << 8) | buf[1];
    
    return raw_data;
}

/**
 * @brief 将原始数据转换为弧度 (FOC 核心计算需要弧度)
 */
float AS5600_Get_Angle(void) {
    uint16_t raw = AS5600_Read_Raw();
    
    // 弧度 = (原始值 / 分辨率) * 2PI
    // 这里的 6.2831853f 就是 2 * PI
    return ((float)raw / 4096.0f) * 6.2831853f;
}