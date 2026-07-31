#include "mks_motor.h"
#include "can.h"  // 引入 CubeMX 自动生成的 hcan1 句柄

// 实例化物理内存房间
MKS_Motor_t Motor_X;
MKS_Motor_t Motor_Y;


/**
 * @brief  电机节点基因初始化
 */
void MKS_Motor_System_Init(void)
{
    Motor_X.tx_id = 0x01;
    Motor_X.rx_id = 0x01;
    Motor_X.is_enabled = 0;
    Motor_X.actual_encoder_val = 0;

    Motor_Y.tx_id = 0x02;
    Motor_Y.rx_id = 0x02;
    Motor_Y.is_enabled = 0;
    Motor_Y.actual_encoder_val = 0;
}

/**
 * @brief  bxCAN 接收过滤器配置 (32位掩码模式)
 * @note   精准放行来自 1 号电机的 0x101 反馈帧，其余总线杂讯在硬件层直接拦截
 */
void STM32_bxCAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

/**
 * @brief  下行军令：控制电机使能/释放 (对齐 v1.0.9 手册 0xF3 规范与求和校验)
 */
void MKS_Set_Enable_State(MKS_Motor_t *motor, uint8_t enable_state) {
   CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_buf[3] = {0};
    uint32_t tx_mailbox = 0;
    // 1. 装填 CAN 报文车头信息
    tx_header.StdId = motor->tx_id;       // 目标电机 ID (0x01)
    tx_header.RTR = CAN_RTR_DATA;         // 数据帧
    tx_header.IDE = CAN_ID_STD;           // 标准标识符
    tx_header.DLC = 3;                    // 固定3 字节长度
    tx_header.TransmitGlobalTime = DISABLE;
    // 2. 装填数据袋
    tx_buf[0] = 0xF3;                     // 使能控制命令码
    tx_buf[1] = enable_state;             // 0x01:使能锁死, 0x00:脱机释放
    tx_buf[2] = (uint8_t)((motor->tx_id + tx_buf[0] + tx_buf[1]) & 0xFF);

    // 4. 轰入总线邮箱
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buf, &tx_mailbox) == HAL_OK) {
        motor->is_enabled = enable_state; // 同步本地状态机
    }
}
/**
 * @brief  分布式上行报文中断解包引擎（指针可重入、无缩写设计）
 * @param  std_id:   当前在双绞线上抓到的反馈帧 ID
 * @param  rx_data:  传进来的数据包指针（指向硬件 FIFO0 内存地址，保护栈空间）
 */
void MKS_Parse_Feedback_Payload(uint32_t std_id, uint8_t *rx_data) {
    MKS_Motor_t *active_motor = NULL;

    // 1. 【动态路由分流】根据进站的 ID，瞬间判定这是谁吐回来的数据
    if (std_id == Motor_X.rx_id) {
        active_motor = &Motor_X;       // 路由锁定：这是 X 轴电机的回执 (0x101)
    } 
    else if (std_id == Motor_Y.rx_id) {
        active_motor = &Motor_Y;       // 路由锁定：这是 Y 轴电机的回执 (0x102)
    } 
    else {
        return;                        // 非法杂讯 ID，硬件拦截并无视
    }

    // 2. 【精准匹配说明书 31H 协议】
    if (rx_data[0] == 0x31) {
        // 大端拼装魔术：将连续的 6 个字节拼接成一个 64 位临时变量
        int64_t raw_48bit_value = 0;
        raw_48bit_value |= ((int64_t)rx_data[1] << 40); // 对应说明书 字节2
        raw_48bit_value |= ((int64_t)rx_data[2] << 32); // 对应说明书 字节3
        raw_48bit_value |= ((int64_t)rx_data[3] << 24); // 对应说明书 字节4
        raw_48bit_value |= ((int64_t)rx_data[4] << 16); // 对应说明书 字节5
        raw_48bit_value |= ((int64_t)rx_data[5] << 8);  // 对应说明书 字节6
        raw_48bit_value |= ((int64_t)rx_data[6]);       // 对应说明书 字节7

        // 3. 【核心数学防线：符号位扩展】
        // 因为 int48_t 的最高位（第47位）是符号位。如果它为 1，代表电机在反转，坐标是负数。
        // C 语言没有原生的 int48 类型，直接存入 64 位会导致负数丢失，我们必须强行把高 16 位补齐为 0xFFFF。
        if (raw_48bit_value & ((int64_t)1 << 47)) {
            raw_48bit_value |= 0xFFFF000000000000ULL;
        }

        // 4. 将满血复活的绝对位置值安全写入电机的专属房间
        active_motor->actual_encoder_val = raw_48bit_value;
    }
}
void MKS_Read_Absolute_Position(MKS_Motor_t *motor) {
   CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_buf[2] = {0};
    uint32_t tx_mailbox = 0;

    tx_header.StdId = motor->tx_id;     // 目标 1 号电机 (0x01)
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.DLC   = 2;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_buf[0] = 0x31;                   // 严格对照说明书：读取多圈绝对位置命令码

     tx_buf[1] = (uint8_t)((motor->tx_id + tx_buf[0]) & 0xFF);

    // 轰入总线
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buf, &tx_mailbox);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buf[8] = {0};
    
    if (hcan->Instance == CAN1) {
        // 核心铁律：必须调用 GetRxMessage 强行把报文从硬件 FIFO0 中弹出来！
        // 如果不弹出来，FIFO0 满载，中断标志位无法清除，单片机会瞬间死机。
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buf) == HAL_OK) {
            
            // 成功提到货后，立刻送入你写的静态分发交战区进行解析！
            MKS_Parse_Feedback_Payload(rx_header.StdId, rx_buf);
        }
    }
}
void MKS_Set_Work_Mode(MKS_Motor_t *motor, uint8_t mode)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_buf[3] = {0};
    uint32_t tx_mailbox = 0;

    tx_header.StdId = motor->tx_id;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 3;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_buf[0] = 0x82;
    tx_buf[1] = mode;
    tx_buf[2] = (uint8_t)((motor->tx_id + tx_buf[0] + tx_buf[1]) & 0xFF);

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buf, &tx_mailbox);
}
void MKS_Move_Relative_Axis(MKS_Motor_t *motor, uint16_t speed, uint8_t acc, int32_t rel_axis)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_buf[8] = {0};
    uint32_t tx_mailbox = 0;

   uint32_t rel24 = ((uint32_t)rel_axis) & 0xFFFFFF;
    
    tx_header.StdId = motor->tx_id;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_buf[0] = 0xF4;
    tx_buf[1] = (uint8_t)(speed >> 8);
    tx_buf[2] = (uint8_t)(speed & 0xFF);
    tx_buf[3] = acc;
    tx_buf[4] = (uint8_t)(rel24 >> 16);
    tx_buf[5] = (uint8_t)(rel24 >> 8);
    tx_buf[6] = (uint8_t)(rel24 & 0xFF);


    tx_buf[7] = (uint8_t)((motor->tx_id + tx_buf[0] + tx_buf[1] + tx_buf[2] +
                           tx_buf[3] + tx_buf[4] + tx_buf[5] + tx_buf[6]) & 0xFF);

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buf, &tx_mailbox);
}
