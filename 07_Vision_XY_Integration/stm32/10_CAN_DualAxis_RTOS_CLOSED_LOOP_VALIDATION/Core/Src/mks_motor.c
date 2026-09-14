#include "mks_motor.h"
#include "can.h"
#include <string.h>

MKS_Motor_t Motor_X;
MKS_Motor_t Motor_Y;

static HAL_StatusTypeDef MKS_TxFrame(
    MKS_Motor_t *motor,
    uint8_t *payload,
    uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0U;

    tx_header.StdId = motor->tx_id;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    /*
     * A temporarily full mailbox is not a protocol failure.
     * The caller may retry on the next ControlTask cycle.
     */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(
        &hcan1,
        &tx_header,
        payload,
        &tx_mailbox);
}

static MKS_Motor_t *MKS_FindMotorByStdId(uint32_t std_id)
{
    if (std_id == Motor_X.rx_id)
    {
        return &Motor_X;
    }

    if (std_id == Motor_Y.rx_id)
    {
        return &Motor_Y;
    }

    return NULL;
}

static void MKS_ResetMotorState(
    MKS_Motor_t *motor,
    uint32_t tx_id,
    uint32_t rx_id)
{
    motor->tx_id = tx_id;
    motor->rx_id = rx_id;
    motor->is_enabled = 0U;

    motor->actual_encoder_val = 0;
    motor->encoder_valid = 0U;
    motor->encoder_seq = 0U;
    motor->encoder_tick = 0U;

    motor->move_ack = MKS_MOVE_ACK_NONE;
    motor->move_ack_seq = 0U;
    motor->move_start_count = 0U;
    motor->move_complete_count = 0U;
    motor->move_fail_count = 0U;
    motor->move_stopped_count = 0U;
}

void MKS_Motor_System_Init(void)
{
    MKS_ResetMotorState(&Motor_X, 0x01U, 0x01U);
    MKS_ResetMotorState(&Motor_Y, 0x02U, 0x02U);
}

void STM32_bxCAN_Filter_Config(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14;

    /* Motor X, StdId 0x01 -> FIFO0 */
    filter.FilterBank = 0;
    filter.FilterIdHigh = (0x01U << 5);
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = (0x7FFU << 5);
    filter.FilterMaskIdLow = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* Motor Y, StdId 0x02 -> FIFO1 */
    filter.FilterBank = 1;
    filter.FilterIdHigh = (0x02U << 5);
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = (0x7FFU << 5);
    filter.FilterMaskIdLow = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO1;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef MKS_Set_Enable_State(
    MKS_Motor_t *motor,
    uint8_t enable_state)
{
    uint8_t tx_buf[3] = {0};
    HAL_StatusTypeDef status;

    tx_buf[0] = 0xF3U;
    tx_buf[1] = enable_state;
    tx_buf[2] = (uint8_t)(
        (motor->tx_id + tx_buf[0] + tx_buf[1]) & 0xFFU);

    status = MKS_TxFrame(motor, tx_buf, 3U);

    if (status == HAL_OK)
    {
        /*
         * This flag means "enable frame queued successfully".
         * It is not a driver-side acknowledgement.
         */
        motor->is_enabled = enable_state;
    }

    return status;
}

HAL_StatusTypeDef MKS_Read_Absolute_Position(
    MKS_Motor_t *motor)
{
    uint8_t tx_buf[2] = {0};

    tx_buf[0] = 0x31U;
    tx_buf[1] = (uint8_t)(
        (motor->tx_id + tx_buf[0]) & 0xFFU);

    return MKS_TxFrame(motor, tx_buf, 2U);
}

void MKS_Set_Work_Mode(
    MKS_Motor_t *motor,
    uint8_t mode)
{
    uint8_t tx_buf[3] = {0};

    tx_buf[0] = 0x82U;
    tx_buf[1] = mode;
    tx_buf[2] = (uint8_t)(
        (motor->tx_id + tx_buf[0] + tx_buf[1]) & 0xFFU);

    (void)MKS_TxFrame(motor, tx_buf, 3U);
}

HAL_StatusTypeDef MKS_Move_Relative_Axis(
    MKS_Motor_t *motor,
    uint16_t speed,
    uint8_t acc,
    int32_t rel_axis)
{
    uint8_t tx_buf[8] = {0};
    uint32_t rel24 = ((uint32_t)rel_axis) & 0x00FFFFFFU;

    /*
     * Keep the already verified F4 encoding.
     * rel_axis is encoded as signed 24-bit two's complement.
     */
    tx_buf[0] = 0xF4U;
    tx_buf[1] = (uint8_t)(speed >> 8);
    tx_buf[2] = (uint8_t)(speed & 0xFFU);
    tx_buf[3] = acc;
    tx_buf[4] = (uint8_t)((rel24 >> 16) & 0xFFU);
    tx_buf[5] = (uint8_t)((rel24 >> 8) & 0xFFU);
    tx_buf[6] = (uint8_t)(rel24 & 0xFFU);
    tx_buf[7] = (uint8_t)(
        (motor->tx_id +
         tx_buf[0] +
         tx_buf[1] +
         tx_buf[2] +
         tx_buf[3] +
         tx_buf[4] +
         tx_buf[5] +
         tx_buf[6]) & 0xFFU);

    return MKS_TxFrame(motor, tx_buf, 8U);
}

void MKS_Parse_Feedback_Payload(
    uint32_t std_id,
    const uint8_t *rx_data,
    uint8_t dlc)
{
    MKS_Motor_t *motor;

    if ((rx_data == NULL) || (dlc == 0U))
    {
        return;
    }

    motor = MKS_FindMotorByStdId(std_id);

    if (motor == NULL)
    {
        return;
    }

    /*
     * Only parse bytes that are actually present in this CAN frame.
     *
     * We intentionally validate the minimum number of bytes consumed by the
     * parser instead of assuming an unverified checksum format:
     *   0x31 parser consumes data[0..6] -> DLC >= 7
     *   0xF4 parser consumes data[0..1] -> DLC >= 2
     *
     * Once the exact reply checksum format is confirmed from the matching MKS
     * protocol document, this can be tightened further.
     */
    if ((rx_data[0] == 0x31U) && (dlc >= 7U))
    {
        int64_t raw = 0;

        raw |= ((int64_t)rx_data[1] << 40);
        raw |= ((int64_t)rx_data[2] << 32);
        raw |= ((int64_t)rx_data[3] << 24);
        raw |= ((int64_t)rx_data[4] << 16);
        raw |= ((int64_t)rx_data[5] << 8);
        raw |= ((int64_t)rx_data[6]);

        if ((raw & ((int64_t)1 << 47)) != 0)
        {
            raw |= (int64_t)0xFFFF000000000000ULL;
        }

        motor->actual_encoder_val = raw;
        motor->encoder_valid = 1U;
        motor->encoder_tick = HAL_GetTick();
        motor->encoder_seq++;
    }
    else if ((rx_data[0] == 0xF4U) && (dlc >= 2U))
    {
        MKS_MoveAck_t ack = MKS_MOVE_ACK_NONE;
        uint8_t valid_ack = 1U;

        switch (rx_data[1])
        {
            case 0x00U:
                ack = MKS_MOVE_ACK_FAIL;
                break;

            case 0x01U:
                ack = MKS_MOVE_ACK_START;
                break;

            case 0x02U:
                ack = MKS_MOVE_ACK_COMPLETE;
                break;

            case 0x03U:
                ack = MKS_MOVE_ACK_STOPPED;
                break;

            default:
                valid_ack = 0U;
                break;
        }

        if (valid_ack != 0U)
        {
            motor->move_ack = ack;
            motor->move_ack_seq++;

            switch (ack)
            {
                case MKS_MOVE_ACK_START:
                    motor->move_start_count++;
                    break;

                case MKS_MOVE_ACK_COMPLETE:
                    motor->move_complete_count++;
                    break;

                case MKS_MOVE_ACK_FAIL:
                    motor->move_fail_count++;
                    break;

                case MKS_MOVE_ACK_STOPPED:
                    motor->move_stopped_count++;
                    break;

                default:
                    break;
            }
        }
    }
}

void MKS_Read_Encoder_Safe(
    MKS_Motor_t *motor,
    int64_t *value,
    uint32_t *seq,
    uint32_t *tick,
    uint8_t *valid)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    *value = motor->actual_encoder_val;
    *seq = motor->encoder_seq;
    *tick = motor->encoder_tick;
    *valid = motor->encoder_valid;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void MKS_Read_Move_Feedback_Safe(
    MKS_Motor_t *motor,
    MKS_MoveFeedback_t *feedback)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    feedback->ack = motor->move_ack;
    feedback->seq = motor->move_ack_seq;
    feedback->start_count = motor->move_start_count;
    feedback->complete_count = motor->move_complete_count;
    feedback->fail_count = motor->move_fail_count;
    feedback->stopped_count = motor->move_stopped_count;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint32_t MKS_Get_Move_Ack_Seq(
    MKS_Motor_t *motor)
{
    uint32_t seq;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    seq = motor->move_ack_seq;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return seq;
}

uint32_t MKS_Get_Encoder_Seq(
    MKS_Motor_t *motor)
{
    uint32_t seq;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    seq = motor->encoder_seq;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return seq;
}

static void MKS_Drain_RxFifo(
    CAN_HandleTypeDef *hcan,
    uint32_t fifo)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buf[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0U)
    {
        /*
         * HAL copies only DLC bytes. Clear the whole buffer on every frame so
         * a short but otherwise valid CAN frame can never reuse bytes from the
         * preceding frame.
         */
        memset(rx_buf, 0, sizeof(rx_buf));

        if (HAL_CAN_GetRxMessage(
                hcan,
                fifo,
                &rx_header,
                rx_buf) != HAL_OK)
        {
            break;
        }

        if ((rx_header.IDE == CAN_ID_STD) &&
            (rx_header.RTR == CAN_RTR_DATA))
        {
            MKS_Parse_Feedback_Payload(
                rx_header.StdId,
                rx_buf,
                (uint8_t)rx_header.DLC);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        MKS_Drain_RxFifo(
            hcan,
            CAN_RX_FIFO0);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(
    CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        MKS_Drain_RxFifo(
            hcan,
            CAN_RX_FIFO1);
    }
}
