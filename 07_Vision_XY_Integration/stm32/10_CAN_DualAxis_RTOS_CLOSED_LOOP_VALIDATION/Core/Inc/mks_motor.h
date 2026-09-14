#ifndef __MKS_MOTOR_H
#define __MKS_MOTOR_H

#include "main.h"

typedef enum
{
    MKS_MOVE_ACK_FAIL     = 0x00U,
    MKS_MOVE_ACK_START    = 0x01U,
    MKS_MOVE_ACK_COMPLETE = 0x02U,
    MKS_MOVE_ACK_STOPPED  = 0x03U,

    /* No valid F4 status received. */
    MKS_MOVE_ACK_NONE     = 0xFFU

} MKS_MoveAck_t;

typedef struct
{
    MKS_MoveAck_t ack;
    uint32_t seq;

    /*
     * Sticky per-status counters.
     * These prevent a later ACK from overwriting an earlier FAIL/STOP
     * before ControlTask has a chance to observe it.
     */
    uint32_t start_count;
    uint32_t complete_count;
    uint32_t fail_count;
    uint32_t stopped_count;

} MKS_MoveFeedback_t;

typedef struct
{
    uint32_t tx_id;
    uint32_t rx_id;

    uint8_t is_enabled;

    /* Updated in CAN RX interrupt after a valid 0x31 reply. */
    volatile int64_t actual_encoder_val;
    volatile uint8_t encoder_valid;
    volatile uint32_t encoder_seq;
    volatile uint32_t encoder_tick;

    /*
     * F4 status snapshot + monotonic counters.
     * move_ack_seq increments for every accepted F4 status.
     */
    volatile MKS_MoveAck_t move_ack;
    volatile uint32_t move_ack_seq;
    volatile uint32_t move_start_count;
    volatile uint32_t move_complete_count;
    volatile uint32_t move_fail_count;
    volatile uint32_t move_stopped_count;

} MKS_Motor_t;

extern MKS_Motor_t Motor_X;
extern MKS_Motor_t Motor_Y;
extern CAN_HandleTypeDef hcan1;

void MKS_Motor_System_Init(void);
void STM32_bxCAN_Filter_Config(void);

HAL_StatusTypeDef MKS_Set_Enable_State(
    MKS_Motor_t *motor,
    uint8_t enable_state);

HAL_StatusTypeDef MKS_Read_Absolute_Position(
    MKS_Motor_t *motor);

void MKS_Set_Work_Mode(
    MKS_Motor_t *motor,
    uint8_t mode);

HAL_StatusTypeDef MKS_Move_Relative_Axis(
    MKS_Motor_t *motor,
    uint16_t speed,
    uint8_t acc,
    int32_t rel_axis);

void MKS_Parse_Feedback_Payload(
    uint32_t std_id,
    const uint8_t *rx_data,
    uint8_t dlc);

void MKS_Read_Encoder_Safe(
    MKS_Motor_t *motor,
    int64_t *value,
    uint32_t *seq,
    uint32_t *tick,
    uint8_t *valid);

void MKS_Read_Move_Feedback_Safe(
    MKS_Motor_t *motor,
    MKS_MoveFeedback_t *feedback);

uint32_t MKS_Get_Move_Ack_Seq(
    MKS_Motor_t *motor);

uint32_t MKS_Get_Encoder_Seq(
    MKS_Motor_t *motor);

#endif
