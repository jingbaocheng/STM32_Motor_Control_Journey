#include "motion_control.h"
#include "stm32f4xx_hal.h"

static int64_t ABS64(int64_t x)
{
    return (x >= 0) ? x : -x;
}

void Motion_Init(MotionController_t *motion, MKS_Motor_t *motor)
{
    motion->motor = motor;

    motion->state = MOTION_IDLE;

    motion->start_pos = 0;
    motion->target_pos = 0;
    motion->actual_pos = 0;
    motion->last_pos = 0;
    motion->pos_error = 0;
    motion->pos_delta = 0;

    motion->move_axis = 0;
    motion->speed = 0;
    motion->acc = 0;

    motion->tolerance = 80;
    motion->still_threshold = 10;

    motion->last_read_tick = 0;
    motion->prepare_count = 0;
    motion->motion_done = 0;
}

void Motion_StartRelative(MotionController_t *motion, int32_t move_axis, uint16_t speed, uint8_t acc)
{
    if (motion->state != MOTION_IDLE && motion->state != MOTION_DONE)
    {
        return;
    }

    motion->move_axis = move_axis;
    motion->speed = speed;
    motion->acc = acc;

    motion->prepare_count = 0;
    motion->motion_done = 0;

    motion->state = MOTION_PREPARE;
}

void Motion_Task(MotionController_t *motion)
{
    if (HAL_GetTick() - motion->last_read_tick < 50)
    {
        return;
    }

    motion->last_read_tick = HAL_GetTick();

    MKS_Read_Absolute_Position(motion->motor);

    motion->actual_pos = motion->motor->actual_encoder_val;
    motion->pos_delta = motion->actual_pos - motion->last_pos;
    motion->last_pos = motion->actual_pos;

    switch (motion->state)
    {
        case MOTION_IDLE:
            break;

        case MOTION_PREPARE:
            motion->prepare_count++;

            if (motion->prepare_count >= 3)
            {
                motion->start_pos = motion->actual_pos;
                motion->target_pos = motion->start_pos + motion->move_axis;
                motion->last_pos = motion->actual_pos;

                MKS_Move_Relative_Axis(motion->motor,
                                       motion->speed,
                                       motion->acc,
                                       motion->move_axis);

                motion->state = MOTION_MOVING;
            }
            break;

        case MOTION_MOVING:
            motion->pos_error = motion->target_pos - motion->actual_pos;

            if ((ABS64(motion->pos_error) < motion->tolerance) &&
                (ABS64(motion->pos_delta) < motion->still_threshold))
            {
                motion->motion_done = 1;
                motion->state = MOTION_DONE;
            }
            break;

        case MOTION_DONE:
            break;

        case MOTION_ERROR:
            break;

        default:
            motion->state = MOTION_ERROR;
            break;
    }
}
