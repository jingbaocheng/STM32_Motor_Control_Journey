#include "motion_control.h"
#include "stm32f4xx_hal.h"

#define MOTION_F4_MIN_COUNTS   (-8388608LL)
#define MOTION_F4_MAX_COUNTS   ( 8388607LL)

static int64_t Motion_Abs64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t Motion_SeqAfter(
    uint32_t seq,
    uint32_t baseline)
{
    /*
     * Signed subtraction handles uint32_t wrap correctly as long as fewer than
     * 2^31 samples occur inside one motion transaction, which is guaranteed.
     */
    return (((int32_t)(seq - baseline)) > 0) ? 1U : 0U;
}

static uint32_t Motion_CalcTimeoutMs(int32_t move_axis)
{
    uint64_t distance;
    uint64_t timeout;

    distance = (move_axis < 0) ?
        (uint64_t)(-(int64_t)move_axis) :
        (uint64_t)move_axis;

    timeout = (uint64_t)MOTION_TIMEOUT_BASE_MS +
              (distance / MOTION_TIMEOUT_COUNTS_DIV);

    if (timeout > MOTION_TIMEOUT_MAX_MS)
    {
        timeout = MOTION_TIMEOUT_MAX_MS;
    }

    return (uint32_t)timeout;
}

static int64_t Motion_CalcTolerance(int32_t move_axis)
{
    int64_t distance = Motion_Abs64((int64_t)move_axis);
    int64_t tolerance;

    /*
     * For a non-zero command larger than the no-op floor:
     *   tolerance ~= half the move, capped at the old 32-count limit.
     *
     * Using (distance - 1) / 2 guarantees tolerance < distance, so the
     * pre-command position can never satisfy the completion condition.
     *
     * HOME_SLOW_STEP=64 therefore gets tolerance=31, almost identical to the
     * old 32-count behaviour while still fixing the short-move false-DONE bug.
     */
    tolerance = (distance - 1LL) / 2LL;

    if (tolerance > MOTION_DEFAULT_TOLERANCE)
    {
        tolerance = MOTION_DEFAULT_TOLERANCE;
    }

    if (tolerance < MOTION_MIN_TOLERANCE)
    {
        tolerance = MOTION_MIN_TOLERANCE;
    }

    return tolerance;
}

static void Motion_ClearStableTimer(MotionController_t *motion)
{
    motion->stable_start_tick = 0U;
    motion->stable_active = 0U;
}

static void Motion_Fail(
    MotionController_t *motion,
    MotionResultCode_t result,
    MotionErrorCode_t error_code)
{
    motion->last_result = result;
    motion->error_code = error_code;
    motion->state = MOTION_ERROR;
    Motion_ClearStableTimer(motion);
}

static void Motion_BeginPrepare(
    MotionController_t *motion,
    uint16_t speed,
    uint8_t acc,
    uint8_t stop_is_expected,
    MotionCompletionPolicy_t completion_policy)
{
    motion->speed = speed;
    motion->acc = acc;
    motion->stop_is_expected = stop_is_expected;
    motion->completion_policy = completion_policy;

    motion->last_result = MOTION_RESULT_NONE;
    motion->error_code = MOTION_ERR_NONE;

    motion->prepare_start_tick = HAL_GetTick();
    motion->move_start_tick = 0U;
    motion->timeout_ms = MOTION_TIMEOUT_BASE_MS;

    motion->ack_seq_start = 0U;
    motion->ack_start_count_start = 0U;
    motion->ack_complete_count_start = 0U;
    motion->ack_fail_count_start = 0U;
    motion->ack_stopped_count_start = 0U;

    motion->last_motion_tick = 0U;
    motion->start_seen = 0U;
    motion->complete_seen = 0U;

    Motion_ClearStableTimer(motion);

    motion->state = MOTION_PREPARE;
}

void Motion_Init(
    MotionController_t *motion,
    MKS_Motor_t *motor)
{
    motion->motor = motor;

    motion->state = MOTION_IDLE;
    motion->last_result = MOTION_RESULT_NONE;
    motion->error_code = MOTION_ERR_NONE;

    motion->start_pos = 0;
    motion->target_pos = 0;
    motion->actual_pos = 0;
    motion->home_offset = 0;

    motion->last_pos = 0;
    motion->pos_error = 0;
    motion->pos_delta = 0;

    motion->requested_move_axis = 0;
    motion->move_axis = 0;
    motion->target_machine = 0;
    motion->is_absolute = 0U;

    motion->speed = 0U;
    motion->acc = 0U;

    motion->tolerance = MOTION_DEFAULT_TOLERANCE;
    motion->still_threshold = MOTION_DEFAULT_STILL_THRESH;

    motion->last_poll_tick = HAL_GetTick();
    motion->last_encoder_seq = 0U;
    motion->last_encoder_tick = 0U;
    motion->position_valid = 0U;
    motion->polling_enabled = 1U;

    motion->prepare_start_tick = 0U;
    motion->move_start_tick = 0U;
    motion->timeout_ms = MOTION_TIMEOUT_BASE_MS;
    motion->move_encoder_seq_start = 0U;

    motion->ack_seq_start = 0U;
    motion->ack_start_count_start = 0U;
    motion->ack_complete_count_start = 0U;
    motion->ack_fail_count_start = 0U;
    motion->ack_stopped_count_start = 0U;

    motion->stable_start_tick = 0U;
    motion->stable_active = 0U;

    motion->last_motion_tick = 0U;
    motion->start_seen = 0U;
    motion->complete_seen = 0U;
    motion->completion_policy = MOTION_COMPLETE_POSITION_STABLE;
    motion->stop_is_expected = 0U;
}

static uint8_t Motion_StartRelativePolicy(
    MotionController_t *motion,
    int32_t move_axis,
    uint16_t speed,
    uint8_t acc,
    uint8_t stop_is_expected,
    MotionCompletionPolicy_t completion_policy)
{
    int64_t distance;

    if (Motion_IsBusy(motion))
    {
        return 0U;
    }

    if (((int64_t)move_axis < MOTION_F4_MIN_COUNTS) ||
        ((int64_t)move_axis > MOTION_F4_MAX_COUNTS))
    {
        motion->last_result = MOTION_RESULT_ERROR;
        motion->error_code = MOTION_ERR_RANGE;
        motion->state = MOTION_ERROR;
        return 0U;
    }

    distance = Motion_Abs64((int64_t)move_axis);

    /*
     * Explicit resolution floor:
     * do not send an F4 command that is smaller than the minimum meaningful
     * correction. This avoids the old "|move| <= tolerance" false-DONE case.
     */
    if (distance <= MOTION_MIN_COMMAND_COUNTS)
    {
        /*
         * Explicit no-op: keep diagnostic position fields self-consistent
         * instead of leaving target_pos from the previous transaction.
         */
        motion->requested_move_axis = 0;
        motion->move_axis = 0;
        motion->is_absolute = 0U;
        motion->start_pos = motion->actual_pos;
        motion->target_pos = motion->actual_pos;
        motion->pos_error = 0;
        motion->last_result = MOTION_RESULT_OK;
        motion->error_code = MOTION_ERR_NONE;
        motion->state = MOTION_DONE;
        return 1U;
    }

    motion->requested_move_axis = move_axis;
    motion->move_axis = move_axis;
    motion->is_absolute = 0U;

    Motion_BeginPrepare(
        motion,
        speed,
        acc,
        stop_is_expected,
        completion_policy);

    return 1U;
}

uint8_t Motion_StartRelativeEx(
    MotionController_t *motion,
    int32_t move_axis,
    uint16_t speed,
    uint8_t acc,
    uint8_t stop_is_expected)
{
    return Motion_StartRelativePolicy(
        motion,
        move_axis,
        speed,
        acc,
        stop_is_expected,
        MOTION_COMPLETE_POSITION_STABLE);
}

uint8_t Motion_StartHomeRelative(
    MotionController_t *motion,
    int32_t move_axis,
    uint16_t speed,
    uint8_t acc,
    uint8_t stop_is_expected)
{
    return Motion_StartRelativePolicy(
        motion,
        move_axis,
        speed,
        acc,
        stop_is_expected,
        MOTION_COMPLETE_HOME_ACK);
}

uint8_t Motion_StartRelative(
    MotionController_t *motion,
    int32_t move_axis,
    uint16_t speed,
    uint8_t acc)
{
    return Motion_StartRelativeEx(
        motion,
        move_axis,
        speed,
        acc,
        0U);
}

uint8_t Motion_StartAbsoluteMachine(
    MotionController_t *motion,
    int64_t target_machine,
    uint16_t speed,
    uint8_t acc)
{
    if (Motion_IsBusy(motion))
    {
        return 0U;
    }

    /*
     * Store only the machine-coordinate target here.
     * The absolute->relative conversion is deliberately deferred until
     * PREPARE has a fresh encoder sample, so target and residual share the
     * same physical-time reference.
     */
    motion->target_machine = target_machine;
    motion->requested_move_axis = 0;
    motion->move_axis = 0;
    motion->is_absolute = 1U;

    Motion_BeginPrepare(
        motion,
        speed,
        acc,
        0U,
        MOTION_COMPLETE_POSITION_STABLE);

    return 1U;
}

uint8_t Motion_IsBusy(
    const MotionController_t *motion)
{
    return ((motion->state == MOTION_PREPARE) ||
            (motion->state == MOTION_MOVING)) ? 1U : 0U;
}

MotionResultCode_t Motion_GetResult(
    const MotionController_t *motion)
{
    return motion->last_result;
}

int64_t Motion_GetMachinePosition(
    const MotionController_t *motion)
{
    /*
     * actual_pos is already raw_encoder - home_offset.
     * Current machine-coordinate convention is the opposite sign.
     */
    return -motion->actual_pos;
}

uint8_t Motion_PositionIsFresh(
    const MotionController_t *motion)
{
    if (motion->position_valid == 0U)
    {
        return 0U;
    }

    return ((HAL_GetTick() - motion->last_encoder_tick) <=
            MOTION_ENCODER_MAX_AGE_MS) ? 1U : 0U;
}

void Motion_SetPollingEnabled(
    MotionController_t *motion,
    uint8_t enabled)
{
    motion->polling_enabled = (enabled != 0U) ? 1U : 0U;
}


uint8_t Motion_SetZero(MotionController_t *motion)
{
    int64_t raw = 0;
    uint32_t seq = 0U;
    uint32_t tick = 0U;
    uint8_t valid = 0U;

    MKS_Read_Encoder_Safe(
        motion->motor,
        &raw,
        &seq,
        &tick,
        &valid);

    if (valid == 0U)
    {
        return 0U;
    }

    if ((HAL_GetTick() - tick) > MOTION_ZERO_MAX_AGE_MS)
    {
        return 0U;
    }

    motion->home_offset = raw;

    motion->actual_pos = 0;
    motion->start_pos = 0;
    motion->target_pos = 0;
    motion->last_pos = 0;
    motion->pos_error = 0;
    motion->pos_delta = 0;

    motion->last_encoder_seq = seq;
    motion->last_encoder_tick = tick;
    motion->position_valid = 1U;

    Motion_ClearStableTimer(motion);

    return 1U;
}

void Motion_Task(MotionController_t *motion)
{
    uint32_t now = HAL_GetTick();
    uint32_t poll_period;
    int64_t raw = 0;
    uint32_t seq = 0U;
    uint32_t encoder_tick = 0U;
    uint8_t valid = 0U;
    uint8_t new_encoder_sample = 0U;

    poll_period =
        (motion->state == MOTION_MOVING) ?
        MOTION_MOVING_POLL_PERIOD_MS :
        MOTION_IDLE_POLL_PERIOD_MS;

    /*
     * 0x31 is asynchronous:
     * request periodically, then consume the later ISR reply.
     * If all TX mailboxes are temporarily full, retry next ControlTask cycle.
     */
    if ((motion->polling_enabled != 0U) &&
        ((now - motion->last_poll_tick) >= poll_period))
    {
        HAL_StatusTypeDef poll_status =
            MKS_Read_Absolute_Position(motion->motor);

        if (poll_status == HAL_OK)
        {
            motion->last_poll_tick = now;
        }
    }

    MKS_Read_Encoder_Safe(
        motion->motor,
        &raw,
        &seq,
        &encoder_tick,
        &valid);

    if ((valid != 0U) &&
        (seq != motion->last_encoder_seq))
    {
        new_encoder_sample = 1U;

        motion->last_encoder_seq = seq;
        motion->last_encoder_tick = encoder_tick;
        motion->position_valid = 1U;

        motion->actual_pos =
            raw - motion->home_offset;

        motion->pos_delta =
            motion->actual_pos -
            motion->last_pos;

        motion->last_pos =
            motion->actual_pos;
    }

    switch (motion->state)
    {
        case MOTION_IDLE:
        case MOTION_DONE:
        case MOTION_ERROR:
            break;

        case MOTION_PREPARE:
        {
            HAL_StatusTypeDef tx_status;
            MKS_MoveFeedback_t feedback_pre;
            uint32_t encoder_seq_pre;
            int64_t distance;

            if ((now - motion->prepare_start_tick) >=
                MOTION_PREPARE_TIMEOUT_MS)
            {
                Motion_Fail(
                    motion,
                    MOTION_RESULT_TIMEOUT,
                    MOTION_ERR_PREPARE_TIMEOUT);
                break;
            }

            /*
             * Every TX attempt must be based on a genuinely fresh sample.
             * If HAL_BUSY delays the command, wait for the next 0x31 reply and
             * recompute the target/residual instead of sending from stale data.
             */
            if ((new_encoder_sample == 0U) ||
                (Motion_PositionIsFresh(motion) == 0U))
            {
                break;
            }

            motion->start_pos = motion->actual_pos;

            if (motion->is_absolute != 0U)
            {
                int64_t current_machine =
                    Motion_GetMachinePosition(motion);

                int64_t delta_machine =
                    motion->target_machine -
                    current_machine;

                int64_t effective_move =
                    -delta_machine;

                if ((effective_move < MOTION_F4_MIN_COUNTS) ||
                    (effective_move > MOTION_F4_MAX_COUNTS))
                {
                    Motion_Fail(
                        motion,
                        MOTION_RESULT_ERROR,
                        MOTION_ERR_RANGE);
                    break;
                }

                distance = Motion_Abs64(effective_move);

                if (distance <= MOTION_MIN_COMMAND_COUNTS)
                {
                    /*
                     * Fresh encoder says the absolute target is already within
                     * the defined correction floor. No F4 command is needed.
                     */
                    motion->move_axis = 0;
                    motion->target_pos = motion->actual_pos;
                    motion->pos_error = 0;
                    motion->last_result = MOTION_RESULT_OK;
                    motion->error_code = MOTION_ERR_NONE;
                    motion->state = MOTION_DONE;
                    break;
                }

                motion->move_axis =
                    (int32_t)effective_move;
            }
            else
            {
                motion->move_axis =
                    motion->requested_move_axis;
            }

            motion->target_pos =
                motion->start_pos +
                motion->move_axis;

            motion->pos_error =
                motion->target_pos -
                motion->actual_pos;

            motion->tolerance =
                Motion_CalcTolerance(motion->move_axis);

            motion->timeout_ms =
                Motion_CalcTimeoutMs(motion->move_axis);

            /*
             * Transaction boundaries are captured BEFORE TX.
             * This makes the observation window fail-safe: a very fast reply
             * cannot be accidentally included in the baseline.
             */
            MKS_Read_Move_Feedback_Safe(
                motion->motor,
                &feedback_pre);

            encoder_seq_pre =
                MKS_Get_Encoder_Seq(motion->motor);

            tx_status = MKS_Move_Relative_Axis(
                motion->motor,
                motion->speed,
                motion->acc,
                motion->move_axis);

            if (tx_status == HAL_BUSY)
            {
                /*
                 * Do not send using the same stale absolute residual later.
                 * Stay in PREPARE and wait for another fresh encoder sample.
                 */
                break;
            }

            if (tx_status != HAL_OK)
            {
                Motion_Fail(
                    motion,
                    MOTION_RESULT_ERROR,
                    MOTION_ERR_TX);
                break;
            }

            motion->move_start_tick = now;
            motion->move_encoder_seq_start = encoder_seq_pre;

            motion->ack_seq_start =
                feedback_pre.seq;
            motion->ack_start_count_start =
                feedback_pre.start_count;
            motion->ack_complete_count_start =
                feedback_pre.complete_count;
            motion->ack_fail_count_start =
                feedback_pre.fail_count;
            motion->ack_stopped_count_start =
                feedback_pre.stopped_count;

            motion->last_motion_tick = now;
            motion->start_seen = 0U;
            motion->complete_seen = 0U;
            Motion_ClearStableTimer(motion);

            motion->state = MOTION_MOVING;
            break;
        }

        case MOTION_MOVING:
        {
            MKS_MoveFeedback_t feedback;

            MKS_Read_Move_Feedback_Safe(
                motion->motor,
                &feedback);

            motion->pos_error =
                motion->target_pos -
                motion->actual_pos;

            /*
             * Sticky status counters mean a FAIL/STOP cannot be overwritten
             * by a later COMPLETE before ControlTask observes it.
             */
            if (feedback.fail_count !=
                motion->ack_fail_count_start)
            {
                Motion_Fail(
                    motion,
                    MOTION_RESULT_ERROR,
                    MOTION_ERR_DRIVER_FAIL);
                break;
            }

            if (feedback.stopped_count !=
                motion->ack_stopped_count_start)
            {
                if (motion->stop_is_expected != 0U)
                {
                    motion->last_result =
                        MOTION_RESULT_STOPPED;
                    motion->error_code =
                        MOTION_ERR_NONE;
                    motion->state =
                        MOTION_DONE;
                }
                else
                {
                    Motion_Fail(
                        motion,
                        MOTION_RESULT_ERROR,
                        MOTION_ERR_DRIVER_FAIL);
                }

                break;
            }

            if (feedback.start_count !=
                motion->ack_start_count_start)
            {
                motion->start_seen = 1U;
            }

            if (feedback.complete_count !=
                motion->ack_complete_count_start)
            {
                motion->complete_seen = 1U;

                /*
                 * Home chunks use the driver's COMPLETE from THIS F4
                 * transaction as their fast completion path.
                 *
                 * This is not the old single-byte move_ack design:
                 * FAIL/STOP/COMPLETE are sticky counters and the baseline for
                 * this command was captured before the F4 was queued.
                 *
                 * CAMERA_READY and future visual correction moves do NOT use
                 * this path; they continue to require encoder in-tolerance +
                 * stillness for 150 ms.
                 */
                if (motion->completion_policy ==
                    MOTION_COMPLETE_HOME_ACK)
                {
                    motion->last_result = MOTION_RESULT_OK;
                    motion->error_code = MOTION_ERR_NONE;
                    motion->state = MOTION_DONE;
                    Motion_ClearStableTimer(motion);
                    break;
                }
            }

            /*
             * Only strict positioning moves use encoder/stillness completion.
             * Home still receives fresh encoder data for SetZero/freshness and
             * for the existing fail-fast stall detector.
             */
            if ((motion->completion_policy ==
                 MOTION_COMPLETE_POSITION_STABLE) &&
                (new_encoder_sample != 0U) &&
                (Motion_SeqAfter(
                     motion->last_encoder_seq,
                     motion->move_encoder_seq_start) != 0U))
            {
                if (Motion_Abs64(motion->pos_delta) >=
                    MOTION_STALL_PROGRESS_COUNTS)
                {
                    motion->last_motion_tick = now;
                }

                if ((Motion_Abs64(motion->pos_error) <=
                     motion->tolerance) &&
                    (Motion_Abs64(motion->pos_delta) <=
                     motion->still_threshold))
                {
                    if (motion->stable_active == 0U)
                    {
                        motion->stable_active = 1U;
                        motion->stable_start_tick = now;
                    }
                    else if ((now - motion->stable_start_tick) >=
                             MOTION_STABLE_CONFIRM_MS)
                    {
                        motion->last_result =
                            MOTION_RESULT_OK;
                        motion->error_code =
                            MOTION_ERR_NONE;
                        motion->state =
                            MOTION_DONE;
                        break;
                    }
                }
                else
                {
                    Motion_ClearStableTimer(motion);
                }
            }

            /*
             * Primary early-failure detector:
             * command had time to start, target is still far away, and the
             * encoder has shown no count-level progress for 600 ms.
             */
            if (((now - motion->move_start_tick) >=
                 MOTION_STALL_GRACE_MS) &&
                (Motion_Abs64(motion->pos_error) >
                 motion->tolerance) &&
                ((now - motion->last_motion_tick) >=
                 MOTION_STALL_MS))
            {
                Motion_Fail(
                    motion,
                    MOTION_RESULT_ERROR,
                    MOTION_ERR_STALL);
                break;
            }

            if ((now - motion->move_start_tick) >=
                motion->timeout_ms)
            {
                Motion_Fail(
                    motion,
                    MOTION_RESULT_TIMEOUT,
                    MOTION_ERR_MOVE_TIMEOUT);
                break;
            }

            break;
        }

        default:
            Motion_Fail(
                motion,
                MOTION_RESULT_ERROR,
                MOTION_ERR_INTERNAL);
            break;
    }
}
