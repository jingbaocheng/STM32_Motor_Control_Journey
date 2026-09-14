#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include <stdint.h>
#include "mks_motor.h"

typedef enum
{
    MOTION_IDLE    = 0U,
    MOTION_PREPARE = 1U,
    MOTION_MOVING  = 2U,
    MOTION_DONE    = 3U,
    MOTION_ERROR   = 4U

} MotionState_t;

typedef enum
{
    MOTION_RESULT_NONE    = 0U,
    MOTION_RESULT_OK      = 1U,
    MOTION_RESULT_TIMEOUT = 2U,
    MOTION_RESULT_ERROR   = 3U,

    /*
     * Expected early stop.
     * Only Home search stages are allowed to interpret this as success, and
     * only after they independently confirm the Home sensor is active.
     */
    MOTION_RESULT_STOPPED = 4U

} MotionResultCode_t;

typedef enum
{
    /*
     * Normal positioning uses encoder + stillness completion.
     * Home chunks use current-transaction F4 completion feedback so every
     * 512/64-count search probe does not wait another 150 ms after the driver
     * has already reported completion.
     */
    MOTION_COMPLETE_POSITION_STABLE = 0U,
    MOTION_COMPLETE_HOME_ACK        = 1U

} MotionCompletionPolicy_t;

typedef enum
{
    MOTION_ERR_NONE            = 0U,
    MOTION_ERR_PREPARE_TIMEOUT = 1U,
    MOTION_ERR_TX              = 2U,
    MOTION_ERR_DRIVER_FAIL     = 3U,
    MOTION_ERR_MOVE_TIMEOUT    = 4U,
    MOTION_ERR_INTERNAL        = 5U,
    MOTION_ERR_STALL           = 6U,
    MOTION_ERR_RANGE           = 7U,
    MOTION_ERR_POSITION_STALE  = 8U

} MotionErrorCode_t;

typedef struct
{
    MKS_Motor_t *motor;

    MotionState_t state;
    MotionResultCode_t last_result;
    MotionErrorCode_t error_code;

    int64_t start_pos;
    int64_t target_pos;
    int64_t actual_pos;
    int64_t home_offset;

    int64_t last_pos;
    int64_t pos_error;
    int64_t pos_delta;

    /* Requested relative move, and effective move actually sent to F4. */
    int32_t requested_move_axis;
    int32_t move_axis;

    /* Machine-coordinate absolute target when is_absolute != 0. */
    int64_t target_machine;
    uint8_t is_absolute;

    uint16_t speed;
    uint8_t acc;

    int64_t tolerance;
    int64_t still_threshold;

    uint32_t last_poll_tick;
    uint32_t last_encoder_seq;
    uint32_t last_encoder_tick;
    uint8_t position_valid;
    uint8_t polling_enabled;

    /* One motion transaction. */
    uint32_t prepare_start_tick;
    uint32_t move_start_tick;
    uint32_t timeout_ms;
    uint32_t move_encoder_seq_start;

    uint32_t ack_seq_start;
    uint32_t ack_start_count_start;
    uint32_t ack_complete_count_start;
    uint32_t ack_fail_count_start;
    uint32_t ack_stopped_count_start;

    uint32_t stable_start_tick;
    uint8_t stable_active;

    uint32_t last_motion_tick;
    uint8_t start_seen;
    uint8_t complete_seen;

    MotionCompletionPolicy_t completion_policy;

    /*
     * Per-transaction policy. Motion_BeginPrepare() overwrites this on every
     * command, so the Home "early stop is expected" policy cannot leak into
     * a later CAMERA_READY move.
     */
    uint8_t stop_is_expected;

} MotionController_t;

/* Reserved for later PC -> ControlTask queue commands. */
typedef enum
{
    MOTION_CMD_ENABLE   = 0U,
    MOTION_CMD_MOVE_REL = 1U

} MotionCommandType_t;

typedef enum
{
    MOTION_AXIS_X = 0U,
    MOTION_AXIS_Y = 1U

} MotionAxis_t;

typedef struct
{
    uint32_t command_id;
    uint8_t type;
    uint8_t axis;
    uint8_t enable;
    uint8_t acceleration;
    uint16_t speed;
    int32_t displacement;
} MotionCommand_t;

typedef struct
{
    uint32_t command_id;
    uint32_t result;
} MotionResult_t;

/*
 * Position / completion policy.
 *
 * The tolerance is adaptive for small moves but never allowed to equal or
 * exceed a non-zero commanded displacement. This prevents a short correction
 * from being declared DONE before the axis moves.
 */
#define MOTION_DEFAULT_TOLERANCE          32LL
#define MOTION_MIN_TOLERANCE               4LL
#define MOTION_MIN_COMMAND_COUNTS          4LL

/*
 * With the frozen 50 ms encoder poll period, 10 counts/sample corresponds to
 * about 200 counts/s. Keep these two values coupled until real static/velocity
 * data are measured; changing only one changes the physical "still" meaning.
 */
#define MOTION_DEFAULT_STILL_THRESH       10LL

#define MOTION_IDLE_POLL_PERIOD_MS        50U
#define MOTION_MOVING_POLL_PERIOD_MS      50U

#define MOTION_PREPARE_TIMEOUT_MS       1500U
#define MOTION_STABLE_CONFIRM_MS         150U

#define MOTION_ENCODER_MAX_AGE_MS        200U
#define MOTION_ZERO_MAX_AGE_MS           100U

#define MOTION_STALL_GRACE_MS            300U
#define MOTION_STALL_MS                  600U
#define MOTION_STALL_PROGRESS_COUNTS       1LL

/*
 * Conservative last-resort timeout. Stall detection is the primary early
 * failure detector; this remains as a hard upper bound.
 */
#define MOTION_TIMEOUT_BASE_MS          5000U
#define MOTION_TIMEOUT_COUNTS_DIV          8U
#define MOTION_TIMEOUT_MAX_MS          30000U

void Motion_Init(
    MotionController_t *motion,
    MKS_Motor_t *motor);

uint8_t Motion_StartRelative(
    MotionController_t *motion,
    int32_t move_axis,
    uint16_t speed,
    uint8_t acc);

uint8_t Motion_StartRelativeEx(
    MotionController_t *motion,
    int32_t move_axis,
    uint16_t speed,
    uint8_t acc,
    uint8_t stop_is_expected);

/*
 * Home-only relative move.
 *
 * FAST/SLOW pass stop_is_expected=1.
 * BACK/RELEASE pass stop_is_expected=0.
 *
 * This keeps Home sensor-driven and responsive while CAMERA_READY continues
 * to use the strict encoder/stillness completion policy.
 */
uint8_t Motion_StartHomeRelative(
    MotionController_t *motion,
    int32_t move_axis,
    uint16_t speed,
    uint8_t acc,
    uint8_t stop_is_expected);

uint8_t Motion_StartAbsoluteMachine(
    MotionController_t *motion,
    int64_t target_machine,
    uint16_t speed,
    uint8_t acc);

void Motion_Task(MotionController_t *motion);

void Motion_SetPollingEnabled(
    MotionController_t *motion,
    uint8_t enabled);

uint8_t Motion_SetZero(MotionController_t *motion);

uint8_t Motion_PositionIsFresh(
    const MotionController_t *motion);

uint8_t Motion_IsBusy(
    const MotionController_t *motion);

MotionResultCode_t Motion_GetResult(
    const MotionController_t *motion);

int64_t Motion_GetMachinePosition(
    const MotionController_t *motion);

#endif
