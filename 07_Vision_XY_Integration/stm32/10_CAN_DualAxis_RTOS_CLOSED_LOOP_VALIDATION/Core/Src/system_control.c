#include "system_control.h"
#include "vision_config.h"
#include "main.h"
#include "stm32f4xx_hal.h"

/*
 * Vision MVP control policy
 * -------------------------
 * One RTOS task owns Motion_X, Motion_Y and System_Task in deterministic order.
 * This file owns the one-shot laboratory start/Home/camera-ready sequence.
 * OpenCV/PC integration will attach AFTER SYS_CAMERA_READY through a command
 * interface; it must not directly manipulate MotionController_t state.
 */

/* KEY0: released=PE4 high, pressed=PE4 low. Return 1 while pressed. */
static uint8_t Read_Key0(void)
{
    return (HAL_GPIO_ReadPin(
                KEY0_GPIO_Port,
                KEY0_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* Existing Home sensor convention confirmed by the current project. */
static uint8_t X_HomeTriggered(void)
{
    return (HAL_GPIO_ReadPin(
                X_HOME_GPIO_Port,
                X_HOME_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t Y_HomeTriggered(void)
{
    return (HAL_GPIO_ReadPin(
                Y_HOME_GPIO_Port,
                Y_HOME_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/* Confirmed real-machine Home directions. */
#define X_HOME_DIR_SIGN   (+1)
#define Y_HOME_DIR_SIGN   (+1)

/* Home motion parameters: intentionally frozen during Vision MVP bring-up. */
#define HOME_FAST_STEP            512
#define HOME_FAST_SPEED           350U
#define HOME_FAST_ACCEL           1U

#define HOME_BACK_STEP            512
#define HOME_BACK_SPEED           150U
#define HOME_BACK_ACCEL           1U

#define HOME_SLOW_STEP            64
#define HOME_SLOW_SPEED           80U
#define HOME_SLOW_ACCEL           1U

/*
 * "Retries" are additional attempts after the first BACK move.
 * Therefore the present maximum release travel is:
 *   (1 initial + 6 retries) * 512 = 3584 counts.
 */
#define HOME_BACK_MAX_RETRIES     6U
#define HOME_AXIS_TIMEOUT_MS      180000U
#define HOME_INTER_MOVE_MS         50U
#define HOME_STABLE_CONFIRM_MS    500U

/*
 * Final closeout filter for PG6/PG7.
 * A Home level is accepted only after remaining unchanged for 30 ms.
 */
#define HOME_SENSOR_DEBOUNCE_MS      30U

#define HOME_RELEASE_STEP         2048
#define HOME_RELEASE_SPEED        150U
#define HOME_RELEASE_ACCEL        1U

/*
 * RELEASE is also an "exit the Home sensor" phase. Give it bounded retries
 * instead of waiting motionless for HOME_AXIS_TIMEOUT_MS if 2048 counts was
 * not enough. One retry gives 2 * 2048 = 4096 counts, already greater
 * than the complete 3584-count BACK span while avoiding an unnecessary
 * third release move.
 */
#define HOME_RELEASE_MAX_RETRIES     1U

/* Enable frames are retried on HAL_BUSY, but startup may not hang forever. */
#define CONTROL_ENABLE_TX_TIMEOUT_MS  1500U

static AxisHomeCtx_t home_x;
static AxisHomeCtx_t home_y;

volatile SystemState_t system_state = SYS_STARTUP;
static uint32_t system_state_tick = 0U;

/*
 * Build marker for Keil Watch.
 * 0x26091453 = V5.5 fresh one-shot closed-loop APPLY build.
 *
 * Normal APPLY path intentionally DOES NOT Home. Correction values come from
 * the CURRENT recapture taken after the full power cycle.
 */
volatile uint32_t g_build_id = 0x26091454U;
volatile uint8_t g_key0_level = 0U;
volatile uint32_t g_key0_hold_ms = 0U;
volatile uint32_t g_key0_start_count = 0U;
volatile uint8_t g_camera_ready = 0U;
volatile uint8_t g_debug_home_request = 0U;
volatile uint8_t g_camera_move_request = 0U;
/* Debug/vision request: take another picture without moving X/Y. */
volatile uint8_t g_camera_trigger_request = 0U;

volatile SystemState_t g_system_error_from_state = SYS_STARTUP;
volatile HomeStage_t g_home_x_stage = HOME_STAGE_FAST_START;
volatile HomeStage_t g_home_y_stage = HOME_STAGE_FAST_START;

volatile uint8_t g_camera_ready_y_enable = CAMERA_READY_Y_ENABLE_DEFAULT;
volatile int64_t g_camera_ready_x_target = CAMERA_READY_X_DEFAULT;
volatile int64_t g_camera_ready_y_target = CAMERA_READY_Y_DEFAULT;
volatile int64_t g_camera_ready_x_actual = 0;
volatile int64_t g_camera_ready_y_actual = 0;

/*
 * Closed-loop BEFORE V2 state.
 *
 * Point numbering follows the camera captures:
 *   0 = READY_REF at the frozen normal CameraReady position
 *   1 = BEFORE at LOOP_BEFORE_TEST_X / LOOP_BEFORE_TEST_Y
 */
volatile uint8_t g_cal_point_index = 0U;
volatile uint8_t g_cal_sequence_done = 0U;

volatile int32_t g_loop_apply_dx = LOOP_APPLY_DX_COUNTS;
volatile int32_t g_loop_apply_dy = LOOP_APPLY_DY_COUNTS;
volatile uint8_t g_loop_apply_done = 0U;

/*
 * Prevent one long KEY0 hold from advancing multiple calibration points.
 * The key must be physically released after Home / after each advance.
 */
static uint8_t cal_key_release_required = 0U;

/* KEY0 level/hold tracking used by Home start and CAL point advance. */
static uint8_t key0_hold_active = 0U;
static uint32_t key0_hold_start_tick = 0U;

/*
 * One-shot start authorization.
 *
 * We deliberately do NOT use a press-edge event here.  The previous project
 * spent too much time on a laboratory button that is not part of the future
 * OpenCV motion path.  A stable physical press for KEY0_START_HOLD_MS is enough
 * to authorize the single Home sequence for this boot.
 */
static uint8_t Key0_HoldStartRequested(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t pressed = Read_Key0();

    g_key0_level = pressed;

    if (pressed == 0U)
    {
        key0_hold_active = 0U;
        key0_hold_start_tick = 0U;
        g_key0_hold_ms = 0U;
        return 0U;
    }

    if (key0_hold_active == 0U)
    {
        key0_hold_active = 1U;
        key0_hold_start_tick = now;
        g_key0_hold_ms = 0U;
        return 0U;
    }

    g_key0_hold_ms = now - key0_hold_start_tick;

    if (g_key0_hold_ms >= KEY0_START_HOLD_MS)
    {
        g_key0_start_count++;
        return 1U;
    }

    return 0U;
}

static void Key0_ResetHold(void)
{
    key0_hold_active = 0U;
    key0_hold_start_tick = 0U;
    g_key0_hold_ms = 0U;
    g_key0_level = Read_Key0();
}


/*
 * Calibration advance button.
 *
 * The Home-start press is not allowed to "carry through" and start P1.
 * Likewise, a press that starts P1 must be released before it can start P2.
 */
static uint8_t Cal_KeyAdvanceRequested(void)
{
    if (CAL_3POINT_ENABLE == 0U)
    {
        return 0U;
    }

    if (cal_key_release_required != 0U)
    {
        if (Read_Key0() == 0U)
        {
            cal_key_release_required = 0U;
            Key0_ResetHold();
        }

        return 0U;
    }

    if (Key0_HoldStartRequested() != 0U)
    {
        cal_key_release_required = 1U;
        Key0_ResetHold();
        return 1U;
    }

    return 0U;
}


/*
 * Prepare the NEXT calibration point by updating the already-existing
 * camera-ready absolute targets. The normal CameraReady motion path then does:
 *
 * target update -> absolute closed-loop move -> position verification
 * -> 500 ms settle -> PB0 hardware trigger.
 *
 * No new motor protocol or second motion owner is introduced.
 */
static uint8_t Cal_StartNextPoint(void)
{
    if (CAL_3POINT_ENABLE == 0U)
    {
        return 0U;
    }

    if (g_cal_point_index != 0U)
    {
        return 0U;
    }

    /*
     * The first capture remains the frozen normal CameraReady capture.
     * Only the second physical KEY0 press requests the independent BEFORE
     * point.  This deliberately avoids changing the proven post-Home target.
     */
    g_camera_ready_x_target = LOOP_BEFORE_TEST_X;
    g_camera_ready_y_target = LOOP_BEFORE_TEST_Y;
    g_cal_point_index = 1U;

    g_camera_ready_y_enable = 1U;
    g_camera_ready = 0U;
    g_camera_move_request = 1U;

    return 1U;
}

static void System_EnterError(void)
{
    /*
     * SYS_ERROR is a terminal SOFTWARE state, not an emergency stop.
     *
     * - PB0 is forced back to camera-idle HIGH.
     * - Future 0x31 polling is disabled so a dead CAN link cannot immediately
     *   refill all TX mailboxes after we abort them.
     * - HAL_CAN_AbortTxRequest() is best-effort only. A frame that has already
     *   started/succeeded on the bus may still have reached the driver.
     * - A motor command already accepted by the external driver may continue
     *   to completion because the verified Stop command is still unknown.
     */
    HAL_GPIO_WritePin(
        CAMERA_TRIG_GPIO_Port,
        CAMERA_TRIG_Pin,
        GPIO_PIN_SET);

    Motion_SetPollingEnabled(home_x.motion, 0U);
    Motion_SetPollingEnabled(home_y.motion, 0U);

    (void)HAL_CAN_AbortTxRequest(
        &hcan1,
        CAN_TX_MAILBOX0 |
        CAN_TX_MAILBOX1 |
        CAN_TX_MAILBOX2);

    g_system_error_from_state = system_state;
    g_camera_ready = 0U;
    system_state = SYS_ERROR;
}

static uint8_t Camera_TargetWithin(
    MotionController_t *motion,
    int64_t target_machine,
    int64_t tolerance)
{
    int64_t error;

    /*
     * A numerically correct but stale encoder value is NOT a valid interlock.
     * Camera trigger is allowed only while position feedback is current.
     */
    if (Motion_PositionIsFresh(motion) == 0U)
    {
        return 0U;
    }

    error =
        target_machine - Motion_GetMachinePosition(motion);

    if (error < 0)
    {
        error = -error;
    }

    return (error <= tolerance) ? 1U : 0U;
}

static uint8_t Camera_TargetAlreadyReached(
    MotionController_t *motion,
    int64_t target_machine)
{
    return Camera_TargetWithin(
        motion,
        target_machine,
        CAMERA_READY_SKIP_TOLERANCE);
}

static uint8_t Camera_AllTargetsVerified(void)
{
    if (!Camera_TargetWithin(
            home_x.motion,
            g_camera_ready_x_target,
            CAMERA_READY_VERIFY_TOLERANCE))
    {
        return 0U;
    }

    if ((g_camera_ready_y_enable != 0U) &&
        !Camera_TargetWithin(
            home_y.motion,
            g_camera_ready_y_target,
            CAMERA_READY_VERIFY_TOLERANCE))
    {
        return 0U;
    }

    return 1U;
}

static void System_StartCameraSettle(void)
{
    g_camera_ready = 0U;

    /* Never let a bad mechanical position reach the camera trigger. */
    if (Camera_AllTargetsVerified() == 0U)
    {
        System_EnterError();
        return;
    }

    system_state_tick = HAL_GetTick();
    system_state = SYS_CAMERA_SETTLE;
}


static void AxisHome_Init(
    AxisHomeCtx_t *ctx,
    MotionController_t *motion,
    HomeSensorFn_t sensor,
    int32_t dir_sign)
{
    ctx->motion = motion;
    ctx->sensor_triggered = sensor;
    ctx->dir_sign = dir_sign;

    ctx->stage = HOME_STAGE_FAST_START;
    ctx->home_start_tick = HAL_GetTick();
    ctx->next_move_tick = 0U;
    ctx->stable_start_tick = 0U;
    ctx->back_retry_count = 0U;
    ctx->release_retry_count = 0U;

    /*
     * Do not trust the first GPIO sample. The same raw level must remain
     * stable for HOME_SENSOR_DEBOUNCE_MS before the first Home motion starts.
     */
    ctx->sensor_raw_last = ctx->sensor_triggered();
    ctx->sensor_debounced = 0U;
    ctx->sensor_ready = 0U;
    ctx->sensor_change_tick = HAL_GetTick();
}


static void AxisHome_UpdateSensor(AxisHomeCtx_t *ctx)
{
    uint32_t now = HAL_GetTick();
    uint8_t raw = ctx->sensor_triggered();

    if (raw != ctx->sensor_raw_last)
    {
        ctx->sensor_raw_last = raw;
        ctx->sensor_change_tick = now;
        return;
    }

    if ((now - ctx->sensor_change_tick) < HOME_SENSOR_DEBOUNCE_MS)
    {
        return;
    }

    ctx->sensor_debounced = raw;
    ctx->sensor_ready = 1U;
}


static uint8_t AxisHome_SensorActive(
    const AxisHomeCtx_t *ctx)
{
    return ((ctx->sensor_ready != 0U) &&
            (ctx->sensor_debounced != 0U)) ? 1U : 0U;
}


/*
 * Start a fresh complete Home sequence.
 *
 * Vision MVP uses one deliberate KEY0 authorization and one complete Home
 * sequence per boot.  Repeated Home from CAMERA_READY is intentionally removed
 * from the main path so camera bring-up cannot accidentally retrigger homing.
 */
static uint8_t System_StartHome(void)
{
    if (Motion_IsBusy(home_x.motion) ||
        Motion_IsBusy(home_y.motion))
    {
        return 0U;
    }

    AxisHome_Init(
        &home_x,
        home_x.motion,
        X_HomeTriggered,
        X_HOME_DIR_SIGN);

    AxisHome_Init(
        &home_y,
        home_y.motion,
        Y_HomeTriggered,
        Y_HOME_DIR_SIGN);

    /*
     * The same KEY0 hold that authorized Home must be released before the
     * first calibration advance can be accepted after P0.
     */
    cal_key_release_required = 1U;

    system_state = SYS_HOME_X;

    return 1U;
}


static uint8_t AxisHome_Task(
    AxisHomeCtx_t *ctx)
{
    MotionResultCode_t result;

    /*
     * Sample raw Home GPIO every ControlTask cycle, including while a motor
     * chunk is moving. State transitions below see only the debounced level.
     */
    AxisHome_UpdateSensor(ctx);

    if (ctx->sensor_ready == 0U)
    {
        return 0U;
    }

    if ((HAL_GetTick() - ctx->home_start_tick) >=
        HOME_AXIS_TIMEOUT_MS)
    {
        ctx->stage = HOME_STAGE_ERROR;
    }

    switch (ctx->stage)
    {
        case HOME_STAGE_FAST_START:

            /*
             * If already on Home at the beginning of this Home sequence,
             * release the switch first.
             *
             * This is also important for repeated Home.
             */
            if (AxisHome_SensorActive(ctx))
            {
                ctx->back_retry_count = 0U;
                ctx->stage = HOME_STAGE_BACK_START;
                break;
            }

            if ((int32_t)(HAL_GetTick() -
                          ctx->next_move_tick) < 0)
            {
                break;
            }

            if (Motion_StartHomeRelative(
                    ctx->motion,
                    ctx->dir_sign * HOME_FAST_STEP,
                    HOME_FAST_SPEED,
                    HOME_FAST_ACCEL,
                    1U))
            {
                ctx->stage = HOME_STAGE_FAST_WAIT;
            }

            break;


        case HOME_STAGE_FAST_WAIT:

            if (Motion_IsBusy(ctx->motion))
            {
                break;
            }

            /*
             * During FAST/SLOW search the Home sensor is the goal.
             * If the axis stopped early, stalled, timed out, or returned
             * STOPPED exactly on the sensor, reaching the sensor still means
             * this search leg succeeded. Only when the sensor is NOT active do
             * we interpret a non-OK Motion result as a real failure.
             *
             * BACK/RELEASE deliberately do NOT use this rule because those
             * phases are trying to leave the sensor.
             */
            if (AxisHome_SensorActive(ctx))
            {
                ctx->back_retry_count = 0U;
                ctx->next_move_tick =
                    HAL_GetTick() + HOME_INTER_MOVE_MS;
                ctx->stage = HOME_STAGE_BACK_START;
                break;
            }

            result = Motion_GetResult(ctx->motion);

            if (result != MOTION_RESULT_OK)
            {
                ctx->stage = HOME_STAGE_ERROR;
                break;
            }

            ctx->next_move_tick =
                HAL_GetTick() + HOME_INTER_MOVE_MS;
            ctx->stage = HOME_STAGE_FAST_START;
            break;


        case HOME_STAGE_BACK_START:

            if ((int32_t)(HAL_GetTick() -
                          ctx->next_move_tick) < 0)
            {
                break;
            }

            if (Motion_StartHomeRelative(
                    ctx->motion,
                    -ctx->dir_sign * HOME_BACK_STEP,
                    HOME_BACK_SPEED,
                    HOME_BACK_ACCEL,
                    0U))
            {
                ctx->stage = HOME_STAGE_BACK_WAIT;
            }

            break;


        case HOME_STAGE_BACK_WAIT:

            if (Motion_IsBusy(ctx->motion))
            {
                break;
            }

            result = Motion_GetResult(ctx->motion);

            if (result != MOTION_RESULT_OK)
            {
                ctx->stage = HOME_STAGE_ERROR;
                break;
            }

            ctx->next_move_tick =
                HAL_GetTick() + HOME_INTER_MOVE_MS;

            if (AxisHome_SensorActive(ctx))
            {
                ctx->back_retry_count++;

                if (ctx->back_retry_count >
                    HOME_BACK_MAX_RETRIES)
                {
                    ctx->stage = HOME_STAGE_ERROR;
                }
                else
                {
                    ctx->stage = HOME_STAGE_BACK_START;
                }
            }
            else
            {
                ctx->back_retry_count = 0U;
                ctx->stage = HOME_STAGE_SLOW_START;
            }

            break;


        case HOME_STAGE_SLOW_START:

            /*
             * If the sensor became active again before the slow approach
             * starts, release it again.
             */
            if (AxisHome_SensorActive(ctx))
            {
                ctx->stage = HOME_STAGE_BACK_START;
                break;
            }

            if ((int32_t)(HAL_GetTick() -
                          ctx->next_move_tick) < 0)
            {
                break;
            }

            if (Motion_StartHomeRelative(
                    ctx->motion,
                    ctx->dir_sign * HOME_SLOW_STEP,
                    HOME_SLOW_SPEED,
                    HOME_SLOW_ACCEL,
                    1U))
            {
                ctx->stage = HOME_STAGE_SLOW_WAIT;
            }

            break;


        case HOME_STAGE_SLOW_WAIT:

            if (Motion_IsBusy(ctx->motion))
            {
                break;
            }

            /*
             * Sensor-first semantics are intentional only for the two search
             * legs. Reaching Home is the successful outcome, even if the
             * Motion layer classified the way the motor stopped as STALL,
             * TIMEOUT or STOPPED.
             */
            if (AxisHome_SensorActive(ctx))
            {
                ctx->stable_start_tick = HAL_GetTick();
                ctx->stage = HOME_STAGE_STABLE_CONFIRM;
                break;
            }

            result = Motion_GetResult(ctx->motion);

            if (result != MOTION_RESULT_OK)
            {
                ctx->stage = HOME_STAGE_ERROR;
                break;
            }

            ctx->next_move_tick =
                HAL_GetTick() + HOME_INTER_MOVE_MS;
            ctx->stage = HOME_STAGE_SLOW_START;
            break;


        case HOME_STAGE_STABLE_CONFIRM:

            if (AxisHome_SensorActive(ctx))
            {
                if ((HAL_GetTick() -
                     ctx->stable_start_tick) >=
                    HOME_STABLE_CONFIRM_MS)
                {
                    /*
                     * Mechanical Home reference is confirmed here.
                     * Repeated Home will establish a new home_offset here.
                     */
                    if (Motion_SetZero(ctx->motion) == 0U)
                    {
                        /*
                         * Never continue Home with an invalid or stale zero.
                         * A bad home_offset would shift the entire machine
                         * coordinate system while still looking self-consistent.
                         */
                        ctx->stage = HOME_STAGE_ERROR;
                        break;
                    }

                    ctx->release_retry_count = 0U;

                    ctx->next_move_tick =
                        HAL_GetTick() + HOME_INTER_MOVE_MS;

                    ctx->stage =
                        HOME_STAGE_RELEASE_START;
                }
            }
            else
            {
                /*
                 * Home signal disappeared before the confirmation time.
                 * Treat it as unstable and approach slowly again.
                 */
                ctx->stable_start_tick = 0U;

                ctx->next_move_tick =
                    HAL_GetTick() + HOME_INTER_MOVE_MS;

                ctx->stage = HOME_STAGE_SLOW_START;
            }

            break;


        case HOME_STAGE_RELEASE_START:

            if ((int32_t)(HAL_GetTick() -
                          ctx->next_move_tick) < 0)
            {
                break;
            }

            /*
             * Home direction is +dir_sign.
             * Release direction is therefore -dir_sign.
             */
            if (Motion_StartHomeRelative(
                    ctx->motion,
                    -ctx->dir_sign * HOME_RELEASE_STEP,
                    HOME_RELEASE_SPEED,
                    HOME_RELEASE_ACCEL,
                    0U))
            {
                ctx->stage =
                    HOME_STAGE_RELEASE_WAIT;
            }

            break;


        case HOME_STAGE_RELEASE_WAIT:

            if (Motion_IsBusy(ctx->motion))
            {
                break;
            }

            result = Motion_GetResult(ctx->motion);

            if (result != MOTION_RESULT_OK)
            {
                ctx->stage = HOME_STAGE_ERROR;
                break;
            }

            /*
             * The release move has finished.
             * Now confirm that the Home sensor remains released.
             */
            ctx->stable_start_tick = HAL_GetTick();

            ctx->stage =
                HOME_STAGE_RELEASE_CONFIRM;

            break;


        case HOME_STAGE_RELEASE_CONFIRM:

            if (!AxisHome_SensorActive(ctx))
            {
                if (ctx->stable_start_tick == 0U)
                {
                    ctx->stable_start_tick =
                        HAL_GetTick();
                }

                if ((HAL_GetTick() -
                     ctx->stable_start_tick) >=
                    HOME_STABLE_CONFIRM_MS)
                {
                    ctx->stage =
                        HOME_STAGE_DONE;
                }
            }
            else
            {
                /*
                 * 2048 counts was not enough to leave the active window.
                 * Retry RELEASE instead of waiting motionless for 180 s.
                 */
                ctx->stable_start_tick = 0U;
                ctx->release_retry_count++;

                if (ctx->release_retry_count >
                    HOME_RELEASE_MAX_RETRIES)
                {
                    ctx->stage = HOME_STAGE_ERROR;
                }
                else
                {
                    ctx->next_move_tick =
                        HAL_GetTick() + HOME_INTER_MOVE_MS;
                    ctx->stage = HOME_STAGE_RELEASE_START;
                }
            }

            break;


        case HOME_STAGE_DONE:
        case HOME_STAGE_ERROR:
        default:
            break;
    }

    return ((ctx->stage == HOME_STAGE_DONE) ||
            (ctx->stage == HOME_STAGE_ERROR)) ? 1U : 0U;
}


void System_Init(
    MotionController_t *motion_x,
    MotionController_t *motion_y)
{
    AxisHome_Init(
        &home_x,
        motion_x,
        X_HomeTriggered,
        X_HOME_DIR_SIGN);

    AxisHome_Init(
        &home_y,
        motion_y,
        Y_HomeTriggered,
        Y_HOME_DIR_SIGN);

    Motion_SetPollingEnabled(home_x.motion, 1U);
    Motion_SetPollingEnabled(home_y.motion, 1U);

    Key0_ResetHold();

    /*
     * Camera trigger idle state: HIGH.
     * Previous standalone hardware test confirmed:
     * PB0 LOW -> optocoupler O1 rises -> camera trigger.
     */
    HAL_GPIO_WritePin(
        CAMERA_TRIG_GPIO_Port,
        CAMERA_TRIG_Pin,
        GPIO_PIN_SET);

    g_key0_start_count = 0U;
    g_camera_ready = 0U;
    g_debug_home_request = 0U;
    g_camera_trigger_request = 0U;
    g_camera_move_request = 0U;
    g_system_error_from_state = SYS_STARTUP;
    g_home_x_stage = home_x.stage;
    g_home_y_stage = home_y.stage;
    g_camera_ready_y_enable = CAMERA_READY_Y_ENABLE_DEFAULT;
    g_camera_ready_x_target = CAMERA_READY_X_DEFAULT;
    g_camera_ready_y_target = CAMERA_READY_Y_DEFAULT;
    g_camera_ready_x_actual = 0;
    g_camera_ready_y_actual = 0;

    g_cal_point_index = 0U;
    g_cal_sequence_done = 0U;
    cal_key_release_required = 0U;

    g_loop_apply_dx = LOOP_APPLY_DX_COUNTS;
    g_loop_apply_dy = LOOP_APPLY_DY_COUNTS;
    g_loop_apply_done = 0U;

    system_state = SYS_STARTUP;
    system_state_tick = HAL_GetTick();
     
}


SystemState_t System_GetState(void)
{
    return system_state;
}


uint8_t System_IsCameraReady(void)
{
    return g_camera_ready;
}


void System_Task(void)
{
    /* Always expose current machine positions to Watch / future comm layer. */
    g_camera_ready_x_actual =
        Motion_GetMachinePosition(home_x.motion);

    g_camera_ready_y_actual =
        Motion_GetMachinePosition(home_y.motion);

    g_home_x_stage = home_x.stage;
    g_home_y_stage = home_y.stage;

    switch (system_state)
    {
        case SYS_STARTUP:
        {
            HAL_StatusTypeDef x_status = HAL_OK;
            HAL_StatusTypeDef y_status = HAL_OK;

            g_camera_ready = 0U;

            if (home_x.motion->motor->is_enabled == 0U)
            {
                x_status = MKS_Set_Enable_State(
                    home_x.motion->motor,
                    1U);
            }

            if (home_y.motion->motor->is_enabled == 0U)
            {
                y_status = MKS_Set_Enable_State(
                    home_y.motion->motor,
                    1U);
            }

            if (((x_status != HAL_OK) &&
                 (x_status != HAL_BUSY)) ||
                ((y_status != HAL_OK) &&
                 (y_status != HAL_BUSY)))
            {
                System_EnterError();
                break;
            }

            if ((home_x.motion->motor->is_enabled != 0U) &&
                (home_y.motion->motor->is_enabled != 0U))
            {
                system_state_tick = HAL_GetTick();
                Key0_ResetHold();
                system_state = SYS_WAIT_ENABLE;
                break;
            }

            /*
             * HAL_BUSY is retryable, but startup is never allowed to wait
             * forever on a saturated CAN TX path.
             */
            if ((HAL_GetTick() - system_state_tick) >=
                CONTROL_ENABLE_TX_TIMEOUT_MS)
            {
                System_EnterError();
            }

            break;
        }


        case SYS_WAIT_ENABLE:

            /*
             * The enable frames being queued is not proof that either driver
             * is alive. During the 3 s settle window Motion_Task polls 0x31.
             * At the end, require fresh replies from BOTH axes before allowing
             * Home. A disconnected/unpowered CAN node therefore becomes an
             * explicit SYS_ERROR instead of a later mysterious Home failure.
             */
            if ((HAL_GetTick() - system_state_tick) >=
                CONTROL_ENABLE_SETTLE_MS)
            {
                if ((Motion_PositionIsFresh(home_x.motion) == 0U) ||
                    (Motion_PositionIsFresh(home_y.motion) == 0U))
                {
                    System_EnterError();
                }
                else
                {
                    Key0_ResetHold();
                    system_state = SYS_WAIT_HOME_KEY;
                }
            }
            break;


        case SYS_WAIT_HOME_KEY:

            /*
             * Communication that was healthy at startup must remain healthy.
             * This also prevents the three auto-retransmitting TX mailboxes
             * from staying permanently saturated if the CAN link disappears
             * while the operator is waiting to press KEY0.
             */
            if ((Motion_PositionIsFresh(home_x.motion) == 0U) ||
                (Motion_PositionIsFresh(home_y.motion) == 0U))
            {
                System_EnterError();
                break;
            }

            /*
             * LOOP_APPLY normal path:
             *   first physical KEY0 press -> relative X correction -> relative Y
             *   correction -> 500 ms settle -> one AFTER hardware trigger.
             *
             * There is deliberately NO Home here.  The external motor drivers
             * must have remained powered and the mechanism must not have been
             * moved since the valid BEFORE frame.
             *
             * Debug-only escape: g_debug_home_request=2 still starts the old
             * Home path. Do not use that during the closed-loop APPLY test.
             */
            if (g_debug_home_request == 2U)
            {
                g_debug_home_request = 0U;
                Key0_ResetHold();

                if (!System_StartHome())
                {
                    System_EnterError();
                }
            }
            else if ((g_debug_home_request != 0U) ||
                     (Key0_HoldStartRequested() != 0U))
            {
                g_debug_home_request = 0U;
                Key0_ResetHold();
                system_state = SYS_LOOP_APPLY_X_START;
            }
            break;


        case SYS_LOOP_APPLY_X_START:

            if (Motion_StartRelative(
                    home_x.motion,
                    g_loop_apply_dx,
                    LOOP_APPLY_SPEED,
                    LOOP_APPLY_ACCEL))
            {
                system_state = SYS_LOOP_APPLY_X_WAIT;
            }
            else
            {
                System_EnterError();
            }
            break;


        case SYS_LOOP_APPLY_X_WAIT:

            if (!Motion_IsBusy(home_x.motion))
            {
                if (Motion_GetResult(home_x.motion) == MOTION_RESULT_OK)
                {
                    system_state = SYS_LOOP_APPLY_Y_START;
                }
                else
                {
                    System_EnterError();
                }
            }
            break;


        case SYS_LOOP_APPLY_Y_START:

            if (Motion_StartRelative(
                    home_y.motion,
                    g_loop_apply_dy,
                    LOOP_APPLY_SPEED,
                    LOOP_APPLY_ACCEL))
            {
                system_state = SYS_LOOP_APPLY_Y_WAIT;
            }
            else
            {
                System_EnterError();
            }
            break;


        case SYS_LOOP_APPLY_Y_WAIT:

            if (!Motion_IsBusy(home_y.motion))
            {
                if (Motion_GetResult(home_y.motion) == MOTION_RESULT_OK)
                {
                    system_state_tick = HAL_GetTick();
                    system_state = SYS_LOOP_APPLY_SETTLE;
                }
                else
                {
                    System_EnterError();
                }
            }
            break;


        case SYS_LOOP_APPLY_SETTLE:

            if ((HAL_GetTick() - system_state_tick) >=
                LOOP_APPLY_SETTLE_MS)
            {
                if ((Motion_PositionIsFresh(home_x.motion) == 0U) ||
                    (Motion_PositionIsFresh(home_y.motion) == 0U))
                {
                    System_EnterError();
                }
                else
                {
                    system_state = SYS_LOOP_APPLY_TRIG_START;
                }
            }
            break;


        case SYS_LOOP_APPLY_TRIG_START:

            HAL_GPIO_WritePin(
                CAMERA_TRIG_GPIO_Port,
                CAMERA_TRIG_Pin,
                GPIO_PIN_RESET);

            system_state_tick = HAL_GetTick();
            system_state = SYS_LOOP_APPLY_TRIG_WAIT;
            break;


        case SYS_LOOP_APPLY_TRIG_WAIT:

            if ((HAL_GetTick() - system_state_tick) >=
                CAMERA_TRIGGER_PULSE_MS)
            {
                HAL_GPIO_WritePin(
                    CAMERA_TRIG_GPIO_Port,
                    CAMERA_TRIG_Pin,
                    GPIO_PIN_SET);

                g_loop_apply_done = 1U;
                system_state = SYS_LOOP_DONE;
            }
            break;


        case SYS_LOOP_DONE:

            /* One-shot build: hold position and do nothing else. */
            break;


        case SYS_HOME_X:

            if (AxisHome_Task(&home_x))
            {
                if (home_x.stage == HOME_STAGE_DONE)
                {
                    /* Y timeout starts only when Y actually begins. */
                    home_y.home_start_tick = HAL_GetTick();
                    system_state = SYS_HOME_Y;
                }
                else
                {
                    System_EnterError();
                }
            }
            break;


        case SYS_HOME_Y:

            if (AxisHome_Task(&home_y))
            {
                if (home_y.stage == HOME_STAGE_DONE)
                {
                    system_state = SYS_HOME_DONE;
                }
                else
                {
                    System_EnterError();
                }
            }
            break;


        case SYS_HOME_DONE:

            /*
             * Home is now infrastructure, not the experiment.
             * Immediately transition to the camera working position.
             */
            system_state = SYS_MOVE_CAMERA_X_START;
            break;


        case SYS_MOVE_CAMERA_X_START:

            if (Camera_TargetAlreadyReached(
                    home_x.motion,
                    g_camera_ready_x_target))
            {
                if (g_camera_ready_y_enable != 0U)
                {
                    system_state = SYS_MOVE_CAMERA_Y_START;
                }
                else
                {
                   System_StartCameraSettle();
                }
            }
            else if (Motion_StartAbsoluteMachine(
                         home_x.motion,
                         g_camera_ready_x_target,
                         CAMERA_READY_SPEED,
                         CAMERA_READY_ACCEL))
            {
                system_state = SYS_MOVE_CAMERA_X_WAIT;
            }
            else
            {
                System_EnterError();
            }
            break;


        case SYS_MOVE_CAMERA_X_WAIT:

            if (!Motion_IsBusy(home_x.motion))
            {
                if (Motion_GetResult(home_x.motion) == MOTION_RESULT_OK)
                {
                    if (!Camera_TargetWithin(
                            home_x.motion,
                            g_camera_ready_x_target,
                            CAMERA_READY_VERIFY_TOLERANCE))
                    {
                        System_EnterError();
                    }
                    else if (g_camera_ready_y_enable != 0U)
                    {
                        system_state = SYS_MOVE_CAMERA_Y_START;
                    }
                    else
                    {
                        System_StartCameraSettle();
                    }
                }
                else
                {
                    System_EnterError();
                }
            }
            break;


        case SYS_MOVE_CAMERA_Y_START:

            if (Camera_TargetAlreadyReached(
                    home_y.motion,
                    g_camera_ready_y_target))
            {
                System_StartCameraSettle();
            }
            else if (Motion_StartAbsoluteMachine(
                         home_y.motion,
                         g_camera_ready_y_target,
                         CAMERA_READY_SPEED,
                         CAMERA_READY_ACCEL))
            {
                system_state = SYS_MOVE_CAMERA_Y_WAIT;
            }
            else
            {
                System_EnterError();
            }
            break;


        case SYS_MOVE_CAMERA_Y_WAIT:

            if (!Motion_IsBusy(home_y.motion))
            {
                if (Motion_GetResult(home_y.motion) == MOTION_RESULT_OK)
                {
                    if (!Camera_TargetWithin(
                            home_y.motion,
                            g_camera_ready_y_target,
                            CAMERA_READY_VERIFY_TOLERANCE))
                    {
                        System_EnterError();
                    }
                    else
                    {
                        System_StartCameraSettle();
                    }
                }
                else
                {
                    System_EnterError();
                }
            }
            break;


        case SYS_CAMERA_SETTLE:

                /*
                 * Allow the stage / target / camera image to settle
                 * before issuing the hardware trigger.
                 */
                if ((HAL_GetTick() - system_state_tick) >=
                    CAMERA_SETTLE_MS)
                {
                    if (Camera_AllTargetsVerified() == 0U)
                    {
                        System_EnterError();
                    }
                    else
                    {
                        system_state = SYS_CAMERA_TRIGGER_START;
                    }
                }

                break;

        case SYS_CAMERA_TRIGGER_START:

            /*
             * Previous standalone hardware test confirmed:
             *
             * PB0 HIGH -> optocoupler output O1 ~= 0 V  (idle)
             * PB0 LOW  -> optocoupler output O1 rises   (camera trigger)
             *
             * MVS is configured for Line0 Rising Edge.
             * Therefore pulling PB0 LOW starts one camera trigger.
             */
            g_camera_ready = 0U;

            HAL_GPIO_WritePin(
                CAMERA_TRIG_GPIO_Port,
                CAMERA_TRIG_Pin,
                GPIO_PIN_RESET);

            system_state_tick = HAL_GetTick();
            system_state = SYS_CAMERA_TRIGGER_WAIT;
            break;


        case SYS_CAMERA_TRIGGER_WAIT:

            /*
             * Non-blocking 10 ms active-low pulse.
             * System_Task is serviced periodically by the existing ControlTask.
             */
            if ((HAL_GetTick() - system_state_tick) >=
                CAMERA_TRIGGER_PULSE_MS)
            {
                HAL_GPIO_WritePin(
                    CAMERA_TRIG_GPIO_Port,
                    CAMERA_TRIG_Pin,
                    GPIO_PIN_SET);

                g_camera_ready = 1U;

                /*
                 * BEFORE is complete only after its 10 ms PB0 trigger has
                 * ended.  READY_REF is point 0; BEFORE is point 1.
                 */
                if ((CAL_3POINT_ENABLE != 0U) &&
                    (g_cal_point_index >= 1U))
                {
                    g_cal_sequence_done = 1U;
                }

                system_state = SYS_CAMERA_READY;
            }
            break;

        case SYS_CAMERA_READY:

            /*
             * A move request has priority over ready-position verification.
             *
             * Reason: laboratory tuning / future vision control may update
             * g_camera_ready_*_target and g_camera_move_request together.
             * If we compared the current position against the NEW target first,
             * the system would enter SYS_ERROR before it had a chance to move.
             */
            if (g_camera_move_request != 0U)
            {
                g_camera_move_request = 0U;
                g_camera_ready = 0U;
                system_state = SYS_MOVE_CAMERA_X_START;
                break;
            }

            /*
             * Without a move request, CAMERA_READY remains valid only while
             * the current target is in tolerance and encoder feedback is fresh.
             */
            if (Camera_AllTargetsVerified() == 0U)
            {
                System_EnterError();
                break;
            }

            g_camera_ready = 1U;

            /*
             * Closed-loop BEFORE V2:
             *
             * Capture 1 (READY_REF) is the frozen normal CameraReady frame.
             * After the operator releases the Home-start key:
             *   next KEY0 hold -> move to LOOP_BEFORE_TEST_X/Y
             *                   -> verify position
             *                   -> settle 500 ms
             *                   -> capture 2 (BEFORE)
             *
             * There is no second Home.
             */
            if ((CAL_3POINT_ENABLE != 0U) &&
                (g_cal_sequence_done == 0U) &&
                (Cal_KeyAdvanceRequested() != 0U))
            {
                if (Cal_StartNextPoint() != 0U)
                {
                    break;
                }
            }

            /*
             * Pure camera repeatability test:
             * trigger another frame without commanding X/Y motion.
             */
            if (g_camera_trigger_request != 0U)
            {
                g_camera_trigger_request = 0U;
                g_camera_ready = 0U;
                system_state = SYS_CAMERA_TRIGGER_START;
            }

            break;

        case SYS_ERROR:

            g_camera_ready = 0U;
            /* Terminal error for this boot: no automatic movement. */
            break;


        default:

            g_camera_ready = 0U;
            System_EnterError();
            break;
    }
}
