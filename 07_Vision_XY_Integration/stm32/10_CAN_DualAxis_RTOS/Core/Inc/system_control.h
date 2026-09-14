#ifndef __SYSTEM_CONTROL_H
#define __SYSTEM_CONTROL_H

#include "motion_control.h"

typedef enum
{
    SYS_STARTUP              = 0U,
    SYS_WAIT_ENABLE          = 1U,
    SYS_WAIT_HOME_KEY        = 2U,

    SYS_HOME_X               = 3U,
    SYS_HOME_Y               = 4U,
    SYS_HOME_DONE            = 5U,

    SYS_MOVE_CAMERA_X_START  = 6U,
    SYS_MOVE_CAMERA_X_WAIT   = 7U,
    SYS_MOVE_CAMERA_Y_START  = 8U,
    SYS_MOVE_CAMERA_Y_WAIT   = 9U,

    /* Stable hand-off point for the future PC/OpenCV communication layer. */
    SYS_CAMERA_READY         = 10U,
    SYS_ERROR                = 11U,
    SYS_CAMERA_TRIGGER_START = 12U,
    SYS_CAMERA_TRIGGER_WAIT  = 13U,
    /* Keep existing state numbers unchanged. */
    SYS_CAMERA_SETTLE        = 14U
} SystemState_t;

typedef enum
{
    HOME_STAGE_FAST_START      = 0U,
    HOME_STAGE_FAST_WAIT       = 1U,

    HOME_STAGE_BACK_START      = 2U,
    HOME_STAGE_BACK_WAIT       = 3U,

    HOME_STAGE_SLOW_START      = 4U,
    HOME_STAGE_SLOW_WAIT       = 5U,

    HOME_STAGE_STABLE_CONFIRM  = 6U,

    HOME_STAGE_RELEASE_START   = 7U,
    HOME_STAGE_RELEASE_WAIT    = 8U,
    HOME_STAGE_RELEASE_CONFIRM = 9U,

    HOME_STAGE_DONE            = 10U,
    HOME_STAGE_ERROR           = 11U

    

} HomeStage_t;

typedef uint8_t (*HomeSensorFn_t)(void);

typedef struct
{
    MotionController_t *motion;
    HomeSensorFn_t sensor_triggered;

    int32_t dir_sign;

    HomeStage_t stage;
    uint32_t home_start_tick;
    uint32_t next_move_tick;
    uint32_t stable_start_tick;

    uint8_t back_retry_count;
    uint8_t release_retry_count;

    /*
     * Home GPIO debounce state.
     * Raw PG6/PG7 is sampled every ControlTask cycle, while stage transitions
     * consume only the 30 ms stable debounced level.
     */
    uint8_t sensor_raw_last;
    uint8_t sensor_debounced;
    uint8_t sensor_ready;
    uint32_t sensor_change_tick;

} AxisHomeCtx_t;

void System_Init(
    MotionController_t *motion_x,
    MotionController_t *motion_y);

void System_Task(void);

SystemState_t System_GetState(void);
uint8_t System_IsCameraReady(void);

/*
 * Vision-MVP diagnostics / fast tuning values for Keil Watch.
 * g_camera_ready_x_target and g_camera_ready_y_target may be edited in Watch
 * BEFORE the corresponding move starts, which avoids rebuilding just to probe
 * a nearby camera position.  Put the final values back into vision_config.h.
 */
extern volatile SystemState_t system_state;
extern volatile uint32_t g_build_id;
extern volatile uint8_t g_key0_level;
extern volatile uint32_t g_key0_hold_ms;
extern volatile uint32_t g_key0_start_count;
extern volatile uint8_t g_camera_ready;
extern volatile uint8_t g_debug_home_request;
extern volatile uint8_t g_camera_move_request;
extern volatile uint8_t g_camera_trigger_request;

extern volatile uint8_t g_camera_ready_y_enable;
extern volatile int64_t g_camera_ready_x_target;
extern volatile int64_t g_camera_ready_y_target;
extern volatile int64_t g_camera_ready_x_actual;
extern volatile int64_t g_camera_ready_y_actual;
extern volatile SystemState_t g_system_error_from_state;
extern volatile HomeStage_t g_home_x_stage;
extern volatile HomeStage_t g_home_y_stage;

#endif
