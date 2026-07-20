#ifndef __SYSTEM_CONTROL_H
#define __SYSTEM_CONTROL_H


#include "mks_motor.h"
#include "motion_control.h"

typedef enum
{
    SYS_STARTUP,
    SYS_WAIT_ENABLE,

    SYS_START_X,
    SYS_WAIT_X,

    SYS_START_Y,
    SYS_WAIT_Y,

    SYS_DONE,
    SYS_ERROR
} SystemState_t;

void System_Init(MotionController_t *motion_x,
                 MotionController_t *motion_y);

void System_Task(void);

#endif
