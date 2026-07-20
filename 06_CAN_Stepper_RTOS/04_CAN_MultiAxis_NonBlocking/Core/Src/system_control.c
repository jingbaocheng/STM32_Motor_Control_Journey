#include "system_control.h"

static MotionController_t *motion_x;
static MotionController_t *motion_y;

static SystemState_t system_state = SYS_STARTUP;
static uint32_t system_state_tick = 0;

void System_Init(MotionController_t *x, MotionController_t *y)
{
    motion_x = x;
    motion_y = y;

    system_state = SYS_STARTUP;
    system_state_tick = 0U;
}
void System_Task(void)
{
    switch (system_state)
    {
        case SYS_STARTUP:
            MKS_Set_Enable_State(motion_x->motor, 1U);
            MKS_Set_Enable_State(motion_y->motor, 1U);

            system_state_tick = HAL_GetTick();
            system_state = SYS_WAIT_ENABLE;
            break;

        case SYS_WAIT_ENABLE:
            if ((HAL_GetTick() - system_state_tick) >= 500U)
            {
                system_state = SYS_START_X;
            }
            break;
        case SYS_START_X:
            Motion_StartRelative(motion_x, 8192, 60U, 2U);
            system_state = SYS_WAIT_X;
            break;

        case SYS_WAIT_X:
            if (motion_x->motion_done == 1U)
            {
                system_state = SYS_START_Y;
            }
            break;
        case SYS_START_Y:
            Motion_StartRelative(motion_y, -4096, 30U, 1U);
            system_state = SYS_WAIT_Y;
            break;

        case SYS_WAIT_Y:
            if (motion_y->motion_done == 1U)
            {
                system_state = SYS_DONE;
            }
            break;

        case SYS_DONE:
            break;
        default:
            break;
        
        
    }
}
