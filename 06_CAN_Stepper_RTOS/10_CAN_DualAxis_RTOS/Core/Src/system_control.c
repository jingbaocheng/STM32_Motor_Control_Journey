#include "system_control.h"
#include "cmsis_os2.h"

/* 队列句柄实际定义在 freertos.c */
extern osMessageQueueId_t MotionCommandQueueHandle;
extern osMessageQueueId_t MotionResultQueueHandle;

//static MotionController_t *motion_x;
//static MotionController_t *motion_y;


static SystemState_t system_state = SYS_STARTUP;
static uint32_t system_state_tick = 0U;

/* 接收 MotionTask 返回的完成结果 */
static MotionResult_t received_result;
static uint32_t completed_count = 0U;

void System_Init(MotionController_t *x,
                 MotionController_t *y)
{
   (void)x;
    (void)y;

    system_state = SYS_STARTUP;
    system_state_tick = 0U;

    received_result.command_id = 0U;
    received_result.result = 0U;
    completed_count = 0U;
}

void System_Task(void)
{
    MotionCommand_t command = {0};

    switch (system_state)
    {
        case SYS_STARTUP:

            /* 发送 X 轴使能命令 */
            command.command_id = 0U;
            command.type = MOTION_CMD_ENABLE;
            command.axis = MOTION_AXIS_X;
            command.enable = 1U;

            if (osMessageQueuePut(MotionCommandQueueHandle,
                                  &command,
                                  0U,
                                  osWaitForever) != osOK)
            {
                system_state = SYS_ERROR;
                break;
            }

            /* 发送 Y 轴使能命令 */
            command.axis = MOTION_AXIS_Y;

            if (osMessageQueuePut(MotionCommandQueueHandle,
                                  &command,
                                  0U,
                                  osWaitForever) != osOK)
            {
                system_state = SYS_ERROR;
                break;
            }

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

            /*
             * 准备开始一批新的运动命令，
             * 先清空上一批的完成计数。
             */
            completed_count = 0U;

            /* 第1条：X 正向 */
            command.command_id = 1U;
            command.type = MOTION_CMD_MOVE_REL;
            command.axis = MOTION_AXIS_X;
            command.displacement = 4096;
            command.speed = 50U;
            command.acceleration = 2U;

            if (osMessageQueuePut(MotionCommandQueueHandle,
                                  &command,
                                  0U,
                                  osWaitForever) != osOK)
            {
                system_state = SYS_ERROR;
                break;
            }

            /* 第2条：X 反向 */
            command.command_id = 2U;
            command.type = MOTION_CMD_MOVE_REL;
            command.axis = MOTION_AXIS_X;
            command.displacement = -4096;
            command.speed = 50U;
            command.acceleration = 2U;

            if (osMessageQueuePut(MotionCommandQueueHandle,
                                  &command,
                                  0U,
                                  osWaitForever) != osOK)
            {
                system_state = SYS_ERROR;
                break;
            }

            /* 第3条：Y 正向 */
            command.command_id = 3U;
            command.type = MOTION_CMD_MOVE_REL;
            command.axis = MOTION_AXIS_Y;
            command.displacement = 2048;
            command.speed = 30U;
            command.acceleration = 1U;

            if (osMessageQueuePut(MotionCommandQueueHandle,
                                  &command,
                                  0U,
                                  osWaitForever) != osOK)
            {
                system_state = SYS_ERROR;
                break;
            }

            /* 第4条：Y 反向 */
            command.command_id = 4U;
            command.type = MOTION_CMD_MOVE_REL;
            command.axis = MOTION_AXIS_Y;
            command.displacement = -2048;
            command.speed = 30U;
            command.acceleration = 1U;

            if (osMessageQueuePut(MotionCommandQueueHandle,
                                  &command,
                                  0U,
                                  osWaitForever) != osOK)
            {
                system_state = SYS_ERROR;
                break;
            }

            /*
             * 四条命令只是全部入队，
             * 还不能进入 SYS_DONE。
             */
            system_state = SYS_WAIT_ALL;
            break;

        case SYS_WAIT_ALL:

            /*
             * 非阻塞检查完成结果队列。
             * 每次 System_Task() 最多领取一条结果。
             */
            if (osMessageQueueGet(MotionResultQueueHandle,
                                  &received_result,
                                  NULL,
                                  0U) == osOK)
            {
                if (received_result.result == 0U)
                {
                    completed_count++;

                    if (completed_count >= 4U)
                    {
                        system_state = SYS_DONE;
                    }
                }
                else
                {
                    system_state = SYS_ERROR;
                }
            }

            break;

        /*
         * 下面三个是旧流程状态。
         * 当前新流程不会进入，暂时保留枚举兼容性。
         */
        case SYS_WAIT_X:
            break;

        case SYS_START_Y:
            break;

        case SYS_WAIT_Y:
            break;

        case SYS_DONE:
            break;

        case SYS_ERROR:
            break;

        default:
            system_state = SYS_ERROR;
            break;
         /* 临时调试探针 */
    
    }
}
