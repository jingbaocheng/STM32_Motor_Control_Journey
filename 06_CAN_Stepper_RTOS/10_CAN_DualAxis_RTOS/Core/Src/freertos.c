/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motion_control.h"
#include "system_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern MotionController_t Motion_X;
extern MotionController_t Motion_Y;

/* USER CODE END Variables */
/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SystemTask */
osThreadId_t SystemTaskHandle;
const osThreadAttr_t SystemTask_attributes = {
  .name = "SystemTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotionCommandQueue */
osMessageQueueId_t MotionCommandQueueHandle;
const osMessageQueueAttr_t MotionCommandQueue_attributes = {
  .name = "MotionCommandQueue"
};
/* Definitions for MotionResultQueue */
osMessageQueueId_t MotionResultQueueHandle;
const osMessageQueueAttr_t MotionResultQueue_attributes = {
  .name = "MotionResultQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMotionTask(void *argument);
void StartSystemTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MotionCommandQueue */
  MotionCommandQueueHandle = osMessageQueueNew (5, 16, &MotionCommandQueue_attributes);

  /* creation of MotionResultQueue */
  MotionResultQueueHandle = osMessageQueueNew (5, 8, &MotionResultQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotionTask */
  MotionTaskHandle = osThreadNew(StartMotionTask, NULL, &MotionTask_attributes);

  /* creation of SystemTask */
  SystemTaskHandle = osThreadNew(StartSystemTask, NULL, &SystemTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMotionTask */
/**
  * @brief  Function implementing the MotionTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotionTask */
void StartMotionTask(void *argument)
{
  /* USER CODE BEGIN StartMotionTask */
  /* Infinite loop */
 MotionCommand_t received_command;
    MotionResult_t result_message;
   
    MotionController_t *target_motion = NULL;
    MotionController_t *active_motion = NULL;
     uint32_t active_command_id = 0U;
    osStatus_t status;

    for (;;)
    {
        /* ��ǰû���˶���������ִ�У��ŴӶ���ȡ��һ�� */
        if (active_motion == NULL)
        {
            status = osMessageQueueGet(MotionCommandQueueHandle,
                                       &received_command,
                                       NULL,
                                       0U);

            if (status == osOK)
            {
                if (received_command.axis == MOTION_AXIS_X)
                {
                    target_motion = &Motion_X;
                }
                else if (received_command.axis == MOTION_AXIS_Y)
                {
                    target_motion = &Motion_Y;
                }
                else
                {
                    target_motion = NULL;
                }

                if (target_motion != NULL)
                {
                    if (received_command.type == MOTION_CMD_ENABLE)
                    {
                        MKS_Set_Enable_State(target_motion->motor,
                                             received_command.enable);
                    }
                    else if (received_command.type == MOTION_CMD_MOVE_REL)
                    {
                        Motion_StartRelative(
                            target_motion,
                            received_command.displacement,
                            received_command.speed,
                            received_command.acceleration);

                       /* 记住当前正在执行的是哪一号命令 */
                        active_command_id = received_command.command_id;
                        active_motion = target_motion;
                    }
                }
            }
        }

        /* �����Ƿ���������������ƽ�����״̬�� */
        Motion_Task(&Motion_X);
        Motion_Task(&Motion_Y);

        /* ��ǰ�˶���ɣ�������һ�ִӶ���ȡ������ */
       if ((active_motion != NULL) &&
    (active_motion->motion_done == 1U))
{
    result_message.command_id = active_command_id;
    result_message.result = 0U;  /* 0 暂时表示执行成功 */

    if (osMessageQueuePut(MotionResultQueueHandle,
                          &result_message,
                          0U,
                          osWaitForever) == osOK)
    {
        active_command_id = 0U;
        active_motion = NULL;
    }
}
        osDelay(10U);
    }
  /* USER CODE END StartMotionTask */
}

/* USER CODE BEGIN Header_StartSystemTask */
/**
* @brief Function implementing the SystemTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSystemTask */
void StartSystemTask(void *argument)
{
  /* USER CODE BEGIN StartSystemTask */
  /* Infinite loop */
 for (;;)
    {
        System_Task();

        osDelay(10U);
    }
    
  /* USER CODE END StartSystemTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

