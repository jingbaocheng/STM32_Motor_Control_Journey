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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint32_t command_id;
    int32_t value;
} CommandMessage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_PROCESSED_FLAG    (1U << 0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint32_t received_command_id = 0U;
volatile int32_t received_value = 0;
volatile uint32_t received_count = 0U;
volatile uint32_t produced_count = 0U;

/* USER CODE END Variables */
/* Definitions for ProducerTask */
osThreadId_t ProducerTaskHandle;
const osThreadAttr_t ProducerTask_attributes = {
  .name = "ProducerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ConsumerTask */
osThreadId_t ConsumerTaskHandle;
const osThreadAttr_t ConsumerTask_attributes = {
  .name = "ConsumerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};
/* Definitions for EventCountSem */
osSemaphoreId_t EventCountSemHandle;
const osSemaphoreAttr_t EventCountSem_attributes = {
  .name = "EventCountSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartProducerTask(void *argument);
void StartConsumerTask(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of EventCountSem */
  EventCountSemHandle = osSemaphoreNew(10, 0, &EventCountSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (5, 8, &CommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ProducerTask */
  ProducerTaskHandle = osThreadNew(StartProducerTask, NULL, &ProducerTask_attributes);

  /* creation of ConsumerTask */
  ConsumerTaskHandle = osThreadNew(StartConsumerTask, NULL, &ConsumerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartProducerTask */
/**
  * @brief  Function implementing the ProducerTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartProducerTask */
void StartProducerTask(void *argument)
{
  /* USER CODE BEGIN StartProducerTask */
  /* Infinite loop */
  
   CommandMessage_t message;
    uint32_t next_command_id = 1U;
    int32_t next_value = 100;
    osStatus_t status;

    for (;;)
    {
        /* 等待一次按键事件 */
        status = osSemaphoreAcquire(
            EventCountSemHandle,
            osWaitForever
        );

        if (status == osOK)
        {
            /* 获得一次许可后，准备一条消息 */
            message.command_id = next_command_id;
            message.value = next_value;

            status = osMessageQueuePut(
                CommandQueueHandle,
                &message,
                0U,
                osWaitForever
            );

            if (status == osOK)
            {
                produced_count++;

                next_command_id++;
                next_value += 100;

                /* 等待消费者处理完成 */
                osThreadFlagsWait(
                    COMMAND_PROCESSED_FLAG,
                    osFlagsWaitAny,
                    osWaitForever
                );
            }
        }
    }
  /* USER CODE END StartProducerTask */
}

/* USER CODE BEGIN Header_StartConsumerTask */
/**
* @brief Function implementing the ConsumerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartConsumerTask */
void StartConsumerTask(void *argument)
{
  /* USER CODE BEGIN StartConsumerTask */
  /* Infinite loop */
  CommandMessage_t received_message;
    osStatus_t status;

    for (;;)
    {
        status = osMessageQueueGet(CommandQueueHandle,
                                   &received_message,
                                   NULL,
                                   osWaitForever);

        if (status == osOK)
        {
            received_command_id = received_message.command_id;
            received_value = received_message.value;

            osDelay(1500U);

            received_count++;

            osThreadFlagsSet(ProducerTaskHandle,
                             COMMAND_PROCESSED_FLAG);
        }
    }
  /* USER CODE END StartConsumerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_4)
    {
        osSemaphoreRelease(EventCountSemHandle);
    }
}

/* USER CODE END Application */

