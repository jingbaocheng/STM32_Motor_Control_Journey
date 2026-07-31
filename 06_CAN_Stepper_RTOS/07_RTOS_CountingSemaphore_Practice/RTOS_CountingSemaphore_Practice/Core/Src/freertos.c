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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

#define BUTTON_DEBOUNCE_MS    50U

static uint32_t button_last_tick = 0U;

volatile uint32_t interrupt_count = 0U;
volatile uint32_t processed_count = 0U;
/* USER CODE END Variables */
/* Definitions for EventTask */
osThreadId_t EventTaskHandle;
const osThreadAttr_t EventTask_attributes = {
  .name = "EventTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EventCountSem */
osSemaphoreId_t EventCountSemHandle;
const osSemaphoreAttr_t EventCountSem_attributes = {
  .name = "EventCountSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartEventTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of EventTask */
  EventTaskHandle = osThreadNew(StartEventTask, NULL, &EventTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartEventTask */
/**
  * @brief  Function implementing the EventTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEventTask */
void StartEventTask(void *argument)
{
  /* USER CODE BEGIN StartEventTask */
  /* Infinite loop */
  for(;;)
  {
     if (osSemaphoreAcquire(EventCountSemHandle,
                               osWaitForever) == osOK)
        {
            processed_count++;

            HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);

            /* 故意让任务每秒只处理一个事件 */
            osDelay(1000U);
        }
  }
  /* USER CODE END StartEventTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_tick;

    if ((GPIO_Pin == GPIO_PIN_4) &&
        (EventCountSemHandle != NULL))
    {
        current_tick = HAL_GetTick();

        if ((current_tick - button_last_tick) >= BUTTON_DEBOUNCE_MS)
        {
            button_last_tick = current_tick;
             interrupt_count++;
            (void)osSemaphoreRelease(EventCountSemHandle);
        }
    }
}

/* USER CODE END Application */

