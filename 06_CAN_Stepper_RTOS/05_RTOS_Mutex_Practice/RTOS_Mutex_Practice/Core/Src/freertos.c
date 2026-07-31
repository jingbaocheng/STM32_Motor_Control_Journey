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
 volatile uint32_t shared_counter = 0U;
 volatile uint8_t task_a_done = 0U;
 volatile uint8_t task_b_done = 0U;
/* USER CODE END Variables */
/* Definitions for TaskA */
osThreadId_t TaskAHandle;
const osThreadAttr_t TaskA_attributes = {
  .name = "TaskA",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskB */
osThreadId_t TaskBHandle;
const osThreadAttr_t TaskB_attributes = {
  .name = "TaskB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CounterMutex */
osMutexId_t CounterMutexHandle;
const osMutexAttr_t CounterMutex_attributes = {
  .name = "CounterMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTaskA(void *argument);
void StartTaskB(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of CounterMutex */
  CounterMutexHandle = osMutexNew(&CounterMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of TaskA */
  TaskAHandle = osThreadNew(StartTaskA, NULL, &TaskA_attributes);

  /* creation of TaskB */
  TaskBHandle = osThreadNew(StartTaskB, NULL, &TaskB_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskA */
/**
  * @brief  Function implementing the TaskA thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskA */
void StartTaskA(void *argument)
{
  /* USER CODE BEGIN StartTaskA */
  /* Infinite loop */
 
       uint32_t temp;

    for (uint32_t i = 0U; i < 1000U; i++)
    {
          osMutexAcquire(CounterMutexHandle, osWaitForever);
        temp = shared_counter;

        osThreadYield();

        shared_counter = temp + 1U;
        osMutexRelease(CounterMutexHandle);
    }

    task_a_done = 1U;

    for (;;)
    {
        osDelay(1000U);
    }

  
  /* USER CODE END StartTaskA */
}

/* USER CODE BEGIN Header_StartTaskB */
/**
* @brief Function implementing the TaskB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskB */
void StartTaskB(void *argument)
{
  /* USER CODE BEGIN StartTaskB */
  /* Infinite loop */
    
    uint32_t temp;

    for (uint32_t i = 0U; i < 1000U; i++)
    {
        osMutexAcquire(CounterMutexHandle, osWaitForever);
        temp = shared_counter;

        osThreadYield();

        shared_counter = temp + 1U;
        osMutexRelease(CounterMutexHandle);
    }

    task_b_done = 1U;

    for (;;)
    {
        osDelay(1000U);
    }

  /* USER CODE END StartTaskB */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

