/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 全局变量定义区 */
int target_steps = 3200;      // 总共要走的步数
int current_step = 0;         // 已经走了的步数 (注意：昨天我们是倒着减，今天为了分段，我们正着加)

int accel_steps = 800;        // 用多少步来加速
int decel_steps = 800;        // 用多少步来减速

uint16_t arr_current = 2000;  // 起步时的 ARR (很大，意味着起步很慢)
uint16_t arr_min = 500;       // 巡航时的最小 ARR (速度最快)
uint16_t arr_step = 2;        // 每次进入中断，ARR 改变的步长 (比如每次减2或加2)

void Profile_Check(void)
{
    int expected_accel = 800;
    int expected_decel = 800;

    // 安检门：目标步数太短，发生“梯形退化为三角形”
    if (target_steps < (expected_accel + expected_decel))
    {
        accel_steps = target_steps / 2; // C语言自动向下取整
        decel_steps = accel_steps;
    }
    else 
    {
        accel_steps = expected_accel;
        decel_steps = expected_decel;
    }
}



void Moter_Start()
{
      // 1. 开火前必须先过安检，计算真正的加减速步数！
    Profile_Check(); 

    // 2. 状态全部复位（清空弹夹）
    current_step = 0;
    arr_current = 2000; // 恢复到起步的慢速 ARR
    
    // 3. 把初始速度和 50% 占空比硬塞进底层寄存器
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr_current);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arr_current / 2);


HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
 HAL_Delay(3000); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      Moter_Start();      // 下令开火（走一圈）
    HAL_Delay(5000);    // 休息 2 秒，看它停不停
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
 void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance ==TIM4)
{
 current_step++; 
/* ======= 核心状态机 ======= */
  /* -------- 阶段 1：加速区 -------- */
   if (current_step <= accel_steps)
        {
       if (arr_current > arr_min) 
         {
            arr_current -= arr_step; 
         }
          // 写入新的频率，并严格保持 50% 占空比防止卡死！
          __HAL_TIM_SET_AUTORELOAD(&htim4, arr_current);
          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arr_current / 2);
        }
   else if (current_step > (target_steps - decel_steps))
        {
            if (arr_current < 2000) // 不超过起步速度
            {
                arr_current += arr_step;
            }
            // 写入新的频率，并严格保持 50%占空比
            __HAL_TIM_SET_AUTORELOAD(&htim4, arr_current);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arr_current / 2);
        }

 /* ======= 终点判定 ======= */
 if (current_step >= target_steps) 
        {
            HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1); // 到达终点，停火！
            current_step = 0; // 计数器清零，为下一次开火做准备
            arr_current = 2000; // ARR 恢复到起步速度，为下一次起步做准备
        }

}

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
