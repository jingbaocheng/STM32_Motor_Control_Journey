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
#include "string.h"
#include <stdio.h>
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
void Motor_SetSpeed(int speed)
{
    if(speed>0)
    {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
        if(speed>100){speed =100;}
        __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,speed);
        
    }else if (speed<0)
    {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET); 
        speed =-speed;    
        if(speed>100) {speed =100;}
        
        __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,speed);
        
                    
    }else {
        // 刹车 (speed == 0 的情况)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 0);
    }
    

}
/* =======================================
 * 1. PID 核心结构体定义 (造控制大脑)
 * ======================================= */
typedef struct {
    float Kp;           // 比例系数 (爆发力)
    float Ki;           // 积分系数 (死磕精神)
    float Kd;           // 微分系数 (减震器)
    
    float target;       // 目标转速
    float error;        // 当前误差
    float last_error;   // 上一次误差
    float integral;     // 误差累加值
    
    float output;       // 算出来的 PWM 占空比
} PID_Controller;

PID_Controller myPID;   // 实例化一个 PID 大脑

// 定义虚拟电机的物理状态
float virtual_motor_speed = 0.0; 


/* =======================================
 * 2. PID 初始化函数
 * ======================================= */
void PID_Init(void) {
    // 假设这是我们盲猜的第一套参数
    myPID.Kp = 2.0;
    myPID.Ki = 5.0;
    myPID.Kd = 0.0;
    
    myPID.target = 100.0; // 我们想让电机跑到 100 RPM
    
    myPID.error = 0;
    myPID.last_error = 0;
    myPID.integral = 0;
    myPID.output = 0;
}


/* =======================================
 * 3. PID 核心算法 (每隔 100ms 调用一次)
 * ======================================= */
float PID_Calc(float current_speed) {
    // 1. 算误差
    myPID.error = myPID.target - current_speed;
    
    // 2. 算积分 (累加历史误差)
    myPID.integral += myPID.error;
    
    // 【极其重要的保护】：积分限幅，防止暴走
    if (myPID.integral > 1000) myPID.integral = 1000;
    if (myPID.integral < -1000) myPID.integral = -1000;
    
    // 3. 终极公式：P + I + D
    myPID.output = (myPID.Kp * myPID.error) + 
                   (myPID.Ki * myPID.integral) + 
                   (myPID.Kd * (myPID.error - myPID.last_error));
                   
    // 4. PWM 限幅 (占空比不能超过 100%)
    if (myPID.output > 100) myPID.output = 100;
    if (myPID.output < -100) myPID.output = -100;
    
    // 5. 更新上一次误差
    myPID.last_error = myPID.error;
    
    return myPID.output;
}


/* =======================================
 * 4. 虚拟电机物理世界模拟器 (不要修改)
 * ======================================= */
float Simulate_Motor(float pwm_input) {
    // 牛顿第一定律模拟：当前速度 = 之前速度 + 动力加速 - 摩擦力减速
    virtual_motor_speed = virtual_motor_speed + (pwm_input * 0.08) - (virtual_motor_speed * 0.02);
    return virtual_motor_speed;
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
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
 /* 在 while(1) 外面定义变量 */
int32_t current_count = 0;
int32_t last_count = 0;
int32_t delta_count = 0;
float speed_rpm = 0.0; 
  // --- 2. 启动 PWM (注意：必须是 TIM_CHANNEL_1) ---
  // 之前的 TIM_CHANNEL_3 是错的，TIM14 只有通道 1
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); 
 Motor_SetSpeed(50);
  // --- 3. 启动编码器 (不变) ---
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

 char message[50]="";
 PID_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    HAL_Delay(100);
//     int32_t current_count = __HAL_TIM_GET_COUNTER(&htim2);
//    // --- 1. 设置方向 (对应 PB12 和 PB13) ---
//    // 这里的引脚要和你接的一致
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // BIN1 = 1
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // BIN2 = 0
//        int32_t delta_count = current_count-last_count;
//        last_count = current_count;
//       // 5. 换算成真实的 RPM (转/分钟)
//    // 假设你的电机转一圈，编码器输出 1000 个脉冲
//     speed_rpm = (delta_count / 1560.0) * 10 * 60; 
//    // (除以1000得到圈数，乘10变成1秒的圈数，乘60变成1分钟的圈数)
//      sprintf(message,"speed_rpm: %f\r\n",  speed_rpm);
//      HAL_UART_Transmit_IT(&huart1, (uint8_t*)message, strlen(message));
//       
//    // --- 2. 设置速度 (PF9 / TIM14_CH1) ---
//    // 设为 8000 (接近 100% 全速)
//    // 同样注意：必须是 TIM_CHANNEL_1
//   __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 50);
//    
//    // 延时一下
//    HAL_Delay(100);
/* USER CODE BEGIN 3 */
// 1. 大脑计算：根据当前虚拟速度，PID 算出现在该给多少占空比的油门
float pwm_cmd = PID_Calc(virtual_motor_speed);

// 2. 物理模拟：把算出来的油门传给虚拟电机，计算出它下一刻的真实速度
virtual_motor_speed = Simulate_Motor(pwm_cmd);

// 3. 按照 VOFA+ (FireWater 流水协议) 打包数据
// 极简暗号：目标速度, 真实速度, 当前占空比 \n (注意这里全是逗号，没有其他中英文字符！)
sprintf(message, "%.1f,%.1f,%.1f\n", myPID.target, virtual_motor_speed, pwm_cmd);

// 4. 把打包好的数据通过串口发给电脑上的 VOFA+
HAL_UART_Transmit_IT(&huart1, (uint8_t*)message, strlen(message));

// 5. 充当秒表，100ms 算一次
HAL_Delay(100);
/* USER CODE END 3 */

    
  /* USER CODE END 3 */
}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
