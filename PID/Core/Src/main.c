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
#include "stdio.h"
#include <math.h>
extern UART_HandleTypeDef huart1;
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

typedef struct{
float Kp;
float Ki;
 float Kd;
float Target;
float Actual;
 float Error;
 float LastError;
 float Integral;
 float Out;
    
}PID_TypeDef;
PID_TypeDef SpeedPID ={0};

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
 PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
 PID_TypeDef PosPID = {0};   // 实例化一个位置环控制器
int32_t CurrentPosition = 0; // 【极其关键】：定义一个 32 位的全局变量，记录绝对位置
 float Final_Target = 180000.0f;  // 真实终点
float Current_Target = 0.0f;     // 虚拟兔子当前位置
float Step_Speed = 0.0f;         // 兔子当前速度
float Max_Speed = 30.0f;         //兔子最高限速
float Acceleration = 0.5f;       // 子加速度/减速度

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 PosPID.Kp =0.08f; // 比例系数调小，防止起步猛冲
 PosPID.Ki = 0.01f; // 必须要有积分，消除稳态误差，防止哒哒哒震动
 PosPID.Kd =1.0f;
 PosPID.Target =180000.0f; // 目标速度：每10ms走15个脉冲

    
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
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //
HAL_Delay(5000); 

   
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 开启 pb6/pb7 编码器
HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);      // 开启 PF9 PWM
HAL_TIM_Base_Start_IT(&htim6);                  // 开启 10ms 中断
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {

    if(htim->Instance==TIM6)
    {
//    
//    int16_t encoder_count=(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
//    __HAL_TIM_SET_COUNTER(&htim4,0);
//    
//        SpeedPID.Actual = (0.7f * SpeedPID.Actual) + (0.3f * (float)encoder_count);
//        
        int16_t delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim4); // 读取这 10ms 的脉冲差
        __HAL_TIM_SET_COUNTER(&htim4, 0);                       // 依然清零定时器防溢出
        
        
        CurrentPosition += delta;            // 把这段增量加到总里程碑里！
        PosPID.Actual = (float)CurrentPosition; // 此时Actual 就是绝对位置
        
        float distance_to_go = Final_Target - Current_Target;// 距离 = 终点 - 兔子现在的位置
        // 物理学公式：刹车距离 = 速度的平方 / (2 * 加速度)
        float slow_down_distance = (Step_Speed * Step_Speed) / (2.0f * Acceleration); 
                        
            if (fabs(distance_to_go) > slow_down_distance) {
        // 还没到刹车点，允许加速！
            if (Step_Speed < Max_Speed) {
                Step_Speed += Acceleration; // 慢慢踩油门
            }
        } else {
            // 离终点不够远了，必须开始减速刹车！
            if (Step_Speed > 2.0f) { // // 保留一点最小速度防止卡死
                Step_Speed -= Acceleration; // 慢慢松油门
            }
        }

        /// 移动兔子
        if (distance_to_go > 0) {
            Current_Target += Step_Speed;  // 兔子往正方向跑
            if(Current_Target > Final_Target) Current_Target = Final_Target;// 防止超车
        } else if (distance_to_go < 0) {
            Current_Target -= Step_Speed; // // 兔子往反方向跑
            if(Current_Target < Final_Target) Current_Target = Final_Target;
        }
              // 2. PID 控制：死死咬住这只兔子  
        PosPID.Error = Current_Target - PosPID.Actual; 
        
        PosPID.Integral +=  PosPID.Error; 
        
           // 积分限幅防暴走(非常关键)
        if(PosPID.Integral > 10000) PosPID.Integral = 10000;
        if(PosPID.Integral < -10000) PosPID.Integral = -10000;
        
        PosPID.Out = (PosPID.Kp * PosPID.Error) + 
                     (PosPID.Ki * PosPID.Integral) + 
                     (PosPID.Kd * (PosPID.Error - PosPID.LastError));
        PosPID.LastError = PosPID.Error;
                
      
       
      
          
          // 定义一个死区补偿值（这个值需要你实验，刚好能让电机极其缓慢转动的 PWM 值）
            #define DEADZONE_BIAS 40 
            #define ERROR_THRESHOLD 10  // 误差容忍度，小于 2 个脉冲就不管了，防止震荡

            // ... 之前的 PID 计算代码 ...
            int pwm_output = (int)PosPID.Out;

            if (PosPID.Error > ERROR_THRESHOLD) {
                pwm_output += DEADZONE_BIAS; // 正向补偿
            } else if (PosPID.Error < -ERROR_THRESHOLD) {
                pwm_output -= DEADZONE_BIAS; // 反向补偿
            } else {
                pwm_output = 0; // 到位了，彻底关断，防止嗡嗡响
            }
//        

        
           if (pwm_output >= 0) {
            // ��ת�߼���PB12(IN1) = 1, PB13(IN2) = 0
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            
            if(pwm_output >500) {pwm_output =500;} // PWM 上限限制
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm_output); // 写入 PF9
        } else {
            // ��ת�߼���PB12(IN1) = 0, PB13(IN2) = 1
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            
            pwm_output = -pwm_output; // 取绝对值
            if(pwm_output > 500) {pwm_output =500; }
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm_output); //写入 PF9
        }
         // 4. 打印格式化数据，供 SerialPlot 绘图
        printf("%.1f,%.1f,%.1f\n", Final_Target, Current_Target, PosPID.Actual);

 
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
