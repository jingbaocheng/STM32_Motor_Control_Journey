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
#include <stdlib.h>
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
float Max_Speed = 35.0f;         //兔子最高限速
float Acceleration = 0.5f;       // 子加速度/减速度

// PID 计算工厂：你给它【账本地址】和【当前误差】，它帮你算出【该出的力】
float PID_Calc(PID_TypeDef *pid, float error) 
    {
    // 1. 把最新的误差存进账本
    pid->Error = error;
    
    // 2. 累加积分（也就是把误差一点点攒起来）
    pid->Integral += pid->Error;
    
    // 3. 【核心公式】：比例 + 积分 + 微分
    // Out = Kp * 现在的误差 + Ki * 累积的误差 + Kd * (现在的误差 - 上次的误差)
    pid->Out = (pid->Kp * pid->Error) + 
               (pid->Ki * pid->Integral) + 
               (pid->Kd * (pid->Error - pid->LastError));
    
    // 4. 记住这次的误差，留给下一次（10ms后）计算微分项用
    pid->LastError = pid->Error;
    
    // 5. 把算好的结果吐出来
    return pid->Out;
}
// 硬件层封装：输入一个 -1000 到 1000 的数值，自动控制方向和速度
void Set_Motor_PWM(int pwm_value) 
    {
    // 1. 方向控制
    if (pwm_value >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // IN1 = 1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // IN2 = 0
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // IN1 = 0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // IN2 = 1
        pwm_value = -pwm_value; // 取绝对值，交给 PWM 寄存器
    }

    // 2. 限幅（防止超过定时器最大计数值）
    if (pwm_value > 1000) pwm_value = 1000; 

    // 3. 写入寄存器 (TIM14 CHANNEL 1)
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm_value);
}



 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 PosPID.Kp =0.1f; // 比例系数调小，防止起步猛冲
 PosPID.Ki = 0.01f; // 必须要有积分，消除稳态误差，防止哒哒哒震动
 PosPID.Kd =0.1f;

    // 速度环参数 (经理)：决定电机转得稳不稳
    SpeedPID.Kp = 2.0f; // 速度环 Kp 通常比位置环大很多
    SpeedPID.Ki = 0.1f;
    SpeedPID.Kd = 0.0f;
    
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
        // --- 1. 获取物理数据 ---
        int16_t delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim4); // 读取这 10ms 的脉冲差
        __HAL_TIM_SET_COUNTER(&htim4, 0);                       // 依然清零定时器防溢出
        CurrentPosition += delta;            
        
         // --- 2. 梯形轨迹规划 (兔子跑) ---
        float distance_to_go = Final_Target - Current_Target;// 距离 = 终点 - 兔子现在的位置
        // 物理学公式：刹车距离 = 速度的平方 / (2 * 加速度)
        float slow_down_distance = (Step_Speed * Step_Speed) / (2.0f * Acceleration); 
            
        // 加速或减速判断        
            if (fabs(distance_to_go) > slow_down_distance) {
        
            if (Step_Speed < Max_Speed) {
                Step_Speed += Acceleration; // 慢慢踩油门
            }
        } else {
            // 离终点不够远了，必须开始减速刹车！
            if (Step_Speed > 6.0f) { // // 保留一点最小速度防止卡死
                Step_Speed -= Acceleration; // 慢慢松油门
            }
        }

        /// // 更新兔子的当前位置 (注意方向)
        if (distance_to_go > 0) {
            Current_Target += Step_Speed;  // 兔子往正方向跑
            if(Current_Target > Final_Target) Current_Target = Final_Target;// 防止超车
        } else if (distance_to_go < 0) {
            Current_Target -= Step_Speed; // // 兔子往反方向跑
            if(Current_Target < Final_Target) Current_Target = Final_Target;
        }
              
                
        // ==================== 第三步：外环（位置环）CEO 发号施令 ====================
        // 计算“兔子”和“电机实际位置”的距离差
        float pos_error = (float)Current_Target - (float)CurrentPosition;
          // 【新增：容忍死区】如果误差小于 10 个脉冲（约 0.06 度），直接认为已经到了
        if (fabs(pos_error) < 10) 
            {
            pos_error = 0;
            PosPID.Integral = 0;   // 清空位置环积分，防止由于憋着劲导致的震荡
        }
        
        // 调用我们封装好的工厂，算出：为了追上兔子，现在【理想速度】应该是多少
        float target_speed = PID_Calc(&PosPID, pos_error); 
      
        // ==================== 第四步：内环（速度环）经理拼命执行 ====================
        // 计算“理想速度”和“这10ms实际跑的速度(delta)”的误差
        float speed_error = target_speed - (float)delta;
                
        // 【新增：静止保护】如果目标速度是 0 且实际也没动，清空速度环积分
        if (target_speed == 0 && delta == 0) 
            {
            SpeedPID.Integral = 0;
            }
                
        // 再次调用工厂，算出：为了达到理想速度，现在【PWM 动力】该给多少
        float final_pwm = PID_Calc(&SpeedPID, speed_error);
        
        
          
          
//         // --- 5. 最终输出与死区补偿 ---
//                // 【死区补偿】：如果算出来的力很小，直接给它一个刚好能转动的“推力”
//                if (final_pwm > 10) final_pwm += 40;      // 正向死区
//                else if (final_pwm < -10) final_pwm -= 40; // 反向死区
//                else final_pwm = 0;                      // 误差极小时彻底停下

                  // 1. 如果目标速度已经是 0，且电机已经基本停稳（delta 小于 2）
                    if (target_speed == 0 && abs(delta) < 2) {
                        final_pwm = 0;               // 强行熄火
                        PosPID.Integral = 0;         // 卸掉位置环的劲
                        SpeedPID.Integral = 0;       // 卸掉速度环的劲
                    } 
                    // 2. 只有当 PID 真的想用力（超过门槛）时，才加上死区补偿
                    else {
                        if (final_pwm > 10) {
                            final_pwm += 40;
                        } else if (final_pwm < -10) {
                            final_pwm -= 40;
                        } else {
                            // 如果 PID 算出的力在 -10 到 10 之间，说明它在犹豫
                            // 此时我们不加死区补偿，甚至可以直接让它输出 0，防止抖动
                            final_pwm = 0; 
                        }
                    }
            
                Set_Motor_PWM((int)final_pwm);
        //        

        // --- 6. 绘图监控 ---
        printf("%.1f,%.1f,%.1f\n", Final_Target, Current_Target, (float)CurrentPosition);

 
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
