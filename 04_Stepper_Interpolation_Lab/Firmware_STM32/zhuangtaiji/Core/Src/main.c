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
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include <stdlib.h>
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
// 1. printf 重定向，让单片机能向 SerialPlot 吐数据
// 【终极重定向】绕开 HAL 库死锁陷阱，直接暴力轰炸底层寄存器！
int fputc(int ch, FILE *f) {
    // 1. 死等，直到硬件发送通道(TXE)空闲
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
    
    // 2. 绕过所有 HAL 锁，直接把子弹塞进底层数据寄存器(DR)
    huart1.Instance->DR = (uint8_t)ch;
    
    return ch;
}
// 2. 定义状态机枚举 (为了让 SerialPlot 显示得更清楚，我把数值放大了)
typedef enum{

    MOTOR_IDLE =0,
    MOTOR_ACCEL =10,
   MOTOR_RUN =20,
    MOTOR_DECEL =30,
    MOTOR_STOP =40,

}MotorState_t;
typedef struct{
  MotorState_t State;       // 当前状态
    uint32_t LastTick;        // 时间戳
    uint32_t StepInterval;    // 脉冲间隔 (决定速度，越小越快)
    uint32_t CurrentStep;     // 当前步数
    uint32_t TargetStep;      // 目标步数

}StepperMotor_t;
StepperMotor_t MyMotor = {MOTOR_IDLE, 0, 50, 0, 0}; // 实例化，初始间隔 50ms
// 4. 模拟一个触发器 (为了今天能自动跑起来测试)
uint32_t Trigger_LastTick = 0; 

void Motor_Task_Run(void) {
 uint32_t NowTick = HAL_GetTick();
   // 【非阻塞核心】时间没到，直接退出，把 CPU 留给别人！
    if (NowTick - MyMotor.LastTick < MyMotor.StepInterval) {
        return; 
    }
     MyMotor.LastTick = NowTick; // 更新时间戳

 // 【状态机枢纽】
switch(MyMotor.State){
 case MOTOR_IDLE:
            // 待机状态：什么都不做，等待指令
            break;

 case MOTOR_ACCEL:
      // 加速阶段：让间隔越来越短，速度越来越快
     if (MyMotor.StepInterval > 5) { // 极速限制为 5ms 跑一步
           MyMotor.StepInterval--; 
            } 
     else {
                MyMotor.State = MOTOR_RUN;  // 加速完毕，进入匀速
           }
        MyMotor.CurrentStep++;
        break;

   case MOTOR_RUN:
      // 匀速阶段：保持 5ms 的间隔狂奔
         MyMotor.CurrentStep++;
            
      // 算一下还有多少步到终点？如果只剩 45 步，开始刹车！
        if (MyMotor.TargetStep - MyMotor.CurrentStep <= 45) { 
           MyMotor.State = MOTOR_DECEL;
        }
         break;
     case MOTOR_DECEL:
         // 减速阶段：让间隔越来越长，速度慢下来
      MyMotor.StepInterval++;
      MyMotor.CurrentStep++;
            
       if (MyMotor.CurrentStep >= MyMotor.TargetStep) {
         MyMotor.State = MOTOR_STOP;
           }
         break;

        case MOTOR_STOP:
            // 停车清算阶段
            MyMotor.CurrentStep = 0;
            MyMotor.TargetStep = 0;
            MyMotor.StepInterval = 50;  // 恢复初始慢速
            MyMotor.State = MOTOR_IDLE; // 回归待机
            break;
   
    }
     
        // 【新增消音器】：只有电机不在待机状态时，才输出波形数据
    if (MyMotor.State != MOTOR_IDLE) {
        printf("%d,%d,%d\n", MyMotor.State, MyMotor.CurrentStep, MyMotor.StepInterval);
    }

}

// ================== 上位机通信核心变量 ==================
volatile uint8_t Rx_Data;           // 听筒：每次只接 1 个字符
volatile uint8_t Rx_Buffer[50];    // 竹筐：把听到的字符一个个存起来，最大 50 个字
volatile uint8_t Rx_Index = 0;    // 计数器：记录竹筐里现在装了几个字了
volatile uint8_t Cmd_Ready = 0;    // 信号旗：如果是 1，说明听到了一整句话（遇到回车了）

// 串口接收中断回调函数（每次收到一个字符，单片机自动跑进这里）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) { // 确认是串口1听到的声音
        
        // 1. 判断是不是一句话说完了？（电脑串口助手点了发送，通常会带一个回车符 \n 或 \r）
        if (Rx_Data == '\n' || Rx_Data == '\r') {
            if (Rx_Index > 0) {
                Rx_Buffer[Rx_Index] = '\0'; // 关键！给这句话的末尾画个句号（添加字符串结束符）
                Cmd_Ready = 1;              // 举起信号旗：老板，一整句话接收完毕！可以去翻译了！
            }
        } 
        // 2. 话还没说完，继续往竹筐里装
        else {
            Rx_Buffer[Rx_Index] = Rx_Data;  // 把字放进竹筐
            Rx_Index++;                     // 位置往后挪一格
            
            // 防爆筐保护：如果发的字太多，清零重来，防止内存崩溃
            if (Rx_Index >= 50) Rx_Index = 0; 
        }
        
        // 3. 听完这个字，马上张开耳朵，准备听下一个字！(极其重要)
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_Data, 1);
    }
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
__HAL_UART_CLEAR_OREFLAG(&huart1); // 【神级护盾】开局强行清除溢出错误，防止自闭！
HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_Data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        // 1. 无脑疯狂调用电机任务！它绝对不会卡住死等
    Motor_Task_Run();
    
    // 2. 串口指令解析局
    if (Cmd_Ready == 1) { 
        
        // 判断暗号是不是 'X' 和 ':'
        if (Rx_Buffer[0] == 'X' && Rx_Buffer[1] == ':') {
            
            // 【手工打造底层解析器】提取冒号后面的数字
            int parsed_step = 0;
            int i = 2; 
            
            while (Rx_Buffer[i] >= '0' && Rx_Buffer[i] <= '9') {
                parsed_step = parsed_step * 10 + (Rx_Buffer[i] - '0');
                i++;
            }
            
            // 纯英文汇报！有了刚才完美的 fputc，现在用 printf 绝对不会死机！
            printf("[CMD OK] Target: %d\r\n", parsed_step);
            
            // 扣动扳机，驱动状态机！
            if (MyMotor.State == MOTOR_IDLE) {
                MyMotor.TargetStep = parsed_step;
                MyMotor.State = MOTOR_ACCEL; 
            } else {
                printf("[WARNING] Motor Busy!\r\n");
            }
        } else {
            // 暗号不对时的报错
            printf("[ERROR] Unknown CMD: %s\r\n", Rx_Buffer);
        }
        
        // 打扫战场，准备接下一句话
        Rx_Index = 0;
        Cmd_Ready = 0;
    }


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
