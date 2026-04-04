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
int target_steps = 3200;      // 总共要走的步数
int current_step = 0;         // 已经走了的步数 (注意：昨天我们是倒着减，今天为了分段，我们正着加)

int accel_steps = 800;        // 用多少步来加速
int decel_steps = 800;        // 用多少步来减速

uint16_t arr_current = 2000;  // 起步时的 ARR (很大，意味着起步很慢)
uint16_t arr_min = 500;       // 巡航时的最小 ARR (速度最快)
uint16_t arr_step = 2;        // 每次进入中断，ARR 改变的步长 (比如每次减2

int is_motor_running = 0;  // 电机运行标志位：0代表停止，1代表正在运行
#define TABLE_LEN 800
const uint16_t S_Curve_Table[800] = {
    1990, 1990, 1990, 1990, 1989, 1989, 1989, 1989, 1989, 1989,
    1989, 1988, 1988, 1988, 1988, 1988, 1988, 1988, 1987, 1987,
    1987, 1987, 1987, 1987, 1986, 1986, 1986, 1986, 1986, 1986,
    1985, 1985, 1985, 1985, 1985, 1984, 1984, 1984, 1984, 1984,
    1984, 1983, 1983, 1983, 1983, 1982, 1982, 1982, 1982, 1982,
    1981, 1981, 1981, 1981, 1980, 1980, 1980, 1980, 1979, 1979,
    1979, 1979, 1978, 1978, 1978, 1978, 1977, 1977, 1977, 1976,
    1976, 1976, 1976, 1975, 1975, 1975, 1974, 1974, 1974, 1973,
    1973, 1973, 1972, 1972, 1972, 1971, 1971, 1971, 1970, 1970,
    1969, 1969, 1969, 1968, 1968, 1968, 1967, 1967, 1966, 1966,
    1965, 1965, 1965, 1964, 1964, 1963, 1963, 1962, 1962, 1961,
    1961, 1961, 1960, 1960, 1959, 1959, 1958, 1958, 1957, 1956,
    1956, 1955, 1955, 1954, 1954, 1953, 1953, 1952, 1951, 1951,
    1950, 1950, 1949, 1948, 1948, 1947, 1947, 1946, 1945, 1945,
    1944, 1943, 1943, 1942, 1941, 1940, 1940, 1939, 1938, 1937,
    1937, 1936, 1935, 1934, 1934, 1933, 1932, 1931, 1930, 1930,
    1929, 1928, 1927, 1926, 1925, 1924, 1923, 1922, 1922, 1921,
    1920, 1919, 1918, 1917, 1916, 1915, 1914, 1913, 1912, 1911,
    1910, 1909, 1907, 1906, 1905, 1904, 1903, 1902, 1901, 1900,
    1898, 1897, 1896, 1895, 1894, 1892, 1891, 1890, 1888, 1887,
    1886, 1885, 1883, 1882, 1880, 1879, 1878, 1876, 1875, 1873,
    1872, 1870, 1869, 1868, 1866, 1864, 1863, 1861, 1860, 1858,
    1857, 1855, 1853, 1852, 1850, 1848, 1846, 1845, 1843, 1841,
    1839, 1838, 1836, 1834, 1832, 1830, 1828, 1826, 1825, 1823,
    1821, 1819, 1817, 1815, 1813, 1810, 1808, 1806, 1804, 1802,
    1800, 1798, 1795, 1793, 1791, 1789, 1786, 1784, 1782, 1780,
    1777, 1775, 1772, 1770, 1767, 1765, 1763, 1760, 1757, 1755,
    1752, 1750, 1747, 1744, 1742, 1739, 1736, 1734, 1731, 1728,
    1725, 1723, 1720, 1717, 1714, 1711, 1708, 1705, 1702, 1699,
    1696, 1693, 1690, 1687, 1684, 1681, 1678, 1674, 1671, 1668,
    1665, 1661, 1658, 1655, 1652, 1648, 1645, 1641, 1638, 1634,
    1631, 1628, 1624, 1620, 1617, 1613, 1610, 1606, 1602, 1599,
    1595, 1591, 1588, 1584, 1580, 1576, 1573, 1569, 1565, 1561,
    1557, 1553, 1549, 1545, 1541, 1537, 1533, 1529, 1525, 1521,
    1517, 1513, 1509, 1505, 1500, 1496, 1492, 1488, 1484, 1479,
    1475, 1471, 1467, 1462, 1458, 1454, 1449, 1445, 1441, 1436,
    1432, 1427, 1423, 1418, 1414, 1409, 1405, 1400, 1396, 1391,
    1387, 1382, 1378, 1373, 1369, 1364, 1360, 1355, 1350, 1346,
    1341, 1336, 1332, 1327, 1323, 1318, 1313, 1309, 1304, 1299,
    1295, 1290, 1285, 1280, 1276, 1271, 1266, 1262, 1257, 1252,
    1248, 1243, 1238, 1234, 1229, 1224, 1220, 1215, 1210, 1205,
    1201, 1196, 1191, 1187, 1182, 1177, 1173, 1168, 1164, 1159,
    1154, 1150, 1145, 1140, 1136, 1131, 1127, 1122, 1118, 1113,
    1109, 1104, 1100, 1095, 1091, 1086, 1082, 1077, 1073, 1068,
    1064, 1059, 1055, 1051, 1046, 1042, 1038, 1033, 1029, 1025,
    1021, 1016, 1012, 1008, 1004, 1000, 995, 991, 987, 983,
    979, 975, 971, 967, 963, 959, 955, 951, 947, 943,
    939, 935, 931, 927, 924, 920, 916, 912, 909, 905,
    901, 898, 894, 890, 887, 883, 880, 876, 872, 869,
    866, 862, 859, 855, 852, 848, 845, 842, 839, 835,
    832, 829, 826, 822, 819, 816, 813, 810, 807, 804,
    801, 798, 795, 792, 789, 786, 783, 780, 777, 775,
    772, 769, 766, 764, 761, 758, 756, 753, 750, 748,
    745, 743, 740, 737, 735, 733, 730, 728, 725, 723,
    720, 718, 716, 714, 711, 709, 707, 705, 702, 700,
    698, 696, 694, 692, 690, 687, 685, 683, 681, 679,
    677, 675, 674, 672, 670, 668, 666, 664, 662, 661,
    659, 657, 655, 654, 652, 650, 648, 647, 645, 643,
    642, 640, 639, 637, 636, 634, 632, 631, 630, 628,
    627, 625, 624, 622, 621, 620, 618, 617, 615, 614,
    613, 612, 610, 609, 608, 606, 605, 604, 603, 602,
    600, 599, 598, 597, 596, 595, 594, 593, 591, 590,
    589, 588, 587, 586, 585, 584, 583, 582, 581, 580,
    579, 578, 578, 577, 576, 575, 574, 573, 572, 571,
    570, 570, 569, 568, 567, 566, 566, 565, 564, 563,
    563, 562, 561, 560, 560, 559, 558, 557, 557, 556,
    555, 555, 554, 553, 553, 552, 552, 551, 550, 550,
    549, 549, 548, 547, 547, 546, 546, 545, 545, 544,
    544, 543, 542, 542, 541, 541, 540, 540, 539, 539,
    539, 538, 538, 537, 537, 536, 536, 535, 535, 535,
    534, 534, 533, 533, 532, 532, 532, 531, 531, 531,
    530, 530, 529, 529, 529, 528, 528, 528, 527, 527,
    527, 526, 526, 526, 525, 525, 525, 524, 524, 524,
    524, 523, 523, 523, 522, 522, 522, 522, 521, 521,
    521, 521, 520, 520, 520, 520, 519, 519, 519, 519,
    518, 518, 518, 518, 518, 517, 517, 517, 517, 516,
    516, 516, 516, 516, 516, 515, 515, 515, 515, 515,
    514, 514, 514, 514, 514, 514, 513, 513, 513, 513,
    513, 513, 512, 512, 512, 512, 512, 512, 512, 511,
    511, 511, 511, 511, 511, 511, 510, 510, 510, 510,
};

// 2. 新增一个“时间计步器”，用来记录加速区过了多少毫秒
int time_tick =0;

void Profile_Check(int target, int expected_accel, int expected_decel)
{
    if (target < (expected_accel + expected_decel))
    {
        accel_steps = target / 2;
        decel_steps = accel_steps;
    }
    else 
    {
        accel_steps = expected_accel;
        decel_steps = expected_decel;
    }
}
void Moter_Start(void)
   {
   Profile_Check(target_steps, 800, 800);
 
    // 1. 设置从定时器 (TIM3) 的目标步数 (3200步)
  
    __HAL_TIM_SET_AUTORELOAD(&htim3, target_steps - 1); 

    // 2. 清空 TIM3 的当前计数值，确保从 0 开始数
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    
    // 3.重置起步速度
    //arr_current = 2000;  
      
     time_tick =0;
     arr_current = S_Curve_Table[0]; // 起步速度直接查字典的第 0 页
       
    // 4. 设置主定时器 (TIM4) 的初始速度和占空比 (起步速度)
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr_current);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arr_current / 2);
       
      // 5.标志位点亮
    is_motor_running = 1;
       
    //6. 开启中断
    HAL_TIM_Base_Start_IT(&htim3);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
 
    HAL_TIM_Base_Start_IT(&htim6);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      Moter_Start();
      HAL_Delay(8000);
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
// 当定时器的一个周期跑完（即 TIM3 数到了 ARR 的值）时，自动进入这里
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 确认是 TIM3（计步器）发来的停火信号
    if (htim->Instance == TIM3)
    {
        // 停掉 TIM4 的 PWM
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        //停掉 TIM6 
        HAL_TIM_Base_Stop_IT(&htim6);
        // 停掉 TIM3 自己
        HAL_TIM_Base_Stop_IT(&htim3);
        
        // 标志位熄灭
        is_motor_running = 0; 
    }
    else if(htim->Instance == TIM6)
    {    
        // 标志位安检
        if (is_motor_running == 0) return;
        
        current_step =__HAL_TIM_GET_COUNTER(&htim3);
        
        if(current_step<=accel_steps)
        {
            if(time_tick<= (TABLE_LEN - 1))
            {arr_current = S_Curve_Table[time_tick];}
          //arr_current-=arr_step;
            
            time_tick +=1;
        __HAL_TIM_SET_AUTORELOAD(&htim4,arr_current);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,arr_current/2);
        }
                /* -------- 阶段 2：减速区 -------- */
        else if (current_step > (target_steps - decel_steps))
        {
            // 1. 算出距离终点还剩几步
            int remain_steps = target_steps - current_step;
            int index = remain_steps;
            //2. (防上限也防下限)
            if (index > TABLE_LEN - 1) index = TABLE_LEN - 1; 
            if (index < 0) index = 0; // 绝对不能查负数！
            
            // 3. 翻字典赋值！(没有任何加减法！)
            arr_current = S_Curve_Table[index];
            
            // 4. 压入寄存器，保持 50% 占空比
            __HAL_TIM_SET_AUTORELOAD(&htim4, arr_current);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arr_current / 2);
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
