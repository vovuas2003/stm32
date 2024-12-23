/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PORT GPIOA
#define TRIG_PIN GPIO_PIN_9
#define ECHO_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8

#define CLK_PORT GPIOB
#define CLK_PIN GPIO_PIN_9
#define DIO_PORT GPIOB
#define DIO_PIN GPIO_PIN_8

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_1

#define BTN_PORT GPIOB
#define BTN_PIN GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void hc_sr04get2timers(uint16_t*, uint16_t*);

void tm1637DisplayDecimal(int);
void tm1637SetBrightness(char);
void _tm1637Start();
void _tm1637Stop();
void _tm1637ReadResult();
void _tm1637WriteByte(unsigned char);
void _tm1637DelayUsec(unsigned int);
void _tm1637ClkHigh();
void _tm1637ClkLow();
void _tm1637DioHigh();
void _tm1637DioLow();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  tm1637SetBrightness(8);
  uint16_t random, distance, initial_distance, delta; //not in cm, just timer counter
  uint16_t max_delta = 1000 * 5 * 2 / 340; //intrusion if delta from initial distance is more than 5mm
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    tm1637DisplayDecimal(3);
    HAL_Delay(999);
    tm1637DisplayDecimal(2);
    HAL_Delay(999);
    tm1637DisplayDecimal(1);
    HAL_Delay(999);
    hc_sr04get2timers(&random, &initial_distance);
    tm1637DisplayDecimal(random % 10000);
    while(1) {
        HAL_Delay(999);
        hc_sr04get2timers(&random, &distance);
        if(initial_distance > distance) {
            delta = initial_distance - distance;
        } else {
            delta = distance - initial_distance;
        }
        if(delta > max_delta) {
            tm1637DisplayDecimal(-1);
            break;
        }
        tm1637DisplayDecimal(random % 10000);
    }
    while(1) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(499);
        if(HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN)) {
            break;
        }
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void hc_sr04get2timers(uint16_t* timer_1, uint16_t* timer_2) {
    uint16_t timer_1_1, timer_1_2, timer_2_1, timer_2_2;
    uint32_t pMillis;
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < 10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    pMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 > HAL_GetTick());
    timer_1_1 = __HAL_TIM_GET_COUNTER (&htim1);
    timer_2_1 = __HAL_TIM_GET_COUNTER (&htim2);
    pMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
    timer_1_2 = __HAL_TIM_GET_COUNTER (&htim1);
    timer_2_2 = __HAL_TIM_GET_COUNTER (&htim2);
    if(timer_1_2 > timer_1_1) {
        *timer_1 = timer_1_2 - timer_1_1;
    } else {
        *timer_1 = timer_1_1 - timer_1_2;
    }
    if(timer_2_2 > timer_2_1) {
        *timer_2 = timer_2_2 - timer_2_1;
    } else {
        *timer_2 = timer_2_1 - timer_2_2;
    }
}

//lib for tm1637 display

const char segmentMap[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, // 0-9
    0x00, 0x40 // clear, -
};

void tm1637DisplayDecimal(int n)
{
    //if(n > 9999 || n < -999) return;
    int otr = 0;
    if (n < 0) {
        n *= -1;
        otr = 1;
    }

    unsigned char digitArr[4];
    digitArr[0] = segmentMap[n % 10];
    n /= 10;
    for (int i = 1; i < 4; ++i) {
        if (n % 10) {
        digitArr[i] = segmentMap[n % 10];
        } else {
            digitArr[i] = segmentMap[10];
        }
        n /= 10;
    }
    if (otr) {
        digitArr[3] = segmentMap[11];
    }

    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xc0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }

    _tm1637Stop();
}

void tm1637SetBrightness(char brightness)
{
    _tm1637Start();
    _tm1637WriteByte(0x87 + brightness);
    _tm1637ReadResult();
    _tm1637Stop();
}

void _tm1637Start(void)
{
    _tm1637ClkHigh();
    _tm1637DioHigh();
    _tm1637DelayUsec(20);
    _tm1637DioLow();
}

void _tm1637Stop(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(20);
    _tm1637DioLow();
    _tm1637DelayUsec(20);
    _tm1637ClkHigh();
    _tm1637DelayUsec(20);
    _tm1637DioHigh();
}

void _tm1637ReadResult(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(50);
    _tm1637ClkHigh();
    _tm1637DelayUsec(20);
    _tm1637ClkLow();
}

void _tm1637WriteByte(unsigned char b)
{
    for (int i = 0; i < 8; ++i) {
        _tm1637ClkLow();
        if (b & 0x01) {
            _tm1637DioHigh();
        } else {
            _tm1637DioLow();
        }
        _tm1637DelayUsec(30);
        b >>= 1;
        _tm1637ClkHigh();
        _tm1637DelayUsec(30);
    }
}

void _tm1637DelayUsec(unsigned int i)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while(__HAL_TIM_GET_COUNTER(&htim2) < i);
}

void _tm1637ClkHigh(void)
{
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
}

void _tm1637ClkLow(void)
{
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
}

void _tm1637DioHigh(void)
{
    HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_SET);
}

void _tm1637DioLow(void)
{
    HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_RESET);
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

#ifdef  USE_FULL_ASSERT
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
