/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
    .name = "myTask02",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
    .name = "myTask03",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
    .name = "myTask04",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
    .name = "myTask05",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint32_t i = 1;

uint32_t self_test_flag = 2;

uint32_t Raedy_flag = 0;

uint32_t launch_prepara_Ready_flag = 0;

uint32_t zero_positon_flag = 0;

uint32_t Nearest_ball_position = 0;
uint32_t current_positon = 0;

uint32_t Uswitch_index = 0;
uint32_t total_number_balls = 0;

uint32_t launch_hold_time = 600;
uint32_t inflation_time = 5000;
uint32_t buffer_time = 500;
uint32_t launch_time = 500;

//uint32_t Uswitch_index = 0;
uint32_t laserSwitch1_index = 0;
uint32_t laserSwitch2_index = 0;
uint32_t HallSenser_index = 0;

uint32_t balls_index[12];

uint8_t UART_BUF_Uswitch_counter_buff[] = "index=00";
uint8_t UART_BUF_balls_line[12] = "000000000000";
uint8_t UART_BUF_balls_total_num[] = "00";

uint8_t UART_BUF_receive_data[1] = 0x00;

const uint8_t instruction_moto_on_1 = 0xf2;
const uint8_t instruction_moto_off_1 = 0xf3;

const uint8_t instruction_moto_on_2 = 0xf4;
const uint8_t instruction_moto_off_2 = 0xf5;

const uint8_t instruction_safe_relief_lock = 0xf6;
const uint8_t instruction_safe_relief_unlock = 0xf7;

const uint8_t instruction_launch1 = 0xf8;
const uint8_t instruction_launch2 = 0xf9;

RTC_TimeTypeDef Now_time;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask_blink(void *argument);
void StartTask02_masg(void *argument);
void StartTask03_relay_control(void *argument);
void StartTask04_stepMoto_control(void *argument);
void StartTask05_sensor_cap(void *argument);

/* USER CODE BEGIN PFP */
void startSystem_self_test(void);
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
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  startSystem_self_test();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask_blink, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02_masg, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03_relay_control, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04_stepMoto_control, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05_sensor_cap, NULL, &myTask05_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x15;
  sTime.Minutes = 0x44;
  sTime.Seconds = 0x50;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
  DateToUpdate.Month = RTC_MONTH_JUNE;
  DateToUpdate.Date = 0x12;
  DateToUpdate.Year = 0x21;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 72 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 500 - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  // HAL_UART_Receive_IT(&huart5, UART_BUF_receive_data, sizeof(UART_BUF_receive_data));
  HAL_UART_Receive_IT(&huart5, UART_BUF_receive_data, 1);
  /* USER CODE END UART5_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG4 PG5
                           PG6 PG7 PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG12 PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void startSystem_self_test(void) ///////////////////////////////////////////////////////////////////////////////////////////////////////self test
{
  while (self_test_flag)
  {
    TIM3->CCR1 = 25;                                        //stepPWM_GEN the duty will be increase that this var added
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);               //stipMoto frq
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (GPIO_PinState)1); //dir
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (GPIO_PinState)1); //enable wheel
  }
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  TIM5->CCR1 = 250; //the buzz pwm hight time us
  for (int i = total_number_balls; i > 0; i--)
  {
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); //buzz
    HAL_Delay(200);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_Delay(200);
  }

  HAL_UART_Transmit(&huart5, "self_test_OK!\n\r", 30, 10);
}

void Ball_count(void) ///////////////////////////////////////////////////////////////////////////////////////////ball count
{
  int count = 0;
  if (Uswitch_index % 2 == 0)
  {
    if (Uswitch_index == 6)
    {
      balls_index[Uswitch_index] = 0;
    }
    else
    {
      balls_index[Uswitch_index] = !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12); //laser switch2
    }
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, (GPIO_PinState)1); /////red led
  }
  else if (Uswitch_index % 2 == 1)
  {
    balls_index[Uswitch_index] = !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13); //laser switch1
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, (GPIO_PinState)1);             /////green led
  }

  for (int i = 0; i < (sizeof(balls_index) / sizeof(balls_index[0])); i++)
  {
    if (balls_index[i] == 1)
    {
      count++;
    }
  }

  total_number_balls = count;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //////////////////////////////////////////////////////////////////////GPIO exit
{
  if (GPIO_Pin == GPIO_PIN_11)
  {

    if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14) == 0) ///zero position is detected
    {
      Uswitch_index = 0;
      zero_positon_flag = 1;
      self_test_flag--;
      Nearest_ball_position = 0;
      current_positon = 0;
      HAL_UART_Transmit(&huart5, "zero position detected\n\r", 30, 10);
    }

    if (Uswitch_index > 11)
    {
      Uswitch_index = 0;
    }

    if (zero_positon_flag)
    {
      // UART_BUF_Uswitch_counter_buff[6] = (Uswitch_index / 10) + '0';
      // UART_BUF_Uswitch_counter_buff[7] = (Uswitch_index % 10) + '0';  /////20210617

      Ball_count();

      UART_BUF_balls_line[Uswitch_index] = balls_index[Uswitch_index] + '0';

      Uswitch_index += 1;

      //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, (GPIO_PinState)1); //Blue led
    }
  }

  if (launch_prepara_Ready_flag)
  {
    Nearest_ball_position--;
    current_positon--;
  }
}

void Launch_positioning_preparation(void)
{
  //Nearest_ball_position = 0;
  for (int i = 0; i < 11; i++)
  {
    if (balls_index[i] == 0)
    {
      Nearest_ball_position++; ///zero to ball
      current_positon = Nearest_ball_position - Uswitch_index;
    }
    else if (balls_index[i])
    {
      balls_index[i] = 0;
      goto goto_end;
    }
  }

goto_end:
  while (current_positon)
  {
    launch_prepara_Ready_flag = 1;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  }
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, (GPIO_PinState)1); //launch-1 on
  osDelay(launch_hold_time);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, (GPIO_PinState)0); //launch-1 off
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask_blink */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask_blink */
void StartDefaultTask_blink(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (GPIO_PinState)1);
    osDelay(80);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (GPIO_PinState)0);
    osDelay(100);
    osDelay(1);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, (GPIO_PinState)0); //bule led
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, (GPIO_PinState)0); //red led
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, (GPIO_PinState)0); //green led
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02_masg */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02_masg */
void StartTask02_masg(void *argument)
{
  /* USER CODE BEGIN StartTask02_masg */
  /* Infinite loop */

  for (;;)
  {

    UART_BUF_Uswitch_counter_buff[6] = (Uswitch_index / 10) + '0';
    UART_BUF_Uswitch_counter_buff[7] = (Uswitch_index % 10) + '0';

    HAL_UART_Transmit(&huart5, UART_BUF_Uswitch_counter_buff, sizeof(UART_BUF_Uswitch_counter_buff), 10); /////////////////////////////////////////////////////UART
    HAL_UART_Transmit(&huart5, "     ", 5, 10);
    HAL_UART_Transmit(&huart5, UART_BUF_balls_line, sizeof(UART_BUF_balls_line), 10);
    HAL_UART_Transmit(&huart5, "     ", 5, 10);
    if (total_number_balls < 10)
    {
      UART_BUF_balls_total_num[0] = 0 + '0';
      UART_BUF_balls_total_num[1] = total_number_balls + '0';
    }
    else if (total_number_balls >= 10)
    {
      UART_BUF_balls_total_num[0] = total_number_balls / 10 + '0';
      UART_BUF_balls_total_num[1] = total_number_balls % 10 + '0';
    }
    HAL_UART_Transmit(&huart5, "total=", 6, 10);
    HAL_UART_Transmit(&huart5, UART_BUF_balls_total_num, sizeof(UART_BUF_balls_total_num), 10);
    HAL_UART_Transmit(&huart5, "     ", 5, 10);
    HAL_UART_Transmit(&huart5, "Ready=", 6, 10);
    uint8_t UART_BUF_Raedy_flag[1] = '0';
    UART_BUF_Raedy_flag[0] = Raedy_flag + '0';
    HAL_UART_Transmit(&huart5, UART_BUF_Raedy_flag, sizeof(UART_BUF_Raedy_flag), 10);

    HAL_UART_Transmit(&huart5, "\n\r", 2, 10);
    osDelay(1000);
    osDelay(1);
  }
  /* USER CODE END StartTask02_masg */
}

/* USER CODE BEGIN Header_StartTask03_relay_control */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03_relay_control */
void StartTask03_relay_control(void *argument)
{
  /* USER CODE BEGIN StartTask03_relay_control */
  /* Infinite loop */
  for (;;) ////////////////////////////////////////////////////////////////////////////////////////relay control
  {
    osDelay(1);
    switch (UART_BUF_receive_data[0])
    {
    case instruction_safe_relief_lock:

      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, (GPIO_PinState)1); //safe-relief-lock
      UART_BUF_receive_data[0] = 0x00;
      break;

    case instruction_safe_relief_unlock:
      Raedy_flag = 0;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, (GPIO_PinState)0); //safe-relief-relief-unlock
      UART_BUF_receive_data[0] = 0x00;
      break;

    case instruction_launch1: ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////caution!!!!
      UART_BUF_receive_data[0] = 0x00;
      Launch_positioning_preparation();
      Raedy_flag = 0;
      break;

    case instruction_launch2:
      UART_BUF_receive_data[0] = 0x00;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, (GPIO_PinState)1); //launch-2 on
      osDelay(launch_hold_time);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, (GPIO_PinState)0); //launch-2 off
      Raedy_flag = 0;
      break;

    case instruction_moto_on_1:
      UART_BUF_receive_data[0] = 0x00;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, (GPIO_PinState)1); //moto-on 1  on
      osDelay(inflation_time);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, (GPIO_PinState)0); //moto-off 1 off
      Raedy_flag = 1;
      break;

    case instruction_moto_on_2:
      UART_BUF_receive_data[0] = 0x00;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, (GPIO_PinState)1); //moto-on 1  on
      osDelay(inflation_time);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, (GPIO_PinState)0); //moto-off 1 off
      Raedy_flag = 1;
      break;

    case instruction_moto_off_1:
      UART_BUF_receive_data[0] = 0x00;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, (GPIO_PinState)0); //moto-off 1 off
      break;

    case instruction_moto_off_2:
      UART_BUF_receive_data[0] = 0x00;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, (GPIO_PinState)0); //moto-off 2 off
      break;
    default:
    }
  }
  /* USER CODE END StartTask03_relay_control */
}

/* USER CODE BEGIN Header_StartTask04_stepMoto_control */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04_stepMoto_control */
void StartTask04_stepMoto_control(void *argument)
{
  /* USER CODE BEGIN StartTask04_stepMoto_control */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
    //TIM3->CCR1 = 25; //stepPWM_GEN the duty will be increase that this var added/////////////////////////////////////////////////////////////////////////////////////////step moto
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);               //stipMoto frq
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (GPIO_PinState)1); //dir
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (GPIO_PinState)1); //enable wheel
  }
  /* USER CODE END StartTask04_stepMoto_control */
}

/* USER CODE BEGIN Header_StartTask05_sensor_cap */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05_sensor_cap */
void StartTask05_sensor_cap(void *argument)
{
  /* USER CODE BEGIN StartTask05_sensor_cap */
  /* Infinite loop */
  for (;;)
  {

    laserSwitch2_index = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12); //laser switch2/////////////////////////////////////////////////////////////////////////////////////////sensor_cap
    laserSwitch1_index = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13); //laser switch1
    HallSenser_index = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14);   //Hall senser
    osDelay(1);
  }
  /* USER CODE END StartTask05_sensor_cap */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    HAL_UART_Receive_IT(&huart5, UART_BUF_receive_data, 1);
  }
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
