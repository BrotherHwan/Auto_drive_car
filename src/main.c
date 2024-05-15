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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARRAYNUM 20
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId myTask01Handle;
/* USER CODE BEGIN PV */

uint16_t IC_Val1 = 0;
uint16_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint32_t Distance  = 0;

uint16_t IC_Val1_2 = 0;
uint16_t IC_Val2_2 = 0;
uint32_t Difference2 = 0;
uint8_t Is_First_Captured2 = 0;
uint32_t Distance2  = 0;

uint16_t IC_Val1_3 = 0;
uint16_t IC_Val2_3 = 0;
uint32_t Difference3 = 0;
uint8_t Is_First_Captured3 = 0;
uint32_t Distance3  = 0;

//Bluetooth communication
uint8_t rx1_data;
uint8_t rx2_data;

uint8_t MODE = 0;

//moving average filter 1
uint32_t right_Distance;
uint32_t center_Distance;
uint32_t left_Distance;
uint32_t Distance_array[ARRAYNUM];
uint32_t Distance2_array[ARRAYNUM];
uint32_t Distance3_array[ARRAYNUM];

//moving average filter 2
int idx = 0;
int total = 0;
int total2 = 0;
int total3 = 0;
//int average = 0;
//int average2 = 0;
//int average3 = 0;

//low pass filter
float sensitivity = 0.2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void StartTask01(void const * argument);

/* USER CODE BEGIN PFP */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, &ch, 1, 10);
}

void CarForward()
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}

void CarLeft()
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}

void CarRight()
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
}

void CarBack()
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
}

void CarQuit()
{
 HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
 HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
 HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
 HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}


void Delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER (&htim2) < time);
}


void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while (__HAL_TIM_GET_COUNTER (&htim3) < time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
		}
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
	{
		if (Is_First_Captured2==0) // if the first value is not captured
		{
			IC_Val1_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
			Is_First_Captured2 = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured2==1)   // if the first is already captured
		{
			IC_Val2_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2_2 > IC_Val1_2)
			{
				Difference2 = IC_Val2_2-IC_Val1_2;
			}

			else if (IC_Val1_2 > IC_Val2_2)
			{
				Difference2 = (0xffff - IC_Val1_2) + IC_Val2_2;
			}

			Distance2 = Difference2 * .034/2;  // .034/2
			Is_First_Captured2 = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
		}
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel1
		{
			if (Is_First_Captured3==0) // if the first value is not captured
			{
				IC_Val1_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
				Is_First_Captured3 = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured3==1)   // if the first is already captured
			{
				IC_Val2_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				if (IC_Val2_3 > IC_Val1_3)
				{
					Difference3 = IC_Val2_3-IC_Val1_3;
				}

				else if (IC_Val1_3 > IC_Val2_3)
				{
					Difference3 = (0xffff - IC_Val1_3) + IC_Val2_3;
				}

				Distance3 = Difference3 * .034/2;
				Is_First_Captured3 = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC3);
			}
		}
}



void HCSR04_Read (GPIO_TypeDef *TRIGx_GPIO, uint16_t TRIGx_Pin, uint32_t TIM_IT_CCx)
{
	HAL_GPIO_WritePin(TRIGx_GPIO, TRIGx_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIGx_GPIO, TRIGx_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	//delay(2100);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CCx);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
		HAL_UART_Transmit(&huart1, &rx2_data, sizeof(rx2_data), 10);
		HAL_UART_Receive_IT(&huart2, &rx2_data, sizeof(rx2_data));
	}
	else if(huart->Instance==USART1)
	{
		HAL_UART_Transmit(&huart2, &rx1_data, sizeof(rx1_data), 10);
		HAL_UART_Receive_IT(&huart1, &rx1_data, sizeof(rx1_data));
	}
}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //motor control timmer
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  //ultrasonic echo capture sig timmer
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);

  //Delay timmer
  HAL_TIM_Base_Start(&htim2);

//  printf("access main");
//  uint8_t str[]="Hello\n\r";
//  uint8_t received = '\0';

//  HAL_StatusTypeDef rcvStat;


  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask01 */
  osThreadDef(myTask01, StartTask01, osPriorityNormal, 0, 128);
  myTask01Handle = osThreadCreate(osThread(myTask01), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	 HCSR04_Read (TRIG_GPIO_Port, TRIG_Pin, TIM_IT_CC1);
//	 Delay(60000);
//
//	 HCSR04_Read (TRIG2_GPIO_Port, TRIG2_Pin, TIM_IT_CC2);
//	 Delay(60000);
//
//	 HCSR04_Read (TRIG3_GPIO_Port, TRIG3_Pin, TIM_IT_CC3);
//
//	 printf("Distance : %d cm\r\n",Distance);
//	 printf("Distance2 : %d cm\r\n",Distance2);
//	 printf("Distance3 : %d cm\r\n",Distance3);
//	 printf("============================\r\n");
//	 Delay(60000);
//
////Bluetooth Mode
//	 rcvStat = HAL_UART_Receive_IT(&huart1, &rx1_data, 1);
//	  if(rcvStat==HAL_OK)
//	  {
//		  HAL_UART_Transmit(&huart1, &rx1_data, 1, 1000);
//	  }
//
//	  //Manual mode
//	  if(rx1_data == 'm' || rx1_data == 'M')
//	  {
//		  MODE = 0;
//	  }
//
//	  if(MODE==0)
//	  	  {
//	  		  if(rx1_data == 'w'||rx1_data =='W')
//	  		  {
//	  			  CarForward();
//	  		  }
//	  		  if(rx1_data == 's'||rx1_data =='S')
//	  		  {
//	  			  CarBack();
//	  		  }
//	  		  if(rx1_data == 'a'||rx1_data =='A')
//	  		  {
//	  			  CarLeft();
//	  		  }
//	  		  if(rx1_data == 'd'||rx1_data =='D')
//	  		  {
//	  			  CarRight();
//	  		  }
//	  		  if(rx1_data == 'q'||rx1_data =='Q')
//	  		  {
//	  			  CarQuit();
//	  		  }
//
//	  		  htim4.Instance->CCR1 = 400;
//	  		  htim4.Instance->CCR3 = 400;
//	  	  }
//
//	  //Auto mode
//	  if(rx1_data == 'o' || rx1_data == 'O')
//	  {
//		  MODE = 1;
//	  }
//
//	  if(MODE==1)
//	  {
//		  htim4.Instance->CCR1 = 400;
//		  htim4.Instance->CCR3 = 400;
//		  CarForward();
//
//		  if(Distance<50||Distance2<50||Distance3<50)
//		  {
//			  htim4.Instance->CCR1 = 200;
//			  htim4.Instance->CCR3 = 200;
//		  }
//	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RIGHT_LIGHT_Pin|LEFT_LIGHT_Pin|EXTRA_OUT1_Pin|EXTRA_OUT2_Pin
                          |BRAKE_LIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin|IN3_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN4_Pin|TRIG2_Pin|TRIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_LIGHT_Pin LEFT_LIGHT_Pin EXTRA_OUT1_Pin EXTRA_OUT2_Pin
                           BRAKE_LIGHT_Pin */
  GPIO_InitStruct.Pin = RIGHT_LIGHT_Pin|LEFT_LIGHT_Pin|EXTRA_OUT1_Pin|EXTRA_OUT2_Pin
                          |BRAKE_LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin TRIG2_Pin TRIG3_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|TRIG2_Pin|TRIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
	 //Manual mode
	  //HAL_UART_Receive_IT(&huart1, &rx1_data, 1);
	  HAL_UART_Receive_DMA(&huart1, &rx1_data, sizeof(rx1_data));
	  if(rx1_data == 'm' || rx1_data == 'M')
	  {
		  MODE = 0;
	  }

	  if(MODE==0)
		  {
			  if(rx1_data == 'w'||rx1_data =='W')
			  {
				  CarForward();
			  }
			  if(rx1_data == 's'||rx1_data =='S')
			  {
				  CarBack();
			  }
			  if(rx1_data == 'a'||rx1_data =='A')
			  {
				  CarLeft();
			  }
			  if(rx1_data == 'd'||rx1_data =='D')
			  {
				  CarRight();
			  }
			  if(rx1_data == 'q'||rx1_data =='Q')
			  {
				  CarQuit();
			  }
			  htim4.Instance->CCR1 = 600;
			  htim4.Instance->CCR3 = 600;
		  }

	  //Auto mode
	  if(rx1_data == 'o' || rx1_data == 'O')
	  {
		  MODE = 1;
	  }

	  if(MODE==1)
	  {
		  if(Distance>=100)Distance = 100;
		  if(Distance2>=100)Distance2 = 100;
		  if(Distance3>=100)Distance3 = 100;

//		  //Moving average filter 1
//		  for(int i=0;i<ARRAYNUM-1;i++)
//		  {
//			  Distance_array[i] = Distance_array[i+1];
//			  Distance2_array[i] = Distance2_array[i+1];
//			  Distance3_array[i] = Distance3_array[i+1];
//		  }
//
//		  Distance_array[ARRAYNUM-1] = Distance;
//		  Distance2_array[ARRAYNUM-1] = Distance2;
//		  Distance3_array[ARRAYNUM-1] = Distance3;
//
//		  for(int i=0;i<ARRAYNUM;i++)
//		  {
//			  filtered_Distance += Distance_array[i];
//			  filtered_Distance2 += Distance2_array[i];
//			  filtered_Distance3 += Distance3_array[i];
//		  }
//		  filtered_Distance /= ARRAYNUM;
//		  filtered_Distance2 /= ARRAYNUM;
//		  filtered_Distance3 /= ARRAYNUM;


		  //Moving average filter 2
		  total = total - Distance_array[idx];
		  total2 = total2 - Distance2_array[idx];
		  total3 = total3 - Distance3_array[idx];

		  Distance_array[idx] = Distance;
		  Distance2_array[idx] = Distance2;
		  Distance3_array[idx] = Distance3;

		  total = total + Distance_array[idx];
		  total2 = total2 + Distance2_array[idx];
		  total3 = total3 + Distance3_array[idx];

		  idx = idx + 1;
		  if (idx >= ARRAYNUM)idx = 0;
		  right_Distance = total / ARRAYNUM;
		  center_Distance = total2 / ARRAYNUM;
		  left_Distance = total3 / ARRAYNUM;


//		  //Low pass filter
//		  right_Distance = filtered_Distance*(1-sensitivity) + Distance*sensitivity;
//		  center_Distance = filtered_Distance2*(1-sensitivity) + Distance2*sensitivity;
//		  left_Distance = filtered_Distance3*(1-sensitivity) + Distance3*sensitivity;

	//Car movement algorithm
		  //car start
		  htim4.Instance->CCR1 = 450; //left wheel
		  htim4.Instance->CCR3 = 450; //right wheel
		  CarForward();

		  //if center is too close
		  if(center_Distance<=15)
		  {
			  if(right_Distance>left_Distance)
			  {
				  CarRight();
				  htim4.Instance->CCR1 = 400;
				  htim4.Instance->CCR3 = 400;
			  }
			  if(right_Distance<left_Distance)
			  {
				  CarLeft();
				  htim4.Instance->CCR1 = 400;
				  htim4.Instance->CCR3 = 400;
			  }
			  if(center_Distance<=6)
			  {
				  CarBack();
				  htim4.Instance->CCR1 = 350;
				  htim4.Instance->CCR3 = 350;
			  }
		  }

		  //turn left
		  if(right_Distance<=25) //originals are 40,25,10   seconds are 30,20,10,5,8
		  {
			  CarLeft();
			  htim4.Instance->CCR1 = 500;
			  htim4.Instance->CCR3 = 500; //originals are 400, 500, 700
			  if(right_Distance<=20)
			  {
				  htim4.Instance->CCR1 = 600;
				  htim4.Instance->CCR3 = 600;
				  if(right_Distance<=10)
				  {
					  htim4.Instance->CCR1 = 700;
					  htim4.Instance->CCR3 = 700;
					  if(right_Distance<=5)
					  {
						  htim4.Instance->CCR1 = 350;
						  htim4.Instance->CCR3 = 350;
						  CarBack();
					  }
				  }
			  }
		  }

		  //turn right
		  if(left_Distance<=25)
		  {
			  CarRight();
			  htim4.Instance->CCR1 = 500;
			  htim4.Instance->CCR3 = 500;
			  if(left_Distance<=20)
			  {
				  htim4.Instance->CCR1 = 600;
				  htim4.Instance->CCR3 = 600;
				  if(left_Distance<=10)
				  {
					  htim4.Instance->CCR1 = 700;
					  htim4.Instance->CCR3 = 700;
					  if(left_Distance<=5)
					  {
						  htim4.Instance->CCR1 = 350;
						  htim4.Instance->CCR3 = 350;
						  CarBack();
					  }
				  }
			  }
		  }

	  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask01 */
/**
* @brief Function implementing the myTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask01 */
void StartTask01(void const * argument)
{
  /* USER CODE BEGIN StartTask01 */
  /* Infinite loop */
  for(;;)
  {
	 HCSR04_Read (TRIG_GPIO_Port, TRIG_Pin, TIM_IT_CC1);
	 Delay(25000);
	 HCSR04_Read (TRIG2_GPIO_Port, TRIG2_Pin, TIM_IT_CC2);
	 Delay(25000);
	 HCSR04_Read (TRIG3_GPIO_Port, TRIG3_Pin, TIM_IT_CC3);
	 Delay(25000);
    osDelay(1);
  }
  /* USER CODE END StartTask01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
