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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @details state machine states used to program a new parking distance
 */
typedef enum _ConfigState_e
{
	eConfigState_Idle,
	eConfigState_ButtonPressed,
	eConfigState_NewDistance,
	eConfigState_ProgramDistance,
	eConfigState_Done,
}eConfigState_t;

/**
 * Structure of state machine control states used of when a new parking
 * distance is being requested by the user.
 */
typedef struct _UserState_s
{
	uint32_t buttonCnt; //< delay count
	eConfigState_t state; //< what the program state is
	uint32_t distance; //< distance to program
}sUserState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DISTANCE (1200)
#define DISTANCE_MARGINE (100)
#define FLASH_SECTOR_START_ADDRESS (0x800c000)
#define TIME_ARRAY_SIZE (4) //< number of samples to avg for better distance

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
static uint32_t tl_TimestampIdx = 0;
static uint32_t tl_TimeArraySum = 0;
static uint32_t tl_AvgTime;
static uint32_t tl_StartTime;
static uint32_t tl_EndTime;
static uint32_t tl_DeltaTime;
static uint32_t tl_Distance;
static uint32_t tl_MeasureDistance = DISTANCE;

static uint8_t tl_DistanceReadyFlag = 0;
static sUserState_t tl_UserConfig = {.buttonCnt=0,.state=eConfigState_Idle};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/**
 * GPIO pin for timing/testing
 */
static void SetDebug1()
{
	HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_SET);
}

/**
 * GPIO pin for timing/testing
 */
static void ClrDebug1()
{
	HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_RESET);
}


/**
 * GPIO pin for timing/testing
 */
static void SetDebug2()
{
	HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, GPIO_PIN_SET);
}

/**
 * GPIO pin for timing/testing
 */
static void ClrDebug2()
{
	HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, GPIO_PIN_RESET);
}

/**
 * GPIO pin for timing/testing
 */
static void SetDebug3()
{
	HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, GPIO_PIN_SET);
}

/**
 * GPIO pin for timing/testing
 */
static void ClrDebug3()
{
	HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, GPIO_PIN_RESET);
}

/**
 * GPIO pin for timing/testing
 */
static void SetDebug4()
{
	HAL_GPIO_WritePin(DEBUG4_GPIO_Port, DEBUG4_Pin, GPIO_PIN_SET);
}

/**
 * GPIO pin for timing/testing
 */
static void ClrDebug4()
{
	HAL_GPIO_WritePin(DEBUG4_GPIO_Port, DEBUG4_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Set the Trigger GPIO
 */
static void SetTrigger()
{
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
}

/**
 * @brief Clear the Trigger GPIO
 */
static void ClrTrigger()
{
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Set the Blue LED
 */
static void SetBlueLED()
{
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
}

/**
 * @brief Clear the Blue LED
 */
static void ClrBlueLED()
{
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Toggle the Blue LED
 */
static void ToggleBlueLED()
{
	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}

/**
 * @brief Set the Green LED
 */
static void SetGreenLED()
{
	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
}

/**
 * @brief Clear the Green LED
 */
static void ClrGreenLED()
{
	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Toggle the Green LED
 */
static void ToggleGreenLED()
{
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}

///**
// * @brief Set the Yellow LED
// */
//static void SetYellowLED()
//{
//	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
//}

/**
 * @brief Clear the Yellow LED
 */
static void ClrYellowLED()
{
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Toggle the Yellow LED
 */
static void ToggleYellowLED()
{
	HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
}

/**
 * @brief Toggle buzzer output
 */
static void ToggleBuzzer()
{
	HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
}

/**
 * @brief Turn Buzzer OFF
 */
static void BuzzerOFF()
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Turn Buzzer ON
 */
static void BuzzerON()
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

/**
 * @brief Read the user button press
 */
static uint8_t ReadButton()
{
	uint8_t buttonFlag = 0;
	if( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) )
	{
		buttonFlag = 1;
	}
	return buttonFlag;
}


/**
 * @brief State machine to program new parking distance to FLASH
 */
static void SetUserConfig(sUserState_t *ptrConfig)
{
#define PRE_BUTTON_PRESSED_TIME  (2) // number of counts before we deem the button is pressed
#define BUTTON_PRESSED_TIME (6)
#define DEFINE_NEW_DISTANCE (4)

	switch(ptrConfig->state)
	{
	case eConfigState_Idle:
		SetDebug1();
		ClrDebug1();
		if( ptrConfig->buttonCnt < PRE_BUTTON_PRESSED_TIME )
		{
			ptrConfig->buttonCnt++;
		}
		else
		{
			ptrConfig->buttonCnt=0;
			ptrConfig->state = eConfigState_ButtonPressed;
			ClrBlueLED();
			ClrGreenLED();
			ptrConfig->distance = 0;
		}
		break;
	case eConfigState_ButtonPressed:
		SetDebug2();
		ClrDebug2();
		ToggleYellowLED();
		if( ptrConfig->buttonCnt < BUTTON_PRESSED_TIME )
		{
			ptrConfig->buttonCnt++;
		}
		else
		{
			ptrConfig->buttonCnt=0;
			ptrConfig->state = eConfigState_NewDistance;
			ClrYellowLED();
		}
		break;
	case eConfigState_NewDistance:
		SetDebug3();
		ClrDebug3();
		ToggleBlueLED();
		ToggleGreenLED();
		if( ptrConfig->buttonCnt < DEFINE_NEW_DISTANCE )
		{
			ptrConfig->buttonCnt++;
			ptrConfig->distance += tl_Distance;
		}
		else
		{
			ptrConfig->buttonCnt=0;
			ptrConfig->distance = ptrConfig->distance / DEFINE_NEW_DISTANCE;
			ptrConfig->state = eConfigState_ProgramDistance;
		}
		break;
	case eConfigState_ProgramDistance:
		SetDebug4();
		ClrDebug4();
		ptrConfig->state = eConfigState_Done;
		break;
	case eConfigState_Done:
		SetDebug4();
		ClrDebug4();
		break;
	}
}

/**
 * @brief API to read word of data from FLASH
 */
static void FlashReadData (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{
		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 4;
		RxBuf++;
		if (!(numberofwords--))
			break;
	}
}

/**
 * @brief Read the parking distance from FLASH
 */
static uint32_t ReadDistanceFromFlash()
{
	uint32_t distance = 0;

	FlashReadData (FLASH_SECTOR_START_ADDRESS,&distance,1);

	if( distance > (20*12*100) )
		distance = DISTANCE;
	return distance;
}

/**
 * @brief program parking distance word to FLASH
 */
static uint32_t ProgramDistanceToFlash(uint32_t NewDistance)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;

	uint32_t errorMsg = 0;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_SECTOR_START_ADDRESS;
	EraseInitStruct.NbPages     = 1;

	HAL_FLASH_Unlock();

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &errorMsg) != HAL_OK)
	{
		/*Error occurred while page erase.*/
		errorMsg = HAL_FLASH_GetError ();
	}

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD
			, FLASH_SECTOR_START_ADDRESS
			, NewDistance)
			!= HAL_OK)
	{
		/*Error occurred while page erase.*/
		errorMsg = HAL_FLASH_GetError ();
	}

	HAL_FLASH_Lock();
	return errorMsg;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief TIM14 Callback handler - Trigger pulse generation
 *
 * @detail This callback will be called every 100ms from the
 * TIM14 timer.  This is needed as the HC-SR04 requires a short
 * trigger pulse to start a measurement.  According to the
 * HC-SR04 spec this pulse should not happen any faster than 60ms.
 * This method will also capture the time stamp to be used as part
 * of the HC-SR04 pulse distance pulse measurement.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance== TIM14)
	{
		tl_StartTime = TIM2->CNT;
		SetTrigger();
		ClrTrigger();
	}
}

/**
 * @brief Interrupt Callback to be called when PC7 detects a falling edge
 *
 * @detail This callback method will be called on External Interrupt with
 * a falling edge trigger assigned to Port C7 pin.
 * When a falling edge is detected, this code will read the 32bit counter TIM2
 * and determine how long the time pulse generated by the HC-SR04 is.  This
 * pulse size divided by 148 will provide inches.
 * Now there is a constant time from when the Trigger pulse is generated until
 * when the HC-SR04 pulse is started.  This constant time of 500uS is subtracted.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#define TIME_TO_PULSE (4000) // there is a constant time from start of pulse to subtract
#define CLOCK_RATE (8) // clock rate in MHz
#define TIME_TO_INCHES (148)
	if( tl_TimestampIdx++ < TIME_ARRAY_SIZE)
	{
		tl_EndTime = TIM2->CNT;

		if( tl_StartTime >  tl_EndTime )
		{
			tl_DeltaTime = (0xFFFFFFFF - tl_StartTime) + tl_EndTime;
		}
		else
		{
			tl_DeltaTime = tl_EndTime - tl_StartTime;
		}

		tl_TimeArraySum += tl_DeltaTime;

	}
	else
	{

		tl_AvgTime = tl_TimeArraySum / TIME_ARRAY_SIZE;
		tl_Distance = ((tl_AvgTime - TIME_TO_PULSE)*100)/ (CLOCK_RATE * TIME_TO_INCHES);
		tl_TimestampIdx = 0;
		tl_TimeArraySum = 0;
		tl_DistanceReadyFlag = 1;
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
  MX_TIM14_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  TIM1->CCR1 = 1;
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start(&htim2);


  tl_MeasureDistance = ReadDistanceFromFlash();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(tl_DistanceReadyFlag)
	  {
		  /*
		   * loop every half second to provide details
		   * to user
		   */
		  tl_DistanceReadyFlag = 0;
		  if (ReadButton() )
		  {
			  SetUserConfig( &tl_UserConfig);
			  if( tl_UserConfig.state > eConfigState_ProgramDistance )
			  {
				  tl_MeasureDistance = tl_UserConfig.distance;
				  ProgramDistanceToFlash(tl_MeasureDistance);
			  }
		  }
		  else
		  {
			  if( tl_UserConfig.state)
			  {

				  tl_UserConfig.buttonCnt = 0;
				  tl_UserConfig.state = eConfigState_Idle;
			  }

			  if( tl_Distance < (tl_MeasureDistance - DISTANCE_MARGINE) )
			  {
				  /*
				   * indicate that distance is too close
				   */
				  ClrGreenLED();
				  SetBlueLED();
				  BuzzerON();

			  }
			  else if(tl_Distance < (tl_MeasureDistance+ DISTANCE_MARGINE) )
			  {
				  /*
				   * indicate that distance is now within 1" of the programmed distance
				   */
				  SetBlueLED();
				  SetGreenLED();
				  ToggleBuzzer();
			  }
			  else
			  {
				  /*
				   * indicate that distance is too far away
				   */
				  BuzzerOFF();
				  ClrBlueLED();
				  SetGreenLED();
			  }
		  }
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 100;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 8000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_YELLOW_Pin|DEBUG4_Pin|DEBUG3_Pin|DEBUG2_Pin
                          |DEBUG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin|LED_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_YELLOW_Pin DEBUG4_Pin DEBUG3_Pin DEBUG2_Pin
                           DEBUG1_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin|DEBUG4_Pin|DEBUG3_Pin|DEBUG2_Pin
                          |DEBUG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_PULSE_Pin */
  GPIO_InitStruct.Pin = ECHO_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_BLUE_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin TRIGGER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
