/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMCLOCK 131062
#define PRESCALAR 16
#define BUFFER_SIZE 8
#define ENCODERS 4
#define ENCODERCHL1 0
#define ENCODERCHL2 1
#define ENCODERCHL3 2
#define ENCODERCHL4 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void capture_time(int channel, TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TX_Buffer[BUFFER_SIZE] = {0};
bool Edge_Captured[ENCODERS] = {false};
uint32_t Edge_Time1[ENCODERS] = {0};
uint32_t Edge_Time2[ENCODERS] = {0};
uint32_t TIM_CHANNEL[ENCODERS] = {TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};

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
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Turn on LED
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);

  // Start Timer 2 and ch1
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_SPI_Transmit(&hspi1, TX_Buffer, BUFFER_SIZE+1, 1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void capture_time(int chl,TIM_HandleTypeDef *htim)
{
	uint32_t Difference = 0;
	int temp = chl*2;

	if (Edge_Captured[chl] == false) // if the first rising edge is not captured
	{
		// Read the captured value from Capture Compare unit
		Edge_Time1[chl] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL[chl]); // read the first value
		Edge_Captured[chl] = true;  // set the first captured as true
	}
	else   // If the first rising edge is captured, now we will capture the second edge
	{
		Edge_Time2[chl] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL[chl]);  // read second value

		if (Edge_Time2[chl] > Edge_Time1[chl])
		{
			Difference = Edge_Time2[chl]-Edge_Time1[chl];
		}

		else if (Edge_Time1[chl] > Edge_Time2[chl])
		{
			Difference = (0xffff - Edge_Time1[chl]) + Edge_Time2[chl];
		}
	TX_Buffer[temp] = (Difference & 0xFF00) >> 8 ;
	TX_Buffer[temp+1] = (Difference & 0x00FF);

	Edge_Captured[chl] = false; // set it back to false

	}

}

// Hall sensor interrupt routine
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// check which encoder wheel was triggered
	switch(htim->Channel)
	{
	  case HAL_TIM_ACTIVE_CHANNEL_1 : // Front Right
		    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		    capture_time(ENCODERCHL1,htim);
		    /*
		    if (Edge_Captured[ENCODERCHL1] == false) // if the first rising edge is not captured
			{
				// Read the captured value from Capture Compare unit
				Edge_Time1[ENCODERCHL1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
				Edge_Captured[ENCODERCHL1] = true;  // set the first captured as true
			}
			else   // If the first rising edge is captured, now we will capture the second edge
			{
				Edge_Time2[ENCODERCHL1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

				if (Edge_Time2[ENCODERCHL1] > Edge_Time1[ENCODERCHL1])
				{
					Difference = Edge_Time2[ENCODERCHL1]-Edge_Time1[ENCODERCHL1];
				}

				else if (Edge_Time1[ENCODERCHL1] > Edge_Time2[ENCODERCHL1])
				{
					Difference = (0xffff - Edge_Time1[ENCODERCHL1]) + Edge_Time2[ENCODERCHL1];
				}

			TX_Buffer[0] = (Difference & 0xFF00) >> 8 ;
			TX_Buffer[1] = (Difference & 0x00FF);

			Edge_Captured[ENCODERCHL1] = false; // set it back to false

			}
			*/

		 break;

	  case HAL_TIM_ACTIVE_CHANNEL_2 : // Front Left
		  capture_time(ENCODERCHL2,htim);
		  /*
		  if (Edge_Captured[ENCODERCHL2] == false) // if the first rising edge is not captured
			{
				// Read the captured value from Capture Compare unit
				Edge_Time1[ENCODERCHL2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
				Edge_Captured[ENCODERCHL2] = true;  // set the first captured as true
			}
			else   // If the first rising edge is captured, now we will capture the second edge
			{
				Edge_Time2[ENCODERCHL2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value

				if (Edge_Time2[ENCODERCHL2] > Edge_Time1[ENCODERCHL2])
				{
					Difference = Edge_Time2[ENCODERCHL2]-Edge_Time1[ENCODERCHL2];
				}

				else if (Edge_Time1[ENCODERCHL2] > Edge_Time2[ENCODERCHL2])
				{
					Difference = (0xffff - Edge_Time1[ENCODERCHL2]) + Edge_Time2[ENCODERCHL2];
				}

			TX_Buffer[2] = (Difference & 0xFF00) >> 8 ;
			TX_Buffer[3] = (Difference & 0x00FF);

			Edge_Captured[ENCODERCHL2] = false; // set it back to false

			}
			*/
		 break;

	  case HAL_TIM_ACTIVE_CHANNEL_3 : // Rear Right
		  capture_time(ENCODERCHL3,htim);
		  /*
		  if (Edge_Captured[ENCODERCHL3] == false) // if the first rising edge is not captured
			{
				// Read the captured value from Capture Compare unit
				Edge_Time1[ENCODERCHL3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
				Edge_Captured[ENCODERCHL3] = true;  // set the first captured as true
			}
			else   // If the first rising edge is captured, now we will capture the second edge
			{
				Edge_Time2[ENCODERCHL3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value

				if (Edge_Time2[ENCODERCHL3] > Edge_Time1[ENCODERCHL3])
				{
					Difference = Edge_Time2[ENCODERCHL3]-Edge_Time1[ENCODERCHL3];
				}

				else if (Edge_Time1[ENCODERCHL3] > Edge_Time2[ENCODERCHL3])
				{
					Difference = (0xffff - Edge_Time1[ENCODERCHL3]) + Edge_Time2[ENCODERCHL3];
				}

			TX_Buffer[4] = (Difference & 0xFF00) >> 8 ;
			TX_Buffer[5] = (Difference & 0x00FF);

			Edge_Captured[ENCODERCHL3] = false; // set it back to false

			}
			*/
		 break;

	  case HAL_TIM_ACTIVE_CHANNEL_4 : // Rear Left
		  capture_time(ENCODERCHL4,htim);
		  /*
		  	if (Edge_Captured[ENCODERCHL4] == false) // if the first rising edge is not captured
			{
				// Read the captured value from Capture Compare unit
				Edge_Time1[ENCODERCHL4] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
				Edge_Captured[ENCODERCHL4] = true;  // set the first captured as true
			}
			else   // If the first rising edge is captured, now we will capture the second edge
			{
				Edge_Time2[ENCODERCHL4] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value

				if (Edge_Time2[ENCODERCHL4] > Edge_Time1[ENCODERCHL4])
				{
					Difference = Edge_Time2[ENCODERCHL4]-Edge_Time1[ENCODERCHL4];
				}

				else if (Edge_Time1[ENCODERCHL4] > Edge_Time2[ENCODERCHL4])
				{
					Difference = (0xffff - Edge_Time1[ENCODERCHL4]) + Edge_Time2[ENCODERCHL4];
				}

			TX_Buffer[6] = (Difference & 0xFF00) >> 8 ;
			TX_Buffer[7] = (Difference & 0x00FF);

			Edge_Captured[ENCODERCHL4] = false; // set it back to false

			}
			*/
		 break;

	  default :
		  break;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
