/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

TIM_OC_InitTypeDef sConfig;
TIM_IC_InitTypeDef sICConfig;


#define OC_bufferSize	18
#define IC_bufferSize 	10
uint32_t forcedLow_buffer[OC_bufferSize] = {0,0,0,0};
uint32_t icBuffer[IC_bufferSize] = {};
uint16_t bufferCount = 0;
uint16_t testing = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//uint16_t getOCR(uint16_t arr, uint16_t currentOCR, uint16_t period_us);
void configure_entry_sequence();
//void configureAndStart_OC_DMA(uint16_t period,  uint32_t *array,  uint16_t size);

void setTimeout(uint16_t period_us);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define period_600us (uint16_t)(43200 - 1)
#define period_500us (uint16_t)((72000000/(1000 * 2)) - 1)
#define period_250us (uint16_t)((72000000/(2000 * 2)) - 1)
#define period_16us  (uint16_t)((72000000/62500 ) - 1)
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//  #define period_500us (uint16_t)((SystemCoreClock/(1000 * 2)) - 1)
//  #define period_250us (uint16_t)((SystemCoreClock/(2000 * 2)) - 1)


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  swim.currState = SWIM_DO_NOTHING;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(SWIM_RESET_OUT_GPIO_Port, SWIM_RESET_OUT_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  swim_send_header(SWIM_WOTF);
////
//
//
//  forcedLow_buffer[0] = period_16us * 1;
//  for(int i = 1; i < 9; i++)
//  {
//	  forcedLow_buffer[i] =  getOCR(period_600us, forcedLow_buffer[i-1], period_500us);
//  }
//
//  for(int i = 0; i < 8; i++)
//  {
//	  forcedLow_buffer[i+9] =  getOCR(period_600us, forcedLow_buffer[i+9-1], period_250us);
//  }
//
//  forcedLow_buffer[17] = period_600us+1;

//  configure_entry_sequence();
//  configureAndStart_OC_DMA(period_600us, forcedLow_buffer, OC_bufferSize);
//  setTimeout(6670);
//  swim.currState = SWIM_LISTEN_SYNCHRONIZATION;
//
//  if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, icBuffer, IC_bufferSize) != HAL_OK)
//  {
//	Error_Handler();
//  }

//  HAL_GPIO_WritePin(SWIM_RESET_OUT_GPIO_Port, SWIM_RESET_OUT_Pin, GPIO_PIN_RESET);

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = timeUp_period;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = longPeriod;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SWIM_RESET_OUT_GPIO_Port, SWIM_RESET_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : STM8_POWER_Pin */
  GPIO_InitStruct.Pin = STM8_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STM8_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SWIM_RESET_IN_Pin */
  GPIO_InitStruct.Pin = SWIM_RESET_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWIM_RESET_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SWIM_RESET_OUT_Pin */
  GPIO_InitStruct.Pin = SWIM_RESET_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SWIM_RESET_OUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void configure_entry_sequence()
{
	forcedLow_buffer[0] = period_16us * 1;
	for(int i = 1; i < 9; i++)
	{
	  forcedLow_buffer[i] =  getOCR(period_600us, forcedLow_buffer[i-1], period_500us);
	}

	for(int i = 0; i < 8; i++)
	{
	  forcedLow_buffer[i+9] =  getOCR(period_600us, forcedLow_buffer[i+9-1], period_250us);
	}

	forcedLow_buffer[17] = period_600us+1;
}

//void configureAndStart_OC_DMA(uint16_t period,  uint32_t *array,  uint16_t size)
//{
//	htim3.Instance = TIM3;
//	htim3.Init.Period = period;//(period_16us * 16);//period_500us;//0xFFFF;// 14;
//	htim3.Init.RepetitionCounter = 0;
//	htim3.Init.Prescaler = 0;
//	htim3.Init.ClockDivision = 0;
//	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
//	{
//	Error_Handler();
//	}
//
//	sConfig.OCMode = TIM_OCMODE_TOGGLE;
//	sConfig.OCPolarity = TIM_OCPOLARITY_LOW;
//	sConfig.Pulse = 0;// uhTimerPeriod;// 0; //aCCValue_Buffer[0];//0;//aCCValue_Buffer[0];
//	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1) != HAL_OK)
//	{
//	Error_Handler();
//	}
//
//	if (HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, array, size) != HAL_OK)
//	{
//	  Error_Handler();
//	}
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{

		bufferCount++;
		HAL_TIM_Base_Stop_IT(&htim1);
		if( icBuffer[9] >= 1130 && swim.currState == SWIM_LISTEN_SYNCHRONIZATION)
		{
			swim.prevState = SWIM_LISTEN_SYNCHRONIZATION;
			swim.currState = SWIM_ACTIVATION;
//			swim_send_Zero();
			setTimeout(1);
//			setTimeout(2000);
//			swim_send_header(SWIM_WOTF);
//			if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, icBuffer, 6) != HAL_OK)
////			{
//				Error_Handler();
//			}
		}
		else if(swim.currState == SWIM_LISTEN_ACK)
		{

		}

	}
}
//		if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, icBuffer2, IC_bufferSize) != HAL_OK)
//		{
//			Error_Handler();
//		}
//		if (HAL_TIM_Base_Stop(&htim1) != HAL_OK)
//		  {
//			  Error_Handler();
//		  }
//
//		  if (HAL_TIM_Base_Stop_IT(&htim1) != HAL_OK)
//		  {
//			  Error_Handler();
//		  }
//	else if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//	{
//		bufferCount--;
//	}
//}

/*
 *  didnt be called after OC complete sending.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		HAL_TIM_Base_Stop_IT(&htim1);
		if(swim.currState == SWIM_LISTEN_SYNCHRONIZATION)
		{
			bufferCount = 1000;
		}
		else if(swim.currState == SWIM_ACTIVATION)
		{
//			setTimeout(2);
			setTimeout(3000);
			swim.currState = SWIM_COMMAND_WOTF;
//			swim_send_Zero();
			swim_send_header(SWIM_WOTF);
			if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, icBuffer, 6) != HAL_OK)
			{
				Error_Handler();
			}
		}

		else if(swim.currState == SWIM_COMMAND_WOTF)
		{
//			bufferCount = 100;
//			swim.currState = SWIM_COMMAND_WOTF;
////			swim_send_Zero();
//			swim_send_header(SWIM_WOTF);
//			if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, icBuffer, 6) != HAL_OK)
//			{
//				Error_Handler();
//			}

		}
//		if (HAL_TIM_OC_Stop_DMA(&htim3, TIM_CHANNEL_1) != HAL_OK)
//		{
//		  Error_Handler();
//		}
	}
}

/*
 * period_us : minimum = 16us
 *
 */
void setTimeout(uint16_t period_us)
{
    HAL_TIM_Base_Stop_IT(&htim1);
	uint32_t frequency = (uint32_t)1000000 / period_us;
	uint32_t prescalerValue = (uint16_t)(SystemCoreClock / (frequency * 65536));
    uint32_t timerPeriod = (uint16_t) (SystemCoreClock / (frequency * (prescalerValue + 1))) - 1;
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	/* Set the Autoreload value */
	TIM1->ARR = (uint32_t)timerPeriod;

	  /* Set the Prescaler value */
	TIM1->PSC = (uint32_t)prescalerValue;
	HAL_TIM_Base_Start_IT(&htim1);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
//	bufferCount++;
//	if(bufferCount == 1)
//		swim_send_Zero();
//	else if(bufferCount == 2)
//		swim_send_Zero();
//	else
	HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STM8_POWER_GPIO_Port, STM8_POWER_Pin, GPIO_PIN_RESET);

	swim_send_header(SWIM_WOTF);
//	static int j = 0;
//	if(j++ == 0)
//	{
////		HAL_Delay(1);
//		swim_send_header(SWIM_WOTF);
//	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
