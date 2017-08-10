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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef TimHandle3;
TIM_HandleTypeDef TimHandle2;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;
TIM_IC_InitTypeDef sICConfig;

/* Capture Compare buffer */
uint32_t aCCValue_Buffer[6] = { 0, 0, 0, 0, 0, 0};//, 0 };
uint32_t icBuffer1[4] = {0, 0, 0, 0};
uint32_t icBuffer2[6] = {0, 0, 0, 0, 0, 0};

/* Timer Period*/
uint32_t uhTimerPeriod = 0;

uint32_t counter = 0;
uint32_t uhCapture = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int c = 0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM3_Init();


//  	htim3.Instance = TIM3;
////  	TIM3->CNT = 0;
//  	htim3.Init.Period = uhTimerPeriod;
//  	htim3.Init.RepetitionCounter = 0;
//  	htim3.Init.Prescaler = 0;
//  	htim3.Init.ClockDivision = 0;
//  	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
//  	{
//  		/* Initialization Error */
//  		Error_Handler();
//  	}
//
//  	sConfig.OCMode = TIM_OCMODE_TOGGLE;
//  	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
//  	sConfig.Pulse = 0; //aCCValue_Buffer[0];//0;//aCCValue_Buffer[0];
//  	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1) != HAL_OK)
//  	{
//  		Error_Handler();
//  	}
//
//  	if (HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, aCCValue_Buffer, 3) != HAL_OK)
//  	{
//  		Error_Handler();
//  	}

//  	HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_SET);

//  	HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_RESET);

//
//  	htim2.Instance = TIM2;
//  	htim2.Init.Period            = 0xFFFFFFFF;
//  	htim2.Init.Prescaler         = 0;
//  	htim2.Init.ClockDivision     = 0;
//  	htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
//  	htim2.Init.RepetitionCounter = 0;
//
//    if(HAL_TIM_IC_Init(&htim2) != HAL_OK)
//    {
//      Error_Handler();
//    }
////    HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_SET); //worked
//
//
//
//    /* Configure the Input Capture of channel 2 */
//    sICConfig.ICPolarity  = TIM_ICPOLARITY_FALLING;
//    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
//    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
//    sICConfig.ICFilter    = 0;

//    if(HAL_TIM_IC_ConfigChannel(&htim2, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
//    {
//      /* Configuration Error */
//      Error_Handler();
//    }

//    HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_RESET); //check here, ok
//
//	sICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
//	sICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
//	sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
//	sICConfig.ICFilter = 0;
//	if (HAL_TIM_IC_ConfigChannel(&htim2, &sICConfig, TIM_CHANNEL_2) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_SET); //check here, ok

//    if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)icBuffer1, 4) != HAL_OK)
//    {
//    	Error_Handler();
//    }
//    HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_RESET); //check here, ok
//    HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_SET); //check here, ok


//    sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
//    if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)icBuffer2, 4) != HAL_OK)
//    {
//         // Initialization Error - FAILS HERE DUE TO TIM2 HANDLE BEING BUSY
//    	Error_Handler();
//    }

#define BIT_PERIOD (uint32_t)((SystemCoreClock / 361809) - 1)
#define PERIOD_ONE 18 // 72M^-1 * 18 = 0.25us
#define PERIOD_ZERO 180 // 72M^-1 * 180 = 2.5us
  /* USER CODE BEGIN 2 */
  	uhTimerPeriod = (uint32_t) ((SystemCoreClock / 361809) - 1);
  	//minumum pulse to put is 12??
  	aCCValue_Buffer[0] =  PERIOD_ONE;//2;//8999;
  	aCCValue_Buffer[1] = BIT_PERIOD;//+18;//18+14;//18 + aCCValue_Buffer[0]; //(uint32_t) ((SystemCoreClock / 4000000) - 1 + aCCValue_Buffer[0]);
  	aCCValue_Buffer[2] = PERIOD_ZERO;//18 + aCCValue_Buffer[1];//(uint32_t) ((SystemCoreClock / 400000) - 1 + aCCValue_Buffer[1]);
  	aCCValue_Buffer[3] = BIT_PERIOD;//18 + aCCValue_Buffer[2];//17998;
  	aCCValue_Buffer[4] = PERIOD_ZERO;//18 + aCCValue_Buffer[1];//(uint32_t) ((SystemCoreClock / 400000) - 1 + aCCValue_Buffer[1]);
  	aCCValue_Buffer[5] = BIT_PERIOD+1;//18 + aCCValue_Buffer[2];//17998;

  	htim3.Instance = TIM3;
  	htim3.Init.Period = BIT_PERIOD;// 14;
  	htim3.Init.RepetitionCounter = 0;
  	htim3.Init.Prescaler = 0;
  	htim3.Init.ClockDivision = 0;
  	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  	{
  		Error_Handler();
  	}

  	sConfig.OCMode = TIM_OCMODE_TOGGLE;
  	sConfig.OCPolarity = TIM_OCPOLARITY_LOW;
  	sConfig.Pulse = 0;// uhTimerPeriod;// 0; //aCCValue_Buffer[0];//0;//aCCValue_Buffer[0];
  	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  	{
  		Error_Handler();
  	}

  	if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)icBuffer2, 6) != HAL_OK)
	{
		 // Initialization Error - FAILS HERE DUE TO TIM2 HANDLE BEING BUSY
		Error_Handler();
	}

 	if (HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, aCCValue_Buffer, 6) != HAL_OK)
  	{
  		Error_Handler();
  	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	  if (icBuffer1[0] != 0 || icBuffer2[0] != 0)
	  {

//		  HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_RESET);
	  	  c = 0;
	  }
	  else
	  {

//		  HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_RESET);
		  c = -1;
	  }
//					= {0, 0, 0, 0};
//	  uint32_t icBuffer2[4] = {0, 0, 0, 0};

//	  if(icBuffer1 > 0)
//		  c--;
//	  else
//		  c++;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : test_Pin */
  GPIO_InitStruct.Pin = test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(test_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	c = icBuffer2[0] + 1;
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
	  c--;
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
