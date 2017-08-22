/*
 * hardwareInterface.c
 *
 *  Created on: Aug 22, 2017
 *      Author: user2
 */
#include "stm32f1xx_hal.h"
#include "hardwareInterface.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_OC_InitTypeDef sConfig;
uint16_t count = 0;
uint32_t icBuffer[10];
uint32_t forcedLow_buffer[18];
/*
 * period_us : minimum = 16us
 * Set Timer1 in up counting mode and
 * configure the arr and prescaler.
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

void start_TIM2_CH2_IC_DMA( uint32_t *array,  uint16_t size)
{
  if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, array, size) != HAL_OK)
  {
    Error_Handler();
  }
}

void startSwimEntrySequence(void)
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

	//	  configure_entry_sequence();
	configureAndStart_OC_DMA(period_600us, forcedLow_buffer, 18);
	setTimeout(6670);
	start_TIM2_CH2_IC_DMA(icBuffer,  10);
}

void configureAndStart_OC_DMA(uint16_t period,  uint32_t *array,  uint16_t size)
{
	htim3.Instance = TIM3;
	htim3.Init.Period = period;//(period_16us * 16);//period_500us;//0xFFFF;// 14;
	htim3.Init.RepetitionCounter = 0;
	htim3.Init.Prescaler = 0;
	htim3.Init.ClockDivision = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.OCMode = TIM_OCMODE_TOGGLE;
	if(count%2 == 1)
		sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	else
		sConfig.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfig.Pulse = 0;// uhTimerPeriod;// 0; //aCCValue_Buffer[0];//0;//aCCValue_Buffer[0];
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, array, size) != HAL_OK)
	{
		Error_Handler();
	}
}

uint16_t getOCR(uint16_t arr, uint16_t currentOCR, uint16_t period_us)
{
	uint16_t ocr = 0;
	if((currentOCR + period_us) > arr)
	{
		ocr = (currentOCR + period_us) % arr;
	}
	else
	{
		ocr = currentOCR + period_us;
	}

	return ocr;
}

