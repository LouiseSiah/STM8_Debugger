/*
 * swim.c
 *
 *  Created on: Aug 15, 2017
 *      Author: user2
 */
#include "stm32f1xx_hal.h"
#include "hardwareInterface.h"
#include "swim.h"
extern uint16_t count;
extern uint32_t icBuffer[10];
extern uint32_t forcedLow_buffer[18];
TIM_HandleTypeDef htim3;
TIM_OC_InitTypeDef sConfig;
uint32_t header_buffer[10] = {};

//uint32_t buffer_array[32] = {};
//void configure_swim_out_array(uint32_t* array, uint8_t numOfBits, uint16_t sequenceInBin)
//{
//	uint32_t buffer_array[numOfBits * 2];// = {};
//	for(int i = 1; i < (numOfBits-1) * 2; i+=2 )
//	{
//		array[i] = LOW_SPEED_BIT_PRIOD;
//	}
//
//	for(int i = 0; i < numOfBits; i++ )
//	{
//		array[i*2] = (sequenceInBin << i) & 1;
//	}
//
//	array[numOfBits - 1] = LOW_SPEED_BIT_PRIOD + 1;
//	configureAndStart_OC_DMA(LOW_SPEED_BIT_PRIOD,  array,  (numOfBits*2));
//
////	return buffer_array;
//}




uint32_t swim_send_header(uint8_t command)
{

//		swim.prevState = SWIM_COMMAND_SRST;
//		swim.prevState = SWIM_COMMAND_ROTF;
//		swim.prevState = SWIM_COMMAND_WOTF;
	count++;
	if(command ==SWIM_SRST)
	{
		// 0, b2, b1, b0, pb
		// 0, 0,  0,  0,  0
		for(int i = 0; i < 5; i++)
		{
			header_buffer[i * 2] = LOW_SPEED_BIT_ZERO_PERIOD;
		}

		for(int i = 1; i < 8 ; i+=2 )
		{
			header_buffer[i] = LOW_SPEED_BIT_PRIOD;
		}


	}
	else if(command == SWIM_ROTF)
	{
		// 0, b2, b1, b0, pb
		// 0, 0,  0,  1,  1
		for(int i = 0; i < 3; i++)
		{
			header_buffer[i * 2] = LOW_SPEED_BIT_ZERO_PERIOD;
		}

		for(int i = 3; i < 5; i++)
		{
			header_buffer[i * 2] = LOW_SPEED_BIT_ONE_PERIOD;
		}

		for(int i = 1; i < 8 ; i+=2 )
		{
			header_buffer[i] = LOW_SPEED_BIT_PRIOD;
		}

	}
	else if(command == SWIM_WOTF)
	{
		// 0, b2, b1, b0, pb
		// 0, 0,  0,  1,  1
		for(int i = 0; i < 2; i++)
		{
			header_buffer[i * 2] = LOW_SPEED_BIT_ZERO_PERIOD;
		}

		for(int i = 2; i < 4; i++)
		{
			header_buffer[i * 2] = LOW_SPEED_BIT_ONE_PERIOD;
		}

		header_buffer[8] = LOW_SPEED_BIT_ZERO_PERIOD;

		for(int i = 1; i < 8 ; i+=2 )
		{
			header_buffer[i] = LOW_SPEED_BIT_PRIOD;
		}

	}
	else
	{
		return SWIM_CMD_UNDEFINED;
	}
	header_buffer[9] = LOW_SPEED_BIT_PRIOD + 1 ;
//	htim3.Instance = TIM3;
//	__HAL_TIM_DISABLE(&htim3);
	htim3.Instance->CR1 &= ~(TIM_CR1_CEN);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1| TIM_FLAG_CC2| TIM_FLAG_CC3| TIM_FLAG_CC4| TIM_FLAG_UPDATE);
	CLEAR_BIT(TIM3->DIER, TIM_DIER_CC1DE);

	configureAndStart_OC_DMA(LOW_SPEED_BIT_PRIOD, header_buffer, 10);
	swim.prevState = swim.currState;
	swim.currState = SWIM_LISTEN_ACK;

	return SWIM_OK;

}

void swim_send_One()
{
	header_buffer[0] = LOW_SPEED_BIT_ONE_PERIOD;
	header_buffer[1] = LOW_SPEED_BIT_PRIOD;
	configureAndStart_OC_DMA(LOW_SPEED_BIT_PRIOD, header_buffer, 2);
}

void swim_send_Zero()
{
	header_buffer[0] = LOW_SPEED_BIT_ZERO_PERIOD;
	header_buffer[1] = LOW_SPEED_BIT_PRIOD;
	configureAndStart_OC_DMA(LOW_SPEED_BIT_PRIOD, header_buffer, 2);
}

