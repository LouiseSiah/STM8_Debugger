/* USER CODE BEGIN 4 */

/*	@brief	-	To configure Timer period (1/ frequency) with the prescalerValue and timerPeriod.
 * 				Also configure the chosen channelx with OCmode_PWM1
 * 	@param	-	timx 		:	timer chosen = TIM1, TIM2, ...
 * 				frequency	: 	in kHz, minimum a 1kHz.
 * 				channelx	:	channel chosen to generate PWM = TIM_CHANNEL_1, TIM_CHANNEL_2, ...
 * 				dutyCycle	:	The period of 'High' level before going 'Low' (in %)
 * 								e.g. : duty cycle = 10 :
 * 																   ______________
 * 															______|
 * 															10%		90%
 *
 *	@retVal	-	NON-value
 */
void configureTimerFrequencyOC(TIM_TypeDef* timx, uint32_t frequency, uint32_t channelx, uint8_t dutyCycle)//, uint32_t ocMode, uint32_t ocPolarity, uint32_t ocFastMode, uint32_t ocNPolarity, uint32_t ocNIdleState, uint32_t ocIdleState)//, uint32_t period)
{
	TIM_HandleTypeDef    TimHandle;

	/* Timer Output Compare Configuration Structure declaration */
	TIM_OC_InitTypeDef sConfig;

   TimHandle.Instance = timx;
   uint16_t prescalerValue = (uint16_t)(SystemCoreClock / (frequency * 65536));
   uint16_t timerPeriod = (uint16_t) (SystemCoreClock / (frequency * (prescalerValue + 1))) - 1;

   TimHandle.Init.Prescaler         = prescalerValue;
   TimHandle.Init.Period            = timerPeriod;
   TimHandle.Init.ClockDivision     = 0;
   TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
   // TimHandle.Init.RepetitionCounter = 0;
   if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
   {
     /* Initialization Error */
     Error_Handler();
   }
   uint32_t pulse = (TimHandle.Init.Period - 1) * dutyCycle / 100;

	// sConfig.OCMode       = TIM_OCMODE_PWM1;
	// sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	// sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	// sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	// sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	// sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

   /* Set the pulse value for channel 1 */
   sConfig.Pulse = pulse;
   if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, channelx) != HAL_OK)
   {
     /* Configuration Error */
     Error_Handler();
   }

    if (HAL_TIM_OC_Start_IT(&TimHandle, channelx) != HAL_OK)
   {
     /* PWM Generation Error */
     Error_Handler();
   }

}