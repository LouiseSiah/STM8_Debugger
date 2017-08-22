/*
 * hardwareInterface.h
 *
 *  Created on: Aug 22, 2017
 *      Author: user2
 */

#ifndef HARDWAREINTERFACE_H_
#define HARDWAREINTERFACE_H_

#define period_600us (uint16_t)(43200 - 1)
#define period_500us (uint16_t)((72000000/(1000 * 2)) - 1)
#define period_250us (uint16_t)((72000000/(2000 * 2)) - 1)
#define period_16us  (uint16_t)((72000000/62500 ) - 1)

#define LOW_SPEED_BIT_PRIOD (uint32_t)((SystemCoreClock / 361809) - 1)
#define LOW_SPEED_BIT_ONE_PERIOD 18 // 72M^-1 * 18 = 0.25us
#define LOW_SPEED_BIT_ZERO_PERIOD 180 // 72M^-1 * 180 = 2.5us

void setTimeout(uint16_t period_us);
void start_TIM2_CH2_IC_DMA( uint32_t *array,  uint16_t size);
void startSwimEntrySequence(void);
void configureAndStart_OC_DMA(uint16_t period,  uint32_t *array,  uint16_t size);
uint16_t getOCR(uint16_t arr, uint16_t currentOCR, uint16_t period_us);

#endif /* HARDWAREINTERFACE_H_ */
