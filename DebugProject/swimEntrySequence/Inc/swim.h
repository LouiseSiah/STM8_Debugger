/*
 * swim.h
 *
 *  Created on: Aug 15, 2017
 *      Author: user2
 */

#include <stdint.h>
#ifndef SWIM_H_
#define SWIM_H_

#define period_600us (uint16_t)(43200 - 1)
#define period_500us (uint16_t)((72000000/(1000 * 2)) - 1)
#define period_250us (uint16_t)((72000000/(2000 * 2)) - 1)
#define period_16us  (uint16_t)((72000000/62500 ) - 1)

#define LOW_SPEED_BIT_PRIOD (uint32_t)((SystemCoreClock / 361809) - 1)
#define LOW_SPEED_BIT_ONE_PERIOD 18 // 72M^-1 * 18 = 0.25us
#define LOW_SPEED_BIT_ZERO_PERIOD 180 // 72M^-1 * 180 = 2.5us

//#define SWIM_SRST       (uint8_t)0x00 //000
//#define SWIM_ROTF       (uint8_t)0x01 //001 b2 b1 b0 = 100?
//#define SWIM_WOTF       (uint8_t)0x02 //010
typedef enum
{
	SWIM_SRST, //      (uint8_t)0x00 //000
	SWIM_ROTF, //       (uint8_t)0x01 //001 b2 b1 b0 = 100?
	SWIM_WOTF, //(uint8_t)0x02 //010
}Cmd;


#define SWIM_CSR         0x7F80


#define SWIM_OK           	  0
#define SWIM_CMD_UNDEFINED	  1
//#define SWIM_ERR_SYNC         1
//#define SWIM_ERR_FIRST_WRITE  2
//#define SWIM_ERR_HEADER       3
//#define SWIM_ERR_HEADER_NACK  4
//#define SWIM_ERR_BWRITE       5
//#define SWIM_ERR_BWRITE_NACK  6
//#define SWIM_ERR_WRITE        9
//#define SWIM_ERR_READ         10
//#define SWIM_ERR_DELAY        11

typedef enum
{
  SWIM_DO_NOTHING,
  SWIM_LISTEN_SYNCHRONIZATION,
  SWIM_ACTIVATION,
  SWIM_COMMAND_SRST,
  SWIM_COMMAND_ROTF,
  SWIM_COMMAND_WOTF,
  SWIM_LISTEN_ACK,
}SwimStatus;

typedef struct
{
	SwimStatus prevState;
	SwimStatus currState;
}Swim;


Swim swim;


void configure_swim_out_array(uint32_t* array, uint8_t numOfBits, uint16_t sequenceInBin);
void configureAndStart_OC_DMA(uint16_t period,  uint32_t *array,  uint16_t size);
uint32_t swim_send_header(uint8_t command);
uint16_t getOCR(uint16_t arr, uint16_t currentOCR, uint16_t period_us);
void swim_send_One();
void swim_send_Zero();

#endif /* SWIM_H_ */
