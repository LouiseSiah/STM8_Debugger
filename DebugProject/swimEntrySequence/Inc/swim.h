/*
 * swim.h
 *
 *  Created on: Aug 15, 2017
 *      Author: user2
 */

#include <stdint.h>
#ifndef SWIM_H_
#define SWIM_H_

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
  SWIM_START,
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

typedef enum
{
	TIMER_EXPIRED,
	INPUT_CAPTURED,
}Event;

Swim swim;


void configure_swim_out_array(uint32_t* array, uint8_t numOfBits, uint16_t sequenceInBin);
uint32_t swim_send_header(uint8_t command);
uint32_t swim_send_byte(uint8_t byte);

void swim_send_One();
void swim_send_Zero();

#endif /* SWIM_H_ */
