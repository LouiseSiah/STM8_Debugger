#ifndef __SWIM_H__
#define __SWIM_H__

#include "stm32f10x_gpio.h"
#include "configuration.h"
#include "Timer.h"

#define SWIM_IN_PIN           GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10
#define SWIM_IN_PORT          GPIOB
#define SWIM_OUT_PIN          GPIO_Pin_8 
#define SWIM_OUT_PIN_PASSIVE  GPIO_Pin_11
#define SWIM_OUT_PORT         GPIOA
#define SWIM_RESET_IN_PIN     GPIO_Pin_5
#define SWIM_RESET_OUT_PIN    GPIO_Pin_6
#define SWIM_RESET_PORT       GPIOB

#define SWIM_RESET_Init()      configurePin(SWIM_RESET_PORT, SWIM_RESET_OUT_PIN, GPIO_Mode_Out_PP);\
                               configurePin(SWIM_RESET_PORT, SWIM_RESET_IN_PIN, GPIO_Mode_IN_FLOATING)
#define SWIM_RESET()           GPIO_ResetBits(SWIM_RESET_PORT, SWIM_RESET_OUT_PIN)
#define SWIM_RESET_DEASSERT()  GPIO_SetBits(SWIM_RESET_PORT, SWIM_RESET_OUT_PIN)
    
#define swimInInit()           configurePin(SWIM_IN_PORT, SWIM_IN_PIN, GPIO_Mode_IN_FLOATING)
#define swimOutInit()          configurePin(SWIM_OUT_PORT, SWIM_OUT_PIN, GPIO_Mode_AF_OD); \
                               configurePin(SWIM_OUT_PORT, SWIM_OUT_PIN_PASSIVE, GPIO_Mode_IN_FLOATING);


typedef enum
{
  SWIM_DO_NOTHING,
  SWIM_ACTIVATION,
  SWIM_LISTEN_SYNCHRONIZATION,
  SWIM_COMMAND_SRST,
  SWIM_COMMAND_ROTF,
  SWIM_COMMAND_WOTF,
  SWIM_COMMAND_ACK,
}SwimStatus;

typedef struct
{
  SwimStatus state;
  int counter;
  int cmdBitCounter;
}SwimState;



#define SWIM_OUT_PIN      GPIO_Pin_1
#define SWIM_OUT_PORT     GPIOC
#define RESET_PIN         GPIO_Pin_0
#define RESET_PORT        GPIOC
uint16_t swim_out_pin;	//pointers to redirect GPIO pins
GPIO_TypeDef* swim_out_port;
uint16_t reset_pin;
GPIO_TypeDef* reset_port;
TIM_TypeDef* tim;

TIM_TypeDef* tim2;	//TODO: remove
DMA_Channel_TypeDef* dma;

#define SWIM_SET()        swim_out_port->BSRR = swim_out_pin;
#define SWIM_RESET()      swim_out_port->BRR = swim_out_pin;
#define RESET_ASSERT()    GPIO_ResetBits(reset_port, reset_pin);
#define RESET_DEASSERT()  GPIO_SetBits(reset_port, reset_pin);

#define SWIM_NACK_MAX     100	//maximum number of NACKS in ROTF and WOTF
#define SWIM_DELAY_MAX    (1000*24)	//1ms
#define SWIM_BUF_SIZE     32	//input capture DMA buffer
#define SWIM_LONG_DELAY() {__NOP(); __NOP(); __NOP(); __NOP();}	//used for long bit delay

/* swim_data
* structure that holds data used globally in this file
* primary use is interfacing interrupt routine and main program
*/
struct
{
	uint32_t result;	//received bits
	uint32_t to_listen;	//expected bits, has to be less than SWIM_BUF_SIZE
	uint32_t done;	//true when job is done by interrupt routine
	uint16_t low[SWIM_BUF_SIZE];	//DMA target buffer
	uint32_t bit_sync;	//limit length of short pulse and 1, used when determining bit value
} swim_data;

#define SWIM_ERR_OK           0
#define SWIM_ERR_SYNC         1
#define SWIM_ERR_FIRST_WRITE  2
#define SWIM_ERR_HEADER       3
#define SWIM_ERR_HEADER_NACK  4
#define SWIM_ERR_BWRITE       5
#define SWIM_ERR_BWRITE_NACK  6
#define SWIM_ERR_WRITE        9
#define SWIM_ERR_READ         10
#define SWIM_ERR_DELAY        11

#define SWIM_SRST        0x00
#define SWIM_ROTF        0x03
#define SWIM_WOTF        0x05

#define SWIM_CSR               0x7F80
#define SWIM_FLASH             0x00505a
#define SWIM_PROGRAM           0x008000
#define SWIM_FLASH_CR1         (SWIM_FLASH + 0x00)
#define SWIM_FLASH_CR2         (SWIM_FLASH + 0x01)
#define SWIM_FLASH_NCR2        (SWIM_FLASH + 0x02)
#define SWIM_FLASH_FPR         (SWIM_FLASH + 0x03)
#define SWIM_FLASH_NFPR        (SWIM_FLASH + 0x04)
#define SWIM_FLASH_IAPSR       (SWIM_FLASH + 0x05)
#define SWIM_FLASH_PUKR        (SWIM_FLASH + 0x08)
#define SWIM_FLASH_DUKR        (SWIM_FLASH + 0x0a)
#define SWIM_FLASH_PUKR_KEY1   0x56
#define SWIM_FLASH_PUKR_KEY2   0xae
#define SWIM_FLASH_DUKR_KEY1   0xae
#define SWIM_FLASH_DUKR_KEY2   0x56

uint32_t swim_init();
uint32_t swim_system_reset();
uint32_t swim_write_on_the_fly(uint8_t n, uint32_t address, uint8_t* data);
uint32_t swim_read_on_the_fly(uint8_t n, uint32_t address, uint8_t* data);
//uint32_t erase_flash();
//uint32_t write_flash();


//please don't use these from the outside of this library
void swim_listen_to(uint32_t number);
uint32_t swim_send_header(uint8_t header);
uint32_t swim_send_byte(uint8_t byte);

#endif // __SWIM_H__
