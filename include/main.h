#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_it.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "eeprom.h"
#include "modbus.h"
#include "onewire.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_exti.h"
#define MyDebug 1

//static int res_table[OBJ_SZ];


//7extern  static u16 FLAGS;

/* Private function prototypes -----------------------------------------------*/
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
//void USART2_IRQHandler(void);
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);
void usart_init(void);
//void USART2_IRQHandler(void);
void TestWriteVarEEPROM(unsigned int);
void FatallError (void);
void StopMotor (void);
