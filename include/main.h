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
#include "stm32f10x_iwdg.h"
#include "stm32f10x_dbgmcu.h"
#define MyDebug 1





void TestWriteVarEEPROM(unsigned int);
void FatallError (void);

/*CHIP RESOURSES
 * MODBUS:
 * TIM7_IRQHandler
 * USART2_IRQHandler
 *delay_ms:
 * TIM6
 */

