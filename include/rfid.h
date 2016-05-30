#ifndef __RFID_H
#define __RFID_H
#include "stm32f10x.h"
#include "uart.h"
#include "inc_glob.h"
extern UART_DATA uart1;
extern int16_t res_table[];
extern void main_open_door(void);
void USART1_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void RFID_received(UART_DATA *uart);

#endif /* __RFID_H */
