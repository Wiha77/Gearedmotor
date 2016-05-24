#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

//uart structure
typedef struct {
USART_TypeDef *usart;     //base address usart
USART_InitTypeDef usart_param;
unsigned char *rx_buffer;    //buffer for receiving
unsigned char rx_buffer_size;
unsigned char *tx_buffer;    //buffer for transmitting
unsigned char tx_buffer_size;
unsigned int  rxtimer;
unsigned char rxcnt;
unsigned char txcnt;
unsigned char txlen;
unsigned char delay;  //number symbols for stopping to receive
void (* recived_func)(UART_DATA *uart);
void (* transmited_func)(UART_DATA *uart);
} UART_DATA;

void USART_InitStructure(UART_DATA *uart);
void USART_Init(UART_DATA *uart);
void USART_IRQ(UART_DATA *uart);
void USART_TIMER_IRQ(UART_DATA *uart);
void uart_tx(UART_DATA *uart);
void uart_rx(UART_DATA *uart);



#endif /* __UART_H */
