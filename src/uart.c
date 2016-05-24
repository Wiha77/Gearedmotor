#include "uart.h"


//***************************************************************************
//dummy
//***************************************************************************

void uart_dummy(UART_DATA *uart) {

}
//***************************************************************************
// *  UART init
// **************************************************************************
void USART_Init(UART_DATA *uart) {
	USART_Init(uart->usart, &uart->usart_param);
	USART_Cmd(uart->usart, ENABLE);
	USART_ITConfig(uart->usart, USART_IT_RXNE, ENABLE);
	uart->rxcnt = 0;
}

//***************************************************************************
// *  UART init structure
// **************************************************************************
void USART_InitStructure(UART_DATA *uart) {
	uart->rxcnt = 0;
	uart->txlen = 0;
	uart->rxcnt = 0;
	uart->txlen = 0;
	uart->rx_buffer_size = 0;
	uart->tx_buffer_size = 0;
	uart->rxtimer = 0;
	uart->delay = 4;
	uart->usart_param.USART_BaudRate = 115200;
	uart->usart_param.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	uart->usart_param.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	uart->usart_param.USART_Parity = USART_Parity_No;
	uart->usart_param.USART_StopBits = USART_StopBits_1;
	uart->usart_param.USART_WordLength = USART_WordLength_8b;
	uart->recived_func = uart_dummy;
	uart->transmited_func = uart_dummy;

}

//***************************************************************************
// *  USART interrupt
// **************************************************************************
//
void USART_IRQ(UART_DATA *uart) {
	//Receive Data register not empty interrupt
	if (USART_GetITStatus(uart->usart, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(uart->usart, USART_IT_RXNE);
		uart->rxtimer = 0;

		if (uart->rxcnt > (uart->rx_buffer_size - 2))
			uart->rxcnt = 0;

		uart->rx_buffer[uart2->rxcnt++] = USART_ReceiveData(uart->usart);

	}

	//Transmission complete interrupt
	if (USART_GetITStatus(uart->usart, USART_IT_TC) != RESET) {

		USART_ClearITPendingBit(uart->usart, USART_IT_TC);
		if (uart->txcnt < uart->txlen) {
			USART_SendData(uart->usart, uart->tx_buffer[uart->txcnt++]);
		} else {
			uart->txlen = 0;
			(uart->transmited_func)(uart);

			USART_ITConfig(uart->usart, USART_IT_RXNE, ENABLE);
			USART_ITConfig(uart->usart, USART_IT_TC, DISABLE);
		}
	}

}
//***************************************************************************
//Timer interrupt
//***************************************************************************

void USART_TIMER_IRQ(UART_DATA *uart) {
	if (uart->rxtimer++ > uart->delay)
		if (uart.rxcnt > 0) {
			//окончание приема пакета
			(uart->recived_func)(uart);
		} else
			uart->rxtimer = 0;
}

//***************************************************************************
//send data from uart if data is ready
//***************************************************************************
void uart_tx(UART_DATA *uart) {
	if ((uart->txlen > 0) & (uart->txcnt == 0)) {
		USART_ITConfig(uart->usart, USART_IT_RXNE, DISABLE);
		USART_ITConfig(uart->usart, USART_IT_TC, ENABLE);

		USART_SendData(uart->usart, uart->tx_buffer[uart->txcnt++]);
	}

}

//***************************************************************************
//init reciving data from uart
//***************************************************************************
void uart_rx(UART_DATA *uart) {

	uart->rxcnt = 0;
	uart->rxtimer = 0;
	USART_ITConfig(uart->usart, USART_IT_RXNE,ENABLE );
	USART_ITConfig(uart->usart, USART_IT_TC, DISABLE);
}
