#include "rfid.h"
void USART1_IRQHandler(void)
{
	USART_IRQ(&uart1);
}

void TIM1_UP_TIM16_IRQHandler(void)
{

	if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET) {
		USART_TIMER_IRQ(&uart1);
//		/* Даём знать, что обработали прерывание */
		TIM_ClearITPendingBit(TIM16, TIM_IT_Update);}
}
