#include "rfid.h"

void rfid_CRC16(unsigned char *FromAddr,
 unsigned short *ToAddr,
 unsigned char Size){
int i,ByteNo;
 unsigned short C;
*ToAddr=0;
for (ByteNo=1;ByteNo<=Size;
 ByteNo++,FromAddr++) {
C=((*ToAddr>>8)^*FromAddr)
 <<8;
for (i=0;i<8;i++)
if (C&0x8000) C=(C<<1)
 ^0x1021;
else C=C<<1;
*ToAddr=C^(*ToAddr<<8);
}
}


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

void RFID_received(UART_DATA *uart)
{
	char i;
	unsigned short rezCRC;
	rfid_CRC16(uart->rx_buffer,&rezCRC,9);
	if ((uart->rxcnt==11) && (uart->rx_buffer[10]==(rezCRC & 0xff)) && (uart->rx_buffer[9]==((rezCRC & 0xff00)>>8)))
		{
		res_table[MBReg_CardsData_0]=uart->rx_buffer[3]+uart->rx_buffer[4]*256;
		res_table[MBReg_CardsData_1]=uart->rx_buffer[5]+uart->rx_buffer[6]*256;
		res_table[MBReg_CardsData_2]=uart->rx_buffer[7];





	StateFlags|=StateFlag_ifCard;
	//test open door


	for(i=0;i<30;i+=3)
	{
     if((res_table[MBReg_CardsData_0]==res_table[MBReg_CardsData_Save_0+i]) &&
    		 (res_table[MBReg_CardsData_1]==res_table[MBReg_CardsData_Save_1+i]) &&
			 (res_table[MBReg_CardsData_2]==res_table[MBReg_CardsData_Save_2+i]))
     {
    	main_open_door();
     }
	}

}
	uart_rx(uart);



}
