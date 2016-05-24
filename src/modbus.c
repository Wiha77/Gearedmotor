#include "modbus.h"
//extern  int res_table[OBJ_SZ];






//***************************************************************************
//send data from uart2 if data is ready
//***************************************************************************
void net_tx2(UART_DATA *uart)
{
  if((uart->txlen>0)&(uart->txcnt==0))
  {
	    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	    USART_ITConfig(USART2, USART_IT_TC, ENABLE);

	    GPIO_SetBits(GPIOA,GPIO_Pin_4); //переключаем на передачу

	  	USART_SendData(USART2, uart->buffer[uart->txcnt++]);
  }

}



//***************************************************************************
// *  USART2 interrupt
// **************************************************************************
void USART2_IRQHandler(void)
{
	USART_IRQ(&uart2);
}

//***************************************************************************
//Timer interrupt
//***************************************************************************
void TIM7_IRQHandler(void)
{
	USART_TIMER_IRQ(&uart2);
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
}

//***************************************************************************
//main()
//***************************************************************************
void main_modbus(void)
{




     //Main loop

    	if(uart2.rxgap==1)
    	 {
     	 MODBUS_SLAVE(&uart2);
    	 net_tx2(&uart2);
    	 }


}









//***********************************************************************



//*********************************************************************
//Modbus slave function
//*********************************************************************
 void MODBUS_SLAVE(UART_DATA *MODBUS)
{
  unsigned int tmp;


     //recive and checking rx query
	if((MODBUS->rx_buffer[0]!=0)&(MODBUS->rxcnt>5)& ((MODBUS->rx_buffer[0]==*modbus_addr)|(MODBUS->rx_buffer[0]==255)))
 {
	  tmp=Crc16(MODBUS->rx_buffer,MODBUS->rxcnt-2);

     if((MODBUS->rx_buffer[MODBUS->rxcnt-2]==(tmp&0x00FF)) & (MODBUS->rx_buffer[MODBUS->rxcnt-1]==(tmp>>8)))
    {


         //choosing function
	     switch(MODBUS->buffer[1])
       {
       case 3:
       TX_03_04(MODBUS);
       break;

       case 4:
       TX_03_04(MODBUS);
       break;

       case 6:
       TX_06(MODBUS);
       break;

       default:
       //illegal operation
       TX_EXCEPTION(MODBUS,0x01);
	   }

	     //adding CRC16 to reply
   	     tmp=Crc16(MODBUS->buffer,MODBUS->txlen-2);
         MODBUS->buffer[MODBUS->txlen-2]=tmp;
         MODBUS->buffer[MODBUS->txlen-1]=tmp>>8;
         MODBUS->txcnt=0;

	}

 }

	  MODBUS->rxgap=0;
	  MODBUS->rxcnt=0;
	  MODBUS->rxtimer=0xFFFF;

}

//******************************************************************
//READING input & holding registers
//*******************************************************************
void TX_03_04(UART_DATA *MODBUS)
{
unsigned int tmp,tmp1;
unsigned int m=0,n=0;
int tmp_val,tmp_val_pos;

   //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
   //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
   //MODBUS->buffer[2] = data byte count

    //2-3  - starting address
   tmp=((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]);

   //4-5 - number of registers
   tmp1=((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);

   //default answer length if error
   n=3;

  if((((tmp+tmp1)<OBJ_SZ)&(tmp1<MODBUS_WRD_SZ+1)))
   {

    for(m=0;m<tmp1;m++)
    {
      tmp_val=res_table[m+tmp];

    if(tmp_val<0)
    {
    tmp_val_pos=tmp_val;
    MODBUS->buffer[n]=(tmp_val_pos>>8)|0b10000000;
    MODBUS->buffer[n+1]=tmp_val_pos;
    }
    else
    {
     MODBUS->buffer[n]=tmp_val>>8;
     MODBUS->buffer[n+1]=tmp_val;
    }
     n=n+2;
    }

     MODBUS->buffer[2]=m*2; //byte count
     MODBUS->txlen=m*2+5; //responce length

   }
  else
  {
   //exception illegal data adress 0x02
   TX_EXCEPTION(MODBUS,0x02);
  }

}
//*******************************************************
//Writing
//*******************************************************
void TX_06(UART_DATA *MODBUS)
{
  unsigned int tmp;

   //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
   //MODBUS[1] = 6; //query type - - stay a same as in recived

   //2-3  - adress   , 4-5 - value

   tmp=((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //adress

   //MODBUS->buffer[2]  - byte count a same as in rx query

   if(tmp<OBJ_SZ)
   {
    MODBUS->txlen=MODBUS->rxcnt; //responce length
    res_table[tmp]=(MODBUS->buffer[4]<<8)+MODBUS->buffer[5];
    TestWriteVarEEPROM(tmp); //проверим не нужно ли записать этот регистр в еепром

   }
   else
   {
    //illegal data
    TX_EXCEPTION(MODBUS,0x02) ;
   }

}

//********************************************************************
//Exception if wrong query
//*********************************************************************

//modbus exception - illegal data=01 ,adress=02 etc
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type)
{
 //illegal operation
  MODBUS->buffer[2]=error_type; //exception
  MODBUS->txlen=5; //responce length
}

//*********************************************************************
//CRC16 for Modbus Calculation
//*********************************************************************
unsigned int Crc16(unsigned char *ptrByte, int byte_cnt)
{
unsigned int w=0;
char shift_cnt;

 if(ptrByte)
 {
   w=0xffffU;
   for(; byte_cnt>0; byte_cnt--)
   {
    w=(unsigned int)((w/256U)*256U+((w%256U)^(*ptrByte++)));
    for(shift_cnt=0; shift_cnt<8; shift_cnt++)
    {
     if((w&0x1)==1)
     w=(unsigned int)((w>>1)^0xa001U);
     else
     w>>=1;
    }
   }
 }
return w;
}






