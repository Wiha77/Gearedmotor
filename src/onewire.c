/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */

#include "onewire.h"
uint16_t  CalkDivBaudrate(USART_TypeDef* USARTx, uint32_t MyBautrate);
#ifdef OW_USART1

#undef OW_USART2
#undef OW_USART3
#undef OW_USART4

#define OW_USART 		USART1
#define OW_DMA_CH_RX 	DMA1_Channel5
#define OW_DMA_CH_TX 	DMA1_Channel4
#define OW_DMA_FLAG		DMA1_FLAG_TC5

#endif


#ifdef OW_USART2

#undef OW_USART1
#undef OW_USART3
#undef OW_USART4

#define OW_USART 		USART2
#define OW_DMA_CH_RX 	DMA1_Channel6
#define OW_DMA_CH_TX 	DMA1_Channel7
#define OW_DMA_FLAG		DMA1_FLAG_TC6

#endif


#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff
//"\xcc\x44"
const uint8_t OW_bufPreobr[16]= {OW_0,OW_0,OW_1,OW_1,OW_0,OW_0,OW_1,OW_1,OW_0,OW_0,OW_1,OW_0,OW_0,OW_0,OW_1,OW_0};
//"\xcc\xbe\xff\xff"
const uint8_t OW_bufReadTemp[88]={OW_0,OW_0,OW_1,OW_1,OW_0,OW_0,OW_1,OW_1,OW_0,OW_1,OW_1,OW_1,OW_1,OW_1,OW_0,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,
		OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1,OW_1};

// Буфер для приема/передачи по 1-wire
uint8_t ow_buf[88];
uint16_t  DivBaud9600,DivBaud115200 ;
int16_t RezTemperature=ErrTemperature;
static uint8_t TipTransaction;

uint8_t OW_toBute(uint8_t N_Bute) {
	uint8_t ow_byte, i;
	uint8_t* ow_bits=ow_buf+16+(N_Bute*8);
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}
return ow_byte;
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
uint16_t OW_toRezultat(void) {
uint8_t i,j;
	uint16_t ow_byte;
	uint8_t* ow_bits=ow_buf+16;
	ow_byte = 0;
	for (i = 0; i < 16; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x8000;
		}
		ow_bits++;
	}

	///////////////////////
uint8_t crc = 0;
uint8_t data;

uint8_t tmp;

	  //прочитать 8 байт и вычислить CRC
	  for( i=0; i<8; i++)
	  {
	    data = OW_toBute(i);         //прочитать очередной байт

	    //вычисление CRC - обрабатываем каждый бит принятого байта
	    for( j=0; j<8; j++)
	    {
	      tmp = (crc ^ data) & 0x01;
	      if (tmp==0x01) crc = crc ^ 0x18;
	      crc = (crc >> 1) & 0x7F;
	      if (tmp==0x01) crc = crc | 0x80;
	      data = data >> 1;
	    }
	  }

	  data = OW_toBute(8);          //прочитать CRC датчика
	  if(crc==data) return ow_byte/1.6;                   //если CRC совпали - значит ошибки нет
	  return ErrTemperature;                                 //CRC не совпали, ошибка принятых данных




}

//-----------------------------------------------------------------------------
// инициализирует USART и DMA
//-----------------------------------------------------------------------------
void OW_Init() {
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;


	if (OW_USART == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		// USART TX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		// USART RX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		USART_HalfDuplexCmd(USART1,ENABLE);

		DivBaud9600=CalkDivBaudrate(USART1,9600);
		DivBaud115200=CalkDivBaudrate(USART1,115200);



		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_Init(OW_USART, &USART_InitStructure);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	}

}
void OW_DeInit(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
	// USART TX
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// USART RX
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

}
//-----------------------------------------------------------------------------
// запускает процесс преобразоваания
//-----------------------------------------------------------------------------
void OW_StartTransaction(uint8_t SendCommand) {
	// отключаем DMA
	DMA_Cmd(OW_DMA_CH_TX, DISABLE);
	DMA_Cmd(OW_DMA_CH_RX, DISABLE);
	DMA_ITConfig(OW_DMA_CH_RX,DMA_IT_TC, DISABLE);
	USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);

	USART_Cmd(OW_USART, DISABLE);
	OW_USART->CR1 &=~(USART_Mode_Tx | USART_Mode_Rx);
	TipTransaction=SendCommand;
	// отправляем 0xf0 на скорости 9600 и включим прерывания

	OW_USART->BRR =DivBaud9600;
	USART_ClearFlag(OW_USART, USART_FLAG_RXNE);
	USART_ReceiveData(OW_USART);
	OW_USART->SR &=0;
	USART_ClearITPendingBit(OW_USART,USART_IT_RXNE);
	USART_ITConfig(OW_USART, USART_IT_RXNE, ENABLE);
	OW_USART->CR1 |=USART_Mode_Tx | USART_Mode_Rx;
	USART_Cmd(OW_USART,  ENABLE);
	USART_SendData(OW_USART, 0xf0);

}


//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------


uint16_t  CalkDivBaudrate(USART_TypeDef* USARTx, uint32_t MyBautrate)

{
	  uint32_t tmpreg = 0x00, apbclock = 0x00;
	  uint32_t integerdivider = 0x00;
	  uint32_t fractionaldivider = 0x00;
	  uint32_t usartxbase = 0;
	  RCC_ClocksTypeDef RCC_ClocksStatus;
	  usartxbase = (uint32_t)USARTx;
	  /* USART OverSampling-8 Mask */
	  #define CR1_OVER8_Set             ((u16)0x8000)  /* USART OVER8 mode Enable Mask */








/*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate -------------------------------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  if (usartxbase == USART1_BASE)
  {
    apbclock = RCC_ClocksStatus.PCLK2_Frequency;
  }
  else
  {
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;
  }

  /* Determine the integer part */
  if ((USARTx->CR1 & CR1_OVER8_Set) != 0)
  {
    /* Integer part computing in case Oversampling mode is 8 Samples */
    integerdivider = ((25 * apbclock) / (2 * (MyBautrate)));
  }
  else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
  {
    /* Integer part computing in case Oversampling mode is 16 Samples */
    integerdivider = ((25 * apbclock) / (4 * (MyBautrate)));
  }
  tmpreg = (integerdivider / 100) << 4;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USARTx->CR1 & CR1_OVER8_Set) != 0)
  {
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  }
  else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
  {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }

  /* Write to USART BRR */
   return (uint16_t)tmpreg;
}
//прерывания от таймера, после приема ресет от 1wire
void USART1_IRQHandler(void)
{


	uint8_t ow_presence;
	USART_ITConfig(OW_USART, USART_IT_RXNE, DISABLE); //запрещаем прерываания
	ow_presence = USART_ReceiveData(OW_USART);
	OW_USART->BRR =DivBaud115200;


	if (ow_presence == 0xf0) {
		RezTemperature=ErrTemperature;
		return ;
	}


//запускаем преобразование

			DMA_InitTypeDef DMA_InitStructure;


			// DMA на чтение
			//DMA_DeInit(OW_DMA_CH_RX);
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
			DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
			if (TipTransaction==OW_KomReadTemp)DMA_InitStructure.DMA_BufferSize = 88;
			else DMA_InitStructure.DMA_BufferSize = 16;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
			DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
			DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);


			// DMA на запись
			//DMA_DeInit(OW_DMA_CH_TX);
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);

			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
			if (TipTransaction==OW_KomReadTemp)
				{	DMA_InitStructure.DMA_BufferSize = 88;
					DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) OW_bufReadTemp;

				}
			else
				{
					DMA_InitStructure.DMA_BufferSize = 16;
					DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) OW_bufPreobr;
				}
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
			DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
			DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);


			// старт цикла отправки
			USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
			USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);


				DMA_ClearITPendingBit(DMA1_IT_TC5);
				DMA_ClearFlag(DMA1_FLAG_TC5|DMA1_FLAG_GL5|DMA1_FLAG_HT5|DMA1_FLAG_TE5);
				DMA_ITConfig(OW_DMA_CH_RX,DMA_IT_TC, ENABLE);


			DMA_Cmd(OW_DMA_CH_RX, ENABLE);
			DMA_Cmd(OW_DMA_CH_TX, ENABLE);




	}
void DMA1_Channel5_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TC5) != RESET)
	{

			// отключаем DMA
			DMA_Cmd(OW_DMA_CH_TX, DISABLE);
			DMA_Cmd(OW_DMA_CH_RX, DISABLE);
			DMA_ITConfig(OW_DMA_CH_RX,DMA_IT_TC, DISABLE);
			USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);
			if (TipTransaction==OW_KomReadTemp)RezTemperature= OW_toRezultat();
	}
}
