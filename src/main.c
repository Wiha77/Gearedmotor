#include "main.h"

UART_DATA uart1;
UART_DATA uart2;
unsigned char uart_mobdus_buff[256];
unsigned char uart_rfid_buff[16];
unsigned char *modbus_addr=(unsigned char *)&res_table[MBReg_AdrrModbus];
/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NumbOfVar]={
		1000,
		 MBReg_AdrrModbus,
		 MBReg_Cur_zero_offset,
		 MBReg_Cur_gain,
		 MBReg_CardsData
		 };




ErrorStatus  		HSEStartUpStatus;
FLASH_Status 		FlashStatus;
uint16_t 			VarValue = 0;

#define SensorBitD1   		GPIO_Pin_3
#define SensorPortD1		GPIOB

#define SensorBitD2    		GPIO_Pin_5
#define SensorPortD2		GPIOB

#define SensorBitD3   		GPIO_Pin_10
#define SensorPortD3		GPIOA

#define PWMBitForvard   	GPIO_Pin_1
#define PWMPortForvard		GPIOA

#define PWMBitBack   		GPIO_Pin_10
#define PWMPortBack 		GPIOB

#define PWMRegForvard   	TIM3->CCR2
#define PWMRegBack  		TIM3->CCR1


// биты командных флажков CommandFlags
#define ComFlag_Door				0x0001		//
#define ComFlag_Gate			    0x0002		//
#define ComFlag_Light	    		0x0004		//


// Биты флажков состояния StateFlags
#define StateFlag_Door				0x0001    	//
#define StateFlag_Gate          	0x0002		//
#define StateFlag_ifCard			0x0004		//

#define StateFlag_Reset				0x0200		//




//#define PinCourse   7
//static char USART2_BUFF[USART2_NUM+1] = {'\0'};

//static u16 FLAGS=0;
//#define FLAGS_MotorRun 				0x0001 //Включение мотора



//Структура настройки портов
GPIO_InitTypeDef PORT;




//Функция задержки использует таймер:
void delay_ms(uint16_t value) {
	TIM6->ARR = value;
	TIM6->CNT = 0;
	TIM6->CR1 |= TIM_CR1_CEN;
	while ((TIM6->SR & TIM_SR_UIF) == 0) {
	} // дождаться конца задержки
	TIM6->SR &= ~TIM_SR_UIF;
}
void initial(void) {
	//Структура настройки портов
	GPIO_InitTypeDef PORT;
	//структура настройки прерываний
	 NVIC_InitTypeDef NVIC_InitStructure;
	 TIM_TimeBaseInitTypeDef base_timer;
	 TIM_OCInitTypeDef timer_oc;


	//Включем переферию шина APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN|RCC_APB1ENR_TIM7EN|RCC_APB1Periph_TIM3|RCC_APB1Periph_USART2;

	//Включем переферию шина APB2ENR
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2ENR_TIM16EN|RCC_APB2ENR_ADC1EN|RCC_APB2ENR_AFIOEN|RCC_APB2ENR_TIM1EN;


	//настраиваем портА

	  /* Configure USART2 Tx (PA.02) as alternate function push-pull (MODBUS) */
	  PORT.GPIO_Pin = GPIO_Pin_2;
	  PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	  PORT.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &PORT);

	  /* Configure USART2 Rx (PA.3) as input floating  (MODBUS) */
	  PORT.GPIO_Pin = GPIO_Pin_3;
	  PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOA, &PORT);

	  /* Configure USART2 Rx/Tx switch(PA.4) (MODBUS) */

	  PORT.GPIO_Pin = GPIO_Pin_4;
	  PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOA, &PORT);
	  GPIO_ResetBits(GPIOA,GPIO_Pin_4);

	  /* Configure TIM17 CH1 (PB9) as alternate function in
	  PORT.GPIO_Pin = GPIO_Pin_9;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &PORT);*/

	  /* Configure ADC_cur(PB.1) */

	  PORT.GPIO_Pin = GPIO_Pin_1;
	  PORT.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_Init(GPIOB, &PORT);

	  /* Configure ADC_U(PB.0) */

	  PORT.GPIO_Pin = GPIO_Pin_0;
	  PORT.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_Init(GPIOB, &PORT);

	  //Ножки разъема
// отключаем jtag
	  AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

	  //R6
	  PORT.GPIO_Pin = GPIO_Pin_3;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &PORT);

	  PORT.GPIO_Pin = GPIO_Pin_4;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &PORT);

	  PORT.GPIO_Pin = GPIO_Pin_15;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOA, &PORT);

	  //R7
	  PORT.GPIO_Pin = GPIO_Pin_5;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &PORT);

	  PORT.GPIO_Pin = GPIO_Pin_6;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &PORT);

	  //Trig
	  PORT.GPIO_Pin = GPIO_Pin_7;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &PORT);

	  //PORT.GPIO_Pin = GPIO_Pin_8;
	  //PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	  //GPIO_Init(GPIOB, &PORT);


	  	 // usart1 Rx (RFID)

	  PORT.GPIO_Pin = GPIO_Pin_10;
	  PORT.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_Init(GPIOA, &PORT);

	  	 // usart1 Tx (RFID)

	  PORT.GPIO_Pin = GPIO_Pin_9;
	  PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_Init(GPIOA, &PORT);



	  // PORT.GPIO_Pin = GPIO_Pin_11;
	  //PORT.GPIO_Mode = GPIO_Mode_IPU;
	  //GPIO_Init(GPIOA, &PORT);

	  //PORT.GPIO_Pin = GPIO_Pin_12;
	 //PORT.GPIO_Mode = GPIO_Mode_IPU;
	  //GPIO_Init(GPIOA, &PORT);


//неиспользуемые пины



	  PORT.GPIO_Pin = GPIO_Pin_5;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOA, &PORT);





	// Заблокировать настройки выводов PC8, PC9.
	// GPIOA->LCKR = GPIO_LCKR_LCK6|GPIO_LCKR_LCK7|    GPIO_LCKR_LCKK;
	// GPIOA->LCKR = GPIO_LCKR_LCK6|GPIO_LCKR_LCK7;
	// GPIOA->LCKR = GPIO_LCKR_LCK6|GPIO_LCKR_LCK7|    GPIO_LCKR_LCKK;
	// tmp=GPIOA->LCKR; tmp=GPIOA->LCKR;



//инициализация АЦП


	   RCC->CFGR    &= ~RCC_CFGR_ADCPRE;      				//входной делитель 12МГц
	   ADC1->CR1     =  ADC_CR1_SCAN; 						//ADC_CR1_JEOCIE;       //This bit is set and cleared by software to enable/disable the end of conversion interrupt for
	   	   	   	   	   	   	   	   	   	   	   	   	   	   	//injected channels.
	   ADC1->JSQR = 0x342509;    							// инжекционная последовательность 9-8-9-8
	   ADC1->SQR1    =  0;                    //
	   ADC1->CR2    |=  ADC_CR2_CAL;          				//запуск калибровки
	   while (!(ADC1->CR2 & ADC_CR2_CAL)){};  				//ждем окончания калибровки
	   ADC1->CR2     |=ADC_CR2_JEXTTRIG|ADC_CR2_JEXTSEL_2;    //External trigger conversion mode for injected channels/Timer 3 CC4 event/



	   ADC1->CR2    |=  ADC_CR2_ADON;         //включить АЦП



	//    /*timer config (For: delay_ms())*/
	TIM6->PSC = 24000 - 1;
	TIM6->CR1 |= TIM_CR1_OPM;
	//Инициализация TIM7 используется для приема модбас

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    //0.0001 sec setup APB=24Mhz/(24*100)
    TIM_TimeBaseStructure.TIM_Prescaler= 36;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=50;//till what value timer will count

    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM7, ENABLE);

    //Инициализация таймера TIM16 для определения конца передачи RFID
    /**/
      TIM_TimeBaseStructInit(&base_timer);
      base_timer.TIM_Prescaler = 24 - 1;
      base_timer.TIM_Period = 1000;
      TIM_TimeBaseInit(TIM16, &base_timer);

 	  /* Разрешаем таймеру генерировать прерывание*/
 	  TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);


		//uart ports
		USART_InitStructure(&uart1);
		USART_InitStructure(&uart2);
		uart1.usart_param.USART_BaudRate=9600;
		uart1.rx_buffer=uart_rfid_buff;
		uart1.tx_buffer=uart_rfid_buff;
		uart1.rx_buffer_size=16;
		uart1.tx_buffer_size=16;

		uart2.rx_buffer=uart_mobdus_buff;
		uart2.tx_buffer=uart_mobdus_buff;
		uart2.recived_func=MODBUS_SLAVE;
		uart2.transmited_func=MODBUS_receiver_on;

		uart2.rx_buffer_size=250;
		uart2.tx_buffer_size=250;
		uart2.usart=USART2;

		 //rx tx modbus






	  /* Enable the USART1 Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);


	  /* Enable the USART2 Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	   // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);






	   // Прерывания от TIM16;
	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn ;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);




	  TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);



	delay_ms(1);


	// Настроим Вывод  (PWM)верхние ключи
	PORT.GPIO_Pin = (PWMBitForvard);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWMPortForvard, &PORT);
	GPIO_ResetBits(PWMPortForvard,PWMBitForvard);

	// Настроим Вывод  (PWM)верхние ключи
	PORT.GPIO_Pin = (PWMBitBack);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWMPortBack, &PORT);
	GPIO_ResetBits(PWMPortBack,PWMBitBack);

	// Настроим Вывод PA7 (PWM)
	PORT.GPIO_Pin = (GPIO_Pin_7);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &PORT);
	//GPIO_SetBits(GPIOA,GPIO_Pin_7);

	// Настроим Вывод PA6 (PWM)
	PORT.GPIO_Pin = (GPIO_Pin_6);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &PORT);
	  /* Включаем таймер */
	  TIM_Cmd(TIM3, ENABLE);


		delay_ms(1);

		USART_Initial(&uart1);
		USART_Initial(&uart2);

		uart_rx(&uart2);




}



void TestWriteVarEEPROM(unsigned int temp)
{

//проверим не нужно ли записать этот регистр в еепром
int i=0;
 for(;i<NumbOfVar;i++)
 {
	 if(VirtAddVarTab[i]==(uint16_t)temp)
	 {
		 //записываем в eeprom
		 if (EE_WriteVariable(temp,res_table[temp])!= FLASH_COMPLETE )FatallError ();
		 return;
	 }

 }
}
void FatallError (void)
{
	while(1)
	{

	}
}
void InitVariable(void)
{
	   //////////еепром
	uint16_t TempVar;
	    	  /* Unlock the Flash Program Erase controller */
	    	  FLASH_Unlock();
	      	  /* EEPROM Init */
	    	  EE_Init();
	    	  EE_ReadVariable(VirtAddVarTab[0],&TempVar);
	    	  if (TempVar!=0xeeee)
	    	  {
	    		  // Инициализация еепром по умолчанию
	    		 if (EE_Format()!= FLASH_COMPLETE )FatallError();
	    		 if (EE_WriteVariable(1000, 0xeeee)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_AdrrModbus, 2)!= FLASH_COMPLETE )FatallError (); //модбус адресс по умолчанию
	    		 if (EE_WriteVariable(MBReg_Cur_zero_offset, 0)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_Cur_gain, 920)!= FLASH_COMPLETE )FatallError ();

	    	  }




	    	EE_ReadVariable(MBReg_AdrrModbus,&TempVar);
   	    		res_table[MBReg_AdrrModbus]=TempVar;
			EE_ReadVariable(MBReg_Cur_zero_offset,&TempVar);
				res_table[MBReg_Cur_zero_offset]=TempVar;
			EE_ReadVariable(MBReg_Cur_gain,&TempVar);
				res_table[MBReg_Cur_gain]=TempVar;

}



void SysTick_Handler(void)
{
//1mc

}
int main(void) {


	initial();
	delay_ms(1);
	SysTick_Config(24000);
	InitVariable();

	StateFlags|=StateFlag_Reset; // Установим флаг перезапуска программы

// инициализация сторожевого таймера
	/*
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_4);
	IWDG_SetReload(0xfff);

	IWDG_Enable();*/


    while(1)
	{
    	Sensor_Up=ADC1->JDR4*1.71;
    	//Sensor_Up=ADC1->JDR3;
    	Densor_Imotor=((int)(ADC1->JDR3)-Cur_zero_offset)*Cur_gain/200; //divided on 200 because there's sold 4 wires
    	if (Densor_Imotor<10)Densor_Imotor=0;
		//delay_ms(1);
	   //IWDG_ReloadCounter(); //сброс сторожевого таймера


	}




}

