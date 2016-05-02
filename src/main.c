#include "main.h"

extern  int res_table[OBJ_SZ];
#define  	MaxChangSpeed 300 //максимальное изменение скорости без перестарта ПИД
#define  	ConstWaitForced 500 //Время грубого разгона в мС
#define  	TimeFuseNoTurn   1000 //Время защиты от остановки якоря
#define  	SonarBufSize 8
#define  	SonarKalman 100
uint16_t 	SonarBuf[SonarBufSize];
uint8_t 	SonarCount = 0;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NumbOfVar]={
		1000,
		 MBReg_AdrrModbus,
		 MBReg_K_proporc,
		 MBReg_K_integral,
		 MBReg_K_diferencial,
		 MBReg_PIDMaxErr,
		 MBReg_MaxCurrent,
		 MBReg_TimeMaxCurrent,
		 MBReg_Cur_zero_offset,
		 MBReg_Cur_gain,
		 MBReg_PID_MaxPWM,
		 MBReg_PID_MinPWM,
		 MBReg_PID_MaxTurns,
		 MBReg_PID_MinTurns
		 ,38};




ErrorStatus  		HSEStartUpStatus;
FLASH_Status 		FlashStatus;
uint16_t 			VarValue = 0;
extern int16_t RezTemperature;



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



#define MaxPWM      1000
#define MinPWM      150
#define startPWM   	300
// биты командных флажков CommandFlags
#define ComFlag_STOP				0x0001		//Бит команды СТОП, сбрасывается автоматически после выполнения команды (приоритет по убыванию СТОП,ВПЕРЕД,НАЗАД)
#define ComFlag_ForvardPUSK			0x0002		//Бит команды ВПЕРЕД, сбрасывается автоматически после выполнения команды
#define ComFlag_BackPUSK			0x0004		//Бит команды НАЗАД, сбрасывается автоматически после выполнения команды
#define ComFlag_StopIfD1			0x0008		//Бит разрешает остановку по концевому датчику D1
#define ComFlag_StopIfD2			0x0010		//Бит разрешает остановку по концевому датчику D2
#define ComFlag_StopIfD3			0x0020		//Бит разрешает остановку по концевому датчику D3
#define ComFlag_StopIfNumberTurns	0x0040		//Бит разрешает остановку по колличеству оборотов
#define ComFlag_Izm_DS18B20			0x0080		//Бит разрешает измерение температуры датчиком DS18B20
												//при установленном бите измерения проводятся раз в секунду
#define ComFlag_Izm_Sonar			0x0100		//Бит разрешает измерение сонаром. После измерения бит сбрасывается.
#define ComFlag_OffPID				0x0200		//Бит отключает работу ПИД, заполнение нужно изменять вручную


// Биты флажков состояния StateFlags
#define StateFlag_Motor				0x0001    	// Мотор работает
#define StateFlag_StopOverCurrent	0x0002		//Была остановка по превышению тока двигателя
#define StateFlag_StopIfD1			0x0004		//Была остановка по концевому датчику D1
#define StateFlag_StopIfD2			0x0008		//Была остановка по концевому датчику D2
#define StateFlag_StopIfD3			0x0010		//Была остановка по концевому датчику D3
#define StateFlag_StopIfNumberTurns	0x0020		//Была остановка по полличеству оборотов двигателя
#define StateFlag_D1				0x0040		//текущее состояние концевого датчика D1
#define StateFlag_D2				0x0080		//текущее состояние концевого датчика D2
#define StateFlag_D3				0x0100		//текущее состояние концевого датчика D3
#define StateFlag_Reset				0x0200		//Был перезапуск программы
#define StateFlag_Direction			0x0400		//Показывает направление Когда установлен вперед, сброшен назад
#define StateFlag_NoTurns			0x0800		//Нет вращения вала(защита)




#define PinCourse   7
//static char USART2_BUFF[USART2_NUM+1] = {'\0'};
static u16  CountTimerOW=0;
static u16 FLAGS=0;
#define FLAGS_MotorRun 				0x0001 //Включение мотора
#define FLAGS_StartMotor 			0x0002 //Флаг старта разгона
#define FLAGS_MotorFors 			0x0004 //Флаг показыввает что идет грубый разгон, дла защиты от заклинивания
#define ComFlag_Izm_DS18B20_Loc		0x0008 //Локальный бит разрешает измерение температуры датчиком DS18B20
#define ComFlag_Izm_Sonar_Loc		0x0010 //Локальный бит разрешает измерение сонаром
#define FLAGS_Forvard				0x0020 //Когда установлен вперед, сброшен назад

//Переменные пида
int PIDerr;// ошибка
int PIDPreErr;// прошлая ошибка
u16 PIDt_step;//шаг регулирования, равен периоду оборота вала
s32 PIDintegral; //Накопленная ошибка
u16 MotorSpeedTemp ; // Преведущее значение установленной скорости
u16 WaitForced;  //задержка для разгона движка перед началом работы ПИДа
u16 PWM_about=0; //Грубое значение
u16 CountFuseNoTurn=0; // Счетчик для зашиты от остановки якоря

u16 CountTimeCurrent=0xffff; // Счетчик для защиты по току

//Структура настройки портов
GPIO_InitTypeDef PORT;
// функция грубого определения значения заполнения
uint16_t PID_PreMatch (int16_t Turns)
{
	uint16_t Rezultat;
	if(Turns>PID_MaxTurns)Turns=PID_MaxTurns;
	if(Turns<PID_MinTurns)Turns=PID_MinTurns;
	Rezultat=(uint32_t)(Turns-PID_MinTurns)*(uint32_t)(PID_MaxPWM-PID_MinPWM)/(PID_MaxTurns-PID_MinTurns)+PID_MinPWM;

	return Rezultat;

}

// функции управления мотором

void PIDalg (u16 PIDt_step,u16 Kp,u16 Ki,u16 Kd)
{
	PIDrp=9000000/PIDt_step; //кол-во оборотов в минуту

	if (CommandFlags&ComFlag_OffPID)
		{
		FLAGS&=~FLAGS_StartMotor;
		return;
		}


	/**************    Делать каждый оборот мотора   ********************
	err = РП - ТП                 // вычислить текущую ошибку

	if (MIN < УВ < MAX){ // если УВ не достигло предела, то
	  integral = integral + error   // добавить ошибку в сумму ошибок
	                   }

	УВ_temp = Kp*err + Ki*integral*t_step + Kd*(err - pre_err)/t_step
	// вычисление Управляющего Воздействия - УВ

	УВ = MIN =< УВ_temp =< MAX
	// задание окончательно УВ в рамках допустимых значений.

	pre_err = err
	// текущая ошибка стала "прошлой ошибкой" для след. вычисления

	Примечание:  параметр t_step  можно исключить из повторяющихся вычислений заранее скорректировав коэффициенты  Ki и Kd  в  t_step  раз.
	*****************************************************************************/
	int temp;
	//если был старт или резкое изменение установки скорости то разгоняемся по грубой прикидке

	if(MotorSpeedTemp>MotorSpeed)temp=MotorSpeedTemp-MotorSpeed;
	else temp=MotorSpeed-MotorSpeedTemp;
	if((FLAGS&FLAGS_StartMotor)||(temp>MaxChangSpeed))
	{
		//начало разгона по грубой прикидке
		FLAGS&=~FLAGS_StartMotor;
		WaitForced=ConstWaitForced;
		FLAGS|=FLAGS_MotorFors;
		PIDintegral=0;
	}
	MotorSpeedTemp=MotorSpeed;

	if(WaitForced)
		{
		//грубый разгон
		MotorSpeedPWM=PID_PreMatch (MotorSpeed);
		PWM_about=MotorSpeedPWM;
		return;
		}
	if(FLAGS&FLAGS_MotorFors)
			{
			//здесь защита от заклинивания
			StateFlags|=StateFlag_NoTurns;
			MotorSpeedPWM=0;
			CommandFlags|=ComFlag_STOP;
			}

	//алгоритм ПИД регулятора

	//PIDrp=9000000/PIDt_step; //кол-во оборотов в минуту


	PIDerr = MotorSpeed-PIDrp; //текущая ошибка
	//ограничиваем ошибку
	if(PIDerr<0)temp=-PIDerr;
	else temp=PIDerr;
	if (temp>PIDMaxErr)temp=PIDMaxErr;
	if(PIDerr<0)PIDerr=-temp;
	else PIDerr=temp;
   //интеграл ошибки
	if ((PIDerr >0)&(MotorSpeedPWM<MaxPWM))PIDintegral+=PIDerr;
	if ((PIDerr <0)&(MotorSpeedPWM>MinPWM))PIDintegral+=PIDerr;




//вычисляем новое значение ШИМа
	 proporc=(Kp*PIDerr/1024);
	 integral=(Ki*PIDintegral/10000*PIDt_step/32768);
	 diferencial=(Kd*(PIDerr - PIDPreErr)*10/PIDt_step);


	MotorSpeedPWM=PWM_about+proporc+integral+diferencial;   //
	if(MotorSpeedPWM<MinPWM)MotorSpeedPWM=PID_MinPWM;
	if(MotorSpeedPWM>MaxPWM)MotorSpeedPWM=PID_MaxPWM;
	PIDPreErr=PIDerr;
}
void StopMotor (void)
{
	// Настроим Вывод PA7 (PWM)
	PORT.GPIO_Pin = (GPIO_Pin_7);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &PORT);
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);

	// Настроим Вывод PA6 (PWM)
	PORT.GPIO_Pin = (GPIO_Pin_6);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &PORT);
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);

	PIDintegral=0;
	FLAGS&=~FLAGS_MotorRun;
	PWMRegForvard  =0;
	PWMRegBack  =0;
	GPIO_ResetBits(PWMPortForvard,PWMBitForvard);
	GPIO_ResetBits(PWMPortBack,PWMBitBack);

}
void ForvardRun (void)
{
	// Настроим Вывод PA7 (PWM)
	PORT.GPIO_Pin = (GPIO_Pin_7);
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &PORT);


	FLAGS|=FLAGS_MotorRun|FLAGS_StartMotor|FLAGS_Forvard;
	GPIO_SetBits(PWMPortForvard,PWMBitForvard);
	CountFuseNoTurn=TimeFuseNoTurn;
}
void BackRun (void)
{
	// Настроим Вывод PA6 (PWM)
	PORT.GPIO_Pin = (GPIO_Pin_6);
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &PORT);

	FLAGS|=FLAGS_MotorRun|FLAGS_StartMotor;
	FLAGS&=~FLAGS_Forvard;
	GPIO_SetBits(PWMPortBack,PWMBitBack);
	CountFuseNoTurn=TimeFuseNoTurn;
}
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
	//Включем переферию шина APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN|RCC_APB1ENR_TIM7EN|RCC_APB1Periph_TIM3|RCC_APB1Periph_USART2;

	//Включем переферию шина APB2ENR
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2ENR_TIM16EN|RCC_APB2ENR_ADC1EN|RCC_APB2ENR_AFIOEN|RCC_APB2ENR_TIM1EN;


	//настраиваем портА

	  /* Configure USART2 Tx (PA.02) as alternate function push-pull */
	  PORT.GPIO_Pin = GPIO_Pin_2;
	  PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	  PORT.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &PORT);

	  /* Configure USART2 Rx (PA.3) as input floating */
	  PORT.GPIO_Pin = GPIO_Pin_3;
	  PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOA, &PORT);

	  /* Configure USART2 Rx/Tz switch(PA.4) */

	  PORT.GPIO_Pin = GPIO_Pin_4;
	  PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOA, &PORT);
	  GPIO_ResetBits(GPIOA,GPIO_Pin_4);

	  /* Configure TIM17 CH1 (PB9) as alternate function in */
	  PORT.GPIO_Pin = GPIO_Pin_9;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &PORT);

	  /* Configure ADC_cur(PB.1) */

	  PORT.GPIO_Pin = GPIO_Pin_1;
	  PORT.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_Init(GPIOB, &PORT);

	  /* Configure ADC_cur(PB.0) */

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
	  PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOB, &PORT);

	  PORT.GPIO_Pin = GPIO_Pin_8;
	  PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_Init(GPIOB, &PORT);


	  	 // usart1 Rx

	  PORT.GPIO_Pin = GPIO_Pin_10;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOA, &PORT);

	  PORT.GPIO_Pin = GPIO_Pin_11;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOA, &PORT);

	  PORT.GPIO_Pin = GPIO_Pin_12;
	  PORT.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOA, &PORT);


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



	//    /*timer config*/
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

    //Инициализация таймера TIM16 для запуска сонара

      TIM_TimeBaseInitTypeDef base_timer;
      TIM_TimeBaseStructInit(&base_timer);
      base_timer.TIM_Prescaler = 24 - 1;
      base_timer.TIM_Period = 60000;
      TIM_TimeBaseInit(TIM16, &base_timer);

      /* Теперь настраиваем 1й канал таймера */
       TIM_OCInitTypeDef timer_oc;
       TIM_OCStructInit(&timer_oc);
       timer_oc.TIM_Pulse = 59990;
       timer_oc.TIM_OCMode = TIM_OCMode_PWM2;
       /* Включаем основной  вывод */
       timer_oc.TIM_OutputState = TIM_OutputState_Enable;
       timer_oc.TIM_OCPolarity=TIM_OCPolarity_High;
       /* Активируем каналы */
       TIM_OC1Init(TIM16, &timer_oc);

       /* Включать AOE нужно для всех таймеров, имеющих break input,
          т.к. по умолчанию вывод на все каналы выключен. */
       TIM_BDTRInitTypeDef timer_bdtr;
       TIM_BDTRStructInit(&timer_bdtr);
       timer_bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
       TIM_BDTRConfig(TIM16, &timer_bdtr);
 	  /* Разрешаем таймеру генерировать прерывание по захвату */
 	  TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);

     // TIM16->CR1 |= TIM_CR1_OPM;

	//Инициализация таймера TIM1 для захвата сонара
	/* Настраиваем предделитель так, чтобы таймер считал миллисекунды.
	     На бльших частотах следите, чтобы предделитель не превысил
	     максимальное значение uint16_t - 0xFFFF (65535) */
      TIM_TimeBaseStructInit(&base_timer);
		base_timer.TIM_Prescaler = 24 - 1;
		TIM_TimeBaseInit(TIM1, &base_timer);

	/* Настраиваем захват сигнала:
	   - канал: 4
	   - счёт: по нарастанию
	   - источник: напрямую со входа
	   - делитель: отключен
	   - фильтр: отключен */
	  TIM_ICInitTypeDef timer_ic;
	  timer_ic.TIM_Channel = TIM_Channel_4;
	  timer_ic.TIM_ICPolarity = TIM_ICPolarity_Falling;
	  timer_ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  timer_ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  timer_ic.TIM_ICFilter = 0;
	  TIM_ICInit(TIM1, &timer_ic);

	  timer_ic.TIM_Channel = TIM_Channel_3;
	  timer_ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
	  timer_ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  timer_ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  timer_ic.TIM_ICFilter = 0;
	  TIM_ICInit(TIM1, &timer_ic);


	  /* Разрешаем таймеру генерировать прерывание по захвату */
	  TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
	  /* Включаем таймер */
	  TIM_Cmd(TIM1, ENABLE);



	/* Enable the TIM17 capture Interrupt */

	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM17_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

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


	   // Прерывания от дма сом1;
	  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn ;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	   // Прерывания от TIM1;
	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn ;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	   // Прерывания от TIM16;
	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn ;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);


	  /* Разрешаем прерывания таймера TIM1 */
	  	  NVIC_EnableIRQ(TIM1_CC_IRQn);


	  TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);


		//Инициализация таймера TIM3 для шима
		/* Настраиваем предделитель */
	      TIM_TimeBaseStructInit(&base_timer);
			base_timer.TIM_Prescaler = 0;
			base_timer.TIM_Period=MaxPWM;
			base_timer.TIM_CounterMode=TIM_CounterMode_Up;
			base_timer.TIM_ClockDivision=TIM_CKD_DIV1;
			base_timer.TIM_RepetitionCounter=0;
			TIM_TimeBaseInit(TIM3, &base_timer);


		    TIM_OCStructInit(&timer_oc);
			timer_oc.TIM_OCMode=TIM_OCMode_PWM1;
			timer_oc.TIM_OutputState=TIM_OutputState_Enable;
			timer_oc.TIM_Pulse=0;
			timer_oc.TIM_OCPolarity=TIM_OCPolarity_High;
			timer_oc.TIM_OCIdleState=TIM_OCIdleState_Reset;
			TIM_OC1Init(TIM3,&timer_oc);
			TIM_OC2Init(TIM3,&timer_oc);
			timer_oc.TIM_Pulse=200;
			//timer_oc.TIM_OutputState=TIM_OutputState_Disable;
			TIM_OC4Init(TIM3,&timer_oc);

/*
	//Разрешаем таймеру использовать ШИМ 1
	TIM3->CCER |= (TIM_CCER_CC1E|TIM_CCER_CC4E);
	TIM3->CCR1 =MotorSpeedPWM;
	TIM3->PSC = 0; //prescaller
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; //PWM mode
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_0| TIM_CCMR1_OC1PE);
	TIM3->CCMR2 |=TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4M_1;
	TIM3->CR1 &=~ TIM_CR1_ARPE;
	TIM3->ARR = MaxPWM; //period
	//TIM3->CCER&=~TIM_CCER_CC1P;			   //нормальная полярность
	TIM3->CCER |= TIM_CCER_CC1P; //обратная полярность
	TIM3->CCR4 =200;  //точка измерения тока и напряжения
	//Запускаем PWM
	TIM3->CR1 |= TIM_CR1_CEN;
	delay_ms(1);  */


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
		SetupUSART2();





	//Инициализация TIM17 PB9 для измерения оборотов якоря

	TIM17->EGR|=TIM_EGR_UG;
	TIM17->PSC = 160 - 1;
	TIM17->CCMR1=0x0001; // без фильтра и прескаллера на первый канал
	TIM17->CCER=0x0003; //Захват по спаду
	//TIM17->ARR =0xffff;
	TIM17->DIER|=TIM_DIER_CC1IE|TIM_DIER_UIE; //разрешаем прерывания ghb захвате;
	TIM17->CR1 |= TIM_CR1_URS|TIM_CR1_CEN;



}
void TIM1_UP_TIM16_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM16, TIM_IT_CC1) != RESET)
	{
		TIM1->CNT=0;

		/* Даём знать, что обработали прерывание */
	    TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);

	}
}
void TIM1_CC_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)
  {
    /* Даём знать, что обработали прерывание */
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
   //длительность импульса
   if (CommandFlags&ComFlag_Izm_Sonar)
   {
	   SonarBuf[SonarCount]=(TIM_GetCapture4(TIM1)-TIM_GetCapture3(TIM1))/5.8;
	   SonarCount++;
	   if (SonarCount>=SonarBufSize)
	   {
		   //Считаем среднее значение в SumBuf
		   SonarCount=0;
		   uint8_t temp,temp1;
		   int32_t SumBuf=0;
		   int32_t AproxBufMin,AproxBufMax;
		   for(temp=0;temp<SonarBufSize;temp++)
		   {
			   SumBuf += SonarBuf[temp];
		   }
		   AproxBufMin=SumBuf/SonarBufSize-SonarKalman;
		   if(AproxBufMin<0)AproxBufMin=0;
		   AproxBufMax=SumBuf/SonarBufSize+SonarKalman;

		//Теперь считаем среднее значение отфильтрованных данных
		   SumBuf=0;
		   temp1=0;
		   for(temp=0;temp<SonarBufSize;temp++)
		   {
			  if((SonarBuf[temp]>AproxBufMin)&&(SonarBuf[temp]<AproxBufMax))
					  {
				   SumBuf += SonarBuf[temp];
				   temp1++;
					  }


		   }

		   if (temp1>(SonarBufSize/2))
		   {  //есть усредненный отфильтрованный результат измерений
			   SonarData=SumBuf/temp1;
			   TIM16->CR1 &=~ TIM_CR1_CEN; //останавливаем измерения
			   CommandFlags&=~ComFlag_Izm_Sonar; //сбрасываем флаг запроса измерения
		   }

	   }

   }



    /* Тут как-нибудь обрабатываем событие over-capture, если провороним */
    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC4OF) != RESET)
    {
      TIM_ClearFlag(TIM1, TIM_FLAG_CC4OF);
      // ...
    }
  }
}
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
static u16 temp;
//temp=TIM17->CCR1;
//if(temp<900){
//	TIM17->SR&=~TIM_SR_UIF;
//	return;}


 if((TIM17->SR&TIM_SR_CC1IF)&&~(TIM17->SR&TIM_SR_UIF))
	{
	//прерывание было от мотора
	 CountFuseNoTurn=TimeFuseNoTurn; //сброс ращиты от остановки якоря
	 FLAGS&=~FLAGS_MotorFors; //сбросим флаг, мотор не заклинил
	 temp=TIM17->CCR1;
	 if(temp<900)return;
	 if(NumberOfTurns!=0)NumberOfTurns--;
	}
 else {
	 //Прерывание от переполнения счетчика а не от мотора
	 temp=TIM17->CCR1;
	 temp=0xffff;
     }
 TIM17->CNT=0;
TIM17->SR&=~TIM_SR_UIF;
 if(FLAGS&FLAGS_MotorRun)
 	 {
	 PIDalg (temp,K_proporc,K_integral,K_diferencial);
	 if(FLAGS&FLAGS_Forvard)PWMRegForvard =MotorSpeedPWM;
	 else PWMRegBack =MotorSpeedPWM;
 	 }

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
	    		 if (EE_WriteVariable(MBReg_K_proporc, 250)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_K_integral, 600)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_K_diferencial, MBReg_AdrrModbus)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_AdrrModbus, 100)!= FLASH_COMPLETE )FatallError (); //модбус адресс по умолчанию
	    		 if (EE_WriteVariable(MBReg_PIDMaxErr, 500)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_MaxCurrent, 4000)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_TimeMaxCurrent, 3000)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_Cur_zero_offset, 0)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_Cur_gain, 920)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_PID_MaxPWM, MaxPWM)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_PID_MinPWM, MinPWM)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_PID_MaxTurns, 2500)!= FLASH_COMPLETE )FatallError ();
	    		 if (EE_WriteVariable(MBReg_PID_MinTurns, 100)!= FLASH_COMPLETE )FatallError ();

	    	  }



	    	EE_ReadVariable(MBReg_K_proporc,&TempVar);
	   			res_table[MBReg_K_proporc]=TempVar;
	    	EE_ReadVariable(MBReg_K_integral,&TempVar);
	    		res_table[MBReg_K_integral]=TempVar;
	    	EE_ReadVariable(MBReg_K_diferencial,&TempVar);
	    		res_table[MBReg_K_diferencial]=TempVar;
	    	EE_ReadVariable(MBReg_AdrrModbus,&TempVar);
   	    		res_table[MBReg_AdrrModbus]=TempVar;
   	    	SET_PAR[0]=TempVar;//modbus address
	    	EE_ReadVariable(MBReg_PIDMaxErr,&TempVar);
	   			res_table[MBReg_PIDMaxErr]=TempVar;
		   	EE_ReadVariable(MBReg_MaxCurrent,&TempVar);
			   	res_table[MBReg_MaxCurrent]=TempVar;
			EE_ReadVariable(MBReg_TimeMaxCurrent,&TempVar);
				res_table[MBReg_TimeMaxCurrent]=TempVar;
			EE_ReadVariable(MBReg_Cur_zero_offset,&TempVar);
				res_table[MBReg_Cur_zero_offset]=TempVar;
			EE_ReadVariable(MBReg_Cur_gain,&TempVar);
				res_table[MBReg_Cur_gain]=TempVar;
			EE_ReadVariable(MBReg_PID_MaxPWM,&TempVar);
				res_table[MBReg_PID_MaxPWM]=TempVar;
			EE_ReadVariable(MBReg_PID_MinPWM,&TempVar);
				res_table[MBReg_PID_MinPWM]=TempVar;
			EE_ReadVariable(MBReg_PID_MaxTurns,&TempVar);
				res_table[MBReg_PID_MaxTurns]=TempVar;
			EE_ReadVariable(MBReg_PID_MinTurns,&TempVar);
				res_table[MBReg_PID_MinTurns]=TempVar;





				TempDS18B20=ErrTemperature;
}
void TestMotorCurrent(void)
{
	if(MaxCurrent<Densor_Imotor)
	{
		if(CountTimeCurrent==0)
		{
			CommandFlags|=ComFlag_STOP;
			StateFlags|=StateFlag_StopOverCurrent;
		}
	}
	else CountTimeCurrent=TimeMaxCurrent; //Время для максимального тока
}
void RunsCommands(void){

	if(CommandFlags&ComFlag_STOP)
	{
	StopMotor ();
	CommandFlags&=~(ComFlag_ForvardPUSK|ComFlag_BackPUSK|ComFlag_STOP);
	}
	if((CommandFlags&ComFlag_ForvardPUSK)&& ((StateFlags&StateFlag_Motor)==0))
	{
	ForvardRun ();
	CommandFlags&=~(ComFlag_ForvardPUSK|ComFlag_BackPUSK|ComFlag_STOP);
	StateFlags=0;
	}
	if((CommandFlags&ComFlag_BackPUSK)&& ((StateFlags&StateFlag_Motor)==0))
	{
	BackRun ();
	CommandFlags&=~(ComFlag_ForvardPUSK|ComFlag_BackPUSK|ComFlag_STOP);
	StateFlags=0;
	}
	if(FLAGS&FLAGS_MotorRun)StateFlags|=(StateFlag_Motor);
	else StateFlags&=~(StateFlag_Motor);

	//Защита от остановки якоря
	if((!CountFuseNoTurn)&&(StateFlags&StateFlag_Motor))
	{
	CommandFlags|=ComFlag_STOP;
	StateFlags|=StateFlag_NoTurns;
	}

	if ((NumberOfTurns==0)&&(CommandFlags&ComFlag_StopIfNumberTurns)&&(StateFlags&StateFlag_Motor))
	{
	StopMotor ();
	StateFlags|=(StateFlag_StopIfNumberTurns);
	}

	//перенисим состояние датчиков во флаги(датчик сработал когда 0)
	if (GPIO_ReadInputDataBit(SensorPortD1,SensorBitD1)==Bit_RESET )StateFlags|=StateFlag_D1;
	else StateFlags&=~StateFlag_D1;
	if (GPIO_ReadInputDataBit(SensorPortD2,SensorBitD2)==Bit_RESET )StateFlags|=StateFlag_D2;
	else StateFlags&=~StateFlag_D2;
	if (GPIO_ReadInputDataBit(SensorPortD3,SensorBitD3)==Bit_RESET )StateFlags|=StateFlag_D3;
	else StateFlags&=~StateFlag_D3;
	//перенисим состояние флага направления
	if (FLAGS&FLAGS_Forvard)StateFlags|=StateFlag_Direction;
	else StateFlags&=~StateFlag_Direction;


	//проверяем не нужно ли останавливать по достижению датчика
	if ((CommandFlags&ComFlag_StopIfD1)&&(StateFlags&StateFlag_Motor))
	if (GPIO_ReadInputDataBit(SensorPortD1,SensorBitD1)==Bit_RESET )
	{
		StopMotor ();
		StateFlags|=StateFlag_StopIfD1;
	}

	if ((CommandFlags&ComFlag_StopIfD2)&&(StateFlags&StateFlag_Motor))
	if (GPIO_ReadInputDataBit(SensorPortD2,SensorBitD2)==Bit_RESET )
	{
		StopMotor ();
		StateFlags|=StateFlag_StopIfD2;
	}

	if ((CommandFlags&ComFlag_StopIfD3)&&(StateFlags&StateFlag_Motor))
	if (GPIO_ReadInputDataBit(SensorPortD3,SensorBitD3)==Bit_RESET )
	{
		StopMotor ();
		StateFlags|=StateFlag_StopIfD3;
	}





}
void RequestDS18B20(void)
{
//Проверяем флаг разрешения измерений
	if ((CommandFlags&ComFlag_Izm_DS18B20)==0)
		{
		if (FLAGS&ComFlag_Izm_DS18B20_Loc)
			{OW_DeInit(); //выключили измерения
			FLAGS&=~ComFlag_Izm_DS18B20_Loc;
			TempDS18B20=ErrTemperature;
			}
		return;
		}
	if ((FLAGS&ComFlag_Izm_DS18B20_Loc)==0)
		{
		FLAGS|=ComFlag_Izm_DS18B20_Loc;//включили измерения
		OW_Init();
		CountTimerOW=0;

		}



if(CountTimerOW==0)
	{
	OW_StartTransaction(OW_KomPreobr);
	CountTimerOW=1000;





	}
if(CountTimerOW==100)
	{
	OW_StartTransaction(OW_KomReadTemp);
	 TempDS18B20=RezTemperature;
	}
CountTimerOW--;

}
void RequestSonar(void)
{
	//Структура настройки портов
//	GPIO_InitTypeDef PORT;
	if ((CommandFlags&ComFlag_Izm_Sonar)==0)
		{
		if(FLAGS&ComFlag_Izm_Sonar_Loc)
		{

			FLAGS&=~ComFlag_Izm_Sonar_Loc;
		}

		  return;
		}

if (FLAGS&ComFlag_Izm_Sonar_Loc)return;
FLAGS|=ComFlag_Izm_Sonar_Loc;

	SonarCount = 0;
	TIM16->CNT = 0;
	TIM16->CR1 |= TIM_CR1_CEN;


}
void SysTick_Handler(void)
{

	if(CountFuseNoTurn!=0)CountFuseNoTurn--;
	if(WaitForced!=0)WaitForced--;
	if(CountTimeCurrent!=0)	CountTimeCurrent--;
	RequestDS18B20();
}
int main(void) {


	initial();
	delay_ms(1);
	SysTick_Config(24000);
	InitVariable();
	OW_Init();
	StopMotor();
	StateFlags|=StateFlag_Reset; // Установим флаг перезапуска программы

// инициализация сторожевого таймера
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_4);
	IWDG_SetReload(0xfff);

	IWDG_Enable();


    while(1)
	{
    	main_modbus();
    	TestMotorCurrent();
    	RunsCommands();
    	RequestSonar();
    	//Sensor_Up=ADC1->JDR4*1.71;
    	Sensor_Up=ADC1->JDR3;
    	Densor_Imotor=((int)(ADC1->JDR3)-Cur_zero_offset)*Cur_gain/800;
    	if (Densor_Imotor<10)Densor_Imotor=0;
		//delay_ms(1);
		IWDG_ReloadCounter(); //сброс сторожевого таймера


	}




}

