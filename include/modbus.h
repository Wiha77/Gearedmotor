/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_H
#define __MODBUS_H

#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define OBJ_SZ 50
#define SETUP 4 //setup elements

//PARAMETERRS ARRAY 0 PARAMETER = MODBUS ADDRESS
unsigned char SET_PAR[SETUP];


s16 res_table[OBJ_SZ];
//OBJECT ARRAY WHERE READING AND WRITING OCCURS
 typedef enum
 {



 MBReg_AdrrModbus=0,
 MBReg_proporc,
 MBReg_integral,
 MBReg_diferencial,
 MBReg_K_proporc,
 MBReg_K_integral,
 MBReg_K_diferencial,
 MBReg_PIDMaxErr,
 MBReg_MaxCurrent,
 MBReg_TimeMaxCurrent,
 MBReg_CommandFlags,
 MBReg_StateFlags,
 MBReg_NumberOfTurns,
 MBReg_MotorSpeedPWM ,
 MBReg_MotorSpeed,
 MBReg_PIDrp,
 MBReg_Sensor_Up,
 MBReg_Densor_Imotor,
 MBReg_TempDS18B20,
 MBReg_SonarData,
 MBReg_Cur_zero_offset,
 MBReg_Cur_gain,
 MBReg_PID_MaxPWM,
 MBReg_PID_MinPWM,
 MBReg_PID_MaxTurns,
 MBReg_PID_MinTurns,
 MBReg_PID_KickTurns,
 MBReg_PID_KickPWM,
 MBReg_PID_PWM_zerro,
 MBReg_PID_PWM_gain

 }NumberRegModbas;

#define proporc         	res_table[MBReg_proporc] //вычисленная пророрциональная состовляющая ПИД регулятора
#define integral        	res_table[MBReg_integral] //вычисленная интегральная состовляющая ПИД регулятора
#define diferencial     	res_table[MBReg_diferencial] //вычисленная дифференциальная состовляющая ПИД регулятора
#define K_proporc         	res_table[MBReg_K_proporc] //коэфициэнт пророрциональной состовляющая ПИД регулятора
#define K_integral        	res_table[MBReg_K_integral] //коэфициэнт интегральной состовляющая ПИД регулятора
#define K_diferencial     	res_table[MBReg_K_diferencial] //коэфициэнт дифференциальной состовляющая ПИД регулятора
#define PIDMaxErr			res_table[MBReg_PIDMaxErr]//максимальная ощибка для ПИД регулятора
#define MotorSpeedPWM     	res_table[MBReg_MotorSpeedPWM] //текущее значение шима ПИД регулятора(MinPWM-MaxPWM)
#define MotorSpeed     		res_table[MBReg_MotorSpeed] //заданная  скорость якоря мотора (об/мин) ПИД регулятора
#define PIDrp	     		res_table[MBReg_PIDrp] //реальная(измерянная)  скорость якоря мотора (об/мин) ПИД регулятора
#define NumberOfTurns	    res_table[MBReg_NumberOfTurns] //заданное колво оборотов,
#define MaxCurrent	    	res_table[MBReg_MaxCurrent] // Максимальный ток двигателя, мА
#define TimeMaxCurrent		res_table[MBReg_TimeMaxCurrent] // Задержка до срабатываания защиты по току, мС
#define CommandFlags	    res_table[MBReg_CommandFlags] // Командные флажки
#define StateFlags	    	res_table[MBReg_StateFlags] //  Флажки состояния


#define Sensor_Up		res_table[MBReg_Sensor_Up]     //измерянное напряжение питания
#define Densor_Imotor	res_table[MBReg_Densor_Imotor] //измерянный ток мотора,мА (ниже 500 мА не учитывать)
#define TempDS18B20		res_table[MBReg_TempDS18B20]   //Температура измерянная датчиком, (0.1С) если 10000 то нет датчикаа
#define SonarData		res_table[MBReg_SonarData]     //Расстояние измерянное сонаром
#define Cur_zero_offset	res_table[MBReg_Cur_zero_offset] //коррекция смещения нуля дат.тока (вычитается при измерении)
#define Cur_gain		res_table[MBReg_Cur_gain]        //усиление усилителя датчика тока
#define  PID_MaxPWM		res_table[MBReg_PID_MaxPWM]			//Максимальное заполнение(для работы алгоритма пид)по этим параметрам производится грубый расчет заполнения
#define  PID_MinPWM		res_table[MBReg_PID_MinPWM]			//Минимальное заполнение
#define  PID_MaxTurns	res_table[MBReg_PID_MaxTurns]		//Максимальные обороты
#define  PID_MinTurns	res_table[MBReg_PID_MinTurns]		//Минимальные обороты

//buffer uart
#define BUF_SZ 256
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //max quantity of words in responce

//uart structure
typedef struct {
unsigned char buffer[BUF_SZ];
unsigned int rxtimer;
unsigned char rxcnt;
unsigned char txcnt;
unsigned char txlen;
unsigned char rxgap;
unsigned char protocol;
unsigned char delay;
} UART_DATA;

UART_DATA uart2;
/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void SetupUSART2(void);
void MODBUS_SLAVE(UART_DATA *MODBUS);
unsigned int Crc16(unsigned char *ptrByte, int byte_cnt);
void TX_03_04(UART_DATA *MODBUS);
void TX_06(UART_DATA *MODBUS);
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type);
void main_modbus(void);
void TestWriteVarEEPROM(unsigned int temp);

#endif /* __MODBUS_H */
