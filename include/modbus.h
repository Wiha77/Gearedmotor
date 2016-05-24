/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_H
#define __MODBUS_H

#include "uart.h"


#define OBJ_SZ 50

int16_t res_table[OBJ_SZ];
//OBJECT ARRAY WHERE READING AND WRITING OCCURS
 typedef enum
 {
 MBReg_AdrrModbus=0,
 MBReg_CommandFlags,
 MBReg_StateFlags,
 MBReg_Sensor_Up,
 MBReg_Densor_Imotor,
 MBReg_Cur_zero_offset,
 MBReg_Cur_gain,
 MBReg_CardsData


 }NumberRegModbas;


#define CommandFlags	    res_table[MBReg_CommandFlags] // Командные флажки
#define StateFlags	    	res_table[MBReg_StateFlags] //  Флажки состояния
#define Sensor_Up		    res_table[MBReg_Sensor_Up]     //измерянное напряжение питания
#define Densor_Imotor	    res_table[MBReg_Densor_Imotor] //измерянный ток мотора,мА (ниже 500 мА не учитывать)
#define Cur_zero_offset	    res_table[MBReg_Cur_zero_offset] //коррекция смещения нуля дат.тока (вычитается при измерении)
#define Cur_gain		    res_table[MBReg_Cur_gain]        //усиление усилителя датчика тока


//buffer uart
#define BUF_SZ 256
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //max quantity of words in responce

 extern UART_DATA uart2;
extern unsigned char *modbus_addr;

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
