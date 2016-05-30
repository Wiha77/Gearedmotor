/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_H
#define __MODBUS_H

#include "uart.h"
#include "inc_glob.h"



int16_t res_table[OBJ_SZ];




 extern UART_DATA uart2;
extern unsigned char *modbus_addr;

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/

void MODBUS_SLAVE(UART_DATA *MODBUS);
unsigned int Crc16(unsigned char *ptrByte, int byte_cnt);
void TX_03_04(UART_DATA *MODBUS);
void TX_06(UART_DATA *MODBUS);
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type);
void MODBUS_receiver_on(UART_DATA *uart);
void TestWriteVarEEPROM(unsigned int temp);

#endif /* __MODBUS_H */
