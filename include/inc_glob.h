#ifndef __INC_GLOB_H
#define __INC_GLOB_H


#define CommandFlags	    res_table[MBReg_CommandFlags] // Командные флажки
#define StateFlags	    	res_table[MBReg_StateFlags] //  Флажки состояния
#define Sensor_Up		    res_table[MBReg_Sensor_Up]     //измерянное напряжение питания
#define Densor_Imotor	    res_table[MBReg_Densor_Imotor] //измерянный ток мотора,мА (ниже 500 мА не учитывать)
#define Cur_zero_offset	    res_table[MBReg_Cur_zero_offset] //коррекция смещения нуля дат.тока (вычитается при измерении)
#define Cur_gain		    res_table[MBReg_Cur_gain]        //усиление усилителя датчика тока


//buffer uart
#define BUF_SZ 256
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //max quantity of words in responce


// биты командных флажков CommandFlags
#define ComFlag_Door				0x0001		//
#define ComFlag_Gate			    0x0002		//
#define ComFlag_Light	    		0x0004		//
#define ComFlag_IR_Mod	    		0x0008		//

// Биты флажков состояния StateFlags
#define StateFlag_Door				0x0001    	//
#define StateFlag_Gate          	0x0002		//
#define StateFlag_ifCard			0x0004		//

#define StateFlag_Reset				0x0200		//

#define OBJ_SZ 50
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
 MBReg_CardsData_0,
 MBReg_CardsData_1,
 MBReg_CardsData_2,
 MBReg_CardsData_Save_0,
 MBReg_CardsData_Save_1,
 MBReg_CardsData_Save_2,
 MBReg_CardsData_Save_3,
 MBReg_CardsData_Save_4,
 MBReg_CardsData_Save_5,
 MBReg_CardsData_Save_6,
 MBReg_CardsData_Save_7,
 MBReg_CardsData_Save_8,
 MBReg_CardsData_Save_9,
 MBReg_CardsData_Save_10,
 MBReg_CardsData_Save_11,
 MBReg_CardsData_Save_12,
 MBReg_CardsData_Save_13,
 MBReg_CardsData_Save_14,
 MBReg_CardsData_Save_15,
 MBReg_CardsData_Save_16,
 MBReg_CardsData_Save_17,
 MBReg_CardsData_Save_18,
 MBReg_CardsData_Save_19,
 MBReg_CardsData_Save_20,
 MBReg_CardsData_Save_21,
 MBReg_CardsData_Save_22,
 MBReg_CardsData_Save_23,
 MBReg_CardsData_Save_24,
 MBReg_CardsData_Save_25,
 MBReg_CardsData_Save_26,
 MBReg_CardsData_Save_27,
 MBReg_CardsData_Save_28,
 MBReg_CardsData_Save_29



 }NumberRegModbas;

#endif /* __INC_GLOB_H */
