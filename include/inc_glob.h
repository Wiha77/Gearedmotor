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
#define ComFlag_Light1				0x0001		//
#define ComFlag_Light2			    0x0002		//
#define ComFlag_Light3	    		0x0004		//
#define ComFlag_Light4	    		0x0008		//
#define ComFlag_K2_Fix				0x0100    	//A10
#define ComFlag_K4_Fix       	    0x0200		//A8
#define ComFlag_K6_Fix		    	0x0400		//B3
#define ComFlag_K7_Fix	   		    0x0800		//B5
#define ComFlag_K8_Fix		   		0x1000		//B7
#define ComFlag_D6_Fix		    	0x2000		//
#define ComFlag_D7_Fix		    	0x4000		//

// Биты флажков состояния StateFlags
#define StateFlag_K2				0x0001    	//
#define StateFlag_K4          	    0x0002		//
#define StateFlag_K6			    0x0004		//
#define StateFlag_K7			    0x0008		//
#define StateFlag_K8			    0x0010		//
#define StateFlag_D6			    0x0020		//
#define StateFlag_D7			    0x0040		//
#define StateFlag_K2_Fix			0x0100    	//
#define StateFlag_K4_Fix       	    0x0200		//
#define StateFlag_K6_Fix		    0x0400		//
#define StateFlag_K7_Fix		    0x0800		//
#define StateFlag_K8_Fix		    0x1000		//
#define StateFlag_D6_Fix		    0x2000		//
#define StateFlag_D7_Fix		    0x4000		//

#define StateFlag_Reset				0x8000		//

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
 MBReg_Cur_gain

 }NumberRegModbas;

#endif /* __INC_GLOB_H */
