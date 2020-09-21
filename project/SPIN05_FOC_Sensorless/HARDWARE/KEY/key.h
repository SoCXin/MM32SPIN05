#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//开发板
//按键驱动代码	   
//////////////////////////////////////////////////////////////////////////////////   	 

#define KEY_ONOFF  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)//读取按键3
#define KEY_BRAKE  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)//读取按键4


#define KEY_ONOFF_PRES	3		//KEY3 
#define KEY_BRAKE_PRES	4		//KEY4 
void KEY_Init(void);        //IO初始化
u8 KEY_Scan(u8 mode);  	//按键扫描函数					    
#endif
