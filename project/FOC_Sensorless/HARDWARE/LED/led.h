#ifndef __LED_H
#define __LED_H	 
#include "HAL_conf.h"

//////////////////////////////////////////////////////////////////////////////////	 
//开发板
//LED驱动代码	   
////////////////////////////////////////////////////////////////////////////////// 
//#define LED4_ON()  GPIO_ResetBits(GPIOA,GPIO_Pin_15)	// PA15
//#define LED4_OFF()  GPIO_SetBits(GPIOA,GPIO_Pin_15)	// PA15
//#define LED4_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_15))?(GPIO_ResetBits(GPIOA,GPIO_Pin_15)):(GPIO_SetBits(GPIOA,GPIO_Pin_15))	// PA15

//#define LED3_ON()  GPIO_ResetBits(GPIOB,GPIO_Pin_3)	// PB3
//#define LED3_OFF()  GPIO_SetBits(GPIOB,GPIO_Pin_3)	// PB3
//#define LED3_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3))?(GPIO_ResetBits(GPIOB,GPIO_Pin_3)):(GPIO_SetBits(GPIOB,GPIO_Pin_3))	// PB3

//#define LED2_ON()  GPIO_ResetBits(GPIOB,GPIO_Pin_4)	// PB4
//#define LED2_OFF()  GPIO_SetBits(GPIOB,GPIO_Pin_4)	// PB4
//#define LED2_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_4))?(GPIO_ResetBits(GPIOB,GPIO_Pin_4)):(GPIO_SetBits(GPIOB,GPIO_Pin_4))	// PB4

void LED_Init(void);//初始化

		 				    
#endif
