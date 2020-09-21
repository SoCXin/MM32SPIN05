#ifndef __PWM_H_
#define __PWM_H_
#include "HAL_device.h"
#include "HAL_conf.h"


void TIM1_PWM_Init(uint16_t u16Period,uint16_t u16Prescaler,uint8_t u8DeadTime);
void TIM3_PWM_Init(uint16_t u16Period,uint16_t u16Prescaler);
void TIM1_BKIN_External_Input_Pin_Init(void);
void DMA_Timer3CC1TrigDMA_init(void);
void DMA_ADCTrigDMA_init(void);
#endif
