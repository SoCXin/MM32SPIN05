#ifndef __ADC_H_
#define __ADC_H_
#include "HAL_device.h" 
#include "HAL_conf.h"

//#define DISABLE_ALL_CHANNEL     0x55  //9 //20171122 changed by jason

void ADC1_Initial(void);
void ADC1_SingleChannel(uint8_t ADC_Channel_x);
void ADC1_Channel_Setup_to_1ShuntR_Current_Only(void);
void ADC1_Channel_Setup_to_2Phase_Current_Only(void);
void ADC1_Channel_Setup_to_2Phase_Current_and_Isum_Only(void);
void ADC1_Channel_Setup_to_3Phase_Current_Only(void);
void ADC1_Channel_Setup_to_3Phase_Current_and_Isum_Only(void);//20190626
void ADC1_Channel_Setup_Add_BEMF_AB(void);//20190626
void ADC1_Channel_Setup_Without_Phase_Current(void);
u16 ADC1_SingleChannel_Get(uint8_t ADC_Channel_x);
u16 Get_Adc_Average_2times(uint8_t ADC_Channel_x);
u16 Get_Adc_Average_4times(uint8_t ADC_Channel_x);

#endif
