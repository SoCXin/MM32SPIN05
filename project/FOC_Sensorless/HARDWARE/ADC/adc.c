#include "adc.h"
#include "led.h"
#include "pwm.h" 
#include "sys.h"
#include "Whole_Motor_Parameters.h"

void GPIO_Configuration_ADC(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);   
    
    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_7|GPIO_Pin_6|GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_3|GPIO_Pin_2|GPIO_Pin_1|GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    /*将PAx配置为模拟输入*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_1|GPIO_Pin_0;
    /*将PB0配置为模拟输入*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/********************************************************************************************************
**函数信息 ：void ADC1_Initial(void)      
**功能描述 ：配置ADC1单次转换模式
**输入参数 ：none
**输出参数 ：无
**    备注 ：
********************************************************************************************************/
void ADC1_Initial(void)
{	
	  ADC_InitTypeDef  ADC_InitStructure;
	
	  #ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
	  NVIC_InitTypeDef NVIC_InitStructure;
    #endif
	
    GPIO_Configuration_ADC();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	  ADC_InitStructure.ADC_PRESCARE = ADC_PCLK2_PRESCARE_2;//for 48MHz
	
    /* Initialize the ADC_PRESCARE values */
		#ifdef MM32SPIN05
		 ADC_InitStructure.ADC_PRESCARE = ADC_PCLK2_PRESCARE_4;//for 72MHz
		#endif
   	
    /* Initialize the ADC_Mode member */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Single_Period;
    /* Initialize the ADC_ContinuousConvMode member */
    //ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    /* Initialize the ADC_DataAlign member */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		
    /* Initialize the ADC_ExternalTrigConv member */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_CC1;// TIM3 CCR1 as external Trigger source
		
		#ifdef ENABLE_TIM1_CC4_TO_TRIG_ADC_FOR_2_SHUNTR//use TIM1_CC4 to trigger ADC in 2 shunt R phase current sensing
			ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;// TIM1 CCR4 as external Trigger source
		#endif
		
		#ifdef ENABLE_TIM1_CC4_CC5_TO_TRIG_ADC1_FOR_1_SHUNTR//use TIM1_CC4_CC5 to trigger ADC1 in 1 shunt R phase current sensing
			ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4_CC5;// TIM1 CCR4 CCR5 as external Trigger source
		#endif
		
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_Init(ADC1, &ADC_InitStructure);
	
	  ADC_Cmd(ADC1, ENABLE);
	
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_1_5Cycles);//for setting the sampling time (all channel)
//----------------------------------------------------------------------------------------------------------------------------------		
		#ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
		/* Enable ADC Intterupt */
    ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	  ADC_ExternalTrigConvCmd(ADC1,ENABLE);
	  NVIC_InitStructure.NVIC_IRQChannel = 12;//ADC_COMP_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;//0:highest priority, 3:lowest priority
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//IRQ通道使能
	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
		#endif
//-----------------------------------------------------------------------------------------------------------------------------------		
    ADC_Cmd(ADC1, ENABLE);
		
		#ifdef ENABLE_OVER_CURRENT_COMP1_PROTECTION
    ADC_TempSensorCmd(ENABLE); //Enable internal temperature sensor		
		ADC_VrefintCmd(ENABLE);    //Enable Vrefint 1.2V
		#endif
}

void ADC1_Channel_Setup_to_1ShuntR_Current_Only(void)
{   
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_1_5Cycles);//for setting the sampling time (all channel)
	  ADC1->ADCHS &= CHEN_DISABLE;//disable all channels of ADC
	  ADC1->ADCHS |= ADC_1_SHUNT_R_CHANNEL_ENABLE;//enable 1 shunt R adc channel, for 3 phase current sense
}

void ADC1_Channel_Setup_to_2Phase_Current_and_Isum_Only(void)
{   
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_1_5Cycles);//for setting the sampling time (all channel)
	  ADC1->ADCHS &= CHEN_DISABLE;//disable all channels of ADC
	  ADC1->ADCHS |= ADC_ISUM_CHANNEL_ENABLE;//enable ADC measure Isum current channel,for getting MOSFET Total current consumption 
	  ADC1->ADCHS |= ADC_2_SHUNT_R_CHANNEL_U_ENABLE;//enable ADC channel U, IU phase current sense
	  ADC1->ADCHS |= ADC_2_SHUNT_R_CHANNEL_V_ENABLE;//enable ADC channel V, IV phase current sense
}

void ADC1_Channel_Setup_to_3Phase_Current_and_Isum_Only(void)
{   
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_1_5Cycles);//for setting the sampling time (all channel)
	  ADC1->ADCHS &= CHEN_DISABLE;//disable all channels of ADC
	  ADC1->ADCHS |= ADC_ISUM_CHANNEL_ENABLE;//enable ADC measure Isum current channel,for getting MOSFET Total current consumption 
	  ADC1->ADCHS |= ADC_2_SHUNT_R_CHANNEL_U_ENABLE;//enable ADC channel U, IU phase current sense
	  ADC1->ADCHS |= ADC_2_SHUNT_R_CHANNEL_V_ENABLE;//enable ADC channel V, IV phase current sense
	  ADC1->ADCHS |= ADC_3_SHUNT_R_CHANNEL_W_ENABLE;//enable ADC channel W, IW phase current sense
}

void ADC1_Channel_Setup_to_2Phase_Current_Only(void)
{   
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_1_5Cycles);//for setting the sampling time (all channel)
	  ADC1->ADCHS &= CHEN_DISABLE;//disable all channels of ADC
	  ADC1->ADCHS |= ADC_2_SHUNT_R_CHANNEL_U_ENABLE;//enable ADC channel U, IU phase current sense
	  ADC1->ADCHS |= ADC_2_SHUNT_R_CHANNEL_V_ENABLE;//enable ADC channel V, IV phase current sense
}

void ADC1_Channel_Setup_to_3Phase_Current_Only(void)
{   
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_1_5Cycles);//for setting the sampling time (all channel)
	  ADC1->ADCHS &= CHEN_DISABLE;//disable all channels of ADC
	  ADC1->ADCHS |= ADC_2_SHUNT_R_CHANNEL_U_ENABLE;//enable ADC channel U, IU phase current sense
	  ADC1->ADCHS |= ADC_2_SHUNT_R_CHANNEL_V_ENABLE;//enable ADC channel V, IV phase current sense
	  ADC1->ADCHS |= ADC_3_SHUNT_R_CHANNEL_W_ENABLE;//enable ADC channel W, IW phase current sense
}

void ADC1_Channel_Setup_Add_BEMF_AB(void)
{   
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_13_5Cycles);//for setting the sampling time (all channel)
	  ADC1->ADCHS &= CHEN_DISABLE;//disable all channels of ADC
	  ADC1->ADCHS |= ADC_BEMF_A_CHANNEL_ENBALE;//enable ADC BEMF A channel, bemf U sense
	  ADC1->ADCHS |= ADC_BEMF_B_CHANNEL_ENBALE;//enable ADC BEMF B channel, bemf V sense
	  ADC1->ADCHS |= ADC_SPEED_CMD_IN_CHANNEL_ENABLE;//enable ADC speed cmd input channel, for speed command input
}
void ADC1_Channel_Setup_Without_Phase_Current(void)
{   
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_7_5Cycles);//for setting the sampling time (all channel)
	  ADC1->ADCHS &= CHEN_DISABLE;//disable all channels of ADC
	  ADC1->ADCHS |= ADC_SPEED_CMD_IN_CHANNEL_ENABLE;//enable ADC speed cmd input channel, for speed command input
	  ADC1->ADCHS |= ADC_VBUS_CHANNEL_ENABLE;//enable ADC DC Bus voltage input channel, for measure DC bus voltage
	
	  #ifdef ENABLE_ISUM_MEASUREMENT
	  ADC1->ADCHS |= ADC_ISUM_CHANNEL_ENABLE;//enable ADC measure Isum current channel,for getting MOSFET Total current consumption 
	  #endif
}

/********************************************************************************************************
**函数信息 ：void ADC1_SingleChannel(uint8_t ADC_Channel_x)      
**功能描述 ：配置ADC1单次转换模式
**输入参数 ：ADC_Channel_x , x为0~8
**输出参数 ：无
**    备注 ：
********************************************************************************************************/
void ADC1_SingleChannel(uint8_t ADC_Channel_x)
{      
    /*屏蔽所有通道*/
    ADC_RegularChannelConfig(ADC1, DISABLE_ALL_CHANNEL , 0, 0); 
    /*使能选中通道,后面参数保留*/
    ADC_RegularChannelConfig(ADC1, ADC_Channel_x, 0, ADC_SampleTime_7_5Cycles);       
}
/********************************************************************************************************
**函数信息 ：ADC1_SingleChannel_Get()       
**功能描述 ：获取ADC1转换数据
**输入参数 ：ADC_Channel_x , x为0~8
*puiADData ,ADC1实际转换数据
**输出参数 ：ucStatus ,0 表示数据获取失败,1 表示成功
********************************************************************************************************/
u16 ADC1_SingleChannel_Get(uint8_t ADC_Channel_x)
{ 
    u16 puiADData;
    /*ADCR寄存器的ADST位使能，软件启动转换*/
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==0);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    puiADData=ADC_GetConversionValue(ADC1);
    return puiADData;
}

/********************************************************************************************************
**函数信息 ：Get_Adc_Average_4times(uint8_t ADC_Channel_x)     
**功能描述 ：获取几次ADC1采样值的平均值
**输入参数 ：ADC_Channel_x , x为0~8
**输出参数 ：puiADData为ADC读到的值
********************************************************************************************************/
u16 Get_Adc_Average_4times(uint8_t ADC_Channel_x)
{
    u32 temp_val;
    u8 t;
	
	  temp_val=0;
    for(t=0;t<4;t++)
    {			 
      temp_val+=ADC1_SingleChannel_Get(ADC_Channel_x);		  
    }	
		temp_val = temp_val>>2;
		
    return temp_val;
} 	

/********************************************************************************************************
**函数信息 ：Get_Adc_Average_2times(uint8_t ADC_Channel_x)     
**功能描述 ：获取几次ADC1采样值的平均值
**输入参数 ：ADC_Channel_x , x为0~8
**输出参数 ：puiADData为ADC读到的值
********************************************************************************************************/
u16 Get_Adc_Average_2times(uint8_t ADC_Channel_x)
{
    u32 temp_val;
    u8 t;
	
	  temp_val=0;
    for(t=0;t<2;t++)
    {			 
      temp_val+=ADC1_SingleChannel_Get(ADC_Channel_x);		  
    }	
		temp_val = temp_val>>1;
		
    return temp_val;
} 	
