#include "pwr_pvd.h"
#include "Whole_Motor_Parameters.h"

//bool FLAG4V2 = FALSE;
//bool FLAG2V7 = FALSE;
/********************************************************************************************************
**函数信息 :                       
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void PVD_IRQHandler(void)
{
  if( RESET != EXTI_GetITStatus(EXTI_Line16))
  {
		//屏蔽中断线
    EXTI->IMR &= (~ 0x10000);
		
    //VDD低于PVD阈值
    if( 0x04 ==(PWR->CSR & 0x04))
    {
      //__set_FAULTMASK(1);//关闭所有中断
      NVIC_SystemReset();//复位
    }
    
    EXTI_ClearITPendingBit(EXTI_Line16);
		
		//开放中断线
    EXTI->IMR |= 0x10000;
  }
}
/********************************************************************************************************
**函数信息 :                       
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void PVD_EXTI_Init(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);//外部中断，需要使能AFIO时
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line16;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	PWR_PVDLevelConfig(PVD_VOLTAGE); //Generate PVD Interrupt if VDD voltage under 3.0V 
  PWR_PVDCmd(ENABLE);
  
}

