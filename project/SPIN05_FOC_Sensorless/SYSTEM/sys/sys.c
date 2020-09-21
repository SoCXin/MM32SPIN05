#include "sys.h"

/********************************************************************************************************
**函数信息 ：System_Clock_Init(u8 PLL)                     
**功能描述 ：外部时钟倍频后作为系统时钟
**输入参数 ：PLL：倍频倍数
**输出参数 ：无
********************************************************************************************************/
void System_Clock_Init(u8 PLL)
{
    unsigned char temp=0;   
    RCC->CR|=RCC_CR_HSEON;  //外部高速时钟使能HSEON
    while(!(RCC->CR&RCC_CR_HSERDY));//等待外部时钟就绪
    RCC->CFGR=RCC_CFGR_PPRE1_2; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
    
    RCC->CFGR|=RCC_CFGR_PLLSRC;	  //PLLSRC ON 
    RCC->CR &=~(RCC_CR_PLLON);		//清PLL//	RCC->CR &=~(7<<20);		//清PLL
    
    RCC->CR &=~(0x1f<<26);	
    RCC->CR|=(PLL - 1) << 26;   //设置PLL值 2~16
    
    FLASH->ACR|=FLASH_ACR_LATENCY_1|FLASH_ACR_PRFTBE|FLASH_ACR_PRFTBS;	  //FLASH 2个延时周期
    
    RCC->CR|=RCC_CR_PLLON;  //PLLON
    while(!(RCC->CR&RCC_CR_PLLRDY));//等待PLL锁定
    RCC->CFGR|=RCC_CFGR_SW_PLL;//PLL作为系统时钟	 
    while(temp!=0x02)     //等待PLL作为系统时钟设置成功
    {    
        temp=RCC->CFGR>>2;
        temp&=0x03;
    }    
}	



