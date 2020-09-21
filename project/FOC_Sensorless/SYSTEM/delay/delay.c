#include "delay.h"
#include "led.h"
#define USE_SYSTICK_DELAY 1	 //1为使用systick产生延时，0为普通延时

/*如果使用systick作为延时，一定要先初始化systick，即先调用函数systick_init(u8 pclk2)*/


static __IO uint32_t TimingDelay;
__IO uint32_t u32SystemTick100us;
extern u32 SystemCoreClock;
/********************************************************************************************************
**函数信息 ：systick_init()                 
**功能描述 ：systick延时函数初始化
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void systick_init()
{
    if (SysTick_Config(SystemCoreClock / 10000))//1000 means 1ms, 2000 means 0.5ms, 10000 means 100us
    { 
        /* Capture error */ 
        while (1);
    }
    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x2);//SysTick中断优先级设置, 0:highest priority, 3:lowest priority
}

/********************************************************************************************************
**函数信息 ：SysTick_Handler(void)                    
**功能描述 ：进入该中断函数后，Systick进行递减
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SysTick_Handler(void)
{
	  u32SystemTick100us++;
}

/********************************************************************************************************
**函数信息 ：delay_us(__IO uint32_t nTime)                 
**功能描述 ：程序应用调用延时
**输入参数 ：nTime：延时
**输出参数 ：无
********************************************************************************************************/
void delay_us(__IO uint32_t nTime)
{		
    u16 i=0;   
    while(nTime--)   
    {   
        i= 3;  //自己定义//20190402 FOR SPIN05 72MHz      
        while(i--);       
    }					 
}
/********************************************************************************************************
**函数信息 ：delay_ms(__IO uint32_t nTime)                     
**功能描述 ：程序应用调用延时
**输入参数 ：nTime：延时
**输出参数 ：无
********************************************************************************************************/
void delay_ms(__IO uint32_t nTime)
{	 		  	  
    u16 i=0;   
    while(nTime--)   
    {      
        i=4350;  //自己定义//20190402 FOR SPIN05 72MHz    
        while(i--);        
    }	  	    
} 









































