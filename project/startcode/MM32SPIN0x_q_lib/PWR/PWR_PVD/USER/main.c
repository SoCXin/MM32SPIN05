/**
******************************************************************************
* @file     main.c
* @author   AE team
* @version  V1.1.1
* @date     15/05/2019
* @brief
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, MindMotion SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2019 MindMotion</center></h2>
*/
#include "HAL_device.h"
#include "HAL_conf.h"

void EXTI_Configuration(void);
void NVIC_Configuration(void);

#define LED1_Port             GPIOB
#define LED1_Pin              GPIO_Pin_5
#define LED2_Port             GPIOB
#define LED2_Pin              GPIO_Pin_4
#define LED3_Port             GPIOB
#define LED3_Pin              GPIO_Pin_3
#define LED4_Port             GPIOA
#define LED4_Pin              GPIO_Pin_15

#define LED1_ON()             GPIO_ResetBits(LED1_Port,LED1_Pin)
#define LED1_OFF()            GPIO_SetBits(LED1_Port,LED1_Pin)
#define LED1_TOGGLE()         (GPIO_ReadOutputDataBit(LED1_Port,LED1_Pin))?(GPIO_ResetBits(LED1_Port,LED1_Pin)):(GPIO_SetBits(LED1_Port,LED1_Pin))



#define LED2_ON()             GPIO_ResetBits(LED2_Port,LED2_Pin)
#define LED2_OFF()            GPIO_SetBits(LED2_Port,LED2_Pin)
#define LED2_TOGGLE()         (GPIO_ReadOutputDataBit(LED2_Port,LED2_Pin))?(GPIO_ResetBits(LED2_Port,LED2_Pin)):(GPIO_SetBits(LED2_Port,LED2_Pin))


#define LED3_ON()             GPIO_ResetBits(LED3_Port,LED3_Pin)
#define LED3_OFF()            GPIO_SetBits(LED3_Port,LED3_Pin)
#define LED3_TOGGLE()         (GPIO_ReadOutputDataBit(LED3_Port,LED3_Pin))?(GPIO_ResetBits(LED3_Port,LED3_Pin)):(GPIO_SetBits(LED3_Port,LED3_Pin))


#define LED4_ON()             GPIO_ResetBits(LED4_Port,LED4_Pin)
#define LED4_OFF()            GPIO_SetBits(LED4_Port,LED4_Pin)
#define LED4_TOGGLE()         (GPIO_ReadOutputDataBit(LED4_Port,LED4_Pin))?(GPIO_ResetBits(LED4_Port,LED4_Pin)):(GPIO_SetBits(LED4_Port,LED4_Pin))
static void deleyNop(u32 DlyTime);
void GPIO_Clock_Set(GPIO_TypeDef* GPIOx, FunctionalState NewState);
void LED_Init(void);

u8 lowpowerflag = 0;

/*******************************************************************************
* @name   : main
* @brief  : Adjust mcu voltage generate PVD flag
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    u16 i;
    u32 dly;
    deleyNop(10000);

    LED_Init();

    for(i = 0; i < 10; i++) {
        LED2_TOGGLE();
        deleyNop(3000);
    }
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    /* Configure EXTI Line to generate an interrupt on falling edge */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);




    EXTI_Configuration();

    /* NVIC configuration */
    NVIC_Configuration();

    /* Configure the PVD Level to 1.8 ~ 4.8V*/
//    PWR_PVDLevelConfig(PWR_PVDLevel_1V8);
//    PWR_PVDLevelConfig(PWR_PVDLevel_2V1);
//    PWR_PVDLevelConfig(PWR_PVDLevel_2V4);
//    PWR_PVDLevelConfig(PWR_PVDLevel_2V7);
    PWR_PVDLevelConfig(PWR_PVDLevel_3V0);
//    PWR_PVDLevelConfig(PWR_PVDLevel_3V3);
//    PWR_PVDLevelConfig(PWR_PVDLevel_3V6);
//    PWR_PVDLevelConfig(PWR_PVDLevel_3V9);
//    PWR_PVDLevelConfig(PWR_PVDLevel_4V2);
//    PWR_PVDLevelConfig(PWR_PVDLevel_4V5);
//    PWR_PVDLevelConfig(PWR_PVDLevel_4V8);
    /* Enable the PVD Output */
    PWR_PVDCmd(ENABLE);

    dly = 10000;
    while(1) {
        if(lowpowerflag == 1) {
            dly = 3000; //fast Frequence toggle LED
        }
        LED2_TOGGLE();
        deleyNop(dly);
    }
}

/*******************************************************************************
* @name   : PVD_IRQHandler
* @brief  : PVD IRQ
* @param  : None
* @retval : void
*******************************************************************************/
void PVD_IRQHandler(void)
{

    if(EXTI_GetITStatus(EXTI_Line16) != RESET) {
        /* Clear the EXTI line pending bit */
        EXTI_ClearITPendingBit(EXTI_Line16);
        lowpowerflag = 1;
    }
}
/*******************************************************************************
* @name   : EXTI_Configuration
* @brief  : EXTI config
* @param  : None
* @retval : void
*******************************************************************************/
void EXTI_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure EXTI Line16(PVD Output) to generate an interrupt on rising and
       falling edges */
    EXTI_ClearITPendingBit(EXTI_Line16);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;// PVD map to EXTI_Line16
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
* @name   : NVIC_Configuration
* @brief  : NVIC config
* @param  : None
* @retval : void
*******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the PVD Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



static void deleyNop(u32 DlyTime)
{
    u32 i, j;
    for(i = 0; i < DlyTime; i++) {
        for(j = 0; j < 100; j++) {
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
        }
    }
}


/*******************************************************************************
* @name   : GPIO_Clock_Set
* @brief  : RCC clock set
* @param  : Portx , State
* @retval : void
*******************************************************************************/
void GPIO_Clock_Set(GPIO_TypeDef* GPIOx, FunctionalState NewState)
{

    if(GPIOx == GPIOA) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, NewState);
    }
    if(GPIOx == GPIOB) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, NewState);
    }
    if(GPIOx == GPIOC) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, NewState);
    }
    if(GPIOx == GPIOD) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, NewState);
    }
}
/*******************************************************************************
**函数信息 ：LED_Init(void)
**功能描述 ：LED初始化
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void LED_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_Clock_Set(GPIOA, ENABLE);                                              //开启GPIOA时钟
    GPIO_Clock_Set(GPIOB, ENABLE);                                              //开启GPIOB时钟
    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    LED1_ON();
    LED2_ON();
    LED3_ON();
    LED4_ON();
}

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/
/*-------------------------(C) COPYRIGHT 2019 MindMotion ----------------------*/
