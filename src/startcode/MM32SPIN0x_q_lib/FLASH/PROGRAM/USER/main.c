/**
******************************************************************************
* @file     main.c
* @author   AE team
* @version  V1.0.3
* @date     10/04/2019
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
#include "stdio.h"

char printBuf[100];
extern u32 SystemCoreClock;
void delay_init(void);
void delay_ms(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
void LED_Init(void);
void UartSendGroup(u8* buf, u16 len);
void uart_initwBaudRate(u32 bound);
u8 FLASH_Program(void);

static __IO uint32_t TimingDelay;
#define BANK1_WRITE_START_ADDR  ((uint32_t)0x08002800)
uint32_t Data = 0x12345679;

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;

#define LED4_ON()  GPIO_ResetBits(GPIOA,GPIO_Pin_15)	// PA15
#define LED4_OFF()  GPIO_SetBits(GPIOA,GPIO_Pin_15)	// PA15
#define LED4_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_15))?(GPIO_ResetBits(GPIOA,GPIO_Pin_15)):(GPIO_SetBits(GPIOA,GPIO_Pin_15)) // PA15

#define LED3_ON()  GPIO_ResetBits(GPIOB,GPIO_Pin_3)	// PB3
#define LED3_OFF()  GPIO_SetBits(GPIOB,GPIO_Pin_3)	// PB3
#define LED3_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3))?(GPIO_ResetBits(GPIOB,GPIO_Pin_3)):(GPIO_SetBits(GPIOB,GPIO_Pin_3))	// PB3

#define LED2_ON()  GPIO_ResetBits(GPIOB,GPIO_Pin_4)	// PB4
#define LED2_OFF()  GPIO_SetBits(GPIOB,GPIO_Pin_4)	// PB4
#define LED2_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_4))?(GPIO_ResetBits(GPIOB,GPIO_Pin_4)):(GPIO_SetBits(GPIOB,GPIO_Pin_4))	// PB4

#define LED1_ON()  GPIO_ResetBits(GPIOB,GPIO_Pin_5)	// PB5
#define LED1_OFF()  GPIO_SetBits(GPIOB,GPIO_Pin_5)	// PB5
#define LED1_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_5))?(GPIO_ResetBits(GPIOB,GPIO_Pin_5)):(GPIO_SetBits(GPIOB,GPIO_Pin_5))	// PB5

/********************************************************************************************************
**������Ϣ ��main(void)
**�������� ��
**������� ����
**������� ����
********************************************************************************************************/
int main(void)
{
    //���س���ǰҪѡ��ȫ����
    u8 t;
    delay_init();
    LED_Init();
    uart_initwBaudRate(115200);
    t = FLASH_Program();

    if(t == 0)
        while(1) {
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "program ok\r\n"));
            LED1_TOGGLE();
            LED2_TOGGLE();
            LED3_TOGGLE();
            LED4_TOGGLE();
            delay_ms(1000);
        } else
        while(1) {
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "program error\r\n"));
            delay_ms(1000);
        }
}

/********************************************************************************************************
**������Ϣ ��delay_init(void)
**�������� ��systick��ʱ������ʼ��
**������� ����
**������� ����
********************************************************************************************************/
void delay_init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000)) {
        /* Capture error */
        while (1);
    }
    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);                                        //SysTick�ж����ȼ�����
}

/********************************************************************************************************
**������Ϣ ��SysTick_Handler(void)
**�������� ��������жϺ�����Systick���еݼ�
**������� ����
**������� ����
********************************************************************************************************/
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
}

/********************************************************************************************************
**������Ϣ ��TimingDelay_Decrement(void)
**�������� ����1ms���ٶȵݼ�
**������� ��pclk2������ϵͳʱ��Ϊ8MHz������Ӧ����8
**������� ����
********************************************************************************************************/
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    }
}

/********************************************************************************************************
**������Ϣ ��LED_Init(void)
**�������� ��LED��ʼ��
**������� ����
**������� ����
********************************************************************************************************/
void LED_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);   //����GPIOA,GPIOBʱ��

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

/********************************************************************************************************
**������Ϣ ��uart_initwBaudRate(u32 bound)
**�������� ��UART��ʼ��
**������� ��bound
**������� ����
********************************************************************************************************/
void uart_initwBaudRate(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);                       //ʹ��UART1ʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);                         //����GPIOAʱ��
    //UART ��ʼ������
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    UART_InitStructure.UART_BaudRate = bound;                                   //���ڲ�����
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
    UART_InitStructure.UART_StopBits = UART_StopBits_1;                         //һ��ֹͣλ
    UART_InitStructure.UART_Parity = UART_Parity_No;                            //����żУ��λ
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//��Ӳ������������
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;	                //�շ�ģʽ

    UART_Init(UART1, &UART_InitStructure);                                      //��ʼ������1
    UART_Cmd(UART1, ENABLE);                                                    //ʹ�ܴ���1

    //UART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                                   //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                             //�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                      //��ʼ��GPIOA.9

    //UART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                                  //PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                       //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                      //��ʼ��GPIOA.10

}

/********************************************************************************************************
**������Ϣ ��u8 FLASH_Program(void)
**�������� ��Program FLASH Bank1
**������� ��
**������� ��
**    ��ע ��
********************************************************************************************************/
u8 FLASH_Program(void)
{

    /* Porgram FLASH Bank1 ********************************************************/
    /* Unlock the Flash Bank1 Program Erase controller */
    FLASH_Unlock();

    /* Clear All pending flags */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    FLASH_ErasePage(BANK1_WRITE_START_ADDR);

    FLASH_ClearFlag(FLASH_FLAG_EOP );

    FLASHStatus = FLASH_ProgramWord(BANK1_WRITE_START_ADDR, Data);

    FLASH_ClearFlag(FLASH_FLAG_EOP );

    FLASH_Lock();

    UartSendGroup((u8*)printBuf, sprintf(printBuf, "%x\r\n", (*(__IO uint32_t*) BANK1_WRITE_START_ADDR)));

    if((*(__IO uint32_t*) BANK1_WRITE_START_ADDR) != Data) {
        return 1;
    }
    return 0;
}

/********************************************************************************************************
**������Ϣ ��delay_ms(__IO uint32_t nTime)
**�������� ������Ӧ�õ�����ʱ��ʹ��systick
**������� ��nTime����ʱ
**������� ����
********************************************************************************************************/
void delay_ms(__IO uint32_t nTime)
{
    TimingDelay = nTime;

    while(TimingDelay != 0);
}

/********************************************************************************************************
**������Ϣ ��void UartSendByte(u8 dat)
**�������� ��UART��������
**������� ��dat
**������� ��
**    ��ע ��
********************************************************************************************************/
void UartSendByte(u8 dat)
{
    UART_SendData(UART1, dat);
    while(!UART_GetFlagStatus(UART1, UART_FLAG_TXEPT));
}

/********************************************************************************************************
**������Ϣ ��void UartSendGroup(u8* buf,u16 len)
**�������� ��UART��������
**������� ��buf,len
**������� ��
**    ��ע ��
********************************************************************************************************/
void UartSendGroup(u8* buf, u16 len)
{
    while(len--)
        UartSendByte(*buf++);
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