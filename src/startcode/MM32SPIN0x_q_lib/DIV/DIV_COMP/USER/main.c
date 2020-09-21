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

#include "stdio.h"
#include "HAL_device.h"
#include "HAL_conf.h"

#define GET_SoftDivider(DVD, DVS) ((DVD) / (DVS))

char printBuf[100];
void Uart_ConfigInit(u32 bound);
void UartSendGroup(u8* buf, u16 len);
void UartSendAscii(char *str);
void RCC_ConfigInit(void);
void GPIO_ConfigInit(void);

u32 dataBuf[32] = {0x12345678, 0x9876554, 0x9822346, 0x98734662,
                   0x000001, 0xFFFFFFFF, 0x11111111, 0x22222222,
                   0x33333333, 0x78645323, 0x4399039, 0x4367464,
                   0x42345678, 0x9873554, 0x9822346, 0x98734662,
                   0x100001, 0x1FFFFFFF, 0x11511111, 0x22422222,
                   0x53333333, 0x78645323, 0x4399039, 0x4367464,
                   0x140001, 0x11FFFFFF, 0x12111111, 0x29422222,
                   0x53333333, 0x78644323, 0x4369039, 0x4767464,
                  };

u32 dataBuf1[32] = {9, 7, 8, 2,
                    33, 44, 66, 77,
                    11, 22, 1, 4,
                    44, 44, 11, 8,
                    45, 6, 3, 9,
                    2332, 989, 23, 77,
                    79, 345, 12, 43,
                    67, 89, 78, 23,
                   };
u32 result[2];


/*******************************************************************************
* @name   : main
* @brief  : HWDIV
* @param  : None
* @retval : void
*******************************************************************************/
int main (void)
{
    int i, k;
    RCC_ConfigInit();
    GPIO_ConfigInit();
    Uart_ConfigInit(9600);

    UartSendGroup((u8*)printBuf, sprintf(printBuf, "sprintf ok\r\n"));

    while(1) {
        for(i = 0; i < 32; i ++) {
#if 1
#if 0
            SET_HWDivider(dataBuf[i], dataBuf1[i]);
            result[0] = GET_HWDivider;
            result[1] = GET_Divider(dataBuf[i], dataBuf1[i]);
            if(result[1] != result[0]) {
                UartSendGroup((u8*)printBuf, sprintf(printBuf, "ERROR\r\n");
                              while(1);
            }
#else

            GPIOC->ODR ^= 1 << 9;
            SET_HWDivider(dataBuf[i], dataBuf1[0]);
            result[0] = GET_HWDivider;
#if 0
            SET_HWDivider(dataBuf[i], dataBuf1[1]);
            result[0] = GET_HWDivider;
            SET_HWDivider(dataBuf[i], dataBuf1[2]);
            result[0] = GET_HWDivider;
            SET_HWDivider(dataBuf[i], dataBuf1[3]);
            result[0] = GET_HWDivider;
            SET_HWDivider(dataBuf[i], dataBuf1[4]);
            result[0] = GET_HWDivider;
            SET_HWDivider(dataBuf[i], dataBuf1[5]);
            result[0] = GET_HWDivider;
            SET_HWDivider(dataBuf[i], dataBuf1[6]);
            result[0] = GET_HWDivider;
            SET_HWDivider(dataBuf[i], dataBuf1[7]);
            result[0] = GET_HWDivider;

#endif

            GPIOC->ODR ^= 1 << 9;
            k = 100; while(k--);
            GPIOC->ODR ^= 1 << 9;
            result[1] = GET_SoftDivider(dataBuf[i], dataBuf1[0]);
#if 0
            result[1] = GET_Divider(dataBuf[i], dataBuf1[1]);
            result[1] = GET_Divider(dataBuf[i], dataBuf1[2]);
            result[1] = GET_Divider(dataBuf[i], dataBuf1[3]);
            result[1] = GET_Divider(dataBuf[i], dataBuf1[4]);
            result[1] = GET_Divider(dataBuf[i], dataBuf1[5]);
            result[1] = GET_Divider(dataBuf[i], dataBuf1[6]);
            result[1] = GET_Divider(dataBuf[i], dataBuf1[7]);
#endif
            GPIOC->ODR ^= 1 << 9;
#endif
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "%10d , %10d\r\n", result[0], result[1]));

#endif
        }
        while(1);
    }
}

/*******************************************************************************
* @name   : RCC_ConfigInit
* @brief  : RCC config
* @param  : None
* @retval : void
*******************************************************************************/
void RCC_ConfigInit(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_HWDIV, ENABLE);
}

/*******************************************************************************
* @name   : GPIO_ConfigInit
* @brief  : GPIO config
* @param  : None
* @retval : void
*******************************************************************************/
void GPIO_ConfigInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*******************************************************************************
* @name   : Uart_ConfigInit
* @brief  : Uart Config Init
* @param  : u32 bound
* @retval : void
*******************************************************************************/
void Uart_ConfigInit(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    //Serial baud rate
    UART_InitStructure.UART_BaudRate = bound;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    //Transceiver mode
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;

    UART_Init(UART1, &UART_InitStructure);
    UART_Cmd(UART1, ENABLE);

    //UART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //Multiplexing push-pull output
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //UART1_RX	  GPIOA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    //Floating input
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
* @name   : UartSendByte
* @brief  : Uart Send Byte
* @param  : u8 dat
* @retval : void
*******************************************************************************/
void UartSendByte(u8 dat)
{
    UART_SendData(UART1, dat);
    while(!UART_GetFlagStatus(UART1, UART_FLAG_TXEPT));
}

/*******************************************************************************
* @name   : UartSendGroup
* @brief  : Uart Send Group
* @param  : u8* buf,u16 len
* @retval : void
*******************************************************************************/
void UartSendGroup(u8* buf, u16 len)
{
    while(len--)
        UartSendByte(*buf++);
}

/*******************************************************************************
* @name   : UartSendAscii
* @brief  : Uart Send Ascii
* @param  : char *str
* @retval : void
*******************************************************************************/
void UartSendAscii(char *str)
{
    while(*str)
        UartSendByte(*str++);
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

