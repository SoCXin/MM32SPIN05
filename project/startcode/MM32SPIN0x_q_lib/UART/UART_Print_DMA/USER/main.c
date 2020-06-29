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

void uart_initwBaudRate(u32 bound);
void UartSendGroup(u8* buf, u16 len);
void MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx, u32 cpar, u32 cmar, u16 cndtr);
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx);

u32 TEXT_TO_SEND[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
#define TEXT_LENTH 9
u8 SendBuff[(TEXT_LENTH + 2) * 100];
char printBuf[100];
u16 DMA1_MEM_LEN;

/********************************************************************************************************
**函数信息 ：main(void)
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
int main(void)
{
    u16 i;
    u8 t = 0;

    uart_initwBaudRate(115200);
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nDMA SEND DATA:\r\n"));
    MYDMA_Config(DMA1_Channel2, (u32)&UART1->TDR, (u32)SendBuff, (TEXT_LENTH + 2) * 10); //DMA1通道2,外设为串口1,存储器为SendBuff,长(TEXT_LENTH+2)*10.
    for(i = 0; i < (TEXT_LENTH + 2) * 100; i++) {                               //填充ASCII字符集数据
        if(t >= TEXT_LENTH) {                                                   //加入换行符
            SendBuff[i++] = 0x0d;
            SendBuff[i] = 0x0a;
            t = 0;
        } else SendBuff[i] = TEXT_TO_SEND[t++];                                  //复制TEXT_TO_SEND语句
    }

    UART_DMACmd(UART1, UART_DMAReq_EN, ENABLE);                                 //使能uart1 DMA
    MYDMA_Enable(DMA1_Channel2);                                                //开始一次DMA传输！
    //等待DMA传输完成，此时我们来做另外一些事，点灯
    //实际应用中，传输数据期间，可以执行另外的任务
    while(1) {
        if(DMA_GetFlagStatus(DMA1_FLAG_TC2) != RESET) {                         //等待通道2传输完成
            DMA_ClearFlag(DMA1_FLAG_TC2);                                       //清除通道2传输完成标志
            break;
        }
    }

    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nUART DMA TEST OK!\r\n"));

    while(1);

}

/********************************************************************************************************
**函数信息 ：uart_initwBaudRate(u32 bound)
**功能描述 ：UART初始化
**输入参数 ：bound
**输出参数 ：无
********************************************************************************************************/
void uart_initwBaudRate(u32 bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);                       //使能UART1时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);                         //开启GPIOA时钟
    //UART 初始化设置
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    UART_InitStructure.UART_BaudRate = bound;                                   //串口波特率
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;                    //字长为8位数据格式
    UART_InitStructure.UART_StopBits = UART_StopBits_1;                         //一个停止位
    UART_InitStructure.UART_Parity = UART_Parity_No;                            //无奇偶校验位
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//无硬件数据流控制
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;	                //收发模式

    UART_Init(UART1, &UART_InitStructure);                                      //初始化串口1
    UART_Cmd(UART1, ENABLE);                                                    //使能串口1

    //UART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                                   //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                             //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                      //初始化GPIOA.9

    //UART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                                  //PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                       //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                      //初始化GPIOA.10

}

/********************************************************************************************************
**函数信息 ：MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
**功能描述 ：可修改根据输入参数修改配置的DMA配置
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx, u32 cpar, u32 cmar, u16 cndtr)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	                        //使能DMA传输

    DMA_DeInit(DMA_CHx);                                                        //将DMA的通道1寄存器重设为缺省值
    DMA1_MEM_LEN = cndtr;
    DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;                            //DMA外设ADC基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = cmar;                                //DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          //数据传输方向，从内存读取发送到外设
    DMA_InitStructure.DMA_BufferSize = cndtr;                                   //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               //工作在正常缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       //DMA通道 x拥有中优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA_CHx, &DMA_InitStructure);                                      //根据DMA_InitStruct中指定的参数初始化DMA的通道UART1_Tx_DMA_Channel所标识的寄存器

}
/********************************************************************************************************
**函数信息 ：MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx)
**功能描述 ：开始一次DMA传输
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx)
{
    //	DMA_Cmd(DMA_CHx, DISABLE );  //关闭UART1 TX DMA1 所指示的通道
    DMA_Cmd(DMA_CHx, ENABLE);  //使能UART1 TX DMA1 所指示的通道
}


/********************************************************************************************************
**函数信息 ：void UartSendByte(u8 dat)
**功能描述 ：UART发送数据
**输入参数 ：dat
**输出参数 ：
**    备注 ：
********************************************************************************************************/
void UartSendByte(u8 dat)
{
    UART_SendData(UART1, dat);
    while(!UART_GetFlagStatus(UART1, UART_FLAG_TXEPT));
}

/********************************************************************************************************
**函数信息 ：void UartSendGroup(u8* buf,u16 len)
**功能描述 ：UART发送数据
**输入参数 ：buf,len
**输出参数 ：
**    备注 ：
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

