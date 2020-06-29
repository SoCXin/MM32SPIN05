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
void SPIM_Test(void);
void UartSendGroup(u8* buf, u16 len);
void SPIM_SectorErase(unsigned long address);
void SPIM_Init(unsigned short spi_baud_div);
void SPIM_ReadID(void);
void SPIM_PageRead(unsigned long address, unsigned char *p, unsigned int number);
void SPIM_PageProgram(unsigned long address, unsigned char *p, unsigned int number);

char printBuf[100];
unsigned char tmpdata[256];
unsigned char rxtmpdata[256];

#define READ        	0x03
#define FAST_READ   	0x0B
#define RDID        	0x9F
#define WREN            0x06
#define WRDI            0x04
#define SE              0xD8
#define BE              0xC7
#define PP              0x02
#define RDSR            0x05
#define WRSR            0x01
#define DP              0xB9
#define RES             0xAB

/********************************************************************************************************
**函数信息 ：int main (void)
**功能描述 ：开机后，串口助手打印256页数据
**输入参数 ：
**输出参数 ：
**    备注 ：注意：改变SPI1或SP2，板子上对应短路帽也应改变
********************************************************************************************************/
int main(void)
{
    uart_initwBaudRate(115200);
    SPIM_Test();
    while(1) {

    }
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
**函数信息 ：SPIM_Test(void)
**功能描述 :测试程序，使用串口打印256页数据
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPIM_Test(void)
{
    unsigned int i;
    for(i = 0; i < 256; i++) {
        tmpdata[i] = i;
    }

    UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 test\r\n"));
    SPIM_Init(0x8);

    SPIM_ReadID();

    SPIM_SectorErase(0);

    SPIM_PageProgram(0, tmpdata, 256);

    for(i = 0; i < 50000; i++);                                                 //延时不能去掉，SPI FLASH可能来不及响应

    for(i = 0; i < 256; i++) {
        rxtmpdata[i] = 0x0;
    }
    SPIM_PageRead(0, rxtmpdata, 256);                                           //把写进去的一页256字节读出来

    for(i = 0; i < 20; i++) {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "rx[%d]=0x%x\r\n", i, rxtmpdata[i]));
    }

    UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 test over\r\n"));

}

/********************************************************************************************************
**函数信息 ：SPIM_CSLow(void)
**功能描述 :为选定的SPI 软件重置内部NSS 管脚
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPIM_CSLow(void)
{
    //Spi cs assign to this pin,select
    SPI_CSInternalSelected(SPI2, SPI_CS_BIT0, ENABLE);
}

/********************************************************************************************************
**函数信息 ：SPIM_CSHigh(void)
**功能描述 :为选定的SPI 软件配置内部NSS 管脚
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPIM_CSHigh(void)
{
    //Spi cs release
    SPI_CSInternalSelected(SPI2, SPI_CS_BIT0, DISABLE);
}

/********************************************************************************************************
**函数信息 ：SPIM_TXEn(void)
**功能描述 :关闭 SPI 在双向模式下的数据传输方向
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPIM_TXEn(void)
{
    //Transmit Enable bit TXEN
    SPI_BiDirectionalLineConfig(SPI2, SPI_Direction_Tx);
}

/********************************************************************************************************
**函数信息 ：SPIM_RXEn(void)
**功能描述 :关闭 SPI 在双向模式下的数据传输方向
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPIM_RXEn(void)
{
    //enable RXEN
    SPI_BiDirectionalLineConfig(SPI2, SPI_Direction_Rx);
}

/********************************************************************************************************
**函数信息 ：SPIMReadWriteByte(unsigned char tx_data)
**功能描述 : 通过外设 SPIx 收发数据 ,用于全双工模式(同时收发)
**输入参数 ：tx_data
**输出参数 ：无
********************************************************************************************************/
unsigned int SPIMReadWriteByte(unsigned char tx_data)
{
    SPI_SendData(SPI2, tx_data);
    while (1) {
        if(SPI_GetFlagStatus(SPI2, SPI_FLAG_RXAVL)) {
            return SPI_ReceiveData(SPI2);
        }
    }
}

/********************************************************************************************************
**函数信息 ：SPIM_Init( unsigned short spi_baud_div)
**功能描述 :可修改参数初始化SPI
**输入参数 ：spi_baud_div
**输出参数 ：无
********************************************************************************************************/
void SPIM_Init(unsigned short spi_baud_div)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);                        //SPI2 clk enable
    SPIM_CSHigh();

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);                         //开启GPIOB时钟

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;                                 //spi2_cs  pb12
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;                                 //spi2_sck  pb13
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                             //复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;                                 //spi2_mosi  pb15
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                             //复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14;                                 //spi2_miso  pb14
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);

    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_DataWidth = SPI_DataWidth_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = spi_baud_div;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI2, &SPI_InitStructure);

    SPIM_TXEn();
    SPIM_RXEn();
    SPI_Cmd(SPI2, ENABLE);                                                      //Enables the specified SPI peripheral SPI使能、主机模式 8位数据模式 SPI 的波特率

}

/********************************************************************************************************
**函数信息 ：SPIM_ReadID(void)
**功能描述 :读取ID
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPIM_ReadID(void)
{
    unsigned char temp;
    unsigned int i;

    SPIM_CSLow();                                                               //Spi cs assign to this pin,select
    SPIMReadWriteByte(RDID);

    for(i = 0; i < 3; i++) {
        temp = SPIMReadWriteByte(0x01);
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "temp=0x%x\r\n", temp));
    }
    SPIM_CSHigh();                                                              //Spi cs release
}

/********************************************************************************************************
**函数信息 ：SPIM_WriteEnable(void)
**功能描述 :写数据使能
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPIM_WriteEnable(void)
{
    SPIM_CSLow();                                                               //Spi cs assign to this pin,select
    SPIMReadWriteByte(WREN);
    SPIM_CSHigh();                                                              //Spi cs release
}

/********************************************************************************************************
**函数信息 ：SSPIM_checkStatus(void)
**功能描述 :检查数据发送是否正确
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPIM_checkStatus(void)
{
    unsigned char temp;
    SPIM_CSLow();                                                               //Spi cs assign to this pin,select
    SPIMReadWriteByte(RDSR);
    while(1) {
        temp = SPIMReadWriteByte(0x00);
        if((temp & 0x01) == 0x0)
            break;
    }
    SPIM_CSHigh();                                                              //Spi cs release
}


/********************************************************************************************************
**函数信息 ：SPIM_PageRead(unsigned long address,unsigned char *p,unsigned int number)
**功能描述 :读取256页数据
**输入参数 ：address  ;*p ;number
**输出参数 ：无
********************************************************************************************************/
void SPIM_PageRead(unsigned long address, unsigned char *p, unsigned int number) //page = 256 bytes
{
    unsigned char addr0, addr1, addr2;
    unsigned int i;
    address = address & 0xffffff00;                                             //page address
    addr0 = (unsigned char)(address >> 16);
    addr1 = (unsigned char)(address >> 8);
    addr2 = (unsigned char)address;

    SPIM_CSLow();                                                               //Spi cs assign to this pin,select

    SPIMReadWriteByte(READ);
    SPIMReadWriteByte(addr0);
    SPIMReadWriteByte(addr1);
    SPIMReadWriteByte(addr2);

    for(i = 0; i < 7; i++) { //过滤掉前面7个错误数据
        SPIMReadWriteByte(0);
    }
    for(i = 0; i < 256; i++) {
        rxtmpdata[i] = SPIMReadWriteByte(0x00);
    }


    SPIM_CSHigh();                                                              //Spi cs release
}

/********************************************************************************************************
**函数信息 ：SPIM_PageProgram(unsigned long address,unsigned char *p,unsigned int number)
**功能描述 :发送256页数据
**输入参数 : address;*p;number
**输出参数 ：无
********************************************************************************************************/
void SPIM_PageProgram(unsigned long address, unsigned char *p, unsigned int number)
{
    u32 i;
    DMA_InitTypeDef  DMA_InitStructure;

    unsigned char addr0, addr1, addr2;
    address = address & 0xffffff00;                                             //page address
    addr0 = (unsigned char)(address >> 16);
    addr1 = (unsigned char)(address >> 8);
    addr2 = (unsigned char)address;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);                          //使能DMA传输
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI2->TXREG);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(p);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = number;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);                                //SPI TX为DMA通道15

    SPIM_WriteEnable();
    SPIM_CSLow();                                                               //Spi cs assign to this pin,select
    SPIMReadWriteByte(PP);

    SPIMReadWriteByte(addr0);

    SPIMReadWriteByte(addr1);

    SPIMReadWriteByte(addr2);

    DMA_Cmd(DMA1_Channel5, ENABLE);                                             //使能UART1 TX DMA1 所指示的通道

    SPI_DMACmd(SPI2, SPI_DMAReq_EN, ENABLE);

    while(!DMA_GetFlagStatus(DMA1_FLAG_TC5));

    DMA_ClearFlag(DMA1_FLAG_TC5);

    SPI_DMACmd(SPI2, SPI_DMAReq_EN, DISABLE);
    for(i = 0; i < 10000; i++);
    DMA_Cmd(DMA1_Channel5, DISABLE);                                            //使能UART1 TX DMA1 所指示的通道
    for(i = 0; i < 10000; i++);

    SPIM_CSHigh();                                                              //Spi cs release

    SPIM_checkStatus();
}

/********************************************************************************************************
**函数信息 ：SPIM_SectorErase(unsigned long address)
**功能描述 :擦除数据
**输入参数 ：unsigned long address,删除指定sector的地址 each sector = 64Kbytes
**输出参数 ：无
********************************************************************************************************/
void SPIM_SectorErase(unsigned long address)
{
    unsigned char addr0, addr1, addr2;
    address = address & 0xffff0000;
    addr0 = ((unsigned char)(address >> 16)) & 0xff;
    addr1 = ((unsigned char)(address >> 8)) & 0xff;
    addr2 = ((unsigned char)address) & 0xff;

    SPIM_WriteEnable();

    SPIM_CSLow();                                                               //Spi cs assign to this pin,select

    SPIMReadWriteByte(SE);
    SPIMReadWriteByte(addr0);
    SPIMReadWriteByte(addr1);
    SPIMReadWriteByte(addr2);
    SPIM_CSHigh();                                                              //Spi cs release

    SPIM_checkStatus();
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

