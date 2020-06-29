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
void I2CMasterTest(I2C_TypeDef *I2Cx);
void I2CInitMasterMode(void);
void I2CMasterWrite(I2C_TypeDef *I2Cx, unsigned char device_id, unsigned short mem_byte_addr, unsigned short tx_count, unsigned char *tx_data );
void I2CMasterRead(I2C_TypeDef *I2Cx, unsigned char device_id, unsigned short mem_byte_addr, unsigned short rx_count, unsigned char *rx_data );
void UartSendGroup(u8* buf, u16 len);
void I2CTXByte(I2C_TypeDef *I2Cx, unsigned short cmd, unsigned char temp);
void I2CTXEmptyCheck(I2C_TypeDef *I2Cx);
unsigned char I2CRXByte(I2C_TypeDef *I2Cx);
void I2CRXFullCheck(I2C_TypeDef *I2Cx);

#define FLASH_DEVICE_ADDR 0xa8
char printBuf[100];
unsigned char tx_buffer0[16] = {0x55, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf};
unsigned char rx_buffer0[16] ;

/********************************************************************************************************
**函数信息 ：main(void)
**功能描述 ：
**输入参数 ：
**输出参数 ：
**    备注 ：
********************************************************************************************************/
int main(void)
{
    uart_initwBaudRate(115200);
    I2CMasterTest(I2C1);                                                        //I2C收发数据
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
**函数信息 ：I2CMasterTest(I2C_TypeDef *I2Cx)
**功能描述 ：I2C接收发送数据测试程序
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2
**输出参数 ：无
********************************************************************************************************/
unsigned char rxBuffer[16] = {0};

void I2CMasterTest(I2C_TypeDef *I2Cx)
{
    unsigned char i;

    I2CInitMasterMode();


    I2CMasterWrite(I2C1, FLASH_DEVICE_ADDR, 16 * 1, 16, tx_buffer0 );

    for(i = 0; i < 16 ; i ++) {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "TX data is: %x \r\n", tx_buffer0[i]));
    }

    I2CMasterRead(I2C1, FLASH_DEVICE_ADDR, 16 * 1, 16, rx_buffer0 );

    for(i = 0; i < 16; i++) {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "RX data%d is  : %x \r\n", i, rx_buffer0[i]));
    }

}

/********************************************************************************************************
**函数信息 ：I2CInitMasterMode(void) //unit is Khz
**功能描述 ：初始化I2C
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2
**输出参数 ：无
********************************************************************************************************/
void I2CInitMasterMode(void)
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);                         //开启GPIOB时钟

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_20MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_Mode = I2C_Mode_MASTER;
    I2C_InitStructure.I2C_OwnAddress = FLASH_DEVICE_ADDR;
    I2C_InitStructure.I2C_Speed = I2C_Speed_STANDARD;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_20MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1);

}

/********************************************************************************************************
**函数信息 ：I2CMasterWrite(I2C_TypeDef *I2Cx,unsigned char device_id, unsigned short mem_byte_addr, unsigned short tx_count, unsigned char *tx_data )
**功能描述 ：I2C发送数据
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2；cmd；temp
**输出参数 ：无
********************************************************************************************************/
void I2CMasterWrite(I2C_TypeDef *I2Cx, unsigned char device_id, unsigned short mem_byte_addr, unsigned short tx_count, unsigned char *tx_data )
{
    unsigned short temp;
    unsigned short i;
    unsigned char *p;
    uint32_t j;
    p = tx_data;

    I2C_Cmd(I2Cx, DISABLE);
    I2C_Send7bitAddress(I2Cx, FLASH_DEVICE_ADDR, I2C_Direction_Transmitter);
    I2C_Cmd(I2Cx, ENABLE);
    temp = ((mem_byte_addr) & 0xff);
    I2CTXByte(I2Cx, CMD_WRITE, temp);                                           //tx memory addr

    for(i = 0; i < tx_count; i++) {
        temp = *p;
        p++;
        if(i == (tx_count - 1))
            I2Cx->IC_DATA_CMD = temp | 0x200;                                   //muaul set stop flag
        else
            I2CTXByte(I2Cx, CMD_WRITE, temp);                                   //tx data
    }

    for(j = 0; j < 0x5000; j++);

}

/********************************************************************************************************
**函数信息 ：I2CMasterRead(I2C_TypeDef *I2Cx,unsigned char device_id, unsigned short mem_byte_addr, unsigned short rx_count, unsigned char *rx_data )
**功能描述 ：I2C接收数据
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2；device_id；mem_byte_addr；rx_count；rx_data
**输出参数 ：无
********************************************************************************************************/
void I2CMasterRead(I2C_TypeDef *I2Cx, unsigned char device_id, unsigned short mem_byte_addr, unsigned short rx_count, unsigned char *rx_data )
{
    unsigned char temp = 0;
    unsigned short i;

    I2C_Cmd(I2Cx, DISABLE);
    I2C_Send7bitAddress(I2Cx, FLASH_DEVICE_ADDR, I2C_Direction_Transmitter);

    I2C_Cmd(I2Cx, ENABLE);

    temp = ((mem_byte_addr) & 0xff);

    I2CTXByte(I2Cx, CMD_WRITE, temp); //tx memory addr


    for(i = 0; i < rx_count; i++) {
        rx_data[i] = I2CRXByte(I2Cx);


    }
}

/********************************************************************************************************
**函数信息 ：I2CTXByte(I2C_TypeDef *I2Cx,unsigned short cmd,unsigned char temp)
**功能描述 ：I2C发送数据
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2；cmd；temp
**输出参数 ：无
********************************************************************************************************/
void I2CTXByte(I2C_TypeDef *I2Cx, unsigned short cmd, unsigned char temp)
{
    I2C_SendData(I2Cx, temp);
    I2CTXEmptyCheck(I2Cx);

}

/********************************************************************************************************
**函数信息 ：I2CTXEmptyCheck(I2C_TypeDef *I2Cx)
**功能描述 ：检查发送中断标志位
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2
**输出参数 ：无
********************************************************************************************************/
void I2CTXEmptyCheck(I2C_TypeDef *I2Cx)
{
    while(1) {
        if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TX_EMPTY)) {
            break;
        }
    }
}

/********************************************************************************************************
**函数信息 ：I2CRXByte(I2C_TypeDef *I2Cx)
**功能描述 ：I2C接收数据
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2
**输出参数 ：temp
********************************************************************************************************/
unsigned char I2CRXByte(I2C_TypeDef *I2Cx)
{
    unsigned short temp;

    I2CRXFullCheck(I2Cx);

    temp = I2C_ReceiveData(I2Cx);
    return (unsigned char)temp;
}

/********************************************************************************************************
**函数信息 ：I2CTXEmptyCheck(I2C_TypeDef *I2Cx)
**功能描述 ：检查接收中断标志位
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2
**输出参数 ：无
********************************************************************************************************/
void I2CRXFullCheck(I2C_TypeDef *I2Cx)
{

    while(1) {
        if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_RX_FULL)) {
            break;
        }
    }
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
    UART_SendData( UART1, dat);
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

