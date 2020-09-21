//******************************************************************************
// GPIO.c
//******************************************************************************

#define GPIO_c

#include <stdint.h>
#include "GPIO.h"

//------------------------------------------------------------------------------
#define F031x8_L073_W073  1 //for MM32F031x8, MM32L073 and MM32W073
#define F031x6_F103_L373  0 //for MM32F031x6, MM32F103 and MM32L373

#if F031x8_L073_W073
//--------------------------------------
#define   __I    volatile const
#define   __O    volatile
#define   __IO   volatile
	
typedef struct
{
  __IO uint32_t  CR;
  __IO uint32_t  CFGR;
  __IO uint32_t  CIR;
  __IO uint32_t  APB2RSTR;
  __IO uint32_t  APB1RSTR;
  __IO uint32_t  AHBENR;
  __IO uint32_t  APB2ENR;
  __IO uint32_t  APB1ENR;
  __IO uint32_t  BDCR;
  __IO uint32_t  CSR;
} RCC_TypeDef;

typedef struct
{                                  //offset:
  __IO uint32_t  CRL;              //0x00
  __IO uint32_t  CRH;              //0x04
  __IO uint32_t  IDR;              //0x08
  __IO uint32_t  ODR;              //0x0C
  __IO uint32_t  BSRR;             //0x10
  __IO uint32_t  BRR;              //0x14
  __IO uint32_t  LCKR;             //0x18
  __IO uint32_t  RESERVED0;        //0x1C
  __IO uint32_t  AFRL;             //0x20
  __IO uint32_t  AFRH;             //0x24
} GPIO_TypeDef;

#define PERIPH_BASE        ((uint32_t)0x40000000)
#define APB1PERIPH_BASE    (PERIPH_BASE + 0)
#define APB2PERIPH_BASE    (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE     (PERIPH_BASE + 0x20000)
#define RCC_BASE           (AHBPERIPH_BASE + 0x1000)
#define GPIOA_BASE         ((uint32_t)0x48000000)
#define GPIOB_BASE         ((uint32_t)0x48000400)
#define GPIOC_BASE         ((uint32_t)0x48000800)
#define GPIOD_BASE         ((uint32_t)0x48000C00)

#define RCC                ((RCC_TypeDef *) RCC_BASE)
#define GPIOA              ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB              ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC              ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD              ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOF              ((GPIO_TypeDef *) GPIOF_BASE)

#else //F031x6_F103_L373
//--------------------------------------
#define   __I    volatile const
#define   __O    volatile
#define   __IO   volatile

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;

typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

#define PERIPH_BASE        ((uint32_t)0x40000000)
#define APB1PERIPH_BASE    (PERIPH_BASE + 0)
#define APB2PERIPH_BASE    (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE     (PERIPH_BASE + 0x20000)
#define RCC_BASE           (AHBPERIPH_BASE + 0x1000)
#define AFIO_BASE          (APB2PERIPH_BASE + 0x0000)
#define GPIOA_BASE         (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE         (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE         (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE         (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE         (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE         (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE         (APB2PERIPH_BASE + 0x2000) 

#define RCC                ((RCC_TypeDef *) RCC_BASE)
#define AFIO               ((AFIO_TypeDef *) AFIO_BASE)
#define GPIOA              ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB              ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC              ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD              ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE              ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF              ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG              ((GPIO_TypeDef *) GPIOG_BASE)
//------------------------------------------------------------------------------
#endif

//------------------------------------------------------------------------------
#define Bit(n) ((uint32_t)1<<(n))

#define Bit0   Bit(0)
#define Bit1   Bit(1)
#define Bit2   Bit(2)
#define Bit3   Bit(3)
#define Bit4   Bit(4)
#define Bit5   Bit(5)
#define Bit6   Bit(6)
#define Bit7   Bit(7)
#define Bit8   Bit(8)
#define Bit9   Bit(9)
#define Bit10  Bit(10)
#define Bit11  Bit(11)
#define Bit12  Bit(12)
#define Bit13  Bit(13)
#define Bit14  Bit(14)
#define Bit15  Bit(15)
#define Bit16  Bit(16)
#define Bit17  Bit(17)
#define Bit18  Bit(18)
#define Bit19  Bit(19)
#define Bit20  Bit(20)
#define Bit21  Bit(21)
#define Bit22  Bit(22)
#define Bit23  Bit(23)
#define Bit24  Bit(24)
#define Bit25  Bit(25)
#define Bit26  Bit(26)
#define Bit27  Bit(27)
#define Bit28  Bit(28)
#define Bit29  Bit(29)
#define Bit30  Bit(30)
#define Bit31  Bit(31)

#define TRUE   1
#define FALSE  0

//bit width for macro 'setv()'
#define BW1    1
#define BW2    2
#define BW3    3
#define BW4    4
#define BW5    5
#define BW6    6
#define BW7    7
#define BW8    8
#define BW9    9
#define BW10   10
#define BW11   11
#define BW12   12
#define BW13   13
#define BW14   14
#define BW15   15
#define BW16   16
#define BW17   17
#define BW18   18
#define BW19   19
#define BW20   20
#define BW21   21
#define BW22   22
#define BW23   23
#define BW24   24
#define BW25   25
#define BW26   26
#define BW27   27
#define BW28   28
#define BW29   29
#define BW30   30
#define BW31   31
#define BW32   32

//bit shift for macro 'setv()'
#define BS0    0
#define BS1    1
#define BS2    2
#define BS3    3
#define BS4    4
#define BS5    5
#define BS6    6
#define BS7    7
#define BS8    8
#define BS9    9
#define BS10   10
#define BS11   11
#define BS12   12
#define BS13   13
#define BS14   14
#define BS15   15
#define BS16   16
#define BS17   17
#define BS18   18
#define BS19   19
#define BS20   20
#define BS21   21
#define BS22   22
#define BS23   23
#define BS24   24
#define BS25   25
#define BS26   26
#define BS27   27
#define BS28   28
#define BS29   29
#define BS30   30
#define BS31   31

//set bits and clear bits in a register
#define setb(reg,bit)  reg |= (bit)
#define clrb(reg,bit)  reg &= ~(bit)

//fill bits value in a register (BW: bit width; BS: bit shift)
//(Note: there needs "{}" for the macro "setv()", since it contains more than one statements)
#define setv(reg,value,BW,BS)  {reg &= ~((((uint32_t)1<<(BW))-1)<<(BS)); reg |= ((uint32_t)(value)<<(BS));}

//write '1' to clear bit in a register
#define write_1_clrb(reg,bit)  reg |= (bit)

//check bit value in a register
//(Note: there needs "()" since it may be prefixed with "!")
#define chkb(reg,bit)  (reg & (bit))

//------------------------------------------------------------------------------


//==============================================================================
void GPIO_config(unsigned char pin, unsigned char io, unsigned char type, unsigned char af)
{
	GPIO_TypeDef *GPIOx;
	__IO uint32_t *reg;
	#if F031x8_L073_W073
	__IO uint32_t *regAF;
	#endif
	unsigned char offset;
	
	//assign 'GPIOx' to the corresponding gpio, and enable its clock
	if(pin<PB0){
		GPIOx=GPIOA;
		#if F031x8_L073_W073
		setb(RCC->AHBENR,Bit17); //GPIOAEN=1: enable GPIOA clock
		#else
		setb(RCC->APB2ENR,Bit2); //GPIOAEN=1: enable GPIOA clock
		#endif
	}
	else if(pin<PC0){
		GPIOx=GPIOB;
		#if F031x8_L073_W073
		setb(RCC->AHBENR,Bit18); //GPIOBEN=1: enable GPIOB clock
		#else
		setb(RCC->APB2ENR,Bit3); //GPIOBEN=1: enable GPIOB clock
		#endif
	}
	else if(pin<PD0){
		GPIOx=GPIOC;
		#if F031x8_L073_W073
		setb(RCC->AHBENR,Bit19); //GPIOCEN=1: enable GPIOC clock
		#else
		setb(RCC->APB2ENR,Bit4); //GPIOCEN=1: enable GPIOC clock
		#endif
	}
	else{
		GPIOx=GPIOD;
		#if F031x8_L073_W073		
		setb(RCC->AHBENR,Bit20); //GPIODEN=1: enable GPIOD clock
		#else
		setb(RCC->APB2ENR,Bit5); //GPIODEN=1: enable GPIOD clock
		#endif
	}

	//get real pin number
	pin &= 0x0F; 

	#if F031x8_L073_W073
	//assign 'reg' to the corresponding address of 'CRL' and 'CRH', and 'regAF' to the corresponding address of 'AFRL' and 'AFRH'
	if(pin<8){ reg= &(GPIOx->CRL); regAF= &(GPIOx->AFRL); offset=0;}
	else{ reg= &(GPIOx->CRH); regAF= &(GPIOx->AFRH); pin-=8; offset=8;}	
	#else
	//assign 'reg' to the corresponding address of 'CRL' and 'CRH'
	if(pin<8){ reg= &(GPIOx->CRL); offset=0;}
	else{ reg= &(GPIOx->CRH); pin-=8; offset=8;}	
	#endif
	
	//configure i/o and type
	if(io==INPUT){
		setv(*reg,0,BW2,pin*4); //MODE=[0,0]: as input mode
		if(type==ANALOG) setv(*reg,0,BW2,pin*4+2); //CNF=[0,0]: as analog input
		if(type==FLOATING) setv(*reg,1,BW2,pin*4+2); //CNF=[0,1]: as floating input
		if(type==PULL_HIGH){ setv(*reg,2,BW2,pin*4+2); setb(GPIOx->ODR,Bit(pin+offset));} //CNF=[1,0]: as pull-high input	
		if(type==PULL_LOW){ setv(*reg,2,BW2,pin*4+2); clrb(GPIOx->ODR,Bit(pin+offset));} //CNF=[1,0]: as pull-low input
	}
	if(io==OUTPUT){
		setv(*reg,3,BW2,pin*4); //MODE=[1,1]: as output mode, speed=10MHz (! use this to prevent from overshoot)
		//setv(*reg,1,BW2,pin*4); //MODE=[0,1]: as output mode, speed=50MHz
		//setv(*reg,2,BW2,pin*4); //MODE=[1,0]: as output mode, speed=20MHz		
		if(type==PUSH_PULL) setv(*reg,0,BW2,(pin*4+2)); //CNF=[0,0]: as push-pull output			
		if(type==OPEN_DRAIN) setv(*reg,1,BW2,(pin*4+2)); //CNF=[0,1]: as open-drain output
		if(type==AF_PUSH_PULL) setv(*reg,2,BW2,(pin*4+2)); //CNF=[1,0]: as alternate function push-pull output
		if(type==AF_OPEN_DRAIN) setv(*reg,3,BW2,(pin*4+2)); //CNF=[1,1]: as alternate function open-drain output			
	}

	#if F031x8_L073_W073	
	//configure af (alternate function)
	setv(*regAF,af,BW4,pin*4);
	#endif
}

//==============================================================================
