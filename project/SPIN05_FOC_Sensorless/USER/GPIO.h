//******************************************************************************
// GPIO.h
//******************************************************************************

#ifndef GPIO_c
 extern void GPIO_config(unsigned char pin, unsigned char io, unsigned char type, unsigned char af);
#endif

/*
  Description of the parameters ------------------------------------------------

	'pin' can be the following identifiers:
	PA0, PA1, PA2, .., PA15
	PB0, PB1, PB2, .., PB15
	PC0, PC1, PC2, .., PC15
	PD0, PD1, PD2, .., PD15

	'io' can be the following identifiers:
	INPUT, OUTPUT
  
	'type' can be the following identifiers: 
	when io is INPUT: ANALOG, FLOATING, PULL_HIGH, PULL_LOW
	when io is OUTPUT: PUSH_PULL, OPEN_DRAIN, AF_PUSH_PULL, AF_OPEN_DRAIN
	
	'af' can be the following identifiers:
	AF0, AF1, AF2, .., AF9	


  Usage: (use PA8 as an example) -----------------------------------------------

	GPIO_config(PA8,INPUT,ANALOG,NoAF); //configure PA8 as analog input mode
	GPIO_config(PA8,INPUT,FLOATING,NoAF); //configure PA8 as floating input mode
	GPIO_config(PA8,INPUT,PULL_HIGH,NoAF); //configure PA8 as pull-high input mode
	GPIO_config(PA8,INPUT,PULL_LOW,NoAF); //configure PA8 as pull-low input mode

	GPIO_config(PA8,OUTPUT,PUSH_PULL,NoAF); //configure PA8 as push-pull output mode	
	GPIO_config(PA8,OUTPUT,OPEN_DRAIN,NoAF); //configure PA8 as open-drain output mode
	
	GPIO_config(PA8,INPUT,PULL_HIGH,AF2); //configure PA8 as alternate-function AF2 pull-high input mode, i.e. TIM1_CH1 input
	GPIO_config(PA8,OUTPUT,AF_OPEN_DRAIN,AF0); //configure PA8 as alternate-function AF0 open-drain output mode, i.e. MCO
	GPIO_config(PA8,OUTPUT,AF_PUSH_PULL,AF2); //configure PA8 as alternate-function AF2 push-pull output mode, i.e. TIM1_CH1 output

  Example Code: ----------------------------------------------------------------

	void test_GPIO(void)
	{
		unsigned char io_state;
		
		init_GPIO();

		PA15o0; //PA15 output 0, let LED6 turned on
		PB3o1;  //PB3 output 1, let LED5 turned off
		PB4o0;  //PB4 output 0, let LED4 turned on
		PB5o1;  //PB5 output 1, let LED3 turned off
	
		io_state=PB2i; //read PB2 input
	
		while(1){
			if(PB2ic) PA8ot; //check PB2 input; if high, then toggle PA8 output
		}
	}

	//
	void init_GPIO(void)
	{
		GPIO_config(PA15,OUTPUT,OPEN_DRAIN,NoAF); //PA15 (MiniBoard's LED6), configured as open-drain output
		GPIO_config(PB3,OUTPUT,OPEN_DRAIN,NoAF);  //PB3 (MiniBoard's LED5), configured as open-drain output
		GPIO_config(PB4,OUTPUT,OPEN_DRAIN,NoAF);  //PB4 (MiniBoard's LED4), configured as open-drain output	
		GPIO_config(PB5,OUTPUT,OPEN_DRAIN,NoAF);  //PB5 (MiniBoard's LED3, configured as open-drain output

		GPIO_config(PA8,OUTPUT,PUSH_PULL,NoAF);   //PA8 (MiniBoard's TP4), configured as push-pull output
		GPIO_config(PB2,INPUT,PULL_HIGH,NoAF);    //PB2 (MiniBoard's TP5), configured as pull-high intput	
	}
	
*/

//------------------------------------------------------------------------------
// Macro Definition for GPIO
//------------------------------------------------------------------------------

#define PA0   0xA0
#define PA1   0xA1
#define PA2   0xA2
#define PA3   0xA3
#define PA4   0xA4
#define PA5   0xA5
#define PA6   0xA6
#define PA7   0xA7
#define PA8   0xA8
#define PA9   0xA9
#define PA10  0xAA
#define PA11  0xAB
#define PA12  0xAC
#define PA13  0xAD
#define PA14  0xAE
#define PA15  0xAF

#define PB0   0xB0
#define PB1   0xB1
#define PB2   0xB2
#define PB3   0xB3
#define PB4   0xB4
#define PB5   0xB5
#define PB6   0xB6
#define PB7   0xB7
#define PB8   0xB8
#define PB9   0xB9
#define PB10  0xBA
#define PB11  0xBB
#define PB12  0xBC
#define PB13  0xBD
#define PB14  0xBE
#define PB15  0xBF

#define PC0   0xC0
#define PC1   0xC1
#define PC2   0xC2
#define PC3   0xC3
#define PC4   0xC4
#define PC5   0xC5
#define PC6   0xC6
#define PC7   0xC7
#define PC8   0xC8
#define PC9   0xC9
#define PC10  0xCA
#define PC11  0xCB
#define PC12  0xCC
#define PC13  0xCD
#define PC14  0xCE
#define PC15  0xCF

#define PD0   0xD0
#define PD1   0xD1
#define PD2   0xD2
#define PD3   0xD3
#define PD4   0xD4
#define PD5   0xD5
#define PD6   0xD6
#define PD7   0xD7
#define PD8   0xD8
#define PD9   0xD9
#define PD10  0xDA
#define PD11  0xDB
#define PD12  0xDC
#define PD13  0xDD
#define PD14  0xDE
#define PD15  0xDF

#define INPUT          0x0A
#define ANALOG         0
#define FLOATING       1
#define PULL_HIGH      2
#define PULL_LOW       3

#define OUTPUT         0x0B
#define PUSH_PULL      0
#define OPEN_DRAIN     1
#define AF_PUSH_PULL   2
#define AF_OPEN_DRAIN  3

#define AF0            0
#define AF1            1
#define AF2            2
#define AF3            3
#define AF4            4
#define AF5            5
#define AF6            6
#define AF7            7
#define AF8            8
#define AF9            9
#define NoAF           15

//--------------------------------------------------------------------
// GPIO input, pin=0~15
//--------------------------------------------------------------------

#define Bit(n) ((uint32_t)1<<(n))

//(Note: there needs "()" since it may be prefixed with "!")
#define PAi(pin)  ((GPIOA->IDR & Bit(pin))>>pin)
#define PBi(pin)  ((GPIOB->IDR & Bit(pin))>>pin)
#define PCi(pin)  ((GPIOC->IDR & Bit(pin))>>pin)
#define PDi(pin)  ((GPIOD->IDR & Bit(pin))>>pin)

//Individual pin definition for input
#define PA0i    PAi(0)
#define PA1i    PAi(1)
#define PA2i    PAi(2)
#define PA3i    PAi(3)
#define PA4i    PAi(4)
#define PA5i    PAi(5)
#define PA6i    PAi(6)
#define PA7i    PAi(7)
#define PA8i    PAi(8)
#define PA9i    PAi(9)
#define PA10i   PAi(10)
#define PA11i   PAi(11)
#define PA12i   PAi(12)
#define PA13i   PAi(13)
#define PA14i   PAi(14)
#define PA15i   PAi(15)
//------
#define PB0i    PBi(0)
#define PB1i    PBi(1)
#define PB2i    PBi(2)
#define PB3i    PBi(3)
#define PB4i    PBi(4)
#define PB5i    PBi(5)
#define PB6i    PBi(6)
#define PB7i    PBi(7)
#define PB8i    PBi(8)
#define PB9i    PBi(9)
#define PB10i   PBi(10)
#define PB11i   PBi(11)
#define PB12i   PBi(12)
#define PB13i   PBi(13)
#define PB14i   PBi(14)
#define PB15i   PBi(15)
//------
#define PC0i    PCi(0)
#define PC1i    PCi(1)
#define PC2i    PCi(2)
#define PC3i    PCi(3)
#define PC4i    PCi(4)
#define PC5i    PCi(5)
#define PC6i    PCi(6)
#define PC7i    PCi(7)
#define PC8i    PCi(8)
#define PC9i    PCi(9)
#define PC10i   PCi(10)
#define PC11i   PCi(11)
#define PC12i   PCi(12)
#define PC13i   PCi(13)
#define PC14i   PCi(14)
#define PC15i   PCi(15)
//------
#define PD0i    PDi(0)
#define PD1i    PDi(1)
#define PD2i    PDi(2)
#define PD3i    PDi(3)
#define PD4i    PDi(4)
#define PD5i    PDi(5)
#define PD6i    PDi(6)
#define PD7i    PDi(7)
#define PD8i    PDi(8)
#define PD9i    PDi(9)
#define PD10i   PDi(10)
#define PD11i   PDi(11)
#define PD12i   PDi(12)
#define PD13i   PDi(13)
#define PD14i   PDi(14)
#define PD15i   PDi(15)
//------

//--------------------------------------------------------------------
// GPIO input check, pin=0~15
//--------------------------------------------------------------------

#define PAic(pin)  (GPIOA->IDR & Bit(pin))
#define PBic(pin)  (GPIOB->IDR & Bit(pin))
#define PCic(pin)  (GPIOC->IDR & Bit(pin))
#define PDic(pin)  (GPIOD->IDR & Bit(pin))

//Individual pin definition for input
#define PA0ic   PAic(0)
#define PA1ic   PAic(1)
#define PA2ic   PAic(2)
#define PA3ic   PAic(3)
#define PA4ic   PAic(4)
#define PA5ic   PAic(5)
#define PA6ic   PAic(6)
#define PA7ic   PAic(7)
#define PA8ic   PAic(8)
#define PA9ic   PAic(9)
#define PA10ic  PAic(10)
#define PA11ic  PAic(11)
#define PA12ic  PAic(12)
#define PA13ic  PAic(13)
#define PA14ic  PAic(14)
#define PA15ic  PAic(15)
//------
#define PB0ic   PBic(0)
#define PB1ic   PBic(1)
#define PB2ic   PBic(2)
#define PB3ic   PBic(3)
#define PB4ic   PBic(4)
#define PB5ic   PBic(5)
#define PB6ic   PBic(6)
#define PB7ic   PBic(7)
#define PB8ic   PBic(8)
#define PB9ic   PBic(9)
#define PB10ic  PBic(10)
#define PB11ic  PBic(11)
#define PB12ic  PBic(12)
#define PB13ic  PBic(13)
#define PB14ic  PBic(14)
#define PB15ic  PBic(15)
//------
#define PC0ic   PCic(0)
#define PC1ic   PCic(1)
#define PC2ic   PCic(2)
#define PC3ic   PCic(3)
#define PC4ic   PCic(4)
#define PC5ic   PCic(5)
#define PC6ic   PCic(6)
#define PC7ic   PCic(7)
#define PC8ic   PCic(8)
#define PC9ic   PCic(9)
#define PC10ic  PCic(10)
#define PC11ic  PCic(11)
#define PC12ic  PCic(12)
#define PC13ic  PCic(13)
#define PC14ic  PCic(14)
#define PC15ic  PCic(15)
//------
#define PD0ic   PDic(0)
#define PD1ic   PDic(1)
#define PD2ic   PDic(2)
#define PD3ic   PDic(3)
#define PD4ic   PDic(4)
#define PD5ic   PDic(5)
#define PD6ic   PDic(6)
#define PD7ic   PDic(7)
#define PD8ic   PDic(8)
#define PD9ic   PDic(9)
#define PD10ic  PDic(10)
#define PD11ic  PDic(11)
#define PD12ic  PDic(12)
#define PD13ic  PDic(13)
#define PD14ic  PDic(14)
#define PD15ic  PDic(15)
//------

//--------------------------------------------------------------------
// GPIO output, pin=0~15, data=0/1
//--------------------------------------------------------------------

#define PAo(pin,data)  GPIOA->BSRR = (Bit(pin)<<((1-data)*16))
#define PBo(pin,data)  GPIOB->BSRR = (Bit(pin)<<((1-data)*16))
#define PCo(pin,data)  GPIOC->BSRR = (Bit(pin)<<((1-data)*16))
#define PDo(pin,data)  GPIOD->BSRR = (Bit(pin)<<((1-data)*16))

//Individual pin definition for output
#define PA0o0   PAo(0,0)
#define PA0o1   PAo(0,1)
#define PA1o0   PAo(1,0)
#define PA1o1   PAo(1,1)
#define PA2o0   PAo(2,0)
#define PA2o1   PAo(2,1)
#define PA3o0   PAo(3,0)
#define PA3o1   PAo(3,1)
#define PA4o0   PAo(4,0)
#define PA4o1   PAo(4,1)
#define PA5o0   PAo(5,0)
#define PA5o1   PAo(5,1)
#define PA6o0   PAo(6,0)
#define PA6o1   PAo(6,1)
#define PA7o0   PAo(7,0)
#define PA7o1   PAo(7,1)
#define PA8o0   PAo(8,0)
#define PA8o1   PAo(8,1)
#define PA9o0   PAo(9,0)
#define PA9o1   PAo(9,1)
#define PA10o0  PAo(10,0)
#define PA10o1  PAo(10,1)
#define PA11o0  PAo(11,0)
#define PA11o1  PAo(11,1)
#define PA12o0  PAo(12,0)
#define PA12o1  PAo(12,1)
#define PA13o0  PAo(13,0)
#define PA13o1  PAo(13,1)
#define PA14o0  PAo(14,0)
#define PA14o1  PAo(14,1)
#define PA15o0  PAo(15,0)
#define PA15o1  PAo(15,1)
//------
#define PB0o0   PBo(0,0)
#define PB0o1   PBo(0,1)
#define PB1o0   PBo(1,0)
#define PB1o1   PBo(1,1)
#define PB2o0   PBo(2,0)
#define PB2o1   PBo(2,1)
#define PB3o0   PBo(3,0)
#define PB3o1   PBo(3,1)
#define PB4o0   PBo(4,0)
#define PB4o1   PBo(4,1)
#define PB5o0   PBo(5,0)
#define PB5o1   PBo(5,1)
#define PB6o0   PBo(6,0)
#define PB6o1   PBo(6,1)
#define PB7o0   PBo(7,0)
#define PB7o1   PBo(7,1)
#define PB8o0   PBo(8,0)
#define PB8o1   PBo(8,1)
#define PB9o0   PBo(9,0)
#define PB9o1   PBo(9,1)
#define PB10o0  PBo(10,0)
#define PB10o1  PBo(10,1)
#define PB11o0  PBo(11,0)
#define PB11o1  PBo(11,1)
#define PB12o0  PBo(12,0)
#define PB12o1  PBo(12,1)
#define PB13o0  PBo(13,0)
#define PB13o1  PBo(13,1)
#define PB14o0  PBo(14,0)
#define PB14o1  PBo(14,1)
#define PB15o0  PBo(15,0)
#define PB15o1  PBo(15,1)
//------
#define PC0o0   PCo(0,0)
#define PC0o1   PCo(0,1)
#define PC1o0   PCo(1,0)
#define PC1o1   PCo(1,1)
#define PC2o0   PCo(2,0)
#define PC2o1   PCo(2,1)
#define PC3o0   PCo(3,0)
#define PC3o1   PCo(3,1)
#define PC4o0   PCo(4,0)
#define PC4o1   PCo(4,1)
#define PC5o0   PCo(5,0)
#define PC5o1   PCo(5,1)
#define PC6o0   PCo(6,0)
#define PC6o1   PCo(6,1)
#define PC7o0   PCo(7,0)
#define PC7o1   PCo(7,1)
#define PC8o0   PCo(8,0)
#define PC8o1   PCo(8,1)
#define PC9o0   PCo(9,0)
#define PC9o1   PCo(9,1)
#define PC10o0  PCo(10,0)
#define PC10o1  PCo(10,1)
#define PC11o0  PCo(11,0)
#define PC11o1  PCo(11,1)
#define PC12o0  PCo(12,0)
#define PC12o1  PCo(12,1)
#define PC13o0  PCo(13,0)
#define PC13o1  PCo(13,1)
#define PC14o0  PCo(14,0)
#define PC14o1  PCo(14,1)
#define PC15o0  PCo(15,0)
#define PC15o1  PCo(15,1)
//------
#define PD0o0   PDo(0,0)
#define PD0o1   PDo(0,1)
#define PD1o0   PDo(1,0)
#define PD1o1   PDo(1,1)
#define PD2o0   PDo(2,0)
#define PD2o1   PDo(2,1)
#define PD3o0   PDo(3,0)
#define PD3o1   PDo(3,1)
#define PD4o0   PDo(4,0)
#define PD4o1   PDo(4,1)
#define PD5o0   PDo(5,0)
#define PD5o1   PDo(5,1)
#define PD6o0   PDo(6,0)
#define PD6o1   PDo(6,1)
#define PD7o0   PDo(7,0)
#define PD7o1   PDo(7,1)
#define PD8o0   PDo(8,0)
#define PD8o1   PDo(8,1)
#define PD9o0   PDo(9,0)
#define PD9o1   PDo(9,1)
#define PD10o0  PDo(10,0)
#define PD10o1  PDo(10,1)
#define PD11o0  PDo(11,0)
#define PD11o1  PDo(11,1)
#define PD12o0  PDo(12,0)
#define PD12o1  PDo(12,1)
#define PD13o0  PDo(13,0)
#define PD13o1  PDo(13,1)
#define PD14o0  PDo(14,0)
#define PD14o1  PDo(14,1)
#define PD15o0  PDo(15,0)
#define PD15o1  PDo(15,1)
//------

//--------------------------------------------------------------------
// GPIO output toggle, pin=0~15
// (Note: the toggled output is according to the ODR's value
//--------------------------------------------------------------------

#define PAot(pin)  {if(GPIOA->ODR & Bit(pin)) GPIOA->BRR = Bit(pin); else GPIOA->BSRR = Bit(pin);}
#define PBot(pin)  {if(GPIOB->ODR & Bit(pin)) GPIOB->BRR = Bit(pin); else GPIOB->BSRR = Bit(pin);}
#define PCot(pin)  {if(GPIOC->ODR & Bit(pin)) GPIOC->BRR = Bit(pin); else GPIOC->BSRR = Bit(pin);}
#define PDot(pin)  {if(GPIOD->ODR & Bit(pin)) GPIOD->BRR = Bit(pin); else GPIOD->BSRR = Bit(pin);}

//Individual pin definition for output toggle
#define PA0ot   PAot(0)
#define PA1ot   PAot(1)
#define PA2ot   PAot(2)
#define PA3ot   PAot(3)
#define PA4ot   PAot(4)
#define PA5ot   PAot(5)
#define PA6ot   PAot(6)
#define PA7ot   PAot(7)
#define PA8ot   PAot(8)
#define PA9ot   PAot(9)
#define PA10ot  PAot(10)
#define PA11ot  PAot(11)
#define PA12ot  PAot(12)
#define PA13ot  PAot(13)
#define PA14ot  PAot(14)
#define PA15ot  PAot(15)
//------
#define PB0ot   PBot(0)
#define PB1ot   PBot(1)
#define PB2ot   PBot(2)
#define PB3ot   PBot(3)
#define PB4ot   PBot(4)
#define PB5ot   PBot(5)
#define PB6ot   PBot(6)
#define PB7ot   PBot(7)
#define PB8ot   PBot(8)
#define PB9ot   PBot(9)
#define PB10ot  PBot(10)
#define PB11ot  PBot(11)
#define PB12ot  PBot(12)
#define PB13ot  PBot(13)
#define PB14ot  PBot(14)
#define PB15ot  PBot(15)
//------
#define PC0ot   PCot(0)
#define PC1ot   PCot(1)
#define PC2ot   PCot(2)
#define PC3ot   PCot(3)
#define PC4ot   PCot(4)
#define PC5ot   PCot(5)
#define PC6ot   PCot(6)
#define PC7ot   PCot(7)
#define PC8ot   PCot(8)
#define PC9ot   PCot(9)
#define PC10ot  PCot(10)
#define PC11ot  PCot(11)
#define PC12ot  PCot(12)
#define PC13ot  PCot(13)
#define PC14ot  PCot(14)
#define PC15ot  PCot(15)
//------
#define PD0ot   PDot(0)
#define PD1ot   PDot(1)
#define PD2ot   PDot(2)
#define PD3ot   PDot(3)
#define PD4ot   PDot(4)
#define PD5ot   PDot(5)
#define PD6ot   PDot(6)
#define PD7ot   PDot(7)
#define PD8ot   PDot(8)
#define PD9ot   PDot(9)
#define PD10ot  PDot(10)
#define PD11ot  PDot(11)
#define PD12ot  PDot(12)
#define PD13ot  PDot(13)
#define PD14ot  PDot(14)
#define PD15ot  PDot(15)
//------

//------------------------------------------------------------------------------
