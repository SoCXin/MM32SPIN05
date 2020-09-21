//******************************************************************************
// delay.c
//******************************************************************************

//! Note: Optimization Level 3 & MM32L073 runs at 48MHz

#define delay_cnt_10us 76 //use 76 @48MHz
#define delay_cnt_100us 599 //use 599 @48MHz
#define delay_cnt_1ms 6020 //use 6020 @48MHz

//==============================================================================
void delay_10us(unsigned int n)
{
	unsigned int dly;

	while(n!=0){
		dly=delay_cnt_10us;	while(dly!=0) dly--;
		n--;
	}
}

//----------------------------------------------------------
void delay_100us(unsigned int n)
{
	unsigned int dly;

	while(n!=0){
		dly=delay_cnt_100us;	while(dly!=0) dly--;
		n--;
	}
}

//----------------------------------------------------------
//void delay_ms(unsigned int n)
//{
//	unsigned int dly;

//	while(n!=0){
//		dly=delay_cnt_1ms;	while(dly!=0) dly--;
//		n--;
//	}
//}

//----------------------------------------------------------
void delay_1us(void)
{
	unsigned int dly;
	dly=5; while(dly!=0) dly--; //use dey=5
}

//
void delay_2us(void)
{
	unsigned int dly;
	dly=13; while(dly!=0) dly--; //use dly=13
}

//
void delay_5us(void)
{
	unsigned int dly;
	dly=37; while(dly!=0) dly--; //use dly=37
}

//==============================================================================
