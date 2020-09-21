/**
******************************************************************************
* @file    pwm.c
* @brief   配置定时器TIM1,PA8\PA9\PA10\PB13\PB14\PB15输出6路PWM波形
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "pwm.h"
#include "led.h"
#include "adc.h"
#include "sys.h"
#include "Whole_Motor_Parameters.h"
#include "HAL_conf.h"
//#include "HAL_rcc.h"
//#include "HAL_gpio.h"
#include "HAL_comp.h"
//#include "comp.h"

/*对应PWM输出管脚
PA08->TIM1_CH1,CH1正常输出
PA09->TIM1_CH2,CH2正常输出
PA10->TIM1_CH3,CH3正常输出
PB13->TIM1_CH1N,CH1反向输出
PB14->TIM1_CH2N,CH2反向输出
PB15->TIM1_CH3N,CH3反向输出
*/
//频率=(48*1000000/(999+1))*(0+1)=48KHz Period=999, Prescaler=0
//PWM输出初始化
//Period：自动重装值
//Prescaler：时钟预分频数

__IO uint32_t u32DMAtoT1CC1[2];
__IO uint32_t u32DMAtoT1CC2[2];
__IO uint32_t u32DMAtoT1CC3[2];
__IO int32_t  s32DMAtoADDR5[3];
__IO uint32_t u32DMAtoT3CC1[2];

extern __IO uint32_t u32DMA1ShuntRADCCHAddress;

void NVIC_Configuration4TTIM1(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;

	#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x1;//0:highest priority, 3:lowest priority
	#else
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;//0:highest priority, 3:lowest priority
	#endif

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}

#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
void NVIC_Configuration4TTIM3(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;//0:highest priority, 3:lowest priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
#endif

#ifdef ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION //enable this to do over current protection (PWM OFF), if comparator output Low signal
#ifdef ENABLE_TIM1BKIN_PIN_EXTERNAL_INPUT
void TIM1_BKIN_External_Input_Pin_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  		//开启GPIOB时钟
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12; 				    //PB12=TIM1_BKIN
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;  //浮空输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_2);
}
#endif
#endif

void TIM1_PWM_Init(uint16_t u16Period,uint16_t u16Prescaler,uint8_t u8DeadTime)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;


	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA| RCC_AHBPeriph_GPIOB , ENABLE);//GPIOA,GPIOB外设时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //TIM1外设时钟使能

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;//TIM1_CH1,TIM1_CH2,TIM1_CH3输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//TIM1_CH1N,TIM1_CH2N,TIM1_CH3N输出

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,GPIO_AF_2);//GPIO端口复用功能使能
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_2);


//	TIM_Cmd(TIM1, ENABLE);  //使能TIM1开始计数
//  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = u16Period; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =u16Prescaler; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_CenterAligned2;//for ADC trigger effective when TIM_CounterMode_Up;

	#ifdef ENABLE_TIM1_CC4_CC5_TO_TRIG_ADC1_FOR_1_SHUNTR
		TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_CenterAligned1;//for ADC trigger, TIM Down count effective, TIM_CounterMode_down
	#endif

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; //重复计数器清0 = Update Event at every overflow
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);  //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCStructInit(&TIM_OCInitStructure); //初始化TIM_OCInitStructure
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //CH1比较输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //CH1N比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //CH1输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; //CH1N输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set; //CH1输出空闲状态:1
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //CH1N输出空闲状态:0

	TIM_OCInitStructure.TIM_Pulse = u16Period>>1;//设置待装入捕获比较寄存器的脉冲值
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //CH1,CH1N根据TIM_OCInitStruct中指定的参数初始化外设TIM1
	TIM_OCInitStructure.TIM_Pulse = u16Period>>1;//设置待装入捕获比较寄存器的脉冲值
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //CH2,CH2N根据TIM_OCInitStruct中指定的参数初始化外设TIM1
	TIM_OCInitStructure.TIM_Pulse = u16Period>>1;//设置待装入捕获比较寄存器的脉冲值
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //CH3,CH3N根据TIM_OCInitStruct中指定的参数初始化外设TIM1

	/* Divided clock for TIM1*/
	TIM1->PSC = 0x00;  // CLK = HSI/(0+1) = 48MHz/1 = 48MHz

	#ifdef ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION   //enable this to do over current protection (PWM OFF), if comparator output Low signal
  {
		TIM_BDTRInitTypeDef TIM_BDTRInitStruct;

		TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;//TIM_OSSRState_Enable;//运行模式下关闭状态选择
		TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;//TIM_OSSIState_Enable;//空闲模式下关闭状态选择
		TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;//软件错误锁定配置:锁定关闭无保护
		TIM_BDTRInitStruct.TIM_DeadTime  = u8DeadTime;//DTG[7:0]死区发生器配置:(死区时间DT)
		TIM_BDTRInitStruct.TIM_Break     = TIM_Break_Enable;//TIM_Break_Disable;//TIM_Break_Enable;  //刹车配置：使能刹车
		TIM_BDTRInitStruct.TIM_BreakPolarity   = BREAK_POLARITY;//TIM_BreakPolarity_High;//TIM_BreakPolarity_Low;//刹车输入极性选择
		TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;//TIM_AutomaticOutput_Enable;//自动输出使能配置:MOE只能软件置1
		TIM_BDTRConfig( TIM1, &TIM_BDTRInitStruct); //配置互补输出死区时间
  }
  #else
	/* Dead Time Setting */
	TIM1->BDTR |= 0x00|u8DeadTime; // Setting Dead Time [7:0] = 0x30 = 1us at 48MHz
	#endif
//--------------------------------------------------------------------------------------------------------------------------------
	#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Disable );  //Disable Preload, CCR will be updated right away when new value be written.
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Disable );  //Disable Preload, CCR will be updated right away when new value be written.
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Disable );  //Disable Preload, CCR will be updated right away when new value be written.

	TIM_ARRPreloadConfig(TIM1, ENABLE);    //使能TIM1在ARR上的预装载寄存器
	//TIM_CtrlPWMOutputs(TIM1, ENABLE);    //使能TIM1 PWM互补死区输出

	#ifdef ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION
	TIM_ITConfig(TIM1, TIM_IT_Update | TIM_IT_Break , ENABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Break | TIM_FLAG_Update);
	#else
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  // Enable TIM1 Update Event Interrupt
	#endif

	NVIC_Configuration4TTIM1();
	#endif
//--------------------------------------------------------------------------------------------------------------------------------
  #ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable );  //使能自动装载
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable );  //使能自动装载
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable );  //使能自动装载

	TIM_ARRPreloadConfig(TIM1, ENABLE);    //使能TIM1在ARR上的预装载寄存器
	//TIM_CtrlPWMOutputs(TIM1, ENABLE);      //使能TIM1 PWM互补死区输出

	#ifdef ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION
//	TIM_ITConfig(TIM1, TIM_IT_Update | TIM_IT_Break , ENABLE);
//	TIM_ClearITPendingBit(TIM1, TIM_IT_Break | TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Break , ENABLE);
	  TIM_ClearITPendingBit(TIM1, TIM_IT_Break | TIM_FLAG_Update);
//	#else
//	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  // Enable TIM1 Update Event Interrupt
	#endif

	NVIC_Configuration4TTIM1();

	TIM1->BDTR |= 0x8000; // 主输出使能

	#ifdef ENABLE_TIM1_CC4_TO_TRIG_ADC_FOR_2_SHUNTR//for use TIM1_CC4 to trigger ADC in 2 shunt R phase current sensing
	TIM_Cmd(TIM1, ENABLE);//Enable TIM1 only
	#endif

	#endif
}

void Enable_Motor1_PWM_Output(void)//PWM開始為互補
{
	TIM_CtrlPWMOutputs(TIM1, ENABLE);      //使能TIM1 PWM互补死区输出
//	TIM1->CCMR1=0x6868;   // Output keep PWM1 mode (CH2/CH1)
//	TIM1->CCMR2=0x0068;   // Output keep PWM1 mode (CH4/CH3)
//	TIM1->CCER =0x0555;   // D:CCxNP=0/CCxNE=1/CCxP=0/CCxE=1
//	TIM1->EGR =0x0020;    // Active immediately : PWM is complementary type and H_side = High active L_side = High active
}

void Disable_Motor1_PWM_Output(void)//ＰＷＭ關閉為０輸出
{
	TIM_CtrlPWMOutputs(TIM1, DISABLE);      //Disable TIM1 PWM互补死区输出
//	TIM1->CCMR1=0x5858;   // Output keep OCREF=1 (CH2/CH1)
//	TIM1->CCMR2=0x0058;   // Output keep OCREF=1 (CH4/CH3)
//	TIM1->CCER =0x0CCC;   // D:CCxNP=1/CCxNE=1/CCxP=0/CCxE=0
//	TIM1->EGR =0x0020;    // Active immediately : H_side = 0, Low_side = 0
}


/* TIM3 Initialize */
#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
void TIM3_PWM_Init(uint16_t u16Period,uint16_t u16Prescaler)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);//GPIOA,GPIOB外设时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//TIM3_CH1输出, PB4
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,GPIO_AF_1);//GPIO端口复用功能使能


	TIM_TimeBaseStructure.TIM_Period=u16Period;                                                                      //ARR寄存器值
	TIM_TimeBaseStructure.TIM_Prescaler=u16Prescaler;                                                                //预分频值
	/*数字滤波器采样频率,不影响定时器时钟*/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;                                                         //采样分频值
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned1;                                         //计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* TIM3 need to be symmetric by TIM1 */
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);
	TIM_SelectInputTrigger(TIM3, TIM_TS_ITR0);  // TIM_TS_ITR0 = Symmetric by TIM1
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Gated);
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

	TIM_OCStructInit(&TIM_OCInitStructure); //初始化TIM_OCInitStructure
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //CH1比较输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //CH1N比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //CH1输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; //CH1N输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set; //CH1输出空闲状态:1
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //CH1N输出空闲状态:0

	TIM_OCInitStructure.TIM_Pulse = u16Period>>1; // Close to center value
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);   //CH1,CH1N根据TIM_OCInitStruct中指定的参数初始化外设TIM1

	/* Divided clock for TIM1*/
	TIM3->PSC = 0x00;  // CLK = HSI/(0+1) = 48MHz/1 = 48MHz
  TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Disable );//Disable Preload, CCR will be updated right away when new value be written.
	TIM_ARRPreloadConfig(TIM3, ENABLE);    //使能TIM3在ARR上的预装载寄存器
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

  /* Enable DMA of TIM3 */
	#ifndef ENABLE_TIM1_CC4_CC5_TO_TRIG_ADC1_FOR_1_SHUNTR
		TIM_DMACmd(TIM3,TIM_DMA_CC1,ENABLE);
	#endif

//  TIM_DMACmd(TIM3,TIM_DMA_Update,ENABLE);
	/* Enable UDE Interrupt of TIM3 */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  // Enable TIM3 Update Event Interrupt
	NVIC_Configuration4TTIM3();

	/* Must Enable TIM1, TIM3 Simultaneously */
	TIM_Cmd(TIM3, ENABLE);//TIM3 Controlled by TIM1
	TIM_Cmd(TIM1, ENABLE);//Must Enable TIM1, TIM3 Simultaneously
}
#endif

#ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
	#ifndef ENABLE_TIM1_CC4_TO_TRIG_ADC_FOR_2_SHUNTR
	void TIM3_PWM_Init(uint16_t u16Period,uint16_t u16Prescaler)
	{
	//	GPIO_InitTypeDef GPIO_InitStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);//GPIOA,GPIOB外设时钟使能
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//TIM3_CH1输出
	//
	//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//
	//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6,GPIO_AF_1);//GPIO端口复用功能使能

		TIM_TimeBaseStructure.TIM_Period=u16Period;                                                                      //ARR寄存器值
		TIM_TimeBaseStructure.TIM_Prescaler=u16Prescaler;                                                                //预分频值
		/*数字滤波器采样频率,不影响定时器时钟*/
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;                                                         //采样分频值
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned2;                                         //计数模式
		TIM_TimeBaseStructure.TIM_RepetitionCounter=0;

		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		/* TIM3 need to be symmetric by TIM1 */
		TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);
		TIM_SelectInputTrigger(TIM3, TIM_TS_ITR0);  // TIM_TS_ITR0 = Symmetric by TIM1
		TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Gated);
		TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

		TIM_OCStructInit(&TIM_OCInitStructure); //初始化TIM_OCInitStructure
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //CH1比较输出使能
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //CH1N比较输出使能
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //CH1输出极性:TIM输出比较极性高
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; //CH1N输出极性:TIM输出比较极性高
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set; //CH1输出空闲状态:1
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; //CH1N输出空闲状态:0

		TIM_OCInitStructure.TIM_Pulse = 5; // Close to center value
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);   //CH1,CH1N根据TIM_OCInitStruct中指定的参数初始化外设TIM1

		/* Divided clock for TIM1*/
		TIM3->PSC = 0x00;  // CLK = HSI/(0+1) = 48MHz/1 = 48MHz
		TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable );  //使能自动装载
		TIM_ARRPreloadConfig(TIM3, ENABLE);    //使能TIM3在ARR上的预装载寄存器
		TIM_CtrlPWMOutputs(TIM3, ENABLE);

		/* Must Enable TIM1, TIM3 Simultaneously */
		TIM_Cmd(TIM3, ENABLE);//TIM3 Controlled by TIM1
		TIM_Cmd(TIM1, ENABLE);//Must Enable TIM1, TIM3 Simultaneously
	}
	#endif
#endif


#ifdef ENABLE_OVER_CURRENT_COMP1_PROTECTION

/*

+------------------------------------------+
|                 |                | COMP1 |
|-----------------|----------------|-------|
|                 | CMP1_INM0	00 |  PA5  |
|  CSR(bit5:4)    | CMP1_INM1	01 |  PA6  |
|                 | CMP1_INM2	10 |  PA7  |
| Inverting Input | CMP1_INM3	11 |  CRV  |
|-----------------|----------------|-------|
|  Non Inverting  | CMP1_INP_SEL 00|  PA1  |
|    Input        | CMP1_INP_SEL 01|  PA2  |
|  CSR(bit8:7)    | CMP1_INP_SEL 10|  PA3  |
|    		          | CMP1_INP_SEL 11|  PA4  |
+------------------------------------------+

*/
#ifdef MM32SPIN05
void Init_Comparator(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;                                           //GPIO Structure
    COMP_InitTypeDef COMP_InitStructure;                                           //COMP Structure
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);                            //GPIOA Clock
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                  //GPIO Anolog Input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;// | GPIO_Pin_1;                      //PA4
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                              //GPIO Speed 50Mhz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                         //GPIOA Init

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_COMP, ENABLE);                           //COMP Clock
    //	COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_IO2;				     //COMP Inverting Input PA6
	//	COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO1;		   //COMP Noninverting Input PA0
	//	COMP_InitStructure.COMP_InvertingInput    = COMP_InvertingInput_DAC1;		     //COMP Inverting Input PA4
	//  COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO4;	     //COMP Noninverting Input PA3
    COMP_InitStructure.COMP_InvertingInput    = COMP_InvertingInput_CRV;           //COMP Inverting Input internal CRV reference voltage
    COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO3;	       //COMP Noninverting Input PA4

		COMP_InitStructure.COMP_Output = COMP_Output_TIM1BKIN;                         //Trigger Timer1 Break
		//COMP_InitStructure.COMP_BlankingSrce = COMP_BlankingSrce_None;
		COMP_InitStructure.COMP_OutputPol = COMP_OutputPol_NonInverted;                //Output Polarity
		COMP_InitStructure.COMP_Hysteresis = COMP_Hysteresis_Medium;                   //Hysteresis Voltage Level
		COMP_InitStructure.COMP_Mode = COMP_Mode_MediumSpeed;                          //Speed and Power Consumption Level
	  COMP_InitStructure.COMP_Filter = COMP_Filter_4_Period;                         //COMP_Filter_4_Period;
		COMP_Init(COMP_Selection_COMP1, &COMP_InitStructure);                          //COMP1 Init

		//ADC_TempSensorCmd(ENABLE);//for enable internal 1.2V vref
		#if 0
			ADC_VrefintCmd(ENABLE);
			SET_COMP_CRV(cHardCurrentVref, cHardCurrent);
		#else
			SET_COMP_CRV(COMP_CRV_Sele_AVDD, INTERNAL_COMPARATOR_INM_REF_CRV_VOLTAGE);//(3_24) real is (3-2)/20 of 5V = 0.25V,(6_24) real is (6-2)/20 of 5V = 1V
		#endif

		COMP_Cmd(COMP_Selection_COMP1, ENABLE);                                        //COMP1 Enable
	}
	#endif
//---------------------------------------------------------------------------------------------------------------------------
/*

+--------------------------------------------------+
|                 |                | COMP1 | COMP2 |
|-----------------|----------------|---------------|
|                 | 1/4 VREFINT    |  OK   |  OK   |
|                 | 1/2 VREFINT    |  OK   |  OK   |
|                 | 3/4 VREFINT    |  OK   |  OK   |
| Inverting Input | VREFINT        |  OK   |  OK   |
|                 | DAC1 OUT (PA4) |  OK   |  OK   |
|                 | DAC2 OUT (PA5) |  OK   |  OK   |
|                 | IO1            |  PA0  |  PA2  |
|                 | IO2            |  PA6  |  PA6  |
|-----------------|----------------|-------|-------|
|  Non Inverting  | IO1            |  PA0  |  PA0  |
|    Input        | IO2            |  PA1  |  PA1  |
|    		      | IO3            |  PA2  |  PA2  |
|    		      | IO4            |  PA3  |  PA3  |
|    		      | IO5            |  PA4  |  PA4  |
|    		      | IO6            |  PA5  |  PA5  |
|    		      | IO7            |  PA6  |  PA6  |
|    		      | IO8            |  PA7  |  PA7  |
+--------------------------------------------------+

*/
#ifdef MM32SPIN06
	void Init_Comparator(void)
	{

		GPIO_InitTypeDef GPIO_InitStructure;                                           //GPIO Structure
		COMP_InitTypeDef COMP_InitStructure;                                           //COMP Structure

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);                            //GPIOA Clock
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                  //GPIO Anolog Input
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;// | GPIO_Pin_1;                      //PA4
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                              //GPIO Speed 50Mhz
		GPIO_Init(GPIOA, &GPIO_InitStructure);                                         //GPIOA Init

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_COMP, ENABLE);                           //COMP Clock
	//	COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_IO2;				     //COMP Inverting Input PA6
	//	COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO1;		   //COMP Noninverting Input PA0
	//	COMP_InitStructure.COMP_InvertingInput    = COMP_InvertingInput_DAC1;		     //COMP Inverting Input PA4
	//  COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO4;	     //COMP Noninverting Input PA3
		COMP_InitStructure.COMP_InvertingInput    = INTERNAL_COMPARATOR_INM_REF_VOLTAGE;//COMP Inverting Input internal vref 1.2V
		COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO5;	       //COMP Noninverting Input PA4

		COMP_InitStructure.COMP_Output = COMP_Output_TIM1BKIN;                         //Trigger Timer1 Break
		COMP_InitStructure.COMP_BlankingSrce = COMP_BlankingSrce_None;
		COMP_InitStructure.COMP_OutputPol = COMP_OutputPol_NonInverted;                //Output Polarity
		COMP_InitStructure.COMP_Hysteresis = COMP_Hysteresis_Medium;                   //Hysteresis Voltage Level
		COMP_InitStructure.COMP_Mode = COMP_Mode_MediumSpeed;                          //Speed and Power Consumption Level
		COMP_Init(COMP_Selection_COMP1, &COMP_InitStructure);                          //COMP1 Init

		COMP_Cmd(COMP_Selection_COMP1, ENABLE);                                        //COMP1 Enable
	}
	#endif
#endif

#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
///////////ADC DMA/////////////////////////////////////
void DMA_ADCTrigDMA_init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel1);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	  DMA_InitStructure.DMA_PeripheralBaseAddr = u32DMA1ShuntRADCCHAddress;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)s32DMAtoADDR5;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 2;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA_M2M_Enable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
//	  DMA_Cmd(DMA1_Channel1, ENABLE);

}

///////////TIM3 DMA/////////////////////////////////////
void DMA_Timer3CC1TrigDMA_init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel4);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM3->CCR1));
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)u32DMAtoT3CC1;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA_M2M_Enable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

		DMA_Cmd(DMA1_Channel4, ENABLE);
}
#endif





