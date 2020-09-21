
#ifndef SMO_H
#define SMO_H

#include "FOC_PMSM.h"

#define INITIAL_NEW_STARTUP_STATE    0
#define FREE_RUN_STATE               1
#define FORCE_RE_STARTUP_STATE       2

typedef struct  
{
  __IO int16_t	s16Theta;	                 // Range from 0~32767 
	__IO int16_t	s16Angle;                  // Range from 0~1023
	__IO int32_t	s16Speed;		
	__IO int32_t	s32Kslid;                  // Range from 0~32767
	__IO int32_t	s32Kslf;								   // Range from 0~32767
	__IO int32_t	s32KF;                     // Range from 0~32767 
	__IO int32_t	s32KG;                     // Range from 0~32767 
	__IO int16_t	s16SpeedEstGainQ15;				 // s16SpeedEstGainQ15 = (Freq of speedloop) * 60 *(2/Poles of motor)--> rpm = delta s16Theta * SpeedEST_Gain/32768	
	__IO int8_t  u8SmoPole;                  // assign pole number
	__IO int8_t  u8SpeedLoop500hzFlag;       // output speed calculatio is 500 or 1KHz
	__IO int16_t u16RotationInverseCounter;  // the counter of rotation inverse error, it checks rotation inverse or not in smo speed calculation subroutine 
	__IO int32_t s32ValphaToSMO;       			 // FOC parameter for SMO calculation//20190530
	__IO int32_t s32VbetaToSMO;        			 // FOC parameter for SMO calculation//20190530
	__IO int32_t s32IalphaToSMO;       			 // FOC parameter for SMO calculation//20190530
	__IO int32_t s32IbetaToSMO;        			 // FOC parameter for SMO calculation//20190530
}SMO_Struct;

typedef struct  
{
  __IO uint16_t  u16NewStartupTime_10ms_total;   //unit:10ms, new startup total time define //20190626
	__IO uint16_t  u16NewStartupMaxSpeedLimit;     //unit: RPM, new startup over speed define //20190626
	__IO uint16_t  u16NewStartupTime_10ms_counter; //unit:10ms, new startup 10ms counter //20190626
	__IO uint16_t  u16NewStartupRampUpTime;        //16=1ms for 16KHz PWM Freq.//20190626
	__IO uint16_t  u16NewStartupPWMCyclesToIncrease1Angle;//define how many pwm cycles to increase 1 angle//20190626
	__IO uint16_t  u16NewStartupPWMCounter;        //new startup PWM counter//20190626
	__IO uint16_t  u16NewStartup100msCounter;      //new startup 100ms counter//20190626
	__IO int16_t   s16New_Startup_NowIq;           //new startup now Iq command//20190626
	__IO int16_t   s16New_Startup_TargetIq;        //new startup target Iq command//20190626
	__IO int16_t   u16New_StartupIqSlopeCnt;	     //motor new startup mode conuter for Iq command acceleration//20190626
	__IO int16_t   u16New_StartupIqAddPer_ms;	     //motor new startup mode Iq command increase per ms, for Iq command acceleration//20190626
	__IO uint8_t   u8NewStartupState;              //new startup state control//20190626
}New_Startup_Struct;

short SMO_Position_Calc(SMO_Struct* Motor, FOC_Struct* FOC);
short SMO_Position_Calc_HDIV(SMO_Struct* Motor, FOC_Struct* FOC);
void  New_Startup_Mode_Process(New_Startup_Struct* Motor,SMO_Struct* SMO);//20190626
#endif
