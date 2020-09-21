#include "Initial_BEMF_Speed_Detect.h"
#include "adc.h"
#include "led.h"
#include "iwdg.h"
#include "Whole_Motor_Parameters.h"
#include "delay.h"

extern __IO uint16_t u16IWDGPingPong;
extern __IO uint16_t u16ADC1[8];
uint16_t  u16BEMF_A_B_VoltageDiffAverage;
//extern void DAC12bit_show(int32_t s32DACShowData);
/********************************************************************************************************
**函数信息 ：BEMF_Speed_Detect()  //be executed every 1ms              
**功能描述 ：Detect the initial speed and direction by measuring BEMF , be executed every 1ms before motor startup
**输入参数 ：2 phase BEMF
**输出参数 ：BEMF speed and direction
********************************************************************************************************/
void BEMF_Speed_Detect(BEMF_Speed_Struct *Get_BEMF_Speed)
{	 
	  #define BEMF_MIN_PERIOD_TIME                  3    //unit: 1ms, it define the minimum time of BEMF period.
		
   	uint32_t static u32TempBEMFPhaseA12bit;
	  uint32_t static u32TempBEMFPhaseB12bit;
	  uint32_t u32TempBEMFPhaseABDiff12bit;
	  uint32_t u32TempBEMFComparatorOut;
	  uint8_t  static u8BEMFComparatorOut;
	  
	
	  if(Get_BEMF_Speed->u16BEMFDetectionTime <BEMF_DETECT_LIMIT_TIME)//unit : 1ms
		{ 
			Get_BEMF_Speed->u16BEMFDetectionTime++;//for control the time duration of BEMF detection		
			
			if(Get_BEMF_Speed->u16BEMFDetectionTime <50)//change 10ms to 50ms //20190402
       {
			   Get_BEMF_Speed->bBEMFMotorIsRotatingFlag =0;//reset this flag
				 u16BEMF_A_B_VoltageDiffAverage =0;//reset the difference bemf voltage between phase A,B
			 }
					 
			 //----- Get phase A,B bemf voltage----------------------------			 			
		   u32TempBEMFPhaseA12bit = (2*u32TempBEMFPhaseA12bit + 1*u16ADC1[BEMFA_ADC_CHANNEL])/3;//get the BEMF A voltage //20190413
			 u32TempBEMFPhaseB12bit = (2*u32TempBEMFPhaseB12bit + 1*u16ADC1[BEMFB_ADC_CHANNEL])/3;//get the BEMF B voltage //20190413  
			 
			 
			//-----detect the motor is standstill or rotating------------
			if(u32TempBEMFPhaseA12bit > u32TempBEMFPhaseB12bit)
			 { u32TempBEMFPhaseABDiff12bit = u32TempBEMFPhaseA12bit - u32TempBEMFPhaseB12bit;}
			else
			 { u32TempBEMFPhaseABDiff12bit = u32TempBEMFPhaseB12bit - u32TempBEMFPhaseA12bit;}

      u16BEMF_A_B_VoltageDiffAverage = u16BEMF_A_B_VoltageDiffAverage + (u32TempBEMFPhaseABDiff12bit>>2)-(u16BEMF_A_B_VoltageDiffAverage>>2);
      if(u16BEMF_A_B_VoltageDiffAverage > Get_BEMF_Speed->u16BEMFStandstillThresholdVolt)
			 {Get_BEMF_Speed->bBEMFMotorIsRotatingFlag =1;}//confirm the motor is rotating now
			
      //-----if motor is rotating, then increase EMF_1ms_counter every 1ms, for getting the BEMF speed 			 
			if(Get_BEMF_Speed->bBEMFMotorIsRotatingFlag ==1){Get_BEMF_Speed->u16BEMF1msCounter++;}//for detecting rotor speed use only
			 
			//-----use the software to do a comparator function with hystersis------------------------
			u32TempBEMFComparatorOut = u8BEMFComparatorOut;
			
			if(u32TempBEMFPhaseA12bit > (u32TempBEMFPhaseB12bit + Get_BEMF_Speed->u16BEMFComparatorHystersis))
			 {u8BEMFComparatorOut = 1;}
			else if(u32TempBEMFPhaseB12bit > (u32TempBEMFPhaseA12bit + Get_BEMF_Speed->u16BEMFComparatorHystersis))
			 {u8BEMFComparatorOut = 0;}
			
			//-----detect the motor's initial speed and direction of rotation-----------------------		 
			if((u8BEMFComparatorOut != u32TempBEMFComparatorOut)&&(Get_BEMF_Speed->u16BEMF1msCounter>BEMF_MIN_PERIOD_TIME))//if yes, it means now get the cross point of BEMF A,B
			 { //LED_TOGGLE();
				if(u8BEMFComparatorOut ==0)//if yes,it means the software comparator output from 1 t 0 (falling edge)
				{ 
					//---get the motor initial speed--------------------				
					Get_BEMF_Speed->u16BEMFSpeed = ((uint32_t)120*1000)/((uint16_t)Get_BEMF_Speed->u8BEMFPoleNumber * Get_BEMF_Speed->u16BEMF1msCounter);//20181108
					Get_BEMF_Speed->u16BEMF1msCounter =0;//clear the BEMF time counter					
					
					Get_BEMF_Speed->u16BEMFPhaseABMiddlePoint12bit = (Get_BEMF_Speed->u16BEMFPhaseABMiddlePoint12bit + u32TempBEMFPhaseA12bit)/2;
					//---get the motor direction by comparing the midpoint potential voltage---------- 
					if(u32TempBEMFPhaseA12bit > Get_BEMF_Speed->u16BEMFPhaseABMiddlePoint12bit)
					{ Get_BEMF_Speed->u8BEMFDirectionFlag = BEMF_DIR_CW;}//the initial direction is CW
					else
					{ Get_BEMF_Speed->u8BEMFDirectionFlag = BEMF_DIR_CCW;}//the initial direction is CCW

					if( Get_BEMF_Speed->u16BEMFDetectionTime > (BEMF_DETECT_LIMIT_TIME/2))
					 { Get_BEMF_Speed->u16BEMFDetectionTime = BEMF_DETECT_LIMIT_TIME;}//got real bemf speed, so force out of the BEMF speed detection				
				 }
				//----get the midpoint potential voltage of phase A,B (add the 1/4 digital filter for average midpoint voltage)---------
//				u32TempBEMFVoltage12bit = Get_BEMF_Speed->u16BEMFPhaseABMiddlePoint12bit;
//				u32TempBEMFVoltage12bit = u32TempBEMFVoltage12bit+((u32TempBEMFPhaseA12bit+u32TempBEMFPhaseB12bit)>>3)-(u32TempBEMFVoltage12bit>>2); 
//				Get_BEMF_Speed->u16BEMFPhaseABMiddlePoint12bit = u32TempBEMFVoltage12bit;
				Get_BEMF_Speed->u16BEMFPhaseABMiddlePoint12bit = u32TempBEMFPhaseA12bit;				
			}
	  }
	 else{Get_BEMF_Speed->bBEMFResultValidFlag = 1;}
}










































