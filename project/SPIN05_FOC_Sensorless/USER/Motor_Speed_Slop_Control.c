#include "motor_speed_slop_control.h"
#include "sys.h"
#include "Whole_Motor_Parameters.h"

uint16_t u16IncreaseAngle;//increase angle in every pwm period for open loop
uint16_t u16GlobalTestOnly;

#define OPEN_LOOP_Iq_SLOP1 ((OPEN_LOOP_RAMP_UP_CURRENT_SLOP1*CURRENT_GAIN)/(10*10)) //Iq cmd acceleartion slop1 in open loop 1st stage
#define OPEN_LOOP_Iq_SLOP2 ((OPEN_LOOP_RAMP_UP_CURRENT_SLOP2*CURRENT_GAIN)/(10*10)) //Iq cmd acceleartion slop2 in open loop 2nd stage
#define OPEN_LOOP_Iq_SLOP3 ((OPEN_LOOP_RAMP_UP_CURRENT_SLOP3*CURRENT_GAIN)/(10*10)) //Iq cmd acceleartion slop3 in open loop 3th stage

/********************************************************************************************************
**function name        £ºGenerate_Open_loop_Slope_Counter_Value();                
**function description £ºfor getting the open loop speed,Iq acceleration slope counter, executed it in every 1ms loop
**input parameters     £ºnone
**output parameters    £ºnone
********************************************************************************************************/
void Generate_Open_loop_Slope_Counter_Value(Whole_Motor_Struct* Motor)
{
  if(Motor->command.u16SpeedCommandNow >= RAMP_UP_CHANGE_TO_SLOP3_SPEED)
	 { 		    
			Motor->spec.u16SpeedSlope = Motor->spec.u16SpeedSlope3;//change speed ramp up/down acceleration to slop3
		  if(Motor->spec.u16SpeedSlope <1){Motor->spec.u16SpeedSlope = 1;}//20180807
			Motor->command.u16SpeedSlopCnt = 1000 / Motor->spec.u16SpeedSlope;//get how many ms to increase 1 rpm command
			Motor->command.u16IqSlopeCnt = 1000 / OPEN_LOOP_Iq_SLOP3;//get how many ms to increase 10 of Iq command
	 }	
	else if(Motor->command.u16SpeedCommandNow >= RAMP_UP_CHANGE_TO_SLOP2_SPEED)
	 { 		    
			Motor->spec.u16SpeedSlope = Motor->spec.u16SpeedSlope2;//change speed ramp up/down acceleration to slop2
		  if(Motor->spec.u16SpeedSlope <1){Motor->spec.u16SpeedSlope = 1;}//20180807
			Motor->command.u16SpeedSlopCnt = 1000 / Motor->spec.u16SpeedSlope;//get how many ms to increase 1 rpm command			 
			Motor->command.u16IqSlopeCnt = 1000 / OPEN_LOOP_Iq_SLOP2;//get how many ms to increase 10 of Iq command					 
	 }	
	else 
	 { 		    
			Motor->spec.u16SpeedSlope = Motor->spec.u16SpeedSlope1;//change speed ramp up/down acceleration to slop1
		  if(Motor->spec.u16SpeedSlope <1){Motor->spec.u16SpeedSlope = 1;}//20180807
			Motor->command.u16SpeedSlopCnt = 1000 / Motor->spec.u16SpeedSlope;//get how many ms to increase 1 rpm command
			Motor->command.u16IqSlopeCnt = 1000 / OPEN_LOOP_Iq_SLOP1;//get how many ms to increase 10 of Iq command
	 }
			
	if(Motor->command.u16IqSlopeCnt ==0){Motor->command.u16IqSlopeCnt = 1000;}//if user define out of range, will replace it to protection value
	if(Motor->command.u16SpeedSlopCnt ==0){Motor->command.u16SpeedSlopCnt = 1000;}//if user define out of range, will replace it to protection value
}
/********************************************************************************************************
**function name        £ºGenerate_close_loop_Slope_Counter_Value();                
**function description £ºfor getting the close loop speed acceleration slope counter, executed it in every 1ms loop
**input parameters     £ºnone
**output parameters    £ºnone
********************************************************************************************************/
void Generate_Close_loop_Slope_Counter_Value(Whole_Motor_Struct* Motor)//20190626
{
	if(Motor->command.u16SpeedTarget > Motor->command.u16SpeedCommandNow)	
	{ Motor->command.u16SpeedSlopCnt = (1000/CLOSE_LOOP_RAMP_UP_SPEED_SLOP);}//get how many ms to increase 1 rpm speed command			 		
	
	else if(Motor->command.u16SpeedTarget < Motor->command.u16SpeedCommandNow)
	{
    if(Motor->command.u16SpeedCommandNow < CLOSE_LOOP_ULTRA_LOW_SPEED_DOWN_SPEED_THRESHOLD)
		 { Motor->command.u16SpeedSlopCnt = (1000/CLOSE_LOOP_RAMP_DOWN_SPEED_ULTRA_LOW_SLOP);}//for ultra low speed down, get how many ms to decrease 1 rpm speed command
		else
     { Motor->command.u16SpeedSlopCnt = (1000/CLOSE_LOOP_RAMP_DOWN_SPEED_SLOP);}//get how many ms to decrease 1 rpm speed command			
	 }		
	
	if(Motor->command.u8OverPowerLimitFlag==1)//if yes, means power has over limitation//20190606
	   { Motor->command.u16SpeedSlopCnt = 10000;}//if over max power limitation, decrease the influences of this counter.
		 
	//if(Motor->command.u16SpeedSlopCnt ==0){Motor->command.u16SpeedSlopCnt = 1;}//if user define out of range, will replace it to a protection value
}
/********************************************************************************************************
**function name        £ºGenerate_Now_Speed_Command();                
**function description £ºGenerate now's speed command for motor control, executed this subroutine in every 1ms loop
**input parameters     £ºnone
**output parameters    £ºnone
********************************************************************************************************/
void Generate_Now_Speed_Command(Whole_Motor_Struct* Motor)
{
	static uint16_t cnt = 0;

	if(Motor->command.u16SpeedTarget > Motor->command.u16SpeedCommandNow)	
	{  
		if(++cnt >= Motor->command.u16SpeedSlopCnt)  
		{
			Motor->command.u16SpeedCommandNow++;
			cnt = 0;
		}
	}
	else if(Motor->command.u16SpeedTarget < Motor->command.u16SpeedCommandNow)
	{

		if(++cnt >= Motor->command.u16SpeedSlopCnt)  
		{
			Motor->command.u16SpeedCommandNow--;
			cnt = 0;
		}
	}		
}

void Generate_Now_Speed_Command_Close_Loop(Whole_Motor_Struct* Motor)
{
	static uint16_t cnt = 0;

	if(Motor->command.u16SpeedTarget > Motor->command.u16SpeedCommandNow)	
	{  
		if(++cnt >= Motor->command.u16SpeedSlopCnt)  
		{
			Motor->command.u16SpeedCommandNow+= (1+(CLOSE_LOOP_RAMP_UP_SPEED_SLOP/1000));
			cnt = 0;
		}
	}
	else if(Motor->command.u16SpeedTarget < Motor->command.u16SpeedCommandNow)
	{

		if(++cnt >= Motor->command.u16SpeedSlopCnt)  
		{
			Motor->command.u16SpeedCommandNow-= (1+(CLOSE_LOOP_RAMP_DOWN_SPEED_SLOP/1000));
			cnt = 0;
		}
	}		
}
/********************************************************************************************************
**function name        £ºGenerate_Open_Loop_Now_Iq_Command();                
**function description £ºgenerate the s16OpenLoopNowIq for open loop Iq command ramp up control, executed it in every 1ms loop
**input parameters     £ºnone
**output parameters    £ºnone
********************************************************************************************************/
void Generate_Open_Loop_Now_Iq_Command(Whole_Motor_Struct* Motor)
{
	static uint16_t u16Cnt = 0;

	if(Motor->command.s16OpenLoopNowIq > Motor->command.s16OpenLoopTargetIq)	
	{
		if(++u16Cnt >= Motor->command.u16IqSlopeCnt)  
		{
			Motor->command.s16OpenLoopNowIq = Motor->command.s16OpenLoopNowIq -10;;
			u16Cnt = 0;
		}
	}
	else if(Motor->command.s16OpenLoopNowIq < Motor->command.s16OpenLoopTargetIq)
	{

		if(++u16Cnt >= Motor->command.u16IqSlopeCnt)  
		{
			Motor->command.s16OpenLoopNowIq = Motor->command.s16OpenLoopNowIq +10;
			u16Cnt = 0;
		}
	}		
}
/********************************************************************************************************
**function name        £ºvoid Generate_New_Startup_Now_Iq_Command(Whole_Motor_Struct* Motor)                
**function description £ºgenerate the s16New_Startup_NowIq for new startup mode Iq command ramp up control, executed it in every 1ms loop
**input parameters     £ºnone
**output parameters    £ºnone
********************************************************************************************************/
void Generate_New_Startup_Now_Iq_Command(Whole_Motor_Struct* Motor)
{
	#ifdef ENABLE_NEW_STARTUP_METHOD
	static uint16_t u16Cnt = 0;
	
	if(Motor->NewStartup.s16New_Startup_NowIq > Motor->NewStartup.s16New_Startup_TargetIq)	
	{
		if(++u16Cnt >= Motor->NewStartup.u16New_StartupIqSlopeCnt)  
		{
			Motor->NewStartup.s16New_Startup_NowIq = Motor->NewStartup.s16New_Startup_NowIq - Motor->NewStartup.u16New_StartupIqAddPer_ms;
			u16Cnt = 0;
		}
	}
	else if(Motor->NewStartup.s16New_Startup_NowIq < Motor->NewStartup.s16New_Startup_TargetIq)
	{

		if(++u16Cnt >= Motor->NewStartup.u16New_StartupIqSlopeCnt)  
		{
			Motor->NewStartup.s16New_Startup_NowIq = Motor->NewStartup.s16New_Startup_NowIq + Motor->NewStartup.u16New_StartupIqAddPer_ms;
			u16Cnt = 0;
		}
	}	
 #endif	
}
/********************************************************************************************************
**function name        £ºGenerate_Open_Loop_Angle();               
**function description £ºgenerate the open loop angle when in open loop mode, executed it in every PWM period
**input parameters     £ºnone
**output parameters    £ºnone
********************************************************************************************************/
void Generate_Open_Loop_Angle(Whole_Motor_Struct* Motor)
{
	uint16_t temp_increase_angle;
	
	static uint32_t u32CompensationIncreaseAngle =0;
	
	temp_increase_angle = (uint32_t)(Motor->command.u16SpeedCommandNow * u16IncreaseAngle)>>15;//syn theta output
	
	if(temp_increase_angle ==0)//20190122 for very low speed increase angle calculation
	{
		u32CompensationIncreaseAngle +=(uint32_t)(Motor->command.u16SpeedCommandNow * u16IncreaseAngle);
		if(u32CompensationIncreaseAngle >= 32768)
		{
      u32CompensationIncreaseAngle -= 32768;
			temp_increase_angle =1;
		}					
	}
	
	Motor->command.u16CommandTheta +=temp_increase_angle;
	
	if(Motor->command.u16CommandTheta > 32767){Motor->command.u16CommandTheta -=32768;}
	Motor->command.u16Angle = (Motor->command.u16CommandTheta) >>5;//for loop up table 0~1023	
}
/********************************************************************************************************
**function name        £ºGet_Angle_Difference();               
**function description £ºget the difference angle between smo angle and open loop angle
**input parameters     £ºnone
**output parameters    £ºnone
********************************************************************************************************/
void Get_Angle_Difference(Whole_Motor_Struct* Motor)
{
	#define MAX_DIFF_ANGLE   70   //70 = 25 degree, Range (0~1023) for (0~360) degree
	int16_t temp_angle_difference;
	
	temp_angle_difference = Motor->smo.s16Angle - Motor->command.u16Angle;
	
	if(temp_angle_difference > 800)
	{Motor->command.s16AngleDiff = 1024 + (Motor->command.u16Angle) - Motor->smo.s16Angle;}
	else if(temp_angle_difference < (-800))
	{Motor->command.s16AngleDiff = 1024 + (Motor->smo.s16Angle) - Motor->command.u16Angle;}
	else if(temp_angle_difference < (0))
	{Motor->command.s16AngleDiff = -temp_angle_difference;}
	else
	{Motor->command.s16AngleDiff = temp_angle_difference;}	
	
	if(Motor->command.s16AngleDiff > MAX_DIFF_ANGLE){Motor->command.s16AngleDiff = MAX_DIFF_ANGLE;}
}










































