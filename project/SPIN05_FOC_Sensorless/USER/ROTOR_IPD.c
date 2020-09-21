
#include "ROTOR_IPD.h"
#include "sys.h"
#include "adc.h"
#include "Whole_Motor_Parameters.h"
#include "svpwm.h"

extern int16_t Generate_Rotor_Angle_By_BEMF_AB(int32_t s32BEMF_A,int32_t s32BEMF_B);
extern int16_t Generate_Rotor_Angle_By_BEMF_AB_HDIV(int32_t s32BEMF_A,int32_t s32BEMF_B);
extern int16_t SMO_Atan(int32_t s32Y, int16_t s16X);
extern int16_t SMO_Atan_HDIV(int32_t s32Y, int16_t s16X);
extern __IO uint16_t u16ADC1[8];

int16_t Generate_Rotor_Angle_By_BEMF_AB(int32_t s32BEMF_A,int32_t s32BEMF_B);
uint16_t Generate_Rotor_BEMF_AB_Absolute_Value(int32_t s32BEMF_A,int32_t s32BEMF_B);

uint32_t s32RIPD2IdSumTotal[11];

#ifdef ENABLE_ROTOR_IPD2_FUNCTION 
const uint16_t u16IPD2Table[]={// IPD2  Index  Angle, 85 = 30 degree (1023 = 360 degree)  
 0, 512, 85, 597, 170, 682, 255, 767, 340, 852, 425, 937};
#endif
 
#ifdef ENABLE_ROTOR_IPD1_FUNCTION 
void Rotor_IPD_Mode_Process(void)//execute this program in 1ms loop
{
       Read_IPD_BEMF_AB_Voltage(&Motor1.IPD);//read BEMF A, BEMF B voltage from ADC BEMF Voltage buffer (12bit)
	
	     #ifdef ENABLE_HARDWARE_DIVIDER
			 Motor1.IPD.s16Theta = Generate_Rotor_Angle_By_BEMF_AB_HDIV( Motor1.IPD.s16BEMFPhaseA12bit,Motor1.IPD.s16BEMFPhaseB12bit);//generate s16Theta, Range from 0~32767
	     #else
	     Motor1.IPD.s16Theta = Generate_Rotor_Angle_By_BEMF_AB( Motor1.IPD.s16BEMFPhaseA12bit,Motor1.IPD.s16BEMFPhaseB12bit);//generate s16Theta, Range from 0~32767
	     #endif
	
			 Motor1.IPD.s16Angle = ((int32_t)Motor1.IPD.s16Theta*360)>>15;//generate s16Angle, range is 0~359
			
			 if((Motor1.IPD.u16DecTimer_1ms <= 25)&&(Motor1.IPD.u16DecTimer_1ms >= 1))
			 {
				 Motor1.IPD.s16DiffTheta_1ms = Motor1.IPD.s16Theta - Motor1.IPD.s16LastTheta;//generate the different anlge in 1ms interval
				 
				 if((Motor1.IPD.s16Theta <(32768-(32768/100)))&&(Motor1.IPD.s16Theta >(32768/100)))//360degree+_3.6degree will not test its Theta quality
				 {
           if((Motor1.IPD.s16DiffTheta_1ms >(32768/10))||(Motor1.IPD.s16DiffTheta_1ms <(-32768/10))){Motor1.IPD.u8RIPDQualifyFailFlag = 1;}//qualify the IPD.s16Theta quality.
				 }
				 else{Motor1.IPD.u8RIPDQualifyFailFlag = 1;}//give up when bemf angle is nearly 360 degree
				 
				 Motor1.IPD.u16BEMFABSUM12bit = Generate_Rotor_BEMF_AB_Absolute_Value(Motor1.IPD.s16BEMFPhaseA12bit,Motor1.IPD.s16BEMFPhaseB12bit);
				 if(Motor1.IPD.u16BEMFABSUM12bit < ((ROTOR_IPD1_BEMF_SUM_MINI*5)>>2)){Motor1.IPD.u8RIPDQualifyFailFlag = 1;}//the absolute sum of Bemf A,B must > ROTOR_IPD1_BEMF_SUM_MINI.
				 
				 Motor1.IPD.s16EndTheta = Motor1.IPD.s16Theta;
				 Motor1.IPD.s16DiffTheta = Motor1.IPD.s16EndTheta - Motor1.IPD.s16StartTheta;
				 
				 if((Motor1.IPD.u16DecTimer_1ms <= 5)&&(Motor1.IPD.u16DecTimer_1ms >=1))
				 {   
					   Motor1.IPD.u8RIPDDetectedDIRReverseFlag =0;//Clear the forward/reverse direction judgement flag
					 
						 if(Motor1.IPD.s16DiffTheta >(32768/2))//if yes,it means the rotor direction is in reverse status
						 {
							 Motor1.IPD.s16DiffTheta = 32768 - Motor1.IPD.s16DiffTheta; 
							 Motor1.IPD.s16EndTheta = Motor1.IPD.s16EndTheta + (32768/2);//it is in reverse status, so add 180 degree
							 Motor1.IPD.u8RIPDDetectedDIRReverseFlag =1;//detected the rotor direction is in reverse status
							}
						 else if(Motor1.IPD.s16DiffTheta <(-32768/2))//if yes,it means the rotor direction is in forward status
						 {
							 Motor1.IPD.s16DiffTheta = 32768 + Motor1.IPD.s16DiffTheta; 
							} 
						 else if(Motor1.IPD.s16DiffTheta <(-32768/100))//if yes,it means the rotor direction is in reverse status
						 {
							 Motor1.IPD.s16DiffTheta = -Motor1.IPD.s16DiffTheta;
							 Motor1.IPD.s16EndTheta = Motor1.IPD.s16EndTheta + (32768/2);	//it is in reverse status, so add 180 degree
               Motor1.IPD.u8RIPDDetectedDIRReverseFlag =1;//detected the rotor direction is in reverse status 
							 
							 #ifdef ENABLE_NEW_STARTUP_METHOD
								Motor1.IPD.u8RIPDQualifyFailFlag = 1;//20190626//if direction is inverse, give up this IPD1 once
               #endif							 
							} 
						 else if(Motor1.IPD.s16DiffTheta >(32768/100))//if yes,it means the rotor direction is in forward status
						 {
							 Motor1.IPD.s16DiffTheta = Motor1.IPD.s16DiffTheta;	 
						  } 
						 else {Motor1.IPD.u8RIPDQualifyFailFlag = 1;}
						 
						 if(Motor1.IPD.u8MotorDIRCommand ==1)
						 { Motor1.IPD.s16EndTheta = Motor1.IPD.s16EndTheta + (32768/36);}// 10 degree compensation for the BEMF Theta
						 else
						 { Motor1.IPD.s16EndTheta = Motor1.IPD.s16EndTheta - (32768/3);}// -120 degree compensation for the BEMF Theta
						 
						 if(Motor1.IPD.s16EndTheta > 32767){Motor1.IPD.s16EndTheta = Motor1.IPD.s16EndTheta - 32767;}
						 else if(Motor1.IPD.s16EndTheta < 0){Motor1.IPD.s16EndTheta = Motor1.IPD.s16EndTheta + 32767;}
						 
						 if(Motor1.IPD.u8RIPDDetectedDIRReverseFlag ==1)//if yes, means detected the rotor direction is in reverse status
						 {Motor1.IPD.s16Speed = 0;}
						 else
             {Motor1.IPD.s16Speed = ((int32_t)((40*60*2)/POLE_NUMBER)*Motor1.IPD.s16DiffTheta)>>15;}
					}
			 }
			 
			 if(Motor1.IPD.u16DecTimer_1ms >= 30)
			 { Motor1.IPD.s16StartTheta = Motor1.IPD.s16Theta;}// Range from 0~32767
			
			  
			 Motor1.IPD.s16LastTheta = Motor1.IPD.s16Theta; //keep the last 1ms Theta for IPD process 			                        
}
void Read_IPD_BEMF_AB_Voltage(ROTOR_IPD_Struct* IPD)
{
     if(IPD->u8MotorDIRCommand==1)//if yes, means motor direction command is CW
			{	
				IPD->s16BEMFPhaseA12bit =(int16_t)u16ADC1[BEMFB_ADC_CHANNEL] -(IPD->u16BEMFPhaseBZero12bit);
				IPD->s16BEMFPhaseB12bit =(int16_t)u16ADC1[BEMFA_ADC_CHANNEL] -(IPD->u16BEMFPhaseAZero12bit);
			}
		 else//if yes, means motor direction command is CCW
			{		
				IPD->s16BEMFPhaseA12bit =(int16_t)u16ADC1[BEMFA_ADC_CHANNEL] -(IPD->u16BEMFPhaseAZero12bit);
				IPD->s16BEMFPhaseB12bit =(int16_t)u16ADC1[BEMFB_ADC_CHANNEL] -(IPD->u16BEMFPhaseBZero12bit);
			}
}

void Generate_Rotor_Small_Swing_Sin_Table_Address(ROTOR_IPD_Struct* IPD)
{	
		if(IPD->u16IncTimer_1ms >= (IPD->u16IncAddressTimer_1ms))
		{
			IPD->u16IncTimer_1ms =0;
			IPD->u16TableAddress +=16;
		}
		IPD->u16IncTimer_1ms++;
}

uint16_t Generate_Rotor_BEMF_AB_Absolute_Value(int32_t s32BEMF_A,int32_t s32BEMF_B)
{		
	uint32_t u32TempX, u32TempY;
	
	if(s32BEMF_A <0){u32TempX = -s32BEMF_A;}//generate absolute value of bemf A
	else{u32TempX = s32BEMF_A;}//generate absolute value of bemf A
	
	if(s32BEMF_B <0){u32TempY = -s32BEMF_B;}//generate absolute value of bemf B
	else{u32TempY = s32BEMF_B;}//generate absolute value of bemf B
	
	return (u32TempX + u32TempY) ;		
}

void Open_Loop_Acceleration_Modulate_For_IPD1_Fail(void)
{
		if(Motor1.info.u8ErrorCounter > 4)
		{
			Motor1.spec.u16SpeedSlope1 = OPEN_LOOP_RAMP_UP_SPEED_SLOP1/8;//open loop speed up slop1 setup//20180913 
			Motor1.spec.u16SpeedSlope2 = OPEN_LOOP_RAMP_UP_SPEED_SLOP2/4;//open loop speed up slop2 setup//20180913 
			Motor1.spec.u16SpeedSlope3 = OPEN_LOOP_RAMP_UP_SPEED_SLOP3/4;//open loop speed up slop3 setup//20180913
    }
    else if (Motor1.info.u8ErrorCounter > 2)
    {
			Motor1.spec.u16SpeedSlope1 = OPEN_LOOP_RAMP_UP_SPEED_SLOP1/4;//open loop speed up slop1 setup//20180913 
			Motor1.spec.u16SpeedSlope2 = OPEN_LOOP_RAMP_UP_SPEED_SLOP2/2;//open loop speed up slop2 setup//20180913 
			Motor1.spec.u16SpeedSlope3 = OPEN_LOOP_RAMP_UP_SPEED_SLOP3/2;//open loop speed up slop3 setup//20180913
    }
}
#endif

#ifdef ENABLE_ROTOR_IPD2_FUNCTION 
void Rotor_IPD2_Mode_Process(void)//execute this program once in every pwm cycle when in INDUCTANCE_SAT_POSITION_DET_MODE 
{
		Motor1.FOC.s16Vq =0;//setup Vq =0 when in INDUCTANCE_SAT_POSITION_DET_MODE
	
		if( Motor1.IPD.u16RIPD2PWMCounter > 0)//if Motor1.IPD.u16RIPD2PWMCounter ==0, jumper to next process
		  { Motor1.IPD.u16RIPD2PWMCounter -=1;}
		else
			{
					if(Motor1.IPD.u8RIPD2InjectFlag==0)
					{ 
						Motor1.FOC.s16Vd = -INJECT_VOLTAGE_PULSE_Vd;//setup the Vd value for Vd injection
						if(Motor1.IPD.u16RIPD2AddressCounter > 11){Motor1.IPD.u16RIPD2AddressCounter =11;Motor1.FOC.s16Vd =0;}//address index can not over 11
					
						Motor1.IPD.s16RIPD2Angle = u16IPD2Table[Motor1.IPD.u16RIPD2AddressCounter];//read IPD2 Vd injection angle
						Motor1.IPD.u16RIPD2AddressCounter +=1;//address index increase 1 for next Vd injection anlge
						
						Motor1.IPD.u8RIPD2InjectFlag=1;//for inject Vd voltage to motor when this flag == 1;
						Motor1.IPD.u16RIPD2PWMCounter = PWM_NUM_FOR_INJECT_VOLT;//setup inject voltage time duration.
					}
					else
					{
						Motor1.IPD.u8RIPD2InjectFlag=0;//for off voltage inject to motor
						Motor1.IPD.u16RIPD2PWMCounter = PWM_NUM_FOR_NON_INJECT_VOLT;//setup non-inject voltage time duration						
					}				
			}
		
		if(Motor1.IPD.u8RIPD2InjectFlag==0){Motor1.FOC.s16Vd =0;Motor1.IPD.s32RIPD2IdSum=0;} 
		else
			{
		    Motor1.FOC.s16Vd = -INJECT_VOLTAGE_PULSE_Vd;//setup the inject voltage
			  Motor1.IPD.s32RIPD2IdSum = (-Motor1.FOC.s16Id)+(-Motor1.FOC.s16Iq)+ Motor1.IPD.s32RIPD2IdSum;
				//Motor1.IPD.s32RIPD2IdSum = (-Motor1.FOC.s16Id)+ Motor1.IPD.s32RIPD2IdSum;
				if(Motor1.IPD.s32RIPD2IdSum > Motor1.IPD.s32RIPD2IdSumMax)
					{	
						Motor1.IPD.s32RIPD2IdSumMax = Motor1.IPD.s32RIPD2IdSum;
						Motor1.IPD.u16RIPD2MaxIdAngle = Motor1.IPD.s16RIPD2Angle;
					}					
			}
			
		if(Motor1.IPD.u16RIPD2PWMCounterTotal >0){Motor1.IPD.u16RIPD2PWMCounterTotal = Motor1.IPD.u16RIPD2PWMCounterTotal-1;}//20190626	
}
#endif

