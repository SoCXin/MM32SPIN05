
#ifndef __ROTOR_IPD_H
#define __ROTOR_IPD_H

#include "sys.h"

typedef struct 
{
	__IO int16_t  u8MotorDIRCommand;	
	__IO int16_t  s16Speed;           
	__IO int16_t  s16Theta;// Range from 0~32767                 
	__IO int16_t  s16Angle;// Range from 0~1023
	__IO int16_t  s16RotorSwingAngle;// Range from 0~1023
	__IO uint16_t u16BEMFPhaseAZero12bit;// Range from 0~4095
	__IO uint16_t u16BEMFPhaseBZero12bit;// Range from 0~4095
	__IO int16_t  s16BEMFPhaseA12bit;// Range from 0~4095
	__IO int16_t  s16BEMFPhaseB12bit;// Range from 0~4095
	__IO int16_t  u16BEMFABSUM12bit;// Range from 0~4095*2
	__IO int16_t  s16StartTheta;// Range from 0~32767
	__IO int16_t  s16EndTheta;  // Range from 0~32767
	__IO int16_t  s16LastTheta;  // Range from 0~32767
	__IO int16_t  s16DiffTheta; // Range from 0~32767
	__IO int16_t  s16DiffTheta_1ms;// Range from 0~32767
	__IO uint16_t u16DecTimer_1ms;
	__IO uint16_t u16IncTimer_1ms;
	__IO uint16_t u16TableAddress;
	__IO uint16_t u16IncAddressTimer_1ms;
	__IO uint8_t  u8RIPDQualifyFailFlag;
	__IO uint8_t  u8RIPDDetectedDIRReverseFlag;
	__IO uint16_t u16RIPDQualifyFailCounter;
	__IO uint16_t u16RIPD2PWMCounterTotal;//for IPD2 function//20190626
	__IO uint16_t u16RIPD2PWMCounter;//for IPD2 function
	__IO int32_t  s32RIPD2IdSum;//for IPD2 function
	__IO int32_t  s32RIPD2IdSumMax;//for IPD2 function
	__IO int16_t  s16RIPD2Angle;//for IPD2 function, Range from 0~1023
	__IO uint16_t u16RIPD2AddressCounter;//IPD2 function
	__IO uint8_t  u8RIPD2InjectFlag;
	__IO uint16_t u16RIPD2MaxIdAngle;//IPD2 result angle 0~1023
	
} ROTOR_IPD_Struct;

void Rotor_IPD_Mode_Process(void);
void Generate_Rotor_Small_Swing_Sin_Table_Address(ROTOR_IPD_Struct* IPD);
void Read_IPD_BEMF_AB_Voltage(ROTOR_IPD_Struct* IPD);
void Open_Loop_Acceleration_Modulate_For_IPD1_Fail(void);
void Rotor_IPD2_Mode_Process(void);
#endif
