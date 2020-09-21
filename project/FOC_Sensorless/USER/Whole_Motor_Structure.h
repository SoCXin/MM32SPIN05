#ifndef __WHOLE_MOTOR_STRUCTURE_H
#define __WHOLE_MOTOR_STRUCTURE_H 	

#include "sys.h"
#include "Initial_BEMF_Speed_Detect.h"
#include "pi.h"
#include "CurrentSensing.h"
#include "FOC_PMSM.h"
#include "SMO.h"
#include "SVPWM.h"
#include "ROTOR_IPD.h"



 
typedef struct  // motor specification
{
	uint16_t	u16FreqPwm;			//PWM frequency	
	uint16_t  u16PIOutLimit;
	uint16_t  u16FreqSpeedLoop;//Now is define to 1000Hz (it means 1ms calculate motor speed once ---SMO_Speed_Estimator();)
	uint16_t	u16SpeedSlope;	//unit:RPM, speed acceleration curve
  uint16_t	u16SpeedSlope1;	//unit:RPM, speed acceleration curve1
  uint16_t	u16SpeedSlope2;	//unit:RPM, speed acceleration curve2
  uint16_t	u16SpeedSlope3;	//unit:RPM, speed acceleration curve3
	uint16_t  u16OpenLoopTargetSpeed;//unit:RPM, Open loop target speed define
	int16_t 	s16CurrentGain;		//s16CurrentGain => 1A's Iq command value
	int16_t 	s16CurrentLimit;	//unit: 0.1A, for PI controller 
	uint8_t	  u8MotorPole;      //motor poles number	
	uint16_t  u16OpenLoopAlignTime_10ms;//unit:10ms, Open loop alignment time define //20190623
}Spec_Struct;

typedef struct  // command
{
	__IO uint8_t   u8UserDirectionCommand;//if 1 means user want Motor run at CW direction, 0 at CCW
	__IO int16_t	 s16IdCommand;      //Id command
	__IO int16_t	 s16IqCommand;      //Iq command    
	__IO uint16_t	 u16SpeedTarget;    //it is the target speed command in open & close loop;  
	__IO uint16_t	 u16SpeedCommandNow;//it is now's speed command to motor controller
	__IO uint16_t	 u16VSPSpeedCommand;//it is speed command from VSP interface, will transfer it to u16SpeedTarget in close loop
	__IO uint8_t	 u8MotorStart;	    //motor start flag
	__IO uint8_t   u8MotorRunMode;    //motor running mode
	__IO uint8_t   u8MotorReverseCommand;//if 1 means motor need to run at the reverse direction (for against wind startup motor)
	__IO uint16_t  u16CommandTheta; //motor command theta (0~32767)
	__IO uint16_t  u16Angle;        //motor angle (0~1023)
	__IO int16_t   s16AngleDiff;    //motor angle difference between SMO esti and open loop 
	__IO int16_t   s16OpenLoopTargetIq;//motor open loop maximum Iq command 
  __IO int16_t   s16OpenLoopInitIq;//motor open loop initial Iq command
  __IO int16_t   s16OpenLoopNowIq;//motor open loop now Iq command 
  __IO int16_t   u16IqSlopeCnt;	   //motor open loop conuter for Iq command acceleration
	__IO int16_t   u16SpeedSlopCnt;  //motor open loop counter for speed command acceleration
	__IO uint8_t   u8MotorDirection; //motor direction command
	__IO uint8_t   u8PoleNumber;     //motor pole number
	__IO uint8_t   u8OverPowerLimitFlag; //if 1, means over power limitation, it needs speed down the motor//20190606
}Command_Struct;

typedef struct  // feedback info
{
	__IO int16_t	s16IdError;
	__IO int16_t	s16IqError;
	__IO int16_t	s16SpeedError;
	__IO uint8_t  u8ErrorCode;
	__IO uint8_t  u8ErrorCounter;
	__IO uint16_t	u16VSPInput;
	__IO uint16_t	u16VSPAverage;
	__IO int16_t	s16IsumInput12bit;
	__IO uint16_t	u32IsumCurrentAverage;//unit:0.1mA
	__IO uint32_t	u32IsumCurrent;//unit:0.1mA
	__IO uint32_t	u32PowerConsumption;//unit:0.1W
	__IO uint16_t	u16VBusInput12bit;//data read from 12bit ADC
	__IO uint16_t	u16VBusVoltage;//unit:0.01V
	__IO uint16_t	u16VBusVoltageAverage;//unit:0.01V
	__IO uint16_t	u16RatioSpeedError;//for power limitation control
	__IO uint8_t  u8MiscellaneousADCConverFlag;//if 1, means ADC need convert miscellaneous channel,like VSP, DC BUS Voltage 
	__IO uint32_t	u32IUSumInAlign;//IU absolute value Sum for check rotor alignment is ready or not//20190623
	__IO uint16_t u32IUSumCounterInAlign;//for IU Sum calculation counter //20190623
}Info_Struct;

	 
// whole motor structure
typedef struct 
{
	Spec_Struct           spec;
	Info_Struct			      info;
	Command_Struct		    command;
	SMO_Struct			      smo;
	FOC_Struct            FOC;
	BEMF_Speed_Struct			BEMF;
	ROTOR_IPD_Struct      IPD;
  New_Startup_Struct    NewStartup;
}Whole_Motor_Struct; 


// globel struct 
extern Whole_Motor_Struct Motor1;
extern PI_Struct  Iq_PI, Id_PI, speed_PI;
extern CurrentSensing_Struct CurrentSensing;
extern __IO int16_t s16CosQ15,s16SinQ15;//lookup table

#endif





























