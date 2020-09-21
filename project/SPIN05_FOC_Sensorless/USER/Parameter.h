#ifndef __PARAMETER_H
#define __PARAMETER_H 			   

#include "whole_motor_structure.h"	
#include "Whole_Motor_Parameters.h"
extern PI_Struct  Iq_PI, Id_PI, speed_PI;

__IO int16_t s16CosQ15=0,s16SinQ15=0;
	 
 Whole_Motor_Struct Motor1={

// spec
{
	PWM_FREQUENCY,	//INT16 Freq_pwm;				    //pwm frequence(Hz)
  PI_OUT_LIMIT,   //INT16 u16PI_Out_Max_Value  
  0,              //INT16 u16FreqSpeedLoop;	 //speed loop frequence(Hz), the application speed can not over (u16FreqSpeedLoop/2) Hz			
	0,              //INT16 speed_slope;		 //(x rpm -> per sec)
	0,              //INT16 speed_slope1;		 //(x rpm -> per sec)
	0,              //INT16 speed_slope2;		 //(x rpm -> per sec)
	0,              //INT16 speed_slope3;		 //(x rpm -> per sec)
	0,              //INT16 open loop target speed;	//(x rpm)
	CURRENT_GAIN,	  //INT16	s16CurrentGain;		 //s16CurrentGain => 1A,  600mV/A, (600mV/5000mV)*65536 = 7865dig/1A 
	CURRENT_LIMIT,  //INT16	s16CurrentLimit;	 //unit: 0.1A, (Peak value) for PI controller ,max Iq (150%,rms) 
	POLE_NUMBER,		//INT8	motor_pole;	
},
// info	
{0},

// command
{0}, 	 

//smo
{
  
	0,                   //INT16	theta;	
	0,                   //INT16	angle;
	0	,                  //INT16	speed;	
	SMO_Kslide,          //INT16	Kslid;
	SMO_Kslf_MIN_VALUE,  //INT16	Kslf;
	F_SMO_VALUE,         //INT16	KF;
	G_SMO_VALUE,         //INT16	KG;
	0,                   //INT32	s16SpeedEstGainQ15;	
},

//FOC
{0},

//SVPWM
//{0},

//BEMF
{0},

};
					 
PI_Struct  Iq_PI = {0,0, 0, 0, 0, 0},//current loop for Vq
           Id_PI = {0,0, 0, 0, 0, 0},//current loop for Vd
           speed_PI =  {0,0, 0, 0, 0, 0};//speed   loop for Iq	
					 
CurrentSensing_Struct CurrentSensing = {0, 0, 64, 0, 0, 0, 0, 0, 0};

#endif





























