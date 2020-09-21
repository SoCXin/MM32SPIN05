#ifndef __WHOLE_MOTOR_PARAMETERS_H_
#define __WHOLE_MOTOR_PARAMETERS_H_
#include "HAL_device.h" 
#include "HAL_conf.h"

//#define ENABLE_DAC_SHOW  //for enable DAC output

//------MCU parameters------------------------------------------------------------------------------------------------------
#define MM32SPIN05
//#define MM32SPIN06

//------System parameters---------------------------------------------------------------------------------------------------
#ifdef MM32SPIN05
#define SYSYTEM_CLOCK    		72000000        //unit:Hz
#endif

#ifdef MM32SPIN06
#define SYSYTEM_CLOCK    		48000000        //unit:Hz
#endif

#define PWM_FREQUENCY    		22000           //unit:HZ
#define POLE_NUMBER      		14//8
#define MOTOR_DIR_CW     		1               //1:CW, 0:CCW rotation direction define before power on
#define TARGET_MAX_SPEED    TARGET_SPEED_7  //unit:RPM, max. speed limitation
#define TARGET_MIN_SPEED    TARGET_SPEED_1  //unit:RPM, min. speed limitation

//----Enable/Disable 1 shunt R mode----------------------------------------------------------------------------------------
//#define ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT //if not define, it will use 2 shunt R to measure 3 phase current

//------2 shunt R phase current sensing parameter-------------------------------------------------------------------------------------
#define MAX_PWM_DUTY_PERCENTAGE_2SHUNT_R    95 //unit:%, it need to limit the maximum pwm duty output for doing the 2 shunt R current sensing

//------1 shunt R phase current sensing parameter-------------------------------------------------------------------------------------
#define ADC_DELAY_TIME_1_SHUNT_R   25//20//35 //unit : 0.1us (dead time + OP output signal stable time)
#define ADC_TOTAL_TIME_1_SHUNT_R   33//30//45 //unit : 0.1us (dead time + OP output signal stable time + ADC sample time)

//------Rotor initial position detection method 1 parameters-------------------------------------------------------------------------------------
//#define ENABLE_ROTOR_IPD1_FUNCTION          //if define, it will use IPD1 function to detect the initial position of rotor before startup motor
#define ROTOR_IPD1_TARGET_CURRENT       10  //unit:0.1A define the target torque current in ROTOR_SMALL_SWING_MODE
#define ROTOR_IPD1_SMALL_SWING_TIME     100 //unit:ms, define rotor small swing time with ROTOR_IPD1_TARGET_CURRENT torque current
#define ROTOR_IPD1_DETECT_TIME          60  //unit:ms, define rotor angle detection time after rotor small swing, range 40~100ms
#define ROTOR_IPD1_BEMF_SUM_MINI        50  //unit:mV, define the minimum sum value of bemf |A| + |B| absolute value
#define ROTOR_IPD1_MAX_FAIL_TIMES       6   //range:(6~65535),if IPD failure times over this setup value, motor operation will enter open loop mode directly.

//------Rotor initial position detection method 2 parameters---inductance saturation---------------------------------------------------------------
//#define ENABLE_ROTOR_IPD2_FUNCTION          //if define, it will use IPD2 function to detect the initial position of rotor before startup motor
#define PWM_NUM_FOR_INJECT_VOLT         8     //define PWM numbers as IPD2's inject voltage pulse time, for motor inductance saturation effect //20190626
#define PWM_NUM_FOR_NON_INJECT_VOLT     24    //define PWM numbers as IPD2's off current time after inject voltage pulse to motor//20190626
#define INJECT_VOLT_PULSE_AMPLITUDE     5     //unit:%,  define motor inject voltage pulse amplitude, range (1~40)

//----define enable the new startup method and others parameters--------------------------------------------------------------------------------------------------------- 
#define ENABLE_NEW_STARTUP_METHOD
#define NEW_STARTUP_MIN_TIME        	        			30 //unit:10ms, define new startup mode minimum time, 20 means 200ms as new startup time//20190626
#define NEW_STARTUP_MAX_TIME                        50 //unit:100ms,if over this time in new startup mode, it will show error to system,600 means 60second//20190626
#define NEW_STARTUP_TO_CLOSE_LOOP_SPEED   					600//TARGET_SPEED_1 //unit:RPM, define enter close loop mode speed
#define NEW_STARTUP_RAMP_UP_TIME                    48//90 //range(80~3200),16 = 1ms at 16KHz PWM, 1440 pwm cycles of 16khz = 1440/16 =90ms

#define NEW_STARTUP_INITIAL_CURRENT     	        	8//7//20  //unit:0.1A define the initial torque current in new startup mode for standstill or downwind startup //20190626
#define NEW_STARTUP_TARGET_CURRENT                  8//7//20  //unit:0.1A define the target  torque current in new startup mode //20190626
#define NEW_STARTUP_CURRENT_FOR_AGAINST_WIND        12  //unit:0.1A define the initial torque current in new startup mode for against wind startup //20190626

#define NEW_STARTUP_RAMP_UP_CURRENT_SLOP            10  //unit:0.1A/sec,range(1~400),increase how many torque current per second in new startup mode//20190626

//#define ENABLE_BACK_TO_NEW_STARTUP_MODE_WHEN_LOW_SPEED //if enable, it will back to new startup mode if speed below the define "BACK_TO_NEW_STARTUP_MODE_SPEED"
#define BACK_TO_NEW_STARTUP_MODE_SPEED  						(TARGET_SPEED_1 -(TARGET_SPEED_1>>1)) //if speed under this in close loop mode, it will back to new startup mode
#define NEW_STARTUP_OVER_SPEED                      (TARGET_SPEED_7/2) //new startup over speed define

#ifdef ENABLE_NEW_STARTUP_METHOD
#define ENABLE_NEW_STARTUP_METHOD_FOR_DOWNWIND_STARTUP//20190626, enable motor startup in downwind status
#define ENABLE_NEW_STARTUP_METHOD_FOR_AGAINST_WIND_STARTUP_AFTER_STOPPED//20190626,restart motor use new startup method,for motor has stopped by open loop mode

#define ENABLE_NEW_STARTUP_METHOD_FOR_LOW_SPEED_AGAINST_WIND_STARTUP //20190626//restart motor use new startup method, if motor run in low speed reverse direction by against wind
//#define ENABLE_NEW_STARTUP_METHOD_FOR_HIGH_SPEED_AGAINST_WIND_STARTUP//20190626//restart motor use new startup method, if motor run in high speed reverse direction by against wind
#endif

//-----define Rotor Alignment parameters---------------------------------------------------------------------------------------------------------------- 
//-----user can bypass alignment mode if all alignment parameters "OPEN_LOOP_ALIGNMENT_TIME" fill with 0
//-----if enable the define "ENABLE_ROTOR_IPD1_FUNCTION", the rotor alignment mode will be useless----------------//20190626
//-----if enable the define "ENABLE_ROTOR_IPD2_FUNCTION", the rotor alignment mode will be useless----------------//20190626
//-----if enable the define "ENABLE_NEW_STARTUP_METHOD" only, the rotor alignment mode will be useless------------//20190626
//-----exception : if enable the define "ENABLE_NEW_STARTUP_METHOD" & "ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_MODE", the rotor alignment mode will be used //20190626

//#define ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_MODE //if enable this define & "ENABLE_NEW_STARTUP_METHOD", new startup method will be used after alignment mode//20190626
#define OPEN_LOOP_ALIGNMENT_TIME        	0//200     //unit:10ms, define open loop alignment time, 20 means 200ms as alignment time//20190623
#define OPEN_LOOP_ALIGNMENT_CURRENT     	0//2       //unit:0.1A define the current of rotor alignment//20190623


//-----define Open loop mode parameters----------------------------------------------------------------------------------------------------------------------
//-----if enable the define "ENABLE_NEW_STARTUP_METHOD" , the Open loop parameters will be useless----------------------//20190626 
//-----exception :if enable the define "ENABLE_NEW_STARTUP_METHOD" & "ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_AND_OPEN_LOOP_MODE", the align and open loop mode will be used //20190626

//#define ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_AND_OPEN_LOOP_MODE //if enable this define & "ENABLE_NEW_STARTUP_METHOD", new startup method will be used after align and open loop mode//20190626
#define OPEN_LOOP_SPEED_TO_ENTER_NEW_STARTUP_MODE   120                //unit:RPM, if open loop mode speed over this, it will enter new startup mode//20190626

//----------------------------------------------------------------------------------------------------------------------
#define OPEN_LOOP_TARGET_SPEED         1000//(TARGET_SPEED_7/6) //unit:RPM,define open loop target speed, then will enter close loop control
#define OPEN_LOOP_RAMP_UP_SPEED_SLOP1   400 //30  //unit:RPM/sec,range(1~1000),acceleration slop in open loop 1st stage
#define OPEN_LOOP_RAMP_UP_SPEED_SLOP2   1000 //60  //unit:RPM/sec,range(1~1000),acceleration slop in open loop 2nd stage
#define OPEN_LOOP_RAMP_UP_SPEED_SLOP3   1000 //90  //unit:RPM/sec,range(1~1000),acceleration slop in open loop 3th stage

#define OPEN_LOOP_RAMP_DOWN_SPEED_SLOP1  30 //unit:RPM/sec,range(1~1000),deacceleration slop in open loop 1st stage
#define OPEN_LOOP_RAMP_DOWN_SPEED_SLOP2  60 //unit:RPM/sec,range(1~1000),deacceleration slop in open loop 2nd stage
#define OPEN_LOOP_RAMP_DOWN_SPEED_SLOP3  90 //unit:RPM/sec,range(1~1000),deacceleration slop in open loop 3th stage


#define RAMP_UP_CHANGE_TO_SLOP2_SPEED    10 //unit:RPM,define the open loop threshold speed for ramp up slop1 change to slop2
#define RAMP_UP_CHANGE_TO_SLOP3_SPEED    300 //unit:RPM,define the open loop threshold speed for ramp up slop2 change to slop3


#define OPEN_LOOP_TARGET_CURRENT         15 //unit:0.1A define the target  torque current in open loop ramp up stage
#define OPEN_LOOP_INITIAL_CURRENT        15 //unit:0.1A,define the initial torque current in open loop ramp up stage 
#define OPEN_LOOP_RAMP_DOWN_CURRENT      15 //unit:0.1A,define the torque current in open loop ramp down stage

#define OPEN_LOOP_RAMP_UP_CURRENT_SLOP1   4 //unit:0.1A/sec,range(1~40),increase how many phase current per second in open loop 1st stage
#define OPEN_LOOP_RAMP_UP_CURRENT_SLOP2   4 //unit:0.1A/sec,range(1~40),increase how many phase current per second in open loop 2nd stage
#define OPEN_LOOP_RAMP_UP_CURRENT_SLOP3   10//unit:0.1A/sec,range(1~40),increase how many phase current per second in open loop 3th stage

//-----Close loop parameters------------------------------------------------------------------------------------------------
//-----Close loop parameters------------------------------------------------------------------------------------------------
#define TARGET_SPEED_1                2000 //unit: RPM, define the user's lowest  target speed
#define TARGET_SPEED_2                4000 //unit: RPM, define the user's second  target speed
#define TARGET_SPEED_3                6000 //unit: RPM, define the user's Third   target speed
#define TARGET_SPEED_4                8000 //unit: RPM, define the user's 4th     target speed
#define TARGET_SPEED_5                10000 //unit: RPM, define the user's 5th     target speed
#define TARGET_SPEED_6                12000 //unit: RPM, define the user's 6th     target speed
#define TARGET_SPEED_7                12000 //unit: RPM, define the user's highest target speed

#define CLOSE_LOOP_RAMP_UP_SPEED_SLOP                (TARGET_SPEED_7/3) //unit:RPM/sec,range(1~1000),acceleration slop in close loop
#define CLOSE_LOOP_RAMP_DOWN_SPEED_SLOP              (TARGET_SPEED_7/3) //unit:RPM/sec,range(1~1000),deceleration slop in close loop//20190626

#define CLOSE_LOOP_RAMP_DOWN_SPEED_ULTRA_LOW_SLOP        10  //unit:RPM/sec,range(1~1000),ultra low deceleration slop in close loop//20190626
#define CLOSE_LOOP_ULTRA_LOW_SPEED_DOWN_SPEED_THRESHOLD  200 //unit:RPM, if under this threshold speed, the speed down will use ultra low speed down slop//20190626

#define CLOSE_LOOP_SPEED_TO_STOP_PWM  TARGET_SPEED_1 //unit: RPM,(must > OPEN_LOOP_TARGET_SPEED),if receive motor stop command, pwm will off when speed under this define

//-----Motor Forward/against wind startup parameters------------------------------------------------------------------------
#define DIRECT_ENTER_CLOSE_LOOP_MIN_SPEED 1000//(TARGET_SPEED_7/5) //unit:RPM,define the mini. bemf speed which can directly enter close loop when motor startup
#define DOWNWIND_OPEN_LOOP_TARGET_SPEED   1000//(TARGET_SPEED_7/5) //unit:RPM,define the open loop into close loop's speed in low speed downwind startup status

//-----1,2 Shunt R  PI Control parameters-------------------------------------------------------------------------------------------------
#define SHUNT_R_VALUE                   50 //unit:milli ohm, 50 means 0.05 ohm

#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
  #define CURRENT_AMPLIFICATION_FACTOR  60 //unit:0.1 amplification factor, 60 means 60*0.1 = 6 amplification factor
#else
  #define CURRENT_AMPLIFICATION_FACTOR  50 //unit:0.1 amplification factor, 50 means 50*0.1 = 5 amplification factor
#endif

#define CURRENT_LIMIT    10    //unit: 0.1A, it will limit the maximum sine wave phase current in close loop operation
#define CURRENT_PROTECT  50    //unit: 0.1A, it will stop the motor if sine wave phase current over this value

#define PI_OUT_LIMIT     30000  //range(0~32767),Setup the Vd, Vq PI output limitation 

//-----I_SUM R parameters for totoal current measurement-------------------------------------------------------------------
#define ISUM_R_VALUE                       50 //unit:milli ohm, 40 means 0.04 ohm
#define ISUM_CURRENT_AMPLIFICATION_FACTOR  60 //unit:0.1 amplification factor, 50 means 50*0.1 = 5 amplification factor

//-----DC Bus voltage measurement------------------------------------------------------------------------------------------
#define VBUS_PULL_UP_R     1000 //unit : 0.1K Ohm, 1000 means 100K Ohm //20181215
#define VBUS_PULL_DOWN_R   100  //unit : 0.1K Ohm, 100  means 10K  Ohm //20181215

#define ENABLE_DC_BUS_VOLTAGE_PROTECTION //if enable, it will do DC BUS voltage check and protection
#define DC_BUS_OVER_VOLTAGE_LIMITATION   300//unint : 100mV, 300 means 30V, if DC Bus voltage over this, it will stop motor
#define DC_BUS_UNDER_VOLTAGE_LIMITATION  110//unint : 100mV, 125 means 12.5V, if DC Bus voltage unser this, it will stop motor
#define DC_BUS_OVER_VOLTAGE_HYSTERESIS    20////unint : 100mV, 20 means 2V as hysteresis voltage for over voltage happened//20190626
#define DC_BUS_UNDER_VOLTAGE_HYSTERESIS    5////unint : 100mV, 20 means 2V as hysteresis voltage for under voltage happened//20190626

//-----Motor SMO parameters -----------------------------------------------------------------------------------------------
#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
#define SMO_Kslf_MAX_VALUE      27000//20000//27000//22000//2400 //recommended value (300~6000), range (1~32767)
#define SMO_Kslf_MIN_VALUE      5200//3200//5200//1200 //recommended value (100~2000), range (1~32767)
#else
#define SMO_Kslf_MAX_VALUE      27000//27000//4200 //recommended value (300~6000), range (1~32767)
#define SMO_Kslf_MIN_VALUE      5200//5200//1600 //recommended value (100~2000), range (1~32767)
#endif

#define SMO_Kslide              26000 //recommended value 26000, range (1~32767)
#define G_SMO_VALUE             19000 //recommended value 19000, range (1~32767)//20190626
#define F_SMO_VALUE             30000 //recommended value 30000, range (1~32767)//20190626

//-----Motor FOC current PI parameters ------------------------------------------------------------------------------------
#define PI_CURRENT_Kp_VALUE       24000
#define PI_CURRENT_Ki_MAX_VALUE   12000
#define PI_CURRENT_Ki_MIN_VALUE   5600//1200//280//560

//-----define current PI integral result divider for open loop enter close loop//20190123---------------------------------
#define CURRENT_PI_INTEGRAL_RESULT_DIVIDER_WHEN_ENTER_CLOSE_LOOP  1//divider number, if define 32,it means 1/32 

//-----Motor speed PI parameters ------------------------------------------------------------------------------------------
#define PI_SPEED_Kp_VALUE   24000
#define PI_SPEED_Ki_VALUE   1500

//-----Motor Error code define---------------------------------------------------------------------------------------------
#define MAX_ERROR_ACCUMULATIVE_TOTAL    10 //if error counter over or equal this value, the motor will not restart again until receive off command
#define ERROR_RESTART_WAIT_TIME          5 //unit :100ms, after a error happened, waiting this time then can restart motor

#define MOTOR_SPEED_ERROR                1
#define MOTOR_PHASE_CURRENT_ERROR        2
#define MOTOR_OVER_CURRENT               3
#define MOTOR_ROTATION_INVERSE_ERROR     4
#define MOTOR_OVER_UNDER_VOLTAGE_ERROR   5
#define NEW_STARTUP_MODE_OVER_TIME_ERROR 6//20190626
#define MOTOR_DCBUS_OVER_CURRENT_ERROR   7//20190626
#define MOTOR_LACK_PHASE_ERROR           8//20190626
/********************************************************************************************************
** Advance parameters setup : common user do not change the below parameters            
********************************************************************************************************/
//----Enable/Disable TIM1_CC4 to trigger ADC for 2 shunt R current sensing-------------------------------------------------
#ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT 
  #ifdef MM32SPIN05
	  #define ENABLE_TIM1_CC4_TO_TRIG_ADC_FOR_2_SHUNTR //if define, it will use TIM1_CC4 to trigger ADC1. if not define, it will use TIM3_CC1 to trigger ADC1.
	#endif
#endif

#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT 
  #ifdef MM32SPIN05
	  #define ENABLE_TIM1_CC4_CC5_TO_TRIG_ADC1_FOR_1_SHUNTR //if define, it will use TIM1_CC4,CC5 to trigger ADC1. if not define, it will use TIM3_CC1 to trigger ADC1.
	#endif
#endif

#ifdef MM32SPIN05
 #define ENABLE_HARDWARE_DIVIDER    //if define, use internal hardware divider. if not define, it will use software divider
#endif

//----define OVER CURRENT PROTECT--------------------------------------------------------------------
#define ENABLE_COMP1_AND_iVREF_INTER_CONNECT_TIM1BKIN_DO_OVER_CURRENT_PROTECT //if use internal COMPARATOR and internal Vref 1.2V with TIM1BKIN brake PWM,user can enable this define
//#define ENABLE_TIM1BKIN_INPUT_PIN_ONLY_DO_OVER_CURRENT_PROTECT              //if use external COMPARATOR, user can enable this define (input to PB12 pin)

//----define the internal comparator inverting input (INM) 1,3/4,1/2,1/4Vref voltage ------------------------------------------------------------------------------------------------
#define INTERNAL_COMPARATOR_INM_REF_VOLTAGE   COMP_InvertingInput_VREFINT//COMP_InvertingInput_1_4VREFINT//COMP_InvertingInput_1_2VREFINT//COMP_InvertingInput_3_4VREFINT

//----define the internal comparator inverting input (INM) 1/20,2/20,1/2,...16/20 VDDA(5V) voltage ------------------------------------------------------------------------------------------------
//(3_24) real is (3-2)/20 of 5V = 0.25V(MIN),...(6_24) real is (6-2)/20 of 5V = 1V.....(18_24) real is (18-2)/20 of 5V = 4V(MAX)
#define INTERNAL_COMPARATOR_INM_REF_CRV_VOLTAGE   COMP_CRV_Level_6_24//COMP_CRV_Level_3_24(0.25V),COMP_CRV_Level_4_24(0.5V).......COMP_CRV_Level_18_24

//----define TIM1BKIN input High or Low to make PWM OUTPUT OFF---------------------------------------------------------------------------------------------------------------------
#define BREAK_POLARITY   TIM_BreakPolarity_High //TIM_BreakPolarity_Low//if define High, TIM1BKIN receive high signal will let Motor PWM OFF 


//----define ISUM current measurement enable or not--------------------------------------------------------------------------------
//#define ENABLE_ISUM_MEASUREMENT   //enable this to measure total current and power consumption

//----define Maximum Power Consumption Control--------------------------------------------------------------------------------
#ifdef  ENABLE_ISUM_MEASUREMENT
	//#define ENABLE_MAX_POWER_LIMIT      //for max. power consumption limitation control enable or not
	#define ENABLE_MAX_CURRENT_LIMIT      //for max. DC BUS current limitation control enable or not
#endif

#define USE_MEASURED_DC_BUS_VOLTAGE_TO_GET_POWER //if enable,it will use measured dc bus voltage to calculate power consumption. if not, it will use fixed DC_BUS_VOLTAGE parameter
#define DC_BUS_VOLTAGE                      240     //unit: 0.1V, define the dc bus voltage, 100 means 10V. it will be used if not define "USE_MEASURED_DC_BUS_VOLTAGE_TO_GET_POWER"

//----for MAX DCBUS CURRENT LIMIT //20190626--------------------------------------------------------------------------------------------------------------
#define MAX_DCBUS_CURRENT_PROTECTION        2500 //unit:1mA, define the dc bus max current protection,2000 means 2.0A, if over it, motor will be stopped and show error
#define MAX_DCBUS_CURRENT_LIMIT             1500 //unit:1mA, define the dc bus max current limitation,1500 means 1.5A //20190626
#define MIN_CURRENT_LIMIT_RESOLUTION        40   //10 means 10mA as minimum current limitation resolution,if over max.current 20mA,it means 20mA/10mA =2RPM Speed CMD Decrease every 50ms//20190606

#define SPEED_CMD_DEC_RPM_FOR_POWER_LIMIT  1  //5 means max. 5 rpm decrease every 50ms for speed command, when power over limitation //20190605
#define CALIBRATION_OFFSET_VOLTAGE        (2) //2 means 2*1.25 =2.5mV Offset for calibration OP Amplifier output//20190606
#define UNDO_POWER_LIMIT_TIME_IF_TARGET_SPEED_CHANGED  4  //if 4, mean 4*50ms = 200ms, if target speed command changed, it will not do power limitation for 200ms//20190606

//----for MAX POWER LIMIT-----------------------------------------------------------------------------------------------------------------------
//#define MAX_POWER_LIMIT                     400 //unit: 0.1W, define the max power limitation, 240 means it will limit power consumption in 24W
//#define MIN_POWER_LIMIT_RESOLUTION          10  //10 means 1.0W as minimum power limitation resolution,if over max.power 2W,it means 2W/1W =2RPM Speed CMD Decrease every 50ms//20190606

//#define SPEED_CMD_DEC_RPM_FOR_POWER_LIMIT  5  //5 means max. 5 rpm decrease every 50ms for speed command, when power over limitation //20190605
//#define CALIBRATION_OFFSET_VOLTAGE        (2) //2 means 2*1.25 =2.5mV Offset for calibration OP Amplifier output//20190606
//#define UNDO_POWER_LIMIT_TIME_IF_TARGET_SPEED_CHANGED  4  //if 4, mean 4*50ms = 200ms, if target speed command changed, it will not do power limitation for 200ms//20190606

//----define the against wind startup motor parameters-------------------------------------------------------------------------
//for restart the motor when motor is in againstwind status
#define AGAINST_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA  2  //define open loop speed ramp up slop divider para., 2 means 1/2  normal ramp up speed acceleration//20180807
#define AGAINST_WIND_CLOSE_TO_OPEN_LOOP_SPEED          ((OPEN_LOOP_TARGET_SPEED)+50) //(need > OPEN_LOOP_TARGET_SPEED), define the close to open loop speed in reverse and high speed status//20180807
#define AGAINST_WIND_OPEN_TO_CLOSE_TARGET_SPEED        (OPEN_LOOP_TARGET_SPEED)//(need < OPEN_LOOP_TARGET_SPEED),define the open to close speed after motor is stopped in against wind//20180807
#define K_AGAINST_WIND_OPL                             2  //suggest range 1~4, modulate the open loop initial Iq integral//20180807

//----define the initial parameters when the motor is stopped by reverse action in against wind -----------------------------
#define RESTARTUP_INITIAL_Iq_DIVIDE_PARA               2 //4 means 1/4 Motor1.command.s16OpenLoopInitIq value for initializing the Iq_PI.s32IntegralResult//20190408
#define PHASE_LEAD_ANGLE_IN_AGAINST_WIND_RESTART       0//45 means 45 degree of 360 degree for phase lead (32768/360 = 91)//20190408

//----define the down wind startup motor parameters-------------------------------------------------------------------------
//for restart the motor at down wind low speed status  
#define DOWN_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA     2  //open loop speed up slop setup, 2 means 1/2  normal ramp up speed acceleration//20180807


//----define phase current digital filter parameter for 1 shunt R phase current only--------------------------------------------------------------------------------
#define PHASE_CURRENT_AVERAGE_WEIGHT        2   //Option(1,2,4),only for 1 shunt R.if 2, it means this time new phase current measurement result is 1/2 weight.
#define FORCE_TO_FILTER4_MAX_RPM_1_SHUNT_R  600 //unit: RPM, new measurement phase current will weight to 1/4, if speed below this speed in 1 shunt R 


//----define the default zero voltage of BEMF A, BEMF B when WL MOSFET is ON-------------------------------------------------------
#define DEFAULT_ZERO_BEMF_VOLTAGE  5 //unit:0.1V,define the default zero voltage of BEMF A, BEMF B when WL MOSFET is on

//----enable /disable the angle compensation in the beginning of close loop-------------------------------------------------
#define ENABLE_OPEN_TO_CLOSE_ADVANCE_GRADUALLY //if define,the motor driving angle is gradually from the open loop angle to SMO angle at the beginning of close loop 

//when motor driving angle is gradual from the open loop angle to SMO angle, it define how many pwm cycles close to 0.35 degree 
//difference angle between open loop and SMO angle
#define PWM_CYCLES_CLOSE_TO_035_DEGREE  12//if define 12, it means every 12 pwm cycles close to 0.35 degree until the difference angle is 0

//----define Fix Iq command action in the beginning of close loop-------------------------------------------------------------
#define FIX_Iq_COMMAND_TIME         5 //unit :100ms, it define a the fix Iq command time when first time into the close loop
#define GRADUAL_Iq_COMMAND_TIME     3 //unit :100ms, after FIX_Iq_COMMAND_TIME, it need a gradual time for from FIX_Iq_COMMAND to close loop Iq command
#define FIX_Iq_COMMAND            (OPEN_LOOP_RAMP_UP_TARGET_Iq/4)//range (0~32767), it will fix this Iq command during a FIX_Iq_COMMAND_TIME time

//----define the minimum Iq command, normally the Iq command is negative number-----------------------------------------------
#define LIMIT_MINIMUM_Iq_COMMAND   500//range (0~1000), it limit the minimum Iq command

//----define the Iq command---------------------------------------------------------------------------------------------------
#define Id_COMMAND_VALUE            1000 //range (-2000~+2000)

//----define the maximum difference voltage between BEMF A,B, if under this threshold voltage, the motor is in standstill status before startup--- 
#ifdef ENABLE_ROTOR_IPD1_FUNCTION
#define BEMF_COMPARATOR_HYSTERESIS         100   //unit: mV,  it define the digital comparator's hysteresis voltage
#define BEMF_STANDSTILL_THRESHOLD_VOLTAGE  200  //unit: mV,  it define the standstill if BEMFA,B difference voltage under this limitation 
#else
#define BEMF_COMPARATOR_HYSTERESIS         30   //unit: mV,  it define the digital comparator's hysteresis voltage
#define BEMF_STANDSTILL_THRESHOLD_VOLTAGE  200   //unit: mV,  it define the standstill if BEMFA,B difference voltage under this limitation 
#endif

//----define the Dead time--------------------------------------------------------------------------------------------------------
#ifdef MM32SPIN05
#define DEAD_TIME_SETUP            48 //unit : 13.88ns (at system clock 72MHz),range(0~127)
#endif

#ifdef MM32SPIN06
#define DEAD_TIME_SETUP            48 //unit : 20.83ns (at system clock 48MHz),range(0~127)
#endif

//----define the K parameter for the initial Vq when motor startup in downwind status--------------------------------------------- 
#define K_STARTUP_Vq_INITIAL_PARA  256 //range(1~512),the initial Vq integrator parameter,for motor startup direct into close loop status 

//----Enable/disable force motor always in open loop---------------------------------------------------------------------------------------
//#define FORCE_STAY_IN_OPEN_LOOP       //if enable it, it will always keep in open loop mode


//----define the PVD(programmable voltage detector) voltage, softeware will reset MCU if VDD under this define voltage---------------------
#define PVD_VOLTAGE     PWR_PVDLevel_3V6  //3V6 means 3.6V, (Option : 2V7,3V0,3V6,3V9,4V2...)

//----define enable/disable MOSFET WL pin turn on at CHECK_BEMF_MODE-----------------------------------------------------------------------
//#define  ENABLE_WL_ON_AT_CHECK_BEMF_MODE    //if define, it means enable WL turn on at CHECK BEMF MODE before motor startup and IPD
#define  ENABLE_MINI_WL_ON_WIDTH_AT_CHECK_BEMF_MODE //if define, it means the WL on width will be minimize for reducing the vibration and sound noise
#define  WL_ON_TIME_SETUP_AT_CHECK_BEMF_MODE  250 //unit: us, range(200~500), setup WL on time for every 1ms at CHECK_BEMF_MODE, BEMF U,V will be measured during this time and then off WL

//----if normally stop by PWM OFF, define a minimum time delay then can restart motor-----------------------------------------------------------------------
#define  RESTART_WAIT_TIME_AFTER_PWM_OFF  0//unit: 0.1second, 50 measns wait 5 second after PWM off, then can restart motor

//-----Motor Maximum Speed Calculation parameters setup-----------------------------------------------------------------------------------------------------
#define ENABLE_MAX_120000RPM_SPEED_CALCULATION    //(2 poles)
//#define ENABLE_MAX_24000RPM_SPEED_CALCULATION   //(2 poles)

//-----2 PWM cycles do once FOC,SMO and others calculation Option------------------------------------------------------------------------------------------
//#define ENABLE_EVERY_2_PWM_CYCLES_DO_ONCE_FOC_SMO_CALCULATTION //if not define, do FOC & SMO Calculation in every PWM cycle.

//-----Enable CW TO CCW or CCW to CW Function By KEY1 INPUT------------------------------------------------------------------------------------------------
//#define ENABLE_MOTOR_REVERSIBLE_FUNCTION //20190504

//-----Enable ERROR CODE show to LED-----------------------------------------------------------------------------------------------------------------------
#define ENABLE_ERROR_CODE_SHOW_TO_LED //20190515

//-----Ialpha, Ibeta To SMO angle calculation--------------------------------------------------------------------------------------------------------------
#define I_DIVIDE_4_TO_SMO    //Ialpha, beta Divide 4  input to SMO parameter //20190626
//#define I_DIVIDE_2_TO_SMO    //Ialpha, beta Divide 2  input to SMO parameter //20190626
//#define I_MULTIPLY_TO_SMO  1   //range(1,2,4),if disable "I_DIVIDE_4_TO_SMO"and"I_DIVIDE_2_TO_SMO" , it will Multiply this setup value to Ialpha, beta for SMO parameter//20190626

//-----setup id integrator to 0 when in open loop mode----------------------------------------------------------------------------------------------------
#define ENABLE_Id_INTEGRATOR_TO_0_DURING_OPEN_LOOP	//20190626//set id integrator to 0 when in open loop mode	

//----rotor alignment parameters advance setup------------------------------------------------------------------------------------------------------
//if enable "ENABLE_ROTOR_ALIGN_FINISH_DETECTION", the OPEN_LOOP_ALIGNMENT_ANGLE need set to 0//20190623

//#define ENABLE_ROTOR_ALIGN_FINISH_DETECTION        //if enable, it will auto detect the rotor alignment has finished or not//20190623
#define ALIGNMENT_READY_THRESHOLD         2          //unit:mV, for 12bit Iu current detect threshold, when in align mode (it will measure Iu OP AMPLIFY OUTPUT)//20190623
#define DIVIDER_OF_ALIGN_INITIAL_CURRENT  1          //range(1,2,4),divider to control the current loop initial integral_result//20190623
#define OPEN_LOOP_ALIGNMENT_ANGLE         0          //ANGLE range(0~1023) equal to rotor (0~360degree), it means rotor 60degree = 171 ANGLE//20190623

//----define the new startup advance setup---------------------------------------------------------------------------------------------------------- 
#define NEW_STARTUP_RAMP_UP_ANGLE        171        //171 means 360*(171/1024) = 60degree//20190626
#define PWM_CYCLES_FOR_INCREASE_1_ANGLE             (NEW_STARTUP_RAMP_UP_TIME/NEW_STARTUP_RAMP_UP_ANGLE)//20190626

//----define enable 3 shunt R's  Iw current measurement //20190626------------------------------------------------------------------------------------------------
//#define ENABLE_3_SHUNT_R_CURRENT_MEASURE  //if not enable it, the current measure will be 1 or 2 shunt R, depend on define "ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT"

//----define lack phase check parameters //20190626------------------------------------------------------------------------------------------------------
//#define ENABLE_LACK_PHASE_DETECT_AND_PROTECT         //if enable, it will check there is a lack phase error or not, if yes,motor will be stop//20190626	
#define LACK_PHASE_RATIO                    4        //if average Ix phase current > average(1+ (Ratio/4))Iy, it will think there is a lack phase error//20190626	
#define LACK_PHASE_DETECT_CYCLE_TIME      512        //unit:2ms, 2*LACK_PHASE_DETECT_CYCLE_TIME mili-second to do once checking lack phase.(Ia, Ib, Ic sum current calculation and compare)

//********************************************************************************************************
//** Parameters Auto Calculation
//********************************************************************************************************
#define PWM_PERIOD       		            (SYSYTEM_CLOCK/2/PWM_FREQUENCY)     //48MHz/(PWM_FREQUENCY*2)
#define CURRENT_GAIN                    ((SHUNT_R_VALUE*CURRENT_AMPLIFICATION_FACTOR*655)/500)
#define OPEN_LOOP_RAMP_UP_TARGET_Iq     ((OPEN_LOOP_TARGET_CURRENT * CURRENT_GAIN)/10)
#define OPEN_LOOP_RAMP_DOWM_TARGET_Iq   ((OPEN_LOOP_RAMP_DOWN_CURRENT * CURRENT_GAIN)/10)	
#define CURRENT_LIMIT_Iq                ((CURRENT_LIMIT * CURRENT_GAIN)/10)	
#define NEW_STARTUP_Iq_SLOP             ((NEW_STARTUP_RAMP_UP_CURRENT_SLOP*CURRENT_GAIN)/(10*10)) //Iq cmd acceleartion slop in new startup mode//20190626
#define LACK_PHASE_CURRENT_THRESHOLD    ((LACK_PHASE_CURRENT*CURRENT_GAIN)/1000)//20190626	

#define INJECT_VOLTAGE_PULSE_Vd         (INJECT_VOLT_PULSE_AMPLITUDE * 327)//for IPD2
//#define PWM_NUM_FOR_INJECT_VOLT         (INJECT_VOLT_PULSE_TIME * PWM_NUM_IN_1ms)//define IPD2's inject voltage pulse to motor time//20190626
//#define PWM_NUM_FOR_NON_INJECT_VOLT     (NON_INJECT_VOLT_PULSE_TIME * PWM_NUM_IN_1ms)//define IPD2's off current time//20190626

#define LOW_SIDE_MIN_PWM_DUTY           (PWM_PERIOD - ((PWM_PERIOD*MAX_PWM_DUTY_PERCENTAGE_2SHUNT_R)/100))

#define MOTOR_RUN_CW                   1
#define MOTOR_RUN_CCW                  0

#ifdef ENABLE_COMP1_AND_iVREF_INTER_CONNECT_TIM1BKIN_DO_OVER_CURRENT_PROTECT
//----define over current detection TIM1_BKIN1's output to disable Motor PWM function enable or not--------------------------------------------------------------------------------
#define ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION   //enable this to do over current protection (PWM OFF), if TIM1_BKIN pin receive high signal
//----define over current detection comparator1's output to connect the internal TIM1BKIN-------------------------------------------------------------------------------------------
#define ENABLE_OVER_CURRENT_COMP1_PROTECTION   //enable this to do over current protection (PWM OFF), if comparator1 output high signal
#endif

#ifdef ENABLE_TIM1BKIN_INPUT_PIN_ONLY_DO_OVER_CURRENT_PROTECT
//----define over current detection TIM1_BKIN1's output to disable Motor PWM function enable or not--------------------------------------------------------------------------------
#define ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION   //enable this to do over current protection (PWM OFF), if TIM1_BKIN pin receive high signal
//----define TIM1BKIN signal input from external IO pin, if disable it, the TIM1BKIN input siganl must from internal comparator----------------------------------------------------
#define ENABLE_TIM1BKIN_PIN_EXTERNAL_INPUT        //enable TIM1BKIN input signal(MCU PB12 pin)
#endif

/********************************************************************************************************
** Hardware setup             
********************************************************************************************************/

//----define WL pin I/O ploarity in IPD mode-------------------------------------------------------------------------------
//#define WL_PIN_IO_MODE_LOW_ACTIVE     //if define WL pin will active low in IO mode. if not define, WL pin will active high in IO mode 
#define WL_PIN_IO_MODE_OUTPUT_LOW    GPIOB->BRR |= GPIO_Pin_15  //20190626
#define WL_PIN_IO_MODE_OUTPUT_HIGH   GPIOB->BSRR |= GPIO_Pin_15 //20190626

//----ADC Hardware channel Define------------------------------------------------------------------------------------------
#define BEMFA_ADC_DATA_REGISTER        (ADC1->ADDR0)//BEMF U voltage
#define BEMFB_ADC_DATA_REGISTER        (ADC1->ADDR1)//BEMF V voltage
#define PHASEA_CURR_ADC_DATA_REGISTER  (ADC1->ADDR5)//IU current measurement 
#define PHASEB_CURR_ADC_DATA_REGISTER  (ADC1->ADDR6)//IV current measurement
#define PHASEC_CURR_ADC_DATA_REGISTER  (ADC1->ADDR7)//IW current measurement//20190626
#define VSP_ADC_DATA_REGISTER          (ADC1->ADDR9)//Speed command input
#define ADC_CHANNEL_1_SHUNT_R_REGISTER (ADC1->ADDR5)//define the 1 shunt R ADC channel register
#define ISUM_ADC_DATA_REGISTER         (ADC1->ADDR3)//define Isum ADC channel register, for total current of motor
#define VBUS_ADC_DATA_REGISTER         (ADC1->ADDR8)//DC Bus voltage input//20181215

#define ADC_BEMF_A_CHANNEL_ENBALE       CHEN0_ENABLE
#define ADC_BEMF_B_CHANNEL_ENBALE       CHEN1_ENABLE
#define ADC_1_SHUNT_R_CHANNEL_ENABLE    CHEN5_ENABLE
#define ADC_2_SHUNT_R_CHANNEL_U_ENABLE  CHEN5_ENABLE
#define ADC_2_SHUNT_R_CHANNEL_V_ENABLE  CHEN6_ENABLE
#define ADC_3_SHUNT_R_CHANNEL_W_ENABLE  CHEN7_ENABLE//20190626
#define ADC_SPEED_CMD_IN_CHANNEL_ENABLE CHEN9_ENABLE
#define ADC_ISUM_CHANNEL_ENABLE         CHEN3_ENABLE
#define ADC_VBUS_CHANNEL_ENABLE         CHEN8_ENABLE//DC Bus voltage input//20181215

#define BEMFA_ADC_CHANNEL               0
#define BEMFB_ADC_CHANNEL               1
#define Ia_ADC_CHANNEL                  5
#define Ib_ADC_CHANNEL                  6
#define Ic_ADC_CHANNEL                  7 //20190626

#define ENABLE_LED_SHOW //if disable, it can remove LED pin, key input pin initialize

#define LED0_ON    GPIOA->BRR  |= GPIO_Pin_11   //Green light ON
#define LED0_OFF   GPIOA->BSRR |= GPIO_Pin_11   //Green light OFF
#define LED1_ON    GPIOA->BRR  |= GPIO_Pin_12   //Red light ON
#define LED1_OFF   GPIOA->BSRR |= GPIO_Pin_12   //Red light OFF
#define KEY0       GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)
#define LED1_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_12))?(GPIO_ResetBits(GPIOA,GPIO_Pin_12)):(GPIO_SetBits(GPIOA,GPIO_Pin_12))	

#define LED_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_14))?(GPIO_ResetBits(GPIOC,GPIO_Pin_14)):(GPIO_SetBits(GPIOC,GPIO_Pin_14))	
#define LED_TOGGLE_GPIOC14()  (GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_14))?(GPIO_ResetBits(GPIOC,GPIO_Pin_14)):(GPIO_SetBits(GPIOC,GPIO_Pin_14))	
#define LED_TOGGLE_GPIOA11()  (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_11))?(GPIO_ResetBits(GPIOA,GPIO_Pin_11)):(GPIO_SetBits(GPIOA,GPIO_Pin_11))	
#define LED_TOGGLE_GPIOA12()  (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_12))?(GPIO_ResetBits(GPIOA,GPIO_Pin_12)):(GPIO_SetBits(GPIOA,GPIO_Pin_12))	

#endif
