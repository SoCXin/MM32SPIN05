#ifndef __WHOLE_MOTOR_PARAMETERS_H_
#define __WHOLE_MOTOR_PARAMETERS_H_
#include "HAL_device.h" 
#include "HAL_conf.h"

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

#define PWM_FREQUENCY    		16000           //unit:HZ
#define POLE_NUMBER      		14//8
#define MOTOR_DIR_CW     		1               //1:CW, 0:CCW rotation direction define
#define TARGET_MAX_SPEED    TARGET_SPEED_7  //unit:RPM, max. speed limitation
#define TARGET_MIN_SPEED    TARGET_SPEED_1  //unit:RPM, min. speed limitation

//----Enable/Disable 1 shunt R mode----------------------------------------------------------------------------------------
#define ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT //if not define, it will use 2 shunt R to measure 3 phase current

//------2 shunt R phase current sensing parameter-------------------------------------------------------------------------------------
#define MAX_PWM_DUTY_PERCENTAGE_2SHUNT_R    90 //unit:%, it need to limit the maximum pwm duty output for doing the 2 shunt R current sensing

//------1 shunt R phase current sensing parameter-------------------------------------------------------------------------------------
#define ADC_DELAY_TIME_1_SHUNT_R   35//25//35 //unit : 0.1us (dead time + OP output signal stable time)
#define ADC_TOTAL_TIME_1_SHUNT_R   45//35//45 //unit : 0.1us (dead time + OP output signal stable time + ADC sample time)

//------Rotor initial position detection method 1 parameters-------------------------------------------------------------------------------------
#define ENABLE_ROTOR_IPD1_FUNCTION          //if define, it will use IPD1 function to detect the initial position of rotor before startup motor
#define ROTOR_IPD1_TARGET_CURRENT       2  //unit:0.1A define the target torque current in ROTOR_SMALL_SWING_MODE
#define ROTOR_IPD1_SMALL_SWING_TIME     100 //unit:ms, define rotor small swing time with ROTOR_IPD1_TARGET_CURRENT torque current
#define ROTOR_IPD1_DETECT_TIME          80  //unit:ms, define rotor angle detection time after rotor small swing, range 40~100ms
#define ROTOR_IPD1_BEMF_SUM_MINI        50  //unit:mV, define the minimum sum value of bemf |A| + |B| absolute value
#define ROTOR_IPD1_MAX_FAIL_TIMES       10  //range:(6~65535),if IPD failure times over this setup value, motor operation will enter open loop mode directly.

//-----Open loop parameters-------------------------------------------------------------------------------------------------
#define OPEN_LOOP_TARGET_SPEED         (TARGET_SPEED_7/4) //unit:RPM,define open loop target speed, then will enter close loop control
#define OPEN_LOOP_RAMP_UP_SPEED_SLOP1   15 //30  //unit:RPM/sec,range(1~1000),acceleration slop in open loop 1st stage
#define OPEN_LOOP_RAMP_UP_SPEED_SLOP2   30 //60  //unit:RPM/sec,range(1~1000),acceleration slop in open loop 2nd stage
#define OPEN_LOOP_RAMP_UP_SPEED_SLOP3   60 //90  //unit:RPM/sec,range(1~1000),acceleration slop in open loop 3th stage

#define OPEN_LOOP_RAMP_DOWN_SPEED_SLOP1  30 //unit:RPM/sec,range(1~1000),deacceleration slop in open loop 1st stage
#define OPEN_LOOP_RAMP_DOWN_SPEED_SLOP2  60 //unit:RPM/sec,range(1~1000),deacceleration slop in open loop 2nd stage
#define OPEN_LOOP_RAMP_DOWN_SPEED_SLOP3  90 //unit:RPM/sec,range(1~1000),deacceleration slop in open loop 3th stage

#define RAMP_UP_CHANGE_TO_SLOP2_SPEED    15 //unit:RPM,define the open loop threshold speed for ramp up slop1 change to slop2
#define RAMP_UP_CHANGE_TO_SLOP3_SPEED    60 //unit:RPM,define the open loop threshold speed for ramp up slop2 change to slop3

#define OPEN_LOOP_TARGET_CURRENT         3 //unit:0.1A define the target  torque current in open loop ramp up stage
#define OPEN_LOOP_INITIAL_CURRENT        4 //unit:0.1A,define the initial torque current in open loop ramp up stage 
#define OPEN_LOOP_RAMP_DOWN_CURRENT      3 //unit:0.1A,define the torque current in open loop ramp down stage

#define OPEN_LOOP_RAMP_UP_CURRENT_SLOP1   4 //unit:0.1A/sec,range(1~40),increase how many phase current per second in open loop 1st stage
#define OPEN_LOOP_RAMP_UP_CURRENT_SLOP2   4 //unit:0.1A/sec,range(1~40),increase how many phase current per second in open loop 2nd stage
#define OPEN_LOOP_RAMP_UP_CURRENT_SLOP3   10//unit:0.1A/sec,range(1~40),increase how many phase current per second in open loop 3th stage

//-----Close loop parameters------------------------------------------------------------------------------------------------
#define TARGET_SPEED_1                60 //unit: RPM, define the user's lowest  target speed
#define TARGET_SPEED_2                90 //unit: RPM, define the user's second  target speed
#define TARGET_SPEED_3                150 //unit: RPM, define the user's Third   target speed
#define TARGET_SPEED_4                210 //unit: RPM, define the user's 4th     target speed
#define TARGET_SPEED_5                270 //unit: RPM, define the user's 5th     target speed
#define TARGET_SPEED_6                300 //unit: RPM, define the user's 6th     target speed
#define TARGET_SPEED_7                380 //unit: RPM, define the user's highest target speed

#define CLOSE_LOOP_RAMP_UP_SPEED_SLOP (TARGET_SPEED_7/5) //unit:RPM/sec,range(1~300),acceleration slop in close loop
#define CLOSE_LOOP_SPEED_TO_STOP_PWM  ((OPEN_LOOP_TARGET_SPEED*4)/2) //unit: RPM,(must > OPEN_LOOP_TARGET_SPEED),if receive motor stop command, pwm will off when speed under this define

//-----Motor Forward/against wind startup parameters------------------------------------------------------------------------
#define DIRECT_ENTER_CLOSE_LOOP_MIN_SPEED (TARGET_SPEED_7/5) //unit:RPM,define the mini. bemf speed which can directly enter close loop when motor startup
#define DOWNWIND_OPEN_LOOP_TARGET_SPEED   (TARGET_SPEED_7/5) //unit:RPM,define the open loop into close loop's speed in low speed downwind startup status

//-----1,2 Shunt R  PI Control parameters-------------------------------------------------------------------------------------------------
#define SHUNT_R_VALUE                 250 //unit:milli ohm, 50 means 0.05 ohm

#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
  #define CURRENT_AMPLIFICATION_FACTOR  60 //unit:0.1 amplification factor, 60 means 60*0.1 = 6 amplification factor
#else
  #define CURRENT_AMPLIFICATION_FACTOR  50 //unit:0.1 amplification factor, 50 means 50*0.1 = 5 amplification factor
#endif

#define CURRENT_LIMIT    4   //unit: 0.1A, it will limit the maximum sine wave phase current in close loop operation
#define CURRENT_PROTECT  12   //unit: 0.1A, it will stop the motor if sine wave phase current over this value

#define PI_OUT_LIMIT     28000 //range(0~32767),Setup the Vd, Vq PI output limitation 

//-----I_SUM R parameters for totoal current measurement-------------------------------------------------------------------
#define ISUM_R_VALUE                       250 //unit:milli ohm, 40 means 0.04 ohm
#define ISUM_CURRENT_AMPLIFICATION_FACTOR  60 //unit:0.1 amplification factor, 50 means 50*0.1 = 5 amplification factor

//-----Motor SMO parameters -----------------------------------------------------------------------------------------------
#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
#define SMO_Kslf_MAX_VALUE      900//2400 //recommended value (300~6000), range (1~32767)
#define SMO_Kslf_MIN_VALUE      300//1200 //recommended value (100~2000), range (1~32767)
#else
#define SMO_Kslf_MAX_VALUE      900//4200 //recommended value (300~6000), range (1~32767)
#define SMO_Kslf_MIN_VALUE      300//1600 //recommended value (100~2000), range (1~32767)
#endif

#define SMO_Kslide              26000 //recommended value 26000, range (1~32767)
#define G_SMO_VALUE             19000 //recommended value 19000, range (1~65535)
#define F_SMO_VALUE             30000 //recommended value 30000, range (1~65535)

//-----Motor FOC current PI parameters ------------------------------------------------------------------------------------
#define PI_CURRENT_Kp_VALUE       24000
#define PI_CURRENT_Ki_MAX_VALUE   2400//5600
#define PI_CURRENT_Ki_MIN_VALUE   300//280//560

//-----Motor speed PI parameters ------------------------------------------------------------------------------------------
#define PI_SPEED_Kp_VALUE   24000
#define PI_SPEED_Ki_VALUE   800//1500

//-----Motor Error code define---------------------------------------------------------------------------------------------
#define MAX_ERROR_ACCUMULATIVE_TOTAL   10 //if error counter over or equal this value, the motor will not restart again until receive off command
#define ERROR_RESTART_WAIT_TIME        10 //unit :100ms, after a error happened, waiting this time then can restart motor

#define MOTOR_SPEED_ERROR              1
#define MOTOR_PHASE_CURRENT_ERROR      2
#define MOTOR_OVER_CURRENT             3


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
//#define ENABLE_COMP1_AND_iVREF_INTER_CONNECT_TIM1BKIN_DO_OVER_CURRENT_PROTECT //if use internal COMPARATOR and internal Vref 1.2V with TIM1BKIN brake PWM,user can enable this define
//#define ENABLE_TIM1BKIN_INPUT_PIN_ONLY_DO_OVER_CURRENT_PROTECT              //if use external COMPARATOR, user can enable this define (input to PB12 pin)

//----define the internal comparator inverting input (INM) 1,3/4,1/2,1/4Vref voltage ------------------------------------------------------------------------------------------------
#define INTERNAL_COMPARATOR_INM_REF_VOLTAGE   COMP_InvertingInput_VREFINT//COMP_InvertingInput_1_4VREFINT//COMP_InvertingInput_1_2VREFINT//COMP_InvertingInput_3_4VREFINT

//----define TIM1BKIN input High or Low to make PWM OUTPUT OFF---------------------------------------------------------------------------------------------------------------------
#define BREAK_POLARITY   TIM_BreakPolarity_High //TIM_BreakPolarity_Low//if define High, TIM1BKIN receive high signal will let Motor PWM OFF 


//----define ISUM current measurement enable or not--------------------------------------------------------------------------------
#define ENABLE_ISUM_MEASUREMENT   //enable this to measure total current and power consumption

//----define Maximum Power Consumption Control--------------------------------------------------------------------------------
#define ENABLE_MAX_POWER_LIMIT    //if need enable this define, user must enable the #define ENABLE_ISUM_MEASUREMENT first
#define MAX_POWER_LIMIT     960     //unit: 0.1W, define the max power limitation, 80 means it will limit power consumption in 8W
#define DC_BUS_VOLTAGE      3100     //unit: 0.1V, define the dc bus voltage, 100 means 10V

//------Rotor initial position detection method 2 parameters-------------------------------------------------------------------------------------
//#define ENABLE_ROTOR_IPD2_FUNCTION            //if define, it will use IPD2 function to detect the initial position of rotor before startup motor
#define INJECT_VOLT_PULSE_TIME          1     //unit:ms, define motor inject voltage pulse time to do standstill rotor position detection 
#define NON_INJECT_VOLT_PULSE_TIME      4     //unit:ms, define motor non-inject voltage pulse time to release motor (off voltage injection)
#define INJECT_VOLT_PULSE_AMPLITUDE    20     //unit:%,  define motor inject voltage pulse amplitude,range (1~10)
#define PWM_NUM_IN_1ms                 16     //unit:cycles, define how many pwm cycle in 1ms time duration

//----define the against wind startup motor parameters-------------------------------------------------------------------------
//for restart the motor when motor is in againstwind status
#define AGAINST_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA  2  //define open loop speed ramp up slop divider para., 2 means 1/2  normal ramp up speed acceleration//20180807
#define AGAINST_WIND_CLOSE_TO_OPEN_LOOP_SPEED          ((OPEN_LOOP_TARGET_SPEED)+50) //(need > OPEN_LOOP_TARGET_SPEED), define the close to open loop speed in reverse and high speed status//20180807
#define AGAINST_WIND_OPEN_TO_CLOSE_TARGET_SPEED        (OPEN_LOOP_TARGET_SPEED)//(need < OPEN_LOOP_TARGET_SPEED),define the open to close speed after motor is stopped in against wind//20180807
#define K_AGAINST_WIND_OPL                             2  //suggest range 1~4, modulate the open loop initial Iq integral//20180807

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
#define BEMF_COMPARATOR_HYSTERESIS         30   //unit: mV,  it define the digital comparator's hysteresis voltage
#define BEMF_STANDSTILL_THRESHOLD_VOLTAGE  200  //unit: mV,  it define the standstill if BEMFA,B difference voltage under this limitation 
#else
#define BEMF_COMPARATOR_HYSTERESIS         30   //unit: mV,  it define the digital comparator's hysteresis voltage
#define BEMF_STANDSTILL_THRESHOLD_VOLTAGE  100   //unit: mV,  it define the standstill if BEMFA,B difference voltage under this limitation 
#endif

//----define the Dead time--------------------------------------------------------------------------------------------------------
#define DEAD_TIME_SETUP            48 //unit : 20.83ns (at system clock 48MHz),range(0~127)

//----define the K parameter for the initial Vq when motor startup in downwind status--------------------------------------------- 
#define K_STARTUP_Vq_INITIAL_PARA  256 //range(1~512),the initial Vq integrator parameter,for motor startup direct into close loop status 

//----Enable/disable force motor always in open loop---------------------------------------------------------------------------------------
//#define FORCE_STAY_IN_OPEN_LOOP       //if enable it, it will always keep in open loop mode


//----define the PVD(programmable voltage detector) voltage, softeware will reset MCU if VDD under this define voltage---------------------
#define PVD_VOLTAGE     PWR_PVDLevel_3V6  //3V6 means 3.6V, (Option : 2V7,3V0,3V6,3V9,4V2...)

//----define enable/disable MOSFET WL pin turn on at CHECK_BEMF_MODE-----------------------------------------------------------------------
#define  ENABLE_WL_ON_AT_CHECK_BEMF_MODE  //if define, it means enable WL turn on at CHECK BEMF MODE before motor startup


//********************************************************************************************************
//** Parameters Auto Calculation
//********************************************************************************************************
#define PWM_PERIOD       		            (SYSYTEM_CLOCK/2/PWM_FREQUENCY)     //48MHz/(PWM_FREQUENCY*2)
#define CURRENT_GAIN                    ((SHUNT_R_VALUE*CURRENT_AMPLIFICATION_FACTOR*655)/500)
#define OPEN_LOOP_RAMP_UP_TARGET_Iq     ((OPEN_LOOP_TARGET_CURRENT * CURRENT_GAIN)/10)
#define OPEN_LOOP_RAMP_DOWM_TARGET_Iq   ((OPEN_LOOP_RAMP_DOWN_CURRENT * CURRENT_GAIN)/10)	
#define CURRENT_LIMIT_Iq                ((CURRENT_LIMIT * CURRENT_GAIN)/10)	

#define INJECT_VOLTAGE_PULSE_Vd         (INJECT_VOLT_PULSE_AMPLITUDE * 327)//for IPD2
#define PWM_NUM_FOR_INJECT_VOLT         (INJECT_VOLT_PULSE_TIME * PWM_NUM_IN_1ms)//define IPD2's inject voltage pulse to motor time
#define PWM_NUM_FOR_NON_INJECT_VOLT     (NON_INJECT_VOLT_PULSE_TIME * PWM_NUM_IN_1ms)//define IPD2's off current time

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

//----ADC Hardware channel Define------------------------------------------------------------------------------------------
#define BEMFA_ADC_DATA_REGISTER        (ADC1->ADDR0)
#define BEMFB_ADC_DATA_REGISTER        (ADC1->ADDR1)
#define PHASEA_CURR_ADC_DATA_REGISTER  (ADC1->ADDR5) 
#define PHASEB_CURR_ADC_DATA_REGISTER  (ADC1->ADDR6) 
//#define VSP_ADC_DATA_REGISTER          (ADC1->ADDR8)//Original
#define VSP_ADC_DATA_REGISTER          (ADC1->ADDR9)//24V EVB
//#define ADC_CHANNEL_1_SHUNT_R_REGISTER (ADC1->ADDR2) //define the 1 shunt R ADC channel register
#define ADC_CHANNEL_1_SHUNT_R_REGISTER (ADC1->ADDR5) //24V EVB,define the 1 shunt R ADC channel register
#define ISUM_ADC_DATA_REGISTER         (ADC1->ADDR3) //define Isum ADC channel register, for total current of motor

#define ADC_BEMF_A_CHANNEL_ENBALE       CHEN0_ENABLE
#define ADC_BEMF_B_CHANNEL_ENBALE       CHEN1_ENABLE
//#define ADC_1_SHUNT_R_CHANNEL_ENABLE    CHEN2_ENABLE
#define ADC_1_SHUNT_R_CHANNEL_ENABLE    CHEN5_ENABLE//24V EVB
#define ADC_2_SHUNT_R_CHANNEL_U_ENABLE  CHEN5_ENABLE
#define ADC_2_SHUNT_R_CHANNEL_V_ENABLE  CHEN6_ENABLE
//#define ADC_SPEED_CMD_IN_CHANNEL_ENABLE CHEN8_ENABLE//Original
#define ADC_SPEED_CMD_IN_CHANNEL_ENABLE CHEN9_ENABLE//24V EVB
#define ADC_ISUM_CHANNEL_ENABLE         CHEN3_ENABLE

#define BEMFA_ADC_CHANNEL               0
#define BEMFB_ADC_CHANNEL               1
#define Ia_ADC_CHANNEL                  5
#define Ib_ADC_CHANNEL                  6
//#define VSP_ADC_CHANNEL               8//Original
#define VSP_ADC_CHANNEL                 9//24V EVB

#define LED_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_14))?(GPIO_ResetBits(GPIOC,GPIO_Pin_14)):(GPIO_SetBits(GPIOC,GPIO_Pin_14))	

#endif
