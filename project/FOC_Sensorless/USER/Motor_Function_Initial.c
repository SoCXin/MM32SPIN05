#include "Motor_function_initial.h"
#include "sys.h"
#include "PI.h"
#include "adc.h"
#include "Whole_Motor_Parameters.h"

extern uint16_t u16IncreaseAngle;//increase angle in every pwm period for open loop operation
extern __IO uint32_t u32DMA1ShuntRADCCHAddress;
#define SPEED_LOOP_FREQ    1000//unit : Hz, if define 1000, it means 1ms to calculate motor SMO speed once.(SMO_Speed_Estimator();)

void Function_Init(Whole_Motor_Struct* Motor)
{
	//system parameters setup
	Motor->command.u8MotorDirection = MOTOR_DIR_CW;
	Motor->BEMF.u8BEMFPoleNumber = POLE_NUMBER;
	Motor->spec.u16FreqSpeedLoop = SPEED_LOOP_FREQ;

	//for motor direction command initial
	Motor1.command.u8UserDirectionCommand = MOTOR_DIR_CW;//20190430
	//close loop speed calculation paramters setup
	Motor->smo.s16SpeedEstGainQ15  = ((uint32_t)Motor->spec.u16FreqSpeedLoop * 120) / Motor->spec.u8MotorPole;
  //open loop angle calculation parameters setup, define increase angle in every pwm period for open loop operation
	u16IncreaseAngle = (unsigned long)(Motor->spec.u8MotorPole>>1)*17895697 / PWM_FREQUENCY;
	//speed loop PI parameters setup
	speed_PI.s32PIOutputLimit = ((int32_t)Motor->spec.s16CurrentLimit * Motor->spec.s16CurrentGain)/10;
	speed_PI.s32IntegralResultLimit =  speed_PI.s32PIOutputLimit * SPEED_Ki_MULTIPLY;
	speed_PI.s32KpQ15 = PI_SPEED_Kp_VALUE;
	speed_PI.s32KiQ15 = PI_SPEED_Ki_VALUE;

	//current loop PI parameters setup
	Iq_PI.s32PIOutputLimit = (Motor->spec.u16PIOutLimit);
	Iq_PI.s32IntegralResultLimit = Iq_PI.s32PIOutputLimit * CURRENT_Ki_MULTIPLY;
	Id_PI.s32PIOutputLimit = Iq_PI.s32PIOutputLimit;
	Id_PI.s32IntegralResultLimit = Iq_PI.s32IntegralResultLimit;
	Iq_PI.s32KpQ15 = PI_CURRENT_Kp_VALUE;
	Id_PI.s32KpQ15 = PI_CURRENT_Kp_VALUE;


	//SMO parameters initial value setup
	Motor->smo.s32Kslf = SMO_Kslf_MIN_VALUE;
	Motor->smo.s32Kslid = SMO_Kslide;
	Motor->smo.s32KG = G_SMO_VALUE;
	Motor->smo.s32KF = F_SMO_VALUE;

	//Id command initial value setup
	Motor->command.s16IdCommand =0;//Id_COMMAND_VALUE;
}

/********************************************************************************************************
**函数信息 ：Initial_Motor_Parameter();
**功能描述 ：initial the motor parameters
**输入参数 ：none
**输出参数 ：none
********************************************************************************************************/
void Initial_Motor_Parameter(Whole_Motor_Struct* Motor)
{
	//motor system spec.
	Motor->spec.u16FreqPwm = PWM_FREQUENCY;
	Motor->spec.u8MotorPole = POLE_NUMBER;
	Motor->smo.u8SmoPole = POLE_NUMBER;

	//initial speed detection by measuring BEMF before motor startup
	Motor->BEMF.u16BEMF1msCounter =0;
	Motor->BEMF.u16BEMFSpeed = 0;
	Motor->BEMF.u16BEMFPhaseABMiddlePoint12bit =0;
	Motor->BEMF.u8BEMFDirectionFlag = BEMF_DIR_STANDSTILL;
	Motor->BEMF.u16BEMFDetectionTime = 0;
	Motor->BEMF.bBEMFResultValidFlag = 0;
	Motor->BEMF.bBEMFMotorIsRotatingFlag = 0;
	Motor->BEMF.u16BEMFComparatorHystersis =((BEMF_COMPARATOR_HYSTERESIS*4)/5);
	Motor->BEMF.u16BEMFStandstillThresholdVolt =((BEMF_STANDSTILL_THRESHOLD_VOLTAGE*4)/5);

	//assign user direction command
	Motor->command.u8MotorDirection = Motor1.command.u8UserDirectionCommand;//20190430

	//now's speed command
	Motor->command.u16SpeedCommandNow =0;//the motor now's speed command

	//define the open loop target speed to speed command, let motor run to this speed
	Motor1.command.u16SpeedTarget = OPEN_LOOP_TARGET_SPEED;

	//define the open loop Iq command target
	if(((OPEN_LOOP_TARGET_CURRENT * CURRENT_GAIN)/10) > 30000)
	{ Motor->command.s16OpenLoopTargetIq = 30000;}
	else
	{ Motor->command.s16OpenLoopTargetIq = (OPEN_LOOP_TARGET_CURRENT * CURRENT_GAIN)/10;}

	if(((OPEN_LOOP_INITIAL_CURRENT * CURRENT_GAIN)/10)> 30000)
	{ Motor->command.s16OpenLoopInitIq = 30000;}
	else
	{ Motor->command.s16OpenLoopInitIq = (OPEN_LOOP_INITIAL_CURRENT * CURRENT_GAIN)/10;}


	Motor->command.s16OpenLoopNowIq = Motor->command.s16OpenLoopInitIq;

	//define the open loop ramp up slop
	Motor->spec.u16SpeedSlope1 = OPEN_LOOP_RAMP_UP_SPEED_SLOP1;
	Motor->spec.u16SpeedSlope2 = OPEN_LOOP_RAMP_UP_SPEED_SLOP2;
	Motor->spec.u16SpeedSlope3 = OPEN_LOOP_RAMP_UP_SPEED_SLOP3;

	//define the open loop target speed for enter close loop's criteria
	Motor->spec.u16OpenLoopTargetSpeed = OPEN_LOOP_TARGET_SPEED;

	//define the initial value of motor reverse command
	Motor->command.u8MotorReverseCommand =0;

	//define the integrator initial value of PI controller
	Iq_PI.s32IntegralResult =0; //the Vq integrator reset to 0
	Id_PI.s32IntegralResult =0; //the Vd integrator reset to 0
	speed_PI.s32IntegralResult =0;  //the speed Iq integrator reset to 0

	//define the initial Vd, Vq value
	Motor1.FOC.s16Vq =0;
	Motor1.FOC.s16Vd =0;

	//define the initial PWM DUTY =0;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	//define rotor IPD1 function parameters
	#ifdef ENABLE_ROTOR_IPD1_FUNCTION
	Motor1.IPD.u16IncAddressTimer_1ms = (ROTOR_IPD1_SMALL_SWING_TIME/32);
	Motor1.IPD.u16RIPDQualifyFailCounter = 0;//clear the counter
	#endif

	//for 1 shunt R current sensing
	u32DMA1ShuntRADCCHAddress = (u32)(&(ADC_CHANNEL_1_SHUNT_R_REGISTER));

	//for power consumption limit
	Motor1.info.u16RatioSpeedError =0;

	//for motor rotation inverse check in close loop //20190124
	Motor1.smo.u16RotationInverseCounter =0;

	//initial motor reverse command //20190502
	Motor1.command.u8MotorReverseCommand= 0;//clear to 0 to cancel reverse run command

	//initial over power limit flag
	Motor->command.u8OverPowerLimitFlag =0;//20190606

	Motor->command.u16Angle = 512;//20190626

	//initial new startup mode parameters
	Motor->NewStartup.u16NewStartupPWMCounter=0;//20190626
	Motor->NewStartup.u16NewStartup100msCounter =0;//20190626
	Motor->NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
	Motor->NewStartup.u16NewStartupTime_10ms_total =NEW_STARTUP_MIN_TIME;//20190626
	Motor->NewStartup.u16NewStartupRampUpTime = NEW_STARTUP_RAMP_UP_TIME;//20190626
	Motor->NewStartup.s16New_Startup_TargetIq = -(NEW_STARTUP_TARGET_CURRENT * CURRENT_GAIN)/10;//20190626
	Motor->NewStartup.u16NewStartupMaxSpeedLimit = NEW_STARTUP_OVER_SPEED;//20190626

	Motor->NewStartup.u16New_StartupIqAddPer_ms =10;//default value, it means Iq command increase 10 every 1ms for new startup mode Iq slop control

	if(NEW_STARTUP_Iq_SLOP >=1000)//20190626
	{
		Motor->NewStartup.u16New_StartupIqSlopeCnt =1;
		Motor->NewStartup.u16New_StartupIqAddPer_ms = 10*(1+(NEW_STARTUP_Iq_SLOP/1000));//get 1ms add 10 or 20,30,...of Iq command
	}
	else if(NEW_STARTUP_Iq_SLOP == 0)
	{Motor->NewStartup.u16New_StartupIqSlopeCnt =1000;}
	else
	{Motor->NewStartup.u16New_StartupIqSlopeCnt = 1+(1000 / NEW_STARTUP_Iq_SLOP);}//get how many ms to increase 10 of Iq command

	if(PWM_CYCLES_FOR_INCREASE_1_ANGLE <1)//20190626
	{ Motor->NewStartup.u16NewStartupPWMCyclesToIncrease1Angle =1;}
	else{ Motor->NewStartup.u16NewStartupPWMCyclesToIncrease1Angle =PWM_CYCLES_FOR_INCREASE_1_ANGLE;}
}










































