#include "delay.h"
#include "sys.h"
#include "led.h"
#include "key.h"
#include "adc.h"
#include "iwdg.h"
#include "HAL_hdiv.h"
#include "pwm.h"
#include "Parameter.h"
#include "PI.h"
#include "Sin_table.h"
#include "Motor_function_initial.h"
#include "delay2.h"
#include "GPIO.h"
#include "Whole_Motor_Parameters.h"
#include "HSI_calibration.h"
#include "GPIO.h"
#include "pwr_pvd.h"

#define ADCR_EXTTRIG_Set              ((uint32_t)0x00000004)
#define ADCR_EXTTRIG_Reset            ((uint32_t)0xFFFFFFFB)

extern void Init_Comparator(void);//initial internal comparator
extern void Initial_Motor_Parameter(Whole_Motor_Struct* Motor);//for initial motor parameters
extern void Function_Init(Whole_Motor_Struct* Motor);
extern void Enable_Motor1_PWM_Output(void);//enable pwm output
extern void Disable_Motor1_PWM_Output(void);//disable pwm output
extern void Generate_Now_Speed_Command(Whole_Motor_Struct* Motor);//Generate speed command for control the speed acceleration slop in open/close loop
void Generate_Now_Speed_Command_Close_Loop(Whole_Motor_Struct* Motor);
extern void Generate_Open_Loop_Now_Iq_Command(Whole_Motor_Struct* Motor);//Generate Iq command for control the open loop Iq acceleration slop
extern void Generate_Open_Loop_Angle(Whole_Motor_Struct* Motor);//Generate Angle in open loop mode
extern void Get_Angle_Difference(Whole_Motor_Struct* Motor);//generate the difference angle between open loop angle and smo angle
extern void Generate_Open_loop_Slope_Counter_Value(Whole_Motor_Struct* Motor);
extern void Generate_Close_loop_Slope_Counter_Value(Whole_Motor_Struct* Motor);//20190626
extern int16_t Cos(uint16_t u16TableAddr);
extern int16_t Sin(uint16_t u16TableAddr);
extern int16_t IPD_Sin(uint16_t u16TableAddr);
//extern void New_Startup_Mode_Process(void);//20190626
extern void New_Startup_Common_Setup(void);//20190626
//extern uint16_t Generate_Rotor_BEMF_AB_Absolute_Value(int32_t s32BEMF_A,int32_t s32BEMF_B);
extern int16_t Under_Over_16_cp(int16_t s16InputAngle);
extern void IO_SPI_Init(void);
extern void DAC12bit_show(int32_t s32DACShowData);


//-------for BEMF U,V voltage detection before startup motor when has diode circuits in feedback path ---------------------
void WL_PIN_TURN_OFF_MOSFET(void);     //Setup PB15 pin turn off MOSFET when it is in IO mode
void WL_PIN_TURN_ON_MOSFET(void);      //Setup PB15 pin turn on  MOSFET when it is in IO mode
void SWITCH_WL_PIN_TO_IO_OUTPUT(void); //PB15 pin change to IO output mode
void SWITCH_WL_PIN_TO_PWM_OUTPUT(void);//PB15 pin change to PWM output mode

//-------for motor speed calculation when in close loop mode----------------------------------
void SMO_Speed_Estimator(SMO_Struct* Motor);

//-------for getting the user on/off, speed command ------------------------------------------
void User_Command_Receive(void);

//-------for getting the user direction command for motor ------------------------------------
void User_Motor_Direction_Command_Receive(void);//20190430

//-------total current measurement------------------------------------------------------------
void Isum_Current_Measurement(void);//in 10ms time base loop

//-------DC Bus voltage measurement-----------------------------------------------------------
void VBUS_Voltage_Measurement(void);//in 10ms time base loop//20181215

//-------DC Bus over/under voltage protection-------------------------------------------------
void VBUS_Voltage_Protection(void);//20190515

//-------show error code to LED---------------------------------------------------------------
void Error_Code_Show_to_LED(void);//20190515

//-------for 1 shunt R current sensing-------------------------------------------------------
void Reconstructed_3_Phase_Current_for_1_Shunt_R(void);

//-------Read PWM Break status and processing it---------------------------------------------
void Read_Break_Status_and_Processing(void);

//-------Generate angle for FOC calculation--------------------------------------------------
void Generate_Angle_For_FOC_Calculation(void);

//------BEMF check is Standstill,DownWind or AgainstWind process-----------------------------
void Standstill_BEMF_Process(void);   //Standstill process after BEMF check has done
void DownWind_BEMF_Process(void);     //Downwind process after BEMF check has done
void AgainstWind_BEMF_Process(void);  //Againstwind process after BEMF check has done
void Restart_Setup_After_Motor_Stopped_At_AgainstWind_or_Dir_CMD_Change(void);//after motor is stopped by reverse action,Setup parameters for motor restart

//------rotor initial position detection (IPD1) process---------------------------------------
void Rotor_Small_Swing_Mode_Setup(void);
void Rotor_Position_Detection_Mode_Setup(void);

//------rotor initial position detection (IPD2) process--(by Inductance saturation effect)----
void After_IPD2_Process(void);

//-------for open loop ramp down speed slop1,2, 3 setup and target Iq command setup ---------
void Setup_Open_Loop_Ramp_Down_Speed_Slop_and_Target_Iq(Whole_Motor_Struct* Motor);

//-----new startup mode setup and check-------------------------------------------------------
void New_Startup_Setup_After_IPD1(void);//20190626
void New_Startup_Setup_After_Alignment_Mode(void);//20190626
void New_Startup_Setup_After_Open_Loop_Mode(void);//20190626
void New_Startup_Setup_In_Low_Speed_Against_Wind(void);//20190626
void New_Startup_Setup_After_Lock_Rotor(void);//20190626
void New_Startup_Mode_Over_Time_Error_Check(void);//check new startup of motor is success or fail//20190626
extern void Generate_New_Startup_Now_Iq_Command(Whole_Motor_Struct* Motor);//20190626

//-------for starting/ stopping the motor operation -----------------------------------------
void Stop_The_Motor1(void);
void Start_The_Motor1(void);

//-------motor error protection--------------------------------------------------------------
void Speed_Error_Protect_Motor1(void);
void Software_Over_Current_Protect_Motor1(int16_t s16PhaseCurrentX);
void Rotation_Inverse_Error_Protect_Motor1(void);//20190124
void Lack_Phase_Detect_and_Protect_Motor1(void);//20190626

//-------smo kslf modulation-----------------------------------------------------------------
void SMO_Kslf_Modulate_By_Speed_Motor1(void);
void SMO_Kslf_Modulate_For_New_Startup_By_Speed_Motor1(void);//20190626

//-------Id command modulation-----------------------------------------------------------------
void Id_Command_Modulate_By_Speed_Motor1(void);//20181115

//-------generate Iq command in close loop control-------------------------------------------
int16_t Generate_Iq_Command_IN_Close_Loop_Motor1(void);

//-------Current PI Ki modulation------------------------------------------------------------
void PI_Current_Ki_Modulate_By_Speed_Motor1(void);

//-------limit the max power consumption & dc bus current------------------------------------
void Max_Power_Consumption_limit(void);
void Max_DCBUS_Current_limit(void);//20190626

//-------rotor align is ready or not check---------------------------------------------------
void Rotor_Align_finish_Detect(void);

//-------for reset watchdog timer -----------------------------------------------------------
void Reset_IWatchdog_in_main_Loop(void);
void Reset_IWatchdog_in_500ms_Loop(void);

//-------for IWDG (Independent watchdog) use-------------------------------------------------
__IO uint16_t u16IWDGPingPong = 0x5555;//fixed number check for IWDG counter clear in two place.

//-------for System_Time_Management() use----------------------------------------------------
extern __IO uint32_t u32SystemTick100us;

__IO uint16_t u16SystemTime100us;
__IO uint16_t u16SystemTime200us;
__IO uint16_t u16SystemTime1ms;
__IO uint16_t u16SystemTime2ms;
__IO uint16_t u16SystemTime10ms;
__IO uint16_t u16SystemTime50ms;
__IO uint16_t u16SystemTime100ms;
__IO uint16_t u16SystemTime500ms;

//-------for adc result buffer use----------------------------------------------------
__IO uint16_t u16ADC1[8];

//-------for 12bit DAC control use----------------------------------------------------
void IO_SPI_DAC(uint16_t u16Data);

//-------others-----------------------------------------------------------------------
__IO uint8_t u8EnterCloseCounter;
__IO uint8_t u8RestartupWaitCounter;

#ifdef ENABLE_OPEN_TO_CLOSE_ADVANCE_GRADUALLY
__IO uint8_t u8OpenEnterCloseProcess;
#endif

extern uint16_t  u16BEMF_A_B_VoltageDiffAverage;
extern   __IO uint32_t u32DMAtoT1CC1[2];
extern   __IO uint32_t u32DMAtoT1CC2[2];
extern   __IO uint32_t u32DMAtoT1CC3[2];
extern   __IO int32_t  s32DMAtoADDR5[3];
extern   __IO uint32_t u32DMAtoT3CC1[2];

uint8_t  u8SvpwmZoneSwitch;
int32_t  s32PhaseCurrentU,s32PhaseCurrentV,s32PhaseCurrentW;
int32_t  s32OffsetCurrent = 780;

__IO int32_t s32Temp_1st,s32Temp_2nd;

__IO uint32_t u32DMA1ShuntRADCCHAddress;
//-------parameters define------------------------------------------------------------
#define STOP_MODE                           0
#define ZERO_CURRENT_CALIBRATE_MODE         1
#define CHECK_BEMF_MODE                     2
#define STANDSTILL_BEMF_CALIBRATE_MODE      3
#define DETECT_ROTOR_POSITION_MODE          4
#define RUN_IN_OPEN_LOOP_MODE               5
#define RUN_IN_CLOSE_LOOP_MODE              6
#define ROTOR_SMALL_SWING_MODE              7
#define INDUCTANCE_SAT_POSITION_DET_MODE    8
#define RUN_IN_ALIGNMENT_MODE               9
#define RUN_IN_NEW_STARTUP_MODE            10

/********************************************************************************************************
**function name        ：System_Time_Management(void)
**function description ：for system time management, there are 100us, 1ms, 2ms, 10ms, 50ms, 100ms,500ms,1000ms time loop
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void System_Time_Management(void)
{
	if( u32SystemTick100us >0)//100us loop, 100us executed once in this loop
	{
        u32SystemTick100us--;
        u16SystemTime100us++;
    }
	if(u16SystemTime100us>2)//200us loop, 200us executed once in this loop
	{
        u16SystemTime100us -=2;
        u16SystemTime200us++;
        #ifdef ENABLE_MAX_120000RPM_SPEED_CALCULATION    //(2 poles)
        /*for generating the motor speed in close loop,Motor.smo.s16Speed is speed output, unit is rpm*/
        if(Motor1.command.u8MotorRunMode >= RUN_IN_OPEN_LOOP_MODE)//if yes, means in open / close loop mode
        { SMO_Speed_Estimator(&Motor1.smo);}//for generating the motor speed in close loop,Motor.smo.s16Speed is speed output
		#endif
	}

	if(u16SystemTime200us>5)//1ms loop, 1ms executed once in this loop
	{
		u16SystemTime200us -=5;
		u16SystemTime1ms++;

    #ifdef ENABLE_MAX_24000RPM_SPEED_CALCULATION   //(2 poles)
		/*for generating the motor speed in close loop,Motor.smo.s16Speed is speed output, unit is rpm*/
		if(Motor1.command.u8MotorRunMode >= RUN_IN_OPEN_LOOP_MODE)//if yes, means in open / close loop mode
        { SMO_Speed_Estimator(&Motor1.smo);}//for generating the motor speed in close loop,Motor.smo.s16Speed is speed output
		#endif

		/*for detecting the motor speed by measuring BEMF before motor startup */
		if(Motor1.command.u8MotorRunMode == CHECK_BEMF_MODE)//for initial speed detection by BEMF feedback before startup motor
		 {
		    #ifdef ENABLE_WL_ON_AT_CHECK_BEMF_MODE//20190402
					#ifdef ENABLE_MINI_WL_ON_WIDTH_AT_CHECK_BEMF_MODE
							 WL_PIN_TURN_ON_MOSFET();//WL pin change to I/O mode and turn on MOSFET
			         delay_us(WL_ON_TIME_SETUP_AT_CHECK_BEMF_MODE);//Setup WL on time for measuring BEMF U,V
							 BEMF_Speed_Detect(&Motor1.BEMF);//measure the initial speed & direction of motor before running the motor
							 SWITCH_WL_PIN_TO_PWM_OUTPUT();//WL pin OFF and change to PWM mode, but not enable PWM signal output to pin
					#else
						 BEMF_Speed_Detect(&Motor1.BEMF);//measure the initial speed & direction of motor before running the motor
					#endif
        #else
				 BEMF_Speed_Detect(&Motor1.BEMF);//measure the initial speed & direction of motor before running the motor
			  #endif
		 }

		/*for modulate the open loop speed acceleration slop */
		if(Motor1.command.u8MotorRunMode == RUN_IN_OPEN_LOOP_MODE)//for speed and Iq commmand ramp up control in open loop
		 {
       Generate_Open_loop_Slope_Counter_Value(&Motor1);//generate the acceleration speed & Iq command slope counter value
		   Generate_Now_Speed_Command(&Motor1);//open loop speed ramp up control, generate the u16SpeedCommandNow value

			 if(Motor1.command.u8MotorReverseCommand==1)//20180807
			 {Motor1.command.s16OpenLoopNowIq = OPEN_LOOP_RAMP_DOWM_TARGET_Iq;}//20180807
			 else
			 {Generate_Open_Loop_Now_Iq_Command(&Motor1);}//generate the s16OpenLoopNowIq for open loop Iq command ramp up control

			 Motor1.command.s16IqCommand = -Motor1.command.s16OpenLoopNowIq;
		 }
    else if(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)//for speed and Iq commmand ramp up control in close loop
		{
			Generate_Close_loop_Slope_Counter_Value(&Motor1);//20190626
			//Generate_Now_Speed_Command(&Motor1);//close loop speed ramp up control, get the u16SpeedCommandNow value//20190626
            Generate_Now_Speed_Command_Close_Loop(&Motor1);
		}
    #ifdef ENABLE_NEW_STARTUP_METHOD
		else if(Motor1.command.u8MotorRunMode == 	RUN_IN_NEW_STARTUP_MODE)//20190626
		{
			 Generate_New_Startup_Now_Iq_Command(&Motor1);//generate the s16New_Startup_NowIq for new startup mode Iq command ramp up control
			 Motor1.command.s16IqCommand = Motor1.NewStartup.s16New_Startup_NowIq;
		 }
		#endif

//		#ifdef ENABLE_ROTOR_IPD2_FUNCTION//20190626
//		if(Motor1.IPD.u16RIPD2PWMCounterTotal >0){Motor1.IPD.u16RIPD2PWMCounterTotal = Motor1.IPD.u16RIPD2PWMCounterTotal-1;}
//		#endif

		#ifdef ENABLE_ROTOR_IPD1_FUNCTION
		if(Motor1.IPD.u16DecTimer_1ms >0){Motor1.IPD.u16DecTimer_1ms = Motor1.IPD.u16DecTimer_1ms-1;}

		/* for generate Iq command in rotor small swing mode*/
		if(Motor1.command.u8MotorRunMode == ROTOR_SMALL_SWING_MODE)
		{
			Generate_Rotor_Small_Swing_Sin_Table_Address(&Motor1.IPD);//generate the sine table address for the magnitude of Iq command
		  Motor1.command.s16IqCommand = ((int32_t)IPD_Sin(Motor1.IPD.u16TableAddress)*(-(ROTOR_IPD1_TARGET_CURRENT * CURRENT_GAIN)/10))>>15;
		}
		else if(Motor1.command.u8MotorRunMode == STANDSTILL_BEMF_CALIBRATE_MODE)
		{
			if((Motor1.IPD.u16DecTimer_1ms > 30)&&(Motor1.IPD.u16DecTimer_1ms < 43))//generate the average BEMF A,B voltage when motor is in standstill status
			{
			  Motor1.IPD.u16BEMFPhaseAZero12bit = (Motor1.IPD.u16BEMFPhaseAZero12bit + u16ADC1[BEMFA_ADC_CHANNEL])/2;
				Motor1.IPD.u16BEMFPhaseBZero12bit = (Motor1.IPD.u16BEMFPhaseBZero12bit + u16ADC1[BEMFB_ADC_CHANNEL])/2;
			}
		}
		else if(Motor1.command.u8MotorRunMode == DETECT_ROTOR_POSITION_MODE)
		{
			Rotor_IPD_Mode_Process();
		 }
		#endif
	 }
	else if(u16SystemTime1ms>2)//2ms loop, 2ms executed once in this loop
	{
		u16SystemTime1ms -=2;
		u16SystemTime2ms++;

		/* calculate the estimated speed of motor */
		if(Motor1.command.u8MotorRunMode >= RUN_IN_OPEN_LOOP_MODE)//if yes, means in open / close loop mode
		 {
			 #ifdef ENABLE_NEW_STARTUP_METHOD
			 SMO_Kslf_Modulate_By_Speed_Motor1();
			 //SMO_Kslf_Modulate_For_New_Startup_By_Speed_Motor1();//20190626
			 #else
		   SMO_Kslf_Modulate_By_Speed_Motor1();
		   #endif

		 }//Modulate the SMO s32Kslf value by motor speed

		/* motor speed error protection in close loop mode */
		if(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)
		 {
		   Speed_Error_Protect_Motor1();
		   Rotation_Inverse_Error_Protect_Motor1();//20190124
		 }

		#ifdef ENABLE_LACK_PHASE_DETECT_AND_PROTECT//20190626
	  Lack_Phase_Detect_and_Protect_Motor1();//20190626
	  #endif
	 }
	else if(u16SystemTime2ms>5)//10ms loop, 10ms executed once in this loop
	{
		u16SystemTime2ms -=5;
		u16SystemTime10ms++;

		if(Motor1.spec.u16OpenLoopAlignTime_10ms > 0)//20190623
		 { Motor1.spec.u16OpenLoopAlignTime_10ms -=1; }//for open loop rotor align time decrease

		if(Motor1.NewStartup.u16NewStartupTime_10ms_counter  > 0)//20190626
		 { Motor1.NewStartup.u16NewStartupTime_10ms_counter  -=1; }//for new startup time decrease


		#ifdef ENABLE_ROTOR_ALIGN_FINISH_DETECTION
      Rotor_Align_finish_Detect();//detect the rotor alignment is ready or not//20190623
    #endif

		/* generate the Iq command in close loop mode */
		if(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)
		 {
		   Motor1.command.s16IqCommand =  Generate_Iq_Command_IN_Close_Loop_Motor1();
			 #ifdef ENABLE_ISUM_MEASUREMENT
			 Isum_Current_Measurement();//in 10ms time base loop
			 #endif

			 Id_Command_Modulate_By_Speed_Motor1();//20181115
		 }
		else
		{	Motor1.command.s16IdCommand = 0;}//20190626

    VBUS_Voltage_Measurement();//measure DC Bus votalge, unit:10mV //20181215

		#ifdef ENABLE_DC_BUS_VOLTAGE_PROTECTION
    VBUS_Voltage_Protection();//20190515 //DC BUS Over/Under voltage protection
		#endif

	  Motor1.info.u8MiscellaneousADCConverFlag =1;//enable ADC to convert VSP, ISUM, DC BUS Voltage....miscellaneous items
	 }
	else if(u16SystemTime10ms>5)//50ms loop,  50ms executed once in this loop
	{
		u16SystemTime10ms -=5;
		u16SystemTime50ms++;

		#ifdef ENABLE_MAX_POWER_LIMIT//20190605
		if(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)
		 { Max_Power_Consumption_limit();}//use s16SpeedError to control the max power consumption
	  #endif

		#ifdef ENABLE_MAX_CURRENT_LIMIT//20190626
		if(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)
		 { Max_DCBUS_Current_limit();}//use s16SpeedError to control the max dc bus current
	  #endif

    /* receive VSP speed command */
		User_Command_Receive();//get user on/off, speed command

		/* current loop Ki value modulate by motor speed */
		if(Motor1.command.u8MotorRunMode >= RUN_IN_OPEN_LOOP_MODE)
		 {PI_Current_Ki_Modulate_By_Speed_Motor1();}//modulate current PI's Ki value by motor speed
	 }
	else if(u16SystemTime50ms>2)//100ms loop, 100ms executed once in this loop
	{
		u16SystemTime50ms -=2;
		u16SystemTime100ms++;

		#ifdef ENABLE_MOTOR_REVERSIBLE_FUNCTION //20190504
		//getting motor direction command from KEY1 pin input
		if((Motor1.command.u8MotorReverseCommand==0)&&((Motor1.command.u8MotorRunMode == STOP_MODE)||(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE))) //20190502
		{User_Motor_Direction_Command_Receive();}//20190430 get user direction command from key1
		#endif

		#ifdef ENABLE_NEW_STARTUP_METHOD
		New_Startup_Mode_Over_Time_Error_Check();//check new startup of motor is success or fail
		#endif

    if(u8EnterCloseCounter >0){u8EnterCloseCounter = u8EnterCloseCounter-1;}
    if(u8RestartupWaitCounter >0){u8RestartupWaitCounter = u8RestartupWaitCounter-1;}
	 }
	else if(u16SystemTime100ms>5)//500ms loop,  500ms executed once in this loop
	{
		u16SystemTime100ms -=5;
		u16SystemTime500ms++;
    Reset_IWatchdog_in_500ms_Loop();//clear independent Watchdog timer

    #ifdef ENABLE_ERROR_CODE_SHOW_TO_LED
      Error_Code_Show_to_LED();//20190515
    #endif

	 }
//  else if(u16SystemTime500ms>2)//1000ms loop,  1000ms executed once in this loop
//	{
//		u16SystemTime500ms -=2;
//	 }
}

//-----both PWM period and center align will generate interrupts------------------------------------------------------
#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
/* TIM3 UDE Interrupt for updating CC1 CC2 CC3 duty in every half of period */
void TIM3_IRQHandler(void)
{
 if((TIM3->CR1)&0x10)   // TIM3 down-counter status
 {
	/* Write Down conter value */
  TIM1->CCR1 = u32TIM1CC1D;
  TIM1->CCR2 = u32TIM1CC2D;
  TIM1->CCR3 = u32TIM1CC3D;
  u8SvpwmZoneSwtichFlag = 0;
 }
 else // TIM3 upper-counter status
 {
	/* Write upper conter value */
  TIM1->CCR1 = u32TIM1CC1U;
  TIM1->CCR2 = u32TIM1CC2U;
  TIM1->CCR3 = u32TIM1CC3U;
 }

 TIM_ClearFlag(TIM3, TIM_FLAG_Update);
}
#endif

//-----ADC interrupt 62.5us(16KHz) executed once------------------------------------------------------
#ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)//for 2 shunt R only
{
	#ifdef ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION //enable this to do over current protection (PWM OFF), if comparator output Low signal
		Read_Break_Status_and_Processing();//Read PWM break status and processing it
		if((TIM_GetITStatus(TIM1,TIM_IT_Update)))//(TIM1->SR&TIM_IT_Update)//TIM_GetITStatus(TIM1,TIM_IT_Update))
		{ TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);}
	#else
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);	//clear interrupt flag
	#endif
}
#endif

#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
#else
void ADC_COMP_IRQHandler(void)
#endif
{
  static uint8_t u8DoInOddPWMIntFlag, u8DoInEveryPWMIntFlag;
	static uint16_t u16NonInverseSMOAngle;
	//GPIOC->BRR |= GPIO_Pin_14;

//---------------Inverse Odd PWM cycle flag--------------------------------------------------------------------------
	if(u8DoInOddPWMIntFlag !=0){u8DoInOddPWMIntFlag =0;}
	else if(u8DoInOddPWMIntFlag ==0){u8DoInOddPWMIntFlag =1;}

	#ifdef ENABLE_EVERY_2_PWM_CYCLES_DO_ONCE_FOC_SMO_CALCULATTION
		u8DoInEveryPWMIntFlag = 0;//if this flag ==0, it means depend on u8DoInOddPWMIntFlag to do calculation
	#else
	  u8DoInEveryPWMIntFlag = 1;//if this flag ==1, it means do FOC,SMO,and all others in every pwm interrupt
	#endif

//---------------for 1 shunt R to get 3 phase current-----------------------------------------------------------------
	#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT

		/* Disable ADC transfer when interrupt*/
		ADC1->ADCR &= ~ADCR_SWSTART_Set;   // Disable start of ADC1

		#ifdef ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION
			Read_Break_Status_and_Processing();//Read PWM break status and processing it
		#endif

		#ifdef ENABLE_TIM1_CC4_CC5_TO_TRIG_ADC1_FOR_1_SHUNTR//use TIM1_CC4_CC5 to trigger ADC1 in 1 shunt R current sensing
			TIM1->CCR4 = u32OneShuntADCTri0;
			TIM1->CCR5 = u32OneShuntADCTri1;
		#else
			/*Set first trigger point of ADC */
			TIM3->CCR1 = u32OneShuntADCTri0;

			/* Update TIM3_CCR1 by DMA for setting second point of ADC*/
			u32DMAtoT3CC1[0] = u32OneShuntADCTri1;
			u32DMAtoT3CC1[1] = u32OneShuntADCTri1;

			/* Enable DMA of TIM3 */                        //Enable DMA of ADC trigger
			DMA1_Channel4->CPAR=(uint32_t)(&(TIM3->CCR1));	//DMA1 外设地址 , master address
			DMA1_Channel4->CMAR=(u32)u32DMAtoT3CC1; 	      //DMA1,存储器地址,slave address
			DMA1_Channel4->CNDTR=2;                         //DMA1 of channel1 buffer
			DMA1_Channel4->CCR |= 0x1;                      //DMA1_CH4 enable
		#endif

		/* Disable DMA of ADC */
		ADC1->ADSTA |= ADC_FLAG_EOC;       // clear flag
		ADC1->ADCR &= ~ADCR_DMA_Set;       // Disable DMA of adc
		ADC1->ADCR &= ADCR_EXTTRIG_Reset;  // Disable External Trigger
		DMA1_Channel1->CCR &= ~(0x1);      // DMA1_CH1 Disable


		if((u8DoInOddPWMIntFlag ==0)||u8DoInEveryPWMIntFlag)//do this in even PWM cycle
		{Reconstructed_3_Phase_Current_for_1_Shunt_R();}  //Read DMA data buffer and Reconstructed them to 3 phase current at 1 shunt R current sensing

	#endif

//--------------------for 2 shunt R to get 3 phase current-----------------------------------------------------------------
	#ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
		/* Clear ADC Flag of Conversion */
		//ADC_ClearFlag(ADC1,ADC_IT_EOC);
		ADC1->ADSTA |= ADC_IT_EOC;
		/* Disable ADC Intterupt */
		//ADC_ITConfig(ADC1,ADC_IT_EOC,DISABLE);
		ADC1->ADCR &= (~(uint32_t)ADC_IT_EOC);
		/* Disable HW Trigger */
		//ADC_ExternalTrigConvCmd(ADC1,DISABLE);
		ADC1->ADCR &= ADCR_EXTTRIG_Reset;

		/*get the 2 phase current */
		u16ADC1[Ia_ADC_CHANNEL] = (uint16_t)PHASEA_CURR_ADC_DATA_REGISTER;// Read ADC register for phase A Sensing current
		u16ADC1[Ib_ADC_CHANNEL] = (uint16_t)PHASEB_CURR_ADC_DATA_REGISTER;// Read ADC register for phase B Sensing current
		#ifdef ENABLE_3_SHUNT_R_CURRENT_MEASURE
			u16ADC1[Ic_ADC_CHANNEL] = (uint16_t)PHASEC_CURR_ADC_DATA_REGISTER;// Read ADC register for phase C Sensing current
		#endif

		/* get s16Ia, s16Ib, Ic current */
		Motor1.FOC.s16Ia = -(int16_t)(u16ADC1[Ia_ADC_CHANNEL] -CurrentSensing.s16IaZero )*16;//(-32767~32767)

		#ifdef ENABLE_3_SHUNT_R_CURRENT_MEASURE//20190626
		 	if(Motor1.command.u8MotorDirection==1)//motor direction command is CW
			{
				Motor1.FOC.s16Ib = -(int16_t)(u16ADC1[Ib_ADC_CHANNEL] -CurrentSensing.s16IbZero )*16;//(-32767~32767)
				Motor1.FOC.s16Ic = -(int16_t)(u16ADC1[Ic_ADC_CHANNEL] -CurrentSensing.s16IcZero )*16;//(-32767~32767)
			}
			else//motor direction command is CCW
			{
				Motor1.FOC.s16Ic = -(int16_t)(u16ADC1[Ib_ADC_CHANNEL] -CurrentSensing.s16IbZero )*16;//(-32767~32767)
				Motor1.FOC.s16Ib = -(int16_t)(u16ADC1[Ic_ADC_CHANNEL] -CurrentSensing.s16IcZero )*16;//(-32767~32767)
			}
		#else
			if(Motor1.command.u8MotorDirection==1)//motor direction command is CW
			{
				Motor1.FOC.s16Ib = -(int16_t)(u16ADC1[Ib_ADC_CHANNEL] -CurrentSensing.s16IbZero )*16;//(-32767~32767)
				Motor1.FOC.s16Ic = -(Motor1.FOC.s16Ia + Motor1.FOC.s16Ib);//(-32767~32767)
			}
			else//motor direction command is CCW
			{
				Motor1.FOC.s16Ic = -(int16_t)(u16ADC1[Ib_ADC_CHANNEL] -CurrentSensing.s16IbZero )*16;//(-32767~32767)
				Motor1.FOC.s16Ib = -(Motor1.FOC.s16Ia + Motor1.FOC.s16Ic);//(-32767~32767)
			}
		#endif
	#endif

//-----------------------Do BEMF A,B voltage and other micellaneous ADC transfer (VSP speed cmd,DC Bus,ISum)------------------
	if((u8DoInOddPWMIntFlag ==1)||u8DoInEveryPWMIntFlag)//do this in Odd PWM cycle
	{
		/*get 2 phase BEMF voltage */
		#ifdef ENABLE_ROTOR_IPD1_FUNCTION
			if((Motor1.command.u8MotorRunMode == CHECK_BEMF_MODE)||(Motor1.command.u8MotorRunMode ==DETECT_ROTOR_POSITION_MODE)||(Motor1.command.u8MotorRunMode ==STANDSTILL_BEMF_CALIBRATE_MODE))
		#else
			if(Motor1.command.u8MotorRunMode == CHECK_BEMF_MODE)
		#endif
			{
				ADC1_Channel_Setup_Add_BEMF_AB();//change ADC channels to BEMF voltage measurement
				/*ADCR寄存器的ADST位使能，软件启动转换*/
				ADC1->ADCR |= ADCR_SWSTART_Set;
				/* End conversion */
				while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==0);
				/* Clear Flag */
				ADC1->ADSTA |= ADC_FLAG_EOC;

				u16ADC1[BEMFA_ADC_CHANNEL] = (uint16_t)BEMFA_ADC_DATA_REGISTER;// Read ADC register for BEMF A voltage
				u16ADC1[BEMFB_ADC_CHANNEL] = (uint16_t)BEMFB_ADC_DATA_REGISTER;// Read ADC register for BEMF B voltage
			}
			else if(Motor1.info.u8MiscellaneousADCConverFlag ==1)
			{
				ADC1_Channel_Setup_Without_Phase_Current();//change ADC transfer channels after PWM output off
				/*ADCR寄存器的ADST位使能，软件启动转换*/
				ADC1->ADCR |= ADCR_SWSTART_Set;
				/* End conversion */
				while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==0);
				/* Clear Flag */
				ADC1->ADSTA |= ADC_FLAG_EOC;

				Motor1.info.u8MiscellaneousADCConverFlag =0;//this flag will set to 1 every 2ms in 2ms loop.
			}
	}
//------------------1 shunt R DMA1 ADC transfter result setup and do OP amplifier zero current calibration-----------------
	#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
		/* do the ADC zero current calibration  */
		if(CurrentSensing.bZeroCurrentCalibrationStatusFlag == DO_ZERO_CURRENT_CALIBRATION)
			{
				ADC1_Channel_Setup_to_1ShuntR_Current_Only();/* Enable CH in 1 Shunt R Hardware ADC */
				ADC1->ADCR |= ADCR_SWSTART_Set;/*ADCR寄存器的ADST位使能，软件启动转换*/
				while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==0);/* End conversion */
				ADC1->ADSTA |= ADC_FLAG_EOC;	/* Clear Flag */

				ADC_Zero_Current_Calibration(&CurrentSensing, 2048, 2048, 2048,(uint16_t)ADC_CHANNEL_1_SHUNT_R_REGISTER);
			}

		/* Enable HW Trigger for 1-shunt R current sample*/
		ADC1->ADCR |= ADCR_EXTTRIG_Set ;

		/* Enable ADC CH in 1 Shunt R Hardware ADC */
		ADC1_Channel_Setup_to_1ShuntR_Current_Only();

		/* Enable DMA for ADC to get 1-shunt R current sampling value*/
		ADC1->ADCR |= ADCR_DMA_Set;                  //Enable DMA of ADC trigger
		DMA1_Channel1->CPAR=(u32)(&(ADC_CHANNEL_1_SHUNT_R_REGISTER)); //DMA1 外设地址 ,master address
		DMA1_Channel1->CMAR=(u32)s32DMAtoADDR5; 	   //DMA1,存储器地址, slave address
		DMA1_Channel1->CNDTR=2;                      //DMA1 of channel1 buffer
		DMA1_Channel1->CCR |= 0x1;                   //DMA1_CH1 enable
	#endif

//---------------Do 2 shunt R current sensing OP Amplifier zero current calibration-------------------------------------------
  #ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
		/* do the ADC zero current calibration  */
		if(CurrentSensing.bZeroCurrentCalibrationStatusFlag == DO_ZERO_CURRENT_CALIBRATION)
			{
			 #ifdef ENABLE_ISUM_MEASUREMENT
				#ifdef ENABLE_3_SHUNT_R_CURRENT_MEASURE //20190626
				  ADC1_Channel_Setup_to_3Phase_Current_and_Isum_Only();//for 3 shunt R current measurement
				#else
					ADC1_Channel_Setup_to_2Phase_Current_and_Isum_Only();//for 2 shunt R current measurement
				#endif
			  ADC1->ADCR |= ADCR_SWSTART_Set;/*ADCR寄存器的ADST位使能，软件启动转换*/
			  while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==0);/* End conversion */
			  ADC1->ADSTA |= ADC_FLAG_EOC;	/* Clear Flag */
			 #endif

			 #ifdef ENABLE_3_SHUNT_R_CURRENT_MEASURE //20190626
				  ADC_Zero_Current_Calibration(&CurrentSensing,(uint16_t)PHASEA_CURR_ADC_DATA_REGISTER,(uint16_t)PHASEB_CURR_ADC_DATA_REGISTER,(uint16_t)PHASEC_CURR_ADC_DATA_REGISTER,(uint16_t)ISUM_ADC_DATA_REGISTER);
			 #else
					ADC_Zero_Current_Calibration(&CurrentSensing,(uint16_t)PHASEA_CURR_ADC_DATA_REGISTER,(uint16_t)PHASEB_CURR_ADC_DATA_REGISTER,2048,(uint16_t)ISUM_ADC_DATA_REGISTER);
			 #endif
			}
	#endif

//----------------------Process motor1 angle command for FOC calculation---------------------------------------------------------
  if (Motor1.command.u8MotorRunMode >= RUN_IN_OPEN_LOOP_MODE)//if yes, means in open/close loop mode,and need provide angle to FOC
	{
//-----------------FOC calculation------------------------------------------------------------------------------------------------
		if((u8DoInOddPWMIntFlag ==0)||u8DoInEveryPWMIntFlag)//do this in Even PWM cycle
		{
			Generate_Angle_For_FOC_Calculation();//generate angle (Motor1.command.u16Angle) for FOC calculation

			/* sin cos lock up table */
			s16CosQ15 = Cos(Motor1.command.u16Angle);//angle range: 0~1023
			s16SinQ15 = Sin(Motor1.command.u16Angle);//angle range: 0~1023

			/* ABC to DQ to alpha beta */
			FOC_Coordinate_Transformation(&Motor1.FOC, s16CosQ15, s16SinQ15);//generate Iafa , Ibeta, Id, Iq,Valpha, Vbeta

			/*calculate the Vd, Vq value */
			Motor1.info.s16IdError = Motor1.command.s16IdCommand - Motor1.FOC.s16Id;
			Motor1.info.s16IqError = Motor1.command.s16IqCommand - Motor1.FOC.s16Iq;

			Motor1.FOC.s16Vd = PI_control_current(Motor1.info.s16IdError, &Id_PI);//generate Vd
			Motor1.FOC.s16Vq = PI_control_current(Motor1.info.s16IqError, &Iq_PI);//generate Vq

			/* SVPWM Computation */
			if(Motor1.FOC.s16Vq > 12000){Motor1.FOC.s16Vq = 12000;}
			else if(Motor1.FOC.s16Vq < (-32000)){Motor1.FOC.s16Vq = -32000;}

			if(Motor1.FOC.s16Vd > 12000){Motor1.FOC.s16Vd = 12000;}
			else if(Motor1.FOC.s16Vd < (-12000)){Motor1.FOC.s16Vd = -12000;}

			#ifdef ENABLE_ROTOR_IPD2_FUNCTION
			if(Motor1.command.u8MotorRunMode == INDUCTANCE_SAT_POSITION_DET_MODE)//if yes, means in IPD2 mode (by inductance saturation effect)
			{Rotor_IPD2_Mode_Process();}
			#endif

			SVPWM(&Motor1.FOC);
		}
//-----------------over current protection by software-------------------------------------------------------------------------
		if((u8DoInOddPWMIntFlag ==1)||u8DoInEveryPWMIntFlag)//do this in Odd PWM cycle
	  {
			/* over current protection */
			Software_Over_Current_Protect_Motor1(Motor1.FOC.s16Ia);//check phase current s16Ia is over current or not
			Software_Over_Current_Protect_Motor1(Motor1.FOC.s16Ib);//check phase current Ib is over current or not

//-----------------Motor angle estimation by SMO-------------------------------------------------------------------------------
			#ifdef I_DIVIDE_4_TO_SMO
				Motor1.smo.s32IalphaToSMO= Motor1.FOC.s16Ialpha >>2; // FOC parameter for SMO calculation//20190626
				Motor1.smo.s32IbetaToSMO = Motor1.FOC.s16Ibeta  >>2; // FOC parameter for SMO calculation//20190626
			#elif defined I_DIVIDE_2_TO_SMO
			  Motor1.smo.s32IalphaToSMO= Motor1.FOC.s16Ialpha >>1; // FOC parameter for SMO calculation//20190626
				Motor1.smo.s32IbetaToSMO = Motor1.FOC.s16Ibeta  >>1; // FOC parameter for SMO calculation//20190626
			#else
				Motor1.smo.s32IalphaToSMO= Motor1.FOC.s16Ialpha * I_MULTIPLY_TO_SMO;// FOC parameter for SMO calculation//20190626
				Motor1.smo.s32IbetaToSMO = Motor1.FOC.s16Ibeta  * I_MULTIPLY_TO_SMO;// FOC parameter for SMO calculation//20190626
			#endif

			Motor1.smo.s32ValphaToSMO= Motor1.FOC.s16Valpha;       			 // FOC parameter for SMO calculation//20190530
			Motor1.smo.s32VbetaToSMO = Motor1.FOC.s16Vbeta;        			 // FOC parameter for SMO calculation//20190530

			/* Sliding Mode Observer */
			#ifdef ENABLE_HARDWARE_DIVIDER
			Motor1.smo.s16Theta = SMO_Position_Calc_HDIV(&Motor1.smo, &Motor1.FOC);//Use Hardware divider, generate SMO estimation theta (0~32767)
			#else
			Motor1.smo.s16Theta = SMO_Position_Calc(&Motor1.smo, &Motor1.FOC);//generate SMO estimation theta (0~32767)
			#endif

			Motor1.smo.s16Theta = Under_Over_16_cp(Motor1.smo.s16Theta);


			u16NonInverseSMOAngle = 512 +((Motor1.smo.s16Theta) >> 5);//generate SMO estimation angle (0~1023) for 0~360degree

			if(u16NonInverseSMOAngle > 1023){u16NonInverseSMOAngle = u16NonInverseSMOAngle -1024;}//20190626

			if(Motor1.command.u8MotorRunMode ==RUN_IN_NEW_STARTUP_MODE)
			{
				if((u16NonInverseSMOAngle <1010)&&(u16NonInverseSMOAngle >12))//20190626
				 {
					 if(u16NonInverseSMOAngle > Motor1.smo.s16Angle)
						{ Motor1.smo.s16Angle = u16NonInverseSMOAngle;}
				 }
				else{ Motor1.smo.s16Angle = u16NonInverseSMOAngle;}

				if(Motor1.NewStartup.u8NewStartupState ==FORCE_RE_STARTUP_STATE)//20190626
				 { Motor1.smo.s16Angle = u16NonInverseSMOAngle;}
			 }
			else {Motor1.smo.s16Angle = u16NonInverseSMOAngle;}
	  }
  }

//----------------For 1 shunt R  TIM1 Flag clear-------------------------------------------------------------------------------------
  #ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  #endif
//----------------ADC Setup for 2 shunt R phase current sensing--------------------------------------------------------------------------------------------------------------
	#ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
	/* Enable ADC Intterupt */
  //ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	ADC1->ADCR |= ADC_IT_EOC;
  /* Enable HW Trigger */
  //ADC_ExternalTrigConvCmd(ADC1,ENABLE);
	ADC1->ADCR |= ADCR_EXTTRIG_Set;

	#ifdef ENABLE_3_SHUNT_R_CURRENT_MEASURE
		ADC1_Channel_Setup_to_3Phase_Current_Only();//change ADC channels to 3 phase current measurement, in fact, add Iw current measurement
	#else //for 2 shunt R current measure
		ADC1_Channel_Setup_to_2Phase_Current_Only();//change ADC channels to 2 phase current measurement
	#endif
	#endif
//-------------------------------------------------------------------------------------------------------------------------------

//	#ifdef ENABLE_LACK_PHASE_DETECT_AND_PROTECT//20190626
//	 Lack_Phase_Detect_and_Protect_Motor1();//20190626
//	#endif

		//GPIOC->BSRR |= GPIO_Pin_14;
	#ifdef ENABLE_DAC_SHOW  //for enable DAC output
	//DAC12bit_show(Motor1.command.u16Angle<<2 );
	//DAC12bit_show(((Motor1.FOC.s16Ia*5)/2)+2048 );
	//DAC12bit_show((Motor1.smo.s16Angle<<2) );
	//DAC12bit_show((Motor1.command.s16AngleDiff*10) );
	//DAC12bit_show(Motor1.smo.s16Speed>>2);
	//DAC12bit_show((-Motor1.FOC.s16Vq)>>3);
	//DAC12bit_show(Motor1.BEMF.u16BEMFSpeed <<3);
	//DAC12bit_show((-Motor1.command.s16IqCommand)>>3);
	//DAC12bit_show((-Motor1.FOC.s16Iq)>>3);
	//DAC12bit_show((Motor1.IPD.s16Angle)*10);
	//DAC12bit_show((Motor1.IPD.s16BEMFPhaseB12bit));
	//DAC12bit_show((Motor1.FOC.s16Ia/16)+2048);
	//DAC12bit_show((Motor1.IPD.u16RIPD2MaxIdAngle<<2) );
	//DAC12bit_show( (-Motor1.FOC.s16Id >>3));
	//DAC12bit_show(Motor1.IPD.s32RIPD2IdSumMax>>6);
	//DAC12bit_show(Motor1.info.u32PowerConsumption*100);
	//DAC12bit_show((u8SvpwmZoneSwitch-1)*800 );
	//DAC12bit_show((Motor1.command.u16SpeedCommandNow>>2));
	//DAC12bit_show((Motor1.command.s16OpenLoopTargetIq>>2) );
	//DAC12bit_show((s32DMAtoADDR5[1]&0x00000FFF) );
	//DAC12bit_show((Motor1.info.u16VBusVoltage));
	//DAC12bit_show((Motor1.info.u16VBusVoltageAverage));
	//DAC12bit_show(Motor1.smo.u16RotationInverseCounter*8 );
	//DAC12bit_show(u16ADC1[BEMFB_ADC_CHANNEL] );
	//DAC12bit_show(Motor1.command.u16CommandTheta>>3 );
	//DAC12bit_show((Motor1.IPD.s32RIPD2IdSum)>>6);
	//DAC12bit_show((Motor1.IPD.u16RIPD2MaxIdAngle)<<2);
//	DAC12bit_show((Motor1.FOC.s16Ib/16)+2048);
	//DAC12bit_show((Motor1.command.u16SpeedCommandNow));
	DAC12bit_show((Motor1.command.u8MotorRunMode*400));
	//DAC12bit_show((Motor1.command.u16SpeedTarget>>2));
	#endif

}

void HardFault_Handler(void)
{
	 NVIC_SystemReset();//复位 //software reset
}

/********************************************************************************************************
**function name        ：main(void)
**function description ：
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
int main(void)
{
    systick_init();
    #ifdef ENABLE_LED_SHOW //20190530
    LED_Init();
    #endif
    #ifdef ENABLE_DAC_SHOW  //for enable DAC output
    IO_SPI_Init();//for DAC output use only
    #endif
    #ifdef ENABLE_HARDWARE_DIVIDER
    HDIV_Init();//enable hardware division with signed
    #endif
    /* Assign divider code can load to SRAM and execution it in 96MHz speed*/
    #ifndef ENABLE_HARDWARE_DIVIDER
    flashCode_to_ram(Division,SRAM_RUN_LEN,(int32_t)SRAM_RUN_CODE_ADD);//Copy sub function to assignment zone of sram
    #endif
    delay_ms(200);
    // PWR_DeInit();       //De-initializes the PWR peripheral registers to their default reset values
    // PVD_EXTI_Init();    //PVD外部中初始化
    Function_Init(&Motor1);
    Initial_Motor_Parameter(&Motor1);
    /*Initialize the ADC1*/
	ADC1_Initial();
    ADC1_Channel_Setup_Without_Phase_Current();
	#ifdef ENABLE_OVER_CURRENT_TIM1BKIN_PROTECTION //enable this to do over current protection (PWM OFF), if ext/internal comparator output high signal
	#ifdef ENABLE_TIM1BKIN_PIN_EXTERNAL_INPUT
	TIM1_BKIN_External_Input_Pin_Init(); 	//this break external input pin must initial setup before the TIM1 initial setup
	#endif
    #endif
	#ifdef ENABLE_OVER_CURRENT_COMP1_PROTECTION
	Init_Comparator();//initial comparator1 to do over current protection
	#endif

	TIM1_PWM_Init(PWM_PERIOD,0,DEAD_TIME_SETUP); //PWM initialize
	#ifndef ENABLE_TIM1_CC4_TO_TRIG_ADC_FOR_2_SHUNTR
	TIM3_PWM_Init(PWM_PERIOD,0); //TIM3 initialize, for PWM trigger ADC function to convert 2 phase current
	#endif

    #ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
    #ifndef ENABLE_TIM1_CC4_CC5_TO_TRIG_ADC1_FOR_1_SHUNTR
    /* Intialize and Enable DMA for TIM3*/
    DMA_Timer3CC1TrigDMA_init();
    #endif
    /* Initialize DMA for ADC*/
    DMA_ADCTrigDMA_init();
    #endif
    /*配置为LSI32分频,计数器初始值为0x7ff,复位时长约为1.6s*/
    Write_Iwdg_ON(IWDG_Prescaler_256,0x3ff);//3ff : 6 seconds to reset

    while(1)              //无限循环
    {
        //-------for reset watchdog timer ---------------------------------------------------
        Reset_IWatchdog_in_main_Loop();
        //---time management, it has 100us,1ms,2ms,10ms,50ms,100ms,500ms,1000ms time loop----
        System_Time_Management();
        //---motor1 running state control------------------------------------------------------
        if(Motor1.command.u8MotorRunMode == STOP_MODE)//if yes, motor real status is in stop mode
        {
            if((Motor1.command.u8MotorStart==1)&&(Motor1.info.u8ErrorCounter<MAX_ERROR_ACCUMULATIVE_TOTAL))//if yes, means user want startup the motor
            {
                if(u8RestartupWaitCounter ==0)//if an error has happened, need wait(ERROR_RESTART_WAIT_TIME), then can restart motor
                {
                    CurrentSensing.bZeroCurrentCalibrationStatusFlag = DO_ZERO_CURRENT_CALIBRATION;//do the OP Amplifier zero current output voltage measurement
                    Motor1.command.u8MotorRunMode = ZERO_CURRENT_CALIBRATE_MODE;//set motor to check BEMF status
                }
            }
        }
        switch(Motor1.command.u8MotorRunMode)//motor run mode handle
        {
            case STOP_MODE:         //stop the pwm output
                Disable_Motor1_PWM_Output();    //disable pwm pins output
                Motor1.command.u8MotorReverseCommand= 0;//20190514//clear to 0 to cancel reverse run command
                break;
            case ZERO_CURRENT_CALIBRATE_MODE://do current sensing OP output calibration at zero current status
                if(CurrentSensing.bZeroCurrentCalibrationStatusFlag == ZERO_CURRENT_CALIBRATION_DONE)//if yes, means zero current calibration has already done
                {
                    s32OffsetCurrent = CurrentSensing.s16IsumZero;//zero calibration value for 1 shunt R & Isum measurement
                    Initial_Motor_Parameter(&Motor1);//initial motor parameters
                    delay_us(100);  //20190402
                    #ifdef ENABLE_WL_ON_AT_CHECK_BEMF_MODE
                    WL_PIN_TURN_ON_MOSFET();//WL pin change to I/O mode and turn on MOSFET
                    delay_us(100);  //20190402
                    #endif
                    Motor1.command.u8MotorRunMode = CHECK_BEMF_MODE;//set motor to check BEMF mode before startup motor
                    #ifdef ENABLE_OPEN_TO_CLOSE_ADVANCE_GRADUALLY
                    u8OpenEnterCloseProcess = 0;//start to open loop enter close loop process
                    #endif
                }
            break;
            case CHECK_BEMF_MODE:	//Before startup motor, get the motor status by checking BEMF and go to next state
            if(Motor1.BEMF.bBEMFResultValidFlag==1)//if yes, means BEMF check has already done and make a next state decision
            {
                #ifdef ENABLE_WL_ON_AT_CHECK_BEMF_MODE
                SWITCH_WL_PIN_TO_PWM_OUTPUT();//WL pin change to PWM mode, but not enable PWM signal output to pin
                delay_us(50);//20181016
                #endif
                if(Motor1.command.u8MotorStart==0)//if yes, means user want stop the motor
                { Motor1.command.u8MotorRunMode = STOP_MODE;}
                else if((Motor1.BEMF.bBEMFMotorIsRotatingFlag==0)||(Motor1.BEMF.u16BEMFSpeed <= 3))//20190514//if yes, means motor is in standstill status
                { Standstill_BEMF_Process();}//make decision when BEMF check result is in standstill status
                else if(Motor1.command.u8MotorDirection==Motor1.BEMF.u8BEMFDirectionFlag)//if yes, means BEMF Direction is same as user command
                {
                    DownWind_BEMF_Process();//make decisions when BEMF check result is in Downwind status
                    Start_The_Motor1();//enable PWM output	and current ADC channels setup
                }
                else if(Motor1.command.u8MotorDirection!=Motor1.BEMF.u8BEMFDirectionFlag)//if yes, means in against wind status,BEMF Direction is reverse to user command
                {
                    Motor1.command.u8MotorReverseCommand = 1;//1:means motor need do a reverse action
                    AgainstWind_BEMF_Process();//make decisions when BEMF check result is in againstwind status
                    Start_The_Motor1();//enable PWM output	and current ADC channels setup
                }
            }
            break;
                #ifdef ENABLE_ROTOR_IPD2_FUNCTION
                case INDUCTANCE_SAT_POSITION_DET_MODE:
                if(Motor1.IPD.u16RIPD2PWMCounterTotal == 0)//check the inductance saturation position detect mode, it's time is up or not
                    { After_IPD2_Process();}//setup the next action after IPD2 has finished
                    break;
                #endif
                #ifdef ENABLE_ROTOR_IPD1_FUNCTION
            case STANDSTILL_BEMF_CALIBRATE_MODE:
                if(Motor1.IPD.u16DecTimer_1ms == 0)//check the standstill bemf calibration mode, it's time is up or not
                { Rotor_Small_Swing_Mode_Setup();}//setup the initial value for next rotor small swing mode
                    break;
            case ROTOR_SMALL_SWING_MODE:
                if(Motor1.IPD.u16DecTimer_1ms == 0)//check rotor small swing mode, it's time is up or not
                { Rotor_Position_Detection_Mode_Setup();}//setup the initial value for next rotor initial position detection mode
                break;
            case DETECT_ROTOR_POSITION_MODE:
                if(Motor1.command.u8MotorStart==0)//if yes, means user want stop the motor
                {
                    SWITCH_WL_PIN_TO_PWM_OUTPUT();//PB15 pin change to PWM mode, but not enable PWM signal output to pin
                    Motor1.command.u8MotorRunMode = STOP_MODE;
                }
                else if(((Motor1.IPD.u16DecTimer_1ms == 0)&&(Motor1.IPD.u8RIPDQualifyFailFlag ==0))||(Motor1.IPD.u16RIPDQualifyFailCounter>=ROTOR_IPD1_MAX_FAIL_TIMES))
                {
                    SWITCH_WL_PIN_TO_PWM_OUTPUT();//PB15 pin change to PWM output mode
                    Motor1.command.u16SpeedCommandNow =Motor1.IPD.s16Speed;//initial speed setup
                    Motor1.command.u16CommandTheta = Motor1.IPD.s16EndTheta;//initial open loop theta setup
                    Motor1.command.s16IqCommand = 0;//initial Iq command to 0
                    Iq_PI.s32IntegralResult =(CURRENT_Ki_MULTIPLY) *(-Motor1.command.s16OpenLoopInitIq);//setup the initial value to the current loop Vq integrator
                    if(Motor1.IPD.u16RIPDQualifyFailCounter>=ROTOR_IPD1_MAX_FAIL_TIMES)
                    {
                        Motor1.command.u16SpeedCommandNow = 0;
                        Open_Loop_Acceleration_Modulate_For_IPD1_Fail();
                    }
                    //-----initial parameters before enter new startup mode ----20190626-----------------------------------------
                    #ifdef ENABLE_NEW_STARTUP_METHOD
                    New_Startup_Setup_After_IPD1();//setup the new startup parameters//20190626
                    New_Startup_Common_Setup();    //setup the new startup common parameters//20190626
                    Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
                    //------------------------------------------------------------------------------------------------------------
                    #else
                    Motor1.command.u8MotorRunMode = RUN_IN_OPEN_LOOP_MODE;//change state to open loop mode	//20190626
                    #endif
                    Start_The_Motor1();//enable PWM output	and current ADC channels setup
                }
                else if(Motor1.IPD.u8RIPDQualifyFailFlag ==1)
                {
                    Motor1.IPD.u16RIPDQualifyFailCounter +=1;//increase fail counter
                    Motor1.IPD.s16RotorSwingAngle = Motor1.IPD.s16RotorSwingAngle + 171;//if fail at poor quality of BEMF A, B, add 60 degree then small swing again.
                    if(Motor1.IPD.s16RotorSwingAngle > 1023){Motor1.IPD.s16RotorSwingAngle = Motor1.IPD.s16RotorSwingAngle -1024;}
                    Rotor_Small_Swing_Mode_Setup();//setup the initial value for next rotor small swing mode
                }
                break;
                #endif
                case RUN_IN_ALIGNMENT_MODE://20190623
                if(Motor1.spec.u16OpenLoopAlignTime_10ms ==0)
                {
                    #ifdef ENABLE_NEW_STARTUP_METHOD
                    #ifdef ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_MODE//20190626
                    New_Startup_Setup_After_Alignment_Mode();//setup the new startup parameters//20190626
                    New_Startup_Common_Setup();    //setup the new startup common parameters//20190626
                    Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
                    #else //it means "ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_AND_OPEN_LOOP_MODE" has enabled
                    Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY) *(-Motor1.command.s16OpenLoopInitIq);//setup the initial value to the current loop Vq integrator
                    Motor1.command.u8MotorRunMode = RUN_IN_OPEN_LOOP_MODE;//change state to open loop mode
                    #endif
                    #else
                    Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY) *(-Motor1.command.s16OpenLoopInitIq);//setup the initial value to the current loop Vq integrator
                    Motor1.command.u8MotorRunMode = RUN_IN_OPEN_LOOP_MODE;//change state to open loop mode
                    #endif
                }
                if(Motor1.command.u8MotorStart==0)//if yes, means user want stop the motor
                { Motor1.command.u8MotorRunMode = STOP_MODE;}
                break;
                #ifdef ENABLE_NEW_STARTUP_METHOD
                case RUN_IN_NEW_STARTUP_MODE://20190626
                if((Motor1.NewStartup.u16NewStartupTime_10ms_counter ==0)&&(Motor1.smo.s16Speed>NEW_STARTUP_TO_CLOSE_LOOP_SPEED))//if yes, it means time up of new startup
                {
                        speed_PI.s32IntegralResult = Motor1.command.s16IqCommand * (SPEED_Ki_MULTIPLY/1);//20190626//for syn.the Iq_command output of speed loop
                    Motor1.command.u16SpeedCommandNow = Motor1.smo.s16Speed + (Motor1.smo.s16Speed/16);
                    Motor1.command.u8MotorRunMode = RUN_IN_CLOSE_LOOP_MODE;//change state to close loop mode//20190626
                    u8OpenEnterCloseProcess = 2;//20190626
                }
                    if(Motor1.command.u8MotorStart==0)//if yes, means user want stop the motor
                    { Motor1.command.u8MotorRunMode = STOP_MODE;}
                break;
                #endif
                case RUN_IN_OPEN_LOOP_MODE:
                    #ifdef ENABLE_Id_INTEGRATOR_TO_0_DURING_OPEN_LOOP	//20190626
                    Id_PI.s32IntegralResult =0;     //the Vd integrator reset to 0 //20190626
                    #endif
                    if(Motor1.command.u8MotorReverseCommand==1)//if yes, means in against wind startup status
                    {
                        if(Motor1.command.u16SpeedCommandNow <= 1)//if yes, means against wind speed is below 1 RPM by reverse action(nearly stopped)
                        { Restart_Setup_After_Motor_Stopped_At_AgainstWind_or_Dir_CMD_Change();}//20190502//Setup parameters to restart the motor when motor has stopped at againstwind status or Direction CMD change
                    }
                    #ifndef FORCE_STAY_IN_OPEN_LOOP
                    #ifdef ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_AND_OPEN_LOOP_MODE
                    else if(Motor1.command.u16SpeedCommandNow >=(OPEN_LOOP_SPEED_TO_ENTER_NEW_STARTUP_MODE))//if yes, means can get into new startup mode
                    {
                        New_Startup_Setup_After_Open_Loop_Mode();//setup the new startup parameters//20190626
                        New_Startup_Common_Setup();    //setup the new startup common parameters//20190626
                        Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
                    }
                    #endif
                    else if(Motor1.command.u16SpeedCommandNow >=(Motor1.spec.u16OpenLoopTargetSpeed-10))//if yes, means can get into close loop
                    {
                        u8EnterCloseCounter = FIX_Iq_COMMAND_TIME + GRADUAL_Iq_COMMAND_TIME;//unit 0.1second, it define a the fix Iq command time when first time into the close loop
                        Motor1.command.u8MotorRunMode = RUN_IN_CLOSE_LOOP_MODE;//change state to close loop mode
                        Iq_PI.s32IntegralResult = Iq_PI.s32IntegralResult/CURRENT_PI_INTEGRAL_RESULT_DIVIDER_WHEN_ENTER_CLOSE_LOOP;//20190123
                        //Id_PI.s32IntegralResult = Id_PI.s32IntegralResult/CURRENT_PI_INTEGRAL_RESULT_DIVIDER_WHEN_ENTER_CLOSE_LOOP;//20190626, Need delete this
                    }
                    #endif
                    if(Motor1.command.u8MotorStart==0)//if yes, means user want stop the motor
                    {Motor1.command.u8MotorRunMode = STOP_MODE;}
                    break;
                case RUN_IN_CLOSE_LOOP_MODE:
                    if(Motor1.command.u8MotorStart==0)//if yes, means user want stop the motor
                    {
                        Motor1.command.u16SpeedTarget = OPEN_LOOP_TARGET_SPEED;//setup close loop target speed, force to slow down the close loop speed
                        if((Motor1.smo.s16Speed<CLOSE_LOOP_SPEED_TO_STOP_PWM)||(Motor1.smo.s16Speed<(OPEN_LOOP_TARGET_SPEED+(OPEN_LOOP_TARGET_SPEED/10))))//20180907//if yes, means speed has slow down, then pwm output can off now
                        {
                            Motor1.command.u8MotorRunMode = STOP_MODE;
                            u8RestartupWaitCounter = RESTART_WAIT_TIME_AFTER_PWM_OFF;
                        }
                    }
                    else if(Motor1.command.u8MotorReverseCommand==1)//if yes,means now is in against wind status,and need to do reverse action
                    {
                        Motor1.command.u16SpeedTarget = OPEN_LOOP_TARGET_SPEED;//force to slow down motor to minimum speed in close loop
                        if((Motor1.smo.s16Speed<(AGAINST_WIND_CLOSE_TO_OPEN_LOOP_SPEED)||(Motor1.smo.s16Speed<(OPEN_LOOP_TARGET_SPEED+(OPEN_LOOP_TARGET_SPEED/10))))&&(u8EnterCloseCounter==0))//20180907
                        {//if yes,means speed has slow down,and can run open loop mode
                            #ifdef ENABLE_NEW_STARTUP_METHOD_FOR_HIGH_SPEED_AGAINST_WIND_STARTUP//20190626
                            Motor1.command.u8MotorReverseCommand= 0;//clear to 0 to cancel reverse run command//20190626
                            if(Motor1.command.u8UserDirectionCommand)//20190626
                            {Motor1.command.u8MotorDirection =  MOTOR_RUN_CW;}//non-reverse the motor direction command
                            else
                            {Motor1.command.u8MotorDirection =  MOTOR_RUN_CCW;}//non-reverse the motor direction command
                            New_Startup_Setup_In_Low_Speed_Against_Wind();//setup the parameters for new startup//20190626
                            New_Startup_Common_Setup();//setup the common parameters for new startup//20190626
                            Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
                            #else//20190626
                        Setup_Open_Loop_Ramp_Down_Speed_Slop_and_Target_Iq(&Motor1);//setup the ramp down speed slop & Iq in againstwind's reverse action
                        Motor1.command.u16SpeedTarget = 0;////Open loop target speed command setup to 0
                        Motor1.command.u16CommandTheta = Motor1.smo.s16Angle <<5;//transfer theta to open loop command theta//20180807
                        Motor1.command.u16SpeedCommandNow =Motor1.smo.s16Speed;//transfer smo speed to now speed command
                        Motor1.command.u8MotorRunMode = RUN_IN_OPEN_LOOP_MODE;//change run mode to open loop mode
                        #endif
                        }
                    }
                    else//VSP Speed command input to u16SpeedTarget, u16SpeedTarget is the target speed command in open & close loop
                    {
                        Motor1.command.u16SpeedTarget = Motor1.command.u16VSPSpeedCommand;//transfer user speed command to motor controller
                        #ifdef ENABLE_NEW_STARTUP_METHOD //20190626
                        #ifdef ENABLE_BACK_TO_NEW_STARTUP_MODE_WHEN_LOW_SPEED//20190626
                        if(Motor1.smo.s16Speed <= BACK_TO_NEW_STARTUP_MODE_SPEED)//if yes, means smo.speed is very slow, need back to new startup mode
                        {
                            New_Startup_Setup_After_Lock_Rotor();//setup the new startup parameters after locked rotor or very low speed//20190626
                            New_Startup_Common_Setup();//setup the new startup comman parameters//20190626
                            Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
                        }
                        #endif
                        #endif
                    }
                    break;
                    default:
                    break;
        }
    }
}
/********************************************************************************************************
**function name        ：Read_Break_Status_and_Processing(void)
**function description ：Read PWM Break status and processing it
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Read_Break_Status_and_Processing(void)
{
    if((TIM_GetITStatus(TIM1,TIM_IT_Break)))//(TIM1->SR&TIM_IT_Break)//(TIM_GetITStatus(TIM1,TIM_IT_Break))
    {
        Stop_The_Motor1();
        if (Motor1.info.u8ErrorCode != MOTOR_OVER_CURRENT)
        {
            Motor1.info.u8ErrorCode = MOTOR_OVER_CURRENT;
            Motor1.info.u8ErrorCounter = MAX_ERROR_ACCUMULATIVE_TOTAL + 1;
        }
        u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;		// ysq+ 2018-06-08 故障解除才开始计数
        TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
    }
}
/********************************************************************************************************
**function name        ：void New_Startup_Mode_Over_Time_Error_Check(void)//20190626
**function description ：check the new startup of motor is pass or fail
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void New_Startup_Mode_Over_Time_Error_Check(void)//check new startup of motor is success or fail//20190626
{
	#ifdef ENABLE_NEW_STARTUP_METHOD
	if(Motor1.command.u8MotorRunMode == RUN_IN_NEW_STARTUP_MODE)
	{
	  Motor1.NewStartup.u16NewStartup100msCounter+=1;//this counter increase 1 every 100ms for checking new startup is fail or not
		if(Motor1.NewStartup.u16NewStartup100msCounter > NEW_STARTUP_MAX_TIME)//if yes, means it is over max. time for new startup mode
	  {
		  Stop_The_Motor1();
			Motor1.info.u8ErrorCode = NEW_STARTUP_MODE_OVER_TIME_ERROR;
      Motor1.info.u8ErrorCounter +=1;
      u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;
	  }
	 }
	else
	{Motor1.NewStartup.u16NewStartup100msCounter =0;}
  #endif
}
/********************************************************************************************************
**function name        ：void New_Startup_Setup_After_IPD1(void)//20190626
**function description ：new startup parameters setup after IPD1 finished
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void New_Startup_Setup_After_IPD1(void)//20190626
{
  #ifdef ENABLE_NEW_STARTUP_METHOD
	Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
	Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
	Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626
	Motor1.command.u16SpeedCommandNow = Motor1.IPD.s16Speed;//initial speed setup//20190626
	Motor1.command.u16CommandTheta = Motor1.IPD.s16EndTheta;//initial rotor new startup theta setup, ANGLE(0~1024) equal to Theta(0~32767)//20190626
	Motor1.command.u16Angle = Motor1.IPD.s16EndTheta>>5;//20190626
	Motor1.NewStartup.s16New_Startup_NowIq =-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
	Motor1.command.s16IqCommand = -(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
	Motor1.smo.s16Speed = Motor1.IPD.s16Speed;//initial speed setup//20190626
  #endif
}
/********************************************************************************************************
**function name        ：void New_Startup_Setup_After_Alignment_Mode(void)//20190626
**function description ：new startup parameters setup after alignment mode finished
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void New_Startup_Setup_After_Alignment_Mode(void)
{
  #ifdef ENABLE_NEW_STARTUP_METHOD
	Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
	Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
	Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626
	Motor1.command.u16SpeedCommandNow = 0;//initial speed setup//20190626
	Motor1.command.u16CommandTheta = OPEN_LOOP_ALIGNMENT_ANGLE*32;//initial rotor new startup theta setup, ANGLE(0~1024) equal to Theta(0~32767)//20190626
	Motor1.command.u16Angle = OPEN_LOOP_ALIGNMENT_ANGLE;//20190626
	Motor1.NewStartup.s16New_Startup_NowIq =-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
	Motor1.command.s16IqCommand = -(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
	Motor1.smo.s16Speed = 0;//initial speed setup//20190626
  #endif
}
/********************************************************************************************************
**function name        ：void New_Startup_Setup_After_Open_Loop_Mode(void)//20190626
**function description ：new startup parameters setup after open loop speed over a defined speed "OPEN_LOOP_SPEED_TO_ENTER_NEW_STARTUP_MODE"
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void New_Startup_Setup_After_Open_Loop_Mode(void)
{
  #ifdef ENABLE_NEW_STARTUP_METHOD
	Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
	Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
	Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626
	Motor1.command.u16SpeedCommandNow = OPEN_LOOP_SPEED_TO_ENTER_NEW_STARTUP_MODE;//initial speed setup//20190626
	//Motor1.command.u16CommandTheta = OPEN_LOOP_ALIGNMENT_ANGLE*32;//initial rotor new startup theta setup, ANGLE(0~1024) equal to Theta(0~32767)//20190626
	//Motor1.command.u16Angle = OPEN_LOOP_ALIGNMENT_ANGLE;//20190626
	Motor1.NewStartup.s16New_Startup_NowIq =-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
	Motor1.command.s16IqCommand = -(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
	Motor1.smo.s16Speed = OPEN_LOOP_SPEED_TO_ENTER_NEW_STARTUP_MODE;//initial speed setup//20190626
  #endif
}

/********************************************************************************************************
**function name        ：void New_Startup_Setup_In_Low_Speed_Against_Wind(void)//20190626
**function description ：new startup parameters setup for low speed against wind startup
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void New_Startup_Setup_In_Low_Speed_Against_Wind(void)//20190626
{
	#ifdef ENABLE_NEW_STARTUP_METHOD
	Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_CURRENT_FOR_AGAINST_WIND * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
	Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
	Motor1.command.u16SpeedCommandNow = 0;//transfer initial speed to now's speed command;
	Motor1.smo.s16Speed = 0;//initial speed setup//20190626
	Motor1.command.u16CommandTheta = 0;//initial rotor new startup theta setup, ANGLE(0~1024) equal to Theta(0~32767)//20190626
  Motor1.NewStartup.s16New_Startup_NowIq =-(NEW_STARTUP_CURRENT_FOR_AGAINST_WIND * CURRENT_GAIN)/10;//20190626
	Motor1.command.s16IqCommand = -(NEW_STARTUP_CURRENT_FOR_AGAINST_WIND * CURRENT_GAIN)/10;//20190626
	Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626
  #endif
}
/********************************************************************************************************
**function name        ：void New_Startup_Setup_After_Lock_Rotor(void)//20190626
**function description ：new startup parameters setup after locked rotor
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void New_Startup_Setup_After_Lock_Rotor(void)//20190626
{
 #ifdef ENABLE_NEW_STARTUP_METHOD
 Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
 Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
 Motor1.NewStartup.s16New_Startup_NowIq =-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
 Motor1.command.s16IqCommand = -(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
 Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626
 #endif
}
/********************************************************************************************************
**function name        ：Generate_Angle_For_FOC_Calculation(void) execute it in every pwm cycle
**function description ：Generate angle for FOC calculation
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Generate_Angle_For_FOC_Calculation(void)
{
	#ifdef ENABLE_OPEN_TO_CLOSE_ADVANCE_GRADUALLY
	static uint8_t u8AccumulatedCount =0;
	#endif

	/* generate Angle, range 0~1023 */
	if (Motor1.command.u8MotorRunMode == RUN_IN_OPEN_LOOP_MODE)//if Yes, means run in open loop
	{
		Generate_Open_Loop_Angle(&Motor1);//generate angle:Motor1.command.u16Angle in open loop mode

		#ifdef ENABLE_OPEN_TO_CLOSE_ADVANCE_GRADUALLY
			Get_Angle_Difference(&Motor1);//get the angle difference between open loop and smo estimate angle
			u8OpenEnterCloseProcess = 1;//enter close loop process step1
		#endif
	}
	else if (Motor1.command.u8MotorRunMode == RUN_IN_ALIGNMENT_MODE)//if Yes, means run in rotor alignment mode //20190623
	{ Motor1.command.u16Angle = OPEN_LOOP_ALIGNMENT_ANGLE;}

	#ifdef ENABLE_NEW_STARTUP_METHOD //20190626
	else if (Motor1.command.u8MotorRunMode == RUN_IN_NEW_STARTUP_MODE)//if Yes, means run in rotor new startup mode //20190626
	{ New_Startup_Mode_Process(&Motor1.NewStartup,&Motor1.smo);}
	#endif
	#ifdef ENABLE_ROTOR_IPD2_FUNCTION
	else if(Motor1.command.u8MotorRunMode == INDUCTANCE_SAT_POSITION_DET_MODE)
	{ Motor1.command.u16Angle = Motor1.IPD.s16RIPD2Angle;}//asign the Angle for initial position detection 2's inductance saturation mode
	#endif
	#ifdef ENABLE_ROTOR_IPD1_FUNCTION
	else if(Motor1.command.u8MotorRunMode == ROTOR_SMALL_SWING_MODE)
	{ Motor1.command.u16Angle = Motor1.IPD.s16RotorSwingAngle;}//asign the Angle for initial position detection's rotor swing mode
	#endif
	#ifdef ENABLE_OPEN_TO_CLOSE_ADVANCE_GRADUALLY
	/*	generate open loop angle and calculate the diff angle between SMO and open loop*/
	else if((Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)&&(u8OpenEnterCloseProcess==1))
	{
		if(Motor1.smo.s16Angle >= Motor1.command.s16AngleDiff)
		{ Motor1.command.u16Angle = Motor1.smo.s16Angle - Motor1.command.s16AngleDiff;}
		else
		{ Motor1.command.u16Angle = (1024+Motor1.smo.s16Angle) - Motor1.command.s16AngleDiff;}

		if(u8AccumulatedCount>0){u8AccumulatedCount = u8AccumulatedCount-1;}

		if((Motor1.command.s16AngleDiff >=1)&&(u8AccumulatedCount ==0))
		{Motor1.command.s16AngleDiff = Motor1.command.s16AngleDiff-1;u8AccumulatedCount = PWM_CYCLES_CLOSE_TO_035_DEGREE;}
		else if(Motor1.command.s16AngleDiff ==0)
		{u8OpenEnterCloseProcess = 2;}//enter close loop process step2, after this, the FOC angle use smo angle
	}
	#endif
	else//if yes, means in close loop
	{Motor1.command.u16Angle = Motor1.smo.s16Angle;}//Angle comes from SMO estimation angle in close loop mode
}

/********************************************************************************************************
**function name        ：Reconstructed_3_Phase_Current_for_1_Shunt_R(void)
**function description ：Read DMA data buffer and Reconstructed them to 3 phase current at 1 shunt R current sensing
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Reconstructed_3_Phase_Current_for_1_Shunt_R(void)
{
	/* Get Phase Current of 1-shunt by SVPWM zone*/
  s32Temp_1st = s32DMAtoADDR5[0]&0x00000FFF;
	s32Temp_2nd = s32DMAtoADDR5[1]&0x00000FFF;

	if(u8SvpwmZoneSwtichFlag)
	{u8SvpwmZoneSwitch = u8SvpwmZoneOld;}
	else
	{u8SvpwmZoneSwitch = u8SvpwmZone;}

  if(Motor1.command.u8MotorDirection==1)
	 {
			switch(u8SvpwmZoneSwitch)
			{
				case 1:
				 s32PhaseCurrentW = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentV = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentU = - s32PhaseCurrentV - s32PhaseCurrentW;
				break;
				case 2:
				 s32PhaseCurrentV = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentU = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentW = - s32PhaseCurrentU - s32PhaseCurrentV;
				break;
				case 3:
				 s32PhaseCurrentW = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentU = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentV = - s32PhaseCurrentU - s32PhaseCurrentW;
				break;
				case 4:
				 s32PhaseCurrentU = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentW = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentV = - s32PhaseCurrentW - s32PhaseCurrentU;
				break;
				case 5:
				 s32PhaseCurrentU = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentV = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentW = - s32PhaseCurrentV - s32PhaseCurrentU;
				break;
				case 6:
				 s32PhaseCurrentV = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentW = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentU = - s32PhaseCurrentW - s32PhaseCurrentV;
				break;
				default:
				break;
			 }
		 }
   else
	  {
		  switch(u8SvpwmZoneSwitch)
			 {
				case 1:
				 s32PhaseCurrentV = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentW = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentU = - s32PhaseCurrentW - s32PhaseCurrentV;
				break;
				case 2:
				 s32PhaseCurrentW = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentU = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentV = - s32PhaseCurrentU - s32PhaseCurrentW;
				break;
				case 3:
				 s32PhaseCurrentV = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentU = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentW = - s32PhaseCurrentU - s32PhaseCurrentV;
				break;
				case 4:
				 s32PhaseCurrentU = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentV = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentW = - s32PhaseCurrentV - s32PhaseCurrentU;
				break;
				case 5:
				 s32PhaseCurrentU = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentW = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentV = - s32PhaseCurrentW - s32PhaseCurrentU;
				break;
				case 6:
				 s32PhaseCurrentW = -(s32Temp_1st - s32OffsetCurrent);
				 s32PhaseCurrentV = s32Temp_2nd - s32OffsetCurrent;
				 s32PhaseCurrentU = - s32PhaseCurrentV - s32PhaseCurrentW;
				break;
				default:
				break;
	    }
	 }


	if(Motor1.command.u16SpeedCommandNow < FORCE_TO_FILTER4_MAX_RPM_1_SHUNT_R)
	{
			Motor1.FOC.s16Ia = ((s32PhaseCurrentU*16) + Motor1.FOC.s16Ia*(3))/4;

			if(Motor1.command.u8MotorDirection==1)//motor direction command is CW
			{
				Motor1.FOC.s16Ib = ((s32PhaseCurrentV*16) + Motor1.FOC.s16Ib*(3))/4;
				Motor1.FOC.s16Ic = ((s32PhaseCurrentW*16) + Motor1.FOC.s16Ic*(3))/4;
			}
			else
			{
				Motor1.FOC.s16Ic = ((s32PhaseCurrentV*16) + Motor1.FOC.s16Ic*(3))/4;
				Motor1.FOC.s16Ib = ((s32PhaseCurrentW*16) + Motor1.FOC.s16Ib*(3))/4;
			}
	}
	else
	{
		Motor1.FOC.s16Ia = ((s32PhaseCurrentU*16) + Motor1.FOC.s16Ia*(PHASE_CURRENT_AVERAGE_WEIGHT-1))/PHASE_CURRENT_AVERAGE_WEIGHT;

		if(Motor1.command.u8MotorDirection==1)//motor direction command is CW
		{
			Motor1.FOC.s16Ib = ((s32PhaseCurrentV*16) + Motor1.FOC.s16Ib*(PHASE_CURRENT_AVERAGE_WEIGHT-1))/PHASE_CURRENT_AVERAGE_WEIGHT;
			Motor1.FOC.s16Ic = ((s32PhaseCurrentW*16) + Motor1.FOC.s16Ic*(PHASE_CURRENT_AVERAGE_WEIGHT-1))/PHASE_CURRENT_AVERAGE_WEIGHT;
		}
		else
		{
			Motor1.FOC.s16Ic = ((s32PhaseCurrentV*16) + Motor1.FOC.s16Ic*(PHASE_CURRENT_AVERAGE_WEIGHT-1))/PHASE_CURRENT_AVERAGE_WEIGHT;
			Motor1.FOC.s16Ib = ((s32PhaseCurrentW*16) + Motor1.FOC.s16Ib*(PHASE_CURRENT_AVERAGE_WEIGHT-1))/PHASE_CURRENT_AVERAGE_WEIGHT;
		}
	}
}
/********************************************************************************************************
**function name        ：Isum_Current_Measurement(void)
**function description ：Measure MOSFET Total current consumption every 2ms
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
#ifdef ENABLE_ISUM_MEASUREMENT
void Isum_Current_Measurement(void)
{
	//(5000mV*100000)/(4096) = 122070
	#define ISUM_GAIN     (122070/(ISUM_R_VALUE*ISUM_CURRENT_AMPLIFICATION_FACTOR))   //parameter for unit : 0.1mA
  Motor1.info.s16IsumInput12bit = (uint16_t)ISUM_ADC_DATA_REGISTER - s32OffsetCurrent;//get Isum 12bit result from ADC register

	/*get total current value, unit:0.1mA*/
	if(Motor1.info.s16IsumInput12bit <0){Motor1.info.s16IsumInput12bit =0;}//minimum current is 0
	Motor1.info.u32IsumCurrent =(uint32_t)Motor1.info.s16IsumInput12bit*ISUM_GAIN;//get total current value, unit:0.1mA

	/*get total current average value, unit:0.1mA*/
	Motor1.info.u32IsumCurrentAverage = ((Motor1.info.u32IsumCurrentAverage*63) + Motor1.info.u32IsumCurrent)/64;//get average total current value, unit:0.1mA

	/*get total power consumption value, unit:0.1W*/
	#ifdef USE_MEASURED_DC_BUS_VOLTAGE_TO_GET_POWER//20190606
	Motor1.info.u32PowerConsumption = (Motor1.info.u32IsumCurrentAverage *Motor1.info.u16VBusVoltageAverage)/100000;//unit:0.1W//20190606
	#else
	Motor1.info.u32PowerConsumption = (Motor1.info.u32IsumCurrentAverage *DC_BUS_VOLTAGE)/10000;//unit:0.1W
	#endif
}
#endif
/********************************************************************************************************
**function name        ：VBUS_Voltage_Measurement(void) //20181215
**function description ：Measure DC Bus voltage every 10ms
**input parameters     ：None
**output parameters    ：Motor1.info.u16VBusVoltageAverage //unit:10mV
********************************************************************************************************/
void VBUS_Voltage_Measurement(void)
{
	//(5000mV/4096)*100 = 122
	#define VBUS_GAIN   ((122*(VBUS_PULL_UP_R + VBUS_PULL_DOWN_R)*1024)/(1000 * VBUS_PULL_DOWN_R))

  Motor1.info.u16VBusInput12bit = (uint16_t)VBUS_ADC_DATA_REGISTER;//get VBus 12bit result from ADC register

	Motor1.info.u16VBusVoltage = ((uint32_t)Motor1.info.u16VBusInput12bit * VBUS_GAIN)>>10;//unit : 10mV

	/*get DC Bus voltage average value, unit:10mV*/
	Motor1.info.u16VBusVoltageAverage = (((uint32_t)Motor1.info.u16VBusVoltageAverage*15) + Motor1.info.u16VBusVoltage)/16;//get average total VBus value, unit:10mV
}
/********************************************************************************************************
**function name        ：VBUS_Voltage_Protection(void) //20190515
**function description ：Check DC Bus voltage every 10ms
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
#ifdef ENABLE_DC_BUS_VOLTAGE_PROTECTION
void VBUS_Voltage_Protection(void)//20190515
{
	 uint8_t  static u8TempOverVoltageFlag =0;

	 if(u8TempOverVoltageFlag ==1)//20190626//add hystersis function
	  {
			if((Motor1.info.u16VBusVoltageAverage>((DC_BUS_OVER_VOLTAGE_LIMITATION -DC_BUS_OVER_VOLTAGE_HYSTERESIS)*10))||(Motor1.info.u16VBusVoltageAverage<((DC_BUS_UNDER_VOLTAGE_LIMITATION+DC_BUS_UNDER_VOLTAGE_HYSTERESIS)*10)))
				{
					Stop_The_Motor1();
					Motor1.info.u8ErrorCode = MOTOR_OVER_UNDER_VOLTAGE_ERROR;
					//Motor1.info.u8ErrorCounter +=1;
					u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;
				}
      else //DC Bus voltage check ok
       {
			   u8TempOverVoltageFlag =0;
			   if(Motor1.info.u8ErrorCode == MOTOR_OVER_UNDER_VOLTAGE_ERROR){Motor1.info.u8ErrorCode = 0;}//20190626//clear the DC BUS error code
			 }
	  }

	 if((Motor1.info.u16VBusVoltageAverage>(DC_BUS_OVER_VOLTAGE_LIMITATION*10))||(Motor1.info.u16VBusVoltageAverage<(DC_BUS_UNDER_VOLTAGE_LIMITATION*10)))
		{
			Stop_The_Motor1();
			Motor1.info.u8ErrorCode = MOTOR_OVER_UNDER_VOLTAGE_ERROR;
      //Motor1.info.u8ErrorCounter +=1;
      u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;
      u8TempOverVoltageFlag =1;//DC Bus fail//20190626//add hystersis function
		}
}
#endif
/********************************************************************************************************
**function name        ：Error_Code_Show_to_LED(void) //20190515//in 500ms timebase loop
**function description ：if there is an error, it will show error code to LED
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
#ifdef ENABLE_ERROR_CODE_SHOW_TO_LED
void Error_Code_Show_to_LED(void)//20190515
{
	 uint8_t  static u8Temp500msCounter =0;//decrease 1 for every 500ms
	 uint8_t  static u8TempKeepLEDOffFlag =0;//0:show error code by LED, 1:off LED for 3 second

	 if(u8Temp500msCounter>0){u8Temp500msCounter--;}

	 if(Motor1.info.u8ErrorCode !=0)
		{
			if(u8TempKeepLEDOffFlag ==1)//if yes, means LED off 3 second to show start bits
			{ LED1_OFF;}//LED off 3 second to show start bits
			else
			{ LED1_TOGGLE();}//show error code to LED

			if(	u8Temp500msCounter ==0)
			{
         if(u8TempKeepLEDOffFlag ==0)
				  {
				    u8Temp500msCounter = 6;//OFF LED for 3 second
					  u8TempKeepLEDOffFlag=1;//for show start bit (use LED OFF 3 second)
				  }
				 else
					{
				    u8Temp500msCounter = Motor1.info.u8ErrorCode*2;//if error code = 1, show LED on 500ms and then LED off 500ms, if error code=2, repeat twice
					  u8TempKeepLEDOffFlag=0;//for show error code led
				  }
      }
		}
   else
	 {
     LED1_OFF;//stop show error code to LED
		 u8Temp500msCounter =0;
		 u8TempKeepLEDOffFlag=0;
	 }
}
#endif
/********************************************************************************************************
**function name        ：User_Motor_Direction_Command_Receive(void)//20190430
**function description ：Receive user direction command,executed this subroutine every 100ms
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
#ifdef ENABLE_MOTOR_REVERSIBLE_FUNCTION //20190504
void User_Motor_Direction_Command_Receive(void)//20190430
{
    uint8_t  static u8TempKeyReleaseFlag;

	  if(KEY0 == 1)//if yes, means key has already released by user
		{
	     u8TempKeyReleaseFlag=1;//it means key has released by user
	  }
	  else if(u8TempKeyReleaseFlag==1)
		{
			 if(KEY0 == 0)
        {
				   if(Motor1.command.u8UserDirectionCommand==1)
						{ Motor1.command.u8UserDirectionCommand=0;}
				   else
						{ Motor1.command.u8UserDirectionCommand=1;}

           if(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)//20190502
					 { Motor1.command.u8MotorReverseCommand = 1;}//change the motor direction

           u8TempKeyReleaseFlag=0;//if set this flag to 0, it will not change the user direction command
				}
		}

		 if(Motor1.command.u8UserDirectionCommand==1)
			{ LED0_ON;}//LED1_ON;}
		 else
			{ LED0_OFF;}//LED1_OFF;;}
}
#endif
/********************************************************************************************************
**function name        ：User_Command_Receive(void)
**function description ：Receive user on/off, speed, direction command,executed this subroutine every 50ms
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void User_Command_Receive(void)
{
		#define VSP_ON_VOLTAGE       7  //unit :0.1V,  7 means 0.7V
		#define VSP_OFF_VOLTAGE      4  //unit :0.1V,  4 means 0.4V

		#define VSP_SPEED1_VOLTAGE  12  //unit :0.1V,  12 means 1.2V
		#define VSP_SPEED2_VOLTAGE  18  //unit :0.1V,  17 means 1.7V
		#define VSP_SPEED3_VOLTAGE  24  //unit :0.1V,  22 means 2.2V
		#define VSP_SPEED4_VOLTAGE  30  //unit :0.1V,  27 means 2.7V
		#define VSP_SPEED5_VOLTAGE  36  //unit :0.1V,  32 means 3.2V
		#define VSP_SPEED6_VOLTAGE  42  //unit :0.1V,  37 means 3.7V

		#define VSP_HYSTERESIS      2   //unit :0.1V,   2 means 0.2V hysteresis voltage for VSP speed command
		#define V01_UNIT_12BIT      82  //0.1V's 12bit ADC value = 4096/50 = 82

		#define VSP7_MIN_VOLT       (VSP_SPEED6_VOLTAGE*V01_UNIT_12BIT)
		#define VSP6_MIN_VOLT       (VSP_SPEED5_VOLTAGE*V01_UNIT_12BIT)
		#define VSP6_MAX_VOLT       ((VSP_SPEED6_VOLTAGE-VSP_HYSTERESIS)*V01_UNIT_12BIT)
		#define VSP5_MIN_VOLT       (VSP_SPEED4_VOLTAGE*V01_UNIT_12BIT)
		#define VSP5_MAX_VOLT       ((VSP_SPEED5_VOLTAGE-VSP_HYSTERESIS)*V01_UNIT_12BIT)
		#define VSP4_MIN_VOLT       (VSP_SPEED3_VOLTAGE*V01_UNIT_12BIT)
		#define VSP4_MAX_VOLT       ((VSP_SPEED4_VOLTAGE-VSP_HYSTERESIS)*V01_UNIT_12BIT)
		#define VSP3_MIN_VOLT       (VSP_SPEED2_VOLTAGE*V01_UNIT_12BIT)
		#define VSP3_MAX_VOLT       ((VSP_SPEED3_VOLTAGE-VSP_HYSTERESIS)*V01_UNIT_12BIT)
		#define VSP2_MIN_VOLT       (VSP_SPEED1_VOLTAGE*V01_UNIT_12BIT)
		#define VSP2_MAX_VOLT       ((VSP_SPEED2_VOLTAGE-VSP_HYSTERESIS)*V01_UNIT_12BIT)
		#define VSP1_MAX_VOLT       ((VSP_SPEED1_VOLTAGE-VSP_HYSTERESIS)*V01_UNIT_12BIT)

		/* get VSP command input voltage */
	  Motor1.info.u16VSPInput = (uint16_t)VSP_ADC_DATA_REGISTER;//get VSP input command voltage from ADC register
		Motor1.info.u16VSPAverage = ((Motor1.info.u16VSPAverage*3) + Motor1.info.u16VSPInput)/4;

		if(Motor1.command.u8MotorStart==0)//non-hystersis
		{
			if(Motor1.info.u16VSPAverage > VSP7_MIN_VOLT)
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_7;}
			else if(Motor1.info.u16VSPAverage > VSP6_MIN_VOLT)
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_6;}
			else if(Motor1.info.u16VSPAverage > VSP5_MIN_VOLT)
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_5;}
			else if(Motor1.info.u16VSPAverage > VSP4_MIN_VOLT)
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_4;}
			else if(Motor1.info.u16VSPAverage > VSP3_MIN_VOLT)
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_3;}
			else if(Motor1.info.u16VSPAverage > VSP2_MIN_VOLT)
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_2;}
			else
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_1;}
	  }
		else//applied with hystersis function
		{
			if(Motor1.info.u16VSPAverage > VSP7_MIN_VOLT)
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_7;}
			else if((Motor1.info.u16VSPAverage > VSP6_MIN_VOLT)&&(Motor1.info.u16VSPAverage < VSP6_MAX_VOLT))
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_6;}
			else if((Motor1.info.u16VSPAverage > VSP5_MIN_VOLT)&&(Motor1.info.u16VSPAverage < VSP5_MAX_VOLT))
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_5;}
			else if((Motor1.info.u16VSPAverage > VSP4_MIN_VOLT)&&(Motor1.info.u16VSPAverage < VSP4_MAX_VOLT))
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_4;}
			else if((Motor1.info.u16VSPAverage > VSP3_MIN_VOLT)&&(Motor1.info.u16VSPAverage < VSP3_MAX_VOLT))
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_3;}
			else if((Motor1.info.u16VSPAverage > VSP2_MIN_VOLT)&&(Motor1.info.u16VSPAverage < VSP2_MAX_VOLT))
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_2;}
			else if(Motor1.info.u16VSPAverage < VSP1_MAX_VOLT)
			 {Motor1.command.u16VSPSpeedCommand = TARGET_SPEED_1;}
		}

		if(Motor1.info.u16VSPAverage > (VSP_ON_VOLTAGE*V01_UNIT_12BIT))
		{
			 Motor1.command.u8MotorStart=1;//user command : 1: user want startup the motor
		}
		else if(Motor1.info.u16VSPAverage < (VSP_OFF_VOLTAGE*V01_UNIT_12BIT))
		{
			 Motor1.command.u8MotorStart=0;//user command : 0 means user want stop the motor
			 Motor1.info.u8ErrorCounter=0;//clear the error counter
		}
}
/********************************************************************************************************
**function name        ：void Standstill_BEMF_Process(void)
**function description ：Made the decision when motor BEMF check result is in standstill status
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Standstill_BEMF_Process(void)
{
	 Motor1.command.u16SpeedCommandNow =0;//initial speed setup to 0
	 Motor1.command.s16IqCommand = 0;//initial Iq command to 0

	 #ifdef ENABLE_ROTOR_IPD2_FUNCTION 	//for inductance saturation initial position detection
		  Motor1.IPD.u16RIPD2PWMCounterTotal = 2+ ((PWM_NUM_FOR_INJECT_VOLT + PWM_NUM_FOR_NON_INJECT_VOLT)*14);//it is the total time to detect motor initial position//20190626
	    Motor1.IPD.s16RIPD2Angle = 0;//initial IPD2 angle to 0
	    Motor1.IPD.u16RIPD2PWMCounter = PWM_NUM_FOR_NON_INJECT_VOLT;//initial IPD2 PWM counter
	    Motor1.IPD.u8RIPD2InjectFlag = 0;//initial IPD2 to non-injection (Vd =0)
      Motor1.IPD.u16RIPD2AddressCounter =0;//IPD2 angle Address index clear to 0
	    Motor1.IPD.s32RIPD2IdSum =0;//initial the Id sum value
	    Motor1.IPD.s32RIPD2IdSumMax =0;//initial the max Id sum value
	    Motor1.IPD.u16RIPD2MaxIdAngle =0;//initial the max Id angle 0~1023 for 0~360degree
      Motor1.command.u8MotorRunMode = INDUCTANCE_SAT_POSITION_DET_MODE;//change state to do IPD by inductance saturation effect
	    Start_The_Motor1();//enable PWM output	and current ADC channels setup

   #elif defined ENABLE_ROTOR_IPD1_FUNCTION //for special initial position detection
			WL_PIN_TURN_ON_MOSFET();//WL pin change to I/O mode and turn on WL side MOSFET
			Motor1.IPD.s16RotorSwingAngle = 0;//initial the rotor swing angle to 0, (0~1023)
			Motor1.IPD.u16DecTimer_1ms = 50;//minimum is 50ms, it is the time to measure the Standstill bemf u,v voltage for calibration
			Motor1.IPD.u16BEMFPhaseAZero12bit = DEFAULT_ZERO_BEMF_VOLTAGE * 80;//define the default value of standstill bemf A
			Motor1.IPD.u16BEMFPhaseBZero12bit = DEFAULT_ZERO_BEMF_VOLTAGE * 80;//define the default value of standstill bemf B
			Motor1.command.u8MotorRunMode = STANDSTILL_BEMF_CALIBRATE_MODE;//change state to standstill bemf u,v voltage calibration mode

	 #elif defined ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_MODE//20190626
	    Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/DIVIDER_OF_ALIGN_INITIAL_CURRENT) *(-(OPEN_LOOP_ALIGNMENT_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
			Motor1.command.u16SpeedCommandNow = 0;//initial speed setup//20190623
			Motor1.command.u16CommandTheta = OPEN_LOOP_ALIGNMENT_ANGLE*32;//initial rotor alignment theta setup//20190623
			Motor1.command.s16IqCommand = -(OPEN_LOOP_ALIGNMENT_CURRENT * CURRENT_GAIN)/10;//20190623
			Motor1.spec.u16OpenLoopAlignTime_10ms = OPEN_LOOP_ALIGNMENT_TIME;//unit:10ms //20190623
			Motor1.info.u32IUSumInAlign =0;//initial to zero,IU absolute value sumption for alignment is ready or not judgement
			Motor1.info.u32IUSumCounterInAlign =0;	//initial to zero,IU absolute value counter for alignment is ready or not judgement
	    Motor1.smo.s16Speed = 0;//initial speed setup//20190626
			Motor1.command.u8MotorRunMode = RUN_IN_ALIGNMENT_MODE;//change to motor alignment mode //20190623
			Start_The_Motor1();//enable PWM output	and current ADC channels setup

   #elif defined ENABLE_NEW_STARTUP_METHOD_AFTER_ALIGNMENT_AND_OPEN_LOOP_MODE//20190626
	    Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/DIVIDER_OF_ALIGN_INITIAL_CURRENT) *(-(OPEN_LOOP_ALIGNMENT_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
			Motor1.command.u16SpeedCommandNow = 0;//initial speed setup//20190623
			Motor1.command.u16CommandTheta = OPEN_LOOP_ALIGNMENT_ANGLE*32;//initial rotor alignment theta setup//20190623
			Motor1.command.s16IqCommand = -(OPEN_LOOP_ALIGNMENT_CURRENT * CURRENT_GAIN)/10;//20190623
			Motor1.spec.u16OpenLoopAlignTime_10ms = OPEN_LOOP_ALIGNMENT_TIME;//unit:10ms //20190623
			Motor1.info.u32IUSumInAlign =0;//initial to zero,IU absolute value sumption for alignment is ready or not judgement
			Motor1.info.u32IUSumCounterInAlign =0;	//initial to zero,IU absolute value counter for alignment is ready or not judgement
	    Motor1.smo.s16Speed = 0;//initial speed setup//20190626
			Motor1.command.u8MotorRunMode = RUN_IN_ALIGNMENT_MODE;//change to motor alignment mode //20190623
			Start_The_Motor1();//enable PWM output	and current ADC channels setup

	 #elif defined ENABLE_NEW_STARTUP_METHOD //for new startup method used in motor startup
			Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
			Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
			Motor1.command.u16SpeedCommandNow = 0;//initial speed setup//20190626
			Motor1.smo.s16Speed = 0;//initial speed setup//20190626
			Motor1.command.u16CommandTheta = 0;//initial rotor new startup theta setup, ANGLE(0~1024) equal to Theta(0~32767)//20190626
			Motor1.NewStartup.s16New_Startup_NowIq =-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
			Motor1.command.s16IqCommand = -(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
			Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626
			Motor1.command.u8MotorRunMode = RUN_IN_NEW_STARTUP_MODE;//20190626
			New_Startup_Common_Setup();//setup the new startup//20190626
			Start_The_Motor1();//enable PWM output	and current ADC channels setup

		#else
			Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/DIVIDER_OF_ALIGN_INITIAL_CURRENT) *(-(OPEN_LOOP_ALIGNMENT_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
			Motor1.command.u16SpeedCommandNow = 0;//initial speed setup//20190623
			Motor1.command.u16CommandTheta = OPEN_LOOP_ALIGNMENT_ANGLE*32;//initial rotor alignment theta setup//20190623
			Motor1.command.s16IqCommand = -(OPEN_LOOP_ALIGNMENT_CURRENT * CURRENT_GAIN)/10;//20190623
			Motor1.spec.u16OpenLoopAlignTime_10ms = OPEN_LOOP_ALIGNMENT_TIME;//unit:10ms //20190623
			Motor1.info.u32IUSumInAlign =0;//initial to zero,IU absolute value sumption for alignment is ready or not judgement
			Motor1.info.u32IUSumCounterInAlign =0;	//initial to zero,IU absolute value counter for alignment is ready or not judgement
	    Motor1.smo.s16Speed = 0;//initial speed setup//20190626
			Motor1.command.u8MotorRunMode = RUN_IN_ALIGNMENT_MODE;//change to motor alignment mode //20190623
			Start_The_Motor1();//enable PWM output	and current ADC channels setup
		#endif
}
/********************************************************************************************************
**function name        ：void After_IPD2_Process(void)
**function description ：Made the decision when motor BEMF check result is in standstill status
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void After_IPD2_Process(void)
{
	 Motor1.command.u16SpeedCommandNow =0;//initial speed setup to 0
	 Motor1.command.s16IqCommand = 0;//initial Iq command to 0
	 Motor1.FOC.s16Vd = 0;//initial Vd
	 Disable_Motor1_PWM_Output();//disable pwm pins output

	 #ifdef ENABLE_ROTOR_IPD1_FUNCTION
		 WL_PIN_TURN_ON_MOSFET();//WL pin change to I/O mode and turn on WL side MOSFET
		 Motor1.IPD.s16RotorSwingAngle = Motor1.IPD.u16RIPD2MaxIdAngle;//initial the rotor swing angle to 0~1023
		 Motor1.IPD.u16DecTimer_1ms = 50;//minimum is 50ms, it is the time to measure the Standstill bemf u,v voltage for calibration
		 Motor1.IPD.u16BEMFPhaseAZero12bit = DEFAULT_ZERO_BEMF_VOLTAGE * 80;//define the default value of standstill bemf A
		 Motor1.IPD.u16BEMFPhaseBZero12bit = DEFAULT_ZERO_BEMF_VOLTAGE * 80;//define the default value of standstill bemf B
		 Motor1.command.u8MotorRunMode = STANDSTILL_BEMF_CALIBRATE_MODE;//change state to standstill bemf u,v voltage calibration mode

   #elif defined ENABLE_NEW_STARTUP_METHOD	//20190626
	   Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190626
		 Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
		 Motor1.command.u16SpeedCommandNow = 0;//initial speed setup//20190626
		 Motor1.smo.s16Speed = 0;//initial speed setup//20190626

	   Motor1.command.u16Angle = Motor1.IPD.u16RIPD2MaxIdAngle + 85;//phase lead 30degree //20190626
	   if(Motor1.command.u16Angle > 1023){Motor1.command.u16Angle = Motor1.command.u16Angle -1024;}

		 Motor1.command.u16CommandTheta = Motor1.IPD.u16RIPD2MaxIdAngle <<5;//initial open loop theta setup
	   Motor1.command.u16CommandTheta = Motor1.command.u16CommandTheta + 2730;//2730=phase lead 30 degree,8192 =90degree
	   if(Motor1.command.u16CommandTheta > 32767){Motor1.command.u16CommandTheta = Motor1.command.u16CommandTheta -32768;}

		 Motor1.NewStartup.s16New_Startup_NowIq =-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
		 Motor1.command.s16IqCommand = -(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
		 Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626
		 Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
		 New_Startup_Common_Setup();//setup the new startup//20190626
		 Start_The_Motor1();//enable PWM output	and current ADC channels setup
	 #else
	   Motor1.command.u16SpeedCommandNow = 0;//initial speed setup
		 Motor1.command.u16CommandTheta = Motor1.IPD.u16RIPD2MaxIdAngle <<5;//initial open loop theta setup

	   Motor1.command.u16CommandTheta = Motor1.command.u16CommandTheta + 2730;//2730=phase lead 30 degree,8192 =90degree
	   if(Motor1.command.u16CommandTheta > 32767){Motor1.command.u16CommandTheta = Motor1.command.u16CommandTheta -32768;}

		 Iq_PI.s32IntegralResult =(CURRENT_Ki_MULTIPLY) *(-Motor1.command.s16OpenLoopInitIq);//setup the initial value to the current loop Vq integrator
		 Motor1.command.u8MotorRunMode = RUN_IN_OPEN_LOOP_MODE;//change state to open loop mode
		 Start_The_Motor1();//enable PWM output	and current ADC channels setup
	 #endif
}
/********************************************************************************************************
**function name        ：void DownWind_BEMF_Process(void)
**function description ：Made the decision when motor BEMF check result is in downwind status
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void DownWind_BEMF_Process(void)
{
	#ifdef ENABLE_NEW_STARTUP_METHOD_FOR_DOWNWIND_STARTUP//20190626
		 Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
		 Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
		 Motor1.command.u16SpeedCommandNow = Motor1.BEMF.u16BEMFSpeed;//transfer initial speed to now's speed command;
		 Motor1.smo.s16Speed = Motor1.BEMF.u16BEMFSpeed;//initial speed setup//20190626
		 Motor1.command.u16CommandTheta = 0;//initial rotor new startup theta setup, ANGLE(0~1024) equal to Theta(0~32767)//20190626
	   Motor1.NewStartup.s16New_Startup_NowIq =-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
		 Motor1.command.s16IqCommand = -(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10;//20190626
		 Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626

	   New_Startup_Common_Setup();//setup the new startup//20190626
		 Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
	#else
	if(Motor1.BEMF.u16BEMFSpeed>DIRECT_ENTER_CLOSE_LOOP_MIN_SPEED)//if yes, means bemf speed is enough to enter close loop directly
	 {
		 #ifdef ENABLE_OPEN_TO_CLOSE_ADVANCE_GRADUALLY
		 u8OpenEnterCloseProcess = 2;//enter close loop process step2, after this, the FOC angle use smo angle
		 #endif

		 u8EnterCloseCounter = 0;//FIX_Iq_COMMAND_TIME + GRADUAL_Iq_COMMAND_TIME;//unit 0.1second, it define a the fix Iq command time when first time into the close loop

		 /*give a Vq initial value for motor startup speed above a DIRECT_ENTER_CLOSE_LOOP_MIN_SPEED value*/
		 if(Motor1.BEMF.u16BEMFSpeed > TARGET_MAX_SPEED){Motor1.BEMF.u16BEMFSpeed = TARGET_MAX_SPEED;}
		 Iq_PI.s32IntegralResult =-(((int32_t)Motor1.BEMF.u16BEMFSpeed*CURRENT_LIMIT_Iq)/TARGET_MAX_SPEED)*(CURRENT_Ki_MULTIPLY+K_STARTUP_Vq_INITIAL_PARA);//setup the initial value to the current loop Vq integrator

		 Motor1.command.u8MotorRunMode = RUN_IN_CLOSE_LOOP_MODE;//change state to close loop mode
		 Motor1.command.u16SpeedCommandNow =Motor1.BEMF.u16BEMFSpeed;//transfer initial speed to now's speed command
	 }
	else//if yes, means need enter open loop directly
	 {
		 Motor1.spec.u16SpeedSlope1 = OPEN_LOOP_RAMP_UP_SPEED_SLOP1/DOWN_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA;//open loop speed up slop setup//20180807;
		 Motor1.spec.u16SpeedSlope2 = OPEN_LOOP_RAMP_UP_SPEED_SLOP2/DOWN_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA;//open loop speed up slop setup//20180807;
		 Motor1.spec.u16SpeedSlope3 = OPEN_LOOP_RAMP_UP_SPEED_SLOP3/DOWN_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA;//open loop speed up slop setup//20180807;
		 Motor1.spec.u16OpenLoopTargetSpeed = DOWNWIND_OPEN_LOOP_TARGET_SPEED;
		 Motor1.command.u16SpeedTarget =DOWNWIND_OPEN_LOOP_TARGET_SPEED;

		 //Motor1.command.u16CommandTheta = 8192;//20190404//16384;//it means initial angle is 180 degree (0~360)
		 if(Motor1.command.u8UserDirectionCommand)//20190430
		 {Motor1.command.u16CommandTheta = 8192;}//20190502//if def 16384,it means initial angle is 180 degree (0~360)
	   else
		 {Motor1.command.u16CommandTheta = 16384;}//20190502//if def 16384,it means initial angle is 180 degree (0~360)

		 Motor1.command.u16SpeedCommandNow =Motor1.BEMF.u16BEMFSpeed;//transfer initial speed to now's speed command
		 Motor1.command.u8MotorRunMode = RUN_IN_OPEN_LOOP_MODE;//change state to open loop mode
     Iq_PI.s32IntegralResult =(CURRENT_Ki_MULTIPLY/2) *(-Motor1.command.s16OpenLoopTargetIq);//setup the initial value to the current loop Vq integrator
		}
   #endif
}
/********************************************************************************************************
**function name        ：void AgainstWind_BEMF_Process(void)
**function description ：Made the decision when motor BEMF check result is in againstWindwind status
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void AgainstWind_BEMF_Process(void)
{
	 if(Motor1.command.u8UserDirectionCommand)//20190430
		{Motor1.command.u8MotorDirection =  MOTOR_RUN_CCW;}//reverse the motor direction command
	 else
		{Motor1.command.u8MotorDirection =  MOTOR_RUN_CW;}//reverse the motor direction command

	 if(Motor1.BEMF.u16BEMFSpeed>DIRECT_ENTER_CLOSE_LOOP_MIN_SPEED)//if yes, means bemf speed is enough to enter close loop directly
	 {
	   #ifdef ENABLE_OPEN_TO_CLOSE_ADVANCE_GRADUALLY
		 u8OpenEnterCloseProcess = 2;//enter close loop process step2, after this, the FOC angle use smo angle
		 #endif

		 /*force into close loop and keep a certain time for motor stable*/
		 u8EnterCloseCounter = FIX_Iq_COMMAND_TIME + GRADUAL_Iq_COMMAND_TIME;//unit 0.1second, it define a the fix Iq command time when first time into the close loop//20180807
		 if(u8EnterCloseCounter < 5){u8EnterCloseCounter = 5;}//setup the minimum value.//20180807

		 /*give a Vq initial value for motor startup speed above a DIRECT_ENTER_CLOSE_LOOP_MIN_SPEED value*/
		 if(Motor1.BEMF.u16BEMFSpeed > TARGET_MAX_SPEED){Motor1.BEMF.u16BEMFSpeed = TARGET_MAX_SPEED;}
		 Iq_PI.s32IntegralResult =-(((int32_t)Motor1.BEMF.u16BEMFSpeed*CURRENT_LIMIT_Iq)/TARGET_MAX_SPEED)*(CURRENT_Ki_MULTIPLY+K_STARTUP_Vq_INITIAL_PARA);//setup the initial value to the current loop Vq integrator

		 Motor1.command.u8MotorRunMode = RUN_IN_CLOSE_LOOP_MODE;//change state to open loop mode
		 Motor1.command.u16SpeedCommandNow =Motor1.BEMF.u16BEMFSpeed;//transfer initial speed to speed command
	 }
	 #ifdef ENABLE_NEW_STARTUP_METHOD_FOR_LOW_SPEED_AGAINST_WIND_STARTUP//20190626
	 if(Motor1.BEMF.u16BEMFSpeed <= DIRECT_ENTER_CLOSE_LOOP_MIN_SPEED)//if yes, means bemf speed is NOT enough to enter close loop directly
	 {
		 Motor1.command.u8MotorReverseCommand= 0;//clear to 0 to cancel reverse run command//20190626
		 if(Motor1.command.u8UserDirectionCommand)//20190626
			{Motor1.command.u8MotorDirection =  MOTOR_RUN_CW;}//non-reverse the motor direction command
	   else
			{Motor1.command.u8MotorDirection =  MOTOR_RUN_CCW;}//non-reverse the motor direction command

	   Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_CURRENT_FOR_AGAINST_WIND * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
		 Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
		 Motor1.command.u16SpeedCommandNow = 0;//transfer initial speed to now's speed command;
		 Motor1.smo.s16Speed = 0;//initial speed setup//20190626
		 Motor1.command.u16CommandTheta = 0;//initial rotor new startup theta setup, ANGLE(0~1024) equal to Theta(0~32767)//20190626
     Motor1.NewStartup.s16New_Startup_NowIq = -(NEW_STARTUP_CURRENT_FOR_AGAINST_WIND * CURRENT_GAIN)/10;//20190626
		 Motor1.command.s16IqCommand = -(NEW_STARTUP_CURRENT_FOR_AGAINST_WIND * CURRENT_GAIN)/10;//20190626
		 Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626

     New_Startup_Common_Setup();//setup the new startup//20190626
		 Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
	 }
	#else
	 else//if yes, means BEMF's speed is not enough into close loop, so it need enter open loop directly and then speed down to 0
	 {
		 Setup_Open_Loop_Ramp_Down_Speed_Slop_and_Target_Iq(&Motor1);//setup open loop speed ramp down slop and target Iq command	in reverse action
		 Motor1.command.u16SpeedTarget = 0;//open loop target speed set to 0, force open loop speed down to 0
		 Motor1.command.u16CommandTheta = 16384;//20190404//0;//open loop theta set to 0
		 Motor1.command.u16SpeedCommandNow =Motor1.BEMF.u16BEMFSpeed;//transfer BEMF speed to initial speed command
		 Motor1.command.u8MotorRunMode = RUN_IN_OPEN_LOOP_MODE;//change state to open loop mode
     Iq_PI.s32IntegralResult =(((CURRENT_Ki_MULTIPLY/2)* Motor1.BEMF.u16BEMFSpeed * K_AGAINST_WIND_OPL)/OPEN_LOOP_TARGET_SPEED)*(-Motor1.command.s16OpenLoopTargetIq);//20180807//setup the initial value to the current loop Vq integrator
		}
	 #endif
}
/********************************************************************************************************
**function name        ：void Restart_Setup_After_Motor_Stopped_At_AgainstWind_or_Dir_CMD_Change(void)
**function description ：Setup parameters for restart the motor when motor has stopped by reverse action at againstwind status
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Restart_Setup_After_Motor_Stopped_At_AgainstWind_or_Dir_CMD_Change(void)//20190502
{
	 Motor1.command.u8MotorReverseCommand= 0;//clear to 0 to cancel reverse run command
	 Motor1.command.u8MotorDirection = Motor1.command.u8UserDirectionCommand;//Motor direction command back to user direction command//20190430

	 #ifdef ENABLE_NEW_STARTUP_METHOD_FOR_AGAINST_WIND_STARTUP_AFTER_STOPPED //20190626
	   Iq_PI.s32IntegralResult = (CURRENT_Ki_MULTIPLY/(1)) *(-(NEW_STARTUP_INITIAL_CURRENT * CURRENT_GAIN)/10);//setup the initial value to the current loop Vq integrator//20190623
		 Motor1.NewStartup.u8NewStartupState =INITIAL_NEW_STARTUP_STATE;//20190626
		 Motor1.command.u16SpeedCommandNow = 0;//transfer initial speed to now's speed command;
		 Motor1.smo.s16Speed = 0;//initial speed setup//20190626
		 Motor1.command.u16CommandTheta = 0;//initial rotor new startup theta setup, ANGLE(0~1024) equal to Theta(0~32767)//20190626
	   Motor1.NewStartup.s16New_Startup_NowIq = -(NEW_STARTUP_CURRENT_FOR_AGAINST_WIND * CURRENT_GAIN)/10;//20190626
		 Motor1.command.s16IqCommand = -(NEW_STARTUP_CURRENT_FOR_AGAINST_WIND * CURRENT_GAIN)/10;//20190626
		 Motor1.NewStartup.u16NewStartupTime_10ms_counter = Motor1.NewStartup.u16NewStartupTime_10ms_total;//unit:10ms //20190626

	   New_Startup_Common_Setup();//setup the new startup//20190626
	   Motor1.command.u8MotorRunMode = 	RUN_IN_NEW_STARTUP_MODE;//20190626
	 #else
		 Motor1.spec.u16SpeedSlope1 = OPEN_LOOP_RAMP_UP_SPEED_SLOP1/AGAINST_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA;//open loop speed up slop1 setup//20180807
		 Motor1.spec.u16SpeedSlope2 = OPEN_LOOP_RAMP_UP_SPEED_SLOP2/AGAINST_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA;//open loop speed up slop2 setup//20180807
		 Motor1.spec.u16SpeedSlope3 = OPEN_LOOP_RAMP_UP_SPEED_SLOP3/AGAINST_WIND_RESTART_RAMP_UP_SLOP_DIVIDE_PARA;//open loop speed up slop3 setup//20180807

		 if(OPEN_LOOP_RAMP_UP_TARGET_Iq > 30000)
		 {Motor1.command.s16OpenLoopTargetIq = 30000;}
		 else
		 {Motor1.command.s16OpenLoopTargetIq = OPEN_LOOP_RAMP_UP_TARGET_Iq;}//open loop speed up target Iq command setup

		 //20190408
		 Iq_PI.s32IntegralResult = (Iq_PI.s32IntegralResult >>2)+(CURRENT_Ki_MULTIPLY/RESTARTUP_INITIAL_Iq_DIVIDE_PARA) *(-Motor1.command.s16OpenLoopInitIq);//setup the initial value to the current loop Vq integrator//20180807

		 Motor1.spec.u16OpenLoopTargetSpeed = AGAINST_WIND_OPEN_TO_CLOSE_TARGET_SPEED; //20180807
		 Motor1.command.u16SpeedTarget = 	AGAINST_WIND_OPEN_TO_CLOSE_TARGET_SPEED;//Open loop target speed command setup//20180907
		 Motor1.command.u16CommandTheta = Motor1.command.u16CommandTheta + (91*PHASE_LEAD_ANGLE_IN_AGAINST_WIND_RESTART);//inverse direction and phase lead 45 degree//20190408
		 if(Motor1.command.u16CommandTheta > 32767){Motor1.command.u16CommandTheta = Motor1.command.u16CommandTheta -32768;}//20190408
	 #endif
}
/********************************************************************************************************
**function name        ：Rotor_Small_Swing_Mode_Setup(void)
**function description ：setup the initial value for rotor small swing
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Rotor_Small_Swing_Mode_Setup(void)
{
	 Motor1.IPD.u8RIPDQualifyFailFlag = 0;//clear the RIPD bemf u,v signal quality flag
	 Motor1.IPD.u8MotorDIRCommand = Motor1.command.u8UserDirectionCommand;//20190430
	 Motor1.command.u16SpeedCommandNow =0;//initial speed setup to 0
	 Motor1.command.s16IqCommand = 0;//initial Iq command to 0
	 Iq_PI.s32IntegralResult =(CURRENT_Ki_MULTIPLY/4) *(-(ROTOR_IPD1_TARGET_CURRENT * CURRENT_GAIN)/10);
	 Motor1.IPD.u16DecTimer_1ms = ROTOR_IPD1_SMALL_SWING_TIME;//setup the torque current time for rotor small swing
	 Motor1.IPD.u16TableAddress =0;
	 SWITCH_WL_PIN_TO_PWM_OUTPUT();//PB15 pin change to PWM output mode
	 delay_us(200);//20180907
	 Motor1.command.u8MotorRunMode = ROTOR_SMALL_SWING_MODE;//change state to rotor small swing mode
	 Start_The_Motor1();//enable PWM output	and current ADC channels setup
}
/********************************************************************************************************
**function name        ：Rotor_Position_Detection_Mode_Setup(void)
**function description ：setup the initial value for rotor initial position detection
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Rotor_Position_Detection_Mode_Setup(void)
{
	 Disable_Motor1_PWM_Output();//disable pwm pins output
	 Motor1.IPD.u16DecTimer_1ms =ROTOR_IPD1_DETECT_TIME;
	 Motor1.command.u8MotorRunMode = DETECT_ROTOR_POSITION_MODE;
   Motor1.IPD.u8RIPDQualifyFailFlag = 0;//initial the Rotor IPD qualify flag
   delay_us(200);//20180907
	 WL_PIN_TURN_ON_MOSFET();//WL pin change to I/O mode and turn on MOSFET
}
/********************************************************************************************************
**function name        ：Reset_IWatchdog_in_main_Loop(void)
**function description ：clear independent Watchdog timer in main loop
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Reset_IWatchdog_in_main_Loop(void)
{
    if(u16IWDGPingPong == 0x5555){Write_Iwdg_RL();u16IWDGPingPong = 0xaaaa;}//clear independent Watchdog timer
}
/********************************************************************************************************
**function name        ：Reset_IWatchdog_in_500ms_Loop(void)
**function description ：clear independent Watchdog timer in System_Time_Management() every 500ms loop
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Reset_IWatchdog_in_500ms_Loop(void)
{
  if(u16IWDGPingPong == 0xaaaa){Write_Iwdg_RL();u16IWDGPingPong = 0x5555;}//clear independent Watchdog timer
}
/********************************************************************************************************
**function name        ：int16_t Generate_Iq_Command_IN_Close_Loop_Motor1(void)
**function description ：generate Iq command in close loop
**input parameters     ：None
**output parameters    ：Iq command
********************************************************************************************************/
int16_t Generate_Iq_Command_IN_Close_Loop_Motor1(void)
{
		int32_t s32TempIqCommand;
	  static int32_t s32Gradual_IqCommand;

	  /*calculate the Iq command in speed loop*/
		Motor1.info.s16SpeedError = ((int32_t)(Motor1.smo.s16Speed - Motor1.command.u16SpeedCommandNow)*32768)/TARGET_MAX_SPEED;

//	  #ifdef ENABLE_MAX_POWER_LIMIT//20190605
//	  Max_Power_Consumption_limit();//use s16SpeedError to control the max power consumption
//	  #endif

		s32TempIqCommand = PI_control_speed(Motor1.info.s16SpeedError, &speed_PI);

	  /*phase current limit*/
	  if(s32TempIqCommand > (LIMIT_MINIMUM_Iq_COMMAND))
     { s32TempIqCommand = LIMIT_MINIMUM_Iq_COMMAND;}//limit the mini.Iq command.
	  else if(s32TempIqCommand < (-CURRENT_LIMIT_Iq))
		 { s32TempIqCommand =  (-(CURRENT_LIMIT_Iq));}//limit the max Iq command, Iq command is a negative number normally

    /*smoothing the beginning Iq command of close loop. (Smoothing from FIX_Iq_COMMAND to normal speed PI generate Iq command)*/
	  if((u8EnterCloseCounter >0)&&(u8EnterCloseCounter<= GRADUAL_Iq_COMMAND_TIME))//if yes, means smoothing the Iq command
		{
			if(s32Gradual_IqCommand < s32TempIqCommand)
			{ s32Gradual_IqCommand = s32Gradual_IqCommand + (FIX_Iq_COMMAND/16);}//for smoothing the Iq command
			else
      { s32Gradual_IqCommand = s32Gradual_IqCommand - (FIX_Iq_COMMAND/16);}//for smoothing the Iq command

			s32TempIqCommand = s32Gradual_IqCommand;
		 }
		else if(u8EnterCloseCounter >0)//if yes, means use the fixed Iq command
    {
			s32TempIqCommand = -FIX_Iq_COMMAND;
			s32Gradual_IqCommand = -FIX_Iq_COMMAND;
      Motor1.command.u16SpeedCommandNow = Motor1.smo.s16Speed + (Motor1.command.u16VSPSpeedCommand>>4);
		}
    return 	s32TempIqCommand;
}
/********************************************************************************************************
**function name        ：Rotor_Align_finish_Detect(void) , in 10ms timebase loop //20190623
**function description ：detect the rotor alignment is finished or not
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
#ifdef ENABLE_ROTOR_ALIGN_FINISH_DETECTION
void Rotor_Align_finish_Detect(void) //20190623
{
	if(Motor1.command.u8MotorRunMode == RUN_IN_ALIGNMENT_MODE)//20190623
		{
			if(Motor1.FOC.s16Ia <0)
			{
			  Motor1.info.u32IUSumInAlign += (-Motor1.FOC.s16Ia);
				//if detected a IU Pulse, reset the align IU sum accumulation
			  if(((-Motor1.FOC.s16Ia)>>4) > (ALIGNMENT_READY_THRESHOLD*1))
				{
					Motor1.info.u32IUSumCounterInAlign =0;
					Motor1.info.u32IUSumInAlign =0;
				}
			}
			else
			{
			  Motor1.info.u32IUSumInAlign += Motor1.FOC.s16Ia;
				//if detected a IU Pulse, reset the align IU sum accumulation
			  if(((-Motor1.FOC.s16Ia)>>4) > (ALIGNMENT_READY_THRESHOLD*1))
				{
					Motor1.info.u32IUSumCounterInAlign =0;
					Motor1.info.u32IUSumInAlign =0;
				}
			}

			if(Motor1.info.u32IUSumCounterInAlign >63)//63 means (63+1)*10ms =640ms
			{
				//check the rotor alignment is finished or not
				if((Motor1.info.u32IUSumInAlign>>10) < (ALIGNMENT_READY_THRESHOLD))
				{ Motor1.spec.u16OpenLoopAlignTime_10ms =0;}//it means alignment is finished.

				Motor1.info.u32IUSumCounterInAlign =0;
			}
      Motor1.info.u32IUSumCounterInAlign +=1;
		}

}
#endif
/********************************************************************************************************
**function name        ：Max_Power_Consumption_limit(void) , in 50ms timebase loop
**function description ：limit the power consumption
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
#ifdef ENABLE_MAX_POWER_LIMIT
void Max_Power_Consumption_limit(void)
{
	   uint16_t u16Temp;//20190605
     uint16_t static u16TempUnlimitCounter =0;//it user target speed command has changed, use this counter does unlimit max power control for a while//20190606
	   uint32_t static u32TempTargetSpeed;//20190606

	   //detect target speed is changed or not, if changed, it will not limit power for defined X*50ms loop time
	   if(u32TempTargetSpeed > (Motor1.command.u16SpeedTarget + (TARGET_MAX_SPEED/20)))
		 { u16TempUnlimitCounter = UNDO_POWER_LIMIT_TIME_IF_TARGET_SPEED_CHANGED;}
		 else if(u32TempTargetSpeed < (Motor1.command.u16SpeedTarget - (TARGET_MAX_SPEED/20)))
	   { u16TempUnlimitCounter = UNDO_POWER_LIMIT_TIME_IF_TARGET_SPEED_CHANGED;}


		 if(Motor1.info.u32PowerConsumption > MAX_POWER_LIMIT)
			 {
					if((Motor1.command.u16SpeedCommandNow > OPEN_LOOP_TARGET_SPEED)&&(u16TempUnlimitCounter==0))
						{
							u16Temp = (Motor1.info.u32PowerConsumption - MAX_POWER_LIMIT)/MIN_POWER_LIMIT_RESOLUTION;//if MIN_POWER_LIMIT_RESOLUTION=10, means 1.0W as minimum power limit resolution

							if(u16Temp > SPEED_CMD_DEC_RPM_FOR_POWER_LIMIT){u16Temp = SPEED_CMD_DEC_RPM_FOR_POWER_LIMIT;}

							Motor1.command.u16SpeedCommandNow -= u16Temp;//decrease speed command for over power limitation
							Motor1.command.u8OverPowerLimitFlag = 1;//set flag to 1,it means power is over limitation
						}
					else {Motor1.command.u8OverPowerLimitFlag = 0;}//because speed command can not be decrease any more, so free this flag
			 }
	   else{Motor1.command.u8OverPowerLimitFlag = 0;}//set flag to 0, it means power is under limitation


		 u32TempTargetSpeed = Motor1.command.u16SpeedTarget;//keep the last target speed

		 if(u16TempUnlimitCounter >0){u16TempUnlimitCounter -=1;}
}
#endif

/********************************************************************************************************
**function name        ：Max_DCBUS_Current_limit(void) , in 50ms timebase loop //20190626
**function description ：limit the power consumption
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
#ifdef ENABLE_MAX_CURRENT_LIMIT
void Max_DCBUS_Current_limit(void) //20190626
{
	   uint16_t u16Temp;//20190605
     uint16_t static u16TempUnlimitCounter =0;//it user target speed command has changed, use this counter does unlimit max power control for a while//20190606
	   uint32_t static u32TempTargetSpeed;//20190606

	   //detect target speed is changed or not, if changed, it will not limit power for defined X*50ms loop time
	   if(u32TempTargetSpeed > (Motor1.command.u16SpeedTarget + (TARGET_MAX_SPEED/20)))
		 { u16TempUnlimitCounter = UNDO_POWER_LIMIT_TIME_IF_TARGET_SPEED_CHANGED;}
		 else if(u32TempTargetSpeed < (Motor1.command.u16SpeedTarget - (TARGET_MAX_SPEED/20)))
	   { u16TempUnlimitCounter = UNDO_POWER_LIMIT_TIME_IF_TARGET_SPEED_CHANGED;}


		 if((Motor1.info.u32IsumCurrentAverage/10) > MAX_DCBUS_CURRENT_LIMIT)//Motor1.info.u32IsumCurrentAverage, it means average total current value, unit:0.1mA
			 {
					if((Motor1.command.u16SpeedCommandNow > OPEN_LOOP_TARGET_SPEED)&&(u16TempUnlimitCounter==0))
						{
							u16Temp = ((Motor1.info.u32IsumCurrentAverage/10) - MAX_DCBUS_CURRENT_LIMIT)/MIN_CURRENT_LIMIT_RESOLUTION;//if MIN_CURRENT_LIMIT_RESOLUTION=10, means 10mA as minimum current limit resolution

							if(u16Temp > SPEED_CMD_DEC_RPM_FOR_POWER_LIMIT){u16Temp = SPEED_CMD_DEC_RPM_FOR_POWER_LIMIT;}

							Motor1.command.u16SpeedCommandNow -= u16Temp;//decrease speed command for over power limitation
							Motor1.command.u8OverPowerLimitFlag = 1;//set flag to 1,it means power is over limitation
						}
					else {Motor1.command.u8OverPowerLimitFlag = 0;}//because speed command can not be decrease any more, so free this flag
			 }
	   else{Motor1.command.u8OverPowerLimitFlag = 0;}//set flag to 0, it means power is under limitation


		 u32TempTargetSpeed = Motor1.command.u16SpeedTarget;//keep the last target speed

		 if(u16TempUnlimitCounter >0){u16TempUnlimitCounter -=1;}

		 if((Motor1.info.u32IsumCurrentAverage/10) > MAX_DCBUS_CURRENT_PROTECTION)
		 {
			 Stop_The_Motor1();
			 Motor1.info.u8ErrorCode = MOTOR_DCBUS_OVER_CURRENT_ERROR;
       Motor1.info.u8ErrorCounter +=1;
       u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;
		 }

}
#endif
/********************************************************************************************************
**function name        ：SMO_Kslf_Modulate_By_Speed_Motor1(void)
**function description ：Modulate the SMO Kslf value by motor speed
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void SMO_Kslf_Modulate_By_Speed_Motor1(void)
{
	#define K_PLUS   (1+ ((SMO_Kslf_MAX_VALUE-SMO_Kslf_MIN_VALUE)/(TARGET_MAX_SPEED-OPEN_LOOP_TARGET_SPEED)))

	int16_t s16Temp;

	s16Temp= SMO_Kslf_MIN_VALUE + (Motor1.smo.s16Speed - OPEN_LOOP_TARGET_SPEED)* K_PLUS;

	if(s16Temp < SMO_Kslf_MIN_VALUE)
	{Motor1.smo.s32Kslf = SMO_Kslf_MIN_VALUE;}
	else if(s16Temp > SMO_Kslf_MAX_VALUE)
	{Motor1.smo.s32Kslf = SMO_Kslf_MAX_VALUE;}
	else
	{Motor1.smo.s32Kslf = s16Temp;}
}

/********************************************************************************************************
**function name        ：SMO_Kslf_Modulate_For_New_Startup_By_Speed_Motor1(void)//20190626
**function description ：Modulate the SMO Kslf value by motor speed
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void SMO_Kslf_Modulate_For_New_Startup_By_Speed_Motor1(void)//20190626
{
	#define K_PLUS1   (1+ ((SMO_Kslf_MAX_VALUE-SMO_Kslf_MIN_VALUE)/(TARGET_MAX_SPEED-0)))

	int16_t s16Temp;

	s16Temp= SMO_Kslf_MIN_VALUE + (Motor1.smo.s16Speed - 0)* K_PLUS1;

	if(s16Temp < SMO_Kslf_MIN_VALUE)
	{Motor1.smo.s32Kslf = SMO_Kslf_MIN_VALUE;}
	else if(s16Temp > SMO_Kslf_MAX_VALUE)
	{Motor1.smo.s32Kslf = SMO_Kslf_MAX_VALUE;}
	else
	{Motor1.smo.s32Kslf = s16Temp;}
}
/********************************************************************************************************
**function name        ：Id_Command_Modulate_By_Speed_Motor1(void)
**function description ：Modulate the Id Command value by motor speed
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Id_Command_Modulate_By_Speed_Motor1(void)//20181115
{
	#define Id_PLUS   (1+(Id_COMMAND_VALUE/(TARGET_MAX_SPEED/2)))

	int16_t s16Temp;

	#ifdef ENABLE_NEW_STARTUP_METHOD
	if((Id_COMMAND_VALUE == 0)||(Motor1.command.u8MotorRunMode !=RUN_IN_CLOSE_LOOP_MODE)) //20190626
	#else
	if(Id_COMMAND_VALUE == 0)
	#endif
	{		Motor1.command.s16IdCommand = 0;}
	else
	{
			if(Motor1.smo.s16Speed <(TARGET_MAX_SPEED/2))
			{Motor1.command.s16IdCommand = 0;}
			else
			{
				s16Temp =  (Motor1.smo.s16Speed - (TARGET_MAX_SPEED/2))* Id_PLUS;

				if(Id_COMMAND_VALUE >0)
					{
						if(s16Temp > Id_COMMAND_VALUE)
						{s16Temp = Id_COMMAND_VALUE;}
					}
				else //if means (Id_COMMAND_VALUE < 0)
				 {
					 if(s16Temp < Id_COMMAND_VALUE)
						{s16Temp = Id_COMMAND_VALUE;}
				 }
				Motor1.command.s16IdCommand = s16Temp;
			}
	}
}

/********************************************************************************************************
**function name        ：PI_Current_Ki_Modulate_By_Speed_Motor1(void)
**function description ：Modulate the FOC current loop PI control's Ki value by motor speed
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void PI_Current_Ki_Modulate_By_Speed_Motor1(void)
{
	#define S_PLUS (10+ ((PI_CURRENT_Ki_MAX_VALUE - PI_CURRENT_Ki_MIN_VALUE)/(TARGET_MAX_SPEED-OPEN_LOOP_TARGET_SPEED)))

	int16_t s16Temp, s16TempKi;

	s16Temp= PI_CURRENT_Ki_MIN_VALUE + (Motor1.smo.s16Speed - OPEN_LOOP_TARGET_SPEED)* S_PLUS;

	if(s16Temp < PI_CURRENT_Ki_MIN_VALUE)
	{s16TempKi = PI_CURRENT_Ki_MIN_VALUE;}
	else if(s16Temp > PI_CURRENT_Ki_MAX_VALUE)
	{s16TempKi = PI_CURRENT_Ki_MAX_VALUE;}
	else
	{s16TempKi = s16Temp;}

	#ifdef ENABLE_ROTOR_IPD1_FUNCTION
	if(Motor1.command.u8MotorRunMode == ROTOR_SMALL_SWING_MODE)
	{s16TempKi = PI_CURRENT_Ki_MAX_VALUE;}
	#endif

	Iq_PI.s32KiQ15 = s16TempKi;
	Id_PI.s32KiQ15 = s16TempKi;

}
/********************************************************************************************************
**function name        ：Speed_Error_Protect_Motor1(void)
**function description ：detect the estimatation speed of motor, if over/under abnormal limitation, it will stop motor
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Speed_Error_Protect_Motor1(void)
{
	 if((Motor1.smo.s16Speed >(TARGET_MAX_SPEED*8)/4)||(Motor1.smo.s16Speed <(TARGET_MIN_SPEED/4)))
		{
			Stop_The_Motor1();
			Motor1.info.u8ErrorCode = MOTOR_SPEED_ERROR;
      Motor1.info.u8ErrorCounter +=1;
      u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;
		}
}
/********************************************************************************************************
**function name        ：Lack_Phase_Detect_and_Protect_Motor1(void) //run in 2ms cycle
**function description ：detect the motor 3 phase lines connection is ok or not, if not, it will stop motor
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Lack_Phase_Detect_and_Protect_Motor1(void)
{
	#ifdef ENABLE_LACK_PHASE_DETECT_AND_PROTECT//20190626

	static uint32_t u32IaSumForLackPhase, u32IbSumForLackPhase, u32IcSumForLackPhase;
	static uint16_t u16Counter_2ms =0;
	uint32_t TempLackPhaseErrorFlag;

	//detect lack phase error at 3 mode, RUN_IN_OPEN_LOOP_MODE,RUN_IN_CLOSE_LOOP_MODE,RUN_IN_NEW_STARTUP_MODE
	//if((Motor1.command.u8MotorRunMode == STOP_MODE)||(Motor1.command.u8MotorRunMode == RUN_IN_OPEN_LOOP_MODE)||(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)||(Motor1.command.u8MotorRunMode == RUN_IN_NEW_STARTUP_MODE))
	if((Motor1.command.u8MotorRunMode == RUN_IN_OPEN_LOOP_MODE)||(Motor1.command.u8MotorRunMode == RUN_IN_CLOSE_LOOP_MODE)||(Motor1.command.u8MotorRunMode == RUN_IN_NEW_STARTUP_MODE))
	{
	  u16Counter_2ms +=1;

		if(Motor1.FOC.s16Ia <0){u32IaSumForLackPhase += (-Motor1.FOC.s16Ia);}
		else{u32IaSumForLackPhase += (Motor1.FOC.s16Ia);} //calculation Ia sum

		if(Motor1.FOC.s16Ib <0){u32IbSumForLackPhase += (-Motor1.FOC.s16Ib);}
		else{u32IbSumForLackPhase += (Motor1.FOC.s16Ib);} //calculation Ib sum

		if(Motor1.FOC.s16Ic <0){u32IcSumForLackPhase += (-Motor1.FOC.s16Ic);}
		else{u32IcSumForLackPhase += (Motor1.FOC.s16Ic);} //calculation Ic sum
	 }
	else
	{
		u16Counter_2ms =0;//clear the counter
		u32IaSumForLackPhase =0;//clear to 0
		u32IbSumForLackPhase =0;//clear to 0
		u32IcSumForLackPhase =0;//clear to 0
	 }

	if(u16Counter_2ms >= LACK_PHASE_DETECT_CYCLE_TIME)//for about 2*LACK_PHASE_DETECT_CYCLE_TIME mili-second sum current calculation
	{
	  u16Counter_2ms =0;//clear the counter
		TempLackPhaseErrorFlag =0;

	  if(Motor1.command.u8MotorRunMode != STOP_MODE)
		{
			if(u32IaSumForLackPhase > u32IbSumForLackPhase)
			{ if(u32IaSumForLackPhase > (u32IbSumForLackPhase + ((u32IbSumForLackPhase *LACK_PHASE_RATIO)/4))){TempLackPhaseErrorFlag =1;}}
			else
			{ if(u32IbSumForLackPhase > (u32IaSumForLackPhase + ((u32IaSumForLackPhase *LACK_PHASE_RATIO)/4))){TempLackPhaseErrorFlag =1;}}

			if(u32IaSumForLackPhase > u32IcSumForLackPhase)
			{ if(u32IaSumForLackPhase > (u32IcSumForLackPhase + ((u32IcSumForLackPhase *LACK_PHASE_RATIO)/4))){TempLackPhaseErrorFlag =1;}}
			else
			{ if(u32IcSumForLackPhase > (u32IaSumForLackPhase + ((u32IaSumForLackPhase *LACK_PHASE_RATIO)/4))){TempLackPhaseErrorFlag =1;}}

			if(u32IbSumForLackPhase > u32IcSumForLackPhase)
			{ if(u32IbSumForLackPhase > (u32IcSumForLackPhase + ((u32IcSumForLackPhase *LACK_PHASE_RATIO)/4))){TempLackPhaseErrorFlag =1;}}
			else
			{ if(u32IcSumForLackPhase > (u32IbSumForLackPhase + ((u32IbSumForLackPhase *LACK_PHASE_RATIO)/4))){TempLackPhaseErrorFlag =1;}}
	  }

		u32IaSumForLackPhase =0;//clear to 0
		u32IbSumForLackPhase =0;//clear to 0
		u32IcSumForLackPhase =0;//clear to 0

		if(TempLackPhaseErrorFlag ==1)
     {

				Stop_The_Motor1();
				Motor1.info.u8ErrorCode = MOTOR_LACK_PHASE_ERROR;
				Motor1.info.u8ErrorCounter = MAX_ERROR_ACCUMULATIVE_TOTAL + 1;
				u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;
		 }
	}

	#endif
}
/********************************************************************************************************
**function name        ：Rotation_Inverse_Error_Protect_Motor1(void)
**function description ：detect the direction of motor, if inverse, it will stop motor
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Rotation_Inverse_Error_Protect_Motor1(void)//20190124
{
	 if(Motor1.smo.u16RotationInverseCounter > 512)
		{
			Stop_The_Motor1();
			Motor1.info.u8ErrorCode = MOTOR_ROTATION_INVERSE_ERROR;
      Motor1.info.u8ErrorCounter +=1;
      u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;
		}
}
/********************************************************************************************************
**function name        ：Software_Over_Current_Protect_Motor1(int16_t s16PhaseCurrentX)
**function description ：it will check the measured phase current every pwm cylcle, if over once, it will stop motor
**input parameters     ：phase current, range (-32767~32767)
**output parameters    ：none
********************************************************************************************************/
void Software_Over_Current_Protect_Motor1(int16_t s16PhaseCurrentX)
{
	uint16_t u16Temp;

	if(s16PhaseCurrentX <0)
	 {s16PhaseCurrentX = -s16PhaseCurrentX;}

	if(((CURRENT_PROTECT*CURRENT_GAIN)/10) > 32000)
	{ u16Temp = 32000;}
	else
	{ u16Temp = ((CURRENT_PROTECT*CURRENT_GAIN)/10);}

	if(s16PhaseCurrentX > u16Temp)
	{
		Stop_The_Motor1();
		Motor1.info.u8ErrorCode = MOTOR_PHASE_CURRENT_ERROR;
		Motor1.info.u8ErrorCounter +=1;
    u8RestartupWaitCounter = ERROR_RESTART_WAIT_TIME;
	}
}
/********************************************************************************************************
**function name        ：Start_The_Motor1(void)
**function description ：start the motor operation
**input parameters     ：none
**output parameters    ：none
********************************************************************************************************/
void Start_The_Motor1(void)
{
	//define the initial PWM DUTY =0;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	Motor1.info.u8ErrorCode =0;//clear the error code
	Enable_Motor1_PWM_Output(); //enable motor pwm output
}
/********************************************************************************************************
**function name        ：Stop_The_Motor1(void)
**function description ：stop the motor operation
**input parameters     ：none
**output parameters    ：none
********************************************************************************************************/
void Stop_The_Motor1(void)
{
  Disable_Motor1_PWM_Output();
	Motor1.command.u8MotorRunMode = STOP_MODE;
}
/********************************************************************************************************
**function name        ：WL_PIN_TURN_ON_MOSFET(void)
**function description ：Setup PB15 pin turn on MOSFET when this pin in IO mode
**input parameters     ：none
**output parameters    ：none
********************************************************************************************************/
void WL_PIN_TURN_ON_MOSFET(void)//Setup PB15 pin turn on MOSFET when it is in IO mode
{
	SWITCH_WL_PIN_TO_IO_OUTPUT();//change WL pin to IO output

	#ifdef WL_PIN_IO_MODE_LOW_ACTIVE
	WL_PIN_IO_MODE_OUTPUT_LOW;//GPIOB->BRR |= GPIO_Pin_15;//setup IO pin output low //20190626
  #else
  WL_PIN_IO_MODE_OUTPUT_HIGH;//GPIOB->BSRR |= GPIO_Pin_15;//setup IO pin output high//20190626
  #endif
}
/********************************************************************************************************
**function name        ：WL_PIN_TURN_OFF_MOSFET(void)
**function description ：Setup PB15 pin turn off MOSFET when this pin in IO mode
**input parameters     ：none
**output parameters    ：none
********************************************************************************************************/
void WL_PIN_TURN_OFF_MOSFET(void)//Setup PB15 pin turn off MOSFET when it is in IO mode
{
	#ifdef WL_PIN_IO_MODE_LOW_ACTIVE
	WL_PIN_IO_MODE_OUTPUT_HIGH;//GPIOB->BSRR |= GPIO_Pin_15;//setup IO pin output high//20190626
  #else
  WL_PIN_IO_MODE_OUTPUT_LOW;//GPIOB->BRR |= GPIO_Pin_15;//setup IO pin output low	//20190626
  #endif
}
/********************************************************************************************************
**function name        ：SWITCH_WL_PIN_TO_IO_OUTPUT(void)
**function description ：PB15 pin change to IO output
**input parameters     ：none
**output parameters    ：none
********************************************************************************************************/
void SWITCH_WL_PIN_TO_IO_OUTPUT(void)//PB15 pin change to IO output mode
{
	GPIO_InitTypeDef GPIO_InitStructure;

	WL_PIN_TURN_OFF_MOSFET();//turn off MOSFET when this WL pin in IO output mode

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//TIM1_CH1N,TIM1_CH2N,TIM1_CH3N输出

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/********************************************************************************************************
**function name        ：SWITCH_WL_PIN_TO_PWM_OUTPUT(void)
**function description ：PB15 pin change to PWM output
**input parameters     ：none
**output parameters    ：none
********************************************************************************************************/
void SWITCH_WL_PIN_TO_PWM_OUTPUT(void)//PB15 pin change to PWM output mode
{
	GPIO_InitTypeDef GPIO_InitStructure;

	WL_PIN_TURN_OFF_MOSFET();//turn off MOSFET when this WL pin in IO output mode

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//TIM1_CH3N输出

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_2);//setup pin to PWM output mode
}
/********************************************************************************************************
**function name        ：Setup_Open_Loop_Ramp_Down_Speed_Slop_and_Target_Iq(Whole_Motor_Struct* Motor)
**function description ：setup the open loop speed down's slop1,2,3 and target Iq command in against wind's reverse action
**input parameters     ：none
**output parameters    ：none
********************************************************************************************************/
void Setup_Open_Loop_Ramp_Down_Speed_Slop_and_Target_Iq(Whole_Motor_Struct* Motor)
{
	Motor->spec.u16SpeedSlope1 = OPEN_LOOP_RAMP_DOWN_SPEED_SLOP1;//setup ramp down low speed's deacceleration slop
	Motor->spec.u16SpeedSlope2 = OPEN_LOOP_RAMP_DOWN_SPEED_SLOP2;//setup ramp down middle speed's deacceleration slop
	Motor->spec.u16SpeedSlope3 = OPEN_LOOP_RAMP_DOWN_SPEED_SLOP3;//setup ramp down high speed's deacceleration slop

	if(OPEN_LOOP_RAMP_DOWM_TARGET_Iq > 30000)
	{Motor->command.s16OpenLoopTargetIq = 30000;}
	else
	{Motor->command.s16OpenLoopTargetIq = OPEN_LOOP_RAMP_DOWM_TARGET_Iq;}//setup ramp down's target Iq command
}

/********************************************************************************************************
**function name        ：void SMO_Speed_Estimator(SMO_Struct* Motor)
**function description ：for motor speed calculation in close loop control
**input parameters     ：s16Theta
**output parameters    ：s16Speed
********************************************************************************************************/
void SMO_Speed_Estimator(SMO_Struct* Motor)
{
	#ifdef ENABLE_MAX_120000RPM_SPEED_CALCULATION    //(2 poles)
	  #define FINAL_SPEED_MULTIPLY_PARAMETER   5
	#endif

	#ifdef ENABLE_MAX_24000RPM_SPEED_CALCULATION
	  #define FINAL_SPEED_MULTIPLY_PARAMETER   1
	#endif

	#define THRESHOLD_OF_LOW_SPEED        7500  //if Electrical speed under this threshold, use 2ms period to calculate speed once
	#define HYSTERESIS_VALUE_OF_SPEED     (THRESHOLD_OF_LOW_SPEED/8)   //add hysteresis range for 1ms/2ms calculating speed option

	int32_t s32SpeedNow, s32TempTheta;
	uint32_t u32Step, u32Temp;
	static int16_t s16ThetaBefore = 0, s16SpeedOld=0;
	static int8_t u8Flag500hz =1;//1: 2ms calculate speed once, 0: 1ms calculate speed once.
	static int8_t u8SpeedLoop1msCounter;

	u8SpeedLoop1msCounter+=1;

	u32Temp = 1;

	if((u8Flag500hz == 1)&&(u8SpeedLoop1msCounter ==1))
   {u32Temp = 0;}


	if(u32Temp)
  {
		s32TempTheta = Motor->s16Theta;

		u32Step = (((int32_t)s32TempTheta + 32768 - s16ThetaBefore) % 32768);

		//if (u32Step > 16384){ u32Step = 32768 - u32Step;}//20190124

		if (u32Step > 16384)//20190124
    {
			u32Step = 32768 - u32Step;
			Motor->u16RotationInverseCounter +=1;
		}
		else if(Motor->u16RotationInverseCounter >0)
		{ Motor->u16RotationInverseCounter -=1;}

		s16ThetaBefore = s32TempTheta;

		if(u8Flag500hz == 0)//if yes, means 1ms calculate speed once
		{s32SpeedNow = (u32Step * Motor->s16SpeedEstGainQ15)>>15;}
		else//if yes, means 2ms calculate speed once
		{s32SpeedNow = (u32Step * Motor->s16SpeedEstGainQ15)>>16;}

		s32SpeedNow = (int32_t)s16SpeedOld*7 + s32SpeedNow;
		s32SpeedNow = s32SpeedNow/8;

		s16SpeedOld = s32SpeedNow;
		Motor->s16Speed = s32SpeedNow*FINAL_SPEED_MULTIPLY_PARAMETER;
	}

	if(((s32SpeedNow * (Motor->u8SmoPole>>1))< THRESHOLD_OF_LOW_SPEED)&&(u8Flag500hz ==0))
	{
		u8Flag500hz = 1;//2ms calculate speed once
		u8SpeedLoop1msCounter =0;
	}
	else if((s32SpeedNow * (Motor->u8SmoPole>>1))> (THRESHOLD_OF_LOW_SPEED+HYSTERESIS_VALUE_OF_SPEED))
	{ u8Flag500hz = 0;}//1ms calculate speed once

	Motor->u8SpeedLoop500hzFlag = u8Flag500hz;

	if(u8SpeedLoop1msCounter >=2){u8SpeedLoop1msCounter =0;}

}

/********************************************************************************************************
**function         ：DAC12bit_show (int32_t s32DACShowData)
**description      ：Show data to DAC chip output pin
**input parameter  ：int32_t s32DACShowData
**output parameter ：None
********************************************************************************************************/
void DAC12bit_show(int32_t s32DACShowData)
{
	IO_SPI_DAC(0x8000 + (s32DACShowData));
}
/********************************************************************************************************
**function         ：IO_SPI_DAC(uint16_t u16Data)
**description      ：use GPIO to output SPI signal for DAC
**input parameter  ：u16Data : input data for DAC show
**output parameter ：None
********************************************************************************************************/
void IO_SPI_DAC(uint16_t u16Data)
{
	#define SPI_CLK_0  (GPIOB->BRR  |= GPIO_Pin_11)
	#define SPI_CLK_1  (GPIOB->BSRR |= GPIO_Pin_11)
  #define SPI_MOSI_0  (GPIOD->BRR  |= GPIO_Pin_2)
	#define SPI_MOSI_1  (GPIOD->BSRR |= GPIO_Pin_2)
  #define SPI_CS_0    (GPIOD->BRR  |= GPIO_Pin_3)
	#define SPI_CS_1    (GPIOD->BSRR |= GPIO_Pin_3)

//	#define SPI_MOSI_0  PD2o0 //(GPIOD->BRR  |= GPIO_Pin_2)
//	#define SPI_MOSI_1  PD2o1 //(GPIOD->BSRR |= GPIO_Pin_2)
//  #define SPI_CS_0    PD3o0 //(GPIOD->BRR  |= GPIO_Pin_3)
//	#define SPI_CS_1    PD3o1 //(GPIOD->BSRR |= GPIO_Pin_3)

	uint16_t i;

	SPI_CS_1;
	SPI_CS_0;
  SPI_MOSI_0;
	SPI_CLK_1;

  for(i=0;i<16;i++)
	{
		SPI_CLK_1;

		if((u16Data & 0x8000)!=0)
		{SPI_MOSI_1;}
		else
		{SPI_MOSI_0;}

		SPI_CLK_0;

		u16Data = u16Data <<1;

	}
	SPI_CS_1;
}

/********************************************************************************************************
**function         ：IO_SPI_Init(void)
**description      ：use GPIO to output SPI signal for DAC
**input parameter  ：None
**output parameter ：None
********************************************************************************************************/
void IO_SPI_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

//	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  //开启GPIOA,GPIOB时钟
//
//    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  //开启 GPIOB 时钟

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);  //开启 GPIOD 时钟

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}



