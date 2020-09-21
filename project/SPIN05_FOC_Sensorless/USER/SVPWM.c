
#include "HAL_conf.h"
#include "svpwm.h"
#include "sin_table.h"
#include "led.h"
#include "Whole_Motor_Parameters.h"

extern void IO_SPI_DAC(uint16_t u16Data);

u32 sram_run_space[SRAM_RUN_LEN] __attribute__ ((at(0x20000500)));  // sram startup 

#ifdef MM32SPIN05  //72MHz PWM
#define TSDELAY       ((3*5*ADC_DELAY_TIME_1_SHUNT_R)/2) // Delay Time of ADC Current Sample, 1us = 75//20190315
#define SHIFTPERIOD2  ((3*5*ADC_TOTAL_TIME_1_SHUNT_R)/2) // Shift value for Asymmetric PWM,   1us = 75//20190315
#endif

#ifdef MM32SPIN06  //48MHz PWM
#define TSDELAY       (5*ADC_DELAY_TIME_1_SHUNT_R) // Delay Time of ADC Current Sample, 1us = 50
#define SHIFTPERIOD2  (5*ADC_TOTAL_TIME_1_SHUNT_R) // Shift value for Asymmetric PWM,   1us = 50
#endif

///*******************************************************************************
//// Includes
//*******************************************************************************/
uint8_t  u8UnDoCurrentLoopFlag;
uint16_t u16Svpwm_PwmFullScale;
uint8_t  u8SvpwmZoneOld;
uint8_t  u8SvpwmZone;
uint32_t u32PwmDutyCH1[2],u32PwmDutyCH2[2],u32PwmDutyCH3[2];

__IO uint32_t u32TIM1CC1U,u32TIM1CC1D,u32TIM1CC2U, u32TIM1CC2D,u32TIM1CC3U,u32TIM1CC3D;
__IO uint32_t u32OneShuntADCTri0,u32OneShuntADCTri1;
__IO uint8_t  u8SvpwmZoneSwtichFlag; 

void SVPWM(FOC_Struct* FOC)
{
	/* SVPWM Parameter*/
	uint32_t  u32OverModulationFullScale;
	uint8_t   u8ZoneState;
	uint32_t  Taon,Tbon,Tcon;
	
	#ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT
  uint32_t  u32FullScaleError;	
	uint8_t   flag_leftshift;
	uint8_t   flag_rightshift;
	uint32_t  leftshift_error;
	uint32_t  rightshift_error;	
	uint32_t  Taon_Up,Tbon_Up,Tcon_Up;
	uint32_t  Taon_Down,Tbon_Down,Tcon_Down;
	uint32_t  Taoff_Up,Tboff_Up,Tcoff_Up;
	uint32_t  Taoff_Down,Tboff_Down,Tcoff_Down;
	#else	
	uint32_t  Taoff,Tboff,Tcoff;	
	#endif	

	int32_t   T1,T2,T_total;	
	int32_t   Tx,Ty,Tz;
	int32_t   Vref1,Vref2,Vref3;
	
	Vref1 = FOC->s16Vbeta;
	Vref2 = ( -(signed int)FOC->s16Vbeta * 16383 + (signed int)FOC->s16Valpha*28377)/32768;
  Vref3 = ( -(signed int)FOC->s16Vbeta * 16383 - (signed int)FOC->s16Valpha*28377)/32768;
	
	u8SvpwmZoneOld = u8SvpwmZone;
  u8ZoneState=0;
	
	if(Vref3>0) u8ZoneState+=4;
	if(Vref2>0) u8ZoneState+=2;
	if(Vref1>0) u8ZoneState+=1;	 
	u8SvpwmZone = u8ZoneState;
	
	Tx = Vref1;
	Ty = -Vref3;
	Tz = -Vref2;
	
	if(u8SvpwmZone == 1)
	 {
	  T1 = Tz;
	  T2 = Ty;
	 }
	else if(u8SvpwmZone == 2)
	 {
	  T1 = Ty;
	  T2 = -Tx;
	 }
	else if(u8SvpwmZone == 3)
	 {
	  T1 = -Tz;
	  T2 = Tx;
	 }
	else if(u8SvpwmZone == 4)
	 {
	  T1 = -Tx;
	  T2 = Tz;
	 }
	else if(u8SvpwmZone == 5)
	 {
	  T1 = Tx;
	  T2 = -Ty;
	 }
  else 
	 {
	  T1 = -Ty;
	  T2 = -Tz;
	 }
	
	 T_total = (unsigned short int )(T1 + T2); 
	 
	 if(T_total > 32767)
	  {	    
			#ifdef ENABLE_HARDWARE_DIVIDER	
			T1 = Division_HDIV((T1<<15),T_total);//use MCU internal hardware divider
			#else
			T1 = runRamCallVV((uint32_t)SRAM_RUN_CODE_ADD,(T1<<15),T_total);//software divider and execute it in SRAM	
			#endif
			
			T2 = 32767 - T1;		
	  }
		
	 u16Svpwm_PwmFullScale = TPWM;
	 u32OverModulationFullScale = FULL_OVM;  //(1500~3000)

	 T1 =  (signed short int)(((signed int)T1*u32OverModulationFullScale)>>15);	 //Rescaling to pwm full scale   
	 T2 =  (signed short int)(((signed int)T2*u32OverModulationFullScale)>>15);  //Rescaling to pwm full scale 
		
	 Taon = (u32OverModulationFullScale - T1 - T2)>>1;		
	 Tbon = Taon + T1;
	 Tcon = Tbon + T2;	
		
//------------------------------------------------------------------------------------------------------------------------------------		
	 #ifdef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT	
 
	 u32FullScaleError = (u32OverModulationFullScale - u16Svpwm_PwmFullScale)>>1;		
   //if(Tbon>(TPWM - SHIFTPERIOD)) {u8UnDoCurrentLoopFlag =1;}  // If period - Middle_duty < 4us (150 = 4us in 16kHz)
   //else {u8UnDoCurrentLoopFlag = 0;} 

	 u8UnDoCurrentLoopFlag = 0;
	 
	 
	 if((T1>=SHIFTPERIOD2)||(u8UnDoCurrentLoopFlag))            // T1,T2 < 4us do shift 
   {		 
	  flag_leftshift =0;
		leftshift_error = 0;
	 }
	 else 
	 {
 		flag_leftshift =1;
    leftshift_error =SHIFTPERIOD2 - T1;
   }		 
	
	 if((T2>=SHIFTPERIOD2)||(u8UnDoCurrentLoopFlag))
   {		 
	  flag_rightshift =0;
		rightshift_error = 0;
	 }
	 else 
	 {
 		flag_rightshift =1;
    rightshift_error =SHIFTPERIOD2 - T2; 
   }	
	 
   if(flag_leftshift)
	 {
	  Taon_Up = Taon + leftshift_error;
		if(Taon_Up>=u32OverModulationFullScale) Taon_Up = u32OverModulationFullScale;
	  if(Taon>=leftshift_error) Taon_Down = Taon - leftshift_error;
		else Taon_Down = 0;
	 }
	 else
	 {
    Taon_Up = Taon;
		Taon_Down = Taon;
   }
	 	
	 if((flag_rightshift==1)||(flag_leftshift==1))//20190314 
	 {
		 if(Tbon > (u16Svpwm_PwmFullScale-SHIFTPERIOD2)){Tbon = u16Svpwm_PwmFullScale-SHIFTPERIOD2;}
		 else if (Tbon < SHIFTPERIOD2){Tbon = SHIFTPERIOD2;}
   }
	 
	 Tbon_Up = Tbon;
	 Tbon_Down = Tbon;
	  
		if(flag_rightshift)
	  {
		 Tcon_Down = Tcon + rightshift_error;
		 if(Tcon_Down>=u32OverModulationFullScale) Tcon_Down = u32OverModulationFullScale;
		 if(Tcon>=rightshift_error) Tcon_Up = Tcon - rightshift_error;
		 else Tcon_Up = 0;
	  }
	  else
		{
	 	 Tcon_Up = Tcon;
		 Tcon_Down = Tcon;
    }
	 
	 switch(u8SvpwmZone){
		
		case 1:
			 Taoff_Up = Tbon_Up;
			 Tboff_Up = Taon_Up;
			 Tcoff_Up = Tcon_Up;
		
			 Taoff_Down = Tbon_Down;
			 Tboff_Down = Taon_Down;
			 Tcoff_Down = Tcon_Down;
		break;
		
		case 2:
			 Taoff_Up = Taon_Up;
			 Tboff_Up = Tcon_Up;
			 Tcoff_Up = Tbon_Up;
		
			 Taoff_Down = Taon_Down;
			 Tboff_Down = Tcon_Down;
			 Tcoff_Down = Tbon_Down;
		break;
		
		case 3:
			 Taoff_Up = Taon_Up;
			 Tboff_Up = Tbon_Up;
			 Tcoff_Up = Tcon_Up;
		
			 Taoff_Down = Taon_Down;
			 Tboff_Down = Tbon_Down;
			 Tcoff_Down = Tcon_Down;
		break;
		
		case 4:
			 Taoff_Up = Tcon_Up;
			 Tboff_Up = Tbon_Up;
			 Tcoff_Up = Taon_Up;
		
			 Taoff_Down = Tcon_Down;
			 Tboff_Down = Tbon_Down;
			 Tcoff_Down = Taon_Down;	
		break;
		
		case 5:
			 Taoff_Up = Tcon_Up;
			 Tboff_Up = Taon_Up;
			 Tcoff_Up = Tbon_Up;
		
			 Taoff_Down = Tcon_Down;
			 Tboff_Down = Taon_Down;
			 Tcoff_Down = Tbon_Down;
		break;
		
		case 6:
			 Taoff_Up = Tbon_Up;
			 Tboff_Up = Tcon_Up;
			 Tcoff_Up = Taon_Up;

			 Taoff_Down = Tbon_Down;
			 Tboff_Down = Tcon_Down;
			 Tcoff_Down = Taon_Down;
		break;
			
		default: 
			 Taoff_Up = TPWM_Half;
			 Tboff_Up = TPWM_Half;
			 Tcoff_Up = TPWM_Half;	
			 
			 Taoff_Down = TPWM_Half;
			 Tboff_Down = TPWM_Half;
			 Tcoff_Down = TPWM_Half;	
		break;
		
		}
    
    if( Taoff_Up>=u32FullScaleError) Taoff_Up=Taoff_Up-u32FullScaleError;
		else Taoff_Up = 0;
		if( Tboff_Up>=u32FullScaleError) Tboff_Up=Tboff_Up-u32FullScaleError;
		else Tboff_Up = 0;
		if( Tcoff_Up>=u32FullScaleError) Tcoff_Up=Tcoff_Up-u32FullScaleError;
		else Tcoff_Up = 0;		
		
		if( Taoff_Down>=u32FullScaleError) Taoff_Down=Taoff_Down-u32FullScaleError;
		else Taoff_Down = 0;
		if( Tboff_Down>=u32FullScaleError) Tboff_Down=Tboff_Down-u32FullScaleError;
		else Tboff_Down = 0;
		if( Tcoff_Down>=u32FullScaleError) Tcoff_Down=Tcoff_Down-u32FullScaleError;
		else Tcoff_Down =0;
		
		if( Taoff_Up>=u16Svpwm_PwmFullScale) Taoff_Up=u16Svpwm_PwmFullScale;
		if( Tboff_Up>=u16Svpwm_PwmFullScale) Tboff_Up=u16Svpwm_PwmFullScale;
		if( Tcoff_Up>=u16Svpwm_PwmFullScale) Tcoff_Up=u16Svpwm_PwmFullScale;
		
		if( Taoff_Down>=u16Svpwm_PwmFullScale) Taoff_Down=u16Svpwm_PwmFullScale;
		if( Tboff_Down>=u16Svpwm_PwmFullScale) Tboff_Down=u16Svpwm_PwmFullScale;
		if( Tcoff_Down>=u16Svpwm_PwmFullScale) Tcoff_Down=u16Svpwm_PwmFullScale;
				   
			
	 if(Motor1.command.u8MotorDirection==1)
	 {
		u32TIM1CC1U =  Taoff_Up;	//load pwm duty to pwm ouptput pin
		u32TIM1CC2U =  Tboff_Up;	//load pwm duty to pwm ouptput pin
		u32TIM1CC3U =  Tcoff_Up;	//load pwm duty to pwm ouptput pin
		u32TIM1CC1D =  Taoff_Down;	//load pwm duty to pwm ouptput pin
		u32TIM1CC2D =  Tboff_Down;	//load pwm duty to pwm ouptput pin
		u32TIM1CC3D =  Tcoff_Down;	//load pwm duty to pwm ouptput pin		
	 }
	 else
	 {
		u32TIM1CC1U =  Taoff_Up;	//load pwm duty to pwm ouptput pin
		u32TIM1CC2U =  Tcoff_Up;	//load pwm duty to pwm ouptput pin
		u32TIM1CC3U =  Tboff_Up;	//load pwm duty to pwm ouptput pin
		u32TIM1CC1D =  Taoff_Down;	//load pwm duty to pwm ouptput pin
		u32TIM1CC2D =  Tcoff_Down;	//load pwm duty to pwm ouptput pin
		u32TIM1CC3D =  Tboff_Down;	//load pwm duty to pwm ouptput pin			 
	 }
	 
	 /* SVPWM Update Flag */
	u8SvpwmZoneSwtichFlag = 1;
	 
  /* Calculate ADC Trigger Point in PWM UH,VH,WH  falling edge */	
	if(Tbon_Down < TSDELAY){ u32OneShuntADCTri1 = 0;}
	else { u32OneShuntADCTri1 = Tbon_Down - TSDELAY;}
	
	if(Tcon_Down < TSDELAY){u32OneShuntADCTri0 = 0;}
	else{ u32OneShuntADCTri0 = Tcon_Down - TSDELAY;}
	
//	u32OneShuntADCTri1 = Tbon_Down - TSDELAY;   // 100 = 2us for delay 1111111
//	u32OneShuntADCTri0 = Tcon_Down - TSDELAY;   // 100 = 2us for delay
//			
//	if(u32OneShuntADCTri0>u16Svpwm_PwmFullScale) u32OneShuntADCTri0 = u16Svpwm_PwmFullScale;
//	if(u32OneShuntADCTri1>u16Svpwm_PwmFullScale) u32OneShuntADCTri1 = u16Svpwm_PwmFullScale;
//	if(u32OneShuntADCTri0<1) u32OneShuntADCTri0 = 1;
//	if(u32OneShuntADCTri1<1) u32OneShuntADCTri1 = 1;	
	
	#endif
//-----------------------------------------------------------------------------------------------------------------------	 
  #ifndef ENABLE_1_SHUNT_R_TO_MEASURE_3_PHASE_CURRENT //if not define, it will use 2 shunt R to measure 3 phase current
	 
	if(u8SvpwmZone == 1)
	 {
	  Taoff = Tbon;
	  Tboff = Taon;
	  Tcoff = Tcon;
	 }
	else if(u8SvpwmZone == 2)
	 {
	  Taoff = Taon;
	  Tboff = Tcon;
	  Tcoff = Tbon;
	 }
	else if(u8SvpwmZone == 3)
	 {
	  Taoff = Taon;
	  Tboff = Tbon;
	  Tcoff = Tcon;
	 }
	else if(u8SvpwmZone == 4)
	 {
	  Taoff = Tcon;
	  Tboff = Tbon;
	  Tcoff = Taon;
	 }
	else if(u8SvpwmZone == 5)
	 {
	  Taoff = Tcon;
	  Tboff = Taon;
	  Tcoff = Tbon;
	 }
  else 
	 {
	  Taoff = Tbon;
	  Tboff = Tcon;
	  Tcoff = Taon;
	 }	
		 
	if( Taoff>=u16Svpwm_PwmFullScale) Taoff=u16Svpwm_PwmFullScale;
  if( Tboff>=u16Svpwm_PwmFullScale) Tboff=u16Svpwm_PwmFullScale;
  if( Tcoff>=u16Svpwm_PwmFullScale) Tcoff=u16Svpwm_PwmFullScale;
	 
	u32PwmDutyCH1[0] =  Taoff; 
	u32PwmDutyCH2[0] =  Tboff;
	u32PwmDutyCH3[0] =  Tcoff;
	
	//for make sure has enough time which ADC can read correct 2 phase current
  if(u32PwmDutyCH1[0]<=LOW_SIDE_MIN_PWM_DUTY){u32PwmDutyCH1[0]=LOW_SIDE_MIN_PWM_DUTY;}
 	if(u32PwmDutyCH2[0]<=LOW_SIDE_MIN_PWM_DUTY){u32PwmDutyCH2[0]=LOW_SIDE_MIN_PWM_DUTY;}
	if(u32PwmDutyCH3[0]<=LOW_SIDE_MIN_PWM_DUTY){u32PwmDutyCH3[0]=LOW_SIDE_MIN_PWM_DUTY;} 
	
	TIM1->CCR1 = u32PwmDutyCH1[0];
	 
	if(Motor1.command.u8MotorDirection==1)//if yes, means motor direction command is CW
	{	
		TIM1->CCR2 = u32PwmDutyCH2[0];
		TIM1->CCR3 = u32PwmDutyCH3[0];
	}
	else//if yes, means motor direction command is CCW
	{		
		TIM1->CCR2 = u32PwmDutyCH3[0];
		TIM1->CCR3 = u32PwmDutyCH2[0];
	}	
#endif	
}

/********************************************************************************************************
**函数信息 ：除法运算16位(3)                   
**功能描述 ：无
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
int32_t Division(int32_t m, int32_t n)
{
    //calculate m/n without * and /
    unsigned int rest,divisor, op,result = 0;
    int flag;
    int bits = 0;
    //bits用于记录商的1在哪一位
//    assert(n!=0);
    if((m<0 && n>0) || ( m>0 && n<0 ))
        flag = -1;
    else
        flag = 1;
    rest = m>0?m:-m;
    divisor = n>0?n:-n;
    if(rest < divisor)
        return 0;

    op = divisor;

    while(op<=rest) {
        bits++; 
        op=op<<1;
    }
    op=op>>1;
    bits--;

    while(op>=divisor) {
        if(rest>=op) {
            rest-=op;
            result += 1<<bits;
        }
        op = op>>1;
        bits--;
        
    }
    /*      重写部分结束         */
    return flag * result;
}

int32_t Division_HDIV(int32_t m, int32_t n)
{
		if( n == 0 ){return 0;}
			
		HDIV->DVDR = m;         
		HDIV->DVSR = n;
		return ( HDIV->QUOTR );
}
/********************************************************************************************************
**function name        ：flashCode_to_ram               
**function description ：move code to SRAM, and run this code in SRAM
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void flashCode_to_ram(int32_t (pfun_io)(int32_t,int32_t),int32_t size,int32_t ram_add)
{
	unsigned char *p;
	int i;
	p = (unsigned char*)pfun_io;	//给指针赋值
	p--;
	for(i = 0;i < size ;i++)
	{
		*(unsigned char*)(ram_add + i) = *p++;
	}
}
/********************************************************************************************************
**function name        ：runRamCallVV                
**function description ：运行无参数无返回类型的函数
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
int32_t runRamCallVV(uint32_t runCodeAdd,int32_t A,int32_t B)
{
	u32 sum;
	sum=((u32(*)(u32,u32))(runCodeAdd|1))(A,B);
//	((void(*)(void))(runCodeAdd|1))();
	return sum;
}



