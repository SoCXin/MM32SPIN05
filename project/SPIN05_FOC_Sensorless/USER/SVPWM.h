
#ifndef __SVPWM_H
#define __SVPWM_H	 

#include "HAL_conf.h"
#include "whole_motor_structure.h"	
#include "Whole_Motor_Parameters.h"

void SVPWM(FOC_Struct* FOC);
int32_t Division(int32_t m, int32_t n);
int32_t Division_HDIV(int32_t m, int32_t n);
int32_t runRamCallVV(uint32_t runCodeAdd,int32_t A,int32_t B);
void flashCode_to_ram(int32_t (pfun_io)(int32_t,int32_t),int32_t size,int32_t ram_add);

extern uint8_t  u8UnDoCurrentLoop_Flag;
extern uint16_t u16Svpwm_PwmFullScale;
extern uint8_t  u8SvpwmZoneOld;
extern uint8_t  u8SvpwmZone;
extern uint32_t u32PwmDutyCH1[2],u32PwmDutyCH2[2],u32PwmDutyCH3[2];

extern __IO uint32_t u32TIM1CC1U,u32TIM1CC1D,u32TIM1CC2U, u32TIM1CC2D,u32TIM1CC3U,u32TIM1CC3D;
extern __IO uint32_t u32OneShuntADCTri0,u32OneShuntADCTri1;
extern __IO uint8_t  u8SvpwmZoneSwtichFlag;

#define DIV_FACTOR  (5)        // Angle number scale to Table Number - FIXED
#define DIV_FRAC    ((0xFFFF*1024)/PWM_FREQUENCY)   // Angle number scale to Table Number = 0xFFFF *(500/FREQUENCY)
#define PWM_NUM     (1023)      // Table Number
#define TPWM        (PWM_PERIOD)   // PWM period
#define FULL_OVM    (TPWM)     // Over Modulation setting 1500~3000 (1500 is no over modulation, 3000 is Max value)
#define TPWM_Half   (TPWM/2)   // Half Period

#define CLOCKWISE              // Motor Direction
//#define ASYMPWM              // Asymmetric PWM mode Enable
//#define FIVE_SECTOR_SVPWM    // Enable :5-Sector, Disable: 7-Sector
	 
/* Angle of Rotor */ // If Use want to change PWM frequency, Please change FREQENCY value 16k = 16000, 20k = 20000
#define DEG_360   (PWM_FREQUENCY*32)
#define DEG_330   (((PWM_FREQUENCY*11)/12)*32)
#define DEG_270   (((PWM_FREQUENCY*3)/4)*32)
#define DEG_180   ((PWM_FREQUENCY/2)*32)
#define DEG_90    ((PWM_FREQUENCY/4)*32) 
#define DEG_60    ((PWM_FREQUENCY/6)*32)
#define DEG_30    ((PWM_FREQUENCY/12)*32)

//#define  TIM1_CH1_DUTY  (TIM1->CCR1)
//#define  TIM1_CH2_DUTY  (TIM1->CCR2)
//#define  TIM1_CH3_DUTY  (TIM1->CCR3)

//#define SRAM_RUN_CODE_ADD    	0x20001000
//实现局部代码在SRAM运行
//SRAM 开辟copy程序空间大小,适合小于SRAM_RUN_LEN的函数调用,调用函数长度可查看.map文件;
#ifdef ENABLE_HARDWARE_DIVIDER 
#define SRAM_RUN_LEN					1//0x100//20190830
#else
#define SRAM_RUN_LEN					0x100//20190830
#endif
//SRAM 开辟copy程序空间地址
#define SRAM_RUN_CODE_ADD    	sram_run_space
extern u32 sram_run_space[SRAM_RUN_LEN] __attribute__ ((at(0x20000500)));

#endif

