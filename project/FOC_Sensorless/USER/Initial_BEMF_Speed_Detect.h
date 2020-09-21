#ifndef __INITIAL_BEMF_SPEED_DETECT_H
#define __INITIAL_BEMF_SPEED_DETECT_H 			   
#include "sys.h"


#define BEMF_DIR_STANDSTILL         2
#define BEMF_DIR_CW 								1
#define BEMF_DIR_CCW								0

#define BEMF_DETECT_LIMIT_TIME             600  //unit: 1ms, it define the maximum time to detect BEMF's speed and direction

typedef struct 
{
	uint16_t u16BEMFSpeed; //motor BEMF's speed
	uint8_t  u8BEMFDirectionFlag;//motor BEMF's direction
	uint16_t u16BEMF1msCounter;//for BEMF's speed counter
	uint16_t u16BEMFDetectionTime;//BEMF's checking time define
	uint16_t u16BEMFPhaseABMiddlePoint12bit;//phase A,B's BEMF midpoint voltage	
	uint16_t u16BEMFComparatorHystersis;//unit:12bit ADC,digital comparator's hysteresys
	uint16_t u16BEMFStandstillThresholdVolt;//unit:12bit ADC,define the standstill if BEMFA,B difference voltage under this limitation 
	uint8_t  bBEMFResultValidFlag;//if 1, means the result of BEMF checking is valid
	uint8_t  bBEMFMotorIsRotatingFlag;//check the motor is rotating or not, 1:rotating now, 0:standstill
	uint8_t  u8BEMFPoleNumber;//motor pole number;
} BEMF_Speed_Struct;


void BEMF_Speed_Detect(BEMF_Speed_Struct *Get_BEMF_speed);


#endif





























