
#include "PI.h"
#include "sys.h"


int16_t PI_control_speed(int16_t s16Error, PI_Struct* C)
{
	int32_t s32Temp = 0;
                
    C->s32IntegralResult += (((int32_t)(C->s32KiQ15*C->s32ErrorOld))>>15);
	
    if(C->s32IntegralResult >= C->s32IntegralResultLimit) C->s32IntegralResult = C->s32IntegralResultLimit;
    else if(C->s32IntegralResult <= -C->s32IntegralResultLimit) C->s32IntegralResult = -C->s32IntegralResultLimit;
                
	  C->s32ErrorOld = s16Error;
                
    s32Temp = ((C->s32KpQ15*s16Error)>>15) + (C->s32IntegralResult/SPEED_Ki_MULTIPLY);
                
    if(s32Temp >= C->s32PIOutputLimit) s32Temp = C->s32PIOutputLimit;
    else if(s32Temp <= -C->s32PIOutputLimit) s32Temp = -C->s32PIOutputLimit;
	
	return s32Temp;
	
}


int16_t PI_control_current(int16_t s16Error, PI_Struct* C)
{
	int s32Temp = 0;
                
    C->s32IntegralResult += ((int32_t)(C->s32KiQ15*C->s32ErrorOld)>>15);
	
    if(C->s32IntegralResult >= C->s32IntegralResultLimit) C->s32IntegralResult = C->s32IntegralResultLimit;
    else if(C->s32IntegralResult <= -C->s32IntegralResultLimit) C->s32IntegralResult = -C->s32IntegralResultLimit;
                
	  C->s32ErrorOld = s16Error;
                
    s32Temp = ((C->s32KpQ15*s16Error)>>15) + ((C->s32IntegralResult)/CURRENT_Ki_MULTIPLY);
                
    if(s32Temp >= C->s32PIOutputLimit) s32Temp = C->s32PIOutputLimit;
    else if(s32Temp <= -C->s32PIOutputLimit) s32Temp = -C->s32PIOutputLimit;
	
	return s32Temp;
	
}


