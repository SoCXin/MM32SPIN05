
#include "CurrentSensing.h"

void ADC_Zero_Current_Calibration(CurrentSensing_Struct* C, int16_t s16ZeroIaInput, int16_t s16ZeroIbInput, int16_t s16ZeroIcInput,int16_t s16ZeroIsumInput)
{
		if (C->u16Counter < C->u16ExeItems)
		{
			C->s32IaBuffer += s16ZeroIaInput;
			C->s32IbBuffer += s16ZeroIbInput;
			C->s32IcBuffer += s16ZeroIcInput;
			C->s32IsumBuffer += s16ZeroIsumInput;
			C->u16Counter++;
		}
		else
		{
			C->s16IaZero = C->s32IaBuffer / C->u16ExeItems;
			C->s16IbZero = C->s32IbBuffer / C->u16ExeItems;
			C->s16IcZero = C->s32IcBuffer / C->u16ExeItems;
			C->s16IsumZero = C->s32IsumBuffer / C->u16ExeItems;
			C->u16Counter=0;
			C->s32IaBuffer=0; 
			C->s32IbBuffer=0;
			C->s32IcBuffer=0;
			C->s32IsumBuffer=0;
			C->bZeroCurrentCalibrationStatusFlag = ZERO_CURRENT_CALIBRATION_DONE;
		}	
}

