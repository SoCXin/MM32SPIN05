
#ifndef __CurrentSensing_H
#define __CurrentSensing_H

#include "sys.h"

typedef struct {
	uint8_t  bZeroCurrentCalibrationStatusFlag;
	uint16_t u16Counter;
	uint16_t u16ExeItems;   //Execution of items for average.
	int32_t  s32IaBuffer;
	int32_t  s32IbBuffer;
	int32_t  s32IcBuffer;
	int32_t  s32IsumBuffer;
	int16_t  s16IaZero;
	int16_t  s16IbZero;
	int16_t  s16IcZero;
	int16_t  s16IsumZero;
}CurrentSensing_Struct;

#define ZERO_CURRENT_CALIBRATION_DONE   0
#define DO_ZERO_CURRENT_CALIBRATION     1

void ADC_Zero_Current_Calibration(CurrentSensing_Struct* C, int16_t s16ZeroIaInput, int16_t s16ZeroIbInput, int16_t s16ZeroIcInput, int16_t s16ZeroIsumInput);

#endif


