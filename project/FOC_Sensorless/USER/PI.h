
#ifndef __PI_H
#define __PI_H

#include "sys.h"

#define SPEED_Ki_MULTIPLY         16
#define CURRENT_Ki_MULTIPLY       512

typedef struct {

	int32_t s32ErrorOld;
	int32_t s32IntegralResult;
	int32_t s32KpQ15,s32KiQ15;
	int32_t s32PIOutputLimit;
	int32_t s32IntegralResultLimit;
}PI_Struct;

int16_t PI_control_speed(int16_t s16Error, PI_Struct* C);
int16_t PI_control_current(int16_t s16Error, PI_Struct* C);

#endif
