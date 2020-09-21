
#ifndef __FOC_PMSM_H
#define __FOC_PMSM_H

#include "sys.h"

typedef struct 
{
	__IO int16_t s16Vq;
	__IO int16_t s16Vd;
	__IO int16_t s16Valpha;
	__IO int16_t s16Vbeta;
	__IO int16_t s16Ia;
	__IO int16_t s16Ib;
	__IO int16_t s16Ic;
	__IO int16_t s16Ialpha;
	__IO int16_t s16Ibeta;
	__IO int16_t s16Iq;
	__IO int16_t s16Id;	
} FOC_Struct;


void FOC_Coordinate_Transformation(FOC_Struct* FOC, int16_t s16Cos, int16_t s16Sin);

#endif
