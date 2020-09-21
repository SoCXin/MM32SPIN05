
#include "FOC_PMSM.h"
#include "sys.h"

void FOC_Coordinate_Transformation(FOC_Struct* FOC, int16_t s16Cos, int16_t s16Sin)
{
	//a-b-c to alpha-beta (clark)
	FOC->s16Ialpha = FOC->s16Ia;
	FOC->s16Ibeta = ((int32_t)(FOC->s16Ib - FOC->s16Ic)*18918)>>15; 
	
	//alpha-beta to d-q (park)	
	FOC->s16Id = ((int32_t)s16Cos*FOC->s16Ialpha  + (int32_t)s16Sin*FOC->s16Ibeta)>>15;
	FOC->s16Iq = (-(int32_t)s16Sin*FOC->s16Ialpha + (int32_t)s16Cos*FOC->s16Ibeta)>>15;

  // d-q to alpfa-beta (inv park)
	FOC->s16Valpha = ((int32_t)FOC->s16Vd*s16Cos - ((int32_t)FOC->s16Vq*s16Sin))>>15;
	FOC->s16Vbeta  = ((int32_t)FOC->s16Vd*s16Sin + ((int32_t)FOC->s16Vq*s16Cos))>>15;	
} 

