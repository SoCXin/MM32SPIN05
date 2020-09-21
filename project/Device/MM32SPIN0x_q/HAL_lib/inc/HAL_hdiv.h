
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_HDIV_H
#define __HAL_HDIV_H

/* Includes ------------------------------------------------------------------*/
#include "HAL_device.h"


#define DIV_UNSIGN 									0x01
#define DIV_IRQ_ENABLE 							0x02
#define DIV_OVERFLOW 								0x01

#define SET_HWDivider(x, y) \
					HDIV->DVDR = x;         \
					HDIV->DVSR = y;

#define GET_HWDivider HDIV->QUOTR;

void HDIV_Init(void);
#endif


