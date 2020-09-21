#include "HAL_hdiv.h"
#include "HAL_rcc.h"

void HDIV_SignInit(void);


void HDIV_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_HWDIV, ENABLE);
	HDIV_SignInit();
}

uint32_t HDIV_DivCalc( int32_t dvdr,int32_t dvsr,int32_t *quotr, int32_t *rmdr )
{
	uint8_t i;


	HDIV->DVDR = dvdr;
	HDIV->DVSR = dvsr;

	for(i=1;i>0;i--);

	*quotr = HDIV->QUOTR;
	*rmdr  = HDIV->RMDR;

	return (uint32_t)(HDIV->DVSR);
}


/** @defgroup DIV_Private_Defines
* @{
*/

void HDIV_UnsignInit(void)
{
    HDIV->CR |= DIV_UNSIGN ;
}


void HDIV_SignInit(void)
{
    HDIV->CR &= (~DIV_UNSIGN) ;
}

