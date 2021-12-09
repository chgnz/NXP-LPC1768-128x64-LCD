/*********************************************************************
*	IO.C
**********************************************************************/

#include "lpc17xx_exti.h"
#include "io.h"

/*********************************************************************//**
 * @brief		EXTINT_exti0
 * @param[in]	None
 * @return 		int
 **********************************************************************/

void setup_ext_int_io(void)
{
	set_pin_function(2,12,1); // extint 2
	set_pin_function(2,13,1); // extint 3
}


void setup_ext_int(void)
	{
	EXTI_InitTypeDef EXTICfg;

	EXTICfg.EXTI_Line = EXTI_EINT2;
	/* edge sensitive */
	EXTICfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
	EXTICfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
	EXTI_ClearEXTIFlag(EXTI_EINT2);
	EXTI_Config(&EXTICfg);

	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(EINT2_IRQn, 0);
	NVIC_EnableIRQ(EINT2_IRQn);

	EXTICfg.EXTI_Line = EXTI_EINT3;
	/* edge sensitive */
	EXTICfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
	EXTICfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
	EXTI_ClearEXTIFlag(EXTI_EINT3);
	EXTI_Config(&EXTICfg);

	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(EINT3_IRQn, 0);
	NVIC_EnableIRQ(EINT3_IRQn);

}

