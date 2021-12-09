/*********************************************************************
*	analog.h
**********************************************************************/

#ifndef _ANALOG_H_

	#define _ANALOG_H_

	#include "lpc17xx_adc.h"
	#include "lpc17xx_pinsel.h"

	void initADC(void);
	void readADC(void);
	void ADC_IRQHandler(void);

#endif	// _ANALOG_H_ H
