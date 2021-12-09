/*********************************************************************
*	common.H
**********************************************************************/

#ifndef _COMMON_H_

	#define _COMMON_H_
	#define CHECKBIT(x, y)    (((x) & (y)) == (y))
	#define SETBIT(x, y)		(x |= (y))
	#define clear_buffer(x)   memset(x, 0, sizeof(x));

	#include "lpc_types.h"

	void set_pin_function( uint8_t PORT_, uint8_t PIN_, uint8_t FUNC_);
	void delay(void);
	void output_toogle(uint8_t PORT_, uint32_t PIN_);
	void output_high(uint8_t PORT_, uint32_t PIN_);
	void output_low(uint8_t PORT_, uint32_t PIN_);

	void MLIB_USleep(uint32_t us);
	void MLIB_Sleep(uint32_t ms);

	// MLIB defini
	// procesora parametri
	#define MLIB_MCU_CORE					MLIB_CORTEX_M3			///<
	#define MLIB_MCU_TYPE					MLIB_LPC17xx			///< MCU tips
	#define MLIB_MCU_CLOCK_FREQ				SystemCoreClock			///<  frekvence, Hz

	//
	#define MLIB_SYS_CLOCK_TIMER			LPC_TIM0				///<  taimeris
	#define MLIB_SYS_CLOCK_TIMER_IRQ		TIMER0_IRQn				///<
	#define MLIB_SYS_CLOCK_TIMER_IRQFUNC	TIMER0_IRQHandler		///<


#endif	// _COMMON_H_ H
