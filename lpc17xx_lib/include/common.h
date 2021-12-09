/*********************************************************************
*	common.H
**********************************************************************/

#ifndef _COMMON_H_

	#define _COMMON_H_
	#define CHECKBIT(x, y)    (((x) & (y)) == (y))
	#define SETBIT(x, y)		(x |= (y))
	#define clear_buffer(x)   memset(x, 0, sizeof(x));

	#include "lpc_types.h"
	#include "string.h"

	void set_pin_function( uint8_t PORT_, uint8_t PIN_, uint8_t FUNC_);
	void delay(void);
	void output_toogle(uint8_t PORT_, uint32_t PIN_);
	void output_high(uint8_t PORT_, uint32_t PIN_);
	void output_low(uint8_t PORT_, uint32_t PIN_);

	void MLIB_USleep(uint32_t us);
	void MLIB_Sleep(uint32_t ms);

	// MLIB defini
	// procesora parametri
	#define MLIB_MCU_CORE					MLIB_CORTEX_M3			///< kodola arhitektūra
	#define MLIB_MCU_TYPE					MLIB_LPC17xx			///< MCU tips
	#define MLIB_MCU_CLOCK_FREQ				SystemCoreClock			///< galvenā pulksteņa frekvence, Hz

	// koplietošanas aparatūra
	#define MLIB_SYS_CLOCK_TIMER			LPC_TIM0				///< sistēmas pulksteņa taimeris
	#define MLIB_SYS_CLOCK_TIMER_IRQ		TIMER0_IRQn				///< sistēmas pulksteņa taimera pārtraukums
	#define MLIB_SYS_CLOCK_TIMER_IRQFUNC	TIMER0_IRQHandler		///< sistēmas pulksteņa taimera pārtraukuma f-ja


#endif	// _COMMON_H_ H
