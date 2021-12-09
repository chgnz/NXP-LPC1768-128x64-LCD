/*********************************************************************
*	common.c
**********************************************************************/

#include "common.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_GPIO.h"

/*********************************************************************//**
 * @brief 		Setup PIN Function mode for each pin
 * @param[in]	PORT_ 	PORT number,
 * 				should be one of the following:
 * 				0 - 3;
 * @param[in]	PIN_	Pin number,
 * 				should be one of the following:
 *				0 - 31;
 * @param[in]	FUNC_  	Pin function,
 * 				should be one of the following:
 * 				- PINSEL_PINMODE_NORMAL : Pin is in the normal (not open drain) mode
 * 				- PINSEL_PINMODE_OPENDRAIN : Pin is in the open drain mode
 * @return 		None
 **********************************************************************/
void set_pin_function( uint8_t PORT_, uint8_t PIN_, uint8_t FUNC_)
{
	PINSEL_CFG_Type PinCfg;
	PinCfg.Portnum = PORT_;
	PinCfg.Pinnum = PIN_;
	PinCfg.Funcnum = FUNC_;
	PINSEL_ConfigPin(&PinCfg);
}


/*********************************************************************//**
 * TOOGLE OUTPUT
 **********************************************************************/
void output_toogle(uint8_t PORT_, uint32_t PIN_)
{
	if((GPIO_ReadValue(PORT_) & (uint32_t)1<<PIN_) >> PIN_) // ja pins iesleegts tad izsleedzam
	{
		GPIO_ClearValue((uint8_t)PORT_, (uint32_t)1<<PIN_);
	}
	else
	{
		GPIO_SetValue((uint8_t)PORT_, (uint32_t)1<<PIN_);
	}
}

/*********************************************************************
 * OUTPUT HIGH
 **********************************************************************/
void output_high(uint8_t PORT_, uint32_t PIN_)
{
	GPIO_SetValue((uint8_t)PORT_, (uint32_t)1<<PIN_);
}


/*********************************************************************
 * OUTPUT LOW
 **********************************************************************/
void output_low(uint8_t PORT_, uint32_t PIN_)
{
	GPIO_ClearValue((uint8_t)PORT_, (uint32_t)1<<PIN_);
}



/*********************************************************************//**
 * DELAY
 **********************************************************************/
void delay(void)
{
uint32_t delay_i, delay_j;

for(delay_i=0xF; delay_i>0; delay_i--)
    for(delay_j=0xF; delay_j>0; delay_j--) {
    }
}

//-------------------------------------------------------------------------------------------------
/**
	Apstādina izpildi uz norādīto laiku.

	@param		us		laiks, us
*/
void MLIB_USleep(uint32_t us)
{
	uint32_t cycle2loop = MLIB_MCU_CLOCK_FREQ;

	if (cycle2loop >= 2000000)
	{
		cycle2loop /= 1000000;
		cycle2loop *= us;
	}

	else
	{
		cycle2loop *= us;
		cycle2loop /= 1000000;
	}

	if (cycle2loop <= 100)
	{
		return;
	}

	cycle2loop -= 100; 	// ciklu skaits ieejai & izejai
	cycle2loop /= 4; 	// ciklu skaits iterācijā, 4 uz Cortex M0/M3

	if (!cycle2loop)
	{
		return;
	}

	// pauzējam
	// pauzējam
	asm volatile (".syntax unified");

	asm volatile
	(
		" mov r3, %[loops]\n"
		"loop: subs r3, #1\n"
		" bne loop\n\n"
		:
		: [loops] "r" (cycle2loop)
		: "r3"
	);

//	asm volatile (".syntax divided");
}

//-------------------------------------------------------------------------------------------------
/**
	Apstādina izpildi uz norādīto laiku.

	@param		us		laiks, ms
*/
void MLIB_Sleep(uint32_t ms)
{
	uint8_t i;
	while (ms--)
	{
		for (i = 0; i < 4; ++i)
		{
			MLIB_USleep(250);
		}
	}
}

