#include "systime.h"
#include "lpc17xx_rit.h"

static volatile uint32_t systime;

// Setup RIT timer with 1 ms interval and enable interrupt
void DelayInitalize()
{
	RIT_Init(LPC_RIT);
	RIT_TimerConfig(LPC_RIT,1);
	NVIC_EnableIRQ(RIT_IRQn);
}

// Execute blocking delay
void DelayMs(uint32_t ms)
{
	uint32_t time = systime;
	while(systime - time < ms);
}

// interrupt handler, increment systime counter
void RIT_IRQHandler(void)
{
    /* call this to clear interrupt flag */
    RIT_GetIntStatus(LPC_RIT);
    systime++;
}
