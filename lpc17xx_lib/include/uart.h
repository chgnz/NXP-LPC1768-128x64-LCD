/*********************************************************************
*	UART.H
**********************************************************************/
#ifndef _UART_H_

	#define _UART_H_

	#include "common.h"
	#include "lpc17xx_uart.h"

	#define GPS LPC_UART0
	#define DUT1 LPC_UART2
	#define DUT2 LPC_UART3

	#define GPS_IRQ UART0_IRQn
	#define DUT1_IRQ UART2_IRQn
	#define DUT2_IRQ UART3_IRQn

	void uart_io_setup(void);
	void uart_setup(LPC_UART_TypeDef* LPC_UARTx , uint32_t BAUD_);

#endif	// UART H
