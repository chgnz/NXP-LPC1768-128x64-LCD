/*********************************************************************
*	UART.C
**********************************************************************/

 #include "uart.h"
 
/*********************************************************************
 * 	UART INPUT/OUTPUT PIN SETUP
**********************************************************************/
void uart_io_setup(void)
{
// 	set_pin_function(PORT , PIN , FUNC);

	set_pin_function(0,10,1); // TX UART2
	set_pin_function(0,11,1); // RX UART2
	set_pin_function(0,0,2); // TX UART3
	set_pin_function(0,1,2); // RX UART3
	set_pin_function(0,2,1); // TX UART0
	set_pin_function(0,3,1); // RX UART0
}

/*********************************************************************
 * @brief 		Setup UART
 * @param[in]	LPC_UART_TypeDef* LPC_UARTx,
 * 				should be one of the following:
 * 				- LPC_UART0	: Port 0
 * 				- LPC_UART1	: Port 1
 * 				- LPC_UART2	: Port 2
 * 				- LPC_UART3	: Port 3
 * @param[in]	uint32_t BAUD_,
 * 				UART Baudrate - 2400,4800,9600,19200... etc
 * @return 		None
 **********************************************************************/


void uart_setup(LPC_UART_TypeDef* LPC_UARTx , uint32_t BAUD_)
{

	UART_CFG_Type UARTConfigStruct;					/* UART Configuration structure variable */
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;		/* UART FIFO configuration Struct variable */

	UART_ConfigStructInit(&UARTConfigStruct);
		/* ReSet Baudrate to 19200 */
		UARTConfigStruct.Baud_rate = BAUD_;

	UART_Init((LPC_UART_TypeDef *)LPC_UARTx, &UARTConfigStruct);/* Initialize UART0 peripheral with given to corresponding parameter */

	UART_IntConfig((LPC_UART_TypeDef *)LPC_UARTx, UART_INTCFG_RBR, ENABLE); 		//enable interrupt
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UARTx, &UARTFIFOConfigStruct);/* Initialize FIFO for UART0 peripheral */
	UART_TxCmd((LPC_UART_TypeDef *)LPC_UARTx, ENABLE);/*  Enable UART Transmit */

   // NVIC_EnableIRQ(GPS_IRQ);
}
