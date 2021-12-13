//-------------------------------------------------------------------------------------------------
// Universal KS0108 driver library
// NXP LPC2000 MCU low-level driver
// (c) Rados³aw Kwiecieñ, radek@dxp.pl
//-------------------------------------------------------------------------------------------------
#include "lpc17xx_gpio.h"
#include "lpc17xx_clkpwr.h"

/*PORT 0*/
const uint32_t E = ((uint32_t)1<<27); 		//P0.27
const uint32_t BACKLIGHT = ((uint32_t)1<<28); //P0.28
const uint32_t RST = ((uint32_t)1<<29); 		//P0.29
const uint32_t CS2 = ((uint32_t)1<<30); 		//P0.30

/*PORT 1*/
const uint32_t CS1 = ((uint32_t)1<<18); //P1.18
const uint32_t D0 = ((uint32_t)1<<19); //P1.19
const uint32_t D1 = ((uint32_t)1<<20); //P1.20
const uint32_t D2 = ((uint32_t)1<<21); //P1.21
const uint32_t D3 = ((uint32_t)1<<22); //P1.22
const uint32_t D4 = ((uint32_t)1<<23); //P1.23
const uint32_t D5 = ((uint32_t)1<<24); //P1.24
const uint32_t D6 = ((uint32_t)1<<25); //P1.25
const uint32_t D7 = ((uint32_t)1<<26); //P1.26

/*PORT 2*/
const uint32_t LCD_PWR = ((uint32_t)1<<11); //P2.11

/*PORT 3*/
const uint32_t RS = ((uint32_t)1<<26);	//P3.26
const uint32_t RW = ((uint32_t)1<<25);	//P3.25

#define KS0108_D0			19

#define DISPLAY_STATUS_BUSY	0x80

extern unsigned char screen_x;
extern unsigned char screen_y;

//-------------------------------------------------------------------------------------------------
// Delay function
//-------------------------------------------------------------------------------------------------
void GLCD_Delay(void)
{
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

//-------------------------------------------------------------------------------------------------
// enable Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_EnableController(unsigned char controller)
{
	switch(controller)
	{
		case 0 : GPIO_ClearValue(1 , 1 << 18); break;
		case 1 : GPIO_ClearValue(0 , 1 << 30); break;
	}
}
//-------------------------------------------------------------------------------------------------
// Disable Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_DisableController(unsigned char controller)
{
	switch(controller)
	{
		case 0 : GPIO_SetValue(1 , 1 << 18); break;
		case 1 : GPIO_SetValue(0 , 1 << 30); break;
	}
}
//-------------------------------------------------------------------------------------------------
// Read Status byte from specified controller (0-2)
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadStatus(unsigned char controller)
{
	unsigned char status;
	GPIO_SetDir(1, (0xFF << KS0108_D0), 0); // set as input

	GPIO_SetValue(3 , 1 << 25); // set rw
	GPIO_ClearValue(3 , 1 << 26); // clr rs

	GLCD_EnableController(controller);

	GPIO_SetValue(0 , 1 << 27); // set en

	GLCD_Delay();

	status = (GPIO_ReadValue(1) >> KS0108_D0);
	GPIO_ClearValue(0 , 1 << 27); // clr en

	GLCD_DisableController(controller);
	return status;
}
//-------------------------------------------------------------------------------------------------
// Write command to specified controller
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite, unsigned char controller)
{
	while(GLCD_ReadStatus(controller)&DISPLAY_STATUS_BUSY);

	GPIO_SetDir(1, (0xFF << KS0108_D0), 1); // set as output
	GPIO_ClearValue(3 , 1 << 25 | 1 << 26); // clear rw .rs
	GLCD_EnableController(controller);

	GPIO_SetValue(1, (commandToWrite << KS0108_D0));
	commandToWrite ^= 0xFF;
	GPIO_ClearValue(1, (commandToWrite << KS0108_D0));

	GPIO_SetValue(0 , 1 << 27); // set en
	GLCD_Delay();
	GPIO_ClearValue(0 , 1 << 27); // clr en
	GLCD_DisableController(controller);
}

//-------------------------------------------------------------------------------------------------
// Read data from current position
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadData(void)
{
	unsigned char data;
	while(GLCD_ReadStatus(screen_x / 64)&DISPLAY_STATUS_BUSY);

	GPIO_SetDir(1, (0xFF << KS0108_D0), 0); // set as input
	GPIO_SetValue(3 , 1 << 25 | 1 << 26); // clear rw .rs
	GLCD_EnableController(screen_x / 64);
	//GLCD_Delay();
	GPIO_SetValue(0 , 1 << 27); // set en
	GLCD_Delay();
	data = (GPIO_ReadValue(1) >>  KS0108_D0);
	GPIO_ClearValue(0 , 1 << 27); // clr en
	GLCD_DisableController(screen_x / 64);
	screen_x++;
	return data;
}
//-------------------------------------------------------------------------------------------------
// Write data to current position
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite)
{
	while(GLCD_ReadStatus(screen_x / 64) & DISPLAY_STATUS_BUSY);

	GPIO_SetDir(1, (0xFF << KS0108_D0), 1); // set as output

	GPIO_ClearValue(3 , 1 << 25); // clr rw
	GPIO_SetValue(3 , 1 << 26); // set rs

	GPIO_SetValue(1, (dataToWrite << KS0108_D0));
	dataToWrite ^= 0xFF;
	GPIO_ClearValue(1, (dataToWrite << KS0108_D0));

	GLCD_EnableController(screen_x / 64);
	GPIO_SetValue(0 , 1 << 27); // set en
	GLCD_Delay();
	GPIO_ClearValue(0 , 1 << 27); // clr en
	GLCD_DisableController(screen_x / 64);
	screen_x++;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_InitalizePorts(void)
{
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPIO, ENABLE); // enable power for GPIO

	GPIO_SetDir(0, BACKLIGHT | CS2 | E | RST ,1); 					// PORT0
	GPIO_SetDir(1, CS1 | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7, 1); // PORT1
	GPIO_SetDir(2, LCD_PWR, 1); 									// PORT2
	GPIO_SetDir(3, RS | RW, 1); 									// PORT3

	GPIO_ClearValue(0, RST);  		// enter reset state
	GPIO_ClearValue(2, LCD_PWR);  	// disable power
	GLCD_Delay();
	GPIO_SetValue(0, RST);  		// release reset
	GPIO_SetValue(2, LCD_PWR);  	// enable power
}
