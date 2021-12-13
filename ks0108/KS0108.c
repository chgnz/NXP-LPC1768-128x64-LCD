//-------------------------------------------------------------------------------------------------
// Universal KS0108 driver library
// (c) Rados³aw Kwiecieñ, radek@dxp.pl
//-------------------------------------------------------------------------------------------------
#include "KS0108.h"
#include "font5x8.h"
#include "stdint.h"
//-------------------------------------------------------------------------------------------------
extern void GLCD_InitalizePorts(void);
//-------------------------------------------------------------------------------------------------
unsigned char screen_x = 0, screen_y = 0;

/**
 * Setup LCD display.
 * Low layer HW init, and initial setup commands of the LCD driver
 *
 */
void GLCD_Initalize(void)
{
	unsigned char i;
	GLCD_InitalizePorts();

	for(i = 0; i < 2; i++)
	{
		GLCD_EnableController(i);
		GLCD_WriteCommand((DISPLAY_ON_CMD | ON), i);
		GLCD_WriteCommand((DISPLAY_START_LINE), 0);
		GLCD_DisableController(i);
	}

	GLCD_ClearScreen();
	GLCD_GoTo(0,0);
}

/**
 * Move cursor to desired place
 *
 * @param x (offset on the x [0..127])
 * @param y (line [0..7])
 */
void GLCD_GoTo(unsigned char x, unsigned char y)
{
	unsigned char i;
	screen_x = x;
	screen_y = y;

	for(i = 0; i < KS0108_SCREEN_WIDTH/64; i++)
	{
		GLCD_WriteCommand(DISPLAY_SET_Y | 0,i);
		GLCD_WriteCommand(DISPLAY_SET_X | y,i);
		GLCD_WriteCommand(DISPLAY_START_LINE | 0,i);
	}

	GLCD_WriteCommand(DISPLAY_SET_Y | (x % 64), (x / 64));
	GLCD_WriteCommand(DISPLAY_SET_X | y, (x / 64));
}


/**
 * Clear all the screen.
 */
void GLCD_ClearScreen(void)
{
	unsigned char i, j;
	for(j = 0; j < KS0108_SCREEN_HEIGHT/8; j++)
	{
		GLCD_GoTo(0,j);
		for(i = 0; i < KS0108_SCREEN_WIDTH; i++)
		{
			GLCD_WriteData(0x00);
		}
	}
}

/**
 * Write a single character the LCD.
 * Position of the character depends on the current position of the cursor.
 *
 * @param character
 */
void GLCD_WriteChar(char charToWrite)
{
	int i;
	charToWrite -= 32;
	for(i = 0; i < 5; i++)
	{
		uint8_t data = (font5x8[(5 * charToWrite) + i]);
		GLCD_WriteData(data);
	}

	GLCD_WriteData(0x00);
}

/**
 * Write null termianted string to the LCD.
 * Position of the text depends on the current position of the cursor.
 *
 * @param null terminated string
 */
void GLCD_WriteString(char * stringToWrite)
{
	while(*stringToWrite)
	{
		GLCD_WriteChar(*stringToWrite++);
	}
}
