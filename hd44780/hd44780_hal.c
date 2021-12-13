#include "lcd_hal.h"

#include "lpc17xx_GPIO.h"

#include <stdint.h>
#include <stdbool.h>

	#define  E_ 0,27		//P0.27
	#define  BACKLIGHT_ 0,28 //P0.28
	#define  RST_ 0,29 		//P0.29
	#define  CS2_ 0,30 		//P0.30
	#define  LED_ 0,9 		//P0.30

	/*PORT 1*/
	#define CS1_ 1,18 //P1.18
	#define D0_ 1,19 //P1.19
	#define D1_ 1,20 //P1.20
	#define D2_ 1,21 //P1.21
	#define D3_ 1,22 //P1.22
	#define D4_ 1,23 //P1.23
	#define D5_ 1,24 //P1.24
	#define D6_ 1,25 //P1.25
	#define D7_ 1,26 //P1.26

	/*PORT 2*/
	#define LCD_PWR_ 2,11 //P2.11

	/*PORT 3*/
	#define RS_ 3,26	//P3.26
	#define RW_ 3,25	//P3.25

void hd44780_RS_On()
{
}

void hd44780_RS_Off();
void hd44780_RW_On();
void hd44780_RW_Off();
void hd44780_EN_On();
void hd44780_EN_Off();

void hd44780_SetD0(bool state);
void hd44780_SetD1(bool state);
void hd44780_SetD2(bool state);
void hd44780_SetD3(bool state);
void hd44780_SetD4(bool state);
void hd44780_SetD5(bool state);
void hd44780_SetD6(bool state);
void hd44780_SetD7(bool state);

