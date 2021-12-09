/*********************************************************************
*	lcd.h
**********************************************************************/

#ifndef _LCD_H_

	#define _LCD_H_

	#include "common.h"
	#include "lpc17xx_GPIO.h"

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

	#define LEFT output_low(CS1_); output_high(CS2_)
	#define RIGHT output_high(CS1_); output_low(CS2_)
	#define space(x) lcd_write_byte(x,1,0x00)

	struct STATUS_READ_STRUCT
	{
	  //  uint8_t    unused   : 5;
		_Bool RESET_STATE 	: 1; //BIT 0, || 1 - RESET, 0 - WORKING				bit0
		_Bool POWER 		: 1; //BIT 1, || 1 - DISPLAY OFF, 0 - DISPLEY ON	bit1
		_Bool BUSY 			: 1; //BIT 2, || 1 - BUSY, 0 - READY				bit2
	} ;

	typedef union
	{
		struct STATUS_READ_STRUCT bit;
		uint32_t all;
	}STATUS_READ_UNION;

	STATUS_READ_UNION lcd_status_union;

	enum LCD_FONTS  /* Declares an enumeration data type called BOOLEAN */
	{
		TXT = 0,
		NUMBERS_XL = 1,
		NUMBERS_S = 2     /* false = 0, true = 1 */
	} FONTS;

	enum LCD_SIDE  /* Declares an enumeration data type called BOOLEAN */
	{
		BOTH_ = 0,
		LEFT_ = 1,
		RIGHT_  = 2     /* false = 0, true = 1 */
	};

	enum PUTS_SPACE  /* Declares an enumeration data type called BOOLEAN */
	{
		WITHOUT_SPACE = 0,
		WITH_SPACE = 1
	};

		// uint8_t abc666[5] ={0x3E, 0x51, 0x49, 0x45, 0x3E		};
//	char garmin_ping_answ[] = {0x10, 0x06 ,0x02 ,0xa1 ,0x00 ,0x57 ,0x10 ,0x03};

	void lcd_write_byte(uint8_t lcd_half_, _Bool data, uint8_t byte_);
	void lcd_init_on_off(_Bool STATE);
	void lcd_goto(uint8_t LINE_, uint8_t X_);
	void lcd_on_off(_Bool ON_OFF_STATE_);
	void lcd_reset(_Bool RST_STATE_);
	void lcd_io_set_directions(void);
	void lcd_clear_page(uint8_t page_start, uint8_t page_cnt, uint8_t x_start, uint8_t x_end);
	uint8_t lcd_status(void);

	void lcd_putc(uint8_t xpos, char char_,uint8_t font_, _Bool space_);
	void lcd_puts(uint8_t xpos, char* string_, uint8_t font_, _Bool space_);

#endif	// _LCD_H_
