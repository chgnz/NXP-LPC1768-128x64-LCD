
#include "lpc17xx_pinsel.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_rit.h"
#include "lpc17xx_GPIO.h"

#include "stdio.h"
#include "stdlib.h"

#include "main.h"
#include "common.h"
#include "lcd.h"
#include "KS0108.h"


#include "string.h"

	#define GPS_RESET_ 1,0 //P1.26
#define LED_PIN 	9
#define LED_PORT 	0

uint32_t time_var1 = 1,time_var2 = 1, time_var3 = 1;
volatile uint32_t systime;

void RIT_IRQHandler(void)
{
    /* call this to clear interrupt flag */
    RIT_GetIntStatus(LPC_RIT);
    systime++;
}

void delay_ms(uint32_t ms)
{
	uint32_t time = systime;

	while(systime - time < ms);
}

/**************************************
 * START OF MAIN FUNCTION
 **************************************/
int main(void)
{
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPIO, ENABLE); // enable power for GPIO

    lcd_io_set_directions();


  	lcd_write_byte(0,0,0b00111110); // deinit lcd


    RIT_Init(LPC_RIT);
    RIT_TimerConfig(LPC_RIT,1);
    NVIC_EnableIRQ(RIT_IRQn);

    time_var1=1;
//	while(time_var1%10 != 0); //  200uS delay, %50 = 1 sek
    delay_ms(100);

  	output_high(RST_);

    time_var1=1;
    delay_ms(1000);
//	while(time_var1%10 != 0);  //  200uS delay, %50 = 1 sek

	lcd_on_off(1); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(0); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(1); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(0); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(1); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(0); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(1); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(0); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(1); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(0); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(1); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(0); // set power on [5v]
//    delay_ms(1000);
//	lcd_on_off(1); // set power on [5v]

    time_var1=1;
//	while(time_var1%10 != 0); //  200uS delay, %50 = 1 sek

  	output_low(CS1_);
  	output_low(CS2_);

  	lcd_write_byte(0,0,0b00111111); // init lcd

    time_var1=1;
//	while(time_var1%150 != 0) //  200uS delay, %50 = 1 sek
//	while(lcd_status() != 0);
	    delay_ms(1000);

  	lcd_write_byte(0,0,0b11000000); // INIT START LINE
  	lcd_clear_page(0,8,0,127); // clear all display
  	lcd_goto(0,0);


// #warning gps_reset doesnt work *ch //    output_high(GPS_RESET_);

    time_var1=1;
//	while(time_var1%25 != 0); //  1 sek delay, %50 = 3 sek

	GLCD_Initalize();
//    GLCD_ClearScreen();     // Clear screen
//    GLCD_GoTo(1,1);
//	switch (lcd_half_)
//	{
//		case 1:	output_low(CS1_); output_high(CS2_); 	break;
//		case 2: output_high(CS1_); output_low(CS2_); 	break;
//		default:output_low(CS1_); output_low(CS2_); 	break;
//	}
	int rpm = 0;
	while(1)
	{
		/*
			lcd_clear_page(0,8,0,127); // clear all display
			lcd_goto(0,90);
			lcd_puts(90, "GPS", TXT, WITH_SPACE);
			lcd_goto(1,90);
			lcd_puts(90, "TEST", TXT, WITH_SPACE);
			lcd_goto(7,90);
			lcd_puts(90, "Ver 2", TXT, WITH_SPACE);
		*/
//		if(time_var2%5 == 0)
		if(1)
		{
			char buf[32];
			memset(&buf, 0, sizeof(buf));
			lcd_goto(0,0);
			sprintf(buf, "-%lu MHz", MLIB_MCU_CLOCK_FREQ / 1000000);
		   GLCD_WriteString(buf);  // Write text
			//lcd_puts(0, "GPS", TXT, WITH_SPACE);

		    GLCD_GoTo(1,1);
			sprintf(buf, "+%u", rpm++);
		    GLCD_WriteString(buf);  // Write text
			time_var2 = 1;

		    delay_ms(100);
		}

	}
}// main
