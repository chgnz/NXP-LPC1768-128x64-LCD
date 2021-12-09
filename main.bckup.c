/*
 * 17.01.2013 - pielabots lai pec can busa errora atjaunotos suutishana - CAN BUS SIMULAACIJAA
 *
 */

#include "lpc17xx_uart.h"
#include "lpc17xx_can.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_rit.h"
#include "lpc17xx_GPIO.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_ADC.h"

#include "core_cm3.h"
#include "stdio.h"
#include "stdlib.h"

#include "main.h"
#include "common.h"
#include "uart.h"
#include "can.h"
#include "io.h"
#include "lcd.h"
#include "ublox.h"

#include "math.h"

_Bool ON_OFF = 0;

// #define LED_GG 0,((uint32_t)1<<9)


#define LED_ 0,9

uint8_t UNKNOWN =0;
uint8_t OK =1;
uint8_t SHORT =2;
uint8_t OPEN =3;

uint32_t BUTTON_1 = ((uint32_t)1<<13); 	//P2.12
uint32_t BUTTON_2 = ((uint32_t)1<<12);	//P2.13
uint32_t LED = ((uint32_t)1<<9);		//P0.9

char dut1_buf[64], dut2_buf[64], sat_buf[12][2], gps_buf[128][12];
uint8_t i_dut1, i_dut2, i_gps;

uint8_t satellites_cnt = 0;
uint8_t msg;
uint8_t temp_i,temp_j;
uint8_t i = 0, j=0;
_Bool done_gps = FALSE, multiline = FALSE;
uint8_t x=0, y=0;
uint8_t working_mode;
uint32_t time_var1 = 1,time_var2 = 1, time_var3 = 1;

uint8_t ant_status = 0;



_Bool can_allow = TRUE;

//static CAN_MSG_Type tx_msg;

static char buffer_tx[64]; // GPS SEND BUFFER
static CAN_MSG_Type can_msg;

struct DUT_MSG_STRUCT {
    uint8_t  unused       :3 ;
    _Bool    ping      	  :1 ;
    _Bool    serial_nr    :1 ;
    _Bool    settings     :1 ;
    _Bool    parameters   :1 ;
    _Bool    filteredParameters   :1 ;
};

uint8_t j,k ;
uint8_t crc = 0;
uint8_t byte_;

char working_parameters[] =     {
                                0x3e,0xff,0x23,
                                0xee,0x00,0x18,0x00,0xec,0x27,0x84,0xe6,
                                0x03,0x00,0xff,0xe6,0x03,0x00,0xfc,0xff,
                                0xfc,0xff,0x00,0x00,0x00,0x00,0x14,0x27,
                                0x62,// litri - LSB
                                0x00,// litri - MSB
                                0x00,0x00,0x00,0x00,0xde,0xff,0xff,0xff,
                                0xde,0xff,0xff,0xff,0xde,0xff,0xff,0xff,
                                0x00,0x00,0x00,0x00,0x00,0x00,
                                }; // checksum 0x01//char fuel_level[] = "F=1234 t=56 N=7890.0\r\n";

char filtered_parameters[] =     {
                                0x3e,0xff,0x06,
                                0x55,0x55,0x55,0x55,0x55
                                }; // checksum 0x01

char dut_serial_number[] = {0x3E,0xFF,0x02,0xE3,0x50,0xBA,0x09,0x7D};
char lastIButton[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

char additional_settings[] = {0x3E,0xFF,0x1E,0x00,0x00,0x04,0x01,0x02,0x00,0x14};

char garmin_ping_answ[] = {0x10, 0x06 ,0x02 ,0xa1 ,0x00 ,0x57 ,0x10 ,0x03};

char garmin_parameters[] =
		{
		0x10, 0x06, 0x02, 0xa1, 0x00, 0x57, 0x10, 0x03, 0x10, 0xa1, 0x06, 0x02, 0x00, 0xaf, 0x03, 0x4a, 0x01, 0x5a, 0x10, 0x03, 0x10, 0xa1,
		0x80, 0x03, 0x00, 0x50, 0x00, 0x00, 0x4c, 0x01, 0x00, 0x41, 0x0a, 0x00, 0x41, 0xf4, 0x01, 0x44, 0xf5, 0x01, 0x41, 0x58, 0x02, 0x44,
		0x58, 0x02, 0x41, 0x59, 0x02, 0x44, 0x59, 0x02, 0x41, 0x5a, 0x02, 0x44, 0x5a, 0x02, 0x41, 0x5b, 0x02, 0x44, 0x5b, 0x02, 0x41, 0x5c,
		0x02, 0x44, 0x5c, 0x02, 0x41, 0x5e, 0x02, 0x44, 0x5e, 0x02, 0x41, 0xbc, 0x02, 0x44, 0xbc, 0x02, 0x41, 0x84, 0x03, 0x41, 0x86, 0x03,
		0x41, 0x87, 0x03, 0x41, 0x88, 0x03, 0x41, 0x89, 0x03, 0x44, 0x84, 0x03, 0x41, 0x8b, 0x03, 0x44, 0x8b, 0x03, 0x44, 0x8c, 0x03, 0x44,
		0x8d, 0x03, 0x44, 0x8e, 0x03, 0x41, 0x8c, 0x03, 0x44, 0x8f, 0x03, 0x41, 0x90, 0x03, 0x44, 0x90, 0x03, 0x41, 0x91, 0x03, 0x44, 0x91,
		0x03, 0x41, 0x94, 0x03, 0x41, 0x95, 0x03, 0x44, 0x95, 0x03, 0x41, 0x97, 0x03, 0x41, 0x96, 0x03, 0x44, 0x96, 0x03, 0x12, 0x10, 0x03
		};


typedef union
  {
  struct DUT_MSG_STRUCT bit;
  uint8_t all;
  }DUT_MSG_UNION;

  DUT_MSG_UNION send_msg_to_dut1;
  DUT_MSG_UNION send_msg_to_dut2;
  DUT_MSG_UNION send_msg_to_garmin;


void RIT_IRQHandler(void)
{
    /* call this to clear interrupt flag */
    RIT_GetIntStatus(LPC_RIT);
    time_var1++;
    time_var2++;
    time_var3++;
}


/**************************************************
 * CHEKSUM CALCULATION FOR DUT-E METER
 **************************************************/
void calculate_checksum(void)
{
    crc = 0;
    for(j=0; j < sizeof(working_parameters); j++)
    {
        byte_ = working_parameters[j];
        k = 8;
        do
        {
            ((byte_ ^ crc) & 0x01) ? (crc = ((crc ^ 0x18) >> 1) | 0x80) : (crc >>= 1);
            byte_ >>= 1;
        }
        while (--k);
    }
}

void calculate_checksum_filteredPar(void)
{
    crc = 0;
    for(j=0; j < sizeof(filtered_parameters); j++)
    {
        byte_ = filtered_parameters[j];
        k = 8;
        do
        {
            ((byte_ ^ crc) & 0x01) ? (crc = ((crc ^ 0x18) >> 1) | 0x80) : (crc >>= 1);
            byte_ >>= 1;
        }
        while (--k);
    }
}

/**************************************
 * START OF MAIN FUNCTION
 **************************************/
int main(void)
{
    ant_status = UNKNOWN;
    msg = 0;

    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPIO, ENABLE); // enable power for GPIO

    /* %%% INPUTS %%% */
  //  GPIO_SetDir(2, BUTTON_1 | BUTTON_2	,0); 	// PORT2
   GPIO_SetDir(0, LED ,1); 			// PORT0

    NVIC_DisableIRQ(GPS_IRQ);
    uart_io_setup();

    uart_setup(GPS , 9600);    //DUT
    uart_setup(DUT1 , 19200);    //DUT
    uart_setup(DUT2 , 19200);    //DUT

	PINSEL_CFG_Type PinCfg;
	PinCfg.Portnum = PINSEL_PORT_1;
	PinCfg.Pinnum = PINSEL_PIN_0;
	PinCfg.Funcnum = PINSEL_FUNC_0;
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;

	PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(GPS_RESET_,1); 			// PORT0
	output_low(GPS_RESET_); //izsleedzam gps moduli
    output_high(GPS_RESET_); // iesleedzam gps moduli

    can_io_setup();
    can_bus_setup(LPC_CAN1, 250000);

    output_high(LED_);

    setup_ext_int_io();
    setup_ext_int();


	//UART_SendByte(DUT2, 'b');

    lcd_io_set_directions();
  	lcd_write_byte(0,0,0b00111110); // deinit lcd

  	output_low(RST_);  // reset mode
	lcd_on_off(0); // set power on [5v]

    RIT_Init(LPC_RIT);
    RIT_TimerConfig(LPC_RIT,20);
    NVIC_EnableIRQ(RIT_IRQn);

    time_var1=1;
	while(time_var1%10 != 0); //  200uS delay, %50 = 1 sek

	//UART_SendByte(DUT2, 'A');
  	output_high(RST_); // normal mode

    time_var1=1;
	while(time_var1%10 != 0);  //  200uS delay, %50 = 1 sek

	lcd_on_off(1); // set power on [5v]

    time_var1=1;
	while(time_var1%10 != 0) //  200uS delay, %50 = 1 sek

  	output_low(CS1_);
  	output_low(CS2_);

  	lcd_write_byte(0,0,0b00111111); // init lcd

    time_var1=1;
	while(lcd_status() != 0) //  1 sek delay, %50 = 1 sek
	{
		//UART_SendByte(DUT2,lcd_status() + 0x30);
	}

  	lcd_write_byte(0,0,0b11000000); // INIT START LINE

  	lcd_clear_page(0,8,0,127); // clear all display

  	lcd_goto(0,0);

  	/* <3
  	lcd_write_byte(LEFT_,1,14);
  	lcd_write_byte(LEFT_,1,17);
  	lcd_write_byte(LEFT_,1,33);
  	lcd_write_byte(LEFT_,1,65);
  	lcd_write_byte(LEFT_,1,130);
  	lcd_write_byte(LEFT_,1,65);
  	lcd_write_byte(LEFT_,1,33);
  	lcd_write_byte(LEFT_,1,17);
  	lcd_write_byte(LEFT_,1,14);
  	*/

// #warning gps_reset doesnt work *ch //    output_high(GPS_RESET_);

//	NVIC_DisableIRQ(GPS_IRQ);
//	NVIC_DisableIRQ(DUT1_IRQ);
//	NVIC_DisableIRQ(DUT2_IRQ);
//	__disable_irq();

	NVIC_EnableIRQ(DUT1_IRQ);
	NVIC_EnableIRQ(DUT2_IRQ);
	NVIC_EnableIRQ(EINT2_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
//	__enable_irq();

	DUTE1.NO_DATA = FALSE;
	DUTE2.NO_DATA = FALSE;
	DUTE1.ATTACHED = FALSE;
	DUTE2.ATTACHED = FALSE;
	DUTE1.DONE = FALSE;
	DUTE2.DONE = FALSE;
    DUTE1.CHECK = TRUE;
    DUTE2.CHECK = TRUE;

    time_var1=1;
	while(time_var1%25 != 0); //  1 sek delay, %50 = 3 sek

	NVIC_EnableIRQ(GPS_IRQ);
    turn_off_nmea();

	working_mode = IBUTTON_TEST;
	output_high(BACKLIGHT_);

    while(1)
    {
    	/***********************	GPS TESTING MODE	***********************/
    	if(working_mode == GPS_TEST)
    	{
			lcd_clear_page(0,8,0,127); // clear all display
			lcd_goto(0,90);
			lcd_puts(90, "GPS", TXT, WITH_SPACE);
			lcd_goto(1,90);
			lcd_puts(90, "TEST", TXT, WITH_SPACE);
			lcd_goto(7,90);
			lcd_puts(90, "Ver 2", TXT, WITH_SPACE);

			/* NOSUUTAM GPS ANTENAS KONFIGURAACIJU - open detect, short detect, short auto recovery etc.*/
			clear_buffer(buffer_tx);
			sprintf(buffer_tx, "%c%c%c%c%c%c%c%c%c%c%c%c",0xB5, 0x62 , 0x06 , 0x13 , 0x04 , 0x00 , 0x1F , 0x00 , 0x0F , 0x64 , 0xAF , 0xCB);
			UART_Send(GPS, (uint8_t *)buffer_tx, 12 ,BLOCKING);

    		while(working_mode == GPS_TEST)
    		{
				switch (msg)
				{
					case 0	:	nmea_print("EIGPQ,GGA");nmea_print("EIGPQ,GSA");nmea_print("EIGPQ,GSV"); msg = 1; break;
					default	:	break;
				}

				if (done_gps)
				{
				  if (gps_buf[0][3] == 'G' && gps_buf[0][4] == 'A') // GGA
					{

						if(ant_status == OK) // GPS KOORDINAATES DERIIGAS
						{
							lcd_clear_page(2,1,90,127); // clear all display
							lcd_goto(2,90);
							lcd_puts(90, "ANT OK", TXT, WITH_SPACE);
						}
						else if(ant_status == OPEN) // GPS KOORDINAATES DERIIGAS
						{
							lcd_clear_page(2,1,90,127); // clear all display
							lcd_goto(2,90);
							lcd_puts(90, "OPEN", TXT, WITH_SPACE);
						}
						else if(ant_status == SHORT) // GPS KOORDINAATES DERIIGAS
						{
							lcd_clear_page(2,1,90,127); // clear all display
							lcd_goto(2,90);
							lcd_puts(90, "SHORT", TXT, WITH_SPACE);
						}

						if(gps_buf[6][0] != '0') // GPS KOORDINAATES DERIIGAS
							{
								lcd_goto(3,90);
								lcd_puts(90, "GPS OK", TXT, WITH_SPACE);

								uint8_t aa = 0 ;

								lcd_goto(4,90);
								lcd_puts(90, "SAT ", TXT, WITH_SPACE);
								uint8_t xpos = 114;
								while(gps_buf[7][aa] != 0) // deriigo sateliitu skaits
									{

										lcd_putc(xpos, gps_buf[7][aa], TXT, WITH_SPACE);
										xpos = xpos+6;
										aa++;
									}
								satellites_cnt = atoi(gps_buf[7]);
							}

						else // NAV GPS
						{
								satellites_cnt = 0;

								lcd_goto(3,90);
								lcd_puts(90, "GPS ER", TXT, WITH_SPACE);

								lcd_goto(4,90);
								lcd_puts(90, "SAT ER", TXT, WITH_SPACE);
						}

						clear_buffer(gps_buf);
						msg = 0;
					}

				  //$GPGSA,A,3,01,17,28,18,26,08,11,,,,,,2.46,
				  if (gps_buf[0][3] == 'S' && gps_buf[0][4] == 'A') //GSA
				  {
					clear_buffer(sat_buf);

					  int aa = 0, bb=3;
					  while (gps_buf[bb][aa] != '\0') //$$$$ varbuut aizstaat ar for ciklu lai nepaliek tie dati kur pa vidu nav aizpildiits - 67,67,67,,,,67,utt
						  {
							  while (gps_buf[bb][aa] != '\0')
								  {
									  sat_buf[bb-3][aa] = gps_buf[bb][aa];
									  aa++;
								  }
							  aa = 0;
							  bb++;
							}

						///* DERIIGO SATTELIITU ID PRINTS UZ UART'u */
						bb = 0, aa = 0;
						while (sat_buf[bb][aa] != '\0' && bb < 12) //$$$$ varbuut aizstaat ar for ciklu lai nepaliek tie dati kur pa vidu nav aizpildiits - 67,67,67,,,,67,utt
						{
							while (aa < 2)
							{
								//lcd_putc(0, sat_buf[bb][aa] , NUMBERS_S, WITHOUT_SPACE);
								//UART_SendByte(DUT1, sat_buf[bb][aa]);
								aa++;
							}
							aa = 0;
							bb++;
						}

						clear_buffer(gps_buf);
						msg = 0;
				  }

				  else if (gps_buf[0][3] == 'S' && gps_buf[0][4] == 'V') // GSV
				  {
	/*
						for (temp_i = 0; temp_i < 100 ; temp_i++)	// ZONA
						  {
							UART_Send(DUT1, gps_buf[temp_i], strlen(gps_buf[temp_i]), BLOCKING );
						  }
	*/
					  int aa = 0;

						lcd_clear_page(5,1,90,127); //
						lcd_goto(5,90);
						lcd_puts(90, "TOT ", TXT, WITH_SPACE);
						uint8_t xpos = 114;

					  while (gps_buf[3][aa] != 0) // visu sateliitu skaits
						  {
							  lcd_putc(xpos, gps_buf[3][aa], TXT, WITH_SPACE);
							  xpos = xpos +6;
							  aa++;
						  }

					  lcd_clear_page(0,8,0,89); // clear all display
					  lcd_goto(7,0);

					  int gpgsv_mainiigais = 0;
					  for (temp_i = 0; temp_i < 128; temp_i++)
					  {
						  for (temp_j = 0; temp_j < 12 ; temp_j++)
						  {
							  if (temp_j > 1 && gps_buf[temp_i][temp_j - 1] == 'S' && gps_buf[temp_i][temp_j] == 'V')
							  {
								  gpgsv_mainiigais = temp_i;
							  }

							  if (gps_buf[temp_i][temp_j] == '\r')
							  {
								  int temp_gpgsa = 0;
								  while (temp_gpgsa < 4) // maksimaalais sateliitu skaits vienaa gsv liinijaa
								  {
									  gpgsv_mainiigais += 4;

									//  for (i = 0; i < 12; i++)
									  for (i = 0; i < 12; i++)
									  {
										  if (gps_buf[gpgsv_mainiigais][0] == sat_buf[i][0] && gps_buf[gpgsv_mainiigais][1] == sat_buf[i][1] && gps_buf[gpgsv_mainiigais][0] != '\0')
										  {
											lcd_goto(7,(7*i));

											  for (j = 0; j < 2; j++)	// ZONA
											   {
													if(gps_buf[gpgsv_mainiigais + 3][j] >= '0' && gps_buf[gpgsv_mainiigais + 3][j] <= '9')
													{
														lcd_putc(((7*i)+(3*(j))), gps_buf[gpgsv_mainiigais + 3][j] , NUMBERS_S, WITHOUT_SPACE);
													}
											   }

												uint8_t page_cnt;
												_Bool max_csq = 0;
												page_cnt = atoi(gps_buf[gpgsv_mainiigais+3])/8;
												if(page_cnt > 6) {max_csq = 1;}
												for (j = 0; j < atoi(gps_buf[gpgsv_mainiigais+3])/8; j++)	// ZONA
												  {
														lcd_bar(6-j,(7*i),6,8);
														if(page_cnt == (j+1))
														{
															lcd_goto(5-j,(7*i));
															break;
														}
												}

												if(atoi(gps_buf[gpgsv_mainiigais+3])%8 != 0 && !max_csq)
												{
													lcd_bar(255,(7*i),6,atoi(gps_buf[gpgsv_mainiigais+3])%8);
														//bar_NEW(-1,TEMP2_+(7*i),6,atoi(buffer[gpgsv_mainiigais+3])%8);
												}
											  break;
										  }
									  }
									  temp_gpgsa++;
								  }
							  } // if(\r)
						  } // buffer
					  } // for buffer
					msg = 0;
					clear_buffer(gps_buf);
				  }
				  done_gps = FALSE;
				} // endif done
    		}//while
		}// GPS MODE

    	/***********************	DUT TESTING MODE	***********************/
        else if(working_mode == DUT_TEST)
        {
			lcd_goto(0,0);
			lcd_puts(0, "DUT TEST", TXT, WITHOUT_SPACE);
			lcd_goto(7,90);
			lcd_puts(90, "MAPON", TXT, WITH_SPACE);

        	while(working_mode == DUT_TEST)
			{

        		if(time_var3%25 == 0)
        		{
					char buffer[64];
					memset(buffer,0x00,sizeof(buffer));

					lcd_goto(2,0);
					sprintf(buffer, "%02X%02X%02X%02X%02X%02X%02X",dut1_buf[0],dut1_buf[1],dut1_buf[2],dut1_buf[3],dut1_buf[4],dut1_buf[5],dut1_buf[6]);
					lcd_puts(0, buffer, TXT, WITH_SPACE);

					lcd_goto(3,0);
					memset(buffer,0x00,sizeof(buffer));
					sprintf(buffer, "%02X%02X%02X%02X%02X%02X%02X",dut2_buf[0],dut2_buf[1],dut2_buf[2],dut2_buf[3],dut2_buf[4],dut2_buf[5],dut2_buf[6]);
					lcd_puts(0, buffer, TXT, WITH_SPACE);
        		}
/*
				if(time_var3%200 == 0 && DUTE1.CHECK && DUTE2.CHECK) // ik pa 5 sek pieprasam serijas nr %50 = 1 sek
				{
					//UART_SendByte(DUT2, 'R');

					UART_SendByte(DUT1, 0x31);
					UART_SendByte(DUT1, 0xFF);
					UART_SendByte(DUT1, 0x02);
					UART_SendByte(DUT1, 0x48);

					UART_SendByte(DUT2, 0x31);
					UART_SendByte(DUT2, 0xFF);
					UART_SendByte(DUT2, 0x02);
					UART_SendByte(DUT2, 0x48);

					DUTE1.CHECK = FALSE;
					DUTE2.CHECK = FALSE;
				}
				else if(time_var3%300 == 0 && !DUTE1.CHECK && !DUTE2.CHECK) //gaidam sekundi peec seerijas nr pieprasiijuma atbildi
				{
					if(DUTE1.ATTACHED) // ja sagaidiita, tad interuptu tiek pacelts attached flags
					{
						//UART_Send(DUT2, (uint8_t*)"D1A", 3, BLOCKING );
						lcd_clear_page(2,1,0,127); // clear all display
						lcd_goto(2,0);
						//lcd_puts(0, "DUT2 ", TXT, WITHOUT_SPACE);
						//lcd_puts(25, "ATTACHED", TXT, WITH_SPACE);

						DUTE1.ATTACHED = FALSE;
					}
					else
					{
						//UART_Send(DUT2, (uint8_t*)"D1D", 3, BLOCKING );
						lcd_clear_page(2,1,0,127); // clear all display
						lcd_goto(2,0);
						lcd_puts(0, "DUT2 ", TXT, WITHOUT_SPACE);
						lcd_puts(25, "UNPLUGGED", TXT, WITH_SPACE);
						DUTE1.ATTACHED = FALSE;
					}

					if(DUTE2.ATTACHED)
					{
						//UART_Send(DUT2, (uint8_t*)"D2A", 3, BLOCKING );
						lcd_clear_page(3,1,0,127); // clear all display
						lcd_goto(3,0);
					//	lcd_puts(0, buffer, TXT, WITH_SPACE);
				//	lcd_puts(0, "DUT1 ", TXT, WITHOUT_SPACE);
					//	lcd_puts(25, "ATTACHED", TXT, WITH_SPACE);
						DUTE2.ATTACHED = FALSE;
					}
					else
					{
						//UART_Send(DUT2, (uint8_t*)"D2D", 3, BLOCKING );
						lcd_clear_page(3,1,0,127); // clear all display
						lcd_goto(3,0);
						lcd_puts(0, "DUT1 ", TXT, WITHOUT_SPACE);
						lcd_puts(25, "UNPLUGGED", TXT, WITH_SPACE);
						DUTE2.ATTACHED = FALSE;
					}

					DUTE1.CHECK = TRUE;
					DUTE2.CHECK = TRUE;
					time_var3 = 1;
				}

				if(time_var1%175 == 0 && !DUTE1.NO_DATA) // 3 sek
				{
					DUTE1.NO_DATA = TRUE;
					#ifdef DEBUG_
									//UART_Send(DUT2, (uint8_t*)"ND1", 3, BLOCKING );
					#endif
					lcd_clear_page(5,1,0,127); // clear all display
					lcd_goto(5,0);
					lcd_puts(0, "DUT2 ", TXT, WITHOUT_SPACE);
					lcd_puts(25, "NO DATA RECEIVED", TXT, WITH_SPACE);
				}

				if(time_var2%175 == 0 && !DUTE2.NO_DATA) // 3 sek
				{
					DUTE2.NO_DATA = TRUE;
					#ifdef DEBUG_
									//UART_Send(DUT2, (uint8_t*)"ND2", 3, BLOCKING );
					#endif
					lcd_clear_page(6,1,0,127); // clear all display
					lcd_goto(6,0);
					lcd_puts(0, "DUT1 ", TXT, WITHOUT_SPACE);
					lcd_puts(25, "NO DATA RECEIVED", TXT, WITH_SPACE);
				}

				if(DUTE1.DONE)
				{
					DUTE1.NO_DATA = FALSE;
					time_var1 = 1;
					#ifdef DEBUG_
									//UART_Send(DUT2, (uint8_t*)"d1", 2, BLOCKING );
					#endif
					lcd_clear_page(5,1,0,127); // clear all display
					lcd_goto(5,0);
					lcd_puts(0, "DUT2 ", TXT, WITHOUT_SPACE);
					lcd_puts(25, dut1_buf, TXT, WITHOUT_SPACE);

					DUTE1.DONE = FALSE;
					clear_buffer(dut1_buf);
				}

				if(DUTE2.DONE)
				{
					//UART_Send(DUT2, (uint8_t*) dut2_buf, strlen(dut2_buf), BLOCKING );

					DUTE2.NO_DATA = FALSE;
					time_var2 = 1;

					#ifdef DEBUG_
						   //UART_Send(DUT2, (uint8_t*)"d2", 2, BLOCKING );
					#endif

					lcd_clear_page(6,1,0,127); // clear all display
					lcd_goto(6,0);
					lcd_puts(0, "DUT1 ", TXT, WITHOUT_SPACE);
					lcd_puts(25, dut2_buf, TXT, WITHOUT_SPACE);

					DUTE2.DONE = FALSE;
					clear_buffer(dut2_buf);
				}
				*/
        	}
        }

        else if(working_mode == CAN_TEST)
        {
        	unsigned int rpm = 0, fuel_lvl = 0, distance_total = 0, used_fuel = 0, engine_h = 0, service_km_raw = 0;

			lcd_goto(0,0);
			lcd_puts(0, "CAN TEST", TXT, WITH_SPACE);

			lcd_goto(1,0);
			lcd_puts(0, "RPM", TXT, WITH_SPACE);

			lcd_goto(2,0);
			lcd_puts(0, "FUEL LEVEL", TXT, WITH_SPACE);

			lcd_goto(3,0);
			lcd_puts(0, "SERVICE", TXT, WITH_SPACE);

			lcd_goto(4,0);
			lcd_puts(0, "DISTANCE (m)", TXT, WITH_SPACE);

			lcd_goto(5,0);
			lcd_puts(0, "FUEL USED", TXT, WITH_SPACE);

			lcd_goto(6,0);
			lcd_puts(0, "ENGINE HOURS", TXT, WITH_SPACE);

			//lcd_goto(7,90);
			//lcd_puts(90, "MAPON", TXT, WITH_SPACE);


    		_Bool BUS_ACTIVE;
    		char buf[64];

        	while(working_mode == CAN_TEST)
        	{
				unsigned int global_sts = CAN_GetCTRLStatus(LPC_CAN1, CANCTRL_GLOBAL_STS);

            	if (CHECKBIT(global_sts, CAN_GSR_RBS) && CAN_ReceiveMsg(LPC_CAN1, &can_msg) == SUCCESS)
            	{
        			BUS_ACTIVE = TRUE;

       				switch(can_msg.id)
   				    {
   						case 0x00F00400	: rpm = (int)(((can_msg.dataA[3] << 8) | can_msg.dataB[0])/8); break;  //can_send_msg(LPC_CAN1, 0x00F00400, 0x00000001, 0xA0000000);
   						case 0x00FEFC00	: fuel_lvl = can_msg.dataA[1]; break; // can_send_msg(LPC_CAN1, 0x00FEFC00, 0x00460000, 0x00000000); break; //  Fuel level 07=70
   						case 0x00FEC000	: service_km_raw = ((((can_msg.dataA[1] << 8) | can_msg.dataA[2]))*5); break; // can_send_msg(LPC_CAN1, 0x00FEC000, 0x00200300, 0x00000000) Service distance;
   						case 0x00FEC100	: distance_total = (can_msg.dataA[3] << 24 | can_msg.dataA[2] << 16 | can_msg.dataA[1] << 8 | can_msg.dataA[0])*5 ; break; // can_send_msg(LPC_CAN1, 0x00FEC100, 0x84030000, 0x00000000); Total distance
   						case 0x00FEE900	: used_fuel = ((can_msg.dataB[3] << 24 | can_msg.dataB[2] << 16 | can_msg.dataB[1] << 8 | can_msg.dataB[0]))/2 ; break; // can_send_msg(LPC_CAN1, 0x00FEE900, 0x00200300, 0xE8030000);
   						case 0x00FEE500	: engine_h = ((can_msg.dataA[3] << 24 | can_msg.dataA[2] << 16 | can_msg.dataA[1] << 8 | can_msg.dataA[0]))/20; break; // can_send_msg(LPC_CAN1, 0x00FEE500, 0x4C040000, 0x00000000); can_test = 0; break; //Total engine hours 11=1100;
   				        default	: break;
   				    }
           		}
            	else if(CHECKBIT(global_sts, CAN_GSR_BS)) // check CAN BUS status
				{
					// 1 - bus off
					// 0 - bus on
					//lcd_goto(5,0);
					//lcd_puts(0, "CAN BUS ERROR", TXT, WITH_SPACE);
					//lcd_goto(6,0);
					//lcd_puts(0, "RESETING CAN BUS", TXT, WITH_SPACE);

					CAN_ModeConfig(LPC_CAN1, CAN_RESET_MODE, ENABLE);
					CAN_ModeConfig(LPC_CAN1, CAN_RESET_MODE, DISABLE);
				}


				if(BUS_ACTIVE && time_var1%25 == 0) // time var - RIT interuptaa summeejas , +50 sekundee
				{
					BUS_ACTIVE = FALSE;

					lcd_clear_page(7,0,0,90);
        			lcd_goto(7,0);
        			lcd_puts(0, "CAN BUS OK        ", TXT, WITH_SPACE);

        			if(rpm > 0)
        			{
        				lcd_goto(1,90);
        				sprintf(buf, "%u", rpm);
        				lcd_puts(90, buf, TXT, WITH_SPACE);
        			}

        			if(fuel_lvl > 0 && fuel_lvl != 0xFF)
        			{
        				lcd_goto(2,90);
        				sprintf(buf, "%u", fuel_lvl);
        				lcd_puts(90, buf, TXT, WITH_SPACE);
        			}

        			if(service_km_raw > 160635)
        			{
        				lcd_goto(3,90);
        				sprintf(buf, "%u", service_km_raw - 160635);
        				lcd_puts(90, buf, TXT, WITH_SPACE);
        			}
        			else if(service_km_raw > 0)
        			{
        				lcd_goto(3,90);
        				lcd_puts(90, "ER", TXT, WITH_SPACE);
        			}

        			if(distance_total > 0)
        			{
        				lcd_goto(4,90);
        				sprintf(buf, "%u", distance_total);
        				lcd_puts(90, buf, TXT, WITH_SPACE);
        			}

        			if(used_fuel > 0)
        			{
        				lcd_goto(5,90);
        				sprintf(buf, "%u", used_fuel);
        				lcd_puts(90, buf, TXT, WITH_SPACE);
        			}

        			if(engine_h > 0)
        			{
        				lcd_goto(6,90);
        				sprintf(buf, "%u", engine_h);
        				lcd_puts(90, buf, TXT, WITH_SPACE);
        			}

    				//sprintf(buf, "%u\r\n", distance_total);
    				//UART_Send(DUT2, (uint8_t*)buf, strlen(buf), BLOCKING );
    				time_var1 = 1;
    				time_var2 = 1;
				}

				if(time_var2%250 == 0)
				{
					lcd_clear_page(7,0,0,90);
        			lcd_goto(7,0);
        			lcd_puts(0, "No Data in CAN bus", TXT, WITH_SPACE);
					time_var2 = 1;
				}
        	}
        }

        else if(working_mode == CAN_SIMULATE)
        {
        	int global_sts;
    	//if (CHECKBIT(global_sts, CAN_GSR_BS))
    		//{
    			// uzstÄ�dÄ�m kÄ¼Å«das flagu
    		//	SETBIT(val_flags, val_flag_canerr);
    		//}

			lcd_goto(0,0);
			lcd_puts(0, "CAN SIMULATE max", TXT, WITH_SPACE);

			lcd_goto(2,0);
			lcd_puts(0, "ENGINE HOURS 55", TXT, WITH_SPACE);

			lcd_goto(3,0);
			lcd_puts(0, "FUEL LEVEL 70", TXT, WITH_SPACE);

			lcd_goto(7,90);
			lcd_puts(90, "MAPON", TXT, WITH_SPACE);

    		uint8_t can_test = 0;

        	while(working_mode == CAN_SIMULATE)
        	{
    			//	if(time_var1%15 == 0 && can_allow) // ik pa 150ms time var - RIT interuptaa summeejas , +50 sekundee
    			if(TRUE) // ik pa 150ms time var - RIT interuptaa summeejas , +50 sekundee
				{
					can_test++;
					can_allow = FALSE;

					#ifdef DEBUG_
					   //UART_Send(DUT2, (uint8_t*)"cn", 2, BLOCKING );
					#endif

				    switch(can_test)
				    {
						case 1	: can_send_msg(LPC_CAN1, 0x00F00400, 0x00000001, 0xA0000000); break; // can_send_msg(LPC_CAN1, 0x00F00400, 0x00000001, 0xA0000000)
						case 2	: can_send_msg(LPC_CAN1, 0x00FEFC00, 0x00460000, 0x00000000); break; //  Fuel level 07=70
						case 3	: can_send_msg(LPC_CAN1, 0x00FEC000, 0x00200300, 0x00000000); break; //  Service distance;
						case 4	: can_send_msg(LPC_CAN1, 0x00FEC100, 0x84030000, 0x00000000); break; // can_send_msg(LPC_CAN1, 0x00FEC100, 0x84030000, 0x00000000); Total distance
						case 5	: can_send_msg(LPC_CAN1, 0x00FEE900, 0x00200300, 0xE8030000); break; // can_send_msg(LPC_CAN1, 0x00FEE900, 0x00200300, 0xE8030000);
						case 6	: can_send_msg(LPC_CAN1, 0x00FEE500, 0x4C040000, 0x00000000); can_test = 0; break; //Total engine hours 11=1100;
						//case 6	: can_send_msg(LPC_CAN1, 0x11FEE501, 0x11223344, 0x55667788); can_test = 0; break; //Total engine hours 11=1100;
						default	: can_test = 0; break;
				    }
				}
				//D05=40961,06=40961,07=70,08=800,09=900,10=1000,11=1100;

				if(time_var1%20 == 0) // ik pa 200ms time var - RIT interuptaa summeejas , +50 sekundee
				{
					can_allow = TRUE;

		    		global_sts = CAN_GetCTRLStatus(LPC_CAN1, CANCTRL_GLOBAL_STS);

					if(CHECKBIT(global_sts, CAN_GSR_BS)) // check CAN BUS status
					{
						// 1 - bus off
						// 0 - bus on
						lcd_goto(5,0);
						lcd_puts(0, "CAN BUS ERROR", TXT, WITH_SPACE);
						lcd_goto(6,0);
						lcd_puts(0, "RESETING CAN BUS", TXT, WITH_SPACE);

						CAN_ModeConfig(LPC_CAN1, CAN_RESET_MODE, ENABLE);
						CAN_ModeConfig(LPC_CAN1, CAN_RESET_MODE, DISABLE);
					}
					else
					{
						lcd_goto(5,0);
						lcd_puts(0, "             ", TXT, WITH_SPACE);
						lcd_goto(6,0);
						lcd_puts(0, "                ", TXT, WITH_SPACE);
					}

					time_var1 = 1;
				}
        	}
        }
        else if(working_mode == DUT_SIMULATE)
        {
			lcd_goto(0,0);
			lcd_puts(0, "DUT SIMULATOR v1", TXT, WITH_SPACE);
			lcd_goto(2,0);
			lcd_puts(0, "DUT1 - 111.1 L", TXT, WITHOUT_SPACE);
			lcd_goto(3,0);
			lcd_puts(0, "F=1111 t=11 N=1111.0", TXT, WITHOUT_SPACE);
			lcd_goto(5,0);
			lcd_puts(0, "DUT2 - 222.2 L", TXT, WITHOUT_SPACE);
			lcd_goto(6,0);
			lcd_puts(0, "F=2222 t=22 N=2222.0", TXT, WITHOUT_SPACE);
			lcd_goto(7,90);
			lcd_puts(90, "MAPON", TXT, WITH_SPACE);

        	while(working_mode == DUT_SIMULATE)
        	{
				if(time_var1%50 == 0) // time var - RIT interuptaa summeejas , +50 sekundee
				{
					UART_Send(DUT1, (uint8_t*)"F=1111 t=11 N=0111.0\r\n", 22, BLOCKING);
					UART_Send(DUT2, (uint8_t*)"F=2222 t=22 N=0222.0\r\n", 22, BLOCKING);
					time_var1 = 1;
				}
        	}
        }
        else if(working_mode == DUT_SIMULATE_18xxxx)
        {
			lcd_goto(0,0);
			lcd_puts(0, "DUT_SIMULATE_18xxxx", TXT, WITH_SPACE);
			lcd_goto(2,0);
			lcd_puts(0, "TEST Gbox 18xxxx", TXT, WITH_SPACE);
			lcd_goto(3,0);
			lcd_puts(0, "DUT-E Inputs", TXT, WITH_SPACE);


        	while(working_mode == DUT_SIMULATE_18xxxx)
        	{
        		if(send_msg_to_dut1.all > 0)
				{
					if(send_msg_to_dut1.bit.serial_nr)
					{
	        			lcd_goto(4,0);
        				lcd_puts(0, "1", TXT, WITH_SPACE);
						send_msg_to_dut1.bit.serial_nr = FALSE;
						UART_Send(DUT1, (uint8_t*)dut_serial_number , sizeof(dut_serial_number), BLOCKING);
					}
					if(send_msg_to_dut1.bit.settings)
					{
	        			lcd_goto(5,0);
        				lcd_puts(0, "2", TXT, WITH_SPACE);
						send_msg_to_dut1.bit.settings = FALSE;
						UART_Send(DUT1, (uint8_t*)additional_settings , sizeof(additional_settings), BLOCKING);
					}
					if(send_msg_to_dut1.bit.parameters)
					{
	        			lcd_goto(6,0);
        				lcd_puts(0, "3", TXT, WITH_SPACE);
						send_msg_to_dut1.bit.parameters = FALSE;
						working_parameters[27] = 30; // litri
						calculate_checksum();
						UART_Send(DUT1, (uint8_t*)working_parameters , sizeof(working_parameters), BLOCKING);
						UART_SendByte(DUT1, crc);
					}
					if(send_msg_to_dut1.bit.filteredParameters)
					{
	        			lcd_goto(7,0);
        				lcd_puts(0, "4", TXT, WITH_SPACE);
						send_msg_to_dut1.bit.filteredParameters = FALSE;
						UART_Send(DUT1, (uint8_t*)filtered_parameters , sizeof(filtered_parameters), BLOCKING);
						calculate_checksum_filteredPar();
						UART_SendByte(DUT1, crc);
					}
				}

				if(send_msg_to_dut2.all > 0)
				{
					if(send_msg_to_dut2.bit.serial_nr)
					{
	        			lcd_goto(4,20);
        				lcd_puts(0, "1", TXT, WITH_SPACE);
						send_msg_to_dut2.bit.serial_nr = FALSE;
						UART_Send(DUT2, (uint8_t*)dut_serial_number , sizeof(dut_serial_number), BLOCKING);
					}
					if(send_msg_to_dut2.bit.settings)
					{
	        			lcd_goto(5,20);
        				lcd_puts(0, "2", TXT, WITH_SPACE);
						send_msg_to_dut2.bit.settings = FALSE;
						UART_Send(DUT2, (uint8_t*)additional_settings , sizeof(additional_settings), BLOCKING);
					}
					if(send_msg_to_dut2.bit.parameters)
					{
	        			lcd_goto(6,20);
        				lcd_puts(0, "3", TXT, WITH_SPACE);
						working_parameters[27] = 20; // litri
						calculate_checksum();
						UART_Send(DUT2, (uint8_t*)working_parameters , sizeof(working_parameters), BLOCKING);
						UART_SendByte(DUT2, crc);
					}
					if(send_msg_to_dut2.bit.filteredParameters)
					{
	        			lcd_goto(7,20);
        				lcd_puts(0, "4", TXT, WITH_SPACE);
						send_msg_to_dut2.bit.filteredParameters = FALSE;
						UART_Send(DUT2, (uint8_t*)filtered_parameters , sizeof(filtered_parameters), BLOCKING);
						calculate_checksum_filteredPar();
						UART_SendByte(DUT2, crc);
					}
				}
			}
        }
        else if(working_mode == GARMIN_SIMULATE)
        {
			lcd_goto(0,0);
			lcd_puts(0, "GARMIN TEST", TXT, WITH_SPACE);
			lcd_goto(2,0);
			lcd_puts(0, "TEST Gbox 18xxxx", TXT, WITH_SPACE);
			lcd_goto(3,0);
			lcd_puts(0, "GARMIN Input", TXT, WITH_SPACE);


        	while(working_mode == GARMIN_SIMULATE)
        	{
                if(send_msg_to_garmin.all > 0)
                {
                    if(send_msg_to_garmin.bit.ping)
                    {
                    	send_msg_to_garmin.bit.ping = FALSE;
                        UART_Send(DUT1, (uint8_t*)garmin_ping_answ , sizeof(garmin_ping_answ), BLOCKING);
                        UART_Send(DUT2, (uint8_t*)garmin_ping_answ , sizeof(garmin_ping_answ), BLOCKING);
                    }
                    if(send_msg_to_garmin.bit.serial_nr)
                    {
                        // ja tiek sanjemts pieprasiijums peec garmin serial nr, tad komunikaacija straadaa, vajadzetu isleegt diodei, vai nosuutiit canaa kadu msg'u
                        send_msg_to_garmin.bit.serial_nr = FALSE;
                        UART_Send(DUT1, (uint8_t*)garmin_parameters , sizeof(garmin_parameters), BLOCKING);
                        UART_Send(DUT2, (uint8_t*)garmin_parameters , sizeof(garmin_parameters), BLOCKING);
                    	//can_send_msg(LPC_CAN1, 0x00FEFC00, 0x00460000, 0x00000000);
                    }
                }
        	}
		}
        else if(working_mode == ANALOG)
        {
    		char buf[32];
    		int test = 0;

			lcd_goto(0,0);
			lcd_puts(0, "ANALOG VALUES", TXT, WITH_SPACE);

			initADC();

        	while(working_mode == ANALOG)
        	{

				/*
				if (ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_2,ADC_DATA_DONE) == RESET || ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_2,ADC_DATA_BURST) == RESET)
				{
	    			readADC();
				}
*/
				if (ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_1,ADC_DATA_DONE) == RESET || ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_1,ADC_DATA_BURST) == RESET)
				{
	    			readADC();
				}
        		// ik sekundi nosutam :
				if(time_var1%15 == 0) // time var - RIT interuptaa summeejas , +50 sekundee
				{
					/*
					int AnalogValue = ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_1);
					int batteryVoltage = AnalogValue * 5000 / 4096;
    				//sprintf(buf, "%f", batteryVoltage);
    				sprintf(buf, "Battery : %d mV", batteryVoltage);
					lcd_clear_page(1,1,0,123);
    				lcd_goto(1,0);
    				lcd_puts(0, buf, TXT, WITH_SPACE);
*/

					int AnalogValue = 2 * ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_2);

					test++;
					lcd_clear_page(1,1,0,123);
    				sprintf(buf, "RPM : %4u", AnalogValue);

    				lcd_goto(1,0);
    				lcd_puts(0, buf, TXT, WITH_SPACE);

    				int global_sts = CAN_GetCTRLStatus(LPC_CAN1, CANCTRL_GLOBAL_STS);

    				if(CHECKBIT(global_sts, CAN_GSR_BS)) // check CAN BUS status
    				{
    					// 1 - bus off
    					// 0 - bus on
    					lcd_goto(6,0);
    					lcd_puts(0, "RESETING CAN BUS", TXT, WITH_SPACE);

    					CAN_ModeConfig(LPC_CAN1, CAN_RESET_MODE, ENABLE);
    					CAN_ModeConfig(LPC_CAN1, CAN_RESET_MODE, DISABLE);
    				}
    				else
    				{
    					lcd_clear_page(6,1,0,123);
    				}

    				AnalogValue = 8 * AnalogValue;

    	    		can_send_msg(LPC_CAN1, 0x00F00400, AnalogValue & 0xFF, (AnalogValue >> 8) << 24);
    		//		can_send_msg(LPC_CAN1, 0x00FEFC00, AnalogValue, 0x00000000);//Total engine hours 11=1100;


    				time_var1 = 1;
				}
        	}
        }

        else if(working_mode == IBUTTON_TEST)
        {
        	uint8_t iButtonPrint[128];
        	uint8_t batteryVoltageBuffer[128];
        	clear_buffer(iButtonPrint);
        	clear_buffer(batteryVoltageBuffer);
    		lcd_goto(0,0);
    		lcd_puts(0, "IBUTTON TEST", TXT, WITH_SPACE);
    		char inPorgressChar[8] = {'-','-','-','-','+','+','+','+'};
    		char inPorgressIndex = 0;
    		int line = 0;

			initADC();

        	while(working_mode == IBUTTON_TEST)
        	{

				if (ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_1,ADC_DATA_DONE) == RESET || ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_1,ADC_DATA_BURST) == RESET)
				{
	    			readADC();
				}
        		// ik sekundi nosutam :
				int AnalogValue = ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_1);
				int batteryVoltage = AnalogValue * 5000 / 4096;

    			sprintf(batteryVoltageBuffer, "Battery : %d mV", batteryVoltage);
				lcd_clear_page(7,1,0,123);
    			lcd_goto(7,0);
    			lcd_puts(0, batteryVoltageBuffer, TXT, WITH_SPACE);

    			line = 1 + (line++ & 0x03);
        		if(touch_present())
        		{
        			lcd_goto(line,0);

        			GBOX_IO_OneWireSlaveWriteByte(0x33);

        			int i ;
        			uint8_t iButton[8];
                	clear_buffer(iButton);
                	clear_buffer(iButtonPrint);

        			for(i = 0; i < 8; ++i)
        			{
        				iButton[i] = 0x00;
        				iButton[i]=	GBOX_IO_OneWireSlaveReadByte();
        			}
        			uint8_t CRC = 0;
        			CRC = calc_crc(iButton, 7);

        			if (CRC == iButton[7])
        			{
        				if(memcmp(iButton,lastIButton,8) != 0)
        				{
							memcpy(lastIButton,iButton,8);

							lcd_clear_page(5,1 ,0,127);
							lcd_goto(5,0);
	        				sprintf(iButtonPrint, "%02X%02X%02X%02X%02X%02X%02X",lastIButton[0],lastIButton[6],lastIButton[5],lastIButton[4],lastIButton[3],lastIButton[2],lastIButton[1]);
	            			lcd_puts(0, iButtonPrint, TXT, WITH_SPACE);
        				}
        				sprintf(iButtonPrint, "%02X%02X%02X%02X%02X%02X%02X %c %02X",iButton[0],iButton[6],iButton[5],iButton[4],iButton[3],iButton[2],iButton[1],inPorgressChar[(inPorgressIndex++ & 0x07)],CRC);
        			}
        			else
        				sprintf(iButtonPrint, "CRC ERROR %c %02X",inPorgressChar[(inPorgressIndex++ & 0x07)],CRC);
        		}
        		else
        		{
    				sprintf(iButtonPrint, "NOT FOUND      %c   ",inPorgressChar[(inPorgressIndex++ & 0x07)]);
        		}

        		lcd_clear_page(line,1 ,0,127);
    			lcd_goto(line,0);
    			lcd_puts(0, iButtonPrint, TXT, WITH_SPACE);

        		MLIB_Sleep(333);
        	}
        }
    	else if (working_mode == DEV)
    	{
    		lcd_goto(0,0);
    		lcd_puts(0, "DEV TEST", TXT, WITH_SPACE);

    		while(1)
    		{
				if(time_var1%50 == 0) // time var - RIT interuptaa summeejas , +50 sekundee
				{
    				//sprintf(buf, "%f", batteryVoltage);
					char buf[64];
    				sprintf(buf, "%u, %u", i_dut1, i_dut2);
					lcd_clear_page(1,1,0,123);
    				lcd_goto(1,0);
    				lcd_puts(0, buf, TXT, WITH_SPACE);
				}
    		}
    	}
    } // while(1) loop
}// main

/*********************************************************************//**
 * @brief		External interrupt 0 handler sub-routine
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void EINT3_IRQHandler(void) // button 1
{
	/* clear the EINT0 flag */
	EXTI_ClearEXTIFlag(EXTI_EINT3);


//	sprintf(buffer_tx, "%c%c%c%c%c%c%c%c%c%c%c%c",0xB5, 0x62 , 0x06 , 0x13 , 0x04 , 0x00 , 0x1F , 0x00 , 0x0F , 0x64 , 0xAF , 0xCB);
	//  output_toogle(GPS_RESET_);

	//output_toogle(BACKLIGHT_);
	NVIC_SystemReset();
	//output_toogle(LCD_PWR_); // set power on [5v]

}

void EINT2_IRQHandler(void) // button 2
{
	/* clear the EINT0 flag */
	EXTI_ClearEXTIFlag(EXTI_EINT2);

	NVIC_DisableIRQ(GPS_IRQ);
	NVIC_DisableIRQ(DUT1_IRQ);
	NVIC_DisableIRQ(DUT2_IRQ);
	NVIC_DisableIRQ(RIT_IRQn);

	clear_buffer(sat_buf);
	clear_buffer(gps_buf);
  	lcd_clear_page(0,8,0,127); // clear all display

	DUTE1.DONE = FALSE;
	clear_buffer(dut1_buf);
	i_dut1 = 0;

	DUTE2.DONE = FALSE;
	clear_buffer(dut2_buf);
	i_dut2 = 0;

	time_var1 = 1;
	time_var2 = 1;
	time_var3 = 1;
	//	NVIC_DisableIRQ(EINT2_IRQn);


	working_mode++;
	switch (working_mode)
	{
		case DUT_TEST				:  uart_setup(DUT1 , 19200);
		   	   	   	   	   	   	   	   uart_setup(DUT2 , 19200);
		   	   	   	   	   	   	   	   NVIC_EnableIRQ(DUT1_IRQ); NVIC_EnableIRQ(DUT2_IRQ); NVIC_EnableIRQ(RIT_IRQn); time_var1 = 1;time_var2 = 1; DUTE1.NO_DATA = FALSE; DUTE2.NO_DATA = FALSE; break;
		case CAN_TEST				:  NVIC_EnableIRQ(RIT_IRQn); break;
		case CAN_SIMULATE			:  NVIC_EnableIRQ(RIT_IRQn); break;
		case DUT_SIMULATE			:  uart_setup(DUT1 , 19200);
									   uart_setup(DUT2 , 19200);
									   NVIC_EnableIRQ(RIT_IRQn); time_var1 = 1; break;
		case DUT_SIMULATE_18xxxx	:  uart_setup(DUT1 , 19200);
 	   	   	   	   	   	   	   	   	   uart_setup(DUT2 , 19200);
 	   	   	   	   	   	   	   	   	   NVIC_EnableIRQ(DUT1_IRQ); NVIC_EnableIRQ(DUT2_IRQ); NVIC_EnableIRQ(RIT_IRQn); break;
		case GARMIN_SIMULATE		:  uart_setup(DUT1 , 9600);    //GARMIN
	    							   uart_setup(DUT2 , 9600);    //GARMIN
	    							   NVIC_EnableIRQ(DUT1_IRQ); NVIC_EnableIRQ(DUT2_IRQ); NVIC_EnableIRQ(RIT_IRQn); break;
		case ANALOG					:  NVIC_EnableIRQ(RIT_IRQn); time_var1 = 1; break;
		case IBUTTON_TEST			:  NVIC_EnableIRQ(RIT_IRQn); output_toogle(BACKLIGHT_); break;
	//	case DEV					:  uart_setup(DUT1 , 19200);
	 //  	   	   	   	   	   	   	   	   uart_setup(DUT2 , 19200);
	 //  	   	   	   	   	   	   	   	   NVIC_EnableIRQ(DUT1_IRQ); NVIC_EnableIRQ(DUT2_IRQ); NVIC_EnableIRQ(RIT_IRQn); time_var1 = 1; break;
		default						:  working_mode = GPS_TEST; NVIC_EnableIRQ(GPS_IRQ); msg = 0;	break; // ja mianiigais working mode paarsniedz robezhu - nonullejam.
	}

	//UART_SendByte(DUT2, working_mode + 0x30);
}

void UART0_IRQHandler() /* %%% GPS %%% */
{
    i_gps++;
	char c;

    LPC_UART_TypeDef *UARTx = LPC_UART0;
	c = UART_ReceiveByte(UARTx);

	if(c=='K') // OK, OPEN, SHORT
	{
		if(gps_buf[x][y-1] == 'O')	ant_status = OK;
	}
	if(c=='N') // OK, OPEN, SHORT
	{
		if(gps_buf[x][y-1]== 'E')	ant_status = OPEN;
	}
	if(c=='T') // OK, OPEN, SHORT
	{
		if(gps_buf[x][y-1] == 'R')	ant_status = SHORT;
	}

	if(c=='$')
	{
		if(multiline)
		{
			x++;
			y=0;
		}
		else
		{
			x=0;
			y=0;
		}
	}
	else if(c==0x0A)
	{
		if(multiline)
		{
		}
		else
		{
			done_gps=TRUE;
		}
	}
	else if(c==','||c=='*')
	{
		x++;
		y=0;
	}
	else
	{
		gps_buf[x][y]=c;
		y++;
	}

          if (x>=2 && multiline && c != ',' && gps_buf[x - 2][2] == 'G' && gps_buf[x - 2][3] == 'S' && gps_buf[x - 2][4] == 'V')
          {
          	if (gps_buf[x][0] == gps_buf[x - 1][0])
          	{
          		multiline = FALSE;
          	}
          }
          else if (x>=2 && !multiline && c != ',' && gps_buf[x - 2][2] == 'G' && gps_buf[x - 2][3] == 'S' && gps_buf[x - 2][4] == 'V')
          {
          	if (gps_buf[x][0] != gps_buf[x - 1][0])
          	{
          		multiline = TRUE;
          	}
          }
    NVIC_ClearPendingIRQ(UART0_IRQn);
}

void UART2_IRQHandler() /* %%% DUT 2 %%% */
{
	char c_dut2;

    LPC_UART_TypeDef *UARTx = LPC_UART2;
    c_dut2 = UART_ReceiveByte(UARTx);

	if(working_mode == DUT_TEST)
	{
		if(i_dut2 > 6 && dut2_buf[i_dut2-5] == 02 && dut2_buf[i_dut2-7] == 0x3E) //ATBILDE
		{
				//UART_SendByte(DUT2,'X');
				DUTE2.ATTACHED = TRUE;
				i_dut2 = 0;
		}
		else if(c_dut2 == 0x0A || c_dut2 == 0x0D)
		{
			if(i_dut2 > 0) //ja ienÄ�k peec kaartas 0x0a un 0x0d tad done tiek padots tikai vinreiz
			{
				DUTE2.DONE = TRUE;
			}
			i_dut2 = 0;
		}
		else
		{
			dut2_buf[i_dut2] = c_dut2;
			i_dut2++;
		}
	}
	else if(working_mode == DUT_SIMULATE_18xxxx)
	{
	    if(dut2_buf[i_dut2-3] == 0x31 && dut2_buf[i_dut2-2] == 0xFF ) // ja atnaak \r vai \n
	    {
	        switch(dut2_buf[i_dut2-1])
	        {
	            case 0x02    :    send_msg_to_dut2.bit.serial_nr = TRUE;     i_dut2=0;     break;
	            case 0x1E    :    send_msg_to_dut2.bit.settings = TRUE;     i_dut2=0;     break;
	            case 0x23    :    send_msg_to_dut2.bit.parameters = TRUE; i_dut2=0;     break;
	            case 0x06    :    send_msg_to_dut2.bit.filteredParameters = TRUE; i_dut2=0;     break;
	            default     :     i_dut2=0; break;
	        }
	    }
	    else
	    {
			dut2_buf[i_dut2] = c_dut2;
			i_dut2++;
	    }
	}
	else if(working_mode == GARMIN_SIMULATE)
	{
		dut2_buf[i_dut2] = c_dut2;

		if(dut2_buf[i_dut2] == 0x03 && dut2_buf[i_dut2-1] == 0x10 )
		{
			switch(dut2_buf[i_dut2-2])
			{
				case 0xFB    :    send_msg_to_garmin.bit.ping = TRUE;      	i_dut2=0;     break;    /*PING*/
				case 0x5C    :    send_msg_to_garmin.bit.serial_nr = TRUE;  i_dut2=0;     break;    /*SERIAL*/
				case 0x52    :    send_msg_to_garmin.bit.ping = TRUE;		i_dut2=0;     break;    /*PING*/
				default     :     i_dut2=0; break;
			}
		}
		else
		{
			i_dut2++;
		}
	}
	else if (working_mode == DEV)
	{
		if(i_dut2 > 63)
			i_dut2 = 0;

		dut2_buf[i_dut2] = c_dut2;
		i_dut2++;
	}

    NVIC_ClearPendingIRQ(UART2_IRQn);
}

void UART3_IRQHandler() /* %%% DUT 1 %%% */
{
	char c_dut1;

    LPC_UART_TypeDef *UARTx = LPC_UART3;
    c_dut1 = UART_ReceiveByte(UARTx);

	if(working_mode == DUT_TEST)
	{
		//UART_SendByte(DUT2,c_dut1);
		if(i_dut1 > 6 && dut1_buf[i_dut1-5] == 02 && dut1_buf[i_dut1-7] == 0x3E) //ATBILDE
		{
				//UART_SendByte(DUT2,'X');
				DUTE1.ATTACHED = TRUE;
				i_dut1 = 0;
		}
		else if(c_dut1 == 0x0A || c_dut1 == 0x0D)
		{
			if(i_dut1 > 0) //ja ienÄ�k peec kaartas 0x0a un 0x0d tad done tiek padots tikai vinreiz
			{
				DUTE1.DONE = TRUE;
			}
			i_dut1 = 0;
		}
		else
		{
			dut1_buf[i_dut1] = c_dut1;
			i_dut1++;
		}
	}
	else if(working_mode == DUT_SIMULATE_18xxxx)
	{
	    if(dut1_buf[i_dut1-3] == 0x31 && dut1_buf[i_dut1-2] == 0xFF ) // ja atnaak \r vai \n
	    {
	        switch(dut1_buf[i_dut1-1])
	        {
	            case 0x02    :    send_msg_to_dut1.bit.serial_nr = TRUE;     i_dut1=0;     break;
	            case 0x1E    :    send_msg_to_dut1.bit.settings = TRUE;     i_dut1=0;     break;
	            case 0x23    :    send_msg_to_dut1.bit.parameters = TRUE; i_dut1=0;     break;
	            case 0x06   :    send_msg_to_dut1.bit.filteredParameters = TRUE; i_dut1=0;     break;
	            default     :     i_dut1=0; break;
	        }
	    }
	    else
	    {
			dut1_buf[i_dut1] = c_dut1;
			i_dut1++;
	    }
	}
	else if(working_mode == GARMIN_SIMULATE)
	{
		dut1_buf[i_dut1] = c_dut1;

		if(dut1_buf[i_dut1] == 0x03 && dut1_buf[i_dut1-1] == 0x10 )
		{
			switch(dut1_buf[i_dut1-2])
			{
				case 0xFB    :    send_msg_to_garmin.bit.ping = TRUE;      	i_dut1=0;     break;    /*PING*/
				case 0x5C    :    send_msg_to_garmin.bit.serial_nr = TRUE;  i_dut1=0;     break;    /*SERIAL*/
				case 0x52    :    send_msg_to_garmin.bit.ping = TRUE;		i_dut1=0;     break;    /*PING*/
				default     :     i_dut1=0; break;
			}
		}
		else
		{
			i_dut1++;
		}
	}
	else if (working_mode == DEV)
	{
		if(i_dut1 > 63)
			i_dut1 = 0;

		dut1_buf[i_dut1] = c_dut1;
		i_dut1++;
	}

    NVIC_ClearPendingIRQ(UART3_IRQn);
}
