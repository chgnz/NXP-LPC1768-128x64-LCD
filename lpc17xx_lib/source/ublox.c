/*********************************************************************
*	ublox.C
**********************************************************************/

#include "ublox.h"
#include "uart.h"
#include "stdio.h"

void turn_off_nmea(void)
{
	nmea_print("PUBX,40,GSA,0,0,0,0,0,0");
	delay();
	nmea_print("PUBX,40,GLL,0,0,0,0,0,0");
	delay();
	nmea_print("PUBX,40,RMC,0,0,0,0,0,0");
	delay();
	nmea_print("PUBX,40,VTG,0,0,0,0,0,0");
	delay();
	nmea_print("PUBX,40,GGA,0,0,0,0,0,0");
	delay();
	nmea_print("PUBX,40,GSV,0,0,0,0,0,0");
	delay();
	nmea_print("PUBX,40,TXT,1,1,1,1,1,1");
}

void nmea_print(char string[64])
	{
		int checksum = 0, i;
		static char buffer_tx[64];
		//UART_SendByte(DUT1, strlen(string));

		for (i = 0; i < strlen(string); i++)
			{
				checksum ^= string[i];
			//	UART_SendByte(DUT1, string[i]);
			}

		sprintf(buffer_tx, "$%s*%x\r\n",string,checksum);
		UART_Send(GPS, (uint8_t *)buffer_tx, strlen(buffer_tx),BLOCKING);
	}
