
#include <stdio.h>	 // sprintf

#include "KS0108.h"
#include "systime.h"

int main(void)
{
	GLCD_Initalize();
	DelayInitalize();

	int rpm = 0;
	char buf[32];

	while(1)
	{
		GLCD_GoTo(3,3);
		sprintf(buf, "Janis Olehnovics");
		GLCD_WriteString((char*)&buf);

		GLCD_GoTo(5,5);
		sprintf(buf, "abcdef %03u, %u", rpm++, 1);
		GLCD_WriteString((char*)&buf);

		DelayMs(1000);
	}
}
