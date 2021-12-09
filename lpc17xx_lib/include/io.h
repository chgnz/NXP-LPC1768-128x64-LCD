/*********************************************************************
*	IO.H
**********************************************************************/
#ifndef _IO_

	#define _IO_

	#include "common.h"

	#define GPS_RESET_ 1,0 //P1.26

	void setup_ext_int(void);
	void setup_ext_int_io(void);


//	uint32_t BUTTON_1 = ((uint32_t)1<<13); 	//P2.12
//	uint32_t BUTTON_2 = ((uint32_t)1<<12);	//P2.13


#endif	// _IO_
