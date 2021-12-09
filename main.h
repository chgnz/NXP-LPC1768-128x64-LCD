
#ifndef _MAIN_H_
#define _MAIN_H_

enum DEVICE_WORKING_MODE  /* Declares an enumeration data type called BOOLEAN */
	{
		GPS_TEST = 0,
		DUT_TEST = 1,
		CAN_TEST = 2,
		CAN_SIMULATE = 3,
		DUT_SIMULATE = 4,
		DUT_SIMULATE_18xxxx = 5,
		GARMIN_SIMULATE = 6,
		ANALOG = 7,
		IBUTTON_TEST = 8,
		DEV = 9
	} working_mode;

	struct DUT_PAR_STRUCT
		{
			_Bool ATTACHED 	: 1;
			_Bool NO_DATA 	: 1;
			_Bool DONE 		: 1;
			_Bool CHECK 	: 1;
		} ;

struct DUT_PAR_STRUCT DUTE1,DUTE2;


#endif
