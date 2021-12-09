/*********************************************************************
*	CAN.H
**********************************************************************/

#ifndef _CAN_H_

	#define _CAN_H_

	#include "common.h"
	#include "lpc17xx_can.h"

	//static CAN_MSG_Type tx_msg;

	void can_io_setup(void);
	void can_bus_setup(LPC_CAN_TypeDef *CANx, uint32_t BAURATE_);
	void can_send_msg(LPC_CAN_TypeDef *CANx, uint32_t ID_, uint32_t data_A, uint32_t data_B);

#endif	// _CAN_H_
