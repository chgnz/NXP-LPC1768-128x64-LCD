/*********************************************************************
*	CAN.C
**********************************************************************/

#include "can.h"

void can_io_setup(void)
{
// 	set_pin_function(PORT , PIN , FUNC);

	set_pin_function(0,21,3); // RX CAN1
	set_pin_function(0,22,3); // TX CAN1

	//set_pin_function(0,4,2); // RX CAN1
	//set_pin_function(0,5,2); // TX CAN1
}


void can_bus_setup(LPC_CAN_TypeDef *CANx, uint32_t BAURATE_)
{
//	#define CAN1_BITRATE	250000			///< Atrums, bps
//	#define CAN1_CHANNEL	LPC_CAN1		///< CAN kanals #1

	CAN_Init(CANx, BAURATE_);
	CAN_ModeConfig(CANx, CAN_OPERATING_MODE, ENABLE);
	CAN_SetAFMode(LPC_CANAF, CAN_AccBP); // filtri izslegti

	//CAN_IRQCmd(CAN2_CHANNEL, CANINT_RIE, ENABLE); // RECEIVE INT ENABLE
	//NVIC_EnableIRQ(CAN_IRQn); /* Enable CAN Interrupt */
}

static CAN_MSG_Type tx_msg;

void can_send_msg(LPC_CAN_TypeDef *CANx, uint32_t ID_, uint32_t data_A, uint32_t data_B)
{

  tx_msg.format = EXT_ID_FORMAT;
  tx_msg.id = ID_;
  tx_msg.len = 8;
  tx_msg.type = DATA_FRAME;

  tx_msg.dataA[0] = (int)((data_A & 0xFF000000) >> 24);
  tx_msg.dataA[1] = (int)((data_A & 0x00FF0000) >> 16);
  tx_msg.dataA[2] = (int)((data_A & 0x0000FF00) >> 8);
  tx_msg.dataA[3] = (int)((data_A & 0x000000FF) >> 0);

  tx_msg.dataB[0] = (int)((data_B & 0xFF000000) >> 24);
  tx_msg.dataB[1] = (int)((data_B & 0x00FF0000) >> 16);
  tx_msg.dataB[2] = (int)((data_B & 0x0000FF00) >> 8);
  tx_msg.dataB[3] = (int)((data_B & 0x000000FF) >> 0);


  CAN_SendMsg(CANx, &tx_msg);
}
