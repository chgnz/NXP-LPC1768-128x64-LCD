//
//! Data Pin 0
//
#define DB0  GPIO_Pin_8
//
//! Data Pin 1
//
#define DB1  GPIO_Pin_9
//
//! Data Pin 2
//
#define DB2  GPIO_Pin_10
//
//! Data Pin 3
//
#define DB3  GPIO_Pin_11
//
//! Data Pin 4
//
#define DB4  GPIO_Pin_12
//
//! Data Pin 5
//
#define DB5  GPIO_Pin_13
//
//! Data Pin 6
//
#define DB6  GPIO_Pin_14
//
//! Data Pin 7
//
#define DB7  GPIO_Pin_15

//
//! Data Port
//
#define DATA_PORT  GPIOB

//
//! RS Pin
//
#define RS   GPIO_Pin_6
//
//! RW Pin
//
#define RW   GPIO_Pin_7
//
//! E Pin
//
#define E    GPIO_Pin_8
//
//! CS1 Pin
//
#define CS1  GPIO_Pin_9
//
//! CS2 Pin
//
#define CS2  GPIO_Pin_10
//
//! RST Pin
//
#define RST  GPIO_Pin_11

//
//! Control Port
//
#define CONTROL_PORT  GPIOC

#define KS0108_RS_H   GPIO_SetBits(CONTROL_PORT,RS)
#define KS0108_RS_L   GPIO_ResetBits(CONTROL_PORT,RS)
#define KS0108_RW_H   GPIO_SetBits(CONTROL_PORT,RW)
#define KS0108_RW_L   GPIO_ResetBits(CONTROL_PORT,RW)
#define KS0108_E_H    GPIO_SetBits(CONTROL_PORT,E)
#define KS0108_E_L    GPIO_ResetBits(CONTROL_PORT,E)
#define KS0108_CS1_H  GPIO_SetBits(CONTROL_PORT,CS1)
#define KS0108_CS1_L  GPIO_ResetBits(CONTROL_PORT,CS1)
#define KS0108_CS2_H  GPIO_SetBits(CONTROL_PORT,CS2)
#define KS0108_CS2_L  GPIO_ResetBits(CONTROL_PORT,CS2)
#define KS0108_RST_H  GPIO_SetBits(CONTROL_PORT,RST)
#define KS0108_RST_L  GPIO_ResetBits(CONTROL_PORT,RST)

#define KS0108_WRITE_DATA(DDAT) GPIO_Write(DATA_PORT,DDAT)
#define KS0108_READ_DATA        GPIO_ReadInputData(DATA_PORT)

