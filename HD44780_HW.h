
/*
 * \author no1wudi
 * \file HD44780_HW.h
 */

#ifndef _HD44780_H
#define _HD44780_H

//#include "cmsis_lib/include/stm32f4xx_gpio.h"
//#include "cmsis_lib/include/stm32f4xx_rcc.h"


/*
 * \addtogroup HD44780
 * {
 */

/*
 * \addtogroup Hardware layer
 * {
 */

#define RS_H() GPIO_SetBits(GPIOC,GPIO_Pin_1)
#define RS_L() GPIO_ResetBits(GPIOC,GPIO_Pin_1)

#define RW_H() GPIO_SetBits(GPIOC,GPIO_Pin_3)
#define RW_L() GPIO_ResetBits(GPIOC,GPIO_Pin_3)

#define EP_H() GPIO_SetBits(GPIOA,GPIO_Pin_1)
#define EP_L() GPIO_ResetBits(GPIOA,GPIO_Pin_1)

#define D0_H GPIO_SetBits(GPIOA,GPIO_Pin_3)
#define D0_L GPIO_ResetBits(GPIOA,GPIO_Pin_3)

#define D1_H GPIO_SetBits(GPIOA,GPIO_Pin_5)
#define D1_L GPIO_ResetBits(GPIOA,GPIO_Pin_5)

#define D2_H GPIO_SetBits(GPIOA,GPIO_Pin_7)
#define D2_L GPIO_ResetBits(GPIOA,GPIO_Pin_7)

#define D3_H GPIO_SetBits(GPIOC,GPIO_Pin_5)
#define D3_L GPIO_ResetBits(GPIOC,GPIO_Pin_5)

#define D4_H GPIO_SetBits(GPIOB,GPIO_Pin_1)
#define D4_L GPIO_ResetBits(GPIOB,GPIO_Pin_1)

#define D5_H GPIO_SetBits(GPIOE,GPIO_Pin_7)
#define D5_L GPIO_ResetBits(GPIOE,GPIO_Pin_7)

#define D6_H GPIO_SetBits(GPIOE,GPIO_Pin_9)
#define D6_L GPIO_ResetBits(GPIOE,GPIO_Pin_9)

#define D7_H GPIO_SetBits(GPIOE,GPIO_Pin_11)
#define D7_L GPIO_ResetBits(GPIOE,GPIO_Pin_11)


/*
 * \brief Init the GPIO config of the module
 */

void HD44780_Init();

/*
 * \brief Write data to lcd
 * \param Data The data would be writen to lcd
 */

void HD44780_Write(int Data);

/*
 * \brief Read data from lcd
 * \return Data read.
 */

int HD44780_Read();
/*
 * }
 */

/*
 * }
 */


#endif
