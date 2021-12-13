//*****************************************************************************
//
//! \file KS0108.c
//! \brief
//! \version V0.0.2
//! \date 2/22/2013
//! \author Shadow(gzbkey)
//! \copy
//!
//! Copyright (c)  2012-2013, Shadow(gzbkey)
//! All rights reserved.
//
//*****************************************************************************

#include "KS0108.h"
#include "KS0108_Font.h"
#include "hal.h"

enum KS0108_CMD_DAT
{
    cmd, dat
} KS0108_CMD, KS0108_DAT;

enum KS0108_LEFT_RIGHT
{
    left, right
} KS0108_LEFT, KS0108_RIGHT;

enum KS0108_OFF_ON
{
    off, on
} KS0108_OFF, KS0108_ON;

//volatile int datd;



//*****************************************************************************
//
//! \brief GPIO Configuration KS0108
//!
//! \return None.
//
//*****************************************************************************
void KS0108_GPIO_Configuration(void)
{
	// done in lcd.c
}

//*****************************************************************************
//
//! \brief Delay ms for KS0108
//!
//! \param usTime is delay value.
//!
//! \return None.
//
//*****************************************************************************
void KS0108_Delay(unsigned int uiTime)
{
    unsigned short int i, j;
    for (i = 0; i < uiTime; i++)
        for (j = 0; j < 10000; j++)
        {
        	asm("nop");
        }
}

//*****************************************************************************
//
//! \brief Reset KS0108
//!
//! \return None.
//
//*****************************************************************************
void KS0108_Reset(void)
{
    KS0108_RST_L;
    KS0108_Delay(500);
    KS0108_RST_H;
    KS0108_Delay(500);
}

//*****************************************************************************
//
//! \brief Check busy for KS0108
//!
//! \return None.
//
//*****************************************************************************
void KS0108_Busy(void)
{

    /*  GPIO_InitTypeDef GPIO_InitStructure;
     GPIO_InitStructure.GPIO_Pin   = DB0|DB1|DB2|DB3|DB4|DB5|DB6|DB7;
     GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
     GPIO_Init(DATA_PORT, &GPIO_InitStructure);
     */

    KS0108_RS_L; //di=0;
    KS0108_RW_H; //rw=1;
    KS0108_Delay(100);
    /*  do
     {
     KS0108_WRITE_DATA(0xff00);//LCD12864DataPort=0xff;
     KS0108_E_H;//en=1;
     KS0108_Delay(500);
     datd=KS0108_READ_DATA;
     KS0108_Delay(500);
     KS0108_E_L;//en=0;
     datd=0x8000&datd; //仅当第4,7位为0时才可操作
     }
     while(!(datd==0x00));


     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
     GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
     GPIO_Init(DATA_PORT, &GPIO_InitStructure);*/
}

//*****************************************************************************
//
//! \brief Send data or command to KS0108
//!
//! \param ucDatOrCmd decide send data or command.
//! \param ucSendByte is data(8bit) or command(8bit).
//!
//! ucDatOrCmd can be KS0108_DAT or KS0108_CMD
//!
//! \return None.
//
//*****************************************************************************
void KS0108_Write(unsigned char ucDAT_or_CMD, unsigned char ucLeft_or_Right,
        unsigned char ucData)
{
    uint16_t temp;
    if (ucDAT_or_CMD == KS0108_DAT) //dat
    {
        if (ucLeft_or_Right == KS0108_LEFT) //left
        {
            KS0108_CS1_H; //cs1=1;
            KS0108_CS2_L; //cs2=0;
        }
        else //right
        {
            KS0108_CS1_L; //cs1=0;
            KS0108_CS2_H; //cs2=1;
        }
        KS0108_Delay(10);

        KS0108_Busy();
        KS0108_RS_H; //di=1;
        KS0108_RW_L; //rw=0;
    }
    else //cmd
    {
        KS0108_CS1_H; //cs1=1;
        KS0108_CS2_H; //cs2=1;
        KS0108_Delay(10);

        KS0108_Busy();
        KS0108_RW_L; //rw=0;
        KS0108_RS_L; //di=0;
    }

    KS0108_Delay(10);
    temp = ucData;
    temp = temp << 8;
    KS0108_WRITE_DATA(temp);
    //lcm=a;
    KS0108_Delay(10);
    KS0108_E_H; //e=1;
    KS0108_Delay(10);
    KS0108_E_L; //e=0;
    KS0108_Delay(10);
}

//*****************************************************************************
//
//! \brief Provided to 'void KS0108_Clear(unsigned char ucData)' dedicated
//!
//! \param ucX is row.
//! \param ucY is column.
//! \param ucData is usually 0.
//!
//! \return None.
//
//*****************************************************************************
void KS0108_Write_Screan(char x, char y, unsigned char ucData)
{
    if (x >= 64) /*右屏*/
    {
        x = x - 64; /*设右屏写入数据的地址*/
        x = x + 0x40;
        y = y + 0xb8;

        KS0108_Write(KS0108_CMD, 0, x);
        KS0108_Write(KS0108_CMD, 0, y);
        KS0108_Write(KS0108_DAT, KS0108_RIGHT, ucData); /*写空数据右屏*/
    }
    else
    { /*左屏*/
        x = x + 0x40; /*设左屏幕写入数据的地址*/
        y = y + 0xb8;
        KS0108_Write(KS0108_CMD, 0, x);
        KS0108_Write(KS0108_CMD, 0, y);
        KS0108_Write(KS0108_DAT, KS0108_LEFT, ucData); /*写空数据到左屏*/

    }
}

//*****************************************************************************
//
//! \brief Clear the srceen (display white srceen).
//!
//! \return None.
//
//*****************************************************************************
void KS0108_Clear(unsigned char ucData)
{
    unsigned char x, y;
    for (y = 0; y < 8; y++)
    {
        for (x = 0; x < 128; x++)
        {
            KS0108_Write_Screan(x, y, ucData);
        }
    }
}

//*****************************************************************************
//
//! \brief Open or Close KS0108
//!
//! ucSwitch:KS0108_ON or KS0108_OFF
//!
//! \return None.
//
//*****************************************************************************
void KS0108_Switch(unsigned char ucSwitch)
{
    ucSwitch = ucSwitch + 0x3e;
    KS0108_Write(KS0108_CMD, 0, ucSwitch);
}

//*****************************************************************************
//
//! \brief Initialization KS0108
//!
//! \return None.
//
//*****************************************************************************
void KS0108_Init(void)
{

    KS0108_CMD = cmd;
    KS0108_DAT = dat;
    KS0108_LEFT = left;
    KS0108_RIGHT = right;

    KS0108_OFF = off;
    KS0108_ON = on;

    KS0108_GPIO_Configuration();

    KS0108_Reset();
    KS0108_Switch(KS0108_OFF);
    KS0108_Clear(0);
    KS0108_Switch(KS0108_ON);

    KS0108_Write(KS0108_CMD, 0, 0xc0);
}

//*****************************************************************************
//
//! \brief Display a Chinese character (32*32)
//!
//! \param ucX is row
//! \param ucY is column
//! \param ucChn is Chinese sequence number
//!
//! user_cn[ucChn][]is array of Chinese data
//!
//! \return None.
//
//*****************************************************************************
void KS0108_User_CN(unsigned char ucRow, unsigned char ucColumn,
        unsigned char ucChn)
{
    int i, dx;
    for (i = 0; i < 16; i++)
    {
        dx = HZ[2 * i + ucChn * 32];
        KS0108_Write_Screan(ucRow * 16 + i, ucColumn, dx);
        dx = HZ[(2 * i + 1) + ucChn * 32];
        KS0108_Write_Screan(ucRow * 16 + i, ucColumn + 1, dx);
    }
}

