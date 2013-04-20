/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File 	:	eink.h
Desc 	:
Author 	:
Date 	:
Notes 	:
$Log: Rk_EPD_CTRL.h,v $
Revision 1.1.1.1  2010/05/11 01:58:54  hjk
no message

********************************************************************/

#ifndef _MCU_MCU_RK_EPD_H
#define _MCU_MCU_RK_EPD_H
extern void Rk_EPD_CTRL_SetAlpha(u8 enable);
extern void Rk_EPD_CTRL_SetMode(u8 mode);
extern void Rk_EPD_CTRL_ReFlush(u8 mode);   
extern int   Rk_EPD_CTRL_GetMode();
extern int   Rk_EPD_CTRL_Enable_Force(u8 enable);
extern int   Rk_EPD_CTRL_Pan_Init(void);
extern int   Rk_EPD_CTRL_Pan_Deinit(void);
extern int   Rk_EPD_CTRL_Get_Pan_Buf(u32* bufaddr,u32*bufaddrphy,u32* w, u32* h);
extern int   Rk_EPD_CTRL_Alpha_Init(u32 bufaddr);
extern int   Rk_EPD_CTRL_Get_Status();
extern int   Rk_EPD_CTRL_Reset(void);
extern int   Rk_EPD_CTRL_GetRotate(void);
extern int   Rk_EPD_Wave_num(char *buf);
extern void RK_EPD_SET_Idle_Time(int time_count);
extern void Rk_EPD_Refresh_White_Panel();
extern void Rk_EPD_CTRL_SetDither(unsigned char status);
extern void   Eink_s_power_on(void);
extern void   Eink_s_power_down(void);
#endif


