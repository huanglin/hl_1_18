/*
 * rockchip_dsp.h  --  Dsp for rockchip
 *
 * Driver for rockchip dsp
 *  Copyright (C) 2009 lhh lhh@rock-chips.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#ifndef __PLAT_ROCKCHIP_DSP_DSP_H
#define __PLAT_ROCKCHIP_DSP_DSP_H


#define PIU_CMD0_OFFSET         (0x30)
#define PIU_REPLY0_OFFSET       (0x3c)
#define PIU_STATUS_OFFSET       (0x4c)
#define PIU_IMASK_OFFSET        (0x48)
#define PIU_STATUS_R0WRS        3


#define CODEC_OUTPUT_PIU_CHANNEL        0
#define CODEC_MSG_PIU_CHANNEL           1
#define CODEC_MSG_PIU_NEXT_CHANNEL      2
#define CODEC_MSG_ICU_CHANNEL           3


#define DSP_IOCTL_RES_REQUEST           (0x00800000)
#define DSP_IOCTL_RES_RELEASE           (0x00800001)
#define DSP_IOCTL_SEND_MSG              (0x00800002)
#define DSP_IOCTL_RECV_MSG              (0x00800003)
#define DSP_IOCTL_SET_FREQ              (0x00800004)
#define DSP_IOCTL_GET_TABLE_PHY         (0x00800005)

#define DSP_IOCTL_DOWNLOAD_FIRMWARE_KERNEL  (0x00800021)     //add by hl
#define DSP_IOCTL_REGIST_IRQ_KERNEL         (0x00800022)     //add by hl
#define DSP_IOCTL_EXIT_KERNEL               (0x00800023)     //add by hl
#define DSP_IOCTL_EPD_SEND_MSG      (0x00800024)
#define DSP_IOCTL_EPD_STANDBY      (0x00800025)
struct rk28dsp_req {
	int reqno;
	char fwname[20];
	int freq;
};

struct rk28dsp_msg {
	int channel;
	uint32 cmd;
	int rcv_timeout;    // 0:no block   -1:block   >0:block with timeout
};

extern void rockchip_add_device_dsp(void);

#endif /* __PLAT_ROCKCHIP_DSP_DSP_H *//* insmod /system/lib/modules/rk28dsp.ko */




