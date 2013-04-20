/*
 * drivers/misc/anx7150/anx7150_sys.c
 *
 * Drivers for rk28 hdmi
 *
 * Copyright (C) 2010 rockchip
 * zyy@rock-chips.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/rk2818_fb.h>
#include <asm/arch/anx7150.h>

#include "anx7150_i2c.h"
#include "anx7150_sys.h"
#include "anx7150_hw.h"

/**
 * ANX7150_Task - anx7150 state machine
 * @void: void
 *
 * anx7150 state machine
 */
void ANX7150_Task(struct anx7150_dev_s *dev)
{
	int state;
	int ret;

	dev->anx7150_detect = ANX7150_API_DetectDevice();
	if(dev->anx7150_detect == 0)
		goto out;
	
	state = ANX7150_Get_System_State();

	if(dev->hdmi_enable == HDMI_DISABLE && dev->hdmi_auto_switch == HDMI_DISABLE){
		if(state > WAIT_HDMI_ENABLE)
			state = INITIAL;
	}

	if(dev->parameter_config){
		if(state > WAIT_HDMI_ENABLE)
			state = WAIT_HDMI_ENABLE;

		dev->parameter_config = 0;
	}

	state = ANX7150_Interrupt_Process(dev, state);

	switch(state){
	case INITIAL:
		ANX7150_API_Initial();
		state = WAIT_HOTPLUG;
		if(dev->hdmi_auto_switch)
			dev->rate = 1;
		else
			dev->rate = 100;
		break;
		
	case WAIT_HOTPLUG:
		if(dev->HPD_status){
			ANX7150_Plug();
			state = READ_PARSE_EDID;
		}
		if(dev->hdmi_auto_switch)
			dev->rate = 50;
		else
			dev->rate = 100;
		break;
		
	case READ_PARSE_EDID:
		ret = ANX7150_Parse_EDID(dev);
		if(ret != 0){
			E("Parse_EDID err, ret=%d\n", ret);
		}

		dev->resolution_real = ANX7150_Get_Optimal_resolution(dev->resolution_set);

		state = WAIT_RX_SENSE;
		if(dev->hdmi_auto_switch)
			dev->rate = 50;
		else
			dev->rate = 100;

		break;
		
	case WAIT_RX_SENSE:
		if(ANX7150_GET_SENSE_STATE() == 1){
			I("reciver active\n");
			state = WAIT_HDMI_ENABLE;
			dev->reciver_status = HDMI_RECIVER_ACTIVE;
			dev->notifier_callback(dev); /* reciver status change */
		}

		if(dev->hdmi_auto_switch)
			dev->rate = 50;
		else
			dev->rate = 100;

		break;

	case WAIT_HDMI_ENABLE:
		if(dev->rk28_output_status == RK28_OUTPUT_STATUS_HDMI){
			state = SYSTEM_CONFIG;
			dev->rate = 1;
		}
		else{
			if(dev->hdmi_auto_switch)
				dev->rate = 50;
			else
				dev->rate = 100;
		}

		break;
		
	case SYSTEM_CONFIG:
		dev->resolution_real = ANX7150_Get_Optimal_resolution(dev->resolution_set);
		HDMI_Set_Video_Format(dev->resolution_real);
		HDMI_Set_Audio_Fs(dev->i2s_Fs);
		ANX7150_API_HDCP_ONorOFF(dev->hdcp_enable);
		ANX7150_API_System_Config();
		state = CONFIG_VIDEO;

		dev->rate = 1;
		break;
		
	case CONFIG_VIDEO:
		if(ANX7150_Config_Video() == 0){
			if(ANX7150_GET_RECIVER_TYPE() == 1)
				state = CONFIG_AUDIO;
			else
				state = HDCP_AUTHENTICATION;

			dev->rate = 50;
		}
		else{
			dev->rate = 100;
		}
		break;
		
	case CONFIG_AUDIO:
		ANX7150_Config_Audio();
		state = CONFIG_PACKETS;
		dev->rate = 1;
		break;
		
	case CONFIG_PACKETS:
		ANX7150_Config_Packet();
		state = HDCP_AUTHENTICATION;
		dev->rate = 1;
		break;
		
	case HDCP_AUTHENTICATION:
		ANX7150_HDCP_Process();
		state = PLAY_BACK;
		dev->rate = 100;
		break;
		
	case PLAY_BACK:
		ret = ANX7150_PLAYBACK_Process();
		if(ret == 1){
			state = CONFIG_PACKETS;
			dev->rate = 1;
		}
		else{
			dev->rate = 100;
		}
		break;
	
	default:
		state = INITIAL;
		dev->rate = 100;
		break;
	}

	if(state != ANX7150_Get_System_State()){
		ANX7150_Set_System_State(state);
	}

out:
	return;
}
