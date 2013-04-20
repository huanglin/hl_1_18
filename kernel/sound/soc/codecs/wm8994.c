/*
 * wm8994.c -- WM8994 ALSA SoC audio driver
 *
 * Copyright (C) 2009 rockchip lhh
 *
 *
 * Based on WM8994.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>

#include "wm8994.h"
//#include "../../../drivers/gpio_extend/gpio_extend.h"

#define AUDIO_NAME "WM8994"
#define WM8994_VERSION "0.1"

/* Debug */
#if 1
#define	DBG(x...)	printk(KERN_INFO x)
#else
#define	DBG(x...)
#endif

/* If digital BB is used,open this define. */
//#define PCM_BB

/* Define what kind of digital BB is used. */
#ifdef PCM_BB
#define TD688_MODE  
//#define MU301_MODE
//#define CHONGY_MODE
//#define THINKWILL_M800_MODE
#endif //PCM_BB

#define wm8994_mic_VCC 0x0020

#define WM8994_DELAY 50
struct snd_soc_codec *gpCodec;

#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)

/* codec private data */
struct wm8994_priv {
	unsigned int sysclk;
};

#define call_maxvol 5

/* call_vol:  save all kinds of system volume value. */
unsigned char call_vol=5;
unsigned short headset_vol_table[6]	={0x0100,0x011d,0x012d,0x0135,0x013b,0x013f};
unsigned short speakers_vol_table[6]	={0x0100,0x011d,0x012d,0x0135,0x013b,0x013f};
unsigned short earpiece_vol_table[6]	={0x0100,0x011d,0x012d,0x0135,0x013b,0x013f};
unsigned short BT_vol_table[6]		={0x0100,0x011d,0x012d,0x0135,0x013b,0x013f};

enum wm8994_codec_mode
{
  wm8994_AP_to_headset,
  wm8994_AP_to_speakers,
  wm8994_recorder_and_AP_to_headset,
  wm8994_recorder_and_AP_to_speakers,
  wm8994_FM_to_headset,
  wm8994_FM_to_headset_and_record,
  wm8994_FM_to_speakers,
  wm8994_FM_to_speakers_and_record,
  wm8994_handsetMIC_to_baseband_to_headset,
  wm8994_handsetMIC_to_baseband_to_headset_and_record,
  wm8994_mainMIC_to_baseband_to_earpiece,
  wm8994_mainMIC_to_baseband_to_earpiece_and_record,
  wm8994_mainMIC_to_baseband_to_speakers,
  wm8994_mainMIC_to_baseband_with_AP_to_speakers,
  wm8994_mainMIC_to_baseband_to_speakers_and_record,
  wm8994_BT_baseband,
  wm8994_BT_baseband_and_record,
  null
};

/* wm8994_current_mode:save current wm8994 mode */
unsigned char wm8994_current_mode=null;

void wm8994_set_volume(unsigned char wm8994_mode,unsigned char volume,unsigned char max_volume);

enum stream_type_wm8994
{
	VOICE_CALL	=0,
	BLUETOOTH_SCO,
};

/* For voice device route set, add by phc  */
enum VoiceDeviceSwitch
{
	SPEAKER_INCALL,
	SPEAKER_NORMAL,
	
	HEADSET_INCALL,
	HEADSET_NORMAL,

	EARPIECE_INCALL,
	EARPIECE_NORMAL,
	
	BLUETOOTH_SCO_INCALL,
	BLUETOOTH_SCO_NORMAL,

	BLUETOOTH_A2DP_INCALL,
	BLUETOOTH_A2DP_NORMAL,
	
	MIC_CAPTURE,

	EARPIECE_RINGTONE,
	SPEAKER_RINGTONE,
	HEADSET_RINGTONE,
	
	ALL_OPEN,
	ALL_CLOSED
};

/* speaker data */
static unsigned int frist_mute;

/*
 * wm8994 register cache
 * We can't read the WM8994 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8994_reg[] = {
	0x013f, 0x013f, 0x00e0, 0x01e0,  /*  0 *////0x0100 0x0180
	0x0000, 0x0008, 0x0000, 0x000a,  /*  4 */
	0x0100, 0x0000, 0x00ff, 0x00ff,  /*  8 */
	0x000f, 0x000f, 0x0000, 0x0000,  /* 12 */
	0x0000, 0x007b, 0x0000, 0x0032,  /* 16 */
	0x0000, 0x00c3, 0x00c3, 0x00c0,  /* 20 */
	0x0184, 0x00c0, 0x0180, 0x0000,  /* 24 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 28 */
	0x0000, 0x0000, 0x0120, 0x0070,  /* 32 */
	0x0070, 0x0120, 0x0070, 0x0070,  /* 36 */
	0x0079, 0x0079, 0x0079,          /* 40 */
};
/*
 * read wm8994 register cache
 */
static inline unsigned int wm8994_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg > WM8994_CACHE_REGNUM)
		return -1;
	return cache[reg];
}

static int wm8994_read(unsigned short reg,unsigned short *value)
{
    	unsigned char data[2];
    	unsigned char dataout[2];

    	data[0] = (u8)(reg & 0x00FF);        /* reg低位 */
    	data[1] = (u8)((reg & 0xFF00)>>8);   /* reg高位 */
	//printk("%s----%d:: date:0x%x,0x%x\n",__FUNCTION__,__LINE__,data[0],data[1]);
	if (gpCodec->hw_read(gpCodec->control_data, data, 2) == 2)
	{
	    	dataout[0]=data[1];
        	dataout[1]=data[0];
			//printk("%s----%d:: change \n",__FUNCTION__,__LINE__);
	  	*value = *((short*)&dataout[0]); /* ARM 小端 */
		return 0;
	}

	printk("codec read error!");
	return -EIO;
}

static int wm8994_write(unsigned short reg,unsigned short value)
{
	unsigned char data[4];
	unsigned char read_back[2];
      
	data[0] = (u8)(reg & 0x00FF);        /* reg低位 */
	data[1] = (u8)((reg & 0xFF00)>>8);   /* reg高位 */
	data[2] = (u8)((value & 0xFF00)>>8);
	data[3] = (u8)(value & 0x00FF);

	if(reg == 0x304){
		printk("!!!!!!!  reg=304, val=0x%04x\n", value);
	}
	
	if (gpCodec->hw_write(gpCodec->control_data, data, 2) != 2)
		goto err;

	read_back[0] = (u8)(reg & 0x00FF); 
	read_back[1] = (u8)((reg & 0xFF00)>>8); 

	if (gpCodec->hw_read(gpCodec->control_data, read_back, 2) != 2)
		goto err;
	printk("reg=0x%04x, val=0x%04x, real_val=0x%04x\n", reg, value, read_back[0] << 8 | read_back[1]);
	
	return 0;
	
err:
	printk("%s----%d::wm8994 write error\n",__FUNCTION__,__LINE__);
	return -EIO;
}

static int wm8994_codec_write(struct snd_soc_codec *codec,unsigned int reg,unsigned int value)
{
	return(wm8994_write(reg,value));
}

#define wm8994_reset()	wm8994_write(WM8994_RESET, 0)

void AP_to_headset(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_AP_to_headset)return;
	wm8994_current_mode=wm8994_AP_to_headset;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(WM8994_DELAY);

	wm8994_write(0x200, 0x0001);
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);

	wm8994_write(0x210, 0x0083); // SR=48KHz
	wm8994_write(0x220, 0x0004);  
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005);
	wm8994_write(0x200, 0x0011);  // sysclk = fll (bit4 =1)   0x0011
	wm8994_write(0x300, 0x4010);  // i2s 16 bits
  
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1/ q
	wm8994_write(0x05,  0x0303);   
	wm8994_write(0x2D,  0x0100);
	wm8994_write(0x2E,  0x0100);
	
	wm8994_write(0x4C,  0x9F25);
	mdelay(5);
	wm8994_write(0x01,  0x0303);
	mdelay(50);
	wm8994_write(0x60,  0x0022);
	wm8994_write(0x60,  0x00FF);
	
	wm8994_write(0x208, 0x000A);
	wm8994_write(0x420, 0x0000);
	wm8994_write(0x601, 0x0001);
	wm8994_write(0x602, 0x0001);
    
	wm8994_write(0x610, 0x01A0);  //DAC1 Left Volume bit0~7  		
	wm8994_write(0x611, 0x01A0);  //DAC1 Right Volume bit0~7	
	wm8994_write(0x03,  0x3030);
	wm8994_write(0x22,  0x0000);
	wm8994_write(0x23,  0x0100);
	wm8994_write(0x36,  0x0003);
	wm8994_write(0x1C,  0x017F);  //HPOUT1L Volume
	wm8994_write(0x1D,  0x017F);  //HPOUT1R Volume

#ifdef  CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x3000); // AIF1_MSTR=1
	mdelay(10);
	wm8994_write(0x302, 0x7000); //解决codec设为master后，i2c不能通讯的问题

#endif
}

void AP_to_speakers(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_AP_to_speakers)return;
	wm8994_current_mode=wm8994_AP_to_speakers;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(WM8994_DELAY);

	wm8994_write(0x200, 0x0001);
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);

	wm8994_write(0x210, 0x0083); // SR=48KHz
	wm8994_write(0x220, 0x0004);  
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005);
	wm8994_write(0x200, 0x0011);  // sysclk = fll (bit4 =1)   0x0011
	wm8994_write(0x300, 0xC010);  // i2s 16 bits
  
	wm8994_write(0x01,  0x3003); 
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1
	wm8994_write(0x05,  0x0303);   
	wm8994_write(0x2D,  0x0100);
	wm8994_write(0x2E,  0x0100);
	wm8994_write(0x4C,  0x9F25);
	wm8994_write(0x60,  0x00EE);
	wm8994_write(0x208, 0x000A);
	wm8994_write(0x420, 0x0000); 
	
	wm8994_write(0x601, 0x0001);
	wm8994_write(0x602, 0x0001);
    
	wm8994_write(0x610, 0x01c0);  //DAC1 Left Volume bit0~7	
	wm8994_write(0x611, 0x01c0);  //DAC1 Right Volume bit0~7	
	wm8994_write(0x03,  0x0330);
	wm8994_write(0x22,  0x0000);
	wm8994_write(0x23,  0x0100);
	wm8994_write(0x36,  0x0003);
	wm8994_write(0x26,  0x017F);  //Speaker Left Output Volume
	wm8994_write(0x27,  0x017F);  //Speaker Right Output Volume

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x3000); // AIF1_MSTR=1
	mdelay(10);
	wm8994_write(0x302, 0x7000); //解决codec设为master后，i2c不能通讯的问题

#endif

	mdelay(WM8994_DELAY);
}
#if 0
void recorder_and_AP_to_headset(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);
	printk("[xxm_]recorder_and_AP_to_headset\r\n");
	if(wm8994_current_mode==wm8994_recorder_and_AP_to_headset)return;
	wm8994_current_mode=wm8994_recorder_and_AP_to_headset;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(WM8994_DELAY);

//MCLK=12MHz
//48KHz, BCLK=48KHz*64=3.072MHz, Fout=12.288MHz

	wm8994_write(0x200, 0x0001); // AIF1CLK_ENA=1
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);

	wm8994_write(0x210, 0x0003); // SR=8KHz

	wm8994_write(0x220, 0x0004); 
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005); // FLL1_FRACN_ENA=1, FLL1_ENA=1
	wm8994_write(0x200, 0x0011); // AIF1CLK_SRC=10, AIF1CLK_ENA=1

	//wm8994_write(0x02,  0x6110); // TSHUT_ENA=1, TSHUT_OPDIS=1, MIXINR_ENA=1,IN1R_ENA=1
	wm8994_write(0x02,  0x6240);								/*xxm TD*/
	wm8994_write(0x03,  0x3030);
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1
	wm8994_write(0x18,  0x015B);								/*xxm TD*/
	wm8994_write(0x1A,  0x015B); // IN1_VU=1, IN1R_ZC=1, IN1R_VOL=1_1011
	//wm8994_write(0x28,  0x0003); // IN1RP_TO_IN1R=1, IN1RN_TO_IN1R=1
	wm8994_write(0x28,  0x0030);								/*xxm TD*/
	//wm8994_write(0x29,  0x0020);								/*xxm TD*/
	wm8994_write(0x2A,  0x0020); // IN1R_TO_MIXINR=1
	wm8994_write(0x200, 0x0011); // AIF1CLK_ENA=1
	wm8994_write(0x208, 0x000A); // DSP_FS1CLK_ENA=1, DSP_FSINTCLK_ENA=1
	//wm8994_write(0x300, 0xC050); // AIF1ADCL_SRC=1, AIF1ADCR_SRC=1, AIF1_WL=10, AIF1_FMT=10
	
	wm8994_write(0x300, 0x0050); // AIF1ADCL_SRC=1, AIF1ADCR_SRC=1, AIF1_WL=10, AIF1_FMT=10		/*xxm TD*/
	wm8994_write(0x606, 0x0002); // ADC1L_TO_AIF1ADC1L=1
	wm8994_write(0x607, 0x0002); // ADC1R_TO_AIF1ADC1R=1
	wm8994_write(0x620, 0x0000); 

	wm8994_write(0x700, 0xA101); 

	wm8994_write(0x01,  0x0313);
	wm8994_write(0x05,  0x0303); // AIF1DAC1L_ENA=1, AIF1DAC1R_ENA=1, DAC1L_ENA=1, DAC1R_ENA=1
	wm8994_write(0x2D,  0x0100); // DAC1L_TO_HPOUT1L=1
	wm8994_write(0x2E,  0x0100); // DAC1R_TO_HPOUT1R=1
	wm8994_write(0x4C,  0x9F25); // CP_ENA=1
	wm8994_write(0x60,  0x00EE); // HPOUT1L_RMV_SHORT=1, HPOUT1L_OUTP=1, HPOUT1L_DLY=1, HPOUT1R_RMV_SHORT=1, HPOUT1R_OUTP=1, HPOUT1R_DLY=1
	wm8994_write(0x601, 0x0001); // AIF1DAC1L_TO_DAC1L=1
	wm8994_write(0x602, 0x0001); // AIF1DAC1R_TO_DAC1R=1
	wm8994_write(0x610, 0x01A0); // DAC1_VU=1, DAC1L_VOL=1100_0000
	wm8994_write(0x611, 0x01A0); // DAC1_VU=1, DAC1R_VOL=1100_0000
	wm8994_write(0x1C,  0x017F);  //HPOUT1L Volume
	wm8994_write(0x1D,  0x017F);  //HPOUT1R Volume
	wm8994_write(0x420, 0x0000); 

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x3000); // AIF1_MSTR=1
	mdelay(10);
	wm8994_write(0x302, 0x7000); //解决codec设为master后，i2c不能通讯的问题
#endif
}
#else
void recorder_and_AP_to_headset(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);
	printk("[xxm_]recorder_and_AP_to_headset\r\n");

	if(wm8994_current_mode==wm8994_recorder_and_AP_to_headset)return;
	wm8994_current_mode=wm8994_recorder_and_AP_to_headset;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(WM8994_DELAY);

//MCLK=12MHz
//48KHz, BCLK=48KHz*64=3.072MHz, Fout=12.288MHz

	wm8994_write(0x200, 0x0001); // AIF1CLK_ENA=1
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);

	wm8994_write(0x210, 0x0083); // SR=48KHz

	wm8994_write(0x220, 0x0004); 
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005); // FLL1_FRACN_ENA=1, FLL1_ENA=1
	wm8994_write(0x200, 0x0011); // AIF1CLK_SRC=10, AIF1CLK_ENA=1

	//wm8994_write(0x02,  0x6110); // TSHUT_ENA=1, TSHUT_OPDIS=1, MIXINR_ENA=1,IN1R_ENA=1
	wm8994_write(0x02,  0x6240);								/*xxm TD*/
	wm8994_write(0x03,  0x3030);
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1
	wm8994_write(0x18,  0x015B);								/*xxm TD*/
	//wm8994_write(0x1A,  0x015B); // IN1_VU=1, IN1R_ZC=1, IN1R_VOL=1_1011
	//wm8994_write(0x28,  0x0003); // IN1RP_TO_IN1R=1, IN1RN_TO_IN1R=1
	wm8994_write(0x28,  0x0030);								/*xxm TD*/
	wm8994_write(0x29,  0x0020);								/*xxm TD*/
	//wm8994_write(0x2A,  0x0020); // IN1R_TO_MIXINR=1
	wm8994_write(0x200, 0x0011); // AIF1CLK_ENA=1
	wm8994_write(0x208, 0x000A); // DSP_FS1CLK_ENA=1, DSP_FSINTCLK_ENA=1
	//wm8994_write(0x300, 0xC050); // AIF1ADCL_SRC=1, AIF1ADCR_SRC=1, AIF1_WL=10, AIF1_FMT=10
	
	wm8994_write(0x300, 0x0050); // AIF1ADCL_SRC=1, AIF1ADCR_SRC=1, AIF1_WL=10, AIF1_FMT=10		/*xxm TD*/
	//wm8994_write(0x301, 0x0001); 
	wm8994_write(0x606, 0x0002); // ADC1L_TO_AIF1ADC1L=1
	wm8994_write(0x607, 0x0002); // ADC1R_TO_AIF1ADC1R=1
	wm8994_write(0x620, 0x0000); 

	wm8994_write(0x700, 0xA101); 

	wm8994_write(0x01,  0x0313);
	wm8994_write(0x05,  0x0303); // AIF1DAC1L_ENA=1, AIF1DAC1R_ENA=1, DAC1L_ENA=1, DAC1R_ENA=1
	wm8994_write(0x2D,  0x0100); // DAC1L_TO_HPOUT1L=1
	wm8994_write(0x2E,  0x0100); // DAC1R_TO_HPOUT1R=1
	wm8994_write(0x4C,  0x9F25); // CP_ENA=1
	wm8994_write(0x60,  0x00EE); // HPOUT1L_RMV_SHORT=1, HPOUT1L_OUTP=1, HPOUT1L_DLY=1, HPOUT1R_RMV_SHORT=1, HPOUT1R_OUTP=1, HPOUT1R_DLY=1
	wm8994_write(0x601, 0x0001); // AIF1DAC1L_TO_DAC1L=1
	wm8994_write(0x602, 0x0001); // AIF1DAC1R_TO_DAC1R=1
	wm8994_write(0x610, 0x01A0); // DAC1_VU=1, DAC1L_VOL=1100_0000
	wm8994_write(0x611, 0x01A0); // DAC1_VU=1, DAC1R_VOL=1100_0000
	wm8994_write(0x1C,  0x017F);  //HPOUT1L Volume
	wm8994_write(0x1D,  0x017F);  //HPOUT1R Volume
	wm8994_write(0x420, 0x0000); 

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x3000); // AIF1_MSTR=1
	mdelay(10);
	wm8994_write(0x302, 0x7000); //解决codec设为master后，i2c不能通讯的问题
#endif
}
#endif

#if 0
void recorder_and_AP_to_speakers(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	printk("[xxm_]recorder_and_AP_to_speakers\r\n");
	if(wm8994_current_mode==wm8994_recorder_and_AP_to_speakers)return;
	wm8994_current_mode=wm8994_recorder_and_AP_to_speakers;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(WM8994_DELAY);

//MCLK=12MHz
//48KHz, BCLK=48KHz*64=3.072MHz, Fout=12.288MHz

	wm8994_write(0x200, 0x0001); // AIF1CLK_ENA=1
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);
	wm8994_write(0x210, 0x0083); // SR=48KHz

	wm8994_write(0x220, 0x0004); 
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005); // FLL1_FRACN_ENA=1, FLL1_ENA=1
	wm8994_write(0x200, 0x0011); // AIF1CLK_SRC=10, AIF1CLK_ENA=1

	wm8994_write(0x02,  0x6110); // TSHUT_ENA=1, TSHUT_OPDIS=1, MIXINR_ENA=1,IN1R_ENA=1
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1
	wm8994_write(0x1A,  0x015B); // IN1_VU=1, IN1R_ZC=1, IN1R_VOL=1_1011
	wm8994_write(0x28,  0x0003); // IN1RP_TO_IN1R=1, IN1RN_TO_IN1R=1
	wm8994_write(0x2A,  0x0020); // IN1R_TO_MIXINR=1
	wm8994_write(0x200, 0x0011); // AIF1CLK_ENA=1
	wm8994_write(0x208, 0x000A); // DSP_FS1CLK_ENA=1, DSP_FSINTCLK_ENA=1
	wm8994_write(0x300, 0xC050); // AIF1ADCL_SRC=1, AIF1ADCR_SRC=1, AIF1_WL=10, AIF1_FMT=10
	wm8994_write(0x606, 0x0002); // ADC1L_TO_AIF1ADC1L=1
	wm8994_write(0x607, 0x0002); // ADC1R_TO_AIF1ADC1R=1
	wm8994_write(0x620, 0x0000); 

	wm8994_write(0x700, 0xA101); 

	wm8994_write(0x01,  0x3033);
	wm8994_write(0x03,  0x0330); // SPKRVOL_ENA=1, SPKLVOL_ENA=1, MIXOUTL_ENA=1, MIXOUTR_ENA=1  
	wm8994_write(0x05,  0x0303); // AIF1DAC1L_ENA=1, AIF1DAC1R_ENA=1, DAC1L_ENA=1, DAC1R_ENA=1
	wm8994_write(0x22,  0x0000);
	wm8994_write(0x23,  0x0100); // SPKOUT_CLASSAB=1

	wm8994_write(0x2D,  0x0001); // DAC1L_TO_MIXOUTL=1
	wm8994_write(0x2E,  0x0001); // DAC1R_TO_MIXOUTR=1
	wm8994_write(0x4C,  0x9F25);
	wm8994_write(0x60,  0x00EE);
	wm8994_write(0x36,  0x000C); // MIXOUTL_TO_SPKMIXL=1, MIXOUTR_TO_SPKMIXR=1
	wm8994_write(0x601, 0x0001); // AIF1DAC1L_TO_DAC1L=1
	wm8994_write(0x602, 0x0001); // AIF1DAC1R_TO_DAC1R=1
	wm8994_write(0x610, 0x01C0); // DAC1_VU=1, DAC1L_VOL=1100_0000
	wm8994_write(0x611, 0x01C0); // DAC1_VU=1, DAC1R_VOL=1100_0000
	wm8994_write(0x26,  0x017F);  //Speaker Left Output Volume
	wm8994_write(0x27,  0x017F);  //Speaker Right Output Volume
	wm8994_write(0x420, 0x0000); 

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x4000); // AIF1_MSTR=1

#endif
}
#else
void recorder_and_AP_to_speakers(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);
	printk("[xxm_]recorder_and_AP_to_speakers\r\n");

	if(wm8994_current_mode==wm8994_recorder_and_AP_to_speakers)return;
	wm8994_current_mode=wm8994_recorder_and_AP_to_speakers;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(WM8994_DELAY);

//MCLK=12MHz
//48KHz, BCLK=48KHz*64=3.072MHz, Fout=12.288MHz

	wm8994_write(0x200, 0x0001); // AIF1CLK_ENA=1
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);
	wm8994_write(0x210, 0x0083); // SR=48KHz

	wm8994_write(0x220, 0x0004); 
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005); // FLL1_FRACN_ENA=1, FLL1_ENA=1
	wm8994_write(0x200, 0x0011); // AIF1CLK_SRC=10, AIF1CLK_ENA=1

	wm8994_write(0x02,  0x6240); // TSHUT_ENA=1, TSHUT_OPDIS=1, MIXINR_ENA=1,IN1R_ENA=1
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1
	wm8994_write(0x18,  0x015B); // IN1_VU=1, IN1R_ZC=1, IN1R_VOL=1_1011
	wm8994_write(0x28,  0x0030); // IN1RP_TO_IN1R=1, IN1RN_TO_IN1R=1
	wm8994_write(0x29,  0x0020); // IN1R_TO_MIXINR=1
	wm8994_write(0x200, 0x0011); // AIF1CLK_ENA=1
	wm8994_write(0x208, 0x000A); // DSP_FS1CLK_ENA=1, DSP_FSINTCLK_ENA=1
	wm8994_write(0x300, 0x0050); // AIF1ADCL_SRC=1, AIF1ADCR_SRC=1, AIF1_WL=10, AIF1_FMT=10
	wm8994_write(0x606, 0x0002); // ADC1L_TO_AIF1ADC1L=1
	wm8994_write(0x607, 0x0002); // ADC1R_TO_AIF1ADC1R=1
	wm8994_write(0x620, 0x0000); 

	wm8994_write(0x700, 0xA101); 

	wm8994_write(0x01,  0x3013);
	wm8994_write(0x03,  0x0330); // SPKRVOL_ENA=1, SPKLVOL_ENA=1, MIXOUTL_ENA=1, MIXOUTR_ENA=1  
	wm8994_write(0x05,  0x0303); // AIF1DAC1L_ENA=1, AIF1DAC1R_ENA=1, DAC1L_ENA=1, DAC1R_ENA=1
	wm8994_write(0x22,  0x0000);
	wm8994_write(0x23,  0x0100); // SPKOUT_CLASSAB=1

	wm8994_write(0x2D,  0x0001); // DAC1L_TO_MIXOUTL=1
	wm8994_write(0x2E,  0x0001); // DAC1R_TO_MIXOUTR=1
	wm8994_write(0x4C,  0x9F25);
	wm8994_write(0x60,  0x00EE);
	wm8994_write(0x36,  0x000C); // MIXOUTL_TO_SPKMIXL=1, MIXOUTR_TO_SPKMIXR=1
	wm8994_write(0x601, 0x0001); // AIF1DAC1L_TO_DAC1L=1
	wm8994_write(0x602, 0x0001); // AIF1DAC1R_TO_DAC1R=1
	wm8994_write(0x610, 0x01C0); // DAC1_VU=1, DAC1L_VOL=1100_0000
	wm8994_write(0x611, 0x01C0); // DAC1_VU=1, DAC1R_VOL=1100_0000
	wm8994_write(0x26,  0x017F);  //Speaker Left Output Volume
	wm8994_write(0x27,  0x017F);  //Speaker Right Output Volume
	wm8994_write(0x420, 0x0000); 

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x3000); // AIF1_MSTR=1
	mdelay(10);
	wm8994_write(0x302, 0x7000); //解决codec设为master后，i2c不能通讯的问题
#endif
}
#endif

void FM_to_headset(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_FM_to_headset)return;
	wm8994_current_mode=wm8994_FM_to_headset;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0323); 
	wm8994_write(0x02,  0x03A0);  
	wm8994_write(0x03,  0x0030);	
	wm8994_write(0x19,  0x010B);  //LEFT LINE INPUT 3&4 VOLUME	
	wm8994_write(0x1B,  0x010B);  //RIGHT LINE INPUT 3&4 VOLUME

	wm8994_write(0x28,  0x0044);  
	wm8994_write(0x29,  0x0100);	 
	wm8994_write(0x2A,  0x0100);
	wm8994_write(0x2D,  0x0040); 
	wm8994_write(0x2E,  0x0040);
	wm8994_write(0x4C,  0x9F25);
	wm8994_write(0x60,  0x00EE);
	wm8994_write(0x220, 0x0003);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x224, 0x0CC0);
	wm8994_write(0x200, 0x0011);
	wm8994_write(0x1C,  0x01F9);  //LEFT OUTPUT VOLUME	
	wm8994_write(0x1D,  0x01F9);  //RIGHT OUTPUT VOLUME
}

void FM_to_headset_and_record(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_FM_to_headset_and_record)return;
	wm8994_current_mode=wm8994_FM_to_headset_and_record;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,   0x0003);
	mdelay(WM8994_DELAY);
	wm8994_write(0x221,  0x1900);  //8~13BIT div

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x302,  0x4000);  // master = 0x4000 // slave= 0x0000
	wm8994_write(0x303,  0x0040);  // master  0x0050 lrck 7.94kHz bclk 510KHz
#endif
	
	wm8994_write(0x220,  0x0004);
	mdelay(WM8994_DELAY);
	wm8994_write(0x220,  0x0005);  

	wm8994_write(0x01,   0x0323);
	wm8994_write(0x02,   0x03A0);
	wm8994_write(0x03,   0x0030);
	wm8994_write(0x19,   0x010B);  //LEFT LINE INPUT 3&4 VOLUME	
	wm8994_write(0x1B,   0x010B);  //RIGHT LINE INPUT 3&4 VOLUME
  
	wm8994_write(0x28,   0x0044);
	wm8994_write(0x29,   0x0100);
	wm8994_write(0x2A,   0x0100);
	wm8994_write(0x2D,   0x0040);
	wm8994_write(0x2E,   0x0040);
	wm8994_write(0x4C,   0x9F25);
	wm8994_write(0x60,   0x00EE);
	wm8994_write(0x200,  0x0011);
	wm8994_write(0x1C,   0x01F9);  //LEFT OUTPUT VOLUME
	wm8994_write(0x1D,   0x01F9);  //RIGHT OUTPUT VOLUME
	wm8994_write(0x04,   0x0303);
	wm8994_write(0x208,  0x000A);
	wm8994_write(0x300,  0x4050);
	wm8994_write(0x606,  0x0002);
	wm8994_write(0x607,  0x0002);
	wm8994_write(0x620,  0x0000);
}

void FM_to_speakers(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_FM_to_speakers)return;
	wm8994_current_mode=wm8994_FM_to_speakers;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,   0x3023);
	wm8994_write(0x02,   0x03A0);
	wm8994_write(0x03,   0x0330);
	wm8994_write(0x19,   0x010B);  //LEFT LINE INPUT 3&4 VOLUME
	wm8994_write(0x1B,   0x010B);  //RIGHT LINE INPUT 3&4 VOLUME
  
	wm8994_write(0x22,   0x0000);
	wm8994_write(0x23,   0x0000);
	wm8994_write(0x36,   0x000C);

	wm8994_write(0x28,   0x0044);
	wm8994_write(0x29,   0x0100);
	wm8994_write(0x2A,   0x0100);
	wm8994_write(0x2D,   0x0040);
	wm8994_write(0x2E,   0x0040);

	wm8994_write(0x220,  0x0003);
	wm8994_write(0x221,  0x0700);
	wm8994_write(0x224,  0x0CC0);

	wm8994_write(0x200,  0x0011);
	wm8994_write(0x20,   0x01F9);
	wm8994_write(0x21,   0x01F9);
}

void FM_to_speakers_and_record(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_FM_to_speakers_and_record)return;
	wm8994_current_mode=wm8994_FM_to_speakers_and_record;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,   0x0003);  
	mdelay(WM8994_DELAY);

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x302,  0x4000);  // master = 0x4000 // slave= 0x0000
	wm8994_write(0x303,  0x0090);  //
#endif
	
	wm8994_write(0x220,  0x0006);
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,   0x3023);
	wm8994_write(0x02,   0x03A0);
	wm8994_write(0x03,   0x0330);
	wm8994_write(0x19,   0x010B);  //LEFT LINE INPUT 3&4 VOLUME
	wm8994_write(0x1B,   0x010B);  //RIGHT LINE INPUT 3&4 VOLUME
  
	wm8994_write(0x22,   0x0000);
	wm8994_write(0x23,   0x0000);
	wm8994_write(0x36,   0x000C);

	wm8994_write(0x28,   0x0044);
	wm8994_write(0x29,   0x0100);
	wm8994_write(0x2A,   0x0100);
	wm8994_write(0x2D,   0x0040);
	wm8994_write(0x2E,   0x0040);

	wm8994_write(0x220,  0x0003);
	wm8994_write(0x221,  0x0700);
	wm8994_write(0x224,  0x0CC0);

	wm8994_write(0x200,  0x0011);
	wm8994_write(0x20,   0x01F9);
	wm8994_write(0x21,   0x01F9);
	wm8994_write(0x04,   0x0303);
	wm8994_write(0x208,  0x000A);	
	wm8994_write(0x300,  0x4050);
	wm8994_write(0x606,  0x0002);	
	wm8994_write(0x607,  0x0002);
	wm8994_write(0x620,  0x0000);
}
#ifndef PCM_BB
void handsetMIC_to_baseband_to_headset(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_handsetMIC_to_baseband_to_headset)return;
	wm8994_current_mode=wm8994_handsetMIC_to_baseband_to_headset;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(50);

	wm8994_write(0x200, 0x0001);
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);
	wm8994_write(0x210, 0x0083);
	
	wm8994_write(0x220, 0x0004);
	mdelay(50);
	wm8994_write(0x220, 0x0005);
	wm8994_write(0x200, 0x0011);
	wm8994_write(0x300, 0xC010);  // i2s 16 bits


	wm8994_write(0x02,  0x6040);
	wm8994_write(0x03,  0x3030);
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1
	wm8994_write(0x05,  0x0303);
	wm8994_write(0x1A,  0x0155); //mic volume
	wm8994_write(0x1E,  0x0006); 
	wm8994_write(0x22,  0x0000);
	wm8994_write(0x23,  0x0100);
	wm8994_write(0x28,  0x0030);  //IN1LN_TO_IN1L IN1LP_TO_IN1L

	wm8994_write(0x02,  0x6240);
	wm8994_write(0x29,  0x0030);
	wm8994_write(0x2D,  0x0003);  //bit 1 IN2LP_TO_MIXOUTL bit 12 DAC1L_TO_HPOUT1L  0x0102 
	wm8994_write(0x2E,  0x0003);  //bit 1 IN2RP_TO_MIXOUTR bit 12 DAC1R_TO_HPOUT1R  0x0102

	wm8994_write(0x34,  0x0004);  //IN1L_TO_LINEOUT1P
	wm8994_write(0x36,  0x0003);

	wm8994_write(0x4C,  0x9F25);
	mdelay(5);
	wm8994_write(0x01,  0x0313);
	mdelay(50);
	wm8994_write(0x60,  0x0022);
	wm8994_write(0x60,  0x00EE);

	wm8994_write(0x208, 0x000A);
	//wm8994_write(0x224, 0x0CC0); //zyy2010-8-26 写这个寄存器会导致i2s采样率变为42k
	wm8994_write(0x420, 0x0000);
	wm8994_write(0x601, 0x0001);
	wm8994_write(0x602, 0x0001);

	wm8994_write(0x610, 0x01A0);  //DAC1 Left Volume bit0~7  		
	wm8994_write(0x611, 0x01A0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x1C,  0x01FF);  //HPOUT1L Volume
	wm8994_write(0x1D,  0x01FF);  //HPOUT1R Volume

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x3000); // AIF1_MSTR=1
	mdelay(10);
	wm8994_write(0x302, 0x7000); //解决codec设为master后，i2c不能通讯的问题
#endif

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void handsetMIC_to_baseband_to_headset_and_record(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_handsetMIC_to_baseband_to_headset_and_record)return;
	wm8994_current_mode=wm8994_handsetMIC_to_baseband_to_headset_and_record;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0303|wm8994_mic_VCC); 
	wm8994_write(0x02,  0x62C0); 
	wm8994_write(0x03,  0x3030); 
	wm8994_write(0x04,  0x0303); 
	wm8994_write(0x18,  0x014B);  //volume
	wm8994_write(0x19,  0x014B);  //volume
	wm8994_write(0x1C,  0x01FF);  //LEFT OUTPUT VOLUME
	wm8994_write(0x1D,  0x01F9);  //RIGHT OUTPUT VOLUME
	wm8994_write(0x1E,  0x0006); 
	wm8994_write(0x28,  0x0030);  //IN2LP_TO_IN2L
	wm8994_write(0x29,  0x0120); 
	wm8994_write(0x2D,  0x0002);  //bit 1 IN2LP_TO_MIXOUTL
	wm8994_write(0x2E,  0x0002);  //bit 1 IN2RP_TO_MIXOUTR
	wm8994_write(0x34,  0x0002); 
	wm8994_write(0x4C,  0x9F25); 
	wm8994_write(0x60,  0x00EE); 
	wm8994_write(0x200, 0x0001); 
	wm8994_write(0x208, 0x000A); 
	wm8994_write(0x300, 0x0050); 

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x302, 0x4000);  // master = 0x4000 // slave= 0x0000
	wm8994_write(0x303, 0x0090);  // master lrck 16k
#endif

	wm8994_write(0x606, 0x0002); 
	wm8994_write(0x607, 0x0002); 
	wm8994_write(0x620, 0x0000); 

	wm8994_write(0x1C,  0x01F9); 
	wm8994_write(0x1D,  0x01F9);

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void mainMIC_to_baseband_to_earpiece(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_earpiece)return;
	wm8994_current_mode=wm8994_mainMIC_to_baseband_to_earpiece;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(WM8994_DELAY);

	wm8994_write(0x200, 0x0001);
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);

	wm8994_write(0x210, 0x0083); // SR=48KHz
	wm8994_write(0x220, 0x0004);
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005);
	wm8994_write(0x200, 0x0011);  // sysclk = fll (bit4 =1)   0x0011
	wm8994_write(0x300, 0x4010);  // i2s 16 bits

	wm8994_write(0x01,  0x0803|wm8994_mic_VCC); //HPOUT2_ENA=1, VMID_SEL=01, BIAS_ENA=1
	wm8994_write(0x02,  0x6240);
	wm8994_write(0x03,  0x30F0);
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1
	wm8994_write(0x18,  0x0159);   /// Left Line Input 1&2 Volume(18H): 014B  IN1_VU=1, IN1L_MUTE=0, IN1L_ZC=1, IN1L_VOL=0_1011
	wm8994_write(0x05,  0x0303);
	wm8994_write(0x1A,  0x015F);  //main mic volume
	wm8994_write(0x1E,  0x0006);
	wm8994_write(0x1F,  0x0000);
	wm8994_write(0x28,  0x0030);
	wm8994_write(0x2B,  0x0005); //VRX_MIXINL_VOL
	wm8994_write(0x2D,  0x0041); //DAC1L_TO_MIXOUTL=1
	wm8994_write(0x2E,  0x0001); //DAC1R_TO_MIXOUTR=1
	wm8994_write(0x33,  0x0010);
	wm8994_write(0x34,  0x0002);  //MIXOUTR_TO_SPKMIXR =1 un-mute*/

	wm8994_write(0x208, 0x000A); //DSP_FS1CLK_ENA=1, DSP_FSINTCLK_ENA=1
	wm8994_write(0x601, 0x0001); //AIF1DAC1L_TO_DAC1L=1
	wm8994_write(0x602, 0x0001); //AIF1DAC1R_TO_DAC1R=1
	wm8994_write(0x610, 0x01C0); //DAC1_VU=1, DAC1L_VOL=1100_0000
	wm8994_write(0x611, 0x01C0); //DAC1_VU=1, DAC1R_VOL=1100_0000

	wm8994_write(0x420, 0x0000);

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x4000); // AIF1_MSTR=1

#endif

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void mainMIC_to_baseband_to_earpiece_and_record(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_earpiece_and_record)return;
	wm8994_current_mode=wm8994_mainMIC_to_baseband_to_earpiece_and_record;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01  ,0x0803|wm8994_mic_VCC);
	wm8994_write(0x02  ,0x6310);
	wm8994_write(0x03  ,0x30A0);
	wm8994_write(0x04  ,0x0303);
	wm8994_write(0x1A  ,0x014F);
	wm8994_write(0x1E  ,0x0006);
	wm8994_write(0x1F  ,0x0000);
	wm8994_write(0x28  ,0x0003);  //MAINMIC_TO_IN1R  //
	wm8994_write(0x2A  ,0x0020);  //IN1R_TO_MIXINR   //
	wm8994_write(0x2B  ,0x0005);  //VRX_MIXINL_VOL bit 0~2
	wm8994_write(0x2C  ,0x0005);  //VRX_MIXINR_VOL
	wm8994_write(0x2D  ,0x0040);  //MIXINL_TO_MIXOUTL
	wm8994_write(0x33  ,0x0010);  //MIXOUTLVOL_TO_HPOUT2
	wm8994_write(0x34  ,0x0004);  //IN1R_TO_LINEOUT1 //
	wm8994_write(0x200 ,0x0001);
	wm8994_write(0x208 ,0x000A);
	wm8994_write(0x300 ,0xC050);

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x302, 0x4000);  // master = 0x4000 // slave= 0x0000
	wm8994_write(0x303, 0x0090);  // master lrck 16k
#endif

	wm8994_write(0x606 ,0x0002);
	wm8994_write(0x607 ,0x0002);
	wm8994_write(0x620 ,0x0000);

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void mainMIC_to_baseband_to_speakers(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_speakers)return;
	wm8994_current_mode=wm8994_mainMIC_to_baseband_to_speakers;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);
	mdelay(WM8994_DELAY);

	wm8994_write(0x200, 0x0001);
	wm8994_write(0x220, 0x0000);
	wm8994_write(0x221, 0x0700);
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);

	wm8994_write(0x210, 0x0083); // SR=48KHz
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005);
	wm8994_write(0x200, 0x0011);  // sysclk = fll (bit4 =1)   0x0011
  	wm8994_write(0x300, 0xC010);  // i2s 16 bits
	
	wm8994_write(0x01,  0x3013); 
	wm8994_write(0x02,  0x6210);
	wm8994_write(0x03,  0x3330);
	wm8994_write(0x04,  0x0303); // AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, ADCL_ENA=1, ADCR_ENA=1
	wm8994_write(0x05,  0x0303); 
	wm8994_write(0x1A,  0x0155);
	wm8994_write(0x1E,  0x0006);
	wm8994_write(0x22,  0x0000);
	wm8994_write(0x23,  0x0100);
	wm8994_write(0x26,  0x017F);  //Speaker Volume Left bit 0~5
	wm8994_write(0x27,  0x017F);  //Speaker Volume Right bit 0~5
	wm8994_write(0x28,  0x0003);  //IN1RP_TO_IN1R  IN1RN_TO_IN1R
	wm8994_write(0x2D,  0x0003);  //bit 1 IN2LP_TO_MIXOUTL
	wm8994_write(0x2E,  0x0003);  //bit 1 IN2RP_TO_MIXOUTR
	wm8994_write(0x4C,  0x9F25);
	wm8994_write(0x60,  0x00EE);
	wm8994_write(0x34,  0x0004);
	wm8994_write(0x36,  0x000C);  //MIXOUTL_TO_SPKMIXL  MIXOUTR_TO_SPKMIXR

	wm8994_write(0x208, 0x000A);
	wm8994_write(0x420, 0x0000); 
	
	wm8994_write(0x601, 0x0001);
	wm8994_write(0x602, 0x0001);
    
	wm8994_write(0x610, 0x01c0);  //DAC1 Left Volume bit0~7	
	wm8994_write(0x611, 0x01c0);  //DAC1 Right Volume bit0~7
#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x303, 0x0040); // AIF1 BCLK DIV--------AIF1CLK/4
	wm8994_write(0x304, 0x0040); // AIF1 ADCLRCK DIV-----BCLK/64
	wm8994_write(0x305, 0x0040); // AIF1 DACLRCK DIV-----BCLK/64
	wm8994_write(0x302, 0x4000); // AIF1_MSTR=1

#endif
//*/

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void mainMIC_to_baseband_to_speakers_and_record(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_speakers_and_record)return;
	wm8994_current_mode=wm8994_mainMIC_to_baseband_to_speakers_and_record;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01, 0x3003|wm8994_mic_VCC);
	wm8994_write(0x02, 0x6330);
	wm8994_write(0x03, 0x3330);
	wm8994_write(0x04, 0x0303);
	wm8994_write(0x1A, 0x014B);
	wm8994_write(0x1B, 0x014B);
	wm8994_write(0x1E, 0x0006);
	wm8994_write(0x22, 0x0000);
	wm8994_write(0x23, 0x0100);
 	wm8994_write(0x28, 0x0007);
	wm8994_write(0x2A, 0x0120);
	wm8994_write(0x2D, 0x0002);  //bit 1 IN2LP_TO_MIXOUTL
	wm8994_write(0x2E, 0x0002);  //bit 1 IN2RP_TO_MIXOUTR
	wm8994_write(0x34, 0x0004);
	wm8994_write(0x36, 0x000C);
	wm8994_write(0x200, 0x0001);
	wm8994_write(0x208, 0x000A);
 	wm8994_write(0x300, 0xC050);

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x302, 0x4000);  // master = 0x4000 // slave= 0x0000
	wm8994_write(0x303, 0x0090);  // master lrck 16k
#endif

 	wm8994_write(0x606, 0x0002);
 	wm8994_write(0x607, 0x0002);
 	wm8994_write(0x620, 0x0000);

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void BT_baseband(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_BT_baseband)return;
	wm8994_current_mode=wm8994_BT_baseband;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01,  0x0003);  
	mdelay(WM8994_DELAY);
	wm8994_write(0x221, 0x0700);  
	wm8994_write(0x222, 0x3126);	
	wm8994_write(0x223, 0x0100);	
	wm8994_write(0x220, 0x0004);
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005);

	wm8994_write(0x01,  0x0003);
	wm8994_write(0x03,  0x30F0);
	wm8994_write(0x05,  0x3003);
	wm8994_write(0x2D,  0x0001);
	wm8994_write(0x2E,  0x0001);

	wm8994_write(0x200, 0x0001); 
	wm8994_write(0x204, 0x0001);
	wm8994_write(0x208, 0x0007);
	wm8994_write(0x520, 0x0000);
	wm8994_write(0x601, 0x0004);
	wm8994_write(0x602, 0x0004);
	wm8994_write(0x610, 0x01C0);
	wm8994_write(0x611, 0x01C0);
	wm8994_write(0x613, 0x01C0);
   
	wm8994_write(0x702, 0xC100); 
	wm8994_write(0x703, 0xC100);
	wm8994_write(0x704, 0xC100);
	wm8994_write(0x706, 0x4100);
	
	wm8994_write(0x204, 0x0011);  // AIF2 MCLK=FLL                            //MASTER
	wm8994_write(0x211, 0x0039);  //LRCK=8KHZ,Rate=MCLK/1536	                 //MASTER
	wm8994_write(0x310, 0xC118);  //DSP/PCM; 16bits; ADC L channel = R channel;MODE A

	wm8994_write(0x313, 0x00F0);
	wm8994_write(0x314, 0x0020);    
	wm8994_write(0x315, 0x0020);		
	wm8994_write(0x2B,  0x0005);    
	wm8994_write(0x2C,  0x0005);
	wm8994_write(0x02,  0x6300);
	wm8994_write(0x04,  0x3003);

	wm8994_write(0x1E,  0x0006);  //LINEOUT1N_MUTE(001Eh);
	wm8994_write(0x34,  0x0001);  //LINEOUT1_MODE=1;LINEOUT_VMID_BUF_ENA=1;

	wm8994_write(0x603, 0x018C);	 
	wm8994_write(0x604, 0x0010);
	wm8994_write(0x605, 0x0010);
	wm8994_write(0x621, 0x0001);
	wm8994_write(0x317, 0x0003);

#ifdef CONFIG_SND_ROCKCHIP_SOC_SLAVE
	wm8994_write(0x312, 0x4000);  //set 0x312 PCM2 as Master
	wm8994_write(0x313, 0x0090);  //master   0x0090 lrck2 8kHz bclk2 1MH
	wm8994_write(0x315, 0x007D);  //master   0x007D lrck2 8kHz bclk2 1MH
#endif

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void BT_baseband_and_record(void)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_BT_baseband_and_record)return;
	wm8994_current_mode=wm8994_BT_baseband_and_record;
	wm8994_reset();
	mdelay(WM8994_DELAY);

	wm8994_write(0x01, 0x0003);
	wm8994_write(0x02, 0x63A0);
	wm8994_write(0x03, 0x30A0);
	wm8994_write(0x04, 0x3303);
	wm8994_write(0x05, 0x3002);
	wm8994_write(0x06, 0x000A);
	wm8994_write(0x19, 0x014B);
	wm8994_write(0x1B, 0x014B);
	wm8994_write(0x1E, 0x0006);
	wm8994_write(0x28, 0x00CC);
	wm8994_write(0x29, 0x0100);
	wm8994_write(0x2A, 0x0100);
	wm8994_write(0x2D, 0x0001);
	wm8994_write(0x34, 0x0001);
	wm8994_write(0x200, 0x0001);

	//roger_chen@20100524
	//8KHz, BCLK=8KHz*128=1024KHz, Fout=2.048MHz
	wm8994_write(0x204, 0x0001);    // SMbus_16inx_16dat     Write  0x34      * AIF2 Clocking (1)(204H): 0011  AIF2CLK_SRC=00, AIF2CLK_INV=0, AIF2CLK_DIV=0, AIF2CLK_ENA=1
	wm8994_write(0x208, 0x000F);
	wm8994_write(0x220, 0x0000);    // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (1)(220H):  0005  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
	wm8994_write(0x221, 0x2F00);    // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=2Fh, FLL1_CTRL_RATE=000, FLL1_FRATIO=000
	wm8994_write(0x222, 0x3126);    // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (3)(222H):  8FD5  FLL1_K=3126h
	wm8994_write(0x223, 0x0100);    // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (4)(223H):  00E0  FLL1_N=8h, FLL1_GAIN=0000
	wm8994_write(0x302, 0x4000);
	wm8994_write(0x303, 0x0090);    
	wm8994_write(0x310, 0xC118);  //DSP/PCM; 16bits; ADC L channel = R channel;MODE A
	wm8994_write(0x312, 0x4000);    // SMbus_16inx_16dat     Write  0x34      * AIF2 Master/Slave(312H): 7000  AIF2_TRI=0, AIF2_MSTR=1, AIF2_CLK_FRC=0, AIF2_LRCLK_FRC=0
	wm8994_write(0x313, 0x0020);    // SMbus_16inx_16dat     Write  0x34      * AIF2 BCLK DIV--------AIF1CLK/2
	wm8994_write(0x314, 0x0080);    // SMbus_16inx_16dat     Write  0x34      * AIF2 ADCLRCK DIV-----BCLK/128
	wm8994_write(0x315, 0x0080);    // SMbus_16inx_16dat     Write  0x34      * AIF2 DACLRCK DIV-----BCLK/128
	wm8994_write(0x210, 0x0003);    // SMbus_16inx_16dat     Write  0x34      * SR=8KHz
	wm8994_write(0x220, 0x0004);    // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (1)(220H):  0005  FLL1_FRACN_ENA=1, FLL1_OSC_ENA=0, FLL1_ENA=0
	mdelay(WM8994_DELAY);
	wm8994_write(0x220, 0x0005);    // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (1)(220H):  0005  FLL1_FRACN_ENA=1, FLL1_OSC_ENA=0, FLL1_ENA=1
	wm8994_write(0x204, 0x0011);    // SMbus_16inx_16dat     Write  0x34      * AIF2 Clocking (1)(204H): 0011  AIF2CLK_SRC=10, AIF2CLK_INV=0, AIF2CLK_DIV=0, AIF2CLK_ENA=1

	wm8994_write(0x440, 0x0018);
	wm8994_write(0x450, 0x0018);
	wm8994_write(0x480, 0x0000);
	wm8994_write(0x481, 0x0000);
	wm8994_write(0x4A0, 0x0000);
	wm8994_write(0x4A1, 0x0000);
	wm8994_write(0x520, 0x0000);
	wm8994_write(0x540, 0x0018);
	wm8994_write(0x580, 0x0000);
	wm8994_write(0x581, 0x0000);
	wm8994_write(0x601, 0x0004);
	wm8994_write(0x603, 0x000C);
	wm8994_write(0x604, 0x0010);
	wm8994_write(0x605, 0x0010);
	wm8994_write(0x606, 0x0003);
	wm8994_write(0x607, 0x0003);
	wm8994_write(0x610, 0x01C0);
	wm8994_write(0x612, 0x01C0);
	wm8994_write(0x613, 0x01C0);
	wm8994_write(0x620, 0x0000);

	//roger_chen@20100519
	//enable AIF2 BCLK,LRCK
	//Rev.B and Rev.D is different
	wm8994_write(0x702, 0xA100);    
	wm8994_write(0x703, 0xA100);

	wm8994_write(0x704, 0xA100);
	wm8994_write(0x707, 0xA100);
	wm8994_write(0x708, 0x2100);
	wm8994_write(0x709, 0x2100);
	wm8994_write(0x70A, 0x2100);

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

#else //PCM_BB

/******************PCM BB BEGIN*****************/

void handsetMIC_to_baseband_to_headset(void) //pcmbaseband
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_handsetMIC_to_baseband_to_headset)return;
	wm8994_current_mode=wm8994_handsetMIC_to_baseband_to_headset;
	wm8994_reset();
	mdelay(50);
	
	wm8994_write(0x01,  0x0003|wm8994_mic_VCC);  
	mdelay(50);
	wm8994_write(0x221, 0x0700);  
	wm8994_write(0x222, 0x3126);	
	wm8994_write(0x223, 0x0100);	
	wm8994_write(0x220, 0x0004);
	mdelay(50);
	wm8994_write(0x220, 0x0005);  

	wm8994_write(0x01,  0x0303|wm8994_mic_VCC);  ///0x0303);	 // sysclk = fll (bit4 =1)   0x0011 
	wm8994_write(0x02,  0x0240);
	wm8994_write(0x03,  0x0030);
	wm8994_write(0x04,  0x3003);
	wm8994_write(0x05,  0x3003);  // i2s 16 bits
	wm8994_write(0x18,  0x010B);
	wm8994_write(0x28,  0x0030);
	wm8994_write(0x29,  0x0020);
	wm8994_write(0x2D,  0x0100);  //0x0100);DAC1L_TO_HPOUT1L    ;;;bit 8 
	wm8994_write(0x2E,  0x0100);  //0x0100);DAC1R_TO_HPOUT1R    ;;;bit 8 
	wm8994_write(0x4C,  0x9F25);
	wm8994_write(0x60,  0x00EE);
	wm8994_write(0x200, 0x0001);	
	wm8994_write(0x204, 0x0001);
	wm8994_write(0x208, 0x0007);	
	wm8994_write(0x520, 0x0000);	
	wm8994_write(0x601, 0x0004);  //AIF2DACL_TO_DAC1L
	wm8994_write(0x602, 0x0004);  //AIF2DACR_TO_DAC1R

	wm8994_write(0x610, 0x01C0);  //DAC1 Left Volume bit0~7
	wm8994_write(0x611, 0x01C0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x612, 0x01C0);  //DAC2 Left Volume bit0~7	
	wm8994_write(0x613, 0x01C0);  //DAC2 Right Volume bit0~7

	wm8994_write(0x702, 0xC100);
	wm8994_write(0x703, 0xC100);
	wm8994_write(0x704, 0xC100);
	wm8994_write(0x706, 0x4100);
	wm8994_write(0x204, 0x0011);
	wm8994_write(0x211, 0x0009);
	#ifdef TD688_MODE
	wm8994_write(0x310, 0x4108); ///0x4118);  ///interface dsp mode 16bit
	#endif
	#ifdef CHONGY_MODE
	wm8994_write(0x310, 0x4118); ///0x4118);  ///interface dsp mode 16bit
	#endif	
	#ifdef MU301_MODE
	wm8994_write(0x310, 0x4118); ///0x4118);  ///interface dsp mode 16bit
	wm8994_write(0x241, 0x2f04);
	wm8994_write(0x242, 0x0000);
	wm8994_write(0x243, 0x0300);
	wm8994_write(0x240, 0x0004);
	mdelay(40);
	wm8994_write(0x240, 0x0005);
	wm8994_write(0x204, 0x0019); 
	wm8994_write(0x211, 0x0003);
	wm8994_write(0x244, 0x0c83);
	wm8994_write(0x620, 0x0000);
	#endif
	#ifdef THINKWILL_M800_MODE
	wm8994_write(0x310, 0x4118); ///0x4118);  ///interface dsp mode 16bit
	#endif
	wm8994_write(0x313, 0x00F0);
	wm8994_write(0x314, 0x0020);
	wm8994_write(0x315, 0x0020);
	wm8994_write(0x603, 0x018c);  ///0x000C);  //Rev.D ADCL SideTone
	wm8994_write(0x604, 0x0010); //XX
	wm8994_write(0x605, 0x0010); //XX
	wm8994_write(0x621, 0x0000);  //0x0001);   ///0x0000);
	wm8994_write(0x317, 0x0003);
	wm8994_write(0x312, 0x0000); /// as slave  ///0x4000);  //AIF2 SET AS MASTER
	
	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void handsetMIC_to_baseband_to_headset_and_record(void) //pcmbaseband
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_handsetMIC_to_baseband_to_headset_and_record)return;
	wm8994_current_mode=wm8994_handsetMIC_to_baseband_to_headset_and_record;
	wm8994_reset();
	mdelay(50);

	wm8994_write(0x01,  0x0003|wm8994_mic_VCC);  
	mdelay(50);
	wm8994_write(0x221, 0x0700);  //MCLK=12MHz
	wm8994_write(0x222, 0x3126);	
	wm8994_write(0x223, 0x0100);	
	wm8994_write(0x220, 0x0004);
	mdelay(50);
	wm8994_write(0x220, 0x0005);  

	wm8994_write(0x01,  0x0303|wm8994_mic_VCC);	 
	wm8994_write(0x02,  0x0240);
	wm8994_write(0x03,  0x0030);
	wm8994_write(0x04,  0x3003);
	wm8994_write(0x05,  0x3003); 
	wm8994_write(0x18,  0x010B);  // 0x011F=+30dB for MIC
	wm8994_write(0x28,  0x0030);
	wm8994_write(0x29,  0x0020);
	wm8994_write(0x2D,  0x0100);
	wm8994_write(0x2E,  0x0100);
	wm8994_write(0x4C,  0x9F25);
	wm8994_write(0x60,  0x00EE);
	wm8994_write(0x200, 0x0001);	
	wm8994_write(0x204, 0x0001);
	wm8994_write(0x208, 0x0007);	
	wm8994_write(0x520, 0x0000);	
	wm8994_write(0x601, 0x0004);
	wm8994_write(0x602, 0x0004);

	wm8994_write(0x610, 0x01C0);  //DAC1 Left Volume bit0~7
	wm8994_write(0x611, 0x01C0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x612, 0x01C0);  //DAC2 Left Volume bit0~7	
	wm8994_write(0x613, 0x01C0);  //DAC2 Right Volume bit0~7

	wm8994_write(0x700, 0x8141);  //SYNC issue, AIF1 ADCLRC1 from LRCK1
	wm8994_write(0x702, 0xC100);
	wm8994_write(0x703, 0xC100);
	wm8994_write(0x704, 0xC100);
	wm8994_write(0x706, 0x4100);
	wm8994_write(0x204, 0x0011);  //AIF2 MCLK=FLL1
	wm8994_write(0x211, 0x0009);  //LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x310, 0x4118);  //DSP/PCM 16bits
	wm8994_write(0x313, 0x00F0);
	wm8994_write(0x314, 0x0020);
	wm8994_write(0x315, 0x0020);

	wm8994_write(0x603, 0x018c);  ///0x000C);  //Rev.D ADCL SideTone
	wm8994_write(0x604, 0x0010);
	wm8994_write(0x605, 0x0010);
	wm8994_write(0x621, 0x0000);
	//wm8994_write(0x317, 0x0003);
	//wm8994_write(0x312, 0x4000);  //AIF2 SET AS MASTER
////AIF1
	wm8994_write(0x04,   0x3303);
	wm8994_write(0x200,  0x0001);
	wm8994_write(0x208,  0x000F);
	wm8994_write(0x210,  0x0009);  //LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x300,  0x0118);  //DSP/PCM 16bits, R ADC = L ADC 
	wm8994_write(0x606,  0x0003);	
	wm8994_write(0x607,  0x0003);

////AIF1 Master Clock(SR=8KHz)
	wm8994_write(0x200,  0x0011);
	wm8994_write(0x302,  0x4000);
	wm8994_write(0x303,  0x00F0);
	wm8994_write(0x304,  0x0020);
	wm8994_write(0x305,  0x0020);

////AIF1 DAC1 HP
	wm8994_write(0x05,   0x3303);
	wm8994_write(0x420,  0x0000);
	wm8994_write(0x601,  0x0001);
	wm8994_write(0x602,  0x0001);
	wm8994_write(0x700,  0x8140);//SYNC issue, AIF1 ADCLRC1 from FLL after AIF1 MASTER!!!
	
	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void mainMIC_to_baseband_to_earpiece(void) //pcmbaseband
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_earpiece)return;
	wm8994_current_mode=wm8994_mainMIC_to_baseband_to_earpiece;
	wm8994_reset();
	mdelay(50);

	wm8994_write(0x01,  0x0003|wm8994_mic_VCC);  
	mdelay(50);
	wm8994_write(0x221, 0x0700);  //MCLK=12MHz
	wm8994_write(0x222, 0x3126);	
	wm8994_write(0x223, 0x0100);	
	wm8994_write(0x220, 0x0004);
	mdelay(50);
	wm8994_write(0x220, 0x0005);  

	wm8994_write(0x01,  0x0803|wm8994_mic_VCC);   ///0x0813);	 
	wm8994_write(0x02,  0x0240);   ///0x0110);
	wm8994_write(0x03,  0x00F0);
	wm8994_write(0x04,  0x3003);
	wm8994_write(0x05,  0x3003); 
	wm8994_write(0x18,  0x011F); 
	//wm8994_write(0x1A,  0x010B); 
	wm8994_write(0x1F,  0x0000); 
	wm8994_write(0x28,  0x0030);  ///0x0003);
	wm8994_write(0x29,  0x0020);
	//wm8994_write(0x2A,  0x0020);
	wm8994_write(0x2D,  0x0001);
	wm8994_write(0x2E,  0x0001);
	wm8994_write(0x33,  0x0018);
	//wm8994_write(0x4C,  0x9F25);
	//wm8994_write(0x60,  0x00EE);
	wm8994_write(0x200, 0x0001);
	wm8994_write(0x204, 0x0001);
	wm8994_write(0x208, 0x0007);
	wm8994_write(0x520, 0x0000);
	wm8994_write(0x601, 0x0004);
	wm8994_write(0x602, 0x0004);

	wm8994_write(0x610, 0x01C0);  //DAC1 Left Volume bit0~7
	wm8994_write(0x611, 0x01C0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x612, 0x01C0);  //DAC2 Left Volume bit0~7	
	wm8994_write(0x613, 0x01C0);  //DAC2 Right Volume bit0~7

	//wm8994_write(0x700, 0x8141);
	wm8994_write(0x702, 0xC100);
	wm8994_write(0x703, 0xC100);
	wm8994_write(0x704, 0xC100);
	wm8994_write(0x706, 0x4100);
	wm8994_write(0x204, 0x0011);  //AIF2 MCLK=FLL1
	wm8994_write(0x211, 0x0009);  //LRCK=8KHz, Rate=MCLK/1536
	///wm8994_write(0x310, 0x4108); /// 0x4118);  ///0x4118);  //DSP/PCM 16bits
	#ifdef TD688_MODE
	wm8994_write(0x310, 0x4108); ///0x4118);  ///interface dsp mode 16bit
	#endif
	#ifdef CHONGY_MODE
	wm8994_write(0x310, 0x4118); ///0x4118);  ///interface dsp mode 16bit
	#endif
	#ifdef MU301_MODE
	wm8994_write(0x310, 0x4118); ///0x4118);  ///interface dsp mode 16bit
	wm8994_write(0x241, 0x2f04);
	wm8994_write(0x242, 0x0000);
	wm8994_write(0x243, 0x0300);
	wm8994_write(0x240, 0x0004);
	mdelay(40);
	wm8994_write(0x240, 0x0005);
	wm8994_write(0x204, 0x0019); 
	wm8994_write(0x211, 0x0003);
	wm8994_write(0x244, 0x0c83);
	wm8994_write(0x620, 0x0000);
	#endif
	#ifdef THINKWILL_M800_MODE
	wm8994_write(0x310, 0x4118); ///0x4118);  ///interface dsp mode 16bit
	#endif
	wm8994_write(0x313, 0x00F0);
	wm8994_write(0x314, 0x0020);
	wm8994_write(0x315, 0x0020);

	wm8994_write(0x603, 0x018C);  //Rev.D ADCL SideTone
	wm8994_write(0x604, 0x0010);
	wm8994_write(0x605, 0x0010);
	wm8994_write(0x621, 0x0000);  ///0x0001);
	wm8994_write(0x317, 0x0003);
	wm8994_write(0x312, 0x0000);  //AIF2 SET AS MASTER
	
	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void mainMIC_to_baseband_to_earpiece_and_record(void) //pcmbaseband
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_earpiece_and_record)return;
	wm8994_current_mode=wm8994_mainMIC_to_baseband_to_earpiece_and_record;
	wm8994_reset();
	mdelay(50);

	wm8994_write(0x01,  0x0003|wm8994_mic_VCC);  
	mdelay(50);
	wm8994_write(0x221, 0x0700);  //MCLK=12MHz
	wm8994_write(0x222, 0x3126);
	wm8994_write(0x223, 0x0100);
	wm8994_write(0x220, 0x0004);
	mdelay(50);
	wm8994_write(0x220, 0x0005);  

	wm8994_write(0x01,  0x0803|wm8994_mic_VCC);
	wm8994_write(0x02,  0x0110);
	wm8994_write(0x03,  0x00F0);
	wm8994_write(0x04,  0x3003);
	wm8994_write(0x05,  0x3003); 
	wm8994_write(0x1A,  0x010B); 
	wm8994_write(0x1F,  0x0000); 
	wm8994_write(0x28,  0x0003);
	wm8994_write(0x2A,  0x0020);
	wm8994_write(0x2D,  0x0001);
	wm8994_write(0x2E,  0x0001);
	wm8994_write(0x33,  0x0018);
	//wm8994_write(0x4C,  0x9F25);
	//wm8994_write(0x60,  0x00EE);
	wm8994_write(0x200, 0x0001);	
	wm8994_write(0x204, 0x0001);
	wm8994_write(0x208, 0x0007);	
	wm8994_write(0x520, 0x0000);	
	wm8994_write(0x601, 0x0004);
	wm8994_write(0x602, 0x0004);

	wm8994_write(0x610, 0x01C0);  //DAC1 Left Volume bit0~7
	wm8994_write(0x611, 0x01C0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x612, 0x01C0);  //DAC2 Left Volume bit0~7	
	wm8994_write(0x613, 0x01C0);  //DAC2 Right Volume bit0~7

	wm8994_write(0x702, 0xC100);
	wm8994_write(0x703, 0xC100);
	wm8994_write(0x704, 0xC100);
	wm8994_write(0x706, 0x4100);
	wm8994_write(0x204, 0x0011);  //AIF2 MCLK=FLL1
	wm8994_write(0x211, 0x0009);  //LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x310, 0x4118);  //DSP/PCM 16bits
	wm8994_write(0x313, 0x00F0);
	wm8994_write(0x314, 0x0020);
	wm8994_write(0x315, 0x0020);

	wm8994_write(0x603, 0x018C);  //Rev.D ADCL SideTone
	wm8994_write(0x604, 0x0010);
	wm8994_write(0x605, 0x0010);
	wm8994_write(0x621, 0x0001);
	//wm8994_write(0x317, 0x0003);
	//wm8994_write(0x312, 0x4000);  //AIF2 SET AS MASTER

////AIF1
	wm8994_write(0x04,   0x3303);
	wm8994_write(0x200,  0x0001);
	wm8994_write(0x208,  0x000F);
	wm8994_write(0x210,  0x0009);  //LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x300,  0xC118);  //DSP/PCM 16bits, R ADC = L ADC 
	wm8994_write(0x606,  0x0003);	
	wm8994_write(0x607,  0x0003);

////AIF1 Master Clock(SR=8KHz)
	wm8994_write(0x200,  0x0011);
	wm8994_write(0x302,  0x4000);
	wm8994_write(0x303,  0x00F0);
	wm8994_write(0x304,  0x0020);
	wm8994_write(0x305,  0x0020);

////AIF1 DAC1 HP
	wm8994_write(0x05,   0x3303);
	wm8994_write(0x420,  0x0000);
	wm8994_write(0x601,  0x0001);
	wm8994_write(0x602,  0x0001);
	wm8994_write(0x700,  0x8140);//SYNC issue, AIF1 ADCLRC1 from FLL after AIF1 MASTER!!!
	
	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void mainMIC_to_baseband_to_speakers(void) //pcmbaseband
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_speakers)return;
	wm8994_current_mode=wm8994_mainMIC_to_baseband_to_speakers;
	wm8994_reset();
	mdelay(50);

	wm8994_write(0x01,  0x0003|wm8994_mic_VCC);  //0x0013);  
	mdelay(50);
	wm8994_write(0x221, 0x0700);  //MCLK=12MHz   //FLL1 CONTRLO(2)
	wm8994_write(0x222, 0x3126);  //FLL1 CONTRLO(3)	
	wm8994_write(0x223, 0x0100);  //FLL1 CONTRLO(4)	
	wm8994_write(0x220, 0x0004);  //FLL1 CONTRLO(1)
	mdelay(50);
	wm8994_write(0x220, 0x0005);  //FLL1 CONTRLO(1)

	wm8994_write(0x01,  0x3003|wm8994_mic_VCC);	 
	wm8994_write(0x02,  0x0110);
	wm8994_write(0x03,  0x0030);  ///0x0330);
	wm8994_write(0x04,  0x3003);
	wm8994_write(0x05,  0x3003); 
	wm8994_write(0x1A,  0x011F);
	wm8994_write(0x22,  0x0000);
	wm8994_write(0x23,  0x0100);  ///0x0000);
	wm8994_write(0x25,  0x0152);
	wm8994_write(0x28,  0x0003);
	wm8994_write(0x2A,  0x0020);
	wm8994_write(0x2D,  0x0001);
	wm8994_write(0x2E,  0x0001);
	wm8994_write(0x36,  0x000C);  //MIXOUTL_TO_SPKMIXL  MIXOUTR_TO_SPKMIXR
	//wm8994_write(0x4C,  0x9F25);
	//wm8994_write(0x60,  0x00EE);
	wm8994_write(0x200, 0x0001);  //AIF1 CLOCKING(1)
	wm8994_write(0x204, 0x0001);  //AIF2 CLOCKING(1)
	wm8994_write(0x208, 0x0007);  //CLOCKING(1)
	wm8994_write(0x520, 0x0000);  //AIF2 DAC FILTERS(1)
	wm8994_write(0x601, 0x0004);  //AIF2DACL_DAC1L
	wm8994_write(0x602, 0x0004);  //AIF2DACR_DAC1R

	wm8994_write(0x610, 0x01C0);  //DAC1 Left Volume bit0~7
	wm8994_write(0x611, 0x01C0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x612, 0x01C0);  //DAC2 Left Volume bit0~7	
	wm8994_write(0x613, 0x01C0);  //DAC2 Right Volume bit0~7

	wm8994_write(0x702, 0xC100);  //GPIO3
	wm8994_write(0x703, 0xC100);  //GPIO4
	wm8994_write(0x704, 0xC100);  //GPIO5
	wm8994_write(0x706, 0x4100);  //GPIO7
	wm8994_write(0x204, 0x0011);  //AIF2 MCLK=FLL1
	wm8994_write(0x211, 0x0009);  //LRCK=8KHz, Rate=MCLK/1536
	#ifdef TD688_MODE
	wm8994_write(0x310, 0xc108); ///0x4118);  ///interface dsp mode 16bit
	#endif
	#ifdef CHONGY_MODE
	wm8994_write(0x310, 0xc018); ///0x4118);  ///interface dsp mode 16bit
	#endif
	#ifdef MU301_MODE
	wm8994_write(0x310, 0xc118); ///0x4118);  ///interface dsp mode 16bit
	wm8994_write(0x241, 0x2f04);
	wm8994_write(0x242, 0x0000);
	wm8994_write(0x243, 0x0300);
	wm8994_write(0x240, 0x0004);
	mdelay(40);
	wm8994_write(0x240, 0x0005);
	wm8994_write(0x204, 0x0019);
	wm8994_write(0x211, 0x0003);
	wm8994_write(0x244, 0x0c83);
	wm8994_write(0x620, 0x0000);
	#endif
	#ifdef THINKWILL_M800_MODE
	wm8994_write(0x310, 0xc118); ///0x4118);  ///interface dsp mode 16bit
	#endif
	//wm8994_write(0x310, 0xc008);  //0xC018);//  //4118);  //DSP/PCM 16bits
	wm8994_write(0x313, 0x00F0);  //AIF2BCLK
	wm8994_write(0x314, 0x0020);  //AIF2ADCLRCK
	wm8994_write(0x315, 0x0020);  //AIF2DACLRCLK

	wm8994_write(0x603, 0x018C);  //Rev.D ADCL SideTone
	wm8994_write(0x604, 0x0020);  ///0x0010);  //ADC2_TO_DAC2L
	wm8994_write(0x605, 0x0020);  //0x0010);  //ADC2_TO_DAC2R
	wm8994_write(0x621, 0x0000);  ///0x0001);
	wm8994_write(0x317, 0x0003);
	wm8994_write(0x312, 0x0000);  //AIF2 SET AS MASTER

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void mainMIC_to_baseband_to_speakers_and_record(void) //pcmbaseband
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_speakers_and_record)return;
	wm8994_current_mode=wm8994_mainMIC_to_baseband_to_speakers_and_record;
	wm8994_reset();
	mdelay(50);

	wm8994_write(0x01,  0x0003|wm8994_mic_VCC);  
	mdelay(50);
	wm8994_write(0x221, 0x0700);  //MCLK=12MHz
	wm8994_write(0x222, 0x3126);	
	wm8994_write(0x223, 0x0100);	
	wm8994_write(0x220, 0x0004);
	mdelay(50);
	wm8994_write(0x220, 0x0005);  

	wm8994_write(0x02,  0x0110);
	wm8994_write(0x03,  0x0330);
	wm8994_write(0x04,  0x3003);
	wm8994_write(0x05,  0x3003); 
	wm8994_write(0x1A,  0x010B); 
	wm8994_write(0x22,  0x0000);
	wm8994_write(0x23,  0x0000);
	wm8994_write(0x28,  0x0003);
	wm8994_write(0x2A,  0x0020);
	wm8994_write(0x2D,  0x0001);
	wm8994_write(0x2E,  0x0001);
	wm8994_write(0x36,  0x000C);
	//wm8994_write(0x4C,  0x9F25);
	//wm8994_write(0x60,  0x00EE);
	wm8994_write(0x200, 0x0001);	
	wm8994_write(0x204, 0x0001);
	wm8994_write(0x208, 0x0007);	
	wm8994_write(0x520, 0x0000);	
	wm8994_write(0x601, 0x0004);
	wm8994_write(0x602, 0x0004);

	wm8994_write(0x610, 0x01C0);  //DAC1 Left Volume bit0~7
	wm8994_write(0x611, 0x01C0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x612, 0x01C0);  //DAC2 Left Volume bit0~7	
	wm8994_write(0x613, 0x01C0);  //DAC2 Right Volume bit0~7

	wm8994_write(0x700, 0x8141);
	wm8994_write(0x702, 0xC100);
	wm8994_write(0x703, 0xC100);
	wm8994_write(0x704, 0xC100);
	wm8994_write(0x706, 0x4100);
	wm8994_write(0x204, 0x0011);  //AIF2 MCLK=FLL1
	wm8994_write(0x211, 0x0009);  //LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x310, 0x4118);  //DSP/PCM 16bits
	wm8994_write(0x313, 0x00F0);
	wm8994_write(0x314, 0x0020);
	wm8994_write(0x315, 0x0020);

	wm8994_write(0x603, 0x018C);  //Rev.D ADCL SideTone
	wm8994_write(0x604, 0x0010);
	wm8994_write(0x605, 0x0010);
	wm8994_write(0x621, 0x0001);
	//wm8994_write(0x317, 0x0003);
	//wm8994_write(0x312, 0x4000);  //AIF2 SET AS MASTER

////AIF1
	wm8994_write(0x04,   0x3303);
	wm8994_write(0x200,  0x0001);
	wm8994_write(0x208,  0x000F);
	wm8994_write(0x210,  0x0009);  //LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x300,  0xC118);  //DSP/PCM 16bits, R ADC = L ADC 
	wm8994_write(0x606,  0x0003);	
	wm8994_write(0x607,  0x0003);

////AIF1 Master Clock(SR=8KHz)
	wm8994_write(0x200,  0x0011);
	wm8994_write(0x302,  0x4000);
	wm8994_write(0x303,  0x00F0);
	wm8994_write(0x304,  0x0020);
	wm8994_write(0x305,  0x0020);

////AIF1 DAC1 HP
	wm8994_write(0x05,   0x3303);
	wm8994_write(0x420,  0x0000);
	wm8994_write(0x601,  0x0001);
	wm8994_write(0x602,  0x0001);
	wm8994_write(0x700,  0x8140);//SYNC issue, AIF1 ADCLRC1 from FLL after AIF1 MASTER!!!
	
	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void BT_baseband(void) //pcmbaseband
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_BT_baseband)return;
	wm8994_current_mode=wm8994_BT_baseband;
	wm8994_reset();
	mdelay(50);

	wm8994_write(0x01 ,0x0003);
	mdelay (50);

	wm8994_write(0x200 ,0x0001);
	wm8994_write(0x221 ,0x0700);//MCLK=12MHz
	wm8994_write(0x222 ,0x3126);
	wm8994_write(0x223 ,0x0100);
	wm8994_write(0x220 ,0x0004);
	mdelay (50);
	wm8994_write(0x220 ,0x0005); 

	wm8994_write(0x02 ,0x0000); 
	wm8994_write(0x200 ,0x0011);// AIF1 MCLK=FLL1
	wm8994_write(0x210 ,0x0009);// LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x300 ,0x4018);// DSP/PCM 16bits

	wm8994_write(0x204 ,0x0011);// AIF2 MCLK=FLL1
	wm8994_write(0x211 ,0x0009);// LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x310 ,0x4118);// DSP/PCM 16bits
	wm8994_write(0x208 ,0x000F); 

/////AIF1
	wm8994_write(0x700 ,0x8101);
/////AIF2
	wm8994_write(0x702 ,0xC100);
	wm8994_write(0x703 ,0xC100);
	wm8994_write(0x704 ,0xC100);
	wm8994_write(0x706 ,0x4100);
/////AIF3
	wm8994_write(0x707 ,0xA100); 
	wm8994_write(0x708 ,0xA100);
	wm8994_write(0x709 ,0xA100); 
	wm8994_write(0x70A ,0xA100);

	wm8994_write(0x06 ,0x0001);

	wm8994_write(0x02 ,0x0300);
	wm8994_write(0x03 ,0x0030);
	wm8994_write(0x04 ,0x3301);//ADCL off
	wm8994_write(0x05 ,0x3301);//DACL off

//	wm8994_write(0x29 ,0x0005);  
	wm8994_write(0x2A ,0x0005);

	wm8994_write(0x313 ,0x00F0);
	wm8994_write(0x314 ,0x0020);
	wm8994_write(0x315 ,0x0020);

//	wm8994_write(0x2D ,0x0001);
	wm8994_write(0x2E ,0x0001);
	wm8994_write(0x420 ,0x0000);
	wm8994_write(0x520 ,0x0000);
	wm8994_write(0x601 ,0x0001);
	wm8994_write(0x602 ,0x0001);
	wm8994_write(0x604 ,0x0001);
	wm8994_write(0x605 ,0x0001);
//	wm8994_write(0x606 ,0x0002);
	wm8994_write(0x607 ,0x0002);
//	wm8994_write(0x610, 0x01C0);  //DAC1 Left Volume bit0~7
	wm8994_write(0x611, 0x01C0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x612, 0x01C0);  //DAC2 Left Volume bit0~7	
	wm8994_write(0x613, 0x01C0);  //DAC2 Right Volume bit0~7


	wm8994_write(0x312 ,0x4000);

	wm8994_write(0x606 ,0x0001);
	wm8994_write(0x607 ,0x0003);//R channel for data mix/CPU record data


////////////HP output test
	wm8994_write(0x01 ,0x0303);
	wm8994_write(0x4C ,0x9F25);
	wm8994_write(0x60 ,0x00EE);
///////////end HP test

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}

void BT_baseband_and_record(void) //pcmbaseband
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(wm8994_current_mode==wm8994_BT_baseband_and_record)return;
	wm8994_current_mode=wm8994_BT_baseband_and_record;
	wm8994_reset();
	mdelay(50);

	wm8994_write(0x01  ,0x0003);
	mdelay (50);

	wm8994_write(0x200 ,0x0001);
	wm8994_write(0x221 ,0x0700);//MCLK=12MHz
	wm8994_write(0x222 ,0x3126);
	wm8994_write(0x223 ,0x0100);
	wm8994_write(0x220 ,0x0004);
	mdelay (50);
	wm8994_write(0x220 ,0x0005); 

	wm8994_write(0x02 ,0x0000); 
	wm8994_write(0x200 ,0x0011);// AIF1 MCLK=FLL1
	wm8994_write(0x210 ,0x0009);// LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x300 ,0x4018);// DSP/PCM 16bits

	wm8994_write(0x204 ,0x0011);// AIF2 MCLK=FLL1
	wm8994_write(0x211 ,0x0009);// LRCK=8KHz, Rate=MCLK/1536
	wm8994_write(0x310 ,0x4118);// DSP/PCM 16bits
	wm8994_write(0x208 ,0x000F); 

/////AIF1
	wm8994_write(0x700 ,0x8101);
/////AIF2
	wm8994_write(0x702 ,0xC100);
	wm8994_write(0x703 ,0xC100);
	wm8994_write(0x704 ,0xC100);
	wm8994_write(0x706 ,0x4100);
/////AIF3
	wm8994_write(0x707 ,0xA100); 
	wm8994_write(0x708 ,0xA100);
	wm8994_write(0x709 ,0xA100); 
	wm8994_write(0x70A ,0xA100);

	wm8994_write(0x06 ,0x0001);

	wm8994_write(0x02 ,0x0300);
	wm8994_write(0x03 ,0x0030);
	wm8994_write(0x04 ,0x3301);//ADCL off
	wm8994_write(0x05 ,0x3301);//DACL off

//   wm8994_write(0x29 ,0x0005);  
	wm8994_write(0x2A ,0x0005);

	wm8994_write(0x313 ,0x00F0);
	wm8994_write(0x314 ,0x0020);
	wm8994_write(0x315 ,0x0020);

//   wm8994_write(0x2D ,0x0001);
	wm8994_write(0x2E ,0x0001);
	wm8994_write(0x420 ,0x0000);
	wm8994_write(0x520 ,0x0000);
//  wm8994_write(0x601 ,0x0001);
	wm8994_write(0x602 ,0x0001);
	wm8994_write(0x604 ,0x0001);
	wm8994_write(0x605 ,0x0001);
//  wm8994_write(0x606 ,0x0002);
	wm8994_write(0x607 ,0x0002);
//	wm8994_write(0x610, 0x01C0);  //DAC1 Left Volume bit0~7
	wm8994_write(0x611, 0x01C0);  //DAC1 Right Volume bit0~7
	wm8994_write(0x612, 0x01C0);  //DAC2 Left Volume bit0~7	
	wm8994_write(0x613, 0x01C0);  //DAC2 Right Volume bit0~7


	wm8994_write(0x312 ,0x4000);

	wm8994_write(0x606 ,0x0001);
	wm8994_write(0x607 ,0x0003);//R channel for data mix/CPU record data


////////////HP output test
	wm8994_write(0x01 ,0x0303);
	wm8994_write(0x4C ,0x9F25); 
	wm8994_write(0x60 ,0x00EE); 
///////////end HP test

	wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
}
#endif //PCM_BB

typedef void (wm8994_codec_fnc_t) (void);

wm8994_codec_fnc_t *wm8994_codec_sequence[] = {
	AP_to_headset,
	AP_to_speakers,
	recorder_and_AP_to_headset,
	recorder_and_AP_to_speakers,
	FM_to_headset,
	FM_to_headset_and_record,
	FM_to_speakers,
	FM_to_speakers_and_record,
	handsetMIC_to_baseband_to_headset,
	handsetMIC_to_baseband_to_headset_and_record,
	mainMIC_to_baseband_to_earpiece,
	mainMIC_to_baseband_to_earpiece_and_record,
	mainMIC_to_baseband_to_speakers,
	mainMIC_to_baseband_to_speakers_and_record,
	BT_baseband,
	BT_baseband_and_record,
};

/********************set wm8994 volume*****volume=0\1\2\3\4\5\6\7*******************/

void wm8994_codec_set_volume(unsigned char system_type,unsigned char volume)
{
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	if(system_type == VOICE_CALL||system_type == BLUETOOTH_SCO )
	{
		if(volume<=call_maxvol)
			call_vol=volume;
		else{
			printk("%s----%d::call volume more than max value 7\n",__FUNCTION__,__LINE__);
			call_vol=call_maxvol;
		}
		if(wm8994_current_mode<null&&wm8994_current_mode>=wm8994_handsetMIC_to_baseband_to_headset)
			wm8994_set_volume(wm8994_current_mode,call_vol,call_maxvol);
	}
	else
		printk("%s----%d::system type error!\n",__FUNCTION__,__LINE__);
}

void wm8994_set_volume(unsigned char wm8994_mode,unsigned char volume,unsigned char max_volume)
{
	unsigned short lvol=0,rvol=0;
	DBG("%s::%d\n",__FUNCTION__,__LINE__);

	dump_stack();
	if(volume>max_volume)volume=max_volume;
	printk("wm8994_mode is %d,volume is %ds,max_volume is %d\n",wm8994_mode,volume,max_volume);
	
	if(wm8994_mode==wm8994_handsetMIC_to_baseband_to_headset_and_record||
	wm8994_mode==wm8994_handsetMIC_to_baseband_to_headset)
	{
		wm8994_read(0x001C, &lvol);
		wm8994_read(0x001D, &rvol);
		//HPOUT1L_VOL bit 0~5 /-57dB to +6dB in 1dB steps
		wm8994_write(0x001C, (lvol&~0x003f)|headset_vol_table[volume]); 
		//HPOUT1R_VOL bit 0~5 /-57dB to +6dB in 1dB steps
		wm8994_write(0x001D, (lvol&~0x003f)|headset_vol_table[volume]); 
	}
	else if(wm8994_mode==wm8994_mainMIC_to_baseband_to_speakers_and_record||
	wm8994_mode==wm8994_mainMIC_to_baseband_to_speakers||
	wm8994_mode==wm8994_mainMIC_to_baseband_with_AP_to_speakers)
	{
		wm8994_read(0x0026, &lvol);
		wm8994_read(0x0027, &rvol);
		//SPKOUTL_VOL bit 0~5 /-57dB to +6dB in 1dB steps
		wm8994_write(0x0026, (lvol&~0x003f)|speakers_vol_table[volume]);
		//SPKOUTR_VOL bit 0~5 /-57dB to +6dB in 1dB steps
		wm8994_write(0x0027, (lvol&~0x003f)|speakers_vol_table[volume]);
	}
	else if(wm8994_mode==wm8994_mainMIC_to_baseband_to_earpiece||
	wm8994_mode==wm8994_mainMIC_to_baseband_to_earpiece_and_record)
	{
		wm8994_read(0x0020, &lvol);
		wm8994_read(0x0021, &rvol);
		//MIXOUTL_VOL bit 0~5 /-57dB to +6dB in 1dB steps
		wm8994_write(0x0020, (lvol&~0x003f)|earpiece_vol_table[volume]);
		//MIXOUTR_VOL bit 0~5 /-57dB to +6dB in 1dB steps
		wm8994_write(0x0021, (lvol&~0x003f)|earpiece_vol_table[volume]);
	}
	else if(wm8994_mode==wm8994_BT_baseband||wm8994_mode==wm8994_BT_baseband_and_record)
	{
		wm8994_read(0x0019, &lvol);
		wm8994_read(0x001b, &rvol);
		//bit 0~4 /-16.5dB to +30dB in 1.5dB steps
		wm8994_write(0x0019, (lvol&~0x000f)|BT_vol_table[volume]);
		//bit 0~4 /-16.5dB to +30dB in 1.5dB steps
		wm8994_write(0x001b, (lvol&~0x000f)|BT_vol_table[volume]);
	}
}

#define SOC_DOUBLE_SWITCH_WM8994CODEC(xname, route) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_route, \
	.get = snd_soc_get_route, .put = snd_soc_put_route, \
	.private_value = route }

int snd_soc_info_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	
	//uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0;
	return 0;
}

int snd_soc_get_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static volatile int first_put_route = 1;

int snd_soc_put_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int route = kcontrol->private_value & 0xff;

	printk("route = %d\n", route);

	first_put_route = 0;

	switch(route)
	{
		/* Speaker*/
		case SPEAKER_NORMAL: //AP-> 8994Codec -> Speaker
			AP_to_headset();
			//AP_to_speakers();
			//handsetMIC_to_baseband_to_headset();
			break;
		case SPEAKER_INCALL: //BB-> 8994Codec -> Speaker
			mainMIC_to_baseband_to_speakers();
			break;		
			
		/* Headset */	
		case HEADSET_NORMAL:	//AP-> 8994Codec -> Headset
			AP_to_headset();
			break;
		case HEADSET_INCALL:	//AP-> 8994Codec -> Headset
			handsetMIC_to_baseband_to_headset();
			break;		    

		/* Earpiece*/			    
		case EARPIECE_INCALL:	//:BB-> 8994Codec -> EARPIECE
			mainMIC_to_baseband_to_earpiece();
			break;

		case EARPIECE_NORMAL:	//:BB-> 8994Codec -> EARPIECE
			printk("EARPIECE_NORMAL-----cjq--------\n");
			if(wm8994_current_mode==wm8994_handsetMIC_to_baseband_to_headset)
				AP_to_headset();
			else if(wm8994_current_mode==wm8994_mainMIC_to_baseband_to_speakers||
				wm8994_current_mode==wm8994_mainMIC_to_baseband_to_earpiece)
				AP_to_speakers();
			else if(wm8994_current_mode==wm8994_AP_to_speakers||
				wm8994_current_mode==wm8994_AP_to_headset)
				break;
			else{
				AP_to_speakers();
				printk("%s--%d--: wm8994 with null mode\n",__FUNCTION__,__LINE__);
			}	
			break;


		/* BLUETOOTH_SCO*/		    	
		case BLUETOOTH_SCO_INCALL:	//BB-> 8994Codec -> BLUETOOTH_SCO  
			BT_baseband();
			break;

		/* BLUETOOTH_A2DP*/			    
		case BLUETOOTH_A2DP_NORMAL:	//AP-> 8994Codec -> BLUETOOTH_A2DP
			break;
		    
		case MIC_CAPTURE:
			if(wm8994_current_mode==wm8994_AP_to_headset)
				recorder_and_AP_to_headset();
			else if(wm8994_current_mode==wm8994_AP_to_speakers)
				recorder_and_AP_to_speakers();
			else if(wm8994_current_mode==wm8994_recorder_and_AP_to_speakers||
				wm8994_current_mode==wm8994_recorder_and_AP_to_headset)
				break;
			else{
				recorder_and_AP_to_speakers();
				printk("%s--%d--: wm8994 with null mode\n",__FUNCTION__,__LINE__);
			}	
			break;

		case EARPIECE_RINGTONE:
			printk("EARPIECE_RINGTONE-----cjq--------\n");
			AP_to_speakers();
			break;

		case HEADSET_RINGTONE:
			printk("HEADSET_RINGTONE-----cjq--------\n");
			AP_to_headset();
			break;

		case SPEAKER_RINGTONE:
			printk("SPEAKER_RINGTONE-----cjq--------\n");
			AP_to_speakers();
			break;

		default:
			//codec_daout_route();
			break;
	}
	return 0;
}

/*
 * WM8994 Controls
 */
static const char *wm8994_bass[] = {"Linear Control", "Adaptive Boost"};
static const char *wm8994_bass_filter[] = { "130Hz @ 48kHz", "200Hz @ 48kHz" };
static const char *wm8994_treble[] = {"8kHz", "4kHz"};
static const char *wm8994_3d_lc[] = {"200Hz", "500Hz"};
static const char *wm8994_3d_uc[] = {"2.2kHz", "1.5kHz"};
static const char *wm8994_3d_func[] = {"Capture", "Playback"};
static const char *wm8994_alc_func[] = {"Off", "Right", "Left", "Stereo"};
static const char *wm8994_ng_type[] = {"Constant PGA Gain","Mute ADC Output"};
static const char *wm8994_line_mux[] = {"Line 1", "Line 2", "Line 3", "PGA","Differential"};
static const char *wm8994_pga_sel[] = {"Line 1", "Line 2", "Line 3","Differential"};
//static const char *wm8994_out3[] = {"VREF", "ROUT1 + Vol", "MonoOut",
	//"ROUT1"};
static const char *wm8994_diff_sel[] = {"Line 1", "Line 2"};
static const char *wm8994_adcpol[] = {"Normal", "L Invert", "R Invert","L + R Invert"};
static const char *wm8994_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *wm8994_mono_mux[] = {"Stereo", "Mono (Left)","Mono (Right)", "Digital Mono"};

static const struct soc_enum wm8994_enum[] = {
SOC_ENUM_SINGLE(WM8994_BASS, 7, 2, wm8994_bass),
SOC_ENUM_SINGLE(WM8994_BASS, 6, 2, wm8994_bass_filter),
SOC_ENUM_SINGLE(WM8994_TREBLE, 6, 2, wm8994_treble),
SOC_ENUM_SINGLE(WM8994_3D, 5, 2, wm8994_3d_lc),
SOC_ENUM_SINGLE(WM8994_3D, 6, 2, wm8994_3d_uc),
SOC_ENUM_SINGLE(WM8994_3D, 7, 2, wm8994_3d_func),
SOC_ENUM_SINGLE(WM8994_ALC1, 7, 4, wm8994_alc_func),
SOC_ENUM_SINGLE(WM8994_NGATE, 1, 2, wm8994_ng_type),
SOC_ENUM_SINGLE(WM8994_LOUTM1, 0, 5, wm8994_line_mux),
SOC_ENUM_SINGLE(WM8994_ROUTM1, 0, 5, wm8994_line_mux),
SOC_ENUM_SINGLE(WM8994_LADCIN, 6, 4, wm8994_pga_sel), /* 10 */
SOC_ENUM_SINGLE(WM8994_RADCIN, 6, 4, wm8994_pga_sel),
///SOC_ENUM_SINGLE(WM8994_ADCTL2, 7, 4, wm8994_out3),
SOC_ENUM_SINGLE(WM8994_ADCIN, 8, 2, wm8994_diff_sel),
SOC_ENUM_SINGLE(WM8994_ADCDAC, 5, 4, wm8994_adcpol),
SOC_ENUM_SINGLE(WM8994_ADCDAC, 1, 4, wm8994_deemph),
SOC_ENUM_SINGLE(WM8994_ADCIN, 6, 4, wm8994_mono_mux), /* 16 */

};

static const struct snd_kcontrol_new wm8994_snd_controls[] = {
#if 1
/* 喇叭 */
SOC_DOUBLE_SWITCH_WM8994CODEC("Speaker incall Switch", SPEAKER_INCALL),	
SOC_DOUBLE_SWITCH_WM8994CODEC("Speaker normal Switch", SPEAKER_NORMAL),
/* 听筒 */
SOC_DOUBLE_SWITCH_WM8994CODEC("Earpiece incall Switch", EARPIECE_INCALL),	
SOC_DOUBLE_SWITCH_WM8994CODEC("Earpiece normal Switch", EARPIECE_NORMAL),
/* 耳机 */
SOC_DOUBLE_SWITCH_WM8994CODEC("Headset incall Switch", HEADSET_INCALL),	
SOC_DOUBLE_SWITCH_WM8994CODEC("Headset normal Switch", HEADSET_NORMAL),
/* 蓝牙SCO */
SOC_DOUBLE_SWITCH_WM8994CODEC("Bluetooth incall Switch", BLUETOOTH_SCO_INCALL),	
SOC_DOUBLE_SWITCH_WM8994CODEC("Bluetooth normal Switch", BLUETOOTH_SCO_NORMAL),
/* 蓝牙A2DP */
SOC_DOUBLE_SWITCH_WM8994CODEC("Bluetooth-A2DP incall Switch", BLUETOOTH_A2DP_INCALL),	
SOC_DOUBLE_SWITCH_WM8994CODEC("Bluetooth-A2DP normal Switch", BLUETOOTH_A2DP_NORMAL),
/* 耳麦 */
SOC_DOUBLE_SWITCH_WM8994CODEC("Capture Switch", MIC_CAPTURE),

SOC_DOUBLE_SWITCH_WM8994CODEC("Earpiece ringtone Switch",EARPIECE_RINGTONE),

SOC_DOUBLE_SWITCH_WM8994CODEC("Speaker ringtone Switch",SPEAKER_RINGTONE),

SOC_DOUBLE_SWITCH_WM8994CODEC("Headset ringtone Switch",HEADSET_RINGTONE),
#endif
};

/* add non dapm controls */
static int wm8994_add_controls(struct snd_soc_codec *codec)
{
	int err, i;
	for (i = 0; i < ARRAY_SIZE(wm8994_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8994_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

/*
 * DAPM Controls
 */

/* Left Mixer */
static const struct snd_kcontrol_new wm8994_left_mixer_controls[] = {
SOC_DAPM_SINGLE("Playback Switch", WM8994_LOUTM1, 8, 1, 0),
SOC_DAPM_SINGLE("Left Bypass Switch", WM8994_LOUTM1, 7, 1, 0),
SOC_DAPM_SINGLE("Right Playback Switch", WM8994_LOUTM2, 8, 1, 0),
SOC_DAPM_SINGLE("Right Bypass Switch", WM8994_LOUTM2, 7, 1, 0),
};

/* Right Mixer */
static const struct snd_kcontrol_new wm8994_right_mixer_controls[] = {
SOC_DAPM_SINGLE("Left Playback Switch", WM8994_ROUTM1, 8, 1, 0),
SOC_DAPM_SINGLE("Left Bypass Switch", WM8994_ROUTM1, 7, 1, 0),
SOC_DAPM_SINGLE("Playback Switch", WM8994_ROUTM2, 8, 1, 0),
SOC_DAPM_SINGLE("Right Bypass Switch", WM8994_ROUTM2, 7, 1, 0),
};

/* Left Line Mux */
static const struct snd_kcontrol_new wm8994_left_line_controls =
SOC_DAPM_ENUM("Route", wm8994_enum[8]);

/* Right Line Mux */
static const struct snd_kcontrol_new wm8994_right_line_controls =
SOC_DAPM_ENUM("Route", wm8994_enum[9]);

/* Left PGA Mux */
static const struct snd_kcontrol_new wm8994_left_pga_controls =
SOC_DAPM_ENUM("Route", wm8994_enum[10]);

/* Right PGA Mux */
static const struct snd_kcontrol_new wm8994_right_pga_controls =
SOC_DAPM_ENUM("Route", wm8994_enum[11]);

/* Differential Mux */
static const struct snd_kcontrol_new wm8994_diffmux_controls =
SOC_DAPM_ENUM("Route", wm8994_enum[12]);  //13]);

/* Mono ADC Mux */
static const struct snd_kcontrol_new wm8994_monomux_controls =
SOC_DAPM_ENUM("Route", wm8994_enum[15]);  //16]);

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:5;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0x6, 0x0},
	{11289600, 8000, 1408, 0x16, 0x0},
	{18432000, 8000, 2304, 0x7, 0x0},
	{16934400, 8000, 2112, 0x17, 0x0},
	{12000000, 8000, 1500, 0x6, 0x1},
    
	/* 11.025k */
	{11289600, 11025, 1024, 0x18, 0x0},
	{16934400, 11025, 1536, 0x19, 0x0},
	{12000000, 11025, 1088, 0x19, 0x1},
    
    /* 12k */
	{12288000, 12000, 1024, 0x8, 0x0},
	{18432000, 12000, 1536, 0x9, 0x0},
	{12000000, 12000, 1000, 0x8, 0x1},
    
	/* 16k */
	{12288000, 16000, 768, 0xa, 0x0},
	{18432000, 16000, 1152, 0xb, 0x0},
	{12000000, 16000, 750, 0xa, 0x1},
    
	/* 22.05k */
	{11289600, 22050, 512, 0x1a, 0x0},
	{16934400, 22050, 768, 0x1b, 0x0},
	{12000000, 22050, 544, 0x1b, 0x1},
    
    /* 24k */
	{12288000, 24000, 512, 0x1c, 0x0},
	{18432000, 24000, 768, 0x1d, 0x0},
	{12000000, 24000, 500, 0x1c, 0x1},
	
	/* 32k */
	{12288000, 32000, 384, 0xc, 0x0},
	{18432000, 32000, 576, 0xd, 0x0},
	{12000000, 32000, 375, 0xa, 0x1},
    
	/* 44.1k */
	{11289600, 44100, 256, 0x10, 0x0},
	{16934400, 44100, 384, 0x11, 0x0},
	{12000000, 44100, 272, 0x11, 0x1},
    
	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0},
	{18432000, 48000, 384, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x1},
    
	/* 88.2k */
	{11289600, 88200, 128, 0x1e, 0x0},
	{16934400, 88200, 192, 0x1f, 0x0},
	{12000000, 88200, 136, 0x1f, 0x1},
    
	/* 96k */
	{12288000, 96000, 128, 0xe, 0x0},
	{18432000, 96000, 192, 0xf, 0x0},
	{12000000, 96000, 125, 0xe, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;
    
	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}
    
	printk(KERN_ERR "wm8994: could not get coeff for mclk %d @ rate %d\n",
		mclk, rate);
	return -EINVAL;
}

static int wm8994_set_dai_sysclk(struct snd_soc_codec_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct wm8994_priv *wm8994 = codec->private_data;

	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
		wm8994->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int wm8994_set_dai_fmt(struct snd_soc_codec_dai *codec_dai,
		unsigned int fmt)
{
	u16 iface = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface = 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}
    
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}
    
	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int wm8994_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct wm8994_priv *wm8994 = codec->private_data;
	
	/*by Vincent Hsiung for EQ Vol Change*/
	#define HW_PARAMS_FLAG_EQVOL_ON 0x21
	#define HW_PARAMS_FLAG_EQVOL_OFF 0x22
	if (params->flags == HW_PARAMS_FLAG_EQVOL_ON)
	{
		u16 r5 = wm8994_read_reg_cache(codec, WM8994_ADCDAC);
		r5 &= (~0x80); //DAC DIV disable
		return 0;
	}
	else if (params->flags == HW_PARAMS_FLAG_EQVOL_OFF)
	{
		u16 r5 = wm8994_read_reg_cache(codec, WM8994_ADCDAC);
		r5 |= 0x80; //DAC DIV enable
		return 0;
	} 

	get_coeff(wm8994->sysclk, params_rate(params));

	/* bit size */
	params_format(params);
    
	return 0;
}

static int wm8994_mute(struct snd_soc_codec_dai *dai, int mute)
{
	unsigned short vol_L = 0;
	unsigned short vol_R = 0;

	wm8994_read(0x001C, &vol_L);
	wm8994_read(0x001D, &vol_R);

	if(mute){
		vol_L &= ~(1 << 6);
		vol_R &= ~(1 << 6);
	}
	else{
		vol_L |= (1 << 6);
		vol_R |= (1 << 6);
	}

	wm8994_write(0x001C, vol_L);
	wm8994_write(0x001D, vol_R);
	
	return 0;
}

static int wm8994_dapm_event(struct snd_soc_codec *codec, int event)
{
	codec->dapm_state = event;
	return 0;
}

#define WM8994_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_8000 |\
		SNDRV_PCM_RATE_48000)

#define WM8994_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	 SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_codec_dai wm8994_dai = {
	.name = "WM8994",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8994_RATES,
		.formats = WM8994_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = WM8994_RATES,
		.formats = WM8994_FORMATS,},
	.ops = {
		.hw_params = wm8994_pcm_hw_params,
	},
	.dai_ops = {
		.digital_mute = wm8994_mute,
		.set_fmt = wm8994_set_dai_fmt,
		.set_sysclk = wm8994_set_dai_sysclk,
		//.set_volume = wm8994_codec_set_volume,//cjq
	},
};
EXPORT_SYMBOL_GPL(wm8994_dai);

static void wm8994_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
		container_of(work, struct snd_soc_codec, delayed_work.work);
	wm8994_dapm_event(codec, codec->dapm_state);
}

static int wm8994_suspend(struct platform_device *pdev, pm_message_t state)
{
    	DBG("%s----%d\n",__FUNCTION__,__LINE__);

	platform_get_drvdata(pdev);

	wm8994_reset();
	mdelay(WM8994_DELAY);

	return 0;
}

static int wm8994_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	wm8994_codec_fnc_t **wm8994_fnc_ptr=wm8994_codec_sequence;
	//int reg;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	DBG("%s----%d\n",__FUNCTION__,__LINE__);

	mdelay(50);
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8994_reg); i++) {
		if (i == WM8994_RESET)
			continue;
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->hw_write(codec->control_data, data, 2);
	}
   
       codec->dapm_state = SNDRV_CTL_POWER_D3hot;
	/* charge wm8994 caps */
	if (codec->suspend_dapm_state == SNDRV_CTL_POWER_D0) {
		codec->dapm_state = SNDRV_CTL_POWER_D0;
		schedule_delayed_work(&codec->delayed_work, msecs_to_jiffies(1000));
	}

	if(wm8994_current_mode<=wm8994_AP_to_speakers)
	{
		wm8994_fnc_ptr+=wm8994_current_mode;
		(*wm8994_fnc_ptr)() ;
	}
	else if(wm8994_current_mode>wm8994_BT_baseband_and_record)
	{
		printk("%s--%d--: Wm8994 resume with null mode\n",__FUNCTION__,__LINE__);
	}
	else
	{
		wm8994_fnc_ptr+=wm8994_current_mode;
		(*wm8994_fnc_ptr)() ;
		printk("%s--%d--: Wm8994 resume with error mode\n",__FUNCTION__,__LINE__);
	}
    return 0;
}

/*
 * initialise the WM8994 driver
 * register the mixer and dsp interfaces with the kernel
 */

static int wm8994_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;
	gpCodec = socdev->codec;

	wm8994_reset();
	mdelay(500);

	codec->name = "WM8994";
	codec->owner = THIS_MODULE;
	codec->read = wm8994_read_reg_cache;
	codec->write = wm8994_codec_write;
	codec->dapm_event = wm8994_dapm_event;
	codec->dai = &wm8994_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(wm8994_reg);
	codec->reg_cache = kmemdup(wm8994_reg, sizeof(wm8994_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
    
    frist_mute = 1;
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {

		printk(KERN_ERR "wm8994: failed to create pcms\n");

		goto pcm_err;
	}
    
	/* charge output caps */
	codec->dapm_state = SNDRV_CTL_POWER_D3hot;
	schedule_delayed_work(&codec->delayed_work, msecs_to_jiffies(1000));

	wm8994_add_controls(codec);
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "wm8994: failed to register card\n");
		goto card_err;
	}
	return ret;

card_err:
	  snd_soc_free_pcms(socdev);
	  snd_soc_dapm_free(socdev);
pcm_err:
	  kfree(codec->reg_cache);
	  return ret;
}




struct wm8994_out_ctrl_t {
	int old_state;
	unsigned long cnt;
	struct delayed_work delay_work;	
};

enum wm8994_out_status {
	NONE,
	HEADPHONE,
	SPEACKERS,
};


struct wm8994_out_ctrl_t wm8994_out_ctrl;

static int wm8994_headphone_detect(void)
{
	if(GPIOGetPinLevel(GPIOPortF_Pin7) == 0){
		return HEADPHONE;
	}
	else{
		return SPEACKERS;
	}
}

static void wm8994_out_switch_delaywork(struct work_struct *work)
{
	struct wm8994_out_ctrl_t *output_ctrl = (struct wm8994_out_ctrl_t *)container_of((void *)work, struct wm8994_out_ctrl_t, delay_work);
	int state;
	
	if(first_put_route == 1){
		goto NEXT;
	}

	state = wm8994_headphone_detect();
	
	if (state != output_ctrl->old_state){
		output_ctrl->cnt++;
	}
	else{
		output_ctrl->cnt = 0;
	}

	if(output_ctrl->cnt > 5){
		
		if(state == HEADPHONE){
			AP_to_headset();
		}
		else{
			AP_to_speakers();
		}

		output_ctrl->cnt = 0;
		output_ctrl->old_state = state;
	}

NEXT:
	schedule_delayed_work(&output_ctrl->delay_work, 20);
}



static int wm8994_headphone_detect_init(void)
{
	struct wm8994_out_ctrl_t *output_ctrl = &wm8994_out_ctrl;
	printk("%s-%s\n", __FILE__, __FUNCTION__);

	rockchip_mux_api_set(GPIOE_SPI1_FLASH_SEL2_NAME, IOMUXA_GPIO1_A3B7);
	GPIOSetPinDirection(GPIOPortF_Pin7,GPIO_IN); /* PF7 headphone detect*/
	GPIOPullUpDown(GPIOPortF_Pin7,GPIOPullUp);

	INIT_DELAYED_WORK(&output_ctrl->delay_work, wm8994_out_switch_delaywork);

	output_ctrl->old_state = NONE;

	schedule_delayed_work(&output_ctrl->delay_work, 100);

	return 0;
}

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */
static struct snd_soc_device *wm8994_socdev;

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)

/*
 * WM8731 2 wire address is determined by GPIO5
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8994_i2c_driver;
static struct i2c_client client_template;

static int wm8994_codec_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = wm8994_socdev;
	struct wm8994_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;

	if (addr != setup->i2c_address)
		return -ENODEV;
    
	client_template.adapter = adap;
	client_template.addr = addr;
    	client_template.mode = NORMALMODE;
	client_template.Channel = I2C_CH0;
	client_template.addressBit=I2C_7BIT_ADDRESS_16BIT_REG;
	client_template.speed = 200;
	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;
    
	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		err("failed to attach codec at addr %x\n", addr);
		goto err;
	}
    
	ret = wm8994_init(socdev);
	if (ret < 0) {
	err("failed to initialise WM8994\n");
		goto err;
	}
	
	return ret;
    
err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int wm8994_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int wm8994_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8994_codec_probe);
}

/* corgi i2c codec control layer */
static struct i2c_driver wm8994_i2c_driver = {
	.driver = {
		.name = "WM8994 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_WM8994,
	.attach_adapter = wm8994_i2c_attach,
	.detach_client =  wm8994_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8994",
	.driver = &wm8994_i2c_driver,
};
#endif


//Located at /sys/class/sound/card0/device
ssize_t wm8994_regs(struct device *dev, const char *buf, size_t count)
{
	unsigned short c;
	int ret, i;

    //printk("CMD: %s", buf);
    
    if (strstr(buf, "dump"))
    {
		printk("wm8994 show regs:\n");
        for (i = 0; i <= WM8994_CACHE_REGNUM; i++){
			ret = wm8994_read(i, &c);
			if(ret != 0){
				printk("wm8994_read err, reg addr: 0x%04x\n", i);
				break;
			}
			
            printk("REG: 0x%02x = 0x%04x\n", i, c);
        }

        for (i = 0x302; i <= 0x305; i++){
			ret = wm8994_read(i, &c);
			if(ret != 0){
				printk("wm8994_read err, reg addr: 0x%04x\n", i);
				break;
			}
			
            printk("REG: 0x%02x = 0x%04x\n", i, c);
        }

    }
	else if(strstr(buf, "headphone")){
		AP_to_headset();
	}
	else if(strstr(buf, "speaker")){
		AP_to_speakers();
	}

    return count;
}

static DEVICE_ATTR(wm8994_test, 0666, NULL, wm8994_regs);

static int wm8994_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct wm8994_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec;
	struct wm8994_priv *wm8994;
	int ret = 0;
    
	info("WM8994 Audio Codec %s", WM8994_VERSION);
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;
    
	wm8994 = kzalloc(sizeof(struct wm8994_priv), GFP_KERNEL);
	if (wm8994 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
    
	codec->private_data = wm8994;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	wm8994_socdev = socdev;
	INIT_DELAYED_WORK(&codec->delayed_work, wm8994_work);
	
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
        codec->hw_read = (hw_read_t)i2c_master_recv; //增加读接口
		ret = i2c_add_driver(&wm8994_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
	}
#else
		/* Add other interfaces here */
#endif

    ret = device_create_file(socdev->dev, &dev_attr_wm8994_test);
    if (ret != 0)
        printk("Create sys file failed.\n");

	//wm8994_headphone_detect_init();

	return ret;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	DBG("%s----%d\n",__FUNCTION__,__LINE__);

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);
    
	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int wm8994_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	DBG("%s----%d\n",__FUNCTION__,__LINE__);

	if (codec->control_data)
		wm8994_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8994_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);
   
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8994 = {
	.probe = 	wm8994_probe,
	.remove = 	wm8994_remove,
	.suspend = 	wm8994_suspend,
	.resume =	wm8994_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_wm8994);

MODULE_DESCRIPTION("ASoC WM8994 driver");
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_LICENSE("GPL");
