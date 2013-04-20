/*
 * wm8904.c -- WM8904 ALSA SoC audio driver
 *
 * Copyright (C) 2011 
 *
 *
 * Based on WM8904.c
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
#include <asm/arch/rk28_scu.h>
#include <sound/tlv.h>


//#include <sound/wm8904.h>
#include "wm8904.h"
#include "wm8988.h"

#if 0
#define printk_d(fmt, args...)  printk(KERN_INFO "[WM8904] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define printk_d(fmt, args...)
#endif

#define printk_e(fmt, args...)  printk(KERN_ERR "[WM8904] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#define printk_i(fmt, args...)  printk(KERN_INFO "[WM8904] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)

typedef enum snd_soc_bias_level
{
	SND_SOC_BIAS_ON,
	SND_SOC_BIAS_PREPARE, 
	SND_SOC_BIAS_STANDBY, 
	SND_SOC_BIAS_OFF, 
};

struct snd_soc_dapm_route {
	const char *sink;
	const char *control;
	const char *source;
};


/* codec private data */
struct wm8904_priv {
	unsigned int sysclk;

	/* Clocking configuration */
	unsigned int mclk_rate;
	int sysclk_src;
	unsigned int sysclk_rate;

	int bclk;
	int fs;

	enum snd_soc_bias_level bias_level;
};

static unsigned int				frist_mute;
static struct snd_soc_device*	wm8904_socdev = NULL;
void wm8904_start_set(struct snd_soc_codec *codec);

static const u16 wm8904_reg[WM8904_MAX_REGISTER + 1] = {
	0x8904,     /* R0   - SW Reset and ID */
	0x0000,     /* R1   - Revision */
	0x0000,     /* R2 */
	0x0000,     /* R3 */
	0x0018,     /* R4   - Bias Control 0 */
	0x0000,     /* R5   - VMID Control 0 */
	0x0000,     /* R6   - Mic Bias Control 0 */
	0x0000,     /* R7   - Mic Bias Control 1 */
	0x0001,     /* R8   - Analogue DAC 0 */
	0x9696,     /* R9   - mic Filter Control */
	0x0001,     /* R10  - Analogue ADC 0 */
	0x0000,     /* R11 */
	0x0000,     /* R12  - Power Management 0 */
	0x0000,     /* R13 */
	0x0000,     /* R14  - Power Management 2 */
	0x0000,     /* R15  - Power Management 3 */
	0x0000,     /* R16 */
	0x0000,     /* R17 */
	0x0000,     /* R18  - Power Management 6 */
	0x0000,     /* R19 */
	0x945E,     /* R20  - Clock Rates 0 */
	0x0C05,     /* R21  - Clock Rates 1 */
	0x0006,     /* R22  - Clock Rates 2 */
	0x0000,     /* R23 */
	0x0050,     /* R24  - Audio Interface 0 */
	0x000A,     /* R25  - Audio Interface 1 */
	0x00E4,     /* R26  - Audio Interface 2 */
	0x0040,     /* R27  - Audio Interface 3 */
	0x0000,     /* R28 */
	0x0000,     /* R29 */
	0x00C0,     /* R30  - DAC Digital Volume Left */
	0x00C0,     /* R31  - DAC Digital Volume Right */
	0x0000,     /* R32  - DAC Digital 0 */
	0x0008,     /* R33  - DAC Digital 1 */
	0x0000,     /* R34 */
	0x0000,     /* R35 */
	0x00C0,     /* R36  - ADC Digital Volume Left */
	0x00C0,     /* R37  - ADC Digital Volume Right */
	0x0010,     /* R38  - ADC Digital 0 */
	0x0000,     /* R39  - Digital Microphone 0 */
	0x01AF,     /* R40  - DRC 0 */
	0x3248,     /* R41  - DRC 1 */
	0x0000,     /* R42  - DRC 2 */
	0x0000,     /* R43  - DRC 3 */
	0x0085,     /* R44  - Analogue Left Input 0 */
	0x0085,     /* R45  - Analogue Right Input 0 */
	0x0044,     /* R46  - Analogue Left Input 1 */
	0x0044,     /* R47  - Analogue Right Input 1 */
	0x0000,     /* R48 */
	0x0000,     /* R49 */
	0x0000,     /* R50 */
	0x0000,     /* R51 */
	0x0000,     /* R52 */
	0x0000,     /* R53 */
	0x0000,     /* R54 */
	0x0000,     /* R55 */
	0x0000,     /* R56 */
	0x002D,     /* R57  - Analogue OUT1 Left */
	0x002D,     /* R58  - Analogue OUT1 Right */
	0x0039,     /* R59  - Analogue OUT2 Left */
	0x0039,     /* R60  - Analogue OUT2 Right */
	0x0000,     /* R61  - Analogue OUT12 ZC */
	0x0000,     /* R62 */
	0x0000,     /* R63 */
	0x0000,     /* R64 */
	0x0000,     /* R65 */
	0x0000,     /* R66 */
	0x0000,     /* R67  - DC Servo 0 */
	0x0000,     /* R68  - DC Servo 1 */
	0xAAAA,     /* R69  - DC Servo 2 */
	0x0000,     /* R70 */
	0xAAAA,     /* R71  - DC Servo 4 */
	0xAAAA,     /* R72  - DC Servo 5 */
	0x0000,     /* R73  - DC Servo 6 */
	0x0000,     /* R74  - DC Servo 7 */
	0x0000,     /* R75  - DC Servo 8 */
	0x0000,     /* R76  - DC Servo 9 */
	0x0000,     /* R77  - DC Servo Readback 0 */
	0x0000,     /* R78 */
	0x0000,     /* R79 */
	0x0000,     /* R80 */
	0x0000,     /* R81 */
	0x0000,     /* R82 */
	0x0000,     /* R83 */
	0x0000,     /* R84 */
	0x0000,     /* R85 */
	0x0000,     /* R86 */
	0x0000,     /* R87 */
	0x0000,     /* R88 */
	0x0000,     /* R89 */
	0x0000,     /* R90  - Analogue HP 0 */
	0x0000,     /* R91 */
	0x0000,     /* R92 */
	0x0000,     /* R93 */
	0x0000,     /* R94  - Analogue Lineout 0 */
	0x0000,     /* R95 */
	0x0000,     /* R96 */
	0x0000,     /* R97 */
	0x0000,     /* R98  - Charge Pump 0 */
	0x0000,     /* R99 */
	0x0000,     /* R100 */
	0x0000,     /* R101 */
	0x0000,     /* R102 */
	0x0000,     /* R103 */
	0x0004,     /* R104 - Class W 0 */
	0x0000,     /* R105 */
	0x0000,     /* R106 */
	0x0000,     /* R107 */
	0x0000,     /* R108 - Write Sequencer 0 */
	0x0000,     /* R109 - Write Sequencer 1 */
	0x0000,     /* R110 - Write Sequencer 2 */
	0x0000,     /* R111 - Write Sequencer 3 */
	0x0000,     /* R112 - Write Sequencer 4 */
	0x0000,     /* R113 */
	0x0000,     /* R114 */
	0x0000,     /* R115 */
	0x0000,     /* R116 - FLL Control 1 */
	0x0007,     /* R117 - FLL Control 2 */
	0x0000,     /* R118 - FLL Control 3 */
	0x2EE0,     /* R119 - FLL Control 4 */
	0x0004,     /* R120 - FLL Control 5 */
	0x0014,     /* R121 - GPIO Control 1 */
	0x0010,     /* R122 - GPIO Control 2 */
	0x0010,     /* R123 - GPIO Control 3 */
	0x0000,     /* R124 - GPIO Control 4 */
	0x0000,     /* R125 */
	0x0000,     /* R126 - Digital Pulls */
	0x0000,     /* R127 - Interrupt Status */
	0xFFFF,     /* R128 - Interrupt Status Mask */
	0x0000,     /* R129 - Interrupt Polarity */
	0x0000,     /* R130 - Interrupt Debounce */
	0x0000,     /* R131 */
	0x0000,     /* R132 */
	0x0000,     /* R133 */
	0x0000,     /* R134 - EQ1 */
	0x000C,     /* R135 - EQ2 */
	0x000C,     /* R136 - EQ3 */
	0x000C,     /* R137 - EQ4 */
	0x000C,     /* R138 - EQ5 */
	0x000C,     /* R139 - EQ6 */
	0x0FCA,     /* R140 - EQ7 */
	0x0400,     /* R141 - EQ8 */
	0x00D8,     /* R142 - EQ9 */
	0x1EB5,     /* R143 - EQ10 */
	0xF145,     /* R144 - EQ11 */
	0x0B75,     /* R145 - EQ12 */
	0x01C5,     /* R146 - EQ13 */
	0x1C58,     /* R147 - EQ14 */
	0xF373,     /* R148 - EQ15 */
	0x0A54,     /* R149 - EQ16 */
	0x0558,     /* R150 - EQ17 */
	0x168E,     /* R151 - EQ18 */
	0xF829,     /* R152 - EQ19 */
	0x07AD,     /* R153 - EQ20 */
	0x1103,     /* R154 - EQ21 */
	0x0564,     /* R155 - EQ22 */
	0x0559,     /* R156 - EQ23 */
	0x4000,     /* R157 - EQ24 */
	0x0000,     /* R158 */
	0x0000,     /* R159 */
	0x0000,     /* R160 */
	0x0000,     /* R161 - Control Interface Test 1 */
	0x0000,     /* R162 */
	0x0000,     /* R163 */
	0x0000,     /* R164 */
	0x0000,     /* R165 */
	0x0000,     /* R166 */
	0x0000,     /* R167 */
	0x0000,     /* R168 */
	0x0000,     /* R169 */
	0x0000,     /* R170 */
	0x0000,     /* R171 */
	0x0000,     /* R172 */
	0x0000,     /* R173 */
	0x0000,     /* R174 */
	0x0000,     /* R175 */
	0x0000,     /* R176 */
	0x0000,     /* R177 */
	0x0000,     /* R178 */
	0x0000,     /* R179 */
	0x0000,     /* R180 */
	0x0000,     /* R181 */
	0x0000,     /* R182 */
	0x0000,     /* R183 */
	0x0000,     /* R184 */
	0x0000,     /* R185 */
	0x0000,     /* R186 */
	0x0000,     /* R187 */
	0x0000,     /* R188 */
	0x0000,     /* R189 */
	0x0000,     /* R190 */
	0x0000,     /* R191 */
	0x0000,     /* R192 */
	0x0000,     /* R193 */
	0x0000,     /* R194 */
	0x0000,     /* R195 */
	0x0000,     /* R196 */
	0x0000,     /* R197 */
	0x0000,     /* R198 */
	0x0000,     /* R199 */
	0x0000,     /* R200 */
	0x0000,     /* R201 */
	0x0000,     /* R202 */
	0x0000,     /* R203 */
	0x0000,     /* R204 - Analogue Output Bias 0 */
	0x0000,     /* R205 */
	0x0000,     /* R206 */
	0x0000,     /* R207 */
	0x0000,     /* R208 */
	0x0000,     /* R209 */
	0x0000,     /* R210 */
	0x0000,     /* R211 */
	0x0000,     /* R212 */
	0x0000,     /* R213 */
	0x0000,     /* R214 */
	0x0000,     /* R215 */
	0x0000,     /* R216 */
	0x0000,     /* R217 */
	0x0000,     /* R218 */
	0x0000,     /* R219 */
	0x0000,     /* R220 */
	0x0000,     /* R221 */
	0x0000,     /* R222 */
	0x0000,     /* R223 */
	0x0000,     /* R224 */
	0x0000,     /* R225 */
	0x0000,     /* R226 */
	0x0000,     /* R227 */
	0x0000,     /* R228 */
	0x0000,     /* R229 */
	0x0000,     /* R230 */
	0x0000,     /* R231 */
	0x0000,     /* R232 */
	0x0000,     /* R233 */
	0x0000,     /* R234 */
	0x0000,     /* R235 */
	0x0000,     /* R236 */
	0x0000,     /* R237 */
	0x0000,     /* R238 */
	0x0000,     /* R239 */
	0x0000,     /* R240 */
	0x0000,     /* R241 */
	0x0000,     /* R242 */
	0x0000,     /* R243 */
	0x0000,     /* R244 */
	0x0000,     /* R245 */
	0x0000,     /* R246 */
	0x0000,     /* R247 - FLL NCO Test 0 */
	0x0019,     /* R248 - FLL NCO Test 1 */
};

static int wm8904_read_reg_cache(struct snd_soc_codec *codec, u32 reg)
{
	u16 *cache = codec->reg_cache;
	
	if (reg > WM8904_MAX_REGISTER) {
		printk_e("Invalid reg number: %d\n", reg);
		return -1;
    }
    
	return cache[reg];
}


static void wm8904_write_reg_cache(struct snd_soc_codec *codec, u32 reg, u32 value)
{
	u16 *cache = codec->reg_cache;
	
	if (reg == 0)
		return;

	if (reg > WM8904_MAX_REGISTER) {
		printk_e("Invalid reg number: %d\n", reg);
		return;
	}

	cache[reg] = value;
}


static int wm8904_write(struct snd_soc_codec *codec, u32 reg, u32 value)
{
	u8 data[4];

	printk_d("WM8904 write: reg=0x%x value=0x%x\n", reg, value);

	data[0] = reg;
	data[1] = (value >> 8) & 0x00ff;
	data[2] = value & 0x00ff;
	data[3] = 0; 

	//i2c_master_send(2818) more one byte
	if (codec->hw_write(codec->control_data, data, 4) != 4)	{
		printk_e("error WM8904 write: reg=0x%x value=0x%x\n", reg, value);
		return -EIO;
	}
	//wm8904_write_reg_cache (codec, reg, value);

	return 0;
}


static int wm8904_read (struct snd_soc_codec *codec, u32 reg)
{
	u8	data[3];
	int value;
  
	printk_d("wm8904_read reg=0x%x \n", reg);

	data[0] = reg;
	data[1] = 0;

	//write reg address  (i2c_master_send(2818) more one byte)
	if (codec->hw_write(codec->control_data, data, 2) != 2)	{
		printk_e("Error reg=0x%x \n", reg);
		return -EIO;
	}

	if (codec->hw_read(codec->control_data, data, 2) != 2) {
		printk_e("Error reg=0x%x \n", reg);
		return -EIO;
	}

	value = ((u16)data[0] << 8 | (u16)data[1]);

	return value;
}


static int wm8904_reset(struct snd_soc_codec *codec)
{
	return wm8904_write(codec, WM8904_SW_RESET_AND_ID, 0);
}


static int wm8904_update_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value)
{
	int change;
	unsigned short old_value, new_value;

//	if (reg == 0x4 || reg == 0x05)
//		return 0;
	return 0;
//	mutex_lock(&io_mutex);
	old_value = wm8904_read_reg_cache(codec, reg);
	new_value = (old_value & ~mask) | value;
	change = old_value != new_value;
//	if (change)
//		wm8904_write(codec, reg, new);

//	mutex_unlock(&io_mutex);
	return change;
}




static int wm8904_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct wm8904_priv *wm8904 = codec->private_data;
	//int ret;

	switch (level) 
	{
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		/* VMID resistance 2*50k */
		wm8904_update_bits(codec, WM8904_VMID_CONTROL_0,
				    WM8904_VMID_RES_MASK,
				    0x1 << WM8904_VMID_RES_SHIFT);

		/* Normal bias current */
		wm8904_update_bits(codec, WM8904_BIAS_CONTROL_0,
				    WM8904_ISEL_MASK, 2 << WM8904_ISEL_SHIFT);
		break;

	case SND_SOC_BIAS_STANDBY:
		printk_d(" SND_SOC_BIAS_STANDBY \n");

		wm8904_write(codec, 0x05, 0x0043); //   0x05 0x0043 SMbus_16_bit_data     Write  0x34      * 
												//   0x12 0x000C SMbus_16_bit_data     Write  0x34      * 
		wm8904_write(codec, 0x0E, 0x0003); //   0x0E 0x0003 SMbus_16_bit_data     Write  0x34      * 
		mdelay(10); //insert_delay_ms 10
		wm8904_write(codec, 0x5A, 0x0011); //  0x5A 0x0011 SMbus_16_bit_data     Write  0x34      * 
		wm8904_write(codec, 0x5A, 0x0033); //  0x5A 0x0033 SMbus_16_bit_data     Write  0x34      * 
		wm8904_write(codec, 0x5A, 0x0077); //  0x5A 0x0077 SMbus_16_bit_data     Write  0x34      * 
 		wm8904_write(codec, 0x5A, 0x00FF); //  0x5A 0x00FF SMbus_16_bit_data     Write  0x34      * 
#if 0
		if (wm8904->bias_level == SND_SOC_BIAS_OFF) {

			/* Enable bias */
			wm8904_update_bits(codec, WM8904_BIAS_CONTROL_0,
					    WM8904_BIAS_ENA, WM8904_BIAS_ENA);

			/* Enable VMID, VMID buffering, 2*5k resistance */
			wm8904_update_bits(codec, WM8904_VMID_CONTROL_0,
					    WM8904_VMID_ENA |
					    WM8904_VMID_RES_MASK,
					    WM8904_VMID_ENA |
					    0x3 << WM8904_VMID_RES_SHIFT);

			/* Let VMID ramp */
			msleep(1);
		}

		/* Maintain VMID with 2*250k */
		wm8904_update_bits(codec, WM8904_VMID_CONTROL_0,
				    WM8904_VMID_RES_MASK,
				    0x2 << WM8904_VMID_RES_SHIFT);

		/* Bias current *0.5 */
		wm8904_update_bits(codec, WM8904_BIAS_CONTROL_0,
				    WM8904_ISEL_MASK, 0);
#endif
		break;

	case SND_SOC_BIAS_OFF:
		printk_d(" SND_SOC_BIAS_OFF \n");
		wm8904_write(codec, 0x5A, 0x0077); //0x5A 0x0077 SMbus_16_bit_data     Write  0x34      * 
		wm8904_write(codec, 0x5A, 0x0033); //   0x5A 0x0033 SMbus_16_bit_data     Write  0x34      * 
		wm8904_write(codec, 0x5A, 0x0011); //   0x5A 0x0011 SMbus_16_bit_data     Write  0x34      * 
		wm8904_write(codec, 0x5A, 0x0000); //   0x5A 0x0000 SMbus_16_bit_data     Write  0x34      * 
		mdelay(10); //insert_delay_ms 10
		wm8904_write(codec, 0x0E, 0x0000); //   0x0E 0x0000 SMbus_16_bit_data     Write  0x34      * 
		//   0x12 0x0000 SMbus_16_bit_data     Write  0x34      * 
		wm8904_write(codec, 0x05, 0x0000); //   0x05 0x0045 SMbus_16_bit_data     Write  0x34      * 

#if 0
		/* Turn off VMID */
		wm8904_update_bits(codec, WM8904_VMID_CONTROL_0,
				    WM8904_VMID_RES_MASK | WM8904_VMID_ENA, 0);

		/* Stop bias generation */
		wm8904_update_bits(codec, WM8904_BIAS_CONTROL_0,
				    WM8904_BIAS_ENA, 0);
#endif
		break;
	}

	wm8904->bias_level = level;
	return 0;
}

#if 0
static const DECLARE_TLV_DB_SCALE(dac_boost_tlv, 0, 600, 0);
static const DECLARE_TLV_DB_SCALE(digital_tlv, -7200, 75, 1);
static const DECLARE_TLV_DB_SCALE(out_tlv, -5700, 100, 0);
static const DECLARE_TLV_DB_SCALE(sidetone_tlv, -3600, 300, 0);
static const DECLARE_TLV_DB_SCALE(eq_tlv, -1200, 100, 0);

static const char *input_mode_text[] = {
	"Single-Ended", "Differential Line", "Differential Mic"
};

static const struct soc_enum lin_mode =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_LEFT_INPUT_1, 0, 3, input_mode_text);

static const struct soc_enum rin_mode =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_RIGHT_INPUT_1, 0, 3, input_mode_text);

static const char *hpf_mode_text[] = {
	"Hi-fi", "Voice 1", "Voice 2", "Voice 3"
};

static const struct soc_enum hpf_mode =
	SOC_ENUM_SINGLE(WM8904_ADC_DIGITAL_0, 5, 4, hpf_mode_text);

static const struct snd_kcontrol_new wm8904_adc_snd_controls[] = {
SOC_DOUBLE_R_TLV("Digital Capture Volume", WM8904_ADC_DIGITAL_VOLUME_LEFT,
		 WM8904_ADC_DIGITAL_VOLUME_RIGHT, 1, 119, 0, digital_tlv),

SOC_ENUM("Left Caputure Mode", lin_mode),
SOC_ENUM("Right Capture Mode", rin_mode),

/* No TLV since it depends on mode */
SOC_DOUBLE_R("Capture Volume", WM8904_ANALOGUE_LEFT_INPUT_0,
	     WM8904_ANALOGUE_RIGHT_INPUT_0, 0, 31, 0),
SOC_DOUBLE_R("Capture Switch", WM8904_ANALOGUE_LEFT_INPUT_0,
	     WM8904_ANALOGUE_RIGHT_INPUT_0, 7, 1, 1),

SOC_SINGLE("High Pass Filter Switch", WM8904_ADC_DIGITAL_0, 4, 1, 0),
SOC_ENUM("High Pass Filter Mode", hpf_mode),

SOC_SINGLE("ADC 128x OSR Switch", WM8904_ANALOGUE_ADC_0, 0, 1, 0),
};

static const char *drc_path_text[] = {
	"ADC", "DAC"
};

static const struct soc_enum drc_path =
	SOC_ENUM_SINGLE(WM8904_DRC_0, 14, 2, drc_path_text);

static const struct snd_kcontrol_new wm8904_dac_snd_controls[] = {
SOC_SINGLE_TLV("Digital Playback Boost Volume", 
	       WM8904_AUDIO_INTERFACE_0, 9, 3, 0, dac_boost_tlv),
SOC_DOUBLE_R_TLV("Digital Playback Volume", WM8904_DAC_DIGITAL_VOLUME_LEFT,
		 WM8904_DAC_DIGITAL_VOLUME_RIGHT, 1, 96, 0, digital_tlv),

SOC_DOUBLE_R_TLV("Headphone Volume", WM8904_ANALOGUE_OUT1_LEFT,
		 WM8904_ANALOGUE_OUT1_RIGHT, 0, 63, 0, out_tlv),
SOC_DOUBLE_R("Headphone Switch", WM8904_ANALOGUE_OUT1_LEFT,
	     WM8904_ANALOGUE_OUT1_RIGHT, 8, 1, 1),
SOC_DOUBLE_R("Headphone ZC Switch", WM8904_ANALOGUE_OUT1_LEFT,
	     WM8904_ANALOGUE_OUT1_RIGHT, 6, 1, 0),

SOC_DOUBLE_R_TLV("Line Output Volume", WM8904_ANALOGUE_OUT2_LEFT,
		 WM8904_ANALOGUE_OUT2_RIGHT, 0, 63, 0, out_tlv),
SOC_DOUBLE_R("Line Output Switch", WM8904_ANALOGUE_OUT2_LEFT,
	     WM8904_ANALOGUE_OUT2_RIGHT, 8, 1, 1),
SOC_DOUBLE_R("Line Output ZC Switch", WM8904_ANALOGUE_OUT2_LEFT,
	     WM8904_ANALOGUE_OUT2_RIGHT, 6, 1, 0),

SOC_SINGLE("EQ Switch", WM8904_EQ1, 0, 1, 0),
SOC_SINGLE("DRC Switch", WM8904_DRC_0, 15, 1, 0),
SOC_ENUM("DRC Path", drc_path),
SOC_SINGLE("DAC OSRx2 Switch", WM8904_DAC_DIGITAL_1, 6, 1, 0),
//SOC_SINGLE_BOOL_EXT("DAC Deemphasis Switch", 0,
//		    wm8904_get_deemph, wm8904_put_deemph),
};

static const struct snd_kcontrol_new wm8904_snd_controls[] = {
SOC_DOUBLE_TLV("Digital Sidetone Volume", WM8904_DAC_DIGITAL_0, 4, 8, 15, 0,
	       sidetone_tlv),
};

static const struct snd_kcontrol_new wm8904_eq_controls[] = {
SOC_SINGLE_TLV("EQ1 Volume", WM8904_EQ2, 0, 24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ2 Volume", WM8904_EQ3, 0, 24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ3 Volume", WM8904_EQ4, 0, 24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ4 Volume", WM8904_EQ5, 0, 24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ5 Volume", WM8904_EQ6, 0, 24, 0, eq_tlv),
};


static int wm8904_add_controls_in(struct snd_soc_codec *codec, const struct snd_kcontrol_new* controls,
				     int count)
{
	int i;
	int err;

	for (i = 0; i < count; i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}


static int wm8904_add_controls(struct snd_soc_codec *codec)
{
	
    printk_d("Enter\n");

	wm8904_add_controls_in(codec, wm8904_adc_snd_controls,
		ARRAY_SIZE(wm8904_adc_snd_controls));
	wm8904_add_controls_in(codec, wm8904_dac_snd_controls,
		ARRAY_SIZE(wm8904_dac_snd_controls));
	wm8904_add_controls_in(codec, wm8904_snd_controls,
		ARRAY_SIZE(wm8904_snd_controls));
	wm8904_add_controls_in(codec, wm8904_eq_controls,
		ARRAY_SIZE(wm8904_eq_controls));

	return 0;
}



static const char *lin_text[] = {
	"IN1L", "IN2L", "IN3L"
};

static const struct soc_enum lin_enum =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_LEFT_INPUT_1, 2, 3, lin_text);

static const struct snd_kcontrol_new lin_mux =
	SOC_DAPM_ENUM("Left Capture Mux", lin_enum);

static const struct soc_enum lin_inv_enum =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_LEFT_INPUT_1, 4, 3, lin_text);

static const struct snd_kcontrol_new lin_inv_mux =
	SOC_DAPM_ENUM("Left Capture Inveting Mux", lin_inv_enum);

static const char *rin_text[] = {
	"IN1R", "IN2R", "IN3R"
};

static const struct soc_enum rin_enum =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_RIGHT_INPUT_1, 2, 3, rin_text);

static const struct snd_kcontrol_new rin_mux =
	SOC_DAPM_ENUM("Right Capture Mux", rin_enum);

static const struct soc_enum rin_inv_enum =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_RIGHT_INPUT_1, 4, 3, rin_text);

static const struct snd_kcontrol_new rin_inv_mux =
	SOC_DAPM_ENUM("Right Capture Inveting Mux", rin_inv_enum);

static const char *aif_text[] = {
	"Left", "Right"
};

static const struct soc_enum aifoutl_enum =
	SOC_ENUM_SINGLE(WM8904_AUDIO_INTERFACE_0, 7, 2, aif_text);

static const struct snd_kcontrol_new aifoutl_mux =
	SOC_DAPM_ENUM("AIFOUTL Mux", aifoutl_enum);

static const struct soc_enum aifoutr_enum =
	SOC_ENUM_SINGLE(WM8904_AUDIO_INTERFACE_0, 6, 2, aif_text);

static const struct snd_kcontrol_new aifoutr_mux =
	SOC_DAPM_ENUM("AIFOUTR Mux", aifoutr_enum);

static const struct soc_enum aifinl_enum =
	SOC_ENUM_SINGLE(WM8904_AUDIO_INTERFACE_0, 5, 2, aif_text);

static const struct snd_kcontrol_new aifinl_mux =
	SOC_DAPM_ENUM("AIFINL Mux", aifinl_enum);

static const struct soc_enum aifinr_enum =
	SOC_ENUM_SINGLE(WM8904_AUDIO_INTERFACE_0, 4, 2, aif_text);

static const struct snd_kcontrol_new aifinr_mux =
	SOC_DAPM_ENUM("AIFINR Mux", aifinr_enum);

static const struct snd_soc_dapm_widget wm8904_core_dapm_widgets[] = {
//SND_SOC_DAPM_SUPPLY("SYSCLK", WM8904_CLOCK_RATES_2, 2, 0, sysclk_event,
//		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
//SND_SOC_DAPM_SUPPLY("CLK_DSP", WM8904_CLOCK_RATES_2, 1, 0, NULL, 0),
//SND_SOC_DAPM_SUPPLY("TOCLK", WM8904_CLOCK_RATES_2, 0, 0, NULL, 0),
};

static const struct snd_soc_dapm_widget wm8904_adc_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("IN1L"),
SND_SOC_DAPM_INPUT("IN1R"),
SND_SOC_DAPM_INPUT("IN2L"),
SND_SOC_DAPM_INPUT("IN2R"),
SND_SOC_DAPM_INPUT("IN3L"),
SND_SOC_DAPM_INPUT("IN3R"),

SND_SOC_DAPM_MICBIAS("MICBIAS", WM8904_MIC_BIAS_CONTROL_0, 0, 0),

SND_SOC_DAPM_MUX("Left Capture Mux", SND_SOC_NOPM, 0, 0, &lin_mux),
SND_SOC_DAPM_MUX("Left Capture Inverting Mux", SND_SOC_NOPM, 0, 0,
		 &lin_inv_mux),
SND_SOC_DAPM_MUX("Right Capture Mux", SND_SOC_NOPM, 0, 0, &rin_mux),
SND_SOC_DAPM_MUX("Right Capture Inverting Mux", SND_SOC_NOPM, 0, 0,
		 &rin_inv_mux),

SND_SOC_DAPM_PGA("Left Capture PGA", WM8904_POWER_MANAGEMENT_0, 1, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("Right Capture PGA", WM8904_POWER_MANAGEMENT_0, 0, 0,
		 NULL, 0),

SND_SOC_DAPM_ADC("ADCL", NULL, WM8904_POWER_MANAGEMENT_6, 1, 0),
SND_SOC_DAPM_ADC("ADCR", NULL, WM8904_POWER_MANAGEMENT_6, 0, 0),

SND_SOC_DAPM_MUX("AIFOUTL Mux", SND_SOC_NOPM, 0, 0, &aifoutl_mux),
SND_SOC_DAPM_MUX("AIFOUTR Mux", SND_SOC_NOPM, 0, 0, &aifoutr_mux),

//SND_SOC_DAPM_AIF_OUT("AIFOUTL", "Capture", 0, SND_SOC_NOPM, 0, 0),
//SND_SOC_DAPM_AIF_OUT("AIFOUTR", "Capture", 1, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_widget wm8904_dac_dapm_widgets[] = {
//SND_SOC_DAPM_AIF_IN("AIFINL", "Playback", 0, SND_SOC_NOPM, 0, 0),
//SND_SOC_DAPM_AIF_IN("AIFINR", "Playback", 1, SND_SOC_NOPM, 0, 0),

SND_SOC_DAPM_MUX("DACL Mux", SND_SOC_NOPM, 0, 0, &aifinl_mux),
SND_SOC_DAPM_MUX("DACR Mux", SND_SOC_NOPM, 0, 0, &aifinr_mux),

SND_SOC_DAPM_DAC("DACL", NULL, WM8904_POWER_MANAGEMENT_6, 3, 0),
SND_SOC_DAPM_DAC("DACR", NULL, WM8904_POWER_MANAGEMENT_6, 2, 0),

//SND_SOC_DAPM_SUPPLY("Charge pump", WM8904_CHARGE_PUMP_0, 0, 0, cp_event,
//		    SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_PGA("HPL PGA", SND_SOC_NOPM, 1, 0, NULL, 0),
SND_SOC_DAPM_PGA("HPR PGA", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_PGA("LINEL PGA", SND_SOC_NOPM, 1, 0, NULL, 0),
SND_SOC_DAPM_PGA("LINER PGA", SND_SOC_NOPM, 0, 0, NULL, 0),

//SND_SOC_DAPM_PGA_E("Headphone Output", SND_SOC_NOPM, WM8904_ANALOGUE_HP_0,
//		   0, NULL, 0, out_pga_event,
//		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
//		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
//SND_SOC_DAPM_PGA_E("Line Output", SND_SOC_NOPM, WM8904_ANALOGUE_LINEOUT_0,
//		   0, NULL, 0, out_pga_event,
//		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
//		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

SND_SOC_DAPM_OUTPUT("HPOUTL"),
SND_SOC_DAPM_OUTPUT("HPOUTR"),
SND_SOC_DAPM_OUTPUT("LINEOUTL"),
SND_SOC_DAPM_OUTPUT("LINEOUTR"),
};

static const char *out_mux_text[] = {
	"DAC", "Bypass"
};

static const struct soc_enum hpl_enum =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_OUT12_ZC, 3, 2, out_mux_text);

static const struct snd_kcontrol_new hpl_mux =
	SOC_DAPM_ENUM("HPL Mux", hpl_enum);

static const struct soc_enum hpr_enum =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_OUT12_ZC, 2, 2, out_mux_text);

static const struct snd_kcontrol_new hpr_mux =
	SOC_DAPM_ENUM("HPR Mux", hpr_enum);

static const struct soc_enum linel_enum =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_OUT12_ZC, 1, 2, out_mux_text);

static const struct snd_kcontrol_new linel_mux =
	SOC_DAPM_ENUM("LINEL Mux", linel_enum);

static const struct soc_enum liner_enum =
	SOC_ENUM_SINGLE(WM8904_ANALOGUE_OUT12_ZC, 0, 2, out_mux_text);

static const struct snd_kcontrol_new liner_mux =
	SOC_DAPM_ENUM("LINEL Mux", liner_enum);

static const char *sidetone_text[] = {
	"None", "Left", "Right"
};

static const struct soc_enum dacl_sidetone_enum =
	SOC_ENUM_SINGLE(WM8904_DAC_DIGITAL_0, 2, 3, sidetone_text);

static const struct snd_kcontrol_new dacl_sidetone_mux =
	SOC_DAPM_ENUM("Left Sidetone Mux", dacl_sidetone_enum);

static const struct soc_enum dacr_sidetone_enum =
	SOC_ENUM_SINGLE(WM8904_DAC_DIGITAL_0, 0, 3, sidetone_text);

static const struct snd_kcontrol_new dacr_sidetone_mux =
	SOC_DAPM_ENUM("Right Sidetone Mux", dacr_sidetone_enum);

static const struct snd_soc_dapm_widget wm8904_dapm_widgets[] = {
//SND_SOC_DAPM_SUPPLY("Class G", WM8904_CLASS_W_0, 0, 1, NULL, 0),
SND_SOC_DAPM_PGA("Left Bypass", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Bypass", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_MUX("Left Sidetone", SND_SOC_NOPM, 0, 0, &dacl_sidetone_mux),
SND_SOC_DAPM_MUX("Right Sidetone", SND_SOC_NOPM, 0, 0, &dacr_sidetone_mux),

SND_SOC_DAPM_MUX("HPL Mux", SND_SOC_NOPM, 0, 0, &hpl_mux),
SND_SOC_DAPM_MUX("HPR Mux", SND_SOC_NOPM, 0, 0, &hpr_mux),
SND_SOC_DAPM_MUX("LINEL Mux", SND_SOC_NOPM, 0, 0, &linel_mux),
SND_SOC_DAPM_MUX("LINER Mux", SND_SOC_NOPM, 0, 0, &liner_mux),
};

static const struct snd_soc_dapm_route core_intercon[] = {
	{ "CLK_DSP", NULL, "SYSCLK" },
	{ "TOCLK", NULL, "SYSCLK" },
};

static const struct snd_soc_dapm_route adc_intercon[] = {
	{ "Left Capture Mux", "IN1L", "IN1L" },
	{ "Left Capture Mux", "IN2L", "IN2L" },
	{ "Left Capture Mux", "IN3L", "IN3L" },

	{ "Left Capture Inverting Mux", "IN1L", "IN1L" },
	{ "Left Capture Inverting Mux", "IN2L", "IN2L" },
	{ "Left Capture Inverting Mux", "IN3L", "IN3L" },

	{ "Right Capture Mux", "IN1R", "IN1R" },
	{ "Right Capture Mux", "IN2R", "IN2R" },
	{ "Right Capture Mux", "IN3R", "IN3R" },

	{ "Right Capture Inverting Mux", "IN1R", "IN1R" },
	{ "Right Capture Inverting Mux", "IN2R", "IN2R" },
	{ "Right Capture Inverting Mux", "IN3R", "IN3R" },

	{ "Left Capture PGA", NULL, "Left Capture Mux" },
	{ "Left Capture PGA", NULL, "Left Capture Inverting Mux" },

	{ "Right Capture PGA", NULL, "Right Capture Mux" },
	{ "Right Capture PGA", NULL, "Right Capture Inverting Mux" },

	{ "AIFOUTL", "Left",  "ADCL" },
	{ "AIFOUTL", "Right", "ADCR" },
	{ "AIFOUTR", "Left",  "ADCL" },
	{ "AIFOUTR", "Right", "ADCR" },

	{ "ADCL", NULL, "CLK_DSP" },
	{ "ADCL", NULL, "Left Capture PGA" },

	{ "ADCR", NULL, "CLK_DSP" },
	{ "ADCR", NULL, "Right Capture PGA" },
};

static const struct snd_soc_dapm_route dac_intercon[] = {
	{ "DACL", "Right", "AIFINR" },
	{ "DACL", "Left",  "AIFINL" },
	{ "DACL", NULL, "CLK_DSP" },

	{ "DACR", "Right", "AIFINR" },
	{ "DACR", "Left",  "AIFINL" },
	{ "DACR", NULL, "CLK_DSP" },

	{ "Charge pump", NULL, "SYSCLK" },

	{ "Headphone Output", NULL, "HPL PGA" },
	{ "Headphone Output", NULL, "HPR PGA" },
	{ "Headphone Output", NULL, "Charge pump" },
	{ "Headphone Output", NULL, "TOCLK" },

	{ "Line Output", NULL, "LINEL PGA" },
	{ "Line Output", NULL, "LINER PGA" },
	{ "Line Output", NULL, "Charge pump" },
	{ "Line Output", NULL, "TOCLK" },

	{ "HPOUTL", NULL, "Headphone Output" },
	{ "HPOUTR", NULL, "Headphone Output" },

	{ "LINEOUTL", NULL, "Line Output" },
	{ "LINEOUTR", NULL, "Line Output" },
};

static const struct snd_soc_dapm_route wm8904_intercon[] = {
	{ "Left Sidetone", "Left", "ADCL" },
	{ "Left Sidetone", "Right", "ADCR" },
	{ "DACL", NULL, "Left Sidetone" },
	
	{ "Right Sidetone", "Left", "ADCL" },
	{ "Right Sidetone", "Right", "ADCR" },
	{ "DACR", NULL, "Right Sidetone" },

	{ "Left Bypass", NULL, "Class G" },
	{ "Left Bypass", NULL, "Left Capture PGA" },

	{ "Right Bypass", NULL, "Class G" },
	{ "Right Bypass", NULL, "Right Capture PGA" },

	{ "HPL Mux", "DAC", "DACL" },
	{ "HPL Mux", "Bypass", "Left Bypass" },

	{ "HPR Mux", "DAC", "DACR" },
	{ "HPR Mux", "Bypass", "Right Bypass" },

	{ "LINEL Mux", "DAC", "DACL" },
	{ "LINEL Mux", "Bypass", "Left Bypass" },

	{ "LINER Mux", "DAC", "DACR" },
	{ "LINER Mux", "Bypass", "Right Bypass" },

	{ "HPL PGA", NULL, "HPL Mux" },
	{ "HPR PGA", NULL, "HPR Mux" },

	{ "LINEL PGA", NULL, "LINEL Mux" },
	{ "LINER PGA", NULL, "LINER Mux" },
};


static int wm8904_add_widgets_in(struct snd_soc_codec *codec, const struct snd_soc_dapm_widget* widgets, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		snd_soc_dapm_new_control(codec, &widgets[i]);
	}

	return 0;
}


static int wm8904_add_dapm_route(struct snd_soc_codec *codec, const struct snd_soc_dapm_route* route, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		snd_soc_dapm_connect_input(codec, route[i].sink, route[i].control, route[i].source);

	}

	return 0;
}



static int wm8904_add_widgets(struct snd_soc_codec *codec)
{
	int i;
	
    printk_d("Enter\n");

	wm8904_add_widgets_in(codec, wm8904_core_dapm_widgets, ARRAY_SIZE(wm8904_core_dapm_widgets));
	wm8904_add_widgets_in(codec, wm8904_adc_dapm_widgets, ARRAY_SIZE(wm8904_adc_dapm_widgets));
	wm8904_add_widgets_in(codec, wm8904_dac_dapm_widgets, ARRAY_SIZE(wm8904_dac_dapm_widgets));
	wm8904_add_widgets_in(codec, wm8904_dapm_widgets, ARRAY_SIZE(wm8904_dapm_widgets));


	wm8904_add_dapm_route(codec, core_intercon, ARRAY_SIZE(core_intercon));
	wm8904_add_dapm_route(codec, adc_intercon, ARRAY_SIZE(adc_intercon));
	wm8904_add_dapm_route(codec, dac_intercon, ARRAY_SIZE(dac_intercon));
	wm8904_add_dapm_route(codec, wm8904_intercon, ARRAY_SIZE(wm8904_intercon));
    
	snd_soc_dapm_new_widgets(codec);
	return 0;
}

#endif


static int wm8904_set_dai_sysclk(struct snd_soc_codec_dai *codec_dai,
		int clk_id, u32 freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct wm8904_priv *wm8904 = codec->private_data;
	
	printk_d("clk_id = %d freq = %u\n", clk_id, freq);

	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
//	case 16934400:
//	case 18432000:
		wm8904->sysclk_src = WM8904_CLK_MCLK;
		wm8904->mclk_rate  = freq;
		wm8904->sysclk     = freq;
		return 0;
	}
	
	return -EINVAL;
}

static int wm8904_set_dai_fmt(struct snd_soc_codec_dai *codec_dai,
		unsigned int fmt)
{
	u16 iface = 0;
	struct snd_soc_codec *codec = codec_dai->codec;
	
	printk_d("Enter::%s----%d fmt=0x%x\n", __FUNCTION__,__LINE__, fmt);
	unsigned int aif1 = 0;
	unsigned int aif3 = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		aif3 |= WM8904_LRCLK_DIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		aif1 |= WM8904_BCLK_DIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		aif1 |= WM8904_BCLK_DIR;
		aif3 |= WM8904_LRCLK_DIR;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_B:
		aif1 |= WM8904_AIF_LRCLK_INV;
	case SND_SOC_DAIFMT_DSP_A:
		aif1 |= 0x3;
		break;
	case SND_SOC_DAIFMT_I2S:
		aif1 |= 0x2;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		aif1 |= 0x1;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		/* frame inversion not valid for DSP modes */
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			break;
		case SND_SOC_DAIFMT_IB_NF:
			aif1 |= WM8904_AIF_BCLK_INV;
			break;
		default:
			return -EINVAL;
		}
		break;

	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			break;
		case SND_SOC_DAIFMT_IB_IF:
			aif1 |= WM8904_AIF_BCLK_INV | WM8904_AIF_LRCLK_INV;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			aif1 |= WM8904_AIF_BCLK_INV;
			break;
		case SND_SOC_DAIFMT_NB_IF:
			aif1 |= WM8904_AIF_LRCLK_INV;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	wm8904_update_bits(codec, WM8904_AUDIO_INTERFACE_1,
			    WM8904_AIF_BCLK_INV | WM8904_AIF_LRCLK_INV |
			    WM8904_AIF_FMT_MASK | WM8904_BCLK_DIR, aif1);
	wm8904_update_bits(codec, WM8904_AUDIO_INTERFACE_3,
			    WM8904_LRCLK_DIR, aif3);

	return 0;
}


static int wm8904_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct wm8904_priv *wm8904 = codec->private_data;
	unsigned int aif1 = 0;
	//unsigned int aif2 = 0;
	//unsigned int aif3 = 0;
	//unsigned int clock1 = 0;
	//unsigned int dac_digital1 = 0;

	printk_d("Enter\n");

	wm8904->fs = params_rate(params);
	printk_d("set new sample rate: %d\n", wm8904->fs);

 	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		aif1 |= 0x40;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		aif1 |= 0x80;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		aif1 |= 0xc0;
		break;
	default:
		return -EINVAL;
	}

	/* Apply the settings */
	wm8904_update_bits(codec, WM8904_AUDIO_INTERFACE_1,
			    WM8904_AIF_WL_MASK, aif1);

	return 0;
}


static int wm8904_mute(struct snd_soc_codec_dai *dai, int mute)
{
	struct snd_soc_codec *codec = wm8904_socdev->codec;
	int val;

	printk_d("Enter mute = %d\n", mute);

	if (mute)
		val = WM8904_DAC_MUTE;
	else
		val = 0;

	if (frist_mute == 1) 
	{
//		wm8904_write(codec, WM8988_PWR2, (WM_DACL|WM_DACR|WM_LOUT1|WM_ROUT1));
//		mdelay(10);
		frist_mute = 0;
	}

	wm8904_update_bits(codec, WM8904_DAC_DIGITAL_1, WM8904_DAC_MUTE, val);
	return 0;
}

static int wm8904_trigger(struct snd_pcm_substream *substream, int trigger)
{
	struct snd_soc_device *socdev = wm8904_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	
	printk_d("Enter::%s----%d trigger = %d\n",__FUNCTION__,__LINE__, trigger);

	if(trigger)
		wm8904_start_set(codec);

	return 0;
}


static int wm8904_dapm_event(struct snd_soc_codec *codec, int event)
{
    printk_d("Enter event = %d\n", event);
    
	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		wm8904_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
		break;

	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		wm8904_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
		break;

	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		wm8904_set_bias_level(codec, SND_SOC_BIAS_OFF);
		break;

	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		wm8904_set_bias_level(codec, SND_SOC_BIAS_OFF);
		break;
	}
	codec->dapm_state = event;
	return 0;
}




#define WM8904_RATES SNDRV_PCM_RATE_8000_96000

#define WM8904_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_codec_dai wm8904_dai = {
	.name = "WM8904",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8904_RATES,
		.formats = WM8904_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8904_RATES,
		.formats = WM8904_FORMATS,},
	.ops = {
		.hw_params = wm8904_pcm_hw_params,
		.trigger = wm8904_trigger,
	},
	.dai_ops = {
		.digital_mute = wm8904_mute,
		.set_fmt = wm8904_set_dai_fmt,
		.set_sysclk = wm8904_set_dai_sysclk,
	},
};
struct snd_soc_codec_dai *rk1000_codec_dai = &wm8904_dai;
EXPORT_SYMBOL_GPL(wm8904_dai);


static void wm8904_work(struct work_struct *work)
{
	struct snd_soc_codec *codec = container_of(work, struct snd_soc_codec, delayed_work.work);

	wm8904_dapm_event(codec, codec->dapm_state);
	
	printk_d("Enter\n");
}


static int wm8904_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int reg;

	printk_d("Enter\n");
	#if 0
	for (reg = 1; reg < WM8904_MAX_REGISTER; reg++) {
		wm8904_write_reg_cache (codec, reg, wm8904_read(codec, reg));
	}
	#endif
	wm8904_reset(codec);

	return 0;
}


static int wm8904_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int reg;
	u8 data[4];
	
	printk_d("Enter\n");
#if 0
	for (reg = 1; reg < WM8904_MAX_REGISTER; reg++) {
		wm8904_write(codec, reg, wm8904_read_reg_cache(codec, reg));
	}
#endif
    return 0;
}

static int wm8904_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int reg, ret = 0;
	int value;

	printk_d("Enter\n");

	codec->name  = "WM8904";
	codec->owner = THIS_MODULE;
	codec->read  = wm8904_read_reg_cache;
	codec->write = wm8904_write;
	codec->dapm_event = wm8904_dapm_event;
	codec->dai   = &wm8904_dai;
	codec->num_dai		  = 1;
	codec->reg_cache_size = sizeof(wm8904_reg);
	//codec->reg_cache	  = kmemdup(wm8904_reg, sizeof(wm8904_reg), GFP_KERNEL);
	//if (codec->reg_cache == NULL)
	//	return -ENOMEM;
#if 0
	for (reg = 1; reg < WM8904_MAX_REGISTER; reg++) {
		wm8904_write(codec, reg, wm8904_read_reg_cache(codec, reg));
	}
#endif
	frist_mute = 1;
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "wm8904: failed to create pcms\n");
		goto pcm_err;
	}
    
	/* charge output caps */
	codec->dapm_state = SNDRV_CTL_POWER_D3hot;
	schedule_delayed_work(&codec->delayed_work, msecs_to_jiffies(1000));

//	wm8904_add_controls(codec);
//	wm8904_add_widgets(codec);

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "wm8904: failed to register card\n");
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

void wm8904_start_set(struct snd_soc_codec *codec)
{	
	printk_d("Enter\n");

	wm8904_write(codec, 0x00, 0x0000); //0x00 0x0000 SMbus_16_bit_data	   Write  0x34		* SW Reset and ID(00H):    0000  SW_RST_DEV_ID1=0000_0000_0000_0000

	wm8904_write(codec, 0x04, 0x001A); // 0x04 0x001A SMbus_16_bit_data 	Write  0x34 	 * Bias Control 0(04H): 	000B  POBCTRL=1, ISEL=10, STARTUP_BIAS_ENA=1, BIAS_ENA=0
	wm8904_write(codec, 0x05, 0x0047); // 0x05 0x0047 SMbus_16_bit_data 	Write  0x34 	 * VMID Control 0(05H): 	0043  VMID_BUF_ENA=1, VMID_RES=11, VMID_ENA=1
	mdelay(5); //insert_delay_ms 5
	wm8904_write(codec, 0x05, 0x0043); //0x05 0x0043 SMbus_16_bit_data	   Write  0x34		* VMID Control 0(05H):	   0043  VMID_BUF_ENA=1, VMID_RES=01, VMID_ENA=1
	wm8904_write(codec, 0x04, 0x001B); //0x04 0x001B SMbus_16_bit_data	   Write  0x34		* Bias Control 0(04H):	   000B  POBCTRL=1, ISEL=10, STARTUP_BIAS_ENA=1, BIAS_ENA=1
	wm8904_write(codec, 0x0E, 0x0003); //0x0E 0x0003 SMbus_16_bit_data	   Write  0x34		* Power Management 2(0EH): 0003  HPL_PGA_ENA=1, HPR_PGA_ENA=1
	//fibo15	wm8904_write(codec, 0x0F, 0x0003); //0x0F 0x0003 SMbus_16_bit_data	   Write  0x34		* Power Management 3(0FH): 0003  LINEOUTL_PGA_ENA=1, LINEOUTR_PGA_ENA=1

	//ADC path, diff MIC = IN1L + IN2L
	wm8904_write(codec, 0x06, 0x0001); //MICBIAS_ENA=1
	wm8904_write(codec, 0x07, 0x0007); //MICBIAS_SEL=2.7V
	wm8904_write(codec, 0x0C, 0x0002); //INL_ENA=1, INR_ENA=0
	//wm8904_write(codec, 0x0C, 0x0001); //INL_ENA=0, INR_ENA=1
	wm8904_write(codec, 0x12, 0x0003); //ADCL_ENA=1, ADCR_ENA=1
	wm8904_write(codec, 0x18, 0x0010); //R & L channel = L ADC
	//wm8904_write(codec, 0x18, 0x00D0); //R & L channel = R ADC
	//wm8904_write(codec, 0x24, 0x01C0); //0x1FFh=+17.625dB
	//wm8904_write(codec, 0x25, 0x01C0); //0x1FFh=+17.625dB
	wm8904_write(codec, 0x2C, 0x0007); //LIN_VOL=+30dB
	//wm8904_write(codec, 0x2D, 0x0007); //RIN_VOL=+30dB
	wm8904_write(codec, 0x2E, 0x0046); //L_IP_SEL_N=IN1L, L_IP_SEL_P=IN2L	
	//wm8904_write(codec, 0x2F, 0x0046); //R_IP_SEL_N=IN1R, R_IP_SEL_P=IN2R	
	////////ADC DRC 
	//wm8904_write(codec, 0x28, 0x81AF);
	//wm8904_write(codec, 0x29, 0x3248);
	//wm8904_write(codec, 0x2A, 0x0000);
	//wm8904_write(codec, 0x2B, 0x0200); //-12dB threshold

	//***12MHZ FLL, 44.1KHz
	wm8904_write(codec, 0x75, 0x0700); //0x75 0x0700 SMbus_16_bit_data	   Write  0x34		* FLL Control 2(75H):	   0700  FLL_OUTDIV=00_0111, FLL_CTRL_RATE=000, FLL_FRATIO=000
	wm8904_write(codec, 0x76, 0x86C2); //0x76 0x86C2 SMbus_16_bit_data	   Write  0x34		* FLL Control 3(76H):	   8FD5  FLL_K=1000_0110_1100_0010
	wm8904_write(codec, 0x77, 0x00E0); //0x77 0x00E0 SMbus_16_bit_data	   Write  0x34		* FLL Control 4(77H):	   00E0  FLL_N=00_0000_0111, FLL_GAIN=0000
	wm8904_write(codec, 0x74, 0x0005); //0x74 0x0005 SMbus_16_bit_data	   Write  0x34		* FLL Control 1(74H):	   0005  FLL_FRACN_ENA=1, FLL_OSC_ENA=0, FLL_ENA=1
	mdelay(5);//insert_delay_ms 5
	wm8904_write(codec, 0x16, 0x4006); //0x16 0x4006 SMbus_16_bit_data	   Write  0x34		* Clock Rates 2(16H):	   4006  MCLK_INV=0, SYSCLK_SRC=1, MCLK_SRC=0, TOCLK_RATE=0, ADC_DIV=000, DAC_DIV=000, OPCLK_ENA=0, CLK_SYS_ENA=1, CLK_DSP_ENA=1, TOCLK_ENA=0

	//***WM8904 IIS master
	wm8904_write(codec, 0x19, 0x0042); //0x19 0x0042 SMbus_16_bit_data	   Write  0x34		* BCLK=master, IIS 16bit
	wm8904_write(codec, 0x1B, 0x0840); //0x1B 0x0840 SMbus_16_bit_data	   Write  0x34		* LRCK=master

	//*   0x16 0x0006 SMbus_16_bit_data 	Write  0x34 	 * 

	wm8904_write(codec, 0x12, 0x000C); //0x12 0x000C SMbus_16_bit_data	   Write  0x34		* Power Management 6(12H): 000C  DACL_ENA=1, DACR_ENA=1, ADCL_ENA=0, ADCR_ENA=0
	mdelay(3);//insert_delay_ms 3
	//wm8904_write(codec, 0xFF, 0x0000); //0xFF 0x0000 SMbus_16_bit_data	   Write  0x34		* 
	wm8904_write(codec, 0x04, 0x000B); //0x04 0x000B SMbus_16_bit_data	   Write  0x34		* Bias Control 0(04H):	   000B  POBCTRL=0, ISEL=10, STARTUP_BIAS_ENA=1, BIAS_ENA=1
	wm8904_write(codec, 0x62, 0x0001); //0x62 0x0001 SMbus_16_bit_data	   Write  0x34		* Charge Pump 0(62H):	   0001  CP_ENA=1
	mdelay(5); //insert_delay_ms 5
	//wm8904_write(codec, 0xFF, 0x0000); //0xFF 0x0000 SMbus_16_bit_data	   Write  0x34		* 
	wm8904_write(codec, 0x5A, 0x0011); //0x5A 0x0011 SMbus_16_bit_data	   Write  0x34		* Analogue HP 0(5AH):	   00FF  HPL_RMV_SHORT=0, HPL_ENA_OUTP=0, HPL_ENA_DLY=0, HPL_ENA=1, HPR_RMV_SHORT=0, HPR_ENA_OUTP=0, HPR_ENA_DLY=0, HPR_ENA=1
	//fibo15	wm8904_write(codec, 0x5E, 0x0011); //0x5E 0x0011 SMbus_16_bit_data	   Write  0x34		* Analogue Lineout 0(5EH): 00FF  LINEOUTL_RMV_SHORT=0, LINEOUTL_ENA_OUTP=0, LINEOUTL_ENA_DLY=0, LINEOUTL_ENA=1, LINEOUTR_RMV_SHORT=0, LINEOUTR_ENA_OUTP=0, LINEOUTR_ENA_DLY=0, LINEOUTR_ENA=1
	wm8904_write(codec, 0x5A, 0x0033); //0x5A 0x0033 SMbus_16_bit_data	   Write  0x34		* Analogue HP 0(5AH):	   00FF  HPL_RMV_SHORT=0, HPL_ENA_OUTP=0, HPL_ENA_DLY=1, HPL_ENA=1, HPR_RMV_SHORT=0, HPR_ENA_OUTP=0, HPR_ENA_DLY=1, HPR_ENA=1
	//fibo15	wm8904_write(codec, 0x5E, 0x0033); //0x5E 0x0033 SMbus_16_bit_data	   Write  0x34		* Analogue Lineout 0(5EH): 00FF  LINEOUTL_RMV_SHORT=0, LINEOUTL_ENA_OUTP=0, LINEOUTL_ENA_DLY=1, LINEOUTL_ENA=1, LINEOUTR_RMV_SHORT=0, LINEOUTR_ENA_OUTP=0, LINEOUTR_ENA_DLY=1, LINEOUTR_ENA=1
	//fibo15	wm8904_write(codec, 0x43, 0x000F); //0x43 0x000F SMbus_16_bit_data	   Write  0x34		* DC Servo 0(43H):		   000F  DCS_ENA_CHAN_3=1, DCS_ENA_CHAN_2=1, DCS_ENA_CHAN_1=1, DCS_ENA_CHAN_0=1
	//fibo15	wm8904_write(codec, 0x44, 0x00F0); //0x44 0x00F0 SMbus_16_bit_data	   Write  0x34		*
	wm8904_write(codec, 0x43, 0x0003); //0x43 0x000F SMbus_16_bit_data	   Write  0x34		* DC Servo 0(43H):		   000F  DCS_ENA_CHAN_3=1, DCS_ENA_CHAN_2=1, DCS_ENA_CHAN_1=1, DCS_ENA_CHAN_0=1
	wm8904_write(codec, 0x44, 0x0030); //0x44 0x00F0 SMbus_16_bit_data	   Write  0x34		*

	mdelay(256);//insert_delay_ms 256
	wm8904_write(codec, 0xFF, 0x0000); //0xFF 0x0000 SMbus_16_bit_data	   Write  0x34		* 
	wm8904_write(codec, 0x5A, 0x0077); //0x5A 0x0077 SMbus_16_bit_data	   Write  0x34		* Analogue HP 0(5AH):	   00FF  HPL_RMV_SHORT=0, HPL_ENA_OUTP=1, HPL_ENA_DLY=1, HPL_ENA=1, HPR_RMV_SHORT=0, HPR_ENA_OUTP=1, HPR_ENA_DLY=1, HPR_ENA=1
	//fibo15	wm8904_write(codec, 0x5E, 0x0077); //0x5E 0x0077 SMbus_16_bit_data	   Write  0x34		* Analogue Lineout 0(5EH): 00FF  LINEOUTL_RMV_SHORT=0, LINEOUTL_ENA_OUTP=1, LINEOUTL_ENA_DLY=1, LINEOUTL_ENA=1, LINEOUTR_RMV_SHORT=0, LINEOUTR_ENA_OUTP=1, LINEOUTR_ENA_DLY=1, LINEOUTR_ENA=1
	wm8904_write(codec, 0x5A, 0x00FF); //0x5A 0x00FF SMbus_16_bit_data	   Write  0x34		* Analogue HP 0(5AH):	   00FF  HPL_RMV_SHORT=1, HPL_ENA_OUTP=1, HPL_ENA_DLY=1, HPL_ENA=1, HPR_RMV_SHORT=1, HPR_ENA_OUTP=1, HPR_ENA_DLY=1, HPR_ENA=1
	//fibo15	wm8904_write(codec, 0x5E, 0x00FF); //0x5E 0x00FF SMbus_16_bit_data	   Write  0x34		* Analogue Lineout 0(5EH): 00FF  LINEOUTL_RMV_SHORT=1, LINEOUTL_ENA_OUTP=1, LINEOUTL_ENA_DLY=1, LINEOUTL_ENA=1, LINEOUTR_RMV_SHORT=1, LINEOUTR_ENA_OUTP=1, LINEOUTR_ENA_DLY=1, LINEOUTR_ENA=1
	wm8904_write(codec, 0xFF, 0x0000); //0xFF 0x0000 SMbus_16_bit_data	   Write  0x34		* 
	wm8904_write(codec, 0xFF, 0x0000); //0xFF 0x0000 SMbus_16_bit_data	   Write  0x34		* 

	mdelay(50); //mdelay(500);//insert_delay_ms 500

	//*needs to pause for write sequencer to finish before further writes
	wm8904_write(codec, 0x14, 0x845E); //0x14 0x845E SMbus_16_bit_data	   Write  0x34		* Clock Rates 0(14H):	   845E  TOCLK_RATE_DIV16=0, TOCLK_RATE_X4=0, SR_MODE=0, MCLK_DIV=0
	wm8904_write(codec, 0x39, 0x00ED); //0x39 0x00ED SMbus_16_bit_data	   Write  0x34		* Analogue OUT1 Left(39H): 0039  HPOUTL_MUTE=0, HPOUT_VU=0, HPOUTLZC=0, HPOUTL_VOL=10_1101
	wm8904_write(codec, 0x3A, 0x00ED); //0x3A 0x00ED SMbus_16_bit_data	   Write  0x34		* Analogue OUT1 Right(3AH): 00B9  HPOUTR_MUTE=0, HPOUT_VU=1, HPOUTRZC=0, HPOUTR_VOL=10_1101
	wm8904_write(codec, 0x21, 0x0000); //0x21 0x0000 SMbus_16_bit_data	   Write  0x34		* DAC Digital 1(21H):	   0000  DAC_MONO=0, DAC_SB_FILT=0, DAC_MUTERATE=0, DAC_UNMUTE_RAMP=0, DAC_OSR128=0, DAC_MUTE=0, DEEMPH=00
	wm8904_write(codec, 0x68, 0x0005); //0x68 0x0005 SMbus_16_bit_data	   Write  0x34		* Class W 0(68H):		   0005  CP_DYN_PWR=1

}

//Located at /sys/class/sound/card0/device
static ssize_t wm8904_regs(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
    int i;
    char *p, *end;
    u32 addr, value;
    struct snd_soc_device *socdev = wm8904_socdev;
    struct snd_soc_codec *codec = socdev->codec;
    
    //printk("CMD: %s", buf);
    
    if (strstr(buf, "dump"))
    {
        for (i = 0; i <= WM8904_MAX_REGISTER; i++)
            printk("REG: 0x%2x = 0x%4x\n", i, wm8904_read_reg_cache(codec, i));
    }
    else if (strstr(buf, "read"))
    {
        p = strstr(buf, "0x");
        addr = (u32)simple_strtol(p, &end, 16);
        printk("REG: 0x%2x = 0x%4x\n", addr, wm8904_read_reg_cache(codec, addr));
    }
    else if (strstr(buf, "write"))
    {
	p = strstr(buf, "0x");
	addr = (u32)simple_strtol(p, &end, 16);
	p = strstr(end, "0x");
	value = (u32)simple_strtol(p, &end, 16);
	wm8904_write(codec, addr, value);

	printk("Read back after write REG: 0x%2x = 0x%4x\n", addr, 
	wm8904_read_reg_cache(codec, addr));
    }
    
    return count;
}

EXPORT_SYMBOL(wm8904_regs);
static DEVICE_ATTR(wm8904_regs, 0666, NULL, wm8904_regs);


/* If the i2c layer weren't so broken, we could pass this kind of data
   around */

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

static struct i2c_driver wm8904_i2c_driver;
static struct i2c_client client_template;

static int wm8904_codec_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = wm8904_socdev;
	struct wm8904_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;
	
	printk_d("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	if (addr != setup->i2c_address)
		return -ENODEV;
    
	client_template.adapter = adap;
	client_template.addr = addr;
	client_template.mode = NORMALMODE;
	client_template.Channel = I2C_CH1;
	client_template.addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
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
		printk_e("failed to attach codec at addr %x\n", addr);
		goto err;
	}
	ret = wm8904_init(socdev);
	if (ret < 0) {
		printk_e("failed to initialise WM8904\n");
		goto err;
	}
	ret = device_create_file(socdev->dev, &dev_attr_wm8904_regs);
	if (ret != 0)
		printk("Create sys file failed.\n");
        
	return ret;
    
err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int wm8904_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int wm8904_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8904_codec_probe);
}


void wm8904_shutdown(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	printk("WM8904 shutdown..........\n");
}

/* corgi i2c codec control layer */
static struct i2c_driver wm8904_i2c_driver = 
{
	.driver = {
		.name = "WM8904 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             101, //I2C_DRIVERID_WM8904,
	.attach_adapter = wm8904_i2c_attach,
	.detach_client =  wm8904_i2c_detach,
	.command =        NULL,
	.shutdown = wm8904_shutdown,
};

static struct i2c_client client_template = 
{
	.name =   "WM8904",
	.driver = &wm8904_i2c_driver,
};
#endif




static int wm8904_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct wm8904_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = NULL;
	struct wm8904_priv *wm8904 = NULL;

	printk_d("Enter\n");
    
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
	{
	    printk("kmalloc for struct snd_soc_codec failed.\n");
		return -ENOMEM;
	}
    
	wm8904 = kzalloc(sizeof(struct wm8904_priv), GFP_KERNEL);
	if (wm8904 == NULL) 
	{
	    printk("malloc for struct wm8904_priv failed.\n");
		kfree(codec);
		return -ENOMEM;
	}
    
	codec->private_data = wm8904;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	wm8904_socdev = socdev;
	INIT_DELAYED_WORK(&codec->delayed_work, wm8904_work);

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	if (setup->i2c_address) 
	{
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
		codec->hw_read = (hw_write_t)i2c_master_recv;

		ret = i2c_add_driver(&wm8904_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
	}
#else
		/* Add other interfaces here */
#endif
#if 0//test
	wm8904_reset(codec);
	u32 reg;

	printk("    codec regrs:\n");
	for (reg = 0; reg < WM8904_MAX_REGISTER; reg++) {
		if (reg%10 == 0) 
			printk("\n            ");
		printk("0x%04x, ", wm8904_read(codec, reg));
	}
	printk("\n\n");
#endif
	return ret;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;
	
	printk_d("Enter::%s----%d\n",__FUNCTION__,__LINE__);
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
static int wm8904_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	
	printk_d("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    
	if (codec->control_data)
		wm8904_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8904_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);
   
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8904 = 
{
	.probe = 	wm8904_probe,
	.remove = 	wm8904_remove,
	.suspend = 	wm8904_suspend,
	.resume =	wm8904_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_wm8904);

MODULE_DESCRIPTION("ASoC WM8904 driver");
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_LICENSE("GPL");








#if 0//#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static int proc_codec_show (struct seq_file *s, void *v)
{
	struct snd_soc_codec *codec = wm8904_socdev->codec;
	u32 reg;
#if 0
	seq_printf (s, "    codec regrs:\n");
	for (reg = 0; reg < WM8904_MAX_REGISTER; reg++) {
		if (reg%10 == 0) 
			seq_printf (s, "\n            ");
		seq_printf (s, "0x%04x, ", wm8904_read(codec, reg));
	}
	seq_printf (s, "\n\n");

	u8 *cache = codec->reg_cache;
	seq_printf (s, "            cache:\n");
	for (reg = 0; reg < WM8904_MAX_REGISTER; reg++) {
		if (reg%10 == 0) 
			seq_printf (s, "\n            ");
		seq_printf (s, "0x%04x, ", cache[reg]);
	}
	seq_printf (s, "\n\n");
 #else
	int i;

	for (i = 0; i <= WM8904_MAX_REGISTER; i++)
	{
		if (reg%10 == 0) 
			printk("\n");
		printk("0x%04x,  ",wm8904_read_reg_cache(codec, i));
	}


 #endif
	return 0;
}
 
static int proc_codec_open (struct inode *inode, struct file *file)
{
	return single_open (file, proc_codec_show, NULL);
}
 
static const struct file_operations proc_codec_fops = {
	.open  = proc_codec_open,
	.read  = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};
 
static int __init codec_proc_init (void)
{
	proc_create ("codec", 0, NULL, &proc_codec_fops);
	return 0;
}
late_initcall (codec_proc_init);
#endif /* CONFIG_PROC_FS */

