/*
 * es8388.c -- es8388 ALSA SoC audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on es8388.c
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

#include "es8388.h"

#define AUDIO_NAME "ES8388"
#define ES8388_VERSION "v1.0"

#if 0
#define printk_d(fmt, args...)  printk(KERN_INFO "[ES8388] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define printk_d(fmt, args...)
#endif



/* codec private data */
struct es8388_priv {
	unsigned int sysclk;
	enum snd_soc_bias_level bias_level;
};

static struct snd_soc_device*	es8388_socdev = NULL;

static const u16 es8388_reg[] = {
	0x06, 0x1C, 0xC3, 0xFC,  /*  0 *////0x0100 0x0180
	0xC0, 0x00, 0x00, 0x7C,  /*  4 */
	0x80, 0x00, 0x00, 0x06,  /*  8 */
	0x00, 0x06, 0x30, 0x30,  /* 12 */
	0xC0, 0xC0, 0x38, 0xB0,  /* 16 */
	0x32, 0x06, 0x00, 0x00,  /* 20 */
	0x06, 0x32, 0xC0, 0xC0,  /* 24 */
	0x08, 0x06, 0x1F, 0xF7,  /* 28 */
	0xFD, 0xFF, 0x1F, 0xF7,  /* 32 */
	0xFD, 0xFF, 0x00, 0x38,  /* 36 */
	0x38, 0x38, 0x38, 0x38,  /* 40 */
	0x38, 0x00, 0x00, 0x00,  /* 44 */
	0x00, 0x00, 0x00, 0x00,  /* 48 */
	0x00, 0x00, 0x00, 0x00,  /* 52 */
};

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:5;
	u8 single_double:1;
	u8 blckdiv:4;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12000000,  8000, 1500, 0x1B, 0, 0xa},
    
	/* 11.025k */
	{12000000, 11025, 1088, 0x19, 0, 0xa},
    
    /* 12k */
	{12000000, 12000, 1000, 0x18, 0, 0xa},
    
	/* 16k */
	{12000000, 16000,  750, 0x17, 0, 0x6},
    
	/* 22.05k */
	{12000000, 22050,  544, 0x16, 0, 0x6},
    
    /* 24k */
	{12000000, 24000,  500, 0x15, 0, 0x6},
	
	/* 32k */
	{12000000, 32000,  375, 0x14, 0, 0x6},
    
	/* 44.1k */
	{11289600, 44100,  256, 0x02, 0, 0x4}, /* add for hdmi, zyy 2010.6.19 */
	{12000000, 44100,  272, 0x13, 0, 0x4},
    
	/* 48k */
                  
	{12000000, 48000,  250, 0x12, 0, 0x4},
	{12288000, 48000,  256, 0x02, 0, 0x4}, /* add for hdmi */
    
	/* 88.2k */
	{12000000, 88200,  136, 0x11, 1, 0x2},
    
	/* 96k */
	{12000000, 96000,  125, 0x10, 1, 0x2},
};


static inline int get_coeff(int mclk, int rate)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
        if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk) {
            return i;
        }
    }

    pr_err("es8388: could not get coeff for mclk %d @ rate %d\n", mclk, rate);
    return -EINVAL;
}
/*
 * read wm8388 register cache
 */
inline int es8388_read_reg_cache(struct snd_soc_codec *codec, u32 reg)
{
	u16 *cache = codec->reg_cache;
	
	if (reg > ES8388_CACHEREGNUM) {
		printk_d("Invalid reg number: %d\n", reg);
		return -1;
	}

	return cache[reg];
}

/*
 * write wm8388 register cache
 */
static void es8388_write_reg_cache(struct snd_soc_codec *codec,
	u32 reg, u32 value)
{
	u16 *cache = codec->reg_cache;

	if (reg > ES8388_CACHEREGNUM)
	{
	    printk_d("Invalid reg number: %d\n", reg);
		return;
    }
    		
	cache[reg] = value;
}



static unsigned int es8388_read(struct snd_soc_codec *codec, u32 reg)
{
//	printk_d("reg=0x%2x\n",reg);
	return es8388_read_reg_cache(codec, reg);
}

static int es8388_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{
	u8 data[2];
	int ret;
	
//	printk_d("reg=%2d,value=0x%2x\n",reg,value);
	data[0] = reg;
	data[1] = value & 0x00ff;
	
	ret = codec->hw_write(codec->control_data, data, 2);
	if (ret == 2) {
		es8388_write_reg_cache(codec, reg, value);
		return 0;
	}
	
	return ret;
}

static int es8388_set_dai_clkdiv(struct snd_soc_codec_dai *codec_dai,
		int div_id, int div)
{
    return 0;
}

static int es8388_set_dai_sysclk(struct snd_soc_codec_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
    struct snd_soc_codec *codec = codec_dai->codec;
    struct es8388_priv *es8388 = codec->private_data;

    switch (freq) {
        case 11289600:
        case 12000000:
        case 12288000:
        case 16934400:
        case 18432000:
            es8388->sysclk = freq;
            return 0;
    }
    return -EINVAL;
}

static int es8388_set_dai_fmt(struct snd_soc_codec_dai *codec_dai,
		unsigned int fmt)
{
    struct snd_soc_codec *codec = codec_dai->codec;
    u8 iface = 0;
    u8 adciface = 0;
    u8 daciface = 0;

    iface    = snd_soc_read(codec, ES8388_IFACE);
    adciface = snd_soc_read(codec, ES8388_ADC_IFACE);
    daciface = snd_soc_read(codec, ES8388_DAC_IFACE);

    /* set master/slave audio interface */
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBM_CFM:    // MASTER MODE
        	printk_d("es8388 in master mode");
            iface |= 0x80;
            break;
        case SND_SOC_DAIFMT_CBS_CFS:    // SLAVE MODE
        	printk_d("es8388 in slave mode");
            iface &= 0x7F;
            break;
        default:
            return -EINVAL;
    }

    /* interface format */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
        case SND_SOC_DAIFMT_I2S:
            adciface &= 0xFC;
            //daciface &= 0xF9;  //updated by david-everest,5-25           
            daciface &= 0xF9;
            break;
        case SND_SOC_DAIFMT_RIGHT_J:
            break;
        case SND_SOC_DAIFMT_LEFT_J:
            break;
        case SND_SOC_DAIFMT_DSP_A:
            break;
        case SND_SOC_DAIFMT_DSP_B:
            break;
        default:
            return -EINVAL;
    }

    /* clock inversion */
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
        case SND_SOC_DAIFMT_NB_NF:
            iface    &= 0xDF;
            adciface &= 0xDF;
            //daciface &= 0xDF;    //UPDATED BY david-everest,5-25        
            daciface &= 0xBF;
            break;
        case SND_SOC_DAIFMT_IB_IF:
            iface    |= 0x20;
            //adciface &= 0xDF;    //UPDATED BY david-everest,5-25
            adciface |= 0x20;
            //daciface &= 0xDF;   //UPDATED BY david-everest,5-25
            daciface |= 0x40;
            break;
        case SND_SOC_DAIFMT_IB_NF:
            iface    |= 0x20;
           // adciface |= 0x40;  //UPDATED BY david-everest,5-25
            adciface &= 0xDF;
            //daciface |= 0x40;  //UPDATED BY david-everest,5-25
            daciface &= 0xBF;
            break;
        case SND_SOC_DAIFMT_NB_IF:
            iface    &= 0xDF;
            adciface |= 0x20;
            //daciface |= 0x20;  //UPDATED BY david-everest,5-25
            daciface |= 0x40;
            break;
        default:
            return -EINVAL;
    }

    snd_soc_write(codec, ES8388_IFACE    , iface);
    snd_soc_write(codec, ES8388_ADC_IFACE, adciface);
    snd_soc_write(codec, ES8388_DAC_IFACE, daciface);

    return 0;
}

static int es8388_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec =  socdev->codec;
	u16 iface;

	printk_d("\n");
    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        iface = snd_soc_read(codec, ES8388_DAC_IFACE) & 0xC7;
        switch (params_format(params)) {
        case SNDRV_PCM_FORMAT_S16_LE:
            iface |= 0x0018;
            break;
        case SNDRV_PCM_FORMAT_S20_3LE:
            iface |= 0x0008;
            break;
        case SNDRV_PCM_FORMAT_S24_LE:
            break;
        case SNDRV_PCM_FORMAT_S32_LE:
            iface |= 0x0020;
            break;
        }
        snd_soc_write(codec, ES8388_DAC_IFACE, iface);
    }
    else 
	{
        iface = snd_soc_read(codec, ES8388_ADC_IFACE) & 0xE3;
        switch (params_format(params)) {
        case SNDRV_PCM_FORMAT_S16_LE:
            iface |= 0x000C;
            break;
        case SNDRV_PCM_FORMAT_S20_3LE:
            iface |= 0x0004;
            break;
        case SNDRV_PCM_FORMAT_S24_LE:
            break;
        case SNDRV_PCM_FORMAT_S32_LE:
            iface |= 0x0010;
            break;
        }
        snd_soc_write(codec, ES8388_ADC_IFACE, iface);
    }
		
	return 0;
}

static int es8388_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	printk_d("level=%d\n",level);

    switch(level)
    {
        case SND_SOC_BIAS_ON:
            break;

        case SND_SOC_BIAS_PREPARE:
			snd_soc_write(codec, ES8388_ADCPOWER, 0x00);
			snd_soc_write(codec, ES8388_DACPOWER , 0x30);
			snd_soc_write(codec, ES8388_CHIPPOWER , 0x00);                
			break;

        case SND_SOC_BIAS_STANDBY:
            snd_soc_write(codec, ES8388_ADCPOWER, 0x00);
            snd_soc_write(codec, ES8388_DACPOWER , 0x30);
            snd_soc_write(codec, ES8388_CHIPPOWER , 0x00);               
            break;

        case SND_SOC_BIAS_OFF:
            snd_soc_write(codec, ES8388_ADCPOWER, 0xff);
            snd_soc_write(codec, ES8388_DACPOWER , 0xC0);
            snd_soc_write(codec, ES8388_CHIPPOWER , 0xC3);             
            break;
    }

	return 0;
}

static int es8388_mute(struct snd_soc_codec_dai *dai, int mute)
{
    struct snd_soc_codec *codec = dai->codec;
    unsigned char val = 0;
	
	printk_d("%s\n",(mute==1)? "mute":"unmute");
    val = snd_soc_read(codec, ES8388_DAC_MUTE);
    if (mute){
        val |= 0x04;
		GPIOSetPinLevel(GPIOPortF_Pin7,GPIO_LOW);
		snd_soc_write(codec, ES8388_DAC_MUTE, val);
    } else {
        val &= ~0x04;
		snd_soc_write(codec, ES8388_DAC_MUTE, val);
		GPIOSetPinLevel(GPIOPortF_Pin7,GPIO_HIGH);
    }

    return 0;
}
static int es8388_initalize(struct snd_soc_codec *codec)
{
	//snd_soc_write(codec, ES8388_MASTERMODE  , 0x00);    // slave, mclk not divide,||to support slave and master
	
	snd_soc_write(codec, ES8388_CHIPPOWER   , 0xf3);    // Power down: ADC DEM, DAC DSM/DEM, ADC/DAC state machine, ADC/DAC ananlog reference
	snd_soc_write(codec, ES8388_DACCONTROL21, 0x80);    // DACLRC and ADCLRC same, ADC/DAC DLL power up, Enable MCLK input from PAD.

	snd_soc_write(codec, ES8388_CONTROL1   , 0x05);     // VMIDSEL (500 kohme divider enabled)
	snd_soc_write(codec, ES8388_CONTROL2 , 0x72);   // 


	snd_soc_write(codec, ES8388_DACPOWER   , 0x30);     // DAC R/L Power on, OUT1 enable, OUT2 disable
	snd_soc_write(codec, ES8388_ADCPOWER   , 0x00);     // 
	snd_soc_write(codec, ES8388_ANAVOLMANAG, 0x7C);     // 

	//-----------------------------------------------------------------------------------------------------------------
	snd_soc_write(codec, ES8388_ADCCONTROL1, 0xbb);     // MIC PGA gain: +24dB
	snd_soc_write(codec, ES8388_ADCCONTROL2, 0xf0);     // LINSEL(L-R differential), RINGSEL(L-R differential)
	snd_soc_write(codec, ES8388_ADCCONTROL3, 0x82);     // Input Select: LIN2/RIN2
	snd_soc_write(codec, ES8388_ADCCONTROL4, 0x4C);     // Left data = left ADC, right data = right ADC, 24 bits I2S
	snd_soc_write(codec, ES8388_ADCCONTROL5, 0x02);     // 256fs
	snd_soc_write(codec, ES8388_ADCCONTROL6, 0x00);     // Disable High pass filter

	snd_soc_write(codec, ES8388_LADC_VOL, 0x00);        // 0dB
	snd_soc_write(codec, ES8388_RADC_VOL, 0x00);        // 0dB

	//snd_soc_write(codec, ES8388_ADCCONTROL10, 0x3A);    // ALC stereo, Max gain(17.5dB), Min gain(0dB)
	snd_soc_write(codec, ES8388_ADCCONTROL10, 0xe2);    // ALC stereo, Max gain(17.5dB), Min gain(0dB),updated by david-everest,5-25
	snd_soc_write(codec, ES8388_ADCCONTROL11, 0xA0);    // ALCLVL(-1.5dB), ALCHLD(0ms)
	snd_soc_write(codec, ES8388_ADCCONTROL12, 0x05);    // ALCDCY(1.64ms/363us), ALCATK(1664us/363.2us)
	snd_soc_write(codec, ES8388_ADCCONTROL13, 0x06);    // ALCMODE(ALC mode), ALCZC(disable), TIME_OUT(disable), WIN_SIZE(96 samples)
	snd_soc_write(codec, ES8388_ADCCONTROL14, 0xd3);    // NGTH(XXX), NGG(mute ADC output), NGAT(enable)


	//----------------------------------------------------------------------------------------------------------------
	snd_soc_write(codec, ES8388_DACCONTROL1, 0x18);     // I2S 16bits 
	snd_soc_write(codec, ES8388_DACCONTROL2, 0x02);     // 256fs

	snd_soc_write(codec, ES8388_LDAC_VOL, 0x00);    // left DAC volume
	snd_soc_write(codec, ES8388_RDAC_VOL, 0x00);    // right DAC volume

	//snd_soc_write(codec, ES8388_DACCONTROL3, 0xE0);     // DAC unmute

	snd_soc_write(codec, ES8388_DACCONTROL17, 0xb8);    // left DAC to left mixer enable, 
	snd_soc_write(codec, ES8388_DACCONTROL18, 0x38);    // ???
	snd_soc_write(codec, ES8388_DACCONTROL19, 0x38);    // ???
	snd_soc_write(codec, ES8388_DACCONTROL20, 0xb8);    // right DAC to right mixer enable,

	snd_soc_write(codec, ES8388_CHIPPOWER, 0x00);   // ALL Block POWER ON
	//snd_soc_write(codec, ES8388_CONTROL2 , 0x72);   // updated by david-everest,5-25
	//mdelay(100);

	snd_soc_write(codec, ES8388_LOUT1_VOL, 0x1c);   // 
	snd_soc_write(codec, ES8388_ROUT1_VOL, 0x1c);   // 
	snd_soc_write(codec, ES8388_LOUT2_VOL, 0x00);   // Disable LOUT2
	snd_soc_write(codec, ES8388_ROUT2_VOL, 0x00);   // Disable ROUT2

	return 0;
}


#define ES8388_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
                    SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
                    SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define ES8388_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
                    SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_codec_dai es8388_dai = {
	.name = "ES8388",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES8388_RATES,
		.formats = ES8388_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES8388_RATES,
		.formats = ES8388_FORMATS,
	},
	.ops = {
		.hw_params = es8388_hw_params,
	},
	.dai_ops = {
		.set_fmt = es8388_set_dai_fmt,
		.set_sysclk = es8388_set_dai_sysclk,
		.set_clkdiv = es8388_set_dai_clkdiv,
		.digital_mute = es8388_mute,
	},
};

EXPORT_SYMBOL_GPL(es8388_dai);


#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver es8388_i2c_driver;
static struct i2c_client client_template;

static int es8388_i2c_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	printk_d("\n");
	codec->name  = "ES8388";
	codec->owner = THIS_MODULE;
	codec->read  = es8388_read;
	codec->write = es8388_write;
	codec->dai = &es8388_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(es8388_reg);
	codec->reg_cache = kmemdup(es8388_reg, sizeof(es8388_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
    
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		pr_err("es8388: failed to create pcms\n");
		goto err;
	}
	/* register card */
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		pr_err("es8388: failed to register card\n");
		goto err;
	}

	printk_d("end\n");
	return ret;
err:
	  snd_soc_free_pcms(socdev);
	  snd_soc_dapm_free(socdev);
	  return ret;
}

static __devinit int es8323_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = es8388_socdev;
	struct codec_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;

	printk_d("\n");
	if (addr != setup->i2c_address) {
		pr_err("i2c_address is error,addr=%d, i2c_address=%d\n",addr,setup->i2c_address);
		return -ENODEV;
	}
	
	client_template.adapter = adap;
	client_template.addr = addr;
	client_template.mode = NORMALMODE;
	client_template.Channel = I2C_CH1;
	client_template.addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	client_template.speed = 200;
	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL) {
		pr_err("failed to dump memory\n");
		kfree(codec);
		return -ENOMEM;
	}
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;
    
	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		pr_err("failed to attach codec at addr %x\n", addr);
		goto err;
	}
	
	ret = es8388_i2c_init(socdev);
	if (ret < 0) {
		pr_err("failed to initialize ES8388\n");
		goto err;
	}
	es8388_initalize(codec);
	printk_d("end\n");
	return ret;
err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int es8388_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int es8388_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, es8323_i2c_probe);
}
/* turn off pop sound when shutdown */
void es8388_shutdown(struct i2c_client *client)
{
	GPIOSetPinLevel(GPIOPortF_Pin7, GPIO_LOW);
}

static struct i2c_driver es8388_i2c_driver = {
	.driver = {
		.name = "ES8388 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id = 101,
	.attach_adapter = es8388_i2c_attach,
	.detach_client = es8388_i2c_detach,
	.command = NULL,
	.shutdown = es8388_shutdown,
};

static struct i2c_client client_template = 
{
	.name =   "ES8388",
	.driver = &es8388_i2c_driver,
};
#endif

static int es8388_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    struct snd_soc_codec *codec = socdev->codec; 
	printk_d("\n");
	
	snd_soc_write(codec,0x07, 0x7b);
	snd_soc_write(codec,0x05, 0xff);
	snd_soc_write(codec,0x06, 0xff);
	snd_soc_write(codec,0x03, 0xff);
	snd_soc_write(codec,0X02, 0xaa);

	printk_d("end \n");

	return 0;
}

static int es8388_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	printk_d("\n");

	snd_soc_write(codec,0x07, 0x7c);
	snd_soc_write(codec,0x05, 0x00);
	snd_soc_write(codec,0x06, 0x00);
	snd_soc_write(codec,0X03, 0x00);//ADC ON
	snd_soc_write(codec,0X02, 0x00);
	
	printk_d("end \n");
	
	return 0;
}

static int es8388_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct codec_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = NULL;
	struct es8388_priv *es8388 = NULL;
	int ret = 0;

	printk_d("\n");
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
	{
	    pr_err("kmalloc for struct snd_soc_codec failed.\n");
		return -ENOMEM;
	}
	
	es8388 = kzalloc(sizeof(struct es8388_priv), GFP_KERNEL);
	if (es8388 == NULL) 
	{
		pr_err("malloc for struct private failed.\n");
		kfree(codec);
		return -ENOMEM;
	}
	codec->private_data = es8388;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	es8388_socdev = socdev;

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	if (setup->i2c_address) 
	{
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
		codec->hw_read = (hw_read_t)i2c_master_recv;
		ret = i2c_add_driver(&es8388_i2c_driver);
		if (ret != 0)
			pr_err("can't add i2c driver");
	}
#endif

	printk_d("end\n");
	return ret;
}

static int es8388_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_del_driver(&es8388_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_es8388 = {
	.probe = 	es8388_probe,
	.remove = 	es8388_remove,
	.suspend = 	es8388_suspend,
	.resume =	es8388_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_es8388);



MODULE_DESCRIPTION("ASoC es8388 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static int proc_codec_show (struct seq_file *s, void *v)
{
	struct snd_soc_codec *codec = es8388_socdev->codec;
	unsigned int reg;

	seq_printf (s, "ES8388 registers value:\n");
	for (reg = 0; reg < ES8388_CACHEREGNUM; reg++) {
		seq_printf (s, "%2d=0x%2x\n",reg,es8388_read(codec, reg));
	}
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
	proc_create ("es8388", 0, NULL, &proc_codec_fops);
	return 0;
}
late_initcall (codec_proc_init);
#endif /* CONFIG_PROC_FS */

