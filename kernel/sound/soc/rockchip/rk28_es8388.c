/*
 * rk28_es8388.c  --  SoC audio for rockchip
 *
 * Driver for rockchip es8388 audio
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/hw_common.h>

#include "rk28-pcm.h"
#include "rk28-iis.h"
#include "../codecs/es8388.h"


#if 0
#define printk_d(fmt, args...)  printk(KERN_INFO "[RK-ES8388] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define printk_d(fmt, args...)
#endif

static int rockchip_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0; 
	unsigned int lrclk = 0;
	int ret;
	
	printk_d("\n");

	/* set codec DAI configuration */
#if defined (CONFIG_SND_ROCKCHIP_SOC_MASTER) 
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
	      SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS); 
#endif	
#if defined (CONFIG_SND_ROCKCHIP_SOC_SLAVE) 
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
	      SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM ); 
#endif
	if (ret < 0)
		return ret;
	
	/* set cpu DAI configuration */
#if defined (CONFIG_SND_ROCKCHIP_SOC_MASTER) 
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
	      SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#endif	
#if defined (CONFIG_SND_ROCKCHIP_SOC_SLAVE) 
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
	      SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);	
#endif		
	if (ret < 0)
		return ret;
	
	switch(params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
			pll_out = 12288000;
			break;
	case 11025:
	case 22050:
	case 44100:
			pll_out = 11289600;
			break;
	default:
			return -EINVAL;
			break;
	}
	
	printk_d("MCLK=%d,LRCK=%d\n", pll_out, params_rate(params));
	ret = cpu_dai->dai_ops.set_sysclk(cpu_dai, 0, pll_out, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	ret = codec_dai->dai_ops.set_sysclk(codec_dai, 0, pll_out, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;


	printk_d("end\n");
	return ret;
}

static struct snd_soc_ops rockchip_ops = {
	.hw_params = rockchip_hw_params,
};

static struct snd_soc_dai_link rockchip_dai = {
	  .name = "ES8388",
	  .stream_name = "ES8388_PCM",
	  .cpu_dai = &rockchip_i2s_dai,
	  .codec_dai = &es8388_dai,
	  .ops = &rockchip_ops,
};

static struct snd_soc_machine snd_soc_machine_rockchip = {
	  .name = "ROCKCHIP_ES8388",
	  .dai_link = &rockchip_dai,
	  .num_links = 1,
};

static struct codec_setup_data rockchip_es8388_setup = {
	  .i2c_address = 0x10,
};

static struct snd_soc_device rockchip_snd_devdata = {
	  .machine = &snd_soc_machine_rockchip,
	  .platform = &rockchip_soc_platform,
	  .codec_dev = &soc_codec_dev_es8388,
	  .codec_data = &rockchip_es8388_setup,
};

static struct platform_device *rockchip_snd_device;

static int __init audio_card_init(void)
{
	int ret =0;
	
	printk_d("\n");
	if (!request_mem_region(I2S_BASE_ADDR, 0x20, "soc-audio")) {
		return -EBUSY;
	}
	
	rockchip_snd_device = platform_device_alloc("soc-audio", -1);
	if (!rockchip_snd_device) {
		pr_err("Fail to allocate platform device\n");
		ret = -ENOMEM;
		goto fail_release_mem;
	}
	
	platform_set_drvdata(rockchip_snd_device, &rockchip_snd_devdata);
	rockchip_snd_devdata.dev = &rockchip_snd_device->dev;
	ret = platform_device_add(rockchip_snd_device);
	if (ret) {
		pr_err("Fail to add platform device\n");
		platform_device_put(rockchip_snd_device);
		goto fail_release_mem;
	}
	printk_d("end\n");
	return ret;
fail_release_mem:
	release_mem_region(I2S_BASE_ADDR, 0x20);
	return ret;
}
static void __exit audio_card_exit(void)
{
	platform_device_unregister(rockchip_snd_device);
}

module_init(audio_card_init);
module_exit(audio_card_exit);
/* Module information */
MODULE_AUTHOR("rockchip");
MODULE_DESCRIPTION("ROCKCHIP i2s ASoC Interface");
MODULE_LICENSE("GPL");

