/*
 * Windfarm PowerMac thermal control. LM75 sensor
 *
 * (c) Copyright 2005 Benjamin Herrenschmidt, IBM Corp.
 *                    <benh@kernel.crashing.org>
 *
 * Released under the term of the GNU GPL v2.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/i2c.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/sections.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include "eink_s.h"
#include <asm/arch/adc.h>
//#include <asm/arch/rk28_define.h>





struct epd_ntc_sensor {
	struct  epd_sensor	sens;
};


int  epd_ad_value[50] = {3292,3127,2972,2825,2686,2555,2431,2314,2203,2098,
						1999,1904,1815,1731,1651,1575,1503,1435,1371,1309,
						1251,1195,1143,1093,1045,1000,957,916,877,840,
						805,771,739,709,680,652,625,600,576,553,
						531,510,490,471,453,435,419,403,387,373};
#define  EPD_ADC_NUM 	2
extern int get_rock_adc_sync(int ch);
extern int32 ADCInit(void);

static int epd_ntc_shut_down()
{
	
	return 0;
}
static int epd_ntc_wake_up()
{
	return 0;
}
static int epd_ntc_get(s32 *value)
{
	int temp = 27;
	int rc;
	int ad_result;
	int r_value;
	int i;
	ad_result = get_rock_adc_sync(EPD_ADC_NUM);
	printk("ad_result=%d\n",ad_result);
	r_value = (ad_result*1000)/(1024-ad_result);
	for(i = 50; i >0; i--){
		if(r_value < epd_ad_value[i])
			break;
	}
	temp = i + 1;
	if(temp == 50 || temp == 1){
		temp = 27;
	}
		
	*value = temp;
	return 0;
}


static struct epd_sensor_ops epd_ntc_ops = {
	.get_value	= epd_ntc_get,
	.release		= NULL,
	.shutdown 	= epd_ntc_shut_down,
	.wakeup 		= epd_ntc_wake_up,
	.owner		= THIS_MODULE,
};

static int epd_ntc_probe(struct platform_device *pdev)
{   
	struct epd_ntc_sensor *lm;
	int rc;

	
	lm = kzalloc(sizeof(struct epd_ntc_sensor), GFP_KERNEL);
	if (lm == NULL)
		return NULL;
	lm->sens.ops = &epd_ntc_ops;
	if (epd_register_sensor(&lm->sens)) {
		goto fail;
	}
	ADCInit();
	return lm;
 fail:
	kfree(lm);
	return NULL;
}
static struct platform_driver ntc_driver = {
	.probe		= epd_ntc_probe,
	.driver		= {
	.name	= "rk2818-ntc",
	.owner	= THIS_MODULE,
	},
};
static struct platform_device *ntc_platform_device;

static int __init epd_ntc_sensor_init(void)
{

	int retval;
	ntc_platform_device = platform_device_alloc("rk2818-ntc", -1);
	if (!ntc_platform_device)
		return -ENOMEM;
	
	retval = platform_device_add(ntc_platform_device);
	if (retval < 0) {
		platform_device_put(ntc_platform_device);
		return retval;
	}
	return platform_driver_register(&ntc_driver);
}

static void __exit epd_ntc_sensor_exit(void)
{
	 platform_driver_unregister(&ntc_driver);
}

subsys_initcall(epd_ntc_sensor_init);

//fs_initcall_sync(epd_lm75_sensor_init);
module_exit(epd_ntc_sensor_exit);

MODULE_AUTHOR("Benjamin Herrenschmidt <benh@kernel.crashing.org>");
MODULE_DESCRIPTION("LM75 sensor objects for PowerMacs thermal control");
MODULE_LICENSE("GPL");



