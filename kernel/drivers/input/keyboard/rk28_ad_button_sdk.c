/*
 * linux/drivers/input/keyboard/rk28_ad_button.c
 *
 * Driver for the rk28 matrix keyboard controller.
 *
 * Created: 2009-5-4
 * Author:	LZJ <lzj@rockchip.com>
 *
 * This driver program support to AD key which use for rk28 chip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/typedef.h>
#include <asm/arch/gpio.h>
#include <asm/arch/hardware.h>
#include <asm/arch/iomux.h>
#include <asm/arch/adc.h>
#include <asm/arch/rk28_debug.h>

//#define ADKEYDBG

#ifdef ADKEYDBG
#define printk_d(fmt, args...)  printk(KERN_INFO "[ADKEY] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define printk_d(fmt, args...)
#endif

#define KEY_PHYS_NAME	"rk28_AD_button/input0"
#define ADC_CHANNEL_NUM	(1)
#define Valuedrift		(70)
#define EmptyADValue	(950)
#define InvalidADValue	(10)

/*power event button*/
#define  ONESEC_TIMES	(100)
#define  SEC_NUM		(1)
#define  SLEEP_TIME		(2)	/*per 40ms*/
#define  res_size(res)	((res)->end - (res)->start + 1)

static unsigned  int g_code = 0;
static unsigned  int g_wake =0;
static unsigned  int g_io_key_count = 0;
static unsigned  int g_io_key_press = 0;
static unsigned  int g_io_key_state = 0;


static int ADSampleTimes = 0;
//static unsigned  int	pwrscantimes = 0;
//static unsigned int 	encall_times = 100;


typedef  struct tagADC_keyst
{
	unsigned int adc_value;
	unsigned int adc_keycode;
}ADC_keyst,*pADC_keyst;

#define Vref	(3.3)
#define ADCValue(x)	(x*1024/Vref)
/*Only to modify below as different key function*/
#define AD1KEY1		KEY_LEFT
#define AD1KEY2 		KEY_RIGHT
#define AD1KEY3 		KEY_UP	
#define AD1KEY4 		KEY_DOWN
#define AD1KEY5		KEY_BACK
#define AD1KEY6		KEY_REPLY //yuzhe: set this key as DPAD_CENTER

#define AD2KEY1		KEY_VOLUMEDOWN
#define AD2KEY2  	KEY_VOLUMEUP
#define AD2KEY3 		KEY_F1
#define AD2KEY4 		KEY_HOME	
#define AD2KEY5 		KEY_BACK
#define AD2KEY6 		KEY_POWER
/*Only to modify below voltage as different hardware design*/
static  ADC_keyst ad1valuetab[] = 
{
	{ADCValue(0.30), AD1KEY1},
	{ADCValue(0.80), AD1KEY2},
	{ADCValue(1.31), AD1KEY3},
	{ADCValue(1.81), AD1KEY4},
	{ADCValue(2.33), AD1KEY5},
	{ADCValue(2.90), AD1KEY6},
	{EmptyADValue, KEY_RESERVED}
};

static  ADC_keyst ad2valuetab[] = {
	{ADCValue(0.30), AD2KEY1},
	{ADCValue(0.80), AD2KEY2},
	{ADCValue(1.31), AD2KEY3},
	{ADCValue(1.81), AD2KEY4},
	{ADCValue(2.34), AD2KEY5},
	{EmptyADValue, KEY_RESERVED}
};

static unsigned int initkey_code[ ] = 
{
	AD1KEY1,
	AD1KEY2,
	AD1KEY3,
	AD1KEY4,	
	AD1KEY5,
	AD1KEY6,
	
	AD2KEY1,
	AD2KEY2,
	AD2KEY3,
	AD2KEY4,
	AD2KEY5,

	KEY_POWER,
	KEY_WAKEUP,
	KEY_RTC_WAKEUP,
	KEY_RESERVED
};

struct rk28_AD_button_platform_data {
	int x;
};

struct rk28_AD_button {
	struct rk28_AD_button_platform_data *pdata;
	struct timer_list timer;

	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;
	/* matrix key code map */
	unsigned char keycodes[13];
	/* state row bits of each column scan */
	uint32_t direct_key_state;
	unsigned int direct_key_mask;
	int rotary_rel_code[2];
	int rotary_up_key[2];
	int rotary_down_key[2];
};

 struct rk28_AD_button *prockAD_button;


extern void RockAdcScanning(void);
extern void printADCValue(void);
extern void get_rock_adc_scanning_end(void);
extern void get_rock_adc_scanning_restart(void);
extern int get_rock_adc1(void);
extern int get_rock_adc2(void);
extern int get_rock_adc3(void);
extern int get_rock_adc_sync(int ch);
extern int32 ADCInit(void);


unsigned int find_rock_adkeycode(unsigned int advalue,pADC_keyst ptab)
{	
	while(ptab->adc_value!=EmptyADValue)
	{
		if((advalue>ptab->adc_value-Valuedrift)&&(advalue<ptab->adc_value+Valuedrift))
			return ptab->adc_keycode;
		ptab++;
	}
	
	return 0;
}

static int rk28_AD_button_open(struct input_dev *dev)
{
//	struct rk28_AD_button *AD_button = input_get_drvdata(dev);
	return 0;
}

static void rk28_AD_button_close(struct input_dev *dev)
{
//	struct rk28_AD_button *AD_button = input_get_drvdata(dev);
	return;
}

void scan_io_key(void)
{
/*	if(!GPIOGetPinLevel(PLAY_ON_IOPIN))
	{
		pwrscantimes += 1;
		if(pwrscantimes == (SEC_NUM * ONESEC_TIMES)) {
			input_report_key(prockAD_button->input_dev,KEY_POWER,1);
			input_sync(prockAD_button->input_dev);
			printk_d("the kernel come to power down!!!\n");
			encall_times = 100;
		}
		if(pwrscantimes ==( (SEC_NUM + 1)* ONESEC_TIMES)) {
			pwrscantimes = 0;
			input_report_key(prockAD_button->input_dev,KEY_POWER,0);
			input_sync(prockAD_button->input_dev);
			printk_d("the kernel come to power up!!!\n");
			encall_times = 100;
		}
		return ;
	} else {
		if(encall_times > 0)
			encall_times--;
	}
	if( pwrscantimes > SLEEP_TIME )
	{
		pwrscantimes = 0;
		if(rk2818_get_suspend_flags() !=  PM_AWAKE)
		{
			if(rk2818_get_suspend_flags() == PM_TWOLEVEL_SLEEP){
				printk_d("power key:  key wake up\n");
				return;
			}
			if(encall_times == 0) {
				input_report_key(prockAD_button->input_dev,KEY_WAKEUP,1);
				input_sync(prockAD_button->input_dev);
				input_report_key(prockAD_button->input_dev,KEY_WAKEUP,0);
				input_sync(prockAD_button->input_dev);
				printk_d("power key: wake up\n");
			}
		} else {
			if(encall_times == 0) {
				input_report_key(prockAD_button->input_dev,KEY_POWER,1);
				input_sync(prockAD_button->input_dev);
				input_report_key(prockAD_button->input_dev,KEY_POWER,0);
				input_sync(prockAD_button->input_dev);
				printk_d("power key: sleep\n");
			}
		}
	}
	*/
	if (!GPIOGetPinLevel(PLAY_ON_IOPIN))
	{
		if (g_io_key_state == 0)
		{
			g_io_key_state = 1;
			g_io_key_count = 0;
		}
	} 
	else 
	{
		if (g_io_key_state)
		{
			g_io_key_state = 0;
			g_io_key_count = 0;
		}
	}

	g_io_key_count++;
	if (g_io_key_count < 2)
		return;
	
	g_io_key_count = 0;
	if (g_io_key_press == g_io_key_state)
		return ;

	g_io_key_press = g_io_key_state;

	input_report_key(prockAD_button->input_dev, KEY_POWER, g_io_key_press);
	input_sync(prockAD_button->input_dev);
	
	printk_d("key power(%d) state = %d!\n", KEY_POWER, g_io_key_press);

}

void rk28_adkey_check(int code) 
{
	if (g_code == code) {
		return;
	} else if (code != 0 && g_code == 0) {
		g_code = code;
		printk_d(" keycode[%d] up\n",g_code);
		input_report_key(prockAD_button->input_dev,g_code,1);
		input_sync(prockAD_button->input_dev);
	} else if (code == 0 && g_code != 0) {
		printk_d(" keycode[%d] down\n",g_code);
		input_report_key(prockAD_button->input_dev,g_code,0);
		input_sync(prockAD_button->input_dev);
		g_code = 0;
	}
}

static int rk28_adkeyscan_sampler(int channel, int checkCount)
{
	static int adcValue[2];

	if (checkCount > 2) {
		return -2;
	}

	adcValue[checkCount - 1] = get_rock_adc_sync(channel);
	if (checkCount == 2) {
		printk_d("adcValue[0] = %d  adcValue[1] = %d \n", adcValue[0], adcValue[1]);
		
		if ((adcValue[0] < 0) || (adcValue[1] < 0) || (adcValue[0] - adcValue[1] > Valuedrift / 5) || (adcValue[1] - adcValue[0] > Valuedrift / 5))
			return -2;
		else
			return (adcValue[0] + adcValue[1]) / 2;
	} else {
		return -1;
	}
}

static void rk28_adkeyscan_timer(unsigned long data)
{
	unsigned int keyAdc;
	unsigned int code = 0;
	int keyboard;
	static int checkAdcCount = 0;
	static int checkChannel  = 0;

	prockAD_button->timer.expires  = jiffies+msecs_to_jiffies(10);
	add_timer(&prockAD_button->timer);

	if (checkAdcCount) {
		keyAdc = rk28_adkeyscan_sampler(checkChannel, checkAdcCount);
		printk_d("keyAdc=%d checkChannel=%d checkAdcCount=%d\n", keyAdc, checkChannel, checkAdcCount);
		checkAdcCount++;
		if (keyAdc == -1) {
			return;
		} else if (keyAdc == -2) {
			checkAdcCount = 1;
			printk_d("rk28_adkeyscan_sampler error == -2 \n");
			return;
		}

		if (keyAdc > 0) {
			code = find_rock_adkeycode(keyAdc, (checkChannel == 1)? ad1valuetab : ad2valuetab);
			rk28_adkey_check(code);
		}

		get_rock_adc_scanning_restart();
		checkAdcCount =0;
		return;
	}

	RockAdcScanning();
	if (ADSampleTimes < 4) {
		ADSampleTimes ++;
		scan_io_key();
		return;
	} else {
		ADSampleTimes = 0;	
	}

	for (keyboard = 0; keyboard < ADC_CHANNEL_NUM; keyboard++) 
	{
		if (keyboard == 0)
			keyAdc = get_rock_adc1();
		else
			keyAdc = get_rock_adc2();

		if ((keyAdc > EmptyADValue) || (keyAdc < InvalidADValue))
			continue;

		code = g_code;
		if (g_code) {
			break;
		}

		get_rock_adc_scanning_end();
		checkChannel  = keyboard + 1;
		checkAdcCount = 1;
		break;
	}

	rk28_adkey_check(code);
	scan_io_key();
}

void rk28_send_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() == PM_AWAKE )
		return;
	
	printk("%s:line%d\n",__FUNCTION__,__LINE__);
  	rk2818_set_suspend_flags(PM_AWAKE);
	input_report_key(prockAD_button->input_dev,KEY_WAKEUP,1);
	input_sync(prockAD_button->input_dev);
	input_report_key(prockAD_button->input_dev,KEY_WAKEUP,0);
	input_sync(prockAD_button->input_dev);
}

void rk28_send_rtc_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() == PM_AWAKE )
		return;
	
	printk("%s:line%d\n",__FUNCTION__,__LINE__);
  	rk2818_set_suspend_flags(PM_AWAKE);
	input_report_key(prockAD_button->input_dev,KEY_RTC_WAKEUP,1);
	input_sync(prockAD_button->input_dev);
	input_report_key(prockAD_button->input_dev,KEY_RTC_WAKEUP,0);
	input_sync(prockAD_button->input_dev);
}

static irqreturn_t rk28_AD_irq_handler(s32 irq, void *dev_id)
{
	if(rk2818_get_suspend_flags() == PM_TWOLEVEL_SLEEP)
	{
		rk28_send_wakeup_key();
	}
	return IRQ_HANDLED;
}

static int __devinit rk28_AD_button_probe(struct platform_device *pdev)
{
	struct rk28_AD_button *AD_button;
	struct input_dev *input_dev;
	int error = 0;
	int i = 0;
	AD_button = kzalloc(sizeof(struct rk28_AD_button), GFP_KERNEL);
	/* Create and register the input driver. */
	input_dev = input_allocate_device();
	if (!input_dev || !AD_button) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto failed1;
	}

	memcpy(AD_button->keycodes, initkey_code, sizeof(AD_button->keycodes));
	input_dev->name = pdev->name;
	input_dev->open = rk28_AD_button_open;
	input_dev->close = rk28_AD_button_close;
	input_dev->dev.parent = &pdev->dev;
	input_dev->phys = KEY_PHYS_NAME;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->keycode = AD_button->keycodes;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(initkey_code);
	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)
		set_bit(initkey_code[i], input_dev->keybit);
	
	clear_bit(0, input_dev->keybit);
	AD_button->input_dev = input_dev;
	input_set_drvdata(input_dev, AD_button);
	input_dev->evbit[0] = BIT_MASK(EV_KEY) ;
	platform_set_drvdata(pdev, AD_button);

	ADCInit();
	prockAD_button=AD_button;

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed2;
	}

	setup_timer(&AD_button->timer, rk28_adkeyscan_timer, (unsigned long)AD_button);
	AD_button->timer.expires  = jiffies + 3;
	add_timer(&AD_button->timer);
	
	error = request_gpio_irq(PLAY_ON_IOPIN,(pFunc)rk28_AD_irq_handler,GPIOEdgelFalling,NULL);
	if(error) {
		printk("unable to request playon key IRQ\n");
		goto failed2;
	}
	error = request_gpio_irq(VBUS_INT_IOPIN,(pFunc)rk28_AD_irq_handler, GPIOEdgelRising, NULL);
	if(error) {
		printk("unable to request vbus IRQ\n");
		goto failed2;
	}
	return 0;

failed2:
	input_unregister_device(AD_button->input_dev);
	platform_set_drvdata(pdev, NULL);	
failed1:
	input_free_device(input_dev);
	kfree(AD_button);
	return error;
}

static int __devexit rk28_AD_button_remove(struct platform_device *pdev)
{
	struct rk28_AD_button *AD_button = platform_get_drvdata(pdev);

	input_unregister_device(AD_button->input_dev);
	input_free_device(AD_button->input_dev);
	kfree(AD_button);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

int ad_button_suspend(struct platform_device *dev, pm_message_t state)
{
        g_wake = 10;
        return 0;
}
int ad_button_resume(struct platform_device *dev)
{
        return 0;
}
static struct platform_driver rk28_AD_button_driver = {
	.probe		= rk28_AD_button_probe,
	.remove 		= __devexit_p(rk28_AD_button_remove),
	.suspend		= ad_button_suspend,
	.resume 		= ad_button_resume,
	.driver 		= {
		.name	= "rk28_key_button",
		.owner	= THIS_MODULE,
	},
};

 int __init rk28_AD_button_init(void)
{
	return platform_driver_register(&rk28_AD_button_driver);
}

static void __exit rk28_AD_button_exit(void)
{
	platform_driver_unregister(&rk28_AD_button_driver);
}

fs_initcall(rk28_AD_button_init);
module_exit(rk28_AD_button_exit);

MODULE_DESCRIPTION("rk28 AD button Controller Driver");
MODULE_LICENSE("GPL");
