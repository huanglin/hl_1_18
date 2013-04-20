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

/* ************
 *	  DEBUG
 **************/

#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif
#define SHOWME	"Keyboard"
//#define S_LEVEL  S_L_WARN
#define S_LEVEL  S_L_ERROR
#include <asm/arch/rk28_debug.h>


#define WAKEUP_KEY_PORT PLAY_ON_IOPIN
//ROCKCHIP AD KEY CODE ,for demo board
//      key		--->	EV	

#define AD1KEY1		103//DPAD_UP	----------UP
#define AD1KEY2 		106//DPAD_RIGHT-------FFD
#define AD1KEY3 		105//DPAD_LEFT--------FFW	
#define AD1KEY4 		108//DPAD_DOWN------DOWN		#define AD1KEY5		158//62   //ENDCALL           WAKE_DROPPED
#define AD1KEY5		ENDCALL       //    WAKE_DROPPED
#define AD1KEY6		ENDCALL        //   WAKE_DROPPED

#define AD2KEY1		114   //VOLUME_DOWN  	 	59	//MENU			//115   //VOLUME_UP
#define AD2KEY2 		115   //VOLUME_UP      		//114   //VOLUME_DOWN
#define AD2KEY3 		59	//MENU	
#define AD2KEY4 		102   //HOME				//62   //ENDCALL  		
#define AD2KEY5 		158	//BACK----ESC   		115   //VOLUME_UP	//158	//BACK------------ESC
#define AD2KEY6 		116	//POWER

#define Valuedrift		70
#define EmptyADValue			950  //1000
#define InvalidADValue	10
#define ADKEYNum		12

/*power event button*/
#define  POWER				116
#define  ENDCALL				62
#define  ONESEC_TIMES		100
#define  SEC_NUM			1
#define  SLEEP_TIME			2	/*per 40ms*/
static unsigned  int	pwrscantimes = 0;
static unsigned  int valuecount = 0;
static unsigned  int g_code = 0;
static unsigned  int g_wake =0;

static unsigned int 	encall_times = 100;

#define KEY_PHYS_NAME	"rk28_AD_button/input0"
//key code tab
static unsigned char initkey_code[ ] = 
{
	AD1KEY1, AD1KEY2,AD1KEY3,AD1KEY4,AD1KEY5,AD1KEY6,
	AD2KEY1, AD2KEY2,AD2KEY3,AD2KEY4,AD2KEY5,AD2KEY6,ENDCALL	,KEY_WAKEUP
};


struct rk28_AD_button {

	struct timer_list timer;

	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;
	/* matrix key code map */
	unsigned char keycodes[13];
	/* state row bits of each column scan */
	uint32_t direct_key_state;
	unsigned int direct_key_mask;
	unsigned int keyadvalue;
	int rotary_rel_code[2];
	int rotary_up_key[2];
	int rotary_down_key[2];
};

 struct rk28_AD_button *prockAD_button;
extern 	void  RockAdcScanning(void);
extern 	int32 ADCInit(void);
extern 	int get_rock_adc1(void);

static ssize_t adkey_debug_store(struct device *dev,struct device_attribute *attr,char *_buf,size_t count)
{
	struct rk28_AD_button *AD_button = dev_get_drvdata( dev);
	count  -= 1;
	if(strncmp(_buf, "debug",count)== 0)
	{
		printk("%s->open debug\n",__FUNCTION__);
	}
	else if (strncmp(_buf, "closedebug",count)== 0)
	{
		printk("%s->close debug\n",__FUNCTION__);
	}
	else
		printk("%s-->%d no cmd parameter",__FUNCTION__,__LINE__);
	
	printk("ad sample value show: ADKEY value==%d\n", AD_button->keyadvalue);
	return sprintf(_buf, " adkey debug successful\n");
}

static  DEVICE_ATTR(keyboarddebug, 0666, NULL, adkey_debug_store);

static  int ADSampleTimes = 0;
/* only send wakeup when at deep sleep. */
void rk28_send_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() != PM_AWAKE ){	
		printk("%s:line%d\n",__FUNCTION__,__LINE__);
	  	rk2818_set_suspend_flags(PM_AWAKE);
	  	//rk2818_set_suspend_flags(PM_ONELEVEL_SLEEP);
		input_report_key(prockAD_button->input_dev,ENDCALL,1);
		input_sync(prockAD_button->input_dev);
		input_report_key(prockAD_button->input_dev,ENDCALL,0);
		input_sync(prockAD_button->input_dev);
		printk("\n%s::send key:%d ^^^^^!!\n",__FUNCTION__, ENDCALL);
		//mdelay(20);
	  	//rk2818_set_suspend_flags(PM_AWAKE);
	}
}
#if 1  /* HSL@RK,only for test purpose. */
void rk28_wakeup_force( void ) 
{
        input_report_key(prockAD_button->input_dev,ENDCALL,1);
        input_sync(prockAD_button->input_dev);
        input_report_key(prockAD_button->input_dev,ENDCALL,0);
        input_sync(prockAD_button->input_dev);
        //printk("\n%s::send key:%d ^^^^^!!\n",__FUNCTION__, ENDCALL);
}
#endif
static void rk28_adkeyscan_timer(unsigned long data)
{
	unsigned int ADKEY1;
	/*Enable  AD controller to sample */
	prockAD_button->timer.expires  = jiffies+msecs_to_jiffies(10);
	add_timer(&prockAD_button->timer);
	RockAdcScanning();
	if (ADSampleTimes < 4)
	{
		ADSampleTimes ++;		
		goto scan_io_key;  	/* scan gpio button event*/
	}
	ADSampleTimes = 0;	
	ADKEY1=get_rock_adc1();			/*Get button value*/
	prockAD_button->keyadvalue = ADKEY1;
	S_INFO("adc1 value = %d \n",ADKEY1);
	if(ADKEY1>=500)
	{
	        valuecount+=1;						  
		if(valuecount>SLEEP_TIME)
		{
			if(g_code == 1) return;
			valuecount = 0;
			g_code = 1;
			if(rk2818_get_suspend_flags() == PM_AWAKE){
				input_report_key(prockAD_button->input_dev,ENDCALL,1);
				input_sync(prockAD_button->input_dev);
				input_report_key(prockAD_button->input_dev,ENDCALL,0);
				input_sync(prockAD_button->input_dev);
				printk("\n%s^^^Hold Key ^^^^^!!\n",__FUNCTION__);
			}
			gpio_irq_disable(TOUCH_INT_IOPIN);
		}
	} else 
	{
	     	if(g_code == 1) gpio_irq_enable(TOUCH_INT_IOPIN);
	     	g_code = 0;
		valuecount = 0;	
	}
		
scan_io_key :

	if(GPIOGetPinLevel(PLAY_ON_IOPIN))

	{
		pwrscantimes += 1;
		if(pwrscantimes == (SEC_NUM * ONESEC_TIMES))
		{
			input_report_key(prockAD_button->input_dev,ENDCALL,1);
			input_sync(prockAD_button->input_dev);
			printk("the kernel come to power down!!!\n");
		}
		if(pwrscantimes ==( (SEC_NUM + 1)* ONESEC_TIMES))
		{
			pwrscantimes = 0;
			input_report_key(prockAD_button->input_dev,ENDCALL,0);
			input_sync(prockAD_button->input_dev);
			printk("the kernel come to power up!!!\n");
			encall_times = 100;
		
		}
		return ;
	}
	else
	{
		if(encall_times > 0)
			encall_times--;
	}
	if( pwrscantimes > SLEEP_TIME )
	{
		pwrscantimes = 0;
		if(rk2818_get_suspend_flags() !=  PM_AWAKE)
		{
			
			if(rk2818_get_suspend_flags() == PM_TWOLEVEL_SLEEP)	/*already wake up*/
			{
				printk("\n%s^^Wake Up from key IRQ^^!!\n",__FUNCTION__);
				return;
			}
			if(encall_times == 0) {
			input_report_key(prockAD_button->input_dev,158,1);
			input_sync(prockAD_button->input_dev);
			input_report_key(prockAD_button->input_dev,158,0);
			input_sync(prockAD_button->input_dev);
			printk("\n%s^^Wake UP SYSTEM !^^!!\n",__FUNCTION__);
			}
		}
	}
	
}


static irqreturn_t rk28_AD_irq_handler(s32 irq, void *dev_id)
{
    //dump_stack();   /* HSL@RK,only for test purpers. */
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
	int  error,i;
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

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed2;
	}
	setup_timer(&AD_button->timer, rk28_adkeyscan_timer, (unsigned long)AD_button);
	AD_button->timer.expires  = jiffies + 3;
	add_timer(&AD_button->timer);
	error = request_gpio_irq(WAKEUP_KEY_PORT,rk28_AD_irq_handler,GPIOEdgelRising,NULL);
	if(error)
	{
		printk("unable to request playon key IRQ\n");
		goto failed2;
	}
	error = device_create_file(&pdev->dev,&dev_attr_keyboarddebug);
	if(error)
		printk("%s->%d cannot create status attribute\n",__FUNCTION__,__LINE__);
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
        //g_wake = 0;
        return 0;
}
static struct platform_driver rk28_AD_button_driver = {
	.probe		= rk28_AD_button_probe,
	.remove 	= __devexit_p(rk28_AD_button_remove),
	.suspend = ad_button_suspend,
	.resume = ad_button_resume,
	.driver 	= {
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
