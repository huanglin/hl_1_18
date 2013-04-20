/*
 * linux/drivers/input/keyboard/rk28_gpio_button.c
 *
 * Driver for the rk28 matrix keyboard controller.
 *
 * Created: 2012-03-10
 * Author:	XJQ <xjq@rockchip.com>
 *
 * This driver program support to matrix keyboard which use for rk28 chip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//#define MATRIX_DBG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/typedef.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_debug.h>


#ifdef MATRIX_DBG
#define printk_d(fmt, args...)	printk(KERN_INFO "[MATRIX KEYBOARD] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define printk_d(fmt, args...)
#endif

#ifdef CONFIG_KEYBOARD_MATRIX
#define KROW0_IOPIN 					GPIOPortF_Pin5
#define KROW0_IOMUX_PINNAME 		GPIOF5_APWM3_DPWM3_NAME 
#define KROW0_IOMUX_PINDIR			IOMUXB_GPIO1_B5

#define KROW1_IOPIN 					GPIOPortF_Pin2
#define KROW1_IOMUX_PINNAME 		GPIOF2_APWM0_SEL_NAME 
#define KROW1_IOMUX_PINDIR			IOMUXB_GPIO1_B2

#define KROW2_IOPIN 					GPIOPortB_Pin1
#define KROW2_IOMUX_PINNAME 		GPIOB1_SMCS1_MMC0PCA_NAME 
#define KROW2_IOMUX_PINDIR			IOMUXA_GPIO0_B1

#define KROW3_IOPIN 					GPIOPortG_Pin6
#define KROW3_IOMUX_PINNAME 		GPIOG_MMC1D_SEL_NAME 
#define KROW3_IOMUX_PINDIR			IOMUXA_GPIO1_C456

#define KROW4_IOPIN 					GPIOPortA_Pin2
#define KROW4_IOMUX_PINNAME 		NO_IOMUX_PINNAME 
#define KROW4_IOMUX_PINDIR			NO_IO_MUX

#define KCOL0_IOPIN 						GPIOPortA_Pin1
#define KCOL0_IOMUX_PINNAME 			GPIOA1_HOSTDATA17_SEL_NAME 
#define KCOL0_IOMUX_PINDIR			IOMUXB_GPIO0_A1

#define KCOL1_IOPIN 						GPIOPortA_Pin0
#define KCOL1_IOMUX_PINNAME 			GPIOA0_HOSTDATA16_SEL_NAME 
#define KCOL1_IOMUX_PINDIR			IOMUXB_GPIO0_A0
#endif

#define KEY_PHYS_NAME	"rk28_matrix_button/input0"
#define ONESEC		(100)
#define NUMSEC		(1)
#define SLEEPTIME	(2)
static unsigned  int g_io_key_count = 0;
static unsigned  int g_io_key_press = 0;
static unsigned  int g_io_key_state = 0;
static unsigned  int gMatrixSuspend = 0;

struct rk28_matrix_gpiokey_platform_data {
	int x;
};

struct rk28_matrix_gpiokey {
	struct rk28_matrix_gpiokey_platform_data *pdata;
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

struct rk28_matrix_gpiokey *gMatrixKey;
static struct timer_list pwrkey_timer;
static struct work_struct	pwrkey_task;

 static unsigned char initkey_code[ ] = {
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UP,
	KEY_DOWN,	
	KEY_BACK,	
	KEY_ENTER,
	KEY_HOME,
	KEY_WAKEUP,
	KEY_POWER,
	KEY_RESERVED
};
 
static void rk28_matrix_callback(unsigned long arg)
{
	pwrkey_timer.expires = jiffies + msecs_to_jiffies(10);
	add_timer(&pwrkey_timer);
	schedule_work(&pwrkey_task);
}

static void rk28_matrix_init_timer(void)
{
	init_timer(&pwrkey_timer);
	pwrkey_timer.function = rk28_matrix_callback;
	pwrkey_timer.expires = jiffies + msecs_to_jiffies(10);
	add_timer(&pwrkey_timer);
}
void rk28_send_rtc_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() == PM_AWAKE )
		return;
	
	printk("%s:line%d\n",__FUNCTION__,__LINE__);
  	rk2818_set_suspend_flags(PM_AWAKE);
	input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,1);
	input_sync(gMatrixKey->input_dev);
	input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,0);
	input_sync(gMatrixKey->input_dev);
}
void rk28_send_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() != PM_AWAKE ){
		printk("%s:line%d\n",__FUNCTION__,__LINE__);
	  	rk2818_set_suspend_flags(PM_AWAKE);
		input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,1);
		input_sync(gMatrixKey->input_dev);
		input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,0);
		input_sync(gMatrixKey->input_dev);
	}
}

static irqreturn_t rk28_wakeup_handler(s32 irq, void *dev_id)
{
	printk_d("\n");
	if(rk2818_get_suspend_flags() == PM_TWOLEVEL_SLEEP) 
	{
		rk28_send_wakeup_key();
	}
	return IRQ_HANDLED;
}


#define MAX_GPIO_OUT_CNT	5
#define MAX_GPIO_IN_CNT	2

static int gpio_out[MAX_GPIO_OUT_CNT] =
{
	KROW0_IOPIN,//GPIOPortF_Pin5,	//GPIOPortG_Pin2,
	KROW1_IOPIN,//GPIOPortF_Pin2,	//GPIOPortG_Pin3,
	KROW2_IOPIN,//GPIOPortB_Pin1,	//GPIOPortG_Pin4,
	KROW3_IOPIN,//GPIOPortG_Pin6,	//GPIOPortG_Pin5,
	KROW4_IOPIN,//GPIOPortA_Pin2,	//GPIOPortG_Pin6,
};

static int gpio_in[MAX_GPIO_IN_CNT] =
{
	KCOL0_IOPIN,//GPIOPortA_Pin1,	//GPIOPortA_Pin2,
	KCOL1_IOPIN,//GPIOPortA_Pin0,	//GPIOPortA_Pin3,
};

static void key_in_gpio_init(void)
{
	int	i;
	
	rockchip_mux_api_set(KROW0_IOMUX_PINNAME,KROW0_IOMUX_PINDIR);
	rockchip_mux_api_set(KROW1_IOMUX_PINNAME,KROW0_IOMUX_PINDIR);
	rockchip_mux_api_set(KROW2_IOMUX_PINNAME,KROW0_IOMUX_PINDIR);
	rockchip_mux_api_set(KROW3_IOMUX_PINNAME,KROW0_IOMUX_PINDIR);
	rockchip_mux_api_set(KROW4_IOMUX_PINNAME,KROW0_IOMUX_PINDIR);	
	rockchip_mux_api_set(KCOL0_IOMUX_PINNAME,KCOL0_IOMUX_PINDIR);
	rockchip_mux_api_set(KCOL1_IOMUX_PINNAME,KCOL1_IOMUX_PINDIR);

	for(i = 0; i <MAX_GPIO_IN_CNT; i++) {
		GPIOSetPinLevel(gpio_in[i], GPIO_HIGH);
		GPIOSetPinDirection(gpio_in[i], GPIO_IN);
		GPIOPullUpDown(gpio_in[i],GPIONormal);
	}

	for(i = 0; i < MAX_GPIO_OUT_CNT; i++) {
		GPIOSetPinLevel(gpio_out[i], GPIO_HIGH);
		GPIOSetPinDirection(gpio_out[i], GPIO_OUT);
		GPIOPullUpDown(gpio_out[i],GPIONormal);
	}

	return;
}

static void tcc_gpiokey_poll_callback(void);

static void rk28_matrix_scan(struct work_struct *work) 
{
	if (!gMatrixSuspend)
		tcc_gpiokey_poll_callback();
	if (!GPIOGetPinLevel(PLAY_ON_IOPIN)) {
		if (g_io_key_state == 0) {
			g_io_key_state = 1;
			g_io_key_count = 0;
		}
	} else {
		if (g_io_key_state) {
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

	input_report_key(gMatrixKey->input_dev, KEY_POWER, g_io_key_press);
	input_sync(gMatrixKey->input_dev);
	printk_d("send keycode[%d] state = %d!\n", KEY_POWER, g_io_key_press);
}


enum {
	KEY_STATE_OFF = 0,
	KEY_STATE_ON
};
#define MATRIX_OUT_TURNON		(0)
#define MATRIX_OUT_TURNOFF		(1)
#define MATRIX_IN_ON			MATRIX_OUT_TURNON
#define KEY_REPORT_INTERVAL	msecs_to_jiffies(400)

struct gpio_button {
	int gpio;
	int key_code;
	int gpio_out;
};

static	int	key_of_old_gpio = 0;
static	int	key_of_old_key = 0;
static	int	key_of_old_gpio_out = 0;
static	int	key_of_debounce = 0;
static	int	last_key_time;
static	int	touch_key_scan_flag =0 ;
static	int	gpio_callback_times = 0;


static int gpio_key_code[MAX_GPIO_OUT_CNT][MAX_GPIO_IN_CNT] =
{
	{ KEY_BACK,  KEY_HOME },
	{ KEY_BACK,  KEY_MENU },
	{ KEY_LEFT,  KEY_DOWN },
	{ KEY_RIGHT,  KEY_ENTER },
	{ KEY_RESERVED, KEY_UP },
};

static inline int tcc_gpio_ison(int gpio)
{
	int val = GPIOGetPinLevel(gpio);

	return (MATRIX_IN_ON == val) ? KEY_STATE_ON : KEY_STATE_OFF;
}

static void tcc_gpiokey_poll_callback(void)
{
	int i,j;
	int value;	
	struct gpio_button v;

	gpio_callback_times++;
	if(gpio_callback_times<4){
		return;
	} else {
		gpio_callback_times = 0;
	}

	key_in_gpio_init();

	if(touch_key_scan_flag)	{	
		GPIOSetPinLevel(key_of_old_gpio_out, MATRIX_OUT_TURNON);
		value = tcc_gpio_ison(key_of_old_gpio);
		value = tcc_gpio_ison(key_of_old_gpio);
		value = tcc_gpio_ison(key_of_old_gpio);
		
		if(key_of_debounce)	{
			if(KEY_STATE_OFF == value)	{	//released, active low.
				touch_key_scan_flag = 0;
				key_of_debounce = 0;				
			} else {
				if(jiffies - last_key_time > KEY_REPORT_INTERVAL)	{
					key_of_debounce = 0;
					last_key_time = jiffies;
					input_report_key(gMatrixKey->input_dev,key_of_old_key,1);
					input_sync(gMatrixKey->input_dev);
					printk_d("keycode [%d] pressed\n", key_of_old_key);
				}
			}
		} else {
			if(KEY_STATE_OFF == value) {	//released, active low.
				last_key_time = jiffies;
				input_report_key(gMatrixKey->input_dev,key_of_old_key,0);
				input_sync(gMatrixKey->input_dev);
				printk_d("keycode [%d] released\n", key_of_old_key);
				touch_key_scan_flag = 0;
			} else {
			}
		}
	}else{
		int gpio_hasOneKeyPressed = 0;

		// first  turn off all output gpios 
		for (i=0;i<MAX_GPIO_OUT_CNT;i++) {
			GPIOSetPinLevel(gpio_out[i], MATRIX_OUT_TURNOFF);
		}
		
		// then turn on one by one & turn off the pre one
		for (i=0; i<MAX_GPIO_OUT_CNT; i++) {
			// turn on
			{ 					
				GPIOSetPinLevel(gpio_out[i], MATRIX_OUT_TURNON);
				//value = tcc_gpio_ison(gpio_in[0]);		// just for delay
				value = tcc_gpio_ison(gpio_in[0]);
				value = tcc_gpio_ison(gpio_in[0]);
				value = tcc_gpio_ison(gpio_in[0]);
				value = tcc_gpio_ison(gpio_in[0]);
				value = tcc_gpio_ison(gpio_in[0]);
			}

			for (j=0; j<MAX_GPIO_IN_CNT;j++) {
				value = tcc_gpio_ison(gpio_in[j]);
				if(KEY_STATE_ON == value){//pressed.
					gpio_hasOneKeyPressed = 1;
					v.gpio_out = gpio_out[i];
					v.gpio= gpio_in[j];
					v.key_code = gpio_key_code[i][j];
					printk_d("Find key[%d][%d]\n",i,j );
					break;
				}
			}
			
			if (!gpio_hasOneKeyPressed) {  // turn off
				GPIOSetPinLevel(gpio_out[i], MATRIX_OUT_TURNOFF);
			}else{
				break;
			}
		}

		if(gpio_hasOneKeyPressed) {
			key_of_old_gpio = v.gpio;
			key_of_old_key = v.key_code;
			key_of_old_gpio_out = v.gpio_out;
			
			key_of_debounce = 1;
			touch_key_scan_flag = 1;			
			if( (jiffies - last_key_time) > (KEY_REPORT_INTERVAL - msecs_to_jiffies(40)) ) {
				last_key_time = jiffies -KEY_REPORT_INTERVAL + msecs_to_jiffies(40);
			}
		}		
	}
	return;
}

static int rk28_matrix_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	
	printk_d("\n");
	gMatrixSuspend = 1;
	for (i=0; i<MAX_GPIO_OUT_CNT; i++) {
		GPIOPullUpDown(gpio_out[i], GPIONormal);
		GPIOSetPinDirection(gpio_out[i], GPIO_OUT);
		GPIOSetPinLevel(gpio_out[i], GPIO_LOW);
	}
	return 0;
}

static int rk28_matrix_resume(struct platform_device *pdev)
{		
	printk_d("\n");
	gMatrixSuspend = 0;
	return 0;
}

static int __devinit rk28_matrix_probe(struct platform_device *pdev)
{
	struct rk28_matrix_gpiokey *matrix_gpiokey;
	struct input_dev *input_dev;
	int  error,i;
	
	matrix_gpiokey = kzalloc(sizeof(struct rk28_matrix_gpiokey), GFP_KERNEL);
	
	/* Create and register the input driver. */
	input_dev = input_allocate_device();
	if (!input_dev || !matrix_gpiokey) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto failed1;
	}

	memcpy(matrix_gpiokey->keycodes, initkey_code, sizeof(matrix_gpiokey->keycodes));
	input_dev->name = pdev->name;
	input_dev->dev.parent = &pdev->dev;
	input_dev->phys = KEY_PHYS_NAME;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->keycode = matrix_gpiokey->keycodes;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(initkey_code);
	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)	{
		set_bit(initkey_code[i], input_dev->keybit);
	}
	clear_bit(0, input_dev->keybit);
	matrix_gpiokey->input_dev = input_dev;
	input_set_drvdata(input_dev, matrix_gpiokey);
	input_dev->evbit[0] = BIT_MASK(EV_KEY) ;
	platform_set_drvdata(pdev, matrix_gpiokey);
	
	gMatrixKey = matrix_gpiokey;
	
	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed2;
	}
	error = request_gpio_irq(PLAY_ON_IOPIN,(pFunc)rk28_wakeup_handler,GPIOEdgelFalling,NULL);
	if(error) {
		printk("unable to request playon key IRQ\n");
		goto failed2;
	}
	error = request_gpio_irq(VBUS_INT_IOPIN,(pFunc)rk28_wakeup_handler,GPIOEdgelRising,NULL);
	if(error) {
		printk("unable to request vbus IRQ\n");
		goto failed2;
	}
	for(i = 0; i <MAX_GPIO_IN_CNT; i++) {
		error = request_gpio_irq(gpio_in[i],(pFunc)rk28_wakeup_handler,GPIOEdgelFalling,NULL);
		if(error) {
			printk("unable to request gpio_in[%d] IRQ\n",i);
			return error;
		}
	}

	INIT_WORK(&pwrkey_task, rk28_matrix_scan);
	rk28_matrix_init_timer();

	return 0;
failed2:
	input_unregister_device(matrix_gpiokey->input_dev);
	platform_set_drvdata(pdev, NULL);	
failed1:
	input_free_device(input_dev);
	kfree(matrix_gpiokey);
	return error;
}

static int __devexit rk28_matrix_remove(struct platform_device *pdev)
{
	struct rk28_matrix_gpiokey *matrix_gpiokey = platform_get_drvdata(pdev);

	input_unregister_device(matrix_gpiokey->input_dev);
	input_free_device(matrix_gpiokey->input_dev);
	kfree(matrix_gpiokey);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver rk28_matrix_driver = {
	.probe		= rk28_matrix_probe,
	.remove 		= __devexit_p(rk28_matrix_remove),
	.suspend_late  	= rk28_matrix_suspend,
	.resume_early 	= rk28_matrix_resume,
	.driver 		= {
		.name	= "rk28_key_button",
		.owner	= THIS_MODULE,
	},
};

int __init rk28_matrix_init(void)
{
	return platform_driver_register(&rk28_matrix_driver);
}

static void __exit rk28_matrix_exit(void)
{
	platform_driver_unregister(&rk28_matrix_driver);
}

module_init(rk28_matrix_init);
module_exit(rk28_matrix_exit);

MODULE_AUTHOR("xjq <xjq@rock-chips.com>");
MODULE_DESCRIPTION("rk28 matrix keyboard Controller Driver");
MODULE_LICENSE("GPL");
