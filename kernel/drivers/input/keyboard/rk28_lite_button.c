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


#define KROW0_IOPIN 					GPIOPortG_Pin7
#define KROW0_IOMUX_PINNAME 			GPIOG_MMC1_SEL_NAME 
#define KROW0_IOMUX_PINDIR			0

#define KROW1_IOPIN 					GPIOPortG_Pin5
#define KROW1_IOMUX_PINNAME 			GPIOG_MMC1D_SEL_NAME 
#define KROW1_IOMUX_PINDIR			0

#define KROW2_IOPIN 					GPIOPortG_Pin3
#define KROW2_IOMUX_PINNAME 			GPIOG_MMC1_SEL_NAME 
#define KROW2_IOMUX_PINDIR			0


#define KCOL0_IOPIN 						GPIOPortE_Pin3
#define KCOL0_IOMUX_PINNAME 			GPIOE_SPI1_FLASH_SEL2_NAME 
#define KCOL0_IOMUX_PINDIR				0

#define KCOL1_IOPIN 						GPIOPortE_Pin0
#define KCOL1_IOMUX_PINNAME 			GPIOE0_VIPDATA0_SEL_NAME 
#define KCOL1_IOMUX_PINDIR				0

#define KCOL2_IOPIN 						GPIOPortA_Pin2
#define KCOL2_IOMUX_PINNAME 			GPIOA23_UART2_SEL_NAME 
#define KCOL2_IOMUX_PINDIR				0

#define KEY_PHYS_NAME	"rk28_matrix_button/input0"
#define ONESEC		(100)
#define NUMSEC		(1)
#define SLEEPTIME	(2)


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
 static unsigned int pwrScanTimes = 0;
 static unsigned int endCallTimes = 100;
static struct work_struct  scan_page_up_task;
static struct work_struct  scan_page_down_task;
static struct work_struct  scan_vol_up_task;
static struct work_struct  scan_vol_down_task;
 static unsigned char initkey_code[ ] = {
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UP,
	KEY_DOWN,	
	KEY_BACK,	
	KEY_ENTER,
	KEY_HOME,
	KEY_SEARCH,
	KEY_RIGHTBRACE,
	KEY_LEFTBRACE,
	KEY_VOLUMEUP,
	KEY_VOLUMEDOWN,
	KEY_WAKEUP,
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


void rk28_send_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() != PM_AWAKE ){
	  	rk2818_set_suspend_flags(PM_AWAKE);
		input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,1);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] down\n",KEY_WAKEUP);
		input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,0);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] up\n",KEY_WAKEUP);
	}
}
void rk28_send_rtc_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() != PM_AWAKE ){
		printk_d("\n");
		rk2818_set_suspend_flags(PM_AWAKE);
		input_report_key(gMatrixKey->input_dev,KEY_RTC_WAKEUP,1);
		input_sync(gMatrixKey->input_dev);
		input_report_key(gMatrixKey->input_dev,KEY_RTC_WAKEUP,0);
		input_sync(gMatrixKey->input_dev);
	}
}

static irqreturn_t rk28_playon_handler(s32 irq, void *dev_id)
{
	if(rk2818_get_suspend_flags() == PM_TWOLEVEL_SLEEP) {
		rk28_send_wakeup_key();
	}
	return IRQ_HANDLED;
}

#define MAX_GPIO_OUT_CNT	3
#define MAX_GPIO_IN_CNT	3
#define MAX_GPIO_MUX           6
static int gpio_out[MAX_GPIO_OUT_CNT] =
{
	KROW0_IOPIN,
	KROW1_IOPIN,
	KROW2_IOPIN,
	
};

static int gpio_in[MAX_GPIO_IN_CNT] =
{
	KCOL0_IOPIN,
	KCOL1_IOPIN,
	KCOL2_IOPIN,
};


static int rk28_matrix_intr_request(void)
{
	int i;
	int error =0;
	
	//rockchip_mux_api_set(KCOL0_IOMUX_PINNAME,KCOL0_IOMUX_PINDIR);
	//rockchip_mux_api_set(KCOL1_IOMUX_PINNAME,KCOL1_IOMUX_PINDIR);

	for(i = 0; i <MAX_GPIO_IN_CNT; i++) {
		error = request_gpio_irq(gpio_in[i],(pFunc)rk28_playon_handler,GPIOEdgelFalling,NULL);
		if(error) {
			printk("unable to request gpio_in[%d] irq\n",i);
			goto fail;
		}
	}
fail:
	return error;
}

static void key_in_gpio_init(void)
{
	int	i;

	rockchip_mux_api_set(KROW0_IOMUX_PINNAME,0);
	rockchip_mux_api_set(KROW1_IOMUX_PINNAME,0);
	rockchip_mux_api_set(KROW2_IOMUX_PINNAME,0);
	rockchip_mux_api_set(KCOL0_IOMUX_PINNAME,0);
	rockchip_mux_api_set(KCOL1_IOMUX_PINNAME,0);
	rockchip_mux_api_set(KCOL2_IOMUX_PINNAME,0);
	
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
	tcc_gpiokey_poll_callback();

	if(!GPIOGetPinLevel(PLAY_ON_IOPIN)) {
		pwrScanTimes ++;
		if(pwrScanTimes == (NUMSEC*ONESEC)) {//surpport long press down
			input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,1);
			input_sync(gMatrixKey->input_dev);
			printk_d("keycode[%d] down\n",KEY_WAKEUP);
			endCallTimes = 100;
		}
		if(pwrScanTimes == ((NUMSEC+1)*ONESEC)) {//idendify 1s as long press
			input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,0);
			input_sync(gMatrixKey->input_dev);
			printk_d("keycode[%d] up\n",KEY_WAKEUP);
			endCallTimes = 100;
			pwrScanTimes = 0;
		}
		return ;
	} else {
		if(endCallTimes > 0)
			endCallTimes--;//delay for decrease jitter
	}
	if( pwrScanTimes > SLEEPTIME ) {
		pwrScanTimes = 0;
		if(rk2818_get_suspend_flags() !=  PM_AWAKE) {
			if(rk2818_get_suspend_flags() == PM_TWOLEVEL_SLEEP) {
				printk_d("Wake Up\n");
				return;
			}
			if(endCallTimes == 0) {
				input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,1);
				input_sync(gMatrixKey->input_dev);
				input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,0);
				input_sync(gMatrixKey->input_dev);
				printk_d("system to be awake\n");
			}
		} else {
			if(endCallTimes == 0) {
				input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,1);
				input_sync(gMatrixKey->input_dev);
				input_report_key(gMatrixKey->input_dev,KEY_WAKEUP,0);
				input_sync(gMatrixKey->input_dev);
				printk_d("system to be sleep\n");
			}
		}
	}
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
	{ KEY_BACK,  KEY_HOME,KEY_DOWN },
	{ KEY_UP,  KEY_MENU,KEY_ENTER },
	{ KEY_SEARCH,  KEY_LEFT,KEY_RIGHT },
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
	if(gpio_callback_times<4)	{
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
			/*
			if(!suspend_finished){
				printk("%s: key press during suspend\n", __func__);
				touch_key_scan_flag_during_suspend = 1;
			}
			*/
			key_of_old_gpio = v.gpio;
			key_of_old_key = v.key_code;
			key_of_old_gpio_out = v.gpio_out;
			
			key_of_debounce = 1;
			touch_key_scan_flag = 1;			

			#if	1
				//
				if( (jiffies - last_key_time) > (KEY_REPORT_INTERVAL - msecs_to_jiffies(40))  )	{
					last_key_time = jiffies -KEY_REPORT_INTERVAL + msecs_to_jiffies(40);
				}
				
			#else
			
			if(jiffies - last_key_time > KEY_REPORT_INTERVAL)
			{
				last_key_time = jiffies;
				last_key_state = 1;
				input_report_key(tcc_private->poll_dev->input, tcc_private->old_key, 1);
				input_report_key(tcc_private->poll_dev->input, tcc_private->old_key, 0);
				input_sync(tcc_private->poll_dev->input);
				last_key_state = !last_key_state;
				printk("%s: 3 new key %d pressed\n", __func__, tcc_private->old_key);
			}
			#endif
		}		
	}

	if(touch_key_scan_flag)	{
		return;
	} else {
		//touch_key(dev);
		return;
	}
	
}
static irqreturn_t rk28_key_down_irq_handler(s32 irq, void *dev_id)
{
	schedule_work(&scan_page_down_task);
	return IRQ_HANDLED;	
}

static irqreturn_t rk28_key_up_irq_handler(s32 irq, void *dev_id)
{
	schedule_work(&scan_page_up_task);
	return IRQ_HANDLED;
}
static irqreturn_t rk28_key_vol_up_irq_handler(s32 irq, void *dev_id)
{
	schedule_work(&scan_vol_up_task);
	return IRQ_HANDLED;	
}

static irqreturn_t rk28_key_vol_down_irq_handler(s32 irq, void *dev_id)
{
	schedule_work(&scan_vol_down_task);
	return IRQ_HANDLED;
}
static void page_up_key_handle(struct work_struct *work)
{
	if(GPIOGetPinLevel(PAGE_UP_IOPIN)){
		request_gpio_irq(PAGE_UP_IOPIN,(pFunc)rk28_key_up_irq_handler,GPIOEdgelFalling,NULL);
		input_report_key(gMatrixKey->input_dev,KEY_LEFTBRACE,0);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] up\n",KEY_LEFTBRACE);
	} else{
		request_gpio_irq(PAGE_UP_IOPIN,(pFunc)rk28_key_up_irq_handler,GPIOEdgelRising,NULL);
		input_report_key(gMatrixKey->input_dev,KEY_LEFTBRACE,1);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] down\n",KEY_LEFTBRACE);
	}
}

static void page_down_key_handle(struct work_struct *work)
{
	if(GPIOGetPinLevel(PAGE_DOWN_IOPIN)){
		request_gpio_irq(PAGE_DOWN_IOPIN,(pFunc)rk28_key_down_irq_handler,GPIOEdgelFalling,NULL);
		input_report_key(gMatrixKey->input_dev,KEY_RIGHTBRACE,0);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] up\n",KEY_RIGHTBRACE);
	} else{
		request_gpio_irq(PAGE_DOWN_IOPIN,(pFunc)rk28_key_down_irq_handler,GPIOEdgelRising,NULL);
		input_report_key(gMatrixKey->input_dev,KEY_RIGHTBRACE,1);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] down\n",KEY_RIGHTBRACE);
	}
}
static void vol_up_key_handle(struct work_struct *work)
{
	if(GPIOGetPinLevel(VOL_UP_IOPIN)){
		request_gpio_irq(VOL_UP_IOPIN,(pFunc)rk28_key_vol_up_irq_handler,GPIOEdgelFalling,NULL);
		input_report_key(gMatrixKey->input_dev,KEY_VOLUMEUP,0);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] up\n",KEY_VOLUMEUP);
	} else{
		request_gpio_irq(VOL_UP_IOPIN,(pFunc)rk28_key_vol_up_irq_handler,GPIOEdgelRising,NULL);
		input_report_key(gMatrixKey->input_dev,KEY_VOLUMEUP,1);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] down\n",KEY_VOLUMEUP);
	}
}

static void vol_down_key_handle(struct work_struct *work)
{
	if(GPIOGetPinLevel(VOL_DOWN_IOPIN)){
		request_gpio_irq(VOL_DOWN_IOPIN,(pFunc)rk28_key_vol_down_irq_handler,GPIOEdgelFalling,NULL);
		input_report_key(gMatrixKey->input_dev,KEY_VOLUMEDOWN,0);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] up\n",KEY_VOLUMEDOWN);
	} else{
		request_gpio_irq(VOL_DOWN_IOPIN,(pFunc)rk28_key_vol_down_irq_handler,GPIOEdgelRising,NULL);
		input_report_key(gMatrixKey->input_dev,KEY_VOLUMEDOWN,1);
		input_sync(gMatrixKey->input_dev);
		printk_d("keycode[%d] down\n",KEY_VOLUMEDOWN);
	}
}
static void key_signal_io_init_hw(void)
{
	int ret = 0;

	rockchip_mux_api_set(GPIOA0_HOSTDATA16_SEL_NAME, PAGE_DOWN_IOMUX_PINDIR);   
	GPIOPullUpDown(PAGE_DOWN_IOPIN,GPIOPullUp);
	GPIOSetPinDirection(PAGE_DOWN_IOPIN, GPIO_IN);

	rockchip_mux_api_set(GPIOA1_HOSTDATA17_SEL_NAME, IOMUXB_GPIO0_A1);  
	GPIOPullUpDown(PAGE_UP_IOPIN,GPIOPullUp);
	GPIOSetPinDirection(PAGE_UP_IOPIN, GPIO_IN);

	rockchip_mux_api_set(GPIOE_I2C0_SEL_NAME, IOMUXA_GPIO1_A45);   
	GPIOPullUpDown(VOL_UP_IOPIN,GPIOPullUp);
	GPIOSetPinDirection(VOL_UP_IOPIN, GPIO_IN);

	rockchip_mux_api_set(GPIOE_I2C0_SEL_NAME, IOMUXA_GPIO1_A45);  
	GPIOPullUpDown(VOL_DOWN_IOPIN,GPIOPullUp);
	GPIOSetPinDirection(VOL_DOWN_IOPIN, GPIO_IN);

	ret = request_gpio_irq(PAGE_DOWN_IOPIN,(pFunc)rk28_key_down_irq_handler,GPIOEdgelRising,NULL);
	if (ret)
		goto fail;

	ret = request_gpio_irq(PAGE_UP_IOPIN,(pFunc)rk28_key_up_irq_handler,GPIOEdgelRising,NULL);
	if (ret)
		goto fail;

	ret = request_gpio_irq(VOL_UP_IOPIN,(pFunc)rk28_key_vol_up_irq_handler,GPIOEdgelRising,NULL);
	if (ret)
		goto fail;

	ret = request_gpio_irq(VOL_DOWN_IOPIN,(pFunc)rk28_key_vol_down_irq_handler,GPIOEdgelRising,NULL);
	if (ret)
		goto fail;
	INIT_WORK(&scan_page_up_task,page_up_key_handle);
	INIT_WORK(&scan_page_down_task,page_down_key_handle);
	INIT_WORK(&scan_vol_up_task,vol_up_key_handle);
	INIT_WORK(&scan_vol_down_task,vol_down_key_handle);
	
	return;
fail:
	printk("%s %d\n", __FUNCTION__,__LINE__);
	return;
}

static int rk28_matrix_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct rk28_matrix_gpiokey *matrix_gpiokey = platform_get_drvdata(pdev);
	return 0;
}

static int rk28_matrix_resume(struct platform_device *pdev, pm_message_t msg)
{
	struct rk28_matrix_gpiokey *matrix_gpiokey = platform_get_drvdata(pdev);
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
	
	error = request_gpio_irq(PLAY_ON_IOPIN,(pFunc)rk28_playon_handler,GPIOEdgelFalling,NULL);
	if(error) {
		printk("unable to request playon key IRQ\n");
		goto failed2;
	}
	error = request_gpio_irq(VBUS_INT_IOPIN,(pFunc)rk28_playon_handler,GPIOEdgelRising,NULL);
	if(error) {
		printk("unable to request vbus IRQ\n");
		goto failed2;
	}

	INIT_WORK(&pwrkey_task, rk28_matrix_scan);
	rk28_matrix_init_timer();
	rk28_matrix_intr_request();
	key_signal_io_init_hw();
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
	.suspend 		= rk28_matrix_suspend,
	.resume 		= rk28_matrix_resume,
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
