/*
 *  pca953x.c - 4/8/16 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <asm/arch/rk28_debug.h>
#include <linux/interrupt.h>

#define SN7326DBG
#ifdef SN7326DBG
#define printk_d(fmt, args...)  printk(KERN_INFO "[ROCK-CHIP SN7326 KEY] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define printk_d(fmt, args...)
#endif

/*open this for factory key test*/
//#define FACTORY_TEST

#define SN7326_SET			(0x08)
#define SN7326_DATA		(0x10)
#define SN7326_ADDR		(0xb0)
#define EmptyGpioValue		(0xff)
#define KEY_PHYS_NAME	"rk28_AD_button/input1"

struct sn7326_chip *gchip;
struct sn7326_chip {
	unsigned char keycodes[64];
	struct input_dev *input_dev;
	struct i2c_client *client;	
};
typedef  struct tag_gpio_key_value{
	u8 value;
	unsigned int keycode;
}gpio_key_value;

static struct timer_list power_key_timer;
static struct work_struct  scan_io_task;
static struct work_struct  scan_page_up_task;
static struct work_struct  scan_page_down_task;
static struct task_struct *sn7326_key_task=0;
static unsigned  int	pwrscantimes = 0;
static unsigned int 	encall_times = 100;
#ifdef FACTORY_TEST
static int key_code_test = 0;
#endif

#define ONESEC_TIMES		(100)
#define SEC_NUM			(1)
#define SLEEP_TIME		(2)
#define PP(y)					(0x01*(y))
#define OD(x)				(0x08*(x))
#define OD_PP(x,y)			((OD(x))+PP(y))

#define KEY_00	(KEY_DOWN)
#define KEY_10	(KEY_F5)
#define KEY_20	(KEY_M)
#define KEY_30	(KEY_J)
#define KEY_40	(KEY_B)
#define KEY_50	(KEY_C)
#define KEY_60	(KEY_Z)
#define KEY_70	(KEY_RESERVED)

#define KEY_01	(KEY_MENU)
#define KEY_11	(KEY_P)
#define KEY_21	(KEY_I)
#define KEY_31	(KEY_7)
#define KEY_41	(KEY_4)
#define KEY_51	(KEY_W)
#define KEY_61	(KEY_2)
#define KEY_71	(KEY_RESERVED)

#define KEY_02	(KEY_BACK)
#define KEY_12	(KEY_BACKSPACE)
#define KEY_22	(KEY_K)
#define KEY_32	(KEY_H)
#define KEY_42	(KEY_G)
#define KEY_52	(KEY_F)
#define KEY_62	(KEY_S)
#define KEY_72	(KEY_RESERVED)

#define KEY_03	(KEY_UP)
#define KEY_13	(KEY_SEARCH)
#define KEY_23	(KEY_O)
#define KEY_33	(KEY_Y)
#define KEY_43	(KEY_R)
#define KEY_53	(KEY_E)
#define KEY_63	(KEY_Q)
#define KEY_73	(KEY_RESERVED)

#define KEY_04	(KEY_ENTER)
#define KEY_14	(KEY_HOME)
#define KEY_24	(KEY_SPACE)
#define KEY_34	(KEY_N)
#define KEY_44	(KEY_V)
#define KEY_54	(KEY_X)
#define KEY_64	(KEY_LEFTSHIFT)
#define KEY_74	(KEY_RESERVED)

#define KEY_05	(KEY_RIGHT)
#define KEY_15	(KEY_LEFT)
#define KEY_25	(KEY_L)
#define KEY_35	(KEY_U)
#define KEY_45	(KEY_T)
#define KEY_55	(KEY_D)
#define KEY_65	(KEY_A)
#define KEY_75	(KEY_RESERVED)

#define KEY_06	(KEY_0)
#define KEY_16	(KEY_9)
#define KEY_26	(KEY_8)
#define KEY_36	(KEY_6)
#define KEY_46	(KEY_5)
#define KEY_56	(KEY_3)
#define KEY_66	(KEY_1)
#define KEY_76	(KEY_RESERVED)

#define KEY_07	(KEY_RESERVED)
#define KEY_17	(KEY_RESERVED)
#define KEY_27	(KEY_RESERVED)
#define KEY_37	(KEY_RESERVED)
#define KEY_47	(KEY_RESERVED)
#define KEY_57	(KEY_RESERVED)
#define KEY_67	(KEY_RESERVED)
#define KEY_77	(KEY_RESERVED)

static gpio_key_value gpiokey_code[] = 
{
	{OD_PP(0,0), KEY_00},
	{OD_PP(1,0), KEY_10},
	{OD_PP(2,0), KEY_20},
	{OD_PP(3,0), KEY_30},
	{OD_PP(4,0), KEY_40},
	{OD_PP(5,0), KEY_50},
	{OD_PP(6,0), KEY_60},
	{OD_PP(7,0), KEY_70},

	{OD_PP(0,1), KEY_01},
	{OD_PP(1,1), KEY_11},
	{OD_PP(2,1), KEY_21},
	{OD_PP(3,1), KEY_31},
	{OD_PP(4,1), KEY_41},
	{OD_PP(5,1), KEY_51},
	{OD_PP(6,1), KEY_61},
	{OD_PP(7,1), KEY_71},

	{OD_PP(0,2), KEY_02},
	{OD_PP(1,2), KEY_12},
	{OD_PP(2,2), KEY_22},
	{OD_PP(3,2), KEY_32},
	{OD_PP(4,2), KEY_42},
	{OD_PP(5,2), KEY_52},
	{OD_PP(6,2), KEY_62},
	{OD_PP(7,2), KEY_72},

	{OD_PP(0,3), KEY_03},
	{OD_PP(1,3), KEY_13},
	{OD_PP(2,3), KEY_23},
	{OD_PP(3,3), KEY_33},
	{OD_PP(4,3), KEY_43},
	{OD_PP(5,3), KEY_53},
	{OD_PP(6,3), KEY_63},
	{OD_PP(7,3), KEY_73},

	{OD_PP(0,4), KEY_04},
	{OD_PP(1,4), KEY_14},
	{OD_PP(2,4), KEY_24},
	{OD_PP(3,4), KEY_34},
	{OD_PP(4,4), KEY_44},
	{OD_PP(5,4), KEY_54},
	{OD_PP(6,4), KEY_64},
	{OD_PP(7,4), KEY_74},

	{OD_PP(0,5), KEY_05},
	{OD_PP(1,5), KEY_15},
	{OD_PP(2,5), KEY_25},
	{OD_PP(3,5), KEY_35},
	{OD_PP(4,5), KEY_45},
	{OD_PP(5,5), KEY_55},
	{OD_PP(6,5), KEY_65},
	{OD_PP(7,5), KEY_75},

	{OD_PP(0,6), KEY_06},
	{OD_PP(1,6), KEY_16},
	{OD_PP(2,6), KEY_26},
	{OD_PP(3,6), KEY_36},
	{OD_PP(4,6), KEY_46},
	{OD_PP(5,6), KEY_56},
	{OD_PP(6,6), KEY_66},
	{OD_PP(7,6), KEY_76},
	
	{OD_PP(0,7), KEY_07},
	{OD_PP(1,7), KEY_17},
	{OD_PP(2,7), KEY_27},
	{OD_PP(3,7), KEY_37},
	{OD_PP(4,7), KEY_47},
	{OD_PP(5,7), KEY_57},
	{OD_PP(6,7), KEY_67},
	{OD_PP(7,7), KEY_77},
};

static unsigned char initkey_code[ ] = 
{
	(KEY_00),(KEY_01),(KEY_02),(KEY_03),(KEY_04),(KEY_05),(KEY_06),(KEY_07),/*0*/
	(KEY_10),(KEY_11),(KEY_12),(KEY_13),(KEY_14),(KEY_15),(KEY_16),(KEY_17),/*1*/
	(KEY_20),(KEY_21),(KEY_22),(KEY_23),(KEY_24),(KEY_25),(KEY_26),(KEY_27),/*2*/
	(KEY_30),(KEY_31),(KEY_32),(KEY_33),(KEY_34),(KEY_35),(KEY_36),(KEY_37),/*3*/
	(KEY_40),(KEY_41),(KEY_42),(KEY_43),(KEY_44),(KEY_45),(KEY_46),(KEY_47),/*4*/
	(KEY_50),(KEY_51),(KEY_52),(KEY_53),(KEY_54),(KEY_55),(KEY_56),(KEY_57),/*5*/
	(KEY_60),(KEY_61),(KEY_62),(KEY_63),(KEY_64),(KEY_65),(KEY_66),(KEY_67),/*6*/
	(KEY_70),(KEY_71),(KEY_02),(KEY_73),(KEY_74),(KEY_75),(KEY_06),(KEY_77),/*7*/
	KEY_POWER,KEY_WAKEUP,KEY_LEFTBRACE,KEY_RIGHTBRACE
/*other key*/
};

void rk28_send_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() == PM_AWAKE )
		return;
	
	printk("%s:line%d\n",__FUNCTION__,__LINE__);
  	rk2818_set_suspend_flags(PM_AWAKE);
	input_report_key(gchip->input_dev,KEY_WAKEUP,1);
	input_sync(gchip->input_dev);
	input_report_key(gchip->input_dev,KEY_WAKEUP,0);
	input_sync(gchip->input_dev);
}
void rk28_send_rtc_wakeup_key( void ) 
{
	if(rk2818_get_suspend_flags() == PM_AWAKE )
		return;
	
	printk("%s:line%d\n",__FUNCTION__,__LINE__);
  	rk2818_set_suspend_flags(PM_AWAKE);
	input_report_key(gchip->input_dev,KEY_RTC_WAKEUP,1);
	input_sync(gchip->input_dev);
	input_report_key(gchip->input_dev,KEY_RTC_WAKEUP,0);
	input_sync(gchip->input_dev);
}
static irqreturn_t sn7326_wakeup_key(s32 irq, void *dev_id)
{
	printk_d("\n");
	if(rk2818_get_suspend_flags() == PM_TWOLEVEL_SLEEP) 
	{
		rk28_send_wakeup_key();
	}
	return IRQ_HANDLED;
}

static int sn7326_write_reg(struct sn7326_chip *chip, int reg, u8 val)
{	
	int ret = 0;
	u8 i2c_buf[2];
	struct i2c_msg msgs[1] = {
		{ chip->client->addr, 0, 2, i2c_buf }
	};

	i2c_buf[0] = reg;
	i2c_buf[1] = val;
	ret = i2c_transfer(chip->client->adapter, msgs, 1);
	return (ret < 0)?ret:0;
}

static int sn7326_read_reg(struct sn7326_chip *chip, int reg, u8 *val)
{
	int ret = 0;
	u8 buf[2];
	struct i2c_msg msg[1];

	msg->addr = chip->client->addr;
	msg->flags = 1;
	msg->buf = buf;
	msg->len = sizeof(buf);
	buf[0] = reg;
	ret = i2c_transfer(chip->client->adapter, msg, 1);
	if(ret != msg->len) {
		printk("%s; i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return 1;
	}

	//printk_d("reg 0x%02x = 0x%08x\n", reg, buf[0]);
	*val = (u8)buf[0];
	return 0;
}

static unsigned short normal_i2c[] = {SN7326_ADDR >> 1, I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;

static int sn7326_attach(struct i2c_adapter *adapter);
static int sn7326_detach(struct i2c_client *client);

static struct i2c_client_address_data addr_data_sn7326 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

static struct i2c_driver sn7326_driver = {
	.driver = {
		.name	= "sn7326",
	},
	.attach_adapter	= sn7326_attach,
	.detach_client		= sn7326_detach,
};

unsigned int find_rock_gpiokeycode(unsigned int value, gpio_key_value *ptab)
{	
	while(ptab->value != EmptyGpioValue) {
		if( value == ptab->value )
			return ptab->keycode;
		ptab++;
	}
	return 0;
}
static irqreturn_t rk28sn3726_irq(s32 irq, void *dev_id)
{
	if(!sn7326_key_task)
		return IRQ_HANDLED;
	
	wake_up_process(sn7326_key_task);
	return IRQ_HANDLED;
}

void scan_power_key(struct work_struct *work)
{
	if(!GPIOGetPinLevel(PLAY_ON_IOPIN)) {
		printk_d("in power key\n");
		pwrscantimes ++;
		if(pwrscantimes == (SEC_NUM * ONESEC_TIMES)) {
			input_report_key(gchip->input_dev,KEY_POWER,1);
			input_sync(gchip->input_dev);
			printk("the kernel come to power down!!!\n");
			encall_times = 100;
#ifdef FACTORY_TEST
			key_code_test = KEY_POWER;
#endif
		}
		if(pwrscantimes ==( (SEC_NUM + 1)* ONESEC_TIMES))
		{
			pwrscantimes = 0;
			input_report_key(gchip->input_dev,KEY_POWER,0);
			input_sync(gchip->input_dev);
			printk("the kernel come to power up!!!\n");
			encall_times = 100;
			
		}
		return ;
	} else {
		if(encall_times > 0)
			encall_times--;
	}
	if( pwrscantimes > SLEEP_TIME ) {
		pwrscantimes = 0;
		if(rk2818_get_suspend_flags() !=  PM_AWAKE) {
			if(rk2818_get_suspend_flags() == PM_TWOLEVEL_SLEEP) {
				printk("\nWake Up from key IRQ!!\n");
				return;
			}
			if(encall_times == 0) {
				input_report_key(gchip->input_dev,KEY_WAKEUP,1);
				input_sync(gchip->input_dev);
				input_report_key(gchip->input_dev,KEY_WAKEUP,0);
				input_sync(gchip->input_dev);
				printk_d("--power key: wake up\n");
#ifdef FACTORY_TEST
				key_code_test=KEY_WAKEUP;
#endif
			}
		} else {
			if(encall_times == 0) {
				input_report_key(gchip->input_dev,KEY_POWER,1);
				input_sync(gchip->input_dev);
				input_report_key(gchip->input_dev,KEY_POWER,0);
				input_sync(gchip->input_dev);
				printk("--power key: sleep\n");
#ifdef FACTORY_TEST
				key_code_test = KEY_POWER;
#endif
			}
		}
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

static void page_up_key_handle(struct work_struct *work)
{
	if(GPIOGetPinLevel(PAGE_UP_IOPIN)){
		request_gpio_irq(PAGE_UP_IOPIN,(pFunc)rk28_key_up_irq_handler,GPIOEdgelFalling,NULL);
		input_report_key(gchip->input_dev,KEY_LEFTBRACE,0);
		input_sync(gchip->input_dev);
		printk_d("keycode[%d] up\n",KEY_LEFTBRACE);
	} else{
		request_gpio_irq(PAGE_UP_IOPIN,(pFunc)rk28_key_up_irq_handler,GPIOEdgelRising,NULL);
		input_report_key(gchip->input_dev,KEY_LEFTBRACE,1);
		input_sync(gchip->input_dev);
		printk_d("keycode[%d] down\n",KEY_LEFTBRACE);
#ifdef FACTORY_TEST
		key_code_test = KEY_LEFTBRACE;
#endif
	}
}

static void page_down_key_handle(struct work_struct *work)
{
	if(GPIOGetPinLevel(PAGE_DOWN_IOPIN)){
		request_gpio_irq(PAGE_DOWN_IOPIN,(pFunc)rk28_key_down_irq_handler,GPIOEdgelFalling,NULL);
		input_report_key(gchip->input_dev,KEY_RIGHTBRACE,0);
		input_sync(gchip->input_dev);
		printk_d("keycode[%d] up\n",KEY_RIGHTBRACE);
	} else{
		request_gpio_irq(PAGE_DOWN_IOPIN,(pFunc)rk28_key_down_irq_handler,GPIOEdgelRising,NULL);
		input_report_key(gchip->input_dev,KEY_RIGHTBRACE,1);
		input_sync(gchip->input_dev);
		printk_d("keycode[%d] down\n",KEY_RIGHTBRACE);
#ifdef FACTORY_TEST
		key_code_test = KEY_RIGHTBRACE;
#endif
	}
}


static void key_page_up_down_init_hw(void)
{
	int ret = 0;

	rockchip_mux_api_set(GPIOA0_HOSTDATA16_SEL_NAME, PAGE_DOWN_IOMUX_PINDIR);   
	GPIOPullUpDown(PAGE_DOWN_IOPIN,GPIOPullUp);
	GPIOSetPinDirection(PAGE_DOWN_IOPIN, GPIO_IN);

	rockchip_mux_api_set(GPIOA1_HOSTDATA17_SEL_NAME, IOMUXB_GPIO0_A1);  
	GPIOPullUpDown(PAGE_UP_IOPIN,GPIOPullUp);
	GPIOSetPinDirection(PAGE_UP_IOPIN, GPIO_IN);

	ret = request_gpio_irq(PAGE_DOWN_IOPIN,(pFunc)rk28_key_down_irq_handler,GPIOEdgelRising,NULL);
	if (ret)
		goto fail;

	ret = request_gpio_irq(PAGE_UP_IOPIN,(pFunc)rk28_key_up_irq_handler,GPIOEdgelRising,NULL);
	if (ret)
		goto fail;
	INIT_WORK(&scan_page_up_task,page_up_key_handle);
	INIT_WORK(&scan_page_down_task,page_down_key_handle);
	
	return;
fail:
	printk("%s %d\n", __FUNCTION__,__LINE__);
	return;
}

static void sn7326_key_callback(unsigned long arg)
{
	power_key_timer.expires = jiffies + msecs_to_jiffies(10);
	add_timer(&power_key_timer);
	schedule_work(&scan_io_task);
}

static void sn7326_key_timer(void)
{
	printk_d("\n");
	init_timer(&power_key_timer);
	power_key_timer.function = sn7326_key_callback;
	power_key_timer.expires = jiffies + msecs_to_jiffies(10);
	add_timer(&power_key_timer);
}


static void sn7326_init_hw(void)
{
	printk_d("\n");
	rockchip_mux_api_set(SN7326_RESET_IOMUX_PINNAME, SN7326_RESET_IOMUX_PINDIR);
	GPIOPullUpDown(SN7326_RESET_IOPIN,GPIONormal);
	GPIOSetPinDirection(SN7326_RESET_IOPIN, GPIO_OUT);   
	GPIOSetPinLevel(SN7326_RESET_IOPIN, GPIO_LOW);    
	udelay(10);
	GPIOSetPinLevel(SN7326_RESET_IOPIN, GPIO_HIGH);
    
	rockchip_mux_api_set(SN7326_INT_IOMUX_PINNAME, SN7326_INT_IOMUX_PINDIR);  
	GPIOPullUpDown(SN7326_INT_IOPIN, GPIOPullUp);
	GPIOSetPinDirection(SN7326_INT_IOPIN, GPIO_IN);
	sn7326_write_reg(gchip, SN7326_SET, 0x10);
}

static int sn7326_key_thread(void *arg)
{
	u8 value = 0;
	u8 times = 0xff;
	unsigned int keycode = 0;
	unsigned int oldkeycode = 0;

	while(1)
	{
		set_current_state(TASK_INTERRUPTIBLE);
		value=0;
		times=0xff;  
		do {
			sn7326_read_reg(gchip, SN7326_DATA, &value);	
			//while(value==0xff){
			//	sn7326_init_hw();
			//	udelay(20);
			//	sn7326_read_reg(gchip, SN7326_DATA, &value);
			//}

			keycode= find_rock_gpiokeycode((value&0x3f), gpiokey_code);
			if((value & 0x40) && keycode) {                     
				if(keycode != oldkeycode) {                
					input_report_key(gchip->input_dev, keycode, 1);
					input_sync(gchip->input_dev);
					oldkeycode = keycode;
					printk_d("keycode[%d] down\n",keycode);
#ifdef FACTORY_TEST
					key_code_test = keycode;
#endif
				}                    
			} else if(keycode) {
				input_report_key(gchip->input_dev, keycode, 0);
				input_sync(gchip->input_dev);
				oldkeycode = 0;
				printk_d("keycode[%d] up \n",keycode);
			}   	        
		}while(value>>7);
		schedule();
	}
	return 0;
}

static int rk28_SN7326_button_open(struct input_dev *dev)
{
	return 0;
}

static void rk28_SN7326_button_close(struct input_dev *dev)
{
	return;
}

static int sn7326_probe(struct i2c_adapter *adapter, int addr, int kind)
{   
	struct i2c_client *pclient = NULL;    
	struct input_dev *input_dev = NULL;
	struct sn7326_chip *chip = NULL;
	int ret,i;
	
	printk_d("at address 0x%x\n", addr);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
		ret = -EIO;
		goto exit_check_functionality_failed;
	}

	chip = kzalloc(sizeof(struct sn7326_chip), GFP_KERNEL);
	if (chip == NULL)
		return -1;   

	pclient = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (pclient == NULL)
		return -1;       

	pclient->addr = addr;
	pclient->adapter = adapter;
	pclient->driver = &sn7326_driver;
	pclient->Channel = I2C_CH0;
	pclient->speed = 200;
	pclient->mode = NORMALMODE;
	strncpy(pclient->name, sn7326_driver.driver.name, I2C_NAME_SIZE-1);
	chip->client = pclient;
	gchip = chip;

	ret = i2c_attach_client(pclient);
	if (ret) {
		printk("failed to attach %s %d to i2c", pclient->name, ret);
		goto out_failed;
	}

	i2c_set_clientdata(chip->client, chip);
	sn7326_init_hw();

	/* Create and register the input driver. */
	input_dev = input_allocate_device();
	if (!input_dev) {
		printk("failed to allocate input device\n");
		ret = -ENOMEM;
		goto exit_alloc_client_failed;
	}

	memcpy(chip->keycodes, initkey_code, sizeof(initkey_code));
	input_dev->name = NULL;
	input_dev->open = rk28_SN7326_button_open;
	input_dev->close = rk28_SN7326_button_close;
	input_dev->dev.parent = NULL;
	input_dev->phys = KEY_PHYS_NAME;
	input_dev->id.vendor = 0x0002;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->keycode = chip->keycodes;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(initkey_code);
	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)
		set_bit(initkey_code[i], input_dev->keybit);
	
	clear_bit(0, input_dev->keybit);
	chip->input_dev = input_dev;
	input_set_drvdata(input_dev, chip);
	input_dev->evbit[0] = BIT_MASK(EV_KEY) ;

	/* Register the input device */
	ret = input_register_device(input_dev);
	if (ret) {
		printk("failed to register input device\n");
		goto out_failed;
	}

	sn7326_key_task = kthread_run(sn7326_key_thread, NULL, "sn7326_key_task");
	if (IS_ERR(sn7326_key_task)) {
		sn7326_key_task = 0;
		printk(KERN_ERR "sn7326 keyboard: Failed to create sn7326 key read thread.\n");
		return -1;
	}
	ret = request_gpio_irq(SN7326_INT_IOPIN, (pFunc)rk28sn3726_irq, GPIOEdgelFalling, NULL);
	if (ret) {
		ret = -EBUSY;
	}
	ret = request_gpio_irq(PLAY_ON_IOPIN,(pFunc)sn7326_wakeup_key,GPIOEdgelRising,NULL);
	if (ret) {
		ret = -EBUSY;
	}
	ret = request_gpio_irq(VBUS_INT_IOPIN,(pFunc)sn7326_wakeup_key, GPIOEdgelRising, NULL);
	if (ret) {
		ret = -EBUSY;
	}
	sn7326_key_timer();
	INIT_WORK(&scan_io_task,scan_power_key);
	key_page_up_down_init_hw();
	return ret;
out_failed:
	kfree(pclient); 
	kfree(chip); 
exit_alloc_client_failed:
exit_check_functionality_failed:
	return ret;
}

static int sn7326_attach(struct i2c_adapter *adapter)
{
    return i2c_probe(adapter, &addr_data_sn7326, sn7326_probe);
}

static int sn7326_detach(struct i2c_client *client)
{
	printk_d("client->name[%s]\n", client->name);
	if(client)
		kfree(client);

	return 0;
}

static int __init sn7326_init(void)
{
	printk_d("\n");
	return i2c_add_driver(&sn7326_driver);
}
 
static void __exit sn7326_exit(void)
{
	printk_d("\n");
	i2c_del_driver(&sn7326_driver);
}

module_init(sn7326_init);
module_exit(sn7326_exit);


#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_sn7326_show(struct seq_file *s, void *v)
{
	//u8 value = 0;
	//sn7326_read_reg(gchip, SN7326_DATA, &value);
#ifdef FACTORY_TEST
	seq_printf(s, "\n %d \n", key_code_test);
	key_code_test = 0;
#endif
	return 0;
}

static int proc_sn7326_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_sn7326_show, NULL);
}

static const struct file_operations proc_sn7326_fops = {
	.open		= proc_sn7326_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
};

static int __init sn7326_proc_init(void)
{
	proc_create("sn7326", 0, NULL, &proc_sn7326_fops);
	return 0;
}
late_initcall(sn7326_proc_init);
#endif /* CONFIG_PROC_FS */


MODULE_AUTHOR("zyw <zyw@rock-chips.com>");
MODULE_DESCRIPTION("GPIO expander driver for SN7326");
MODULE_LICENSE("GPL");

