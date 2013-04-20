/*
 *  ELAN touchscreen driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
      
#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>

//static const char ELAN_TS_NAME[] = "elan_touch";
#define ELAN_TS_NAME  "elan_touch"
#define ELAN_I2C_ID 	 	0x1588
#define ELAN_I2C_ADDR 	0x20

#define ELAN_TS_X_MAX 		1088
#define ELAN_TS_Y_MAX 		768
#define IDX_PACKET_SIZE		8

#define ELAN_TOUCH_AD_LEFT		0
#define ELAN_TOUCH_AD_RIGHT		ELAN_TS_Y_MAX
#define ELAN_TOUCH_AD_TOP 	ELAN_TS_X_MAX
#define ELAN_TOUCH_AD_BOTTOM	0
#define LCD_MAX_LENGTH				800
#define LCD_MAX_WIDTH				600
#define AD_TO_Y(ady)	(LCD_MAX_LENGTH* (ELAN_TOUCH_AD_TOP- ady) / (ELAN_TOUCH_AD_TOP - ELAN_TOUCH_AD_BOTTOM))
#define AD_TO_X(adx)	(LCD_MAX_WIDTH* (adx - ELAN_TOUCH_AD_LEFT) / ( ELAN_TOUCH_AD_RIGHT- ELAN_TOUCH_AD_LEFT ))


#define ELAN_INT_GPIO 	(GPIOPortE_Pin3)
#define ELAN_RST_GPIO	(GPIOPortF_Pin0)

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { ELAN_I2C_ADDR>>1, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
        .normal_i2c = normal_addr,
        .probe = ignore,
        .ignore = ignore,
};
enum {	
	hello_packet  = 0x55,
	idx_coordinate_packet 	= 0x5a,
	};
enum {	idx_finger_state = 7,};
static struct workqueue_struct *elan_wq = NULL;
static struct elan_data {
	int intr_gpio;
	int use_irq;	
	struct hrtimer timer;	
	struct work_struct work;	
	struct i2c_client client;	
	struct input_dev *input;	
	wait_queue_head_t wait;
};
static struct elan_data elan_touch_data = {0};

#if 0	/* elan_dlx */
#define elan_debug(...) \
	do { \
		printk(__VA_ARGS__); \
	} while (0) 
#else
#define elan_debug(...)
#endif
static int elan_touch_attach(struct i2c_adapter *adap);
static int elan_touch_detach(struct i2c_client *client);
static int elan_touch_remove(struct i2c_client *client);

static struct i2c_driver elan_touch_driver = {
	.driver 	= {
		.name = ELAN_TS_NAME,
		.owner = THIS_MODULE,
	},
	.attach_adapter	= elan_touch_attach,
	.detach_client	= elan_touch_detach,
	//.remove		= elan_touch_remove,
	//.id	= ELAN_I2C_ID,
};


static int elan_touch_detect_int_level(void)
{
	unsigned v;
	v = GPIOGetPinLevel(elan_touch_data.intr_gpio);	
	elan_debug("%s: ----> v = 0x%x\n", __FUNCTION__, v);		/* elan_dlx */
	return v;
}
static int __elan_touch_poll(struct i2c_client *client)
{	
	int status = 0, retry = 20;	
	do {		
		status = elan_touch_detect_int_level();
		retry--;	
		mdelay(20);
		} while (status == 1 && retry > 0);
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int __elan_i2c_transfer(struct i2c_client *client,__u8 *buf,__u16 len)
{
	int  ret;
	struct i2c_msg msg[1] = {
		{ client->addr, 1, len, buf },
	};	

	client->mode = DIRECTMODE;
	client->Channel = I2C_CH1;
	client->speed = 80;
	ret = i2c_transfer(client->adapter, &msg[0], 1);
		
	return ret;
}

static int elan_touch_poll(struct i2c_client *client)
{	
	return __elan_touch_poll(client);
}

static int __hello_packet_handler(struct i2c_client *client)
{	
	int rc;	
	uint8_t buf_recv[5] = { 0 };
	rc = elan_touch_poll(client);
	if (rc < 0) {		
		return -EINVAL;	
		}	
	
	rc = __elan_i2c_transfer(client, buf_recv, 4);
	//rc = i2c_master_recv(client, buf_recv, 4);
	if (rc != 4) {	
		printk("elan transfer rc = %d\n",rc);	
		return -1;	
		} 
	else {		
		int i;		
		printk("hello packet: [0x%02x 0x%02x 0x%02x 0x%02x]\n",	buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);	
		for (i = 0; i < 4; i++)	
			if (buf_recv[i] != hello_packet)	
				return -EINVAL;	
			}	
	return 0;
}

static int __elan_touch_init(struct i2c_client *client)
{	
	int rc;	
	rc = __hello_packet_handler(client);	
	if (rc < 0)
		goto hand_shake_failed;
hand_shake_failed:
	return rc;
}

static int elan_touch_recv_data(struct i2c_client *client, uint8_t *buf)
{	
	int rc, bytes_to_recv = IDX_PACKET_SIZE;
	if (buf == NULL)
		return -EINVAL;	
	memset(buf, 0, bytes_to_recv);
	rc = __elan_i2c_transfer(client, buf, bytes_to_recv);
	if (rc != bytes_to_recv) {	
		return -EINVAL;	
	}	
	return rc;
}

static inline int elan_touch_modify_xy(uint16_t *x, uint16_t *y)
{	
	uint16_t temp;

	temp = *y;
	*y = *x;
	*x = temp;

	*x = AD_TO_X(*x);
	*y = AD_TO_Y(*y);
	
	return 0;
}

static inline int elan_touch_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{	
	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];
	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	elan_touch_modify_xy(x, y);
	return 0;
}

static void elan_touch_report_data(struct i2c_client *client, uint8_t *buf)
{	
	switch (buf[0]) {
		case idx_coordinate_packet: 
			{		
				uint16_t x1, x2, y1, y2;		
				uint8_t finger_stat;	
				finger_stat = (buf[idx_finger_state] & 0x06) >> 1;
				x1=y1=x2=y2=0;
				if (finger_stat == 0) {	
					if ((buf[idx_finger_state] == 1) && !(buf[1]|buf[2]|buf[3]|buf[4]|buf[5]|buf[6])) {
						input_report_key(elan_touch_data.input, BTN_TOUCH, 0);	
						input_report_key(elan_touch_data.input, BTN_2, 0);		
						input_sync(elan_touch_data.input);
						elan_debug("%s(%s):elan touch up\n", __FILE__, __func__);	
						}
					} 
				else if (finger_stat == 1) {			
					elan_touch_parse_xy(&buf[1], &x1, &y1);
					input_report_abs(elan_touch_data.input, ABS_X, x1);
					input_report_abs(elan_touch_data.input, ABS_Y, y1);
					input_report_key(elan_touch_data.input, BTN_TOUCH, 1);	
					input_report_key(elan_touch_data.input, BTN_2, 0);		
					input_sync(elan_touch_data.input);
					elan_debug("%s(%s):%d,%d\n", __FILE__, __func__, x1,y1);	
					} 
				else if (finger_stat == 2) {
					elan_touch_parse_xy(&buf[1], &x1, &y1);	
					input_report_abs(elan_touch_data.input, ABS_X, x1);
					input_report_abs(elan_touch_data.input, ABS_Y, y1);
					input_report_key(elan_touch_data.input, BTN_TOUCH, 1);	
					elan_touch_parse_xy(&buf[4], &x2, &y2);	
					input_report_abs(elan_touch_data.input, ABS_HAT0X, x2);	
					input_report_abs(elan_touch_data.input, ABS_HAT0Y, y2);
					input_report_key(elan_touch_data.input, BTN_2, 1);		
					input_sync(elan_touch_data.input);
					elan_debug("%s(%s):%d,%d--%d,%d\n", __FILE__, __func__, x1,y1,x2,y2);	
					}		
				input_sync(elan_touch_data.input);
				gpio_irq_enable(elan_touch_data.intr_gpio);
				break;	
			}	
		default:	
			break;	
		}
}

static void elan_touch_work_func(struct work_struct *work)
{	
	int rc;	
	uint8_t buf[IDX_PACKET_SIZE] = { 0 };	
	struct i2c_client *client = &elan_touch_data.client;	
	elan_debug("%s: enter.......\n", __FUNCTION__); 		/* elan_dlx */
	if (elan_touch_detect_int_level())		
		return;	
	rc = elan_touch_recv_data(client, buf);
	if (rc < 0)
	{
		elan_debug("%s: elan_touch_recv_data error!!\n", __FUNCTION__); 		/* elan_dlx */
		return;	
	}
	elan_touch_report_data(client, buf);
}

static enum hrtimer_restart elan_touch_timer_func(struct hrtimer *timer)
{	
	queue_work(elan_wq, &elan_touch_data.work);
	hrtimer_start(&elan_touch_data.timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t elan_touch_ts_interrupt(int irq, void *dev_id)
{	
	queue_work(elan_wq, &elan_touch_data.work);
	return IRQ_HANDLED;
}

static int elan_touch_register_interrupt(struct i2c_client *client)
{	
	int err = 0;	
	client->irq = elan_touch_data.intr_gpio;//FIXME
	
	if (!elan_touch_data.use_irq) {	
		hrtimer_init(&elan_touch_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);	
		elan_touch_data.timer.function = elan_touch_timer_func;
		hrtimer_start(&elan_touch_data.timer, ktime_set(1, 0), HRTIMER_MODE_REL);	
		}	
	printk("elan ts starts in %s mode.\n",	elan_touch_data.use_irq == 1 ? "interrupt":"polling");	
	return 0;
}

static irqreturn_t elan_touch_interrupt(int irq, void *dev_id)
{
	struct elan_data *elan_touch = dev_id;

	elan_debug("%s: enter\n", __FUNCTION__);		/* elan_dlx */
	gpio_irq_disable(elan_touch_data.intr_gpio);
	queue_work(elan_wq, &elan_touch->work);

	return IRQ_HANDLED;
}

static void elan_touch_hw_init( void )
{
	rockchip_mux_api_set((char *)TOUCH_INT_IOMUX_PINNAME,(unsigned int)TOUCH_INT_IOMUX_PINDIR);
	GPIOSetPinDirection(TOUCH_INT_IOPIN, GPIO_IN);

	return 0;
}

static void elan_touch_hw_reset( void )
{
	rockchip_mux_api_set(GPIOF0_UART1_CPWM0_NAME, IOMUXA_GPIO1_B0);
	GPIOSetPinLevel(ELAN_RST_GPIO,GPIO_HIGH);
	mdelay(10);
	GPIOSetPinLevel(ELAN_RST_GPIO,GPIO_LOW);
	mdelay(4);

	return 0;
}

static int elan_touch_probe(struct i2c_adapter *adap, int addr, int kind)
{
       struct i2c_client *client;
	int rc = 0;	

	elan_debug("%s: enter, client = 0x%x addr = 0x%x\n", __FUNCTION__, client,addr);		/* elan_dlx */
	client = &elan_touch_data.client;
       i2c_set_clientdata(client, &elan_touch_data);
	strlcpy(client->name, ELAN_TS_NAME, I2C_NAME_SIZE-1);
	client->addr = addr;
	client->adapter = adap;
	client->driver = &elan_touch_driver;
	client->Channel = I2C_CH1;
	client->speed = 80;
	//client->mode = 0;
	client->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	
	rc = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!rc ){
		printk(KERN_ERR "i2c_check_functionality fail: %d\n", rc);
		goto fail;
	}

        rc = i2c_attach_client(client);
        if (rc) {
                printk(KERN_ERR "i2c_attach_client fail: %d\n", rc);
                goto fail;
        }
		
	INIT_WORK(&elan_touch_data.work, elan_touch_work_func);	
	elan_wq = create_singlethread_workqueue("elan_wq");	
	if (!elan_wq) {
		rc = -ENOMEM;
		goto fail;	
	}

	elan_touch_data.intr_gpio = ELAN_INT_GPIO;
	
	elan_touch_data.input = input_allocate_device();
	if (elan_touch_data.input == NULL) {
		rc = -ENOMEM;
		goto fail;
		}

	elan_touch_hw_init();
	elan_touch_hw_reset();
	rc = __elan_touch_init(client);	
	if (rc < 0) {
		printk("Read Hello Packet Fail\n");
		goto fail;	
		}	
	elan_touch_data.input->name = ELAN_TS_NAME;
	elan_touch_data.input->id.bustype = BUS_I2C;
	set_bit(EV_SYN, elan_touch_data.input->evbit);
	set_bit(EV_KEY, elan_touch_data.input->evbit);
	set_bit(BTN_TOUCH, elan_touch_data.input->keybit);	
	set_bit(BTN_2, elan_touch_data.input->keybit);		
	set_bit(EV_ABS, elan_touch_data.input->evbit);	
	set_bit(ABS_X, elan_touch_data.input->absbit);	
	set_bit(ABS_Y, elan_touch_data.input->absbit);	
	set_bit(ABS_HAT0X, elan_touch_data.input->absbit);	
	set_bit(ABS_HAT0Y, elan_touch_data.input->absbit);  
	input_set_abs_params(elan_touch_data.input, ABS_X, 0, LCD_MAX_WIDTH, 0, 0);	
	input_set_abs_params(elan_touch_data.input, ABS_Y, 0, LCD_MAX_LENGTH, 0, 0);	
	input_set_abs_params(elan_touch_data.input, ABS_HAT0X, 0, LCD_MAX_WIDTH, 0, 0);
	input_set_abs_params(elan_touch_data.input, ABS_HAT0Y, 0, LCD_MAX_LENGTH, 0, 0);	
	rc = input_register_device(elan_touch_data.input);	
	if (rc < 0) {		
		goto fail;
		}	

	GPIOPullUpDown(elan_touch_data.intr_gpio,GPIOPullUp);
	rc = request_gpio_irq(elan_touch_data.intr_gpio, elan_touch_interrupt, IRQF_TRIGGER_LOW,&elan_touch_data);
	if (rc < 0) {
		dev_err(&client->dev, "irq %d busy?\n", elan_touch_data.intr_gpio);
		goto fail;
	}	
	elan_touch_data.use_irq = 1;

	elan_touch_register_interrupt(&elan_touch_data.client);	
	return 0;	
fail:	
	if (NULL != elan_touch_data.input)
	{
		input_free_device(elan_touch_data.input);
		elan_touch_data.input = NULL;
	}
	if (NULL !=  elan_wq) {
		destroy_workqueue(elan_wq);
		elan_wq = NULL;
	}
 	printk("Run elan touch probe error %d!\n",rc);
	return rc;
}

static int elan_touch_attach(struct i2c_adapter *adap)
{
       return i2c_probe(adap, &addr_data, &elan_touch_probe);
}

static int elan_touch_detach(struct i2c_client *client)
{
	return 0;
}

static int elan_touch_remove(struct i2c_client *client)
{	
	if (elan_wq)		
		destroy_workqueue(elan_wq);	
	input_unregister_device(elan_touch_data.input);	
	if (elan_touch_data.use_irq)
		free_irq(client->irq, client);	
	else		
		hrtimer_cancel(&elan_touch_data.timer);	
	return 0;
}


static int __init elan_touch_init(void)
{
	elan_debug("%s: enter.......\n", __FUNCTION__);			/* elan_dlx */
	return i2c_add_driver(&elan_touch_driver);
}

static void __exit elan_touch_exit(void)
{
	i2c_del_driver(&elan_touch_driver);
}

subsys_initcall(elan_touch_init);
//module_init(elan_touch_init);
module_exit(elan_touch_exit);

MODULE_AUTHOR("Stanley Zeng <stanley.zeng@emc.com.tw>");
MODULE_DESCRIPTION("ELAN Touch Screen driver");
MODULE_LICENSE("GPL");

