/****************************************************************************************
 * driver/input/touchscreen/rk28_tocuscreen.c
 *Copyright 	:ROCKCHIP  Inc
 *Author	: WQQ 
 *Date		: 2009-04-25
 *This driver use for rk28 chip extern touchscreen. Use gpio to simulate  clock for touchscreen.
 ********************************************************************************************/
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/typedef.h>
#include <asm/arch/gpio.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/hw_define.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
/***********************************************
 *	DEBUG
 * ********************************************/
//#include <asm/arch/rk28_macro.h>
//#define RK28_PRINT	
#include <asm/arch/rk28_debug.h>
#define 	DEBUG_TS_SPI
#ifdef	DEBUG_TS_SPI
#define 	ts_spi_printk(msg...)	printk( "\n----Cong group DEBUG: " msg);
#else
#define 	ts_spi_printk(msg...)
#endif

#include "touchp.h"



#define TOUCHPANEL_PRODUCTION		0
#define TOUCHPANEL_PRODUCTION1		0
#define TOUCHPANEL_ITO2050			1
#define MODNAME 					"xpt2046_ts_input"
#define PT2046_PENIRQ				TOUCH_INT_IOPIN//This Pin is SDK Board GPIOPortE_Pin3 
#define LCD_MAX_LENGTH				800
#define LCD_MAX_WIDTH				480
#define XPT2046_IRQ 				7
#define XPT2046_NAME				"xpt2046 touchscreen"

#define TS_POLL_DELAY				(20*1000*1000)	/* ns delay before the first sample */
#define TS_POLL_PERIOD				(30*1000*1000)	/* ns delay between samples */
#define MAX_12BIT					((1<<12)-1)

/*xpt2046 parameter*/
#define PT2046_START_BIT		(1<<7)
#define PT2046_A2A1A0_x 	(5<<4)
#define PT2046_A2A1A0_y 	(1<<4)
#define PT2046_A2A1A0_z1		(3<<4)
#define PT2046_A2A1A0_z2		(4<<4)
#define PT2046_8_BIT			(1<<3)
#define PT2046_12_BIT			(0<<3)
#define PT2046_DFR				(0<<2)
#define PT2046_PD10_PDOWN		(0<<0)
#define PT2046_PD10_ADC_ON	(1<<0)
#define PT2046_PD10_REF_ON		(2<<0)
#define PT2046_PD10_ALL_ON		(3<<0)

#define READ_X		 (PT2046_START_BIT | PT2046_A2A1A0_x |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define READ_Y		(PT2046_START_BIT | PT2046_A2A1A0_y |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define READ_Z1 		(PT2046_START_BIT | PT2046_A2A1A0_z1 |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define READ_Z2 		(PT2046_START_BIT | PT2046_A2A1A0_z2 |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define PWRDOWN (PT2046_START_BIT | PT2046_A2A1A0_y |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)

#define SAVEPOINTMAXNUM 10
static int gDebugData[3];

struct XPT2046_TS_EVENT{
	struct input_dev *input;
	struct spi_device *spi;
	struct work_struct	x_work;
	struct work_struct	y_work;
	char	phys[32];
	int 		irq;
	spinlock_t	lock;
	uint16	x;
	uint16	y;
	uint16	z1;
	uint16	z2;
	uint16	   touch_x;
	uint16	touch_y;
	bool		pendown;
	bool	 status;
	struct hrtimer	timer;
};
typedef struct {
	unsigned x,y;
	bool flag;
}ts_event_t;

/*----------------------------------------------------------------------------*/

volatile struct adc_point gADPoint;
static uint16 gZvalue[3];
uint16   gLastZvalue[2] = {4095,4095};

/*	add by cong, 2010-06-14
 *	cal_status_android :	calibrate
 * */
static int cal_status_android = 6;
static int tmp_save_ad_point_x[10];
static int tmp_save_ad_point_y[10];
static int tmp_save_ad_point_num = 0;

/*  20100826@wqq
* declare function
*/
extern int uncali_x[5];
extern int uncali_y[5];
extern int TouchFilter(unsigned short* x,unsigned short* y,bool isdown);
extern void TouchReportFilter(unsigned short* x,unsigned short* y);
extern void ClearBuff(void);


static ssize_t touch_pressure(struct device_driver *_drv,char *_buf)
{
	//printk("enter %s gADPoint.x==%d,gADPoint.y==%d\n",__FUNCTION__,gADPoint.x,gADPoint.y);
	return sprintf(_buf,"%d,%d,%d\n",gZvalue[0],gZvalue[1],gZvalue[2]);
}


static ssize_t touch_android_status(struct device_driver *_drv,const char *_buf,size_t _count)	//add by cong, 2010-06-14
{
		char temp[5];
		 int tmp_get_android = 0;
		strncpy(temp, _buf, 1);
	  tmp_get_android = simple_strtol(temp, NULL, 10); 
	 //printk("--1--- cong(kernel)	tmp_get_android = %d	cal_status_android = %d\n", tmp_get_android,cal_status_android);
	 if(tmp_get_android == 7){
		tmp_save_ad_point_num = 0;
		return _count;
	 }else{
			cal_status_android = tmp_get_android;
	 }		  
	 if(cal_status_android <= 5 && cal_status_android >0){
		int tmp_sum_x = 0;
		int tmp_sum_y = 0;		
		int i = 0;
		int last_p = tmp_save_ad_point_num - 1;
		if(tmp_save_ad_point_num > SAVEPOINTMAXNUM)
				last_p = SAVEPOINTMAXNUM - 1;
		for(i = 0; i <= last_p; i ++){
			tmp_sum_x += tmp_save_ad_point_x[i];
			tmp_sum_y += tmp_save_ad_point_y[i];
			//printk(" -----------%d---y = %d		 tmp_sum_y = %d \n", i, tmp_save_ad_point_y[i], tmp_sum_y);
		}
		if(tmp_save_ad_point_num == 1){
			uncali_x[cal_status_android - 1] = tmp_sum_x;
			uncali_y[cal_status_android - 1] = tmp_sum_y;
		}else if(tmp_save_ad_point_num == 2){
			uncali_x[cal_status_android - 1] = tmp_sum_x / 2;
			uncali_y[cal_status_android - 1] = tmp_sum_y / 2;
		}else if(tmp_save_ad_point_num > 2){			
			//printk(" --------------tmp_sum_x = %d,  tmp_sum_y = %d, last_p = %d, last_x = %d, last_y = %d \n", tmp_sum_x, tmp_sum_y, last_p, tmp_save_ad_point_x[last_p], tmp_save_ad_point_y[last_p]);
			uncali_x[cal_status_android - 1] = (tmp_sum_x - tmp_save_ad_point_x[0] - tmp_save_ad_point_x[last_p]) / (last_p + 1 - 2);
			uncali_y[cal_status_android - 1] = (tmp_sum_y - tmp_save_ad_point_y[0] - tmp_save_ad_point_y[last_p]) / (last_p + 1 - 2);
		}	
		gADPoint.x = uncali_y[cal_status_android - 1];
		gADPoint.y = uncali_x[cal_status_android - 1];
			//printk("--2--- cong(kernel)				x = %d			 y = %d 	  tmp_save_ad_point_num = %d\n", uncali_x[cal_status_android - 1], uncali_y[cal_status_android - 1], tmp_save_ad_point_num);			
	 }
	 tmp_save_ad_point_num = 0;
		return _count;
}

static DRIVER_ATTR(calistatus, 0666, NULL, touch_android_status);
static DRIVER_ATTR(pressure, 0666, touch_pressure, NULL);

uint16 PT2046_read_op(struct XPT2046_TS_EVENT *ts_dev,u8 operation)
{	
	u8 tx_buf[1];
	u8 rx_buf[2];
	u16 val = 0;
	int ret;
	tx_buf[0] = operation;
	ret = spi_write_then_read(ts_dev->spi,tx_buf,1,rx_buf,2);
	if(ret)
	{
		printk("spi_read_op failded!!\n");
	}
	else
		val = rx_buf[0] ;		/*correct sample date(clear high bit)*/
		val = (val <<8) + rx_buf[1];
		val = (val&(~(1<<15))) >> 3;	
	return val;
} 
#define ADC_DELAYS 1000
static bool xpt2046_pressure_filter(struct XPT2046_TS_EVENT *ts_dev)
{	/*
	int x= PT2046_read_op(ts_dev,READ_X);
	udelay(ADC_DELAYS);
	gZvalue[0] = PT2046_read_op(ts_dev,READ_Z1);
	udelay(ADC_DELAYS);
	gZvalue[1] = PT2046_read_op(ts_dev,READ_Z2);
	udelay(ADC_DELAYS);
	if(gZvalue[0]  != 0)
		gZvalue[2] = 800*(x*((gZvalue[1]/gZvalue[0] ) - 1))/4096;
	if((gZvalue[2]<1000&&gZvalue[2]>400)||gZvalue[2]==0)
		return true;
	return false;
	*/
	uint16 x= PT2046_read_op(ts_dev,READ_X);
	udelay(500);
	gZvalue[0] = PT2046_read_op(ts_dev,READ_Z1);
	udelay(500);
	gZvalue[1] = PT2046_read_op(ts_dev,READ_Z2);
	udelay(500);
	gDebugData[0] = gZvalue[2];
	if(gZvalue[0]  != 0)
		gZvalue[2] = 800*(x*((gZvalue[1]/gZvalue[0] ) - 1))/4096;

	if(gZvalue[2]<5200)
		return true;
	return false;
}
static int xpt2046_read_values(struct XPT2046_TS_EVENT *ts_dev)
{
	uint16 *ptmpdata;
	int ret = 0,cnt = 0;
	bool accept = false;
	int tmp_p = 0;
	ptmpdata = kzalloc(sizeof(uint16)*2, GFP_KERNEL);
	do {
		ptmpdata[0] = gLastZvalue[0];
		ptmpdata[1] = gLastZvalue[1];
		accept = xpt2046_pressure_filter(ts_dev);
		//accept = true;
		if(accept)
		{
			ptmpdata[0]= PT2046_read_op(ts_dev,READ_X);
		 	udelay(ADC_DELAYS); 
		 	ptmpdata[1]= PT2046_read_op(ts_dev,READ_Y);	
		 	udelay(ADC_DELAYS);	 
			gLastZvalue[0] = ptmpdata[0];
			gLastZvalue[1] = ptmpdata[1];
			ret++;
		}
	//printk("pressure = %d,	x = %d	y = %d	   accept = %d \n",gDebugData[0], ptmpdata[0], ptmpdata[1], accept);
		cnt++;	 
	}while(cnt<2||0!=TouchFilter(&ptmpdata[0],&ptmpdata[1],ts_dev->pendown));
       tmp_p = tmp_save_ad_point_num % SAVEPOINTMAXNUM;
	if(ret)
	{
		ts_dev->x = gLastZvalue[0];
		ts_dev->y = gLastZvalue[1];
		if(cal_status_android < 5 && cal_status_android >= 0)
		{		
			tmp_save_ad_point_x[tmp_p] = ts_dev->x;
			tmp_save_ad_point_y[tmp_p] = ts_dev->y;
			tmp_save_ad_point_num ++;
		}
	}
	else
	{
		ts_dev->x = 0;
		ts_dev->y = 4095;
	}	
		
	//printk("xpt2046_read_values()   ts_dev->x = %d,	  ts_dev->y = %d	 cal_status_android = %d  tmp_save_ad_point_num = %d   tmp_p = %d \n", 
	//	ts_dev->x, ts_dev->y, cal_status_android, tmp_save_ad_point_num, tmp_p);
	
	gDebugData[1] = ts_dev->x;
	gDebugData[2] = ts_dev->y;
	rk28printk("pressure = %d	adx = %d	ady = %d\n",gDebugData[0],gDebugData[1],gDebugData[2]);
	rk28printk("pressure = %d\n",gDebugData[0]);
	kfree(ptmpdata);
	return ret;
}

static bool is_first_availability_point = false;		//the first down point is not availability
static void  xpt2046_send_values(struct XPT2046_TS_EVENT *ts_dev)
{
	struct XPT2046_TS_EVENT *ts = ts_dev;
	struct input_dev *xpt2046_ts_dev;
	u16 x,y,z1,z2;
	int pixelpoint_x = 0;
	int pixelpoint_y = 0;
	int padding = 0;
	x = ts->x;
	y = ts->y;
	z1 = ts->z1;
	z2 = ts->z2;

	if((x == 0)&&(y == 4095))	/*ignored pressure*/
	{/**/
		if(!ts->pendown)
		{
			ts_dev->pendown = 1;
			is_first_availability_point = true;
		}		
		//ts_dev->pendown = 1;	//change by cong, 2010-06-14, old core
		hrtimer_start(&ts_dev->timer,ktime_set(0,TS_POLL_PERIOD),HRTIMER_MODE_REL);
		return ;
	}else		/*valid event*/
	{
		if(is_first_availability_point){
			ts->pendown = 0;
			is_first_availability_point = false;
		}
		xpt2046_ts_dev = ts->input;
		if(!ts->pendown)
		{
			rk28printk("The touchscreen down!!\n");
			//printk("---x = %d, y = %d ---------------------------------------The touchscreen down!!\n", x, y);
			input_report_key(xpt2046_ts_dev,BTN_TOUCH,1);
			ts->pendown = 1;
		}
#if 0
		gADPoint.x = x;
		gADPoint.y = y; 
#else
		gADPoint.x = y;
		gADPoint.y = x; 
#endif
		TouchPanelCalibrateAPoint(gADPoint.x, gADPoint.y, &pixelpoint_x, &pixelpoint_y);
		ts->touch_x = pixelpoint_x/4;
		ts->touch_y = pixelpoint_y/4;  
		TouchReportFilter(&(ts->touch_x),&(ts->touch_y));
		//ts->touch_x = TouchReportFilterX(ts->touch_x);
		//ts->touch_y = TouchReportFilterY(ts->touch_y);
		rk28printk("send ad    value	x=%d	y=%d\n",gADPoint.x,gADPoint.y);
		rk28printk("send value px	 point	   x=%d    y=%d\n\n",ts->touch_x,ts->touch_y);
		if((ts->touch_x > LCD_MAX_LENGTH )||(ts->touch_y > LCD_MAX_WIDTH ))
		{
			/*should be calibrate mode*/
			rk28printk("should be calibrate mode\n");
			input_report_abs(xpt2046_ts_dev,ABS_X,100);
			input_report_abs(xpt2046_ts_dev,ABS_Y,100);
			input_sync(xpt2046_ts_dev);
		}
		input_report_abs(xpt2046_ts_dev,ABS_X,ts->touch_x);
		//padding=(ts->touch_y-300)/30;
		input_report_abs(xpt2046_ts_dev,ABS_Y,ts->touch_y+padding);
		//input_report_key(xpt2046_ts_dev,BTN_TOUCH,1);
		input_sync(xpt2046_ts_dev);
	}
	hrtimer_start(&ts_dev->timer,ktime_set(0,TS_POLL_PERIOD),HRTIMER_MODE_REL);
}

 static enum hrtimer_restart xpt2046_dostimer(struct hrtimer *handle)
{

	struct XPT2046_TS_EVENT *ts_dev = container_of(handle, struct XPT2046_TS_EVENT, timer);
	struct input_dev *xpt2046_ts_dev;
	int ts_io_pin_status;
	rk28printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
	rk28printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
	spin_lock_irq(&ts_dev->lock);
	ts_io_pin_status =  GPIOGetPinLevel(PT2046_PENIRQ);
	rk28printk("************>%s....PE7status=%d\n",__FUNCTION__,ts_io_pin_status);
	rk28printk("--------<%s>--- ts_dev->pendown = %d,  PE7status = %d \n", __FUNCTION__, ts_dev->pendown, ts_io_pin_status);
	if(unlikely(ts_dev->pendown && ts_io_pin_status))
	{
		xpt2046_ts_dev = ts_dev->input;
		rk28printk("The touchscreen up!!\n");
		//printk("-------------------------------------------The touchscreen up!!\n");
		input_report_key(xpt2046_ts_dev,BTN_TOUCH,0);
		input_sync(xpt2046_ts_dev);
		ts_dev->pendown = 0;	
		gpio_irq_enable(PT2046_PENIRQ);
		ClearBuff();
		gLastZvalue[0] = 4095;
		gLastZvalue[1] = 4095;
	}
	else{
		/*still down ,continue with the measurement*/
		xpt2046_read_values(ts_dev);
		xpt2046_send_values(ts_dev);
	}
	spin_unlock_irq(&ts_dev->lock);
 #ifdef CONFIG_ANDROID_POWER
	if(rk2818_get_suspend_flags() != PM_AWAKE) {
		rk28_send_wakeup_key();
		printk("touch screen wake up\n");
	}
#endif
	return HRTIMER_NORESTART;
}

extern void rk28_send_wakeup_key( void );
static irqreturn_t xpt2046_ts_interrupt(int irq,void *handle)
{
	struct XPT2046_TS_EVENT *ts_dev = handle;
	unsigned long flags;
	rk28printk("************>%s.....%s.....%d\n",__FILE__,__FUNCTION__,__LINE__);
	spin_lock_irqsave(&ts_dev->lock,flags);
	gpio_irq_disable(PT2046_PENIRQ);
	hrtimer_start(&ts_dev->timer,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);		
	spin_unlock_irqrestore(&ts_dev->lock, flags);
	return IRQ_HANDLED;
}
#ifdef CONFIG_ANDROID_POWER
static void rk28_ts_suspend(android_early_suspend_t *h)
{
#if 0
 printk("XPT2046 driver suspend!!\n");
 gpio_irq_disable(PT2046_PENIRQ);
#endif

}



static void rk28_ts_resume(android_early_suspend_t *h)
{
#if 0
	printk("XPT2046 driver resume!!\n");
	gpio_irq_enable(PT2046_PENIRQ);
#endif
}


static android_early_suspend_t ts_early_suspend;

#endif



static int	__devinit xpt2046_ts_proble(struct spi_device *spi)
{

	struct XPT2046_TS_EVENT  *ts_dev;
	struct input_dev *xpt2046_ts_dev;
	unsigned int err = 0;

	rk28printk("************>%s.....%s.....%d\n",__FILE__,__FUNCTION__,__LINE__);


	ts_dev=kzalloc(sizeof(struct XPT2046_TS_EVENT), GFP_KERNEL);
	if(!ts_dev)
	{
		printk("failed to allocate memory!!\n");
		goto nomem;
	}

	ts_dev->spi = spi;	/*ts_dev to spi reference*/
	printk("touch device spi speed = %d\n",spi->max_speed_hz);
	
	xpt2046_ts_dev = input_allocate_device();
	if(!xpt2046_ts_dev)
	{
		printk("rk28 xpt2046_ts allocate input device failed!!!\n");	
		goto fail1;
	}
	ts_dev->input = xpt2046_ts_dev;
/*init	timer to dispose workqueue */
	hrtimer_init(&ts_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts_dev->timer.function = xpt2046_dostimer;

	ts_dev->x = 0;
	ts_dev->y = 0;
	ts_dev->z1 = 0;
	ts_dev->z2 = 0;
	ts_dev->touch_x = 0;
	ts_dev->touch_y = 0;
	ts_dev->status = 0;
	ts_dev->pendown = 0;
	ts_dev->irq =XPT2046_IRQ;
	snprintf(ts_dev->phys,sizeof(ts_dev->phys),"%s/input0",spi->dev.bus_id);
	
	xpt2046_ts_dev->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY);
	xpt2046_ts_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	//input_set_abs_params(xpt2046_ts_dev,ABS_X,400,3680,0,0);
	//input_set_abs_params(xpt2046_ts_dev,ABS_Y,550,3350,0,0);
	
	input_set_abs_params(xpt2046_ts_dev,ABS_X,0,LCD_MAX_LENGTH,0,0);
	input_set_abs_params(xpt2046_ts_dev,ABS_Y,0,LCD_MAX_WIDTH,0,0);
	//input_set_abs_params(xpt2046_ts_dev, ABS_PRESSURE,0,4096, 0, 0);
	xpt2046_ts_dev->name = XPT2046_NAME;

#ifdef CONFIG_ANDROID_POWER

	 ts_early_suspend.suspend = rk28_ts_suspend;

		ts_early_suspend.resume = rk28_ts_resume;

		android_register_early_suspend(&ts_early_suspend);

#endif
	xpt2046_ts_dev->phys = ts_dev->phys;

	
	dev_set_drvdata(&spi->dev, ts_dev);
	xpt2046_ts_dev->dev.parent = &spi->dev;

	//xpt2046_read_values(ts_dev,MAX_SAMPLE_TIMES);
	xpt2046_read_values(ts_dev);
	rk28printk("************>%s....x=%d...y=%d...z1=%d...z2=%d\n",__FUNCTION__,ts_dev->x,ts_dev->y,ts_dev->z1,ts_dev->z2);
	
	///__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x30)|0x80,(GPIO1_BASE_ADDR_VA + 0x30));	
	//err = request_irq(ts_dev->irq,xpt2046_ts_interrupt,IRQF_TRIGGER_FALLING,spi->dev.driver->name,ts_dev);
	GPIOPullUpDown(PT2046_PENIRQ,GPIOPullUp);
	err = request_gpio_irq(PT2046_PENIRQ,xpt2046_ts_interrupt,GPIOEdgelFalling,ts_dev); 	
	if(err<0)
	{
		printk("xpt2046 request irq failed !!\n");
		err = -EBUSY;
		goto fail1;
	}
	err = input_register_device(xpt2046_ts_dev);
	if(err)
		goto fail2;
	return err;

fail2:	
	free_irq(XPT2046_IRQ,NULL);

fail1:
	input_free_device(xpt2046_ts_dev);
	hrtimer_cancel(&ts_dev->timer);
nomem:
		kfree(ts_dev);

	return err;
}
static	int __devexit xpt2046_ts_remove(struct spi_device *pdev)
{
	struct XPT2046_TS_EVENT *ts_dev =dev_get_drvdata(&pdev->dev);
	rk28printk("*****************************xpt2046_ts_remove******************\n");
	free_irq(ts_dev->irq,ts_dev);
	hrtimer_cancel(&ts_dev->timer);
	input_free_device(ts_dev->input);
	kfree(ts_dev);
	return 0;
}


static struct spi_driver xpt2046_ts_driver = {
	.driver = {
		.name = "xpt2046_ts",
		.bus	  = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = xpt2046_ts_proble,
	.remove = __devexit_p(xpt2046_ts_remove),

};

static int __init xpt2046_ts_init(void)
{
	int ret = spi_register_driver(&xpt2046_ts_driver);

	rk28printk("Touch panel drive XPT2046 driver init...\n");
	
	if (ret == 0)
	{
		gADPoint.x = 0;
		gADPoint.y = 0;

		ret += driver_create_file(&xpt2046_ts_driver.driver, &driver_attr_calistatus);
		ret += driver_create_file(&xpt2046_ts_driver.driver, &driver_attr_pressure);
	}
	
	return ret;

	
}

static void __exit xpt2046_ts_exit(void)
{
	rk28printk("Touch panel drive XPT2046 driver exit...\n");
	

	driver_remove_file(&xpt2046_ts_driver.driver, &driver_attr_calistatus);
	driver_remove_file(&xpt2046_ts_driver.driver, &driver_attr_pressure);
	
	return spi_unregister_driver(&xpt2046_ts_driver);

}
module_init(xpt2046_ts_init);
module_exit(xpt2046_ts_exit);
MODULE_AUTHOR("WQQ,wqq@rockchip.com");
MODULE_DESCRIPTION("rockchip rk28chip extern touchscreen");
MODULE_LICENSE("GPL");
