/****************************************************************************************
 * driver/input/touchscreen/i2cpca955x.c
 *Copyright 	:ROCKCHIP  Inc
 *Author	: 	 sfm
 *Date		:  2010.2.5
 *This driver use for rk28 chip extern touchscreen. Use i2c IF ,the chip is pca955x
 *description£º
 ********************************************************************************************/
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
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
#include <asm/arch/iomux.h>
#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include "touchp.h"


#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include <asm/arch/api_i2c.h>

#define SISDBG 1
static struct workqueue_struct *IT7260_wq;

#if SISDBG
#define sisdbg(msg...) printk(msg)
#else
#define sisdbg(msg...) 
#endif

/******************************************
		DEBUG
*** ***************************************/
//#define RK28_PRINT
//#include <asm/arch/debug.h>
#define SIS809_IRQ             7
#define SIS809_NAME    "sis809_i2c"
#define SIS809_IRQ_PIN   	TOUCH_INT_IOPIN//GPIOPortE_Pin3
#define SIS809_I2C_ADDR   		0x8c
#define SIS809_IIC_SPEED 		200
#define MAX_810_SMB_READ_BYTES   16

#define SIZE_8_4
#define OUTSIZEX 0
#define OUTSIZEY 0
#ifdef SIZE_8_4
#define SIS_MAX_X	(25*32 - 2*OUTSIZEX)
#define SIS_MAX_Y	(15*32 - 2*OUTSIZEY)
#else
#define SIS_MAX_X	(21*128-2*OUTSIZEX)
#define SIS_MAX_Y	(13*128-2*OUTSIZEY)
#endif

#define Mulitouch_Mode  0
#define Singltouch_Mode 1

struct touch_event{
	uint16 x;
	uint16 y;
};
struct MultiTouch_event{
	uint16 x1;
	uint16 y1;
	uint16 x2;
	uint16 y2;
	char p1_press;
	char p2_press;	
};
#define TS_POLL_DELAY	(13*1000000) /* ns delay before the first sample */
#define TS_POLL_PERIOD	(12*1000000) /* ns delay between samples */

static struct  i2c_client sis809_client;
static int sis809_write_regs(struct i2c_client *client, u8 reg, u8 const buf[], unsigned short len);

/*tochscreen private data*/
static u8 i2c_buf[MAX_810_SMB_READ_BYTES];
//static void sis809_irq_work_handler(struct sis809_dev *ts_dev);
static int sis809_probe(struct i2c_adapter *bus, int address, int kind);

#define P_DELAYTIME  6
struct sis809_dev{	
	struct i2c_client *client;
	struct input_dev *input;
	spinlock_t	lock;
	char	phys[32];
	int 		irq;
#if Singltouch_Mode
	struct touch_event  point;  
#else
	struct MultiTouch_event  point;
	char   P_state;
	char   p_DelayTime;
#endif
	struct work_struct work;	
	bool		pendown;
	bool 	 status;
	struct hrtimer  timer;
	int has_relative_report;
};

struct sis809_dev ite_ts;
struct sis809_dev *g_dev;

/**********************************************************************************************/

#define MAX_FINGER_NUM 3
#define DEVICE_ADDRESS 0x8C
#define COMMAND_BUFFER_INDEX 0x20
#define QUERY_BUFFER_INDEX 0x80
#define COMMAND_RESPONSE_BUFFER_INDEX 0xA0
#define POINT_BUFFER_INDEX 0xE0
#define QUERY_SUCCESS 0x00
#define QUERY_BUSY 0x01
#define QUERY_ERROR 0x02
#define QUERY_POINT 0x80
#define POINT_X_HIGH_MASK 0x0f
#define POINT_Y_HIGH_MASK 0xf0
#define POINT_INFO_MASK 0x07
#define POINT_PLAM_MASK 0x01

/*read the gtt8205 register ,used i2c bus*/
static int cj3b_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 1, len, buf },
	};
	client->mode = TP7260;
	client->Channel = I2C_CH1;
	client->speed = 80;
	buf[0] = reg;
	//printk("%s the slave i2c device mode == %d\n",__FUNCTION__,client->adapter->mode);
	ret = i2c_transfer(client->adapter, msgs, 1);
	
	if (ret > 0)
		ret = 0;	
	return ret;
}


/* set the gtt8205 registe,used i2c bus*/
static int cj3b_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], unsigned short len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};
	client->mode = TP7260;;
	client->Channel = I2C_CH1;
	client->speed = 80;
	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);
	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	
	return ret;
}


BOOL ReadQueryBuffer(struct i2c_client *client,u8 pucData[])
{
	//sisdbg("%s->%d\n",__FUNCTION__,__LINE__);
	return 	cj3b_read_regs(client,QUERY_BUFFER_INDEX,pucData,1);

}

BOOL ReadCommandResponseBuffer(struct i2c_client *client,u8 pucData[], unsigned int unDataLength)
{
	return 	cj3b_read_regs(client,COMMAND_RESPONSE_BUFFER_INDEX,pucData,unDataLength);

}

BOOL ReadPointBuffer(struct i2c_client *client,u8 pucData[])
{
	return 	cj3b_read_regs(client,POINT_BUFFER_INDEX,pucData,14);

}


BOOL WriteCommandBuffer(struct i2c_client *client,u8 pucData[], unsigned int unDataLength)
{
	return 	cj3b_set_regs(client,COMMAND_BUFFER_INDEX,pucData,unDataLength);
}



/**********************************************************************************************/



/*read thesis809 register ,used i2c bus*/
static int sis809_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 1, len, buf },
	};
	buf[0] = reg;
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret< 0)
	{
		printk("error at sis809_read_regs !!! \n");	
	}
	return ret;
}
/* set the pca955x registe,used i2c bus*/
static int sis809_write_regs(struct i2c_client *client, u8 reg, u8 const buf[], unsigned short len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};
	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret< 0)
	{
		printk("error at sis809_write_regs !!! \n");	
	}	
	return ret;
}

static int sis809_chip_Init(struct sis809_dev *ts_dev)
{
#if 0
 	u8 pucData1[9] = {0x0d,0x01,0x01,0x02,0xf2,0x00,0x00,0x2c};
 	u8 pucData2[9] = {0x00,0x01,0x01,0x18,0xf1,0x00,0x00,0x57};
	WriteCommandBuffer(ts_dev->client,pucData1, 8);
	cj3b_read_regs(ts_dev->client,COMMAND_RESPONSE_BUFFER_INDEX,pucData1,8);
	printk("pucData1=[%c][%c][%c][%c] \n",pucData1[1],pucData1[2],pucData1[3],pucData1[4]);
	WriteCommandBuffer(ts_dev->client,pucData2, 8);
	cj3b_read_regs(ts_dev->client,COMMAND_RESPONSE_BUFFER_INDEX,pucData2,8);
	printk("pucData2=[%c][%c][%c][%c] \n",pucData2[1],pucData2[2],pucData2[3],pucData2[4]);
	GPIOPullUpDown(SIS809_IRQ_PIN,GPIOPullUp);
#endif
	return TRUE;
	
	
}
/*
sleep
*/
static void sis809_chip_sleep(void)
{ 	
	char ret=0;
	char buf;
	buf=0;
//	ret = sis809_write_regs(sis809_client,0xf1,&buf,1);
    //ret = i2c_smbus_write_byte_data(sis809_client, 0xf1, 0); // disable interrupt
    if (ret < 0)
        printk(KERN_ERR "sis_ts_suspend 0: i2c_smbus_write_byte_data failed\n");
	buf=0x86;
	//ret = sis809_write_regs(sis809_client,0xf0,&buf,1);
    //ret = i2c_smbus_write_byte_data(sis809_client, 0xf0, 0x86); // deep sleep
    if (ret < 0)
        printk(KERN_ERR "sis_ts_suspend 1: i2c_smbus_write_byte_data failed\n");
}
/*
wake up
*/
static void sis809_chip_wakeup(void)
{

	
}


void ite_ts_test()
{

	u8 ucWriteLength, ucReadLength;
	u8 pucData[128];
	u8 ucQuery;
	ucWriteLength = 1;
	ucReadLength = 0x0A;
	pucData[0] = 0x00;
	
	// Query
	
	do
	{
		ReadQueryBuffer(g_dev->client,&ucQuery);
		if(ucQuery == 0)
			break;
	}while(ucQuery & QUERY_BUSY);

		// Write Command
	
//	while(1)
	{	printk("IdentifyCapSensor\n");
		if(0!=WriteCommandBuffer(g_dev->client,pucData, ucWriteLength))
		{
			printk("WriteCommandBuffer error\n");
			//return false;
		}
		// Query
#ifdef INTERRUPT_MODE
		Wait4INT();
#endif
	
		do
		{
			ReadQueryBuffer(g_dev->client,&ucQuery);
			if(ucQuery == 0)
				break;
		}while(ucQuery & QUERY_BUSY);
		// Read Command Response
		
		if(0!=ReadCommandResponseBuffer(g_dev->client,pucData, ucReadLength))
		{
			printk("ReadCommandResponseBuffer error\n");
			//return false;
		}
	}
	cj3b_read_regs(g_dev->client,COMMAND_RESPONSE_BUFFER_INDEX,pucData,10);
	printk("[%c][%c][%c][%c] \n",pucData[1],pucData[2],pucData[3],pucData[4]);
/*	if(pucData[1] != 'I'
	|| pucData[2] != 'T'
	|| pucData[3] != 'E'
	|| pucData[4] != '7'
	|| pucData[5] != '2'
	|| pucData[6] != '6'
	|| pucData[7] != '0')
	{
		// firmware signature is not match
		return false;
	}*/

	return ;

}

static int read_point(struct sis809_dev *ts_dev )
{


	u8 pucPoint[20];
	u8 ucQuery =0;
	u8 i;
	int xraw, yraw, xtmp, ytmp;
	char pressure_point,z,w;
	int finger2_pressed=0;
for(i=0;i<4;i++)
{
	ReadQueryBuffer(ts_dev->client,&ucQuery);
	printk("ucQuery = 0x%x\n",ucQuery);
	if(ucQuery == 0)
	{
		gpio_irq_enable(SIS809_IRQ_PIN);
		return 0;
	}
	else{
		if(ucQuery & 0x80)
		{
			cj3b_read_regs(ts_dev->client,POINT_BUFFER_INDEX,pucPoint,14);
			//for(i = 0; i < 14; i++)
				//printk("0x%2x  ",pucPoint[i]);
			//printk("\n");	
			if(pucPoint[0] & 0xF0)				
			{
				gpio_irq_enable(SIS809_IRQ_PIN);
				return 0;
			}				
			else	{					
				if(pucPoint[1] & 0x01)					
				{
					gpio_irq_enable(SIS809_IRQ_PIN);
					return 0;
				}			
			}
			if(pucPoint[0] & 0x01)				
			{										
									
				xraw = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];					
				yraw = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];					
				pressure_point=pucPoint[5]&0x0f;										
				xtmp = xraw ;				
				ytmp = yraw ;		
#if Mulitouch_Mode
				ts_dev->point.x1 = xtmp;
				ts_dev->point.y1 = ytmp;
				ts_dev->has_relative_report = 1;
#endif
				if(pressure_point==4)					
				{						
					z=10;						
					w=15;					
				}					
				else					
				{						
					z=10;						
					w=15;					
				}					
				printk("=Read_Point1 x=%d y=%d p=%d=\n",xtmp,ytmp,pressure_point);
				
#if Singltouch_Mode					
				if(ts_dev->status == 0)
				{
					ts_dev->status = 1;
					input_report_abs(ts_dev->input, ABS_X, xtmp);					
					input_report_abs(ts_dev->input, ABS_Y, ytmp);	
					input_report_key(ts_dev->input, BTN_TOUCH, 1);		
				}else{
					input_report_abs(ts_dev->input, ABS_X, xtmp);					
					input_report_abs(ts_dev->input, ABS_Y, ytmp);
				}
			
				input_report_abs(ts_dev->input, ABS_PRESSURE, 1);					
				input_sync(ts_dev->input);
#else										

				input_report_abs(ts_dev->input, ABS_MT_TOUCH_MAJOR, z);					
				input_report_abs(ts_dev->input, ABS_MT_WIDTH_MAJOR, w);										
				input_report_abs(ts_dev->input, ABS_MT_POSITION_X, xtmp);					
				input_report_abs(ts_dev->input, ABS_MT_POSITION_Y, ytmp);		
				input_report_key(ts_dev->input, BTN_TOUCH, 1);
				input_mt_sync(ts_dev->input);
#endif												
			}
#if Mulitouch_Mode

			 if(pucPoint[0] & 0x02)				
			 {					
			 	xraw = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];					
				yraw = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];					
				pressure_point=pucPoint[9]&0x0f;					
				xtmp = xraw ;					
				ytmp = yraw ;          
				ts_dev->point.x2 = xtmp;
				ts_dev->point.y2 = ytmp;
				ts_dev->has_relative_report = 2;
				printk("=Read_Point2 x=%d y=%d p=%d=\n",xtmp,ytmp,pressure_point);					
				if(pressure_point==4)					
				{						
					z=10;						
					w=15;					
				}					
				else					
					{						
					z=10;						
					w=15;					
				}
#if Singltouch_Mode					
				input_report_abs(ts_dev->input, ABS_X, xtmp);					
				input_report_abs(ts_dev->input, ABS_Y, ytmp);					
				input_report_key(ts_dev->input, BTN_TOUCH, 1);					
				//input_report_abs(ts_dev->input, ABS_PRESSURE, 1);					
				input_sync(ts_dev->input);
#else					
				input_report_abs(ts_dev->input, ABS_MT_TOUCH_MAJOR, z);					
				input_report_abs(ts_dev->input, ABS_MT_WIDTH_MAJOR, w);					
				input_report_abs(ts_dev->input, ABS_MT_POSITION_X, xtmp);					
				input_report_abs(ts_dev->input, ABS_MT_POSITION_Y, ytmp);	
				input_report_key(ts_dev->input, BTN_2, 1);
				input_mt_sync(ts_dev->input);
#endif
			
			}		
#endif
			if((pucPoint[0] & 0x07)==0)				
			{				
				printk("=Read_Point pull=\n");
#if Singltouch_Mode					
				ts_dev->status = 0;
				input_report_key(ts_dev->input, BTN_TOUCH, 0);					
				//input_report_abs(ts_dev->input, ABS_PRESSURE, 0);					
				input_sync(ts_dev->input);
#else					
					
				input_report_abs(ts_dev->input, ABS_MT_TOUCH_MAJOR, 0);					
				input_report_abs(ts_dev->input, ABS_MT_WIDTH_MAJOR, 15);	
				input_report_abs(ts_dev->input, ABS_MT_POSITION_X, ts_dev->point.x1);					
				input_report_abs(ts_dev->input, ABS_MT_POSITION_Y, ts_dev->point.y1);
				input_report_key(ts_dev->input, BTN_TOUCH, 0);
				input_mt_sync(ts_dev->input);
				if(ts_dev->has_relative_report ==2)
				{
					ts_dev->has_relative_report = 0;
					input_report_abs(ts_dev->input, ABS_MT_TOUCH_MAJOR, 0);					
					input_report_abs(ts_dev->input, ABS_MT_WIDTH_MAJOR, 15);	
					input_report_abs(ts_dev->input, ABS_MT_POSITION_X, ts_dev->point.x2);					
					input_report_abs(ts_dev->input, ABS_MT_POSITION_Y, ts_dev->point.y2);	
					input_report_key(ts_dev->input, BTN_2, 0);
					input_mt_sync(ts_dev->input);
				}
#endif			
	
			}				

			input_sync(ts_dev->input);	
		}
		
	}
}

	gpio_irq_enable(SIS809_IRQ_PIN);
	return 0;
}



static void sis809_work_handler(struct work_struct *work)
{
	struct input_dev *input;
       struct sis809_dev *ts_dev  = container_of(work,struct sis809_dev,work);
	  int i;
         read_point(ts_dev);
}

 static enum hrtimer_restart sis809_dostimer(struct hrtimer *handle)
{
	u8 pucData[128];
	u8 ucQuery;
	struct sis809_dev *ts_dev = container_of(handle, struct sis809_dev, timer);
	printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
	spin_lock_irq(&ts_dev->lock);

	spin_unlock_irq(&ts_dev->lock);
	return HRTIMER_NORESTART;
}


static irqreturn_t sis809_irq_hander(int irq, void *dev_id)
{
	struct sis809_dev *ts_dev = dev_id;
	unsigned long flags;
	//sisdbg("************>%s.....%s.....%d\n",__FILE__,__FUNCTION__,__LINE__);
	gpio_irq_disable(SIS809_IRQ_PIN);
       queue_work(IT7260_wq, &ts_dev->work);
	return IRQ_HANDLED;
}

static int sis809_detach_client(struct i2c_client *client)
{
	printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
	return 0;
}

static void sis809_shutdown(struct i2c_client *client)
{
	printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
}
static unsigned short sis809_normal_i2c[] = {SIS809_I2C_ADDR>>1,I2C_CLIENT_END};
static unsigned short sis809_ignore = I2C_CLIENT_END;

static struct i2c_client_address_data sis809_addr_data={
	.normal_i2c = sis809_normal_i2c,
	.probe = &sis809_ignore,
	.ignore =& sis809_ignore,
};
static int sis809_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap,&sis809_addr_data,sis809_probe);
}
static struct i2c_driver sis809_driver  = {
	.driver = {
		.name = "sis809_i2c",
		.owner = THIS_MODULE,
	},
	.id = SIS809_I2C_ADDR,
	.attach_adapter = &sis809_attach_adapter,
	.detach_client 	=  &sis809_detach_client,
	.shutdown     	=  &sis809_shutdown,
};
static struct  i2c_client sis809_client = {
		.driver = &sis809_driver,
		.name	= "sis809_i2c",
	};

#ifdef CONFIG_ANDROID_POWER
static void suspend(android_early_suspend_t *h)
{
	sis809_chip_sleep();
	printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
}
static void resume(android_early_suspend_t *h)
{
	  sis809_chip_wakeup();
	  printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
}
static android_early_suspend_t ts_early_suspend;
#endif



static int sis809_probe(struct i2c_adapter *bus, int address, int kind)
{


	struct sis809_dev *ts_dev;
	struct input_dev *input;
	unsigned int err = 0;
	int sr;
	int ret=0;
	u8  buf[12] ;		/*single touch*/
	printk("++++++++++++++++++++++++++enter sis809_probe!!!\n");
	ts_dev=kzalloc(sizeof(struct sis809_dev), GFP_KERNEL);
	if(!ts_dev)
	{
		printk("sis809 failed to allocate memory!!\n");
		goto nomem;
	}
	printk("sis809_client.addr 0x%x\n",address);
	
	sis809_client.adapter = bus;
	sis809_client.addr= address;
	sis809_client.mode = TP7260;//DIRECTMODE;
	sis809_client.Channel = I2C_CH1;
	sis809_client.speed = 100;
	sis809_client.addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	ts_dev->client=&sis809_client; 
	ret = i2c_attach_client(&sis809_client);
	if (ret < 0)
	{
		 printk("sis809 attach client failed!!!!\n");
		 goto nomem;	
	}
	input = input_allocate_device();
	if(!input)
	{
		printk("sis809 allocate input device failed!!!\n"); 
		goto fail1;
	}	
	hrtimer_init(&ts_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts_dev->timer.function = sis809_dostimer;
	ts_dev->status = 0;
	ts_dev->pendown = 0;
	ts_dev->input = input;		
	ts_dev->irq = 7;	
	ts_dev->has_relative_report = 0;
	ts_dev->input->phys="sis809_i2c/input0";
#if Singltouch_Mode
	input->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY)|BIT_MASK(EV_SYN);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input, ABS_X, 0, 800, 0, 0);
  	input_set_abs_params(input, ABS_Y, 35, 480, 0, 0);
#else
	input->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY)|BIT_MASK(EV_SYN);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input->keybit[BIT_WORD(BTN_2)] = BIT_MASK(BTN_2); //jaocbchen for dual

	input_set_abs_params(input, ABS_X, 0, 800, 0, 0);
	input_set_abs_params(input, ABS_Y, 35, 480, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	input_set_abs_params(input, ABS_HAT0X, 0, SIS_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_HAT0Y, 0, SIS_MAX_Y, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X,0, 800, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 40, 480, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

	int i = 0;
	for (i = 0; i < (BITS_TO_LONGS(ABS_CNT)); i++)
		printk("%s::input->absbit[%d] = 0x%x \n",__FUNCTION__,i,input->absbit[i]);

	
	
#endif
	input->name = "sis809_i2c";
       IT7260_wq = create_singlethread_workqueue("IT7260_wq");
	INIT_WORK(&ts_dev->work, sis809_work_handler);
	ret = input_register_device(input);	
	if(ret<0)
	{
		printk("sis809 register input device failed!!!!\n");
		goto fail2;
	}
   // ReCalibrate(ts_dev);
	
#ifdef CONFIG_ANDROID_POWER
   	ts_early_suspend.suspend = suspend;
    ts_early_suspend.resume = resume;
   // android_register_early_suspend(&ts_early_suspend);
#endif	
	g_dev = ts_dev;
	//ite_ts_test(); //

	rockchip_mux_api_set(TOUCH_INT_IOMUX_PINNAME,TOUCH_INT_IOMUX_PINDIR);
	GPIOSetPinDirection(SIS809_IRQ_PIN, GPIO_IN);

	//GPIOPullUpDown(SIS809_IRQ_PIN,GPIOPullUp);
	mdelay(10);
	ret = request_gpio_irq(SIS809_IRQ_PIN,sis809_irq_hander,GPIOLevelLow/*GPIOEdgelFalling*/,ts_dev);
	if(ret<0)
	{
		printk("unable to request pca955x touchscreen IRQ\n");
		goto fail3;
	}
	printk("GPIOPortE_Pin3 level=%d!!!!\n",GPIOGetPinLevel(SIS809_IRQ_PIN));
	ret=sis809_chip_Init(ts_dev);
	if(ret<0)
	{
		printk("\n--%s--pca955x chips init failed !!!\n",__FUNCTION__);
	}

	
	return 0;

fail3:
	free_irq(SIS809_IRQ_PIN,NULL);
fail2:

	input_unregister_device(input);
	input = NULL;
fail1:
	input_free_device(input);
nomem:
	kfree(ts_dev);
	return err;

}


static int __init sis809_init(void)
{ 
	return i2c_add_driver(&sis809_driver);
}
module_init(sis809_init);

static void __exit sis809_exit(void)
{
	i2c_del_driver(&sis809_driver);
}

module_exit(sis809_exit);
MODULE_DESCRIPTION ("sis809 touchscreen driver");
MODULE_AUTHOR("llx<llx@rockchip.com>");
MODULE_LICENSE("GPL");


