/*
 * TPS65180 driver
 *
 * (c) Copyright 2012 Iped digital technology company.
 *                    <chf@iped.com.cn>
 *
 * Released under the term of the GNU GPL v2.
 */
 
//#define TPS_DBG

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

#include "eink_s.h"
#include "tps65180b.h"

#ifdef TPS_DBG
#define lmprintk(args...)	printk(args)
#else
#define lmprintk(args...)	do { } while(0)
#endif


struct tps_sensor {
	int			ds1775 : 1;
	int			inited : 1;
	struct 	i2c_client	i2c;
	struct  epd_sensor	sens;
};

#define epd_to_tps(c) container_of(c, struct tps_sensor, sens)
#define i2c_to_tps(c) container_of(c, struct tps_sensor, i2c)

static	struct epd_sensor *pgepd_tps = NULL;
//static	struct 	i2c_client	*tps_i2c = NULL;

static int tps_attach(struct i2c_adapter *adapter);
static int tps_detach(struct i2c_client *client);

static struct i2c_driver tps_driver = {
	.driver = {
		.name	= "tps",
	},
	.attach_adapter	= tps_attach,
	.detach_client	= tps_detach,
};

//----------------------------------------------------------------------------------------------
extern	void	rk_epd_tps_gpio_sleep2standby(void);
extern	int	get_epd_tps_power_sleep_status(void);
//extern	void	rk_epd_tps_active2standby(void);

//=========================================================================

extern	int	tps_i2c_set_regs( unsigned char  reg, unsigned char const buf[], unsigned short  len);
extern	int	tps_i2c_read_regs(unsigned char reg, unsigned char buf[], unsigned short len);

extern	void	rk_epd_tps_gpio_poweroff(void);
//--------------------------------------------------------------------------------
int tps_i2c_set_regs( unsigned char  reg, unsigned char const buf[], unsigned short  len)
{
	int	temp = 0;
	struct tps_sensor *lm = epd_to_tps(pgepd_tps);
	
	struct i2c_client* client = &(lm->i2c);   
	
	int ret;
	unsigned char  i2c_buf[100+1];

	#if	1
	temp = 0;
	if( get_epd_tps_power_sleep_status() )	{
		lmprintk("\n \n \n \n \n\n \n \n \n \n\n  epd sleep status SET regs !!! \n \n \n \n\n \n \n \n \n\n \n \n \n \n");
		temp = 1;
		rk_epd_tps_gpio_sleep2standby();
		msleep(40);
	}
	#endif
	
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};
		//printk("\n \n tps_i2c_set_regs \n \n , %x, \n \n", tps_i2c_client->addr);


	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);
	
	ret = i2c_transfer(client->adapter, msgs, 1);

    	if(ret != msgs->len)    {
        	lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
			lmprintk("\n\n tps_i2c_set_ERR! \n\n");
			//printk("\n \n \n %s: i2c_transfer ERR ret = %d \n \n \n \n \n \n \n",__FUNCTION__, ret);
	        ret = 1;	//return 1;
	} else {
			//printk("\n set_OK \n");
		ret = 0;
	}

	if(temp)	{
		//printk("\n \n \n \n \n\n \n \n \n \n\n  epd sleep status SET regs AGAIN !!! \n \n \n \n\n \n \n \n \n\n \n \n \n \n");
		rk_epd_tps_gpio_poweroff();
	}

	return ret;
}
EXPORT_SYMBOL(tps_i2c_set_regs);

int tps_i2c_read_regs(unsigned char reg, unsigned char buf[], unsigned short len)
{

	struct tps_sensor *lm = epd_to_tps(pgepd_tps);
	struct i2c_client* client = &(lm->i2c);   
	int  	ret;
	int	temp;

	ret = 0 ;
	
  	#if	1
	temp = 0;
	if( get_epd_tps_power_sleep_status() )	{
		lmprintk("\n \n \n \n \n\n \n \n \n \n\n  epd sleep status!!! \n \n \n \n\n \n \n \n \n\n \n \n \n \n");
		temp = 1;
		rk_epd_tps_gpio_sleep2standby();
		msleep(40);
	}
	#endif
//===========
	#if	1
	struct i2c_msg msgs[1] = {
		{ client->addr, 1, len, buf },
	};
		//printk("\n \n tps_i2c_read_regs \n \n , %x, \n \n", tps_i2c_client->addr);
	
	buf[0] = reg;

	ret = i2c_transfer(client->adapter, msgs, 1);

	
    	if(ret != msgs->len)    {
        	lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
			printk("\n \n \n %s: i2c_transfer ERR ret = %d \n \n \n \n \n \n \n",__FUNCTION__, ret);
	        ret = 1;	//return 1;
	} else {
			//printk("\n read_OK \n");
		ret = 0;
	}


	if(temp)	{
		//printk("\n \n \n \n \n\n \n \n \n \n\n  epd sleep status AGAIN !!! \n \n \n \n\n \n \n \n \n\n \n \n \n \n");
		rk_epd_tps_gpio_poweroff();
	}
	
	return ret;
	
	#else
	
	struct i2c_msg msgs[2] = {
		{client->addr, 0, 1, &reg},
		{client->addr, 1, len, buf}
	};
			//printk("\n \n tps_i2c_read_regs \n \n , %x, \n \n", reg);	
			//printk("\n \n tps_i2c_read_regs \n \n , %x, \n \n", reg);	
			
		//printk("\n \n tps_i2c_read_regs \n \n , %x, \n \n", tps_i2c_client->addr);	
		//printk("\n \n tps_i2c_read_regs \n \n , %x, \n \n", tps_i2c_client->addr);			
		
	ret = i2c_transfer(client->adapter, msgs, 2) ;
	return ret;
	#endif

}
EXPORT_SYMBOL(tps_i2c_read_regs);

//=========================================================================
//----------------------------------------------------------------------------------------------

#if	0
static int tps_get(s32 *value)
{
	struct tps_sensor *lm = epd_to_tps(pgepd_tps);
	
    struct i2c_client* client = &(lm->i2c);   
    int  ret;
    u8 buf[2];
    struct i2c_msg msg[1];
    
    msg->addr = client->addr;
    msg->flags |= I2C_M_RD;
    msg->buf = buf;
    msg->len = sizeof(buf);
   
	buf[0] = 0;
	ret = i2c_transfer(client->adapter, msg, 1);
    if(ret != msg->len)
    {
        lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
        return 1;
    }
	lmprintk("\n*************tps_get:reg=%d,value=%d\n",0,buf[0]);
	*value = buf[0];   

    return 0;
}
#else




static	s32		tps_temperature = 25;
static	int	tps_get_working = 0;

#if	0
static int tps_get(s32 *value)
{
	*value = 0x19;   	// 默认25°
	return 0;
}
#else
static int tps_get(s32 *value)
{
//sets the READ_THERM bit of the TMST_CONFIG register to 1
//delay
//reading the INT_STATUS2 register until EOC  //nINT pin 在6uS后释放
//reads the temperature data from the TMST_VALUE register

	unsigned char	buf[6];
	int	i;
	int	ret;
	int	temp;
	int	err;

	ret = 0;
	
	//printk("\n \n \n \n \n\n \n \n \n \n\n  tps_get START !!! \n \n \n \n\n \n \n \n \n\n \n \n \n \n");

	if(tps_get_working)	{

		tps_get_working --;
		*value = tps_temperature;
		//printk("\n\n\n tps_get_reentry!!! \n\n\n");
		return	0;
	}
	tps_get_working = 5;
	
	
  	#if	1
	temp = 0;
	if( get_epd_tps_power_sleep_status() )	{
		lmprintk("\n \n \n \n \n\n \n \n \n \n\n  tps_get!!! \n \n \n \n\n \n \n \n \n\n \n \n \n \n");
		temp = 1;
		rk_epd_tps_gpio_sleep2standby();
		msleep(50);
	}
	#endif
//------------------------------------------------------------------------------------------

//while(1)	{
	err = 0;	
	buf[0] = 0xa0;
	ret = tps_i2c_set_regs( TPS_TMST_CONFIG,  buf, 1);
		if(ret)	{
			err = 1;
			printk("\n\n tps_get ERR!!! \n\n");
		}

	udelay(20);
	udelay(250);			//==========250-μs
	
	buf[0] = 0;
	for( i=20;   i>0 ; i--)		{
		ret = tps_i2c_read_regs(TPS_INT_STATUS2, buf, 1);
		if(ret)	{
			err = 1;
			break;
		}
		
		if(0x01 & buf[0])	{
			//err = 0;
			break;
		}
		udelay(40);
	}

	buf[0] = 25;   	// 默认25°	
	ret = tps_i2c_read_regs(TPS_TMST_VALUE, buf, 1);
		if(ret)	{
			err = 1;
		}

	if(err)	{
		*value = tps_temperature;
	} else {
		*value = buf[0];
		tps_temperature = buf[0];
	}
	
	//printk("\n\n\n  t = %d ,i= %d, err = %d ,ret = %d ,vaule=%d \n\n\n",buf[0],i,err,ret,*value);

//	msleep(10);
//}

	if(temp)	{
		//printk("\n \n \n \n \n\n \n \n \n \n\n  epd sleep status AGAIN !!! \n \n \n \n\n \n \n \n \n\n \n \n \n \n");
		rk_epd_tps_gpio_poweroff();
	}
	lmprintk(" tps_get  temparature = %d\n",buf[0]);
	tps_get_working = 0;
    	return 0;

}
#endif

#endif



// -------------------------------------------------------
#define	SPI_FLASH_VCOM_ADDR	(0x20000-2)
// save format example:  1650mV saved as  0x1650 (BCD code)
extern	int	spi_flash_vcom_value(u32 add, u8 *buf);

#if	1

int	tps_get_vcom_mv(void)
{
	u8	buf[2];
	int	ret = 1650;
	int	temp;
	
	if( spi_flash_vcom_value(SPI_FLASH_VCOM_ADDR, buf) )	{
		printk("read VCOM from spi flash ERR!!! \n \n \n");
	} else {
		if( ( (buf[0] | buf[1]) == 0 )  || ( (buf[0] & buf[1]) == 0xff )  ) {
			lmprintk(" no VCOM value  in  spi flash !!!  \n \n \n");
		}  else {
			temp = (buf[0]>>4)*10;
			temp += buf[0] & 0x0f;
			ret = temp *100;
			temp = (buf[1]>>4)*10;
			temp += buf[1] & 0x0f;
			ret += temp;
		}
	}

	//lmprintk("get VCOM  = %d \n \n \n",ret);
	return	ret;

}

#else
int	tps_get_vcom_mv(void)
{
	return	1650;
}
#endif



static	int vcom_value_mv = 1650;
static	int	tps_epd_vcom_write(void)
{
	unsigned char	buf[6];
	int	value_mv;
	int	vcom;
	unsigned char	*addr;
	
#if	1
	buf[0] = 0x23 | 0x80;		// I2C interface
	tps_i2c_set_regs( TPS_VN_ADJUST,  buf, 1);

	value_mv = 1650;
	#if	0
	value_mv = 1250;
	#else
	value_mv = vcom_value_mv;
	#endif
	
	// from -0.3 V to -2.5 V only.
	if(value_mv < 300)	{
		//value_mv = 300;
		value_mv = 1650;
	}else if(value_mv > 2500) {
		//value_mv = 2500;
		value_mv = 1650;
	}
	value_mv *= 2550;
	value_mv /= 2750;
	vcom = value_mv/10;		// step size is 2750 mV / 255 
	if( (value_mv-vcom*10)>5 )
		vcom +=1;


	buf[0] = vcom;
	tps_i2c_set_regs( TPS_VCOM_ADJUST,  buf, 1);
	
#else
	buf[0] = 0x23 ;		// VCOM output adjustment method   :  VCOM_XADJ pin
	tps_i2c_set_regs( TPS_VN_ADJUST,  buf, 1);	

	lmprintk("VCOM output adjustment method  \n \n \n \n ");
#endif

	return	buf[0];

}

int	tps_epd_vcom(void)
{
	vcom_value_mv = tps_get_vcom_mv();
	return	tps_epd_vcom_write();
}
EXPORT_SYMBOL(tps_epd_vcom);


#define	TPS_DELAY0				6
#define	TPS_VB_P_DELAY		10
#define	TPS_VB_N_DELAY		10
#define	TPS_VNEG_DELAY		5
#define	TPS_VPOS_DELAY		5
#define	TPS_VDDH_DELAY		5
#define	TPS_VEE_DELAY			5

int	tps_epd_status_clear(void)
{
	unsigned char	buf[6];
	int	i = 0;
	
	tps_i2c_read_regs(TPS_INT_STATUS1, &buf[0], 1);
	tps_i2c_read_regs(TPS_INT_STATUS2, &buf[1], 1);

	i = buf[1];
	i << 8;
	i |= buf[0];

	if( (0x44 & buf[0]) ||(0xfa & buf[1]) )		{
		msleep(10);
		
	}

	//printk("tps_epd_status_clear \n");


//---------------------------------------------------
//--- vp ---
	#if	0
	buf[0] = 0x23 ;
	tps_i2c_set_regs( TPS_VP_ADJUST,  buf, 1);	
	#endif

//----------
	#if	0		// only tps65181 can use
	buf[0] = 0x00;			// 1 – Read pointer is fixed to 0x00,   0 – read pointer is controlled through I2C
	tps_i2c_set_regs( TPS_FIX_READ_POINTER,  buf, 1);
	#endif

	
	return	i;
	
}
EXPORT_SYMBOL(tps_epd_status_clear);


int	tps_epd_sequence(void)
{
	unsigned char	buf[6];
	int	i = 0;
	
	buf[0] = 0xe4;
	tps_i2c_set_regs( TPS_PWR_SEQ0,  buf, 1);	
	buf[0] = ( (1<<4) |2 );
	tps_i2c_set_regs( TPS_PWR_SEQ1,  buf, 1);	
	buf[0] = ( (10<<4) |15 );
	tps_i2c_set_regs( TPS_PWR_SEQ2,  buf, 1);	

	return	0;
}
EXPORT_SYMBOL(tps_epd_sequence);

int	tps_epd_active(void)
{
	unsigned char	buf[6];
	int	i = 0;
	
	buf[0] = 0x9f;
	tps_i2c_set_regs( TPS_ENABLE,  buf, 1);	

	return	0;
}
EXPORT_SYMBOL(tps_epd_active);

int	tps_epd_standby(void)
{
	unsigned char	buf[6];
	int	i = 0;
	
	buf[0] = 0x5f;
	tps_i2c_set_regs( TPS_ENABLE,  buf, 1);	

	return	0;
}
EXPORT_SYMBOL(tps_epd_standby);


static	void	tps_register_print(void)
{
	unsigned char	buf[6];
	int	i;


//-------------- REVID  ---------------
/*
	0101 0000 - TPS65180 1p1
	0110 0000 - TPS65180 1p2
	0111 0000 - TPS65180B (TPS65180 1p3)
	1000 0000 - TPS65180B (TPS65180 1p4)
	0101 0001 - TPS65181 1p1
	0110 0001 - TPS65181 1p2
	0111 0001 - TPS65181B (TPS65181 1p3)
	1000 0001 - TPS65181B (TPS65181 1p4)	
*/
	tps_i2c_read_regs(TPS_REVID, buf, 1);
	switch(buf[0])	{
		case 0x50:
			printk("---TPS65180 1p1, %x \n",buf[0]);
			break;
		case 0x60:
			printk("---TPS65180 1p2, %x \n",buf[0]);
			break;
		case 0x70:
			printk("---TPS65180B 1p3, %x \n",buf[0]);
			break;
		case 0x80:
			printk("---TPS65180B 1p4, %x \n",buf[0]);
			break;
		case 0x51:
			printk("---TPS65181 1p1, %x \n",buf[0]);
			break;
		case 0x61:
			printk("---TPS65181 1p2, %x \n",buf[0]);
			break;
		case 0x71:
			printk("---TPS65181B 1p3, %x \n",buf[0]);
			break;
		case 0x81:
			printk("---TPS65181B 1p4, %x \n",buf[0]);
			break;
		default:
			printk("---unknown tps device, %x \n",buf[0]);
			break;
	}



//--------------------------------------------------------
#if	0
	buf[0] = 0;

		tps_i2c_read_regs(0, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",0, buf[0]);
		tps_i2c_read_regs(1, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",1, buf[0]);
		tps_i2c_read_regs(2, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",2, buf[0]);
		tps_i2c_read_regs(3, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",3, buf[0]);
		tps_i2c_read_regs(4, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",4, buf[0]);
		tps_i2c_read_regs(5, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",5, buf[0]);
		tps_i2c_read_regs(6, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",6, buf[0]);
		tps_i2c_read_regs(7, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",7, buf[0]);
		tps_i2c_read_regs(8, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",8, buf[0]);
		tps_i2c_read_regs(9, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",9, buf[0]);
		tps_i2c_read_regs(10, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",10, buf[0]);
		tps_i2c_read_regs(11, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",11, buf[0]);
		tps_i2c_read_regs(12, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",12, buf[0]);
		tps_i2c_read_regs(13, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",13, buf[0]);
		tps_i2c_read_regs(14, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",14, buf[0]);
		tps_i2c_read_regs(15, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",15, buf[0]);
		tps_i2c_read_regs(16, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",16, buf[0]);
		tps_i2c_read_regs(17, buf, 1);
		printk("tps-reg-read  register-%d = %x \n\n",17, buf[0]);

	printk("tps_register_print OK ! \n \n \n \n \n");
#endif

}

static void tps_release(void)
{
	struct tps_sensor *lm = epd_to_tps(pgepd_tps);

	/* check if client is registered and detach from i2c */
	if (lm->i2c.adapter) {
		i2c_detach_client(&lm->i2c);
		lm->i2c.adapter = NULL;
	}

	kfree(lm);
}

static struct epd_sensor_ops tps_ops = {
	.get_value	= tps_get,
	.release	= tps_release,
	.shutdown 	= NULL,
	.wakeup 		= NULL,
	.owner		= THIS_MODULE,
};


extern	int	set_epd_tps_ram_lost(void);
static struct tps_sensor *tps_create(struct i2c_adapter *adapter,
					     u8 addr, int ds1775,
					     const char *loc)
{
	struct tps_sensor *lm;
	int rc;

	lmprintk("tps: creating  %s device at address 0x%x\n",
	    ds1775 ? "ds1775" : "tps", addr);

	lm = kzalloc(sizeof(struct tps_sensor), GFP_KERNEL);
	if (lm == NULL)
		return NULL;
	
	lm->sens.name = "tps";
	lm->inited = 1;
	lm->sens.ops = &tps_ops;
	lm->ds1775 = ds1775;
	lm->i2c.addr = addr;
	lm->i2c.adapter = adapter;
	lm->i2c.driver = &tps_driver;
    	//lm->i2c.Channel = I2C_CH0;	 // 
    	lm->i2c.Channel = I2C_CH1; 	//for  RK2818	SDK
    	
    	lm->i2c.speed = 200;
   	//lm->i2c.speed = 50;		
    	lm->i2c.mode = NORMALMODE;
    
	strncpy(lm->i2c.name, lm->sens.name, I2C_NAME_SIZE-1);

	//tps_i2c = &lm->i2c;				//*****************
    
	rc = i2c_attach_client(&lm->i2c);
	if (rc) {
		lmprintk(KERN_ERR "failed to attach %s %s to i2c,"
		       " err %d\n", ds1775 ? "ds1775" : "tps",
		       lm->i2c.name, rc);
		goto fail;
	}

	if (epd_register_sensor(&lm->sens)) {
		i2c_detach_client(&lm->i2c);
		goto fail;
	}
	pgepd_tps = &lm->sens;

//--- vcom ---
	vcom_value_mv = 1650;
	tps_epd_vcom_write();
	lmprintk("set VCOM default value 1650 !!! \n \n \n");
			
	tps_epd_sequence();	
	tps_epd_status_clear();
	
	tps_register_print();
	//rk_epd_tps_active2standby();
	set_epd_tps_ram_lost();	//假装没有power on
	
	return lm;
 fail:
	kfree(lm);
	return NULL;
}

//static unsigned short normal_i2c[] = {0x90 >> 1, I2C_CLIENT_END};
static unsigned short normal_i2c[] = {0x48, I2C_CLIENT_END};		// Slave Address 0x48h (1001000)
static unsigned short ignore = I2C_CLIENT_END;

static struct i2c_client_address_data addr_data_tps = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};



static int tps_probe(struct i2c_adapter *adapter, int addr, int kind)
{   
	int ret;
	lmprintk("tps: tps_probe at address 0x%x\n",addr);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
	    ret = -EIO;
            return ret;
	}

	tps_create(adapter, addr, 0, NULL);		
	
	return 0;
}

static int tps_attach(struct i2c_adapter *adapter)
{

	lmprintk("\n \n \n ----- EPD INIT TEMPARATURE SENSOR !!!!  \n \n \n" );
	rk_epd_tps_gpio_sleep2standby();		// i2c 操作需要在退出sleep状态之后
	msleep(40);

    lmprintk("tps: tps_attach\n");
    return i2c_probe(adapter, &addr_data_tps, tps_probe);
}

static int tps_detach(struct i2c_client *client)
{
	struct tps_sensor *lm = i2c_to_tps(client);

	lmprintk("tps: i2c detatch called for %s\n", lm->sens.name);

	/* Mark client detached */
	lm->i2c.adapter = NULL;			// ?

	/* release sensor */
	epd_unregister_sensor(&lm->sens);
	#if	0
	//i2c_detach_client(&lm->i2c);		// tps_release  已经做了这些
	//kfree(lm);
	#endif
	
	return 0;
}

static int __init tps_sensor_init(void)
{
	return i2c_add_driver(&tps_driver);
}

static void __exit tps_sensor_exit(void)
{
	i2c_del_driver(&tps_driver);
}

//subsys_initcall_sync(tps_sensor_init);

//module_init(tps_sensor_init);
//fs_initcall_sync(tps_sensor_init);
subsys_initcall(tps_sensor_init);
//subsys_initcall_sync(tps_sensor_init);
//fs_initcall(tps_sensor_init);

module_exit(tps_sensor_exit);

MODULE_AUTHOR("iped");
MODULE_DESCRIPTION("tps sensor objects for PowerMacs thermal control");
MODULE_LICENSE("GPL");

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_lm_show(struct seq_file *s, void *v)
{
    u32 value=0;
    tps_get(&value);
    
	seq_printf(s, "\nTemperature is:");
	seq_printf(s, " %d °C\n", value);

	return 0;
}

static int proc_lm_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_lm_show, NULL);
}

static const struct file_operations proc_lm_fops = {
	.open		= proc_lm_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init lm_proc_init(void)
{
	proc_create("tps", 0, NULL, &proc_lm_fops);
	return 0;

}
late_initcall(lm_proc_init);
#endif /* CONFIG_PROC_FS */



