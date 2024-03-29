/*
 * rk1000_tv.c 
 *
 * Driver for rockchip rk1000 tv control
 *  Copyright (C) 2009 lhh
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <asm-arm/uaccess.h>
#include <asm/arch/rk1000_tv.h>
#include <asm/arch/rk2818_fb.h>
#include <asm/arch/rk28_scu.h>
#include <linux/android_power.h>

#define DRV_NAME "rk1000_TVOUT"

//#define DEBUG
#ifdef DEBUG
#define D(fmt, arg...) printk("<6>%s:%d: " fmt, __FILE__, __LINE__, ##arg)
#else
#define D(fmt, arg...)
#endif
#define E(fmt, arg...) printk("<3>!!!%s:%d: " fmt, __FILE__, __LINE__, ##arg)

static const unsigned short normal_i2c[] = {
	0x84 >> 1,			/* rk1000 tv control address */
	I2C_CLIENT_END
};

I2C_CLIENT_INSMOD;			/* defines addr_data */

static volatile int rk1000_tv_output_status = RK28_LCD;

struct i2c_client *rk1000_tv_i2c_client = NULL;

static int rk1000_tv_control_probe(struct i2c_adapter *adapter, int addr, int kind);

int rk1000_tv_control_set_reg(struct i2c_client *client, u8 reg, u8 const buf[], u8 len)
{
    int ret;
	u8 i2c_buf[8];
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};

	D("reg = 0x%.2X,value = 0x%.2X\n", reg, buf[0]);
	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);
	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	
	return ret;
}

int rk1000_tv_write_block(u8 addr, u8 *buf, u8 len)
{
	int i;
	int ret = 0;
	
	if(rk1000_tv_i2c_client == NULL){
		printk("<3>rk1000_tv_i2c_client not init!\n");
		return -1;
	}

	for(i=0; i<len; i++){
		ret = rk1000_tv_control_set_reg(rk1000_tv_i2c_client, addr+i, buf+i, 1);
		if(ret != 0){
			printk("<3>rk1000_tv_control_set_reg err, addr=0x%.x, val=0x%.x", addr+i, buf[i]);
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(rk1000_tv_write_block);

static int rk1000_tv_control_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, rk1000_tv_control_probe);
}

static int rk1000_tv_control_detach_client(struct i2c_client *client)
{
	return i2c_detach_client(client);
}

static struct i2c_driver rk1000_tv_control_driver = {
	.driver 	= {
		.name	= DRV_NAME,
	},
	.id 	= I2C_DRIVERID_RK1000_TVOUT,
	.attach_adapter = rk1000_tv_control_attach_adapter,
	.detach_client	= rk1000_tv_control_detach_client,
};

static int rk1000_tv_control_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	int rc = 0;
	u8 buff;
	struct i2c_client *client = NULL;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		rc = -ENODEV;
		goto failout;
	}
	
	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (client == NULL) {
		rc = -ENOMEM;
		goto failout;
	}
	
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &rk1000_tv_control_driver;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH0;
	client->speed = 150;
	client->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	strlcpy(client->name, DRV_NAME, I2C_NAME_SIZE);

	rc = i2c_attach_client(client);
	if (rc < 0)
		goto failout;

	rk1000_tv_i2c_client = client;
	
    //standby tv 
    buff = 0x07;  
    rk1000_tv_control_set_reg(client, 0x03, &buff, 1);
	return 0;

failout:
	kfree(client);
	return rc;
}

/* drivers/video/rk28_fb.c */
extern int rk2818fb_set_cur_screen(int type);

/* drivers/video/rk28_backlight.c */
extern void rk28_bl_suspend(void *);
extern void rk28_bl_resume(void *);

int rk28_tvout_fb_set_par(void)
{
	int ret;
	struct fb_info *win0_info = registered_fb[1];
	struct fb_info *win1_info = registered_fb[0];
	struct fb_var_screeninfo var;

	var = win0_info->var;
	var.activate |= FB_ACTIVATE_FORCE; //makesure update win0 par
	acquire_console_sem();
	win0_info->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(win0_info, &var);
	win0_info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();

	if(ret < 0){
		E("fb_set_var err!\n");
	}

	var = win1_info->var;
	var.activate |= FB_ACTIVATE_FORCE; //makesure update win1 par
	acquire_console_sem();
	win1_info->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(win1_info, &var);
	win1_info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();

	if(ret < 0){
		E("fb_set_var err!\n");
	}

	return ret;
}

int rk1000_tv_set_output(int type)
{
	int ret = 0;

	D("type = %d\n", type);

	switch(type){
	case RK28_LCD:
		if(rk1000_tv_output_status == RK28_LCD){
			break;
		}
		rk1000_tv_output_status = RK28_LCD;
		rk2818fb_set_cur_screen(0);
		rk28_tvout_fb_set_par();
		rk28_bl_resume((void *)0);
		break;
	case Cvbs_NTSC:
		if(rk1000_tv_output_status == Cvbs_NTSC){
			break;
		}
		rk28_bl_suspend((void *)0);
		rk1000_tv_output_status = Cvbs_NTSC;
		rk2818fb_set_cur_screen(1);
		rk28_tvout_fb_set_par();
		break;
	case Cvbs_PAL:
		if(rk1000_tv_output_status == Cvbs_PAL){
			break;
		}
		rk28_bl_suspend((void *)0);
		rk1000_tv_output_status = Cvbs_PAL;
		rk2818fb_set_cur_screen(2);
		rk28_tvout_fb_set_par();
		break;
	case Ypbpr480:
		if(rk1000_tv_output_status == Ypbpr480){
			break;
		}
		rk28_bl_suspend((void *)0);
		rk1000_tv_output_status = Ypbpr480;
		rk2818fb_set_cur_screen(3);
		rk28_tvout_fb_set_par();
		break;
	case Ypbpr576:
		if(rk1000_tv_output_status == Ypbpr576){
			break;
		}
		rk28_bl_suspend((void *)0);
		rk1000_tv_output_status = Ypbpr576;
		rk2818fb_set_cur_screen(4);
		rk28_tvout_fb_set_par();
		break;
	case Ypbpr720:
		if(rk1000_tv_output_status == Ypbpr720){
			break;
		}
		rk28_bl_suspend((void *)0);
		rk1000_tv_output_status = Ypbpr720;
		rk2818fb_set_cur_screen(5);
		rk28_tvout_fb_set_par();
		break;
	default:
		ret = -EINVAL;
		E("Invalid type, type = %d\n", type);
		break;
	}

	return ret;
}

int rk1000_tv_get_output_status(void)
{
	return rk1000_tv_output_status;
}

int rk1000_tv_open(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t rk1000_tv_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	char c = 0;
	int ret;
	
	ret = copy_from_user(&c, buff, 1);
	if(ret < 0){
		E("copy_from_user err!\n");
		return ret;
	}

	rk1000_tv_set_output(c - '0');

	return 1;
}

int rk1000_tv_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	
	switch (cmd){
	case RK1000_TV_SET_OUTPUT:
		ret = rk1000_tv_set_output(arg);
		break;
	default:
		E("unknown ioctl cmd!\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

int rk1000_tv_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations rk1000_tv_fops = {
	.owner   = THIS_MODULE,
	.open    = rk1000_tv_open,
	.write   = rk1000_tv_write,
	.ioctl   = rk1000_tv_ioctl,
	.release = rk1000_tv_release,
};

struct miscdevice rk1000_tv_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DRV_NAME,
	.fops = &rk1000_tv_fops,
};

#ifdef CONFIG_ANDROID_POWER
static void rk1000_tv_early_suspend(android_early_suspend_t *h)
{
	D("enter\n");

	if(rk1000_tv_output_status != RK28_LCD){
		rk2818fb_set_cur_screen(0);
		rk28_tvout_fb_set_par();
		rk28_bl_resume((void *)0);
	}

}

static void rk1000_tv_early_resume(android_early_suspend_t *h)
{
	D("enter\n");

	if(rk1000_tv_output_status != RK28_LCD){
		rk28_bl_suspend((void *)0);
		rk2818fb_set_cur_screen(rk1000_tv_output_status);
		rk28_tvout_fb_set_par();
	}
}

static android_early_suspend_t rk1000_tv_early_suspend_desc = {
	.level = 0,
	.suspend = rk1000_tv_early_suspend,
	.resume = rk1000_tv_early_resume,
};
#endif


static int __init rk1000_tv_init(void)
{    
    int ret;
    ret = i2c_add_driver(&rk1000_tv_control_driver);
	if(ret < 0){
		E("i2c_add_driver err, ret = %d\n", ret);
		goto err1;
	}

	ret = misc_register(&rk1000_tv_misc_dev);
	if(ret < 0){
		E("misc_register err, ret = %d\n", ret);
		goto err2;
	}

#ifdef CONFIG_ANDROID_POWER
	android_register_early_suspend(&rk1000_tv_early_suspend_desc);
#endif

    return 0;
	
err2:
	i2c_del_driver(&rk1000_tv_control_driver);
err1:
	return ret;
}

static void __exit rk1000_tv_exit(void)
{
	misc_deregister(&rk1000_tv_misc_dev);
    i2c_del_driver(&rk1000_tv_control_driver);
}

module_init(rk1000_tv_init);
module_exit(rk1000_tv_exit);
/* Module information */
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_DESCRIPTION("ROCKCHIP rk1000 tv ");
MODULE_LICENSE("GPL");

