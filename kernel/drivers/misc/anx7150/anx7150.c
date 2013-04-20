/*
 * drivers/misc/anx7150/anx7150.c
 *
 * Drivers for rk28 hdmi
 *
 * Copyright (C) 2010 rockchip
 * zyy@rock-chips.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/android_power.h>
#include <linux/workqueue.h>
#include <linux/console.h>
#include <linux/fb.h>
#include <asm/uaccess.h>

#include <asm/arch/rk28_scu.h>
#include <asm/arch/rk2818_fb.h>
#include <asm/arch/anx7150.h>

#include "anx7150_i2c.h"
#include "anx7150_sys.h"
#include "anx7150_hw.h"

/*
 * debug_level
 * 0-I message  1-Include debug message
 */
int g_anx7150_debug_level = 0;

struct anx7150_dev_s *g_anx7150_dev = NULL;

/************************ rk28 hdmi切换时音视频操作函数 *********************************/
extern int hdmi_codec_mute(int mute);
extern int hdmi_codec_set_clk(int mclk, int rate);

/* drivers/video/fbmem.c */
extern int fb_set_var(struct fb_info *info, struct fb_var_screeninfo *var);

/* drivers/video/rk2818_fb.c */
extern int rk2818fb_set_cur_screen(int type);

/* drivers/video/rk28_backlight.c */
extern void rk28_bl_suspend(android_early_suspend_t *h);
extern void rk28_bl_resume(android_early_suspend_t *h);

int rk28_hdmi_fb_set_par(void)
{
	int ret;
	struct fb_info *win0_info = registered_fb[1];
	struct fb_info *win1_info = registered_fb[0];
	struct fb_var_screeninfo var;

	var = win0_info->var;
	var.activate |= FB_ACTIVATE_FORCE; //make sure update win0 par
	acquire_console_sem();
	win0_info->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(win0_info, &var);
	win0_info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();

	if(ret < 0){
		E("win0 fb_set_var err!\n");
	}

	var = win1_info->var;
	var.activate |= FB_ACTIVATE_FORCE; //make sure update win1 par
	acquire_console_sem();
	win1_info->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(win1_info, &var);
	win1_info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();

	if(ret < 0){
		E("win1 fb_set_var err!\n");
	}

	return ret;
}

void rk28_hdmi_enter(struct anx7150_dev_s *dev)
{
	if(dev->rk28_output_status == RK28_OUTPUT_STATUS_HDMI){
		if(dev->parameter_config == 0)
			return;
	}
	else{
		dev->rk28_output_status = RK28_OUTPUT_STATUS_HDMI;
	}

	D("enter\n");

	//关闭LCD背光
	rk28_bl_suspend((void *)0);

	//设置fb
	switch(dev->resolution_real){
	case HDMI_1280x720p_50Hz:
		rk2818fb_set_cur_screen(7);
		rk28_hdmi_fb_set_par();
		break;
	case HDMI_1280x720p_60Hz:
		rk2818fb_set_cur_screen(6);
		rk28_hdmi_fb_set_par();
		break;
	case HDMI_720x576p_50Hz:
		rk2818fb_set_cur_screen(8);
		rk28_hdmi_fb_set_par();
		break;
	default:
		rk2818fb_set_cur_screen(7);
		rk28_hdmi_fb_set_par();
		break;
	}

	hdmi_codec_mute(1);
	hdmi_codec_set_clk(12288000, 48000);
	//hdmi_codec_set_clk(11289600, 44100);
}

void rk28_hdmi_exit(struct anx7150_dev_s *dev)
{
	if(dev->rk28_output_status == RK28_OUTPUT_STATUS_LCD)
		return;
	else
		dev->rk28_output_status = RK28_OUTPUT_STATUS_LCD;

	D("enter\n");

	//set i2s usb mode
    //hdmi_codec_set_clk(12000000, 44100);
	hdmi_codec_set_clk(12000000, 48000);
	hdmi_codec_mute(0);

	//切回LCD显示
	rk2818fb_set_cur_screen(0);
	rk28_hdmi_fb_set_par();

	//打开LCD背光
	rk28_bl_resume((void *)0);

}

/**********************************************************************/

void anx7150_notifier_callback(struct anx7150_dev_s *dev)
{
	I("reciver_status: %d\n", dev->reciver_status);

	kill_fasync(&dev->async_queue, SIGIO, POLL_MSG);
}

int anx7150_get_output_status(void)
{
	return g_anx7150_dev->rk28_output_status;
}

//#define HDMI_TEST
#ifdef HDMI_TEST
int test_cnt = 0;
#endif
void anx7150_work_func(struct work_struct * work)
{
	struct anx7150_dev_s *dev = container_of((void *)work, struct anx7150_dev_s, delay_work);
	//D("enter\n");

	ANX7150_Task(dev);

	if(dev->hdmi_auto_switch){
		if(dev->HPD_status == HDMI_RECIVER_PLUG){
			rk28_hdmi_enter(dev);
		}
		else{
			rk28_hdmi_exit(dev);
		}
	}
	else{
		if(dev->hdmi_enable){
			rk28_hdmi_enter(dev);
		}
		else{
			rk28_hdmi_exit(dev);
		}
	}

#ifdef HDMI_TEST
	if(test_cnt++ > 50){
			if(dev->hdmi_enable == HDMI_ENABLE)
					dev->hdmi_enable = HDMI_DISABLE;
			else
					dev->hdmi_enable = HDMI_ENABLE;

			test_cnt = 0;
	}
#endif

	if(dev->anx7150_detect){
		queue_delayed_work(dev->workqueue, &dev->delay_work, dev->rate);
	}
	else{
		I("ANX7150 not exist!\n");
		rk28_hdmi_exit(dev);
	}
}

int anx7150_open(struct inode *inode, struct file *filp)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;

	filp->private_data = dev;
	
	return 0;
}

ssize_t anx7150_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	struct anx7150_dev_s *dev = filp->private_data;
	char c = 0;
	int ret;
	
	ret = copy_from_user(&c, buff, 1);
	if(ret < 0){
		E("copy_from_user err!\n");
		return ret;
	}

	if(c == '1'){
		E("hdmi on\n");
		dev->hdmi_enable = HDMI_ENABLE;
	}
	else if(c == '0'){
		E("hdmi off\n");
		dev->hdmi_enable = HDMI_DISABLE;
	}

	return 1;
}

static int anx7150_fasync(int fd, struct file *filp, int mode)
{
	struct anx7150_dev_s *dev = filp->private_data;

	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

int anx7150_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct anx7150_dev_s *dev = filp->private_data;
	int ret = 0;

	switch (cmd){
	case IOCTL_ANX7150_ENABLE:
		break;
	case IOCTL_ANX7150_DISABLE:
		break;
	case IOCTL_HDMI_OUTPUT_ENABLE:
		dev->hdmi_enable = HDMI_ENABLE;
		break;
		
	case IOCTL_HDMI_OUTPUT_DISABLE:
		dev->hdmi_enable = HDMI_DISABLE;
		break;
		
	case IOCTL_HDMI_OUTPUT_RESOLUTION:
		dev->resolution_set = (int)arg;
		break;
		
	case IOCTL_HDMI_CONNECT_STATUS:
		ret = dev->reciver_status;
		ret = put_user(ret, (int *)arg);
		if(ret < 0){
			E("put_user err!\n");
		}
		break;
		
	default:
		E("unknown ioctl cmd!\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

int anx7150_release(struct inode *inode, struct file *filp)
{	
	anx7150_fasync(-1, filp, 0);

	return 0;
}

static struct file_operations anx7150_fops = {
	.owner   = THIS_MODULE,
	.open    = anx7150_open,
	.write   = anx7150_write,
	.ioctl   = anx7150_ioctl,
	.fasync  = anx7150_fasync,
	.release = anx7150_release,
};

struct miscdevice g_anx7150_mdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &anx7150_fops,
};

#ifdef CONFIG_ANDROID_POWER
static void anx7150_early_suspend(android_early_suspend_t *h)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	D("enter\n");

	if(dev->anx7150_detect){
		cancel_delayed_work_sync((void *)&dev->delay_work);

		if(dev->rk28_output_status == RK28_OUTPUT_STATUS_HDMI){
			ANX7150_Shutdown();
			rk28_hdmi_exit(dev);
		}
	}
}

static void anx7150_early_resume(android_early_suspend_t *h)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	D("enter\n");

	if(dev->anx7150_detect){
		queue_delayed_work(dev->workqueue, &dev->delay_work, 0);
	}
}

static android_early_suspend_t anx7150_early_suspend_desc = {
	.level = 0,
	.suspend = anx7150_early_suspend,
	.resume = anx7150_early_resume,
};
#endif

static ssize_t debug_msg_store(struct class *cls, const char *_buf, size_t _count)
{
	int debug_level = simple_strtoul(_buf, NULL, 16);

	if(debug_level == 1){
		I("debug_msg enable\n");
		g_anx7150_debug_level = debug_level;
	}
	else if(debug_level == 0){
		I("debug_msg disable\n");
		g_anx7150_debug_level = debug_level;
	}
	else{
		I("invalid value, 1-enable 0-disable\n");
	}

	return _count;
}

static ssize_t debug_msg_show(struct class *cls, char *_buf)
{
	I("debug: %d\n", g_anx7150_debug_level);
	I("Usage: 1-enable, 0-disable\n");

	return sprintf(_buf, "%d\n", g_anx7150_debug_level);
}

static ssize_t hdmi_enable_store(struct class *cls, const char *_buf, size_t _count)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int enable = simple_strtoul(_buf, NULL, 16);

	switch(enable){
	case HDMI_ENABLE:
		I("hdmi enable\n");
		dev->hdmi_enable = enable;
		break;
	case HDMI_DISABLE:
		I("hdmi disable\n");
		dev->hdmi_enable = enable;
		break;
	default:
		E("Invalid cmd, Usage: 1-hdmi enable, 0-hdmi disable\n");
		break;
	}

	return _count;
}

static ssize_t hdmi_enable_show(struct class *cls, char *_buf)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;

	I("hdmi cur state: %s\n", (dev->hdmi_enable) ? "enable" : "disable");
	I("Usage: 1-hdmi enable, 0-hdmi disable\n");

	return sprintf(_buf, "%d\n", dev->hdmi_enable);
}

static ssize_t hdmi_auto_switch_store(struct class *cls, const char *_buf, size_t _count)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int enable = simple_strtoul(_buf, NULL, 16);

	switch(enable){
	case HDMI_ENABLE:
		I("hdmi auto switch enable\n");
		dev->hdmi_auto_switch = enable;
		break;
	case HDMI_DISABLE:
		I("hdmi auto switch disable\n");
		dev->hdmi_auto_switch = enable;
		break;
	default:
		E("Invalid cmd, Usage: 1-hdmi auto switch enable, 0-hdmi auto switch disable\n");
		break;
	}

	return _count;
}

static ssize_t hdmi_auto_switch_show(struct class *cls, char *_buf)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;

	I("hdmi_auto_switch cur state: %s\n", (dev->hdmi_auto_switch) ? "enable" : "disable");
	I("Usage: 1-hdmi auto enable, 0-hdmi auto disable\n");

	return sprintf(_buf, "%d\n", dev->hdmi_auto_switch);
}

static ssize_t reciver_show(struct class *cls, char *_buf)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;

	I("reciver state: %d\n", dev->reciver_status);

	return sprintf(_buf, "%d\n", dev->reciver_status);
}

static ssize_t resolution_store(struct class *cls, const char *_buf, size_t _count)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int resolution = simple_strtoul(_buf, NULL, 16);

	switch(resolution){
	case HDMI_1280x720p_50Hz:
		I("HDMI_1280x720p_50Hz\n");
		dev->resolution_set = resolution;
		dev->parameter_config = 1;
		break;
	case HDMI_1280x720p_60Hz:
		I("HDMI_1280x720p_60Hz\n");
		dev->resolution_set = resolution;
		dev->parameter_config = 1;
		break;
	case HDMI_720x576p_50Hz:
		I("HDMI_720x576p_50Hz\n");
		dev->resolution_set = resolution;
		dev->parameter_config = 1;
		break;
	default:
		E("Invalid resolution, Usage: 0-HDMI_1280x720p_50Hz, 1-hdmi HDMI_1280x720p_60Hz, 2-HDMI_720x576p_50Hz\n");
		break;
	}

	return _count;
}

static ssize_t resolution_show(struct class *cls, char *_buf)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int resolution = dev->resolution_real;
	
	switch(resolution){
	case HDMI_1280x720p_50Hz:
		I("HDMI_1280x720p_50Hz\n");
		break;
	case HDMI_1280x720p_60Hz:
		I("HDMI_1280x720p_60Hz\n");
		break;
	case HDMI_720x576p_50Hz:
		I("HDMI_720x576p_50Hz\n");
		break;
	default:
		E("Invalid resolution\n");
		break;
	}

	I("Usage: 0-HDMI_1280x720p_50Hz, 1-hdmi HDMI_1280x720p_60Hz, 2-HDMI_720x576p_50Hz\n");

	return sprintf(_buf, "%d\n", resolution);
}

static ssize_t audio_store(struct class *cls, const char *_buf, size_t _count)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int fs = simple_strtoul(_buf, NULL, 16);

	switch(fs){
	case HDMI_I2S_Fs_44100:
		I("HDMI_I2S_Fs_44100\n");
		dev->i2s_Fs = HDMI_I2S_Fs_44100;
		dev->parameter_config = 1;
		break;
	case HDMI_I2S_Fs_48000:
		I("HDMI_I2S_Fs_48000\n");
		dev->i2s_Fs = HDMI_I2S_Fs_48000;
		dev->parameter_config = 1;
		break;
	default:
		E("Invalid Fs, Usage: 0-HDMI_I2S_Fs_44100, 2-HDMI_I2S_Fs_48000\n");
		break;
	}

	return _count;
}

static ssize_t audio_show(struct class *cls, char *_buf)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int fs = dev->i2s_Fs;

	switch(fs){
	case HDMI_I2S_Fs_44100:
		I("HDMI_I2S_Fs_44100\n");
		break;
	case HDMI_I2S_Fs_48000:
		I("HDMI_I2S_Fs_48000\n");
		break;
	default:
		E("Invalid Fs\n");
		break;
	}

	I("Usage: 0-HDMI_I2S_Fs_44100, 2-HDMI_I2S_Fs_48000\n");

	return sprintf(_buf, "%d\n", fs);
}

static ssize_t hdcp_store(struct class *cls, const char *_buf, size_t _count)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int hdcp = simple_strtoul(_buf, NULL, 16);

	switch(hdcp){
	case 0:
		I("hdcp disable\n");
		dev->hdcp_enable = HDMI_DISABLE;
		dev->parameter_config = 1;
		break;
	case 1:
		I("hdcp enable\n");
		dev->hdcp_enable = HDMI_ENABLE;
		dev->parameter_config = 1;
		break;
	default:
		E("invalid cmd, Usage: 0-hdcp disable, 1-hdcp enable\n");
		break;
	}

	return _count;
}

static ssize_t hdcp_show(struct class *cls, char *_buf)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int hdcp = dev->hdcp_enable;

	switch(hdcp){
	case HDMI_DISABLE:
		I("hdcp disable\n");
		break;
	case HDMI_ENABLE:
		I("hdcp enable\n");
		break;
	default:
		E("Invalid cmd\n");
		break;
	}

	I("Usage: 0-hdcp disable, 1-hdcp enable\n");

	return sprintf(_buf, "%d\n", hdcp);
}

static ssize_t parameter_config_store(struct class *cls, const char *_buf, size_t _count)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int parameter_config = simple_strtoul(_buf, NULL, 16);

	if(parameter_config == 1){
		I("Config hdmi parameter\n");
		dev->parameter_config = 1;
	}

	return _count;
}

static ssize_t parameter_config_show(struct class *cls, char *_buf)
{
	struct anx7150_dev_s *dev = g_anx7150_dev;
	int parameter_config = dev->hdcp_enable;

	I("Usage: 1 - re-config hdmi parameter\n");

	return sprintf(_buf, "%d\n", parameter_config);
}

static struct class *rk28_hdmi_class = NULL;
static CLASS_ATTR(debug_msg, 0666, debug_msg_show, debug_msg_store);
static CLASS_ATTR(hdmi_enable, 0666, hdmi_enable_show, hdmi_enable_store);
static CLASS_ATTR(hdmi_auto_switch, 0666, hdmi_auto_switch_show, hdmi_auto_switch_store);
static CLASS_ATTR(reciver, 0666, reciver_show, NULL);
static CLASS_ATTR(resolution, 0666, resolution_show, resolution_store);
static CLASS_ATTR(audio, 0666, audio_show, audio_store);
static CLASS_ATTR(hdcp, 0666, hdcp_show, hdcp_store);
static CLASS_ATTR(parameter_config, 0666, parameter_config_show, parameter_config_store);

static int __init anx7150_init(void)
{
	struct anx7150_dev_s *anx7150_dev;
	int ret;

	D("enter\n");

	anx7150_dev = kmalloc(sizeof(struct anx7150_dev_s), GFP_KERNEL);
	if(anx7150_dev == NULL)
		return -ENOMEM;
	memset(anx7150_dev, 0, sizeof(struct anx7150_dev_s));

	misc_register(&g_anx7150_mdev);
	anx7150_dev->mdev = &g_anx7150_mdev;

	anx7150_dev->i2c_driver = &anx7150_i2c_driver;	
	i2c_add_driver(anx7150_dev->i2c_driver);

	anx7150_dev->anx7150_detect = 0;
	anx7150_dev->resolution_set = HDMI_DEFAULT_RESOLUTION;
	anx7150_dev->i2s_Fs = HDMI_I2S_DEFAULT_Fs;
	anx7150_dev->hdmi_enable = HDMI_DISABLE;
	anx7150_dev->hdmi_auto_switch = HDMI_AUTO_SWITCH;
	anx7150_dev->reciver_status = HDMI_RECIVER_INACTIVE;
	anx7150_dev->HPD_status = HDMI_RECIVER_UNPLUG;
	anx7150_dev->HPD_change_cnt = 0;
	anx7150_dev->rk28_output_status = RK28_OUTPUT_STATUS_LCD;
	anx7150_dev->hdcp_enable = ANX7150_HDCP_EN;
	anx7150_dev->rate = 100;
	anx7150_dev->notifier_callback = (void *)anx7150_notifier_callback;

	anx7150_dev->async_queue = NULL;

	anx7150_dev->workqueue = create_singlethread_workqueue("ANX7150_WORKQUEUE");
	INIT_DELAYED_WORK(&anx7150_dev->delay_work, anx7150_work_func);

	rk28_hdmi_class = class_create(THIS_MODULE, "rk28_hdmi");
	if(rk28_hdmi_class == NULL){
		printk("create class rk28_hdmi_class failed!\n");
		goto out;
	}

	ret  = class_create_file(rk28_hdmi_class, &class_attr_debug_msg);
	ret |= class_create_file(rk28_hdmi_class, &class_attr_hdmi_enable);
	ret |= class_create_file(rk28_hdmi_class, &class_attr_hdmi_auto_switch);
	ret |= class_create_file(rk28_hdmi_class, &class_attr_reciver);
	ret |= class_create_file(rk28_hdmi_class, &class_attr_resolution);
	ret |= class_create_file(rk28_hdmi_class, &class_attr_audio);
	ret |= class_create_file(rk28_hdmi_class, &class_attr_hdcp);
	ret |= class_create_file(rk28_hdmi_class, &class_attr_parameter_config);
	if(ret != 0){
		printk("create rk281x_modem class file failed!\n");
		goto out;
	}

	g_anx7150_dev = anx7150_dev;
	
#ifdef CONFIG_ANDROID_POWER
	android_register_early_suspend(&anx7150_early_suspend_desc);
#endif
out:
	return 0;
}

static void __exit anx7150_exit(void)
{
	struct anx7150_dev_s *anx7150_dev = g_anx7150_dev;
	D("enter\n");

	if(anx7150_dev){
		destroy_workqueue(anx7150_dev->workqueue);
		i2c_del_driver(anx7150_dev->i2c_driver);
		misc_deregister(&g_anx7150_mdev);

		kfree(anx7150_dev);
		g_anx7150_dev = NULL;
	}
}

module_init(anx7150_init);
module_exit(anx7150_exit);

MODULE_DESCRIPTION ("ANX7150 driver");
MODULE_AUTHOR("zyy<zyy@rock-chips.com>");
MODULE_LICENSE("GPL");

