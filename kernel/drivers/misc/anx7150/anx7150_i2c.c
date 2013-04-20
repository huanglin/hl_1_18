/*
 * drivers/misc/anx7150/anx7150_i2c.c
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

#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>

#include <asm/arch/rk28_scu.h>
#include <asm/arch/rk2818_fb.h>
#include <asm/arch/api_i2c.h>
#include <asm/arch/anx7150.h>

#include "anx7150_i2c.h"
#include "anx7150_sys.h"
#include "anx7150_hw.h"

static unsigned short anx7150_normal_i2c[] = {
	ANX7150_I2C_ADDR0 >> 1, 
	ANX7150_I2C_ADDR1 >> 1, 
	I2C_CLIENT_END
};

static unsigned short anx7150_i2c_ignore[] = {I2C_CLIENT_END, I2C_CLIENT_END};

static struct i2c_client_address_data anx7150_i2c_addr_data = {
	.normal_i2c = anx7150_normal_i2c,
	.probe  = anx7150_i2c_ignore,
	.ignore = anx7150_i2c_ignore,
};

static int  anx7150_i2c_attach_adapter(struct i2c_adapter *adap);
static int  anx7150_i2c_detach_client(struct i2c_client *client);
static void anx7150_i2c_shutdown(struct i2c_client *client);

struct i2c_client g_anx7150_i2c_client0 = {
	.driver = &anx7150_i2c_driver,
	.name	= "anx7150_i2c_addr0",
};
struct i2c_client g_anx7150_i2c_client1 = {
	.driver = &anx7150_i2c_driver,
	.name	= "anx7150_i2c_addr1",
};

struct i2c_driver anx7150_i2c_driver  = {
	.driver = {
		.name  = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.id = ANX7150_I2C_ADDR0,
	.attach_adapter = &anx7150_i2c_attach_adapter,
	.detach_client 	= &anx7150_i2c_detach_client,
	.shutdown     	= &anx7150_i2c_shutdown,
};

static int anx7150_i2c_probe(struct i2c_adapter *bus, int address, int kind)
{
	int ret;
	struct i2c_client *client0 = &g_anx7150_i2c_client0;
	struct i2c_client *client1 = &g_anx7150_i2c_client1;

	client0->adapter    = bus;
	client0->addr       = ANX7150_I2C_ADDR0 >> 1;
	client0->mode       = NORMALNOSTOPMODE;
	client0->Channel    = ANX7150_I2C_CHANNEL;
	client0->addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	client0->speed      = ANX7150_I2C_SPEED;

	client1->adapter    = bus;
	client1->addr       = ANX7150_I2C_ADDR1 >> 1;
	client1->mode       = NORMALNOSTOPMODE;
	client1->Channel    = ANX7150_I2C_CHANNEL;
	client1->addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	client1->speed      = ANX7150_I2C_SPEED;
	
	ret = i2c_attach_client(client0);
	if(ret < 0)
		goto out;
	ret = i2c_attach_client(client1);
	if(ret < 0)
		goto out;

	g_anx7150_dev->anx7150_detect = ANX7150_API_DetectDevice();
	if(g_anx7150_dev->anx7150_detect){
		I("ANX7150 detected!\n");
		queue_delayed_work(g_anx7150_dev->workqueue, &g_anx7150_dev->delay_work, 200);
	}

out:
	return ret;
}

static int anx7150_i2c_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &anx7150_i2c_addr_data, anx7150_i2c_probe);
}

static int anx7150_i2c_detach_client(struct i2c_client *client)
{
	int ret;

	ret  = i2c_detach_client(&g_anx7150_i2c_client0);
	ret |= i2c_detach_client(&g_anx7150_i2c_client1);

	return ret;
}

static void anx7150_i2c_shutdown(struct i2c_client *client)
{
	D("enter anx7150_i2c_shutdown.\n");

	ANX7150_Shutdown();
}


/**
 * ANX7150_i2c_read_p0_reg - read anx7150 i2c addr0 reg
 * @reg: register address
 * @*val: values read from register
 *
 * return
 *        0-success   Negative-error code
 */
int ANX7150_i2c_read_p0_reg(unsigned char reg, unsigned char *val)
{
	unsigned char c;
	int ret = 0;
	int try = 0;

retry:
	c = reg;
	ret = i2c_master_recv(&g_anx7150_i2c_client0, &c, 1);
	if(ret == -EAGAIN && try++ < ANX7150_I2C_REPEAT_MAX)
		goto retry;

	if(ret < 0)
		return ret;

	*val = c;
	return 0;
}

/**
 * ANX7150_i2c_read_p1_reg - read anx7150 i2c addr1 reg
 * @reg: register address
 * @*val: values read from register
 *
 * return
 *        0-success   Negative-error code
 */
int ANX7150_i2c_read_p1_reg(unsigned char reg, unsigned char *val)
{
	unsigned char c;
	int ret = 0;
	int try = 0;

retry:
	c = reg;
	ret = i2c_master_recv(&g_anx7150_i2c_client1, &c, 1);
	if(ret == -EAGAIN && try++ < ANX7150_I2C_REPEAT_MAX)
		goto retry;

	if(ret < 0)
		return ret;

	*val = c;
	return 0;
}


/**
 * ANX7150_i2c_write_p0_reg - write anx7150 i2c addr0 reg
 * @reg: register address
 * @val: Value will be written to register 
 *
 * return
 *        0-success   Negative-error code
 */
int ANX7150_i2c_write_p0_reg(unsigned char reg, unsigned char val)
{
	unsigned char buf[2] = {reg, val};
	int ret = 0;
	int try = 0;

retry:
	ret = i2c_master_send(&g_anx7150_i2c_client0, buf, 2);
	if(ret == -EAGAIN && try++ < ANX7150_I2C_REPEAT_MAX)
		goto retry;

	if(ret < 0)
		return ret;
	else
		return 0;
}

/**
 * ANX7150_i2c_read_p0_reg - write anx7150 i2c addr0 reg
 * @reg: register address
 * @val: Value will be written to register
 *
 * return
 *        0-success   Negative-error code
 */
int ANX7150_i2c_write_p1_reg(unsigned char reg, unsigned char val)
{
	unsigned char buf[2] = {reg, val};
	int ret = 0;
	int try = 0;

retry:
	ret = i2c_master_send(&g_anx7150_i2c_client1, buf, 2);
	if(ret == -EAGAIN && try++ < ANX7150_I2C_REPEAT_MAX)
		goto retry;

	if(ret < 0)
		return ret;
	else
		return 0;
}


