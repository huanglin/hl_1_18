/*
 * drivers/misc/anx7150/anx7150_i2c.h
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

#ifndef _ANX7150_I2C_H
#define _ANX7150_I2C_H

/* ANX7150 I2C Channel */
#define ANX7150_I2C_CHANNEL I2C_CH0

/* ANX7150 I2C ADDR */
#define ANX7150_I2C_ADDR0 0x72
#define ANX7150_I2C_ADDR1 0x7A

/* ANX7150 I2C SPEED */
#define ANX7150_I2C_SPEED 200

/* ANX7150 I2C Max repeat times */
#define ANX7150_I2C_REPEAT_MAX 3

extern struct i2c_driver anx7150_i2c_driver;

/* ANX7150 I2C read and write func */
int ANX7150_i2c_read_p0_reg(unsigned char reg, unsigned char *val);
int ANX7150_i2c_read_p1_reg(unsigned char reg, unsigned char *val);
int ANX7150_i2c_write_p0_reg(unsigned char reg, unsigned char val);
int ANX7150_i2c_write_p1_reg(unsigned char reg, unsigned char val);

#endif /* ANX7150_I2C_H */
