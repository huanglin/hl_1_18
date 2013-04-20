
/*
 * Driver for MT9M001 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/gpio.h>
#include <asm/arch-rockchip/api_i2c.h>

#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>

static const char ov7675_init_data[][2] =
{
#if 1
{0x12, 0x80},
{0x11, 0x80},
{0x2a, 0x00},
{0x2b, 0x00},
{0x92, 0x66},
{0x93, 0x00},
{0x3a, 0x0c},
{0x3D, 0xC0},
{0x12, 0x00},
{0x15, 0x00},
{0xc1, 0x7f},
{0x17, 0x13},
{0x18, 0x01},
{0x32, 0xbF},
{0x19, 0x03},
{0x1a, 0x7b},
{0x03, 0x0a},
{0x0c, 0x00},
{0x3e, 0x00},
{0x70, 0x3a},
{0x71, 0x35},
{0x72, 0x11},
{0x73, 0xf0},
{0xa2, 0x02},
{0x13, 0xe0},
{0x00, 0x00},
{0x10, 0x00},
{0x14, 0x28},
{0xa5, 0x06},
{0xab, 0x07},
{0x24, 0x58},
{0x25, 0x48},
{0x26, 0x93},
{0x9f, 0x78},
{0xa0, 0x68},
{0xa1, 0x03},
{0xa6, 0xD8},
{0xa7, 0xD8},
{0xa8, 0xf0},
{0xa9, 0x90},
{0xaa, 0x14},
{0x13, 0xe5},
{0x0e, 0x61},
{0x0f, 0x4b},
{0x16, 0x02},
{0x1e, 0x07},
{0x21, 0x02},
{0x22, 0x91},
{0x29, 0x07},
{0x33, 0x0b},
{0x35, 0x0b},
{0x37, 0x1d},
{0x38, 0x71},
{0x39, 0x2a},
{0x3c, 0x78},
{0x4d, 0x40},
{0x4e, 0x20},
{0x69, 0x00},
{0x6b, 0x0a},
{0x74, 0x10},
{0x8d, 0x4f},
{0x8e, 0x00},
{0x8f, 0x00},
{0x90, 0x00},
{0x91, 0x00},
{0x96, 0x00},
{0x9a, 0x80},
{0xb0, 0x84},
{0xb1, 0x0c},
{0xb2, 0x0e},
{0xb3, 0x82},
{0xb8, 0x0a},
{0xbb, 0xa1},
{0x0d, 0x60},
{0x42, 0x80},
{0x62, 0x00},
{0x63, 0x00},
{0x64, 0x10},
{0x65, 0x07},
{0x66, 0x05},
{0x94, 0x10},
{0x95, 0x12},
{0x7a, 0x24},
{0x7b, 0x04},
{0x7c, 0x07},
{0x7d, 0x12},
{0x7e, 0x2f},
{0x7f, 0x3f},
{0x80, 0x4d},
{0x81, 0x5a},
{0x82, 0x69},
{0x83, 0x74},
{0x84, 0x7f},
{0x85, 0x91},
{0x86, 0x9e},
{0x87, 0xbb},
{0x88, 0xd2},
{0x89, 0xe5},
{0x43, 0x0a},
{0x44, 0xf0},
{0x45, 0x34},
{0x46, 0x58},
{0x47, 0x28},
{0x48, 0x3a},
{0x59, 0x88},
{0x5a, 0x88},
{0x5b, 0xc2},
{0x5c, 0x60},
{0x5d, 0x58},
{0x5e, 0x10},
{0x6c, 0x0a},
{0x6d, 0x55},
{0x6e, 0x11},
{0x6f, 0x9e},
{0x6a, 0x40},
{0x01, 0x56},
{0x02, 0x44},
{0x13, 0xe7},
{0x4f, 0x95},
{0x50, 0x99},
{0x51, 0x04},
{0x52, 0x1a},
{0x53, 0x7f},
{0x54, 0x99},
{0x58, 0x1a},
{0x3f, 0x02},
{0x75, 0x63},
{0x76, 0xe1},
{0x4c, 0x00},
{0x77, 0x01},
{0x4b, 0x09},
{0xc9, 0x60},
{0x41, 0x38},
{0x56, 0x40},
{0x34, 0x11},
{0x3b, 0xaa},
{0xa4, 0x88},
{0x96, 0x00},
{0x97, 0x30},
{0x98, 0x20},
{0x99, 0x30},
{0x9a, 0x84},
{0x9b, 0x29},
{0x9c, 0x03},
{0x9d, 0x99},
{0x9e, 0x99},
{0x78, 0x04},
{0x79, 0x01},
{0xc8, 0xf0},
{0x79, 0x0f},
{0xc8, 0x00},
{0x79, 0x10},
{0xc8, 0x7e},
{0x79, 0x0a},
{0xc8, 0x80},
{0x79, 0x0b},
{0xc8, 0x01},
{0x79, 0x0c},
{0xc8, 0x0f},
{0x79, 0x0d},
{0xc8, 0x20},
{0x79, 0x09},
{0xc8, 0x80},
{0x79, 0x02},
{0xc8, 0xc0},
{0x79, 0x03},
{0xc8, 0x40},
{0x79, 0x05},
{0xc8, 0x30},
{0x79, 0x26}
#else
{0x12, 0x80},
{0x11, 0x00},
{0x2a, 0x00},
{0x2b, 0x00},
{0x92, 0x66},
{0x93, 0x00},
{0x3a, 0x0c},
{0x3D, 0xC0},
{0x12, 0x00},
{0x15, 0x00},
{0xc1, 0x7f},
{0x17, 0x13},
{0x18, 0x01},
{0x32, 0xbf},
{0x19, 0x03},
{0x1a, 0x7b},
{0x03, 0x0a},
{0x0c, 0x00},
{0x3e, 0x00},
{0x70, 0x3a},
{0x71, 0x35},
{0x72, 0x11},
{0x73, 0xf0},
{0xa2, 0x02},
{0x7a, 0x20},
{0x7b, 0x1c},
{0x7c, 0x28},
{0x7d, 0x3c},
{0x7e, 0x5a},
{0x7f, 0x68},
{0x80, 0x76},
{0x81, 0x80},
{0x82, 0x88},
{0x83, 0x8f},
{0x84, 0x96},
{0x85, 0xa3},
{0x86, 0xaf},
{0x87, 0xc4},
{0x88, 0xd7},
{0x89, 0xe8},
{0x13, 0xe0},
{0x00, 0x00},
{0x10, 0x00},
{0x0d, 0x40},
{0x14, 0x28},
{0xa5, 0x05},
{0xab, 0x07},
{0x24, 0x95},
{0x25, 0x33},
{0x26, 0xe3},
{0x9f, 0x78},
{0xa0, 0x68},
{0xa1, 0x0b},
{0xa6, 0xd8},
{0xa7, 0xd8},
{0xa8, 0xf0},
{0xa9, 0x90},
{0xaa, 0x94},
{0x13, 0xe5},
{0x0e, 0x61},
{0x0f, 0x4b},
{0x16, 0x02},
{0x21, 0x02},
{0x22, 0x91},
{0x29, 0x07},
{0x33, 0x03},
{0x35, 0x0b},
{0x37, 0x1c},
{0x38, 0x71},
{0x3c, 0x78},
{0x4d, 0x40},
{0x4e, 0x20},
{0x69, 0x55},
{0x6b, 0x4a},
{0x74, 0x19},
{0x8d, 0x4f},
{0x8e, 0x00},
{0x8f, 0x00},
{0x90, 0x00},
{0x91, 0x00},
{0x96, 0x00},
{0x9a, 0x80},
{0xb0, 0x8c},
{0xb1, 0x0c},
{0xb2, 0x0e},
{0xb3, 0x82},
{0xb8, 0x0a},
{0x43, 0x14},
{0x44, 0xf0},
{0x45, 0x34},
{0x46, 0x58},
{0x47, 0x28},
{0x48, 0x3a},
{0x59, 0x88},
{0x5a, 0x88},
{0x5b, 0x44},
{0x5c, 0x67},
{0x5d, 0x49},
{0x5e, 0x0e},
{0x6c, 0x0a},
{0x6d, 0x55},
{0x6e, 0x11},
{0x6f, 0x9f},
{0x6a, 0x40},
{0x01, 0x56},
{0x02, 0x44},
{0x13, 0xe7},
{0x4f, 0x95},
{0x50, 0x99},
{0x51, 0x04},
{0x52, 0x1a},
{0x53, 0x7f},
{0x54, 0x99},
{0x58, 0x1a},
{0x41, 0x08},
{0x3f, 0x04},
{0x75, 0x04},
{0x76, 0x61},
{0x4c, 0x00},
{0x77, 0x01},
{0x3d, 0xC0},
{0x4b, 0x09},
{0xc9, 0x60},
{0x41, 0x18},
{0x56, 0x40},
{0x34, 0x11},
{0x3b, 0xaa},
{0xa4, 0x88},
{0x96, 0x00},
{0x97, 0x30},
{0x98, 0x20},
{0x99, 0x20},
{0x9a, 0x84},
{0x9b, 0x29},
{0x9c, 0x03},
{0x9d, 0x99},
{0x9e, 0x99},
{0x78, 0x04},
{0x79, 0x01},
{0xc8, 0xf0},
{0x79, 0x0f},
{0xc8, 0x20},
{0x79, 0x10},
{0xc8, 0x7e},
{0x79, 0x0b},
{0xc8, 0x01},
{0x79, 0x0c},
{0xc8, 0x07},
{0x79, 0x0d},
{0xc8, 0x20},
{0x79, 0x09},
{0xc8, 0x80},
{0x79, 0x02},
{0xc8, 0xc0},
{0x79, 0x03},
{0xc8, 0x40},
{0x79, 0x05},
{0xc8, 0x30},
{0x79, 0x26},
{0x11, 0x80},
{0x13, 0xe7},
{0x00, 0x1f},
{0x14, 0x18},
{0x76, 0xe1},
{0x1e, 0x07},
{0x6b, 0x12}
#endif

};

/*640*480*/
static const char ov7675_sxga[][2]=
{       
	{0x17, 0x13},
    {0x18, 0x01},
    {0x32, 0xbf},
    {0x19, 0x03},
    {0x1a, 0x7b}, 
    {0x03, 0x0a},
    
//  {0x1e, 0x31}   //add by fjp 2010-5-25
};

/*640*480*/
static const char ov7675_vga[][2]= 
{
   /* {0x12, 0x0},
    {0x17, (158>> 3) & 0xff},
    {0x18, (14>> 3) & 0xff},
    {0x32, (0xb6 & 0xc0) | ((14 & 0x7) << 3) | (158 & 0x7)},
    {0x19, (10>> 3) & 0xff},
    {0x1a, (490>> 3) & 0xff}, 
    {0x03, (0x0a & 0xc0) | ((490 & 0x7) << 3) | (10 & 0x7)},
	*/
//  {0x1e, 0x31}   //add by fjp 2010-5-25
      {0x17, 0x13},
    {0x18, 0x01},
    {0x32, 0xbf},
    {0x19, 0x03},
    {0x1a, 0x7b},
    {0x03, 0x0a},
};

/*352*288*/
static const char ov7675_qcif[][2]= 
{
    /*{0x17, (170>> 3) & 0xff},
    {0x18, (90>> 3) & 0xff},
    {0x32, (0xb6 & 0xc0) | ((90 & 0x7) << 3) | (170 & 0x7)},
    {0x19, (14>> 3) & 0xff},
    {0x1a, (494>> 3) & 0xff}, 
    {0x03, (0x0a & 0xc0) | ((494 & 0x7) << 3) | (14 & 0x7)},
*/
//  {0x1e, 0x31}   //add by fjp 2010-5-25
      {0x17, 0x13},
    {0x18, 0x01},
    {0x32, 0xbf},
    {0x19, 0x03},
    {0x1a, 0x7b},
    {0x03, 0x0a},
};

/*320*240*/
static const char ov7675_qvga[][2]=
{
    {0x17, 0x13},
    {0x18, 0x01},
    {0x32, 0xbf},
    {0x19, 0x03},
    {0x1a, 0x7b}, 
    {0x03, 0x0a},
	//  {0x1e, 0x31}   //add by fjp 2010-5-25
};



static const struct soc_camera_data_format ov7675_colour_formats [] = 
{
    {
        /* Order important: first natively supported,
         * second supported with a GPIO extender */
        .name		= "ov7675 YUV420",
        .depth		= 16,	// old depth is 8,  in rk28_caemra, videobuf_setup should change accordingly
        .fourcc		= V4L2_PIX_FMT_YUV420,
     },

    {
        .name = "ov7675 YUV422P",
        .depth = 16,
        .fourcc = V4L2_PIX_FMT_YUV422P,
    },
};

struct ov7675 {
	struct i2c_client *client;
	struct soc_camera_device icd;
	int model;	/* V4L2_IDENT_MT9M001* codes from v4l2-chip-ident.h */
	int switch_gpio;
	unsigned char autoexposure;
	unsigned char datawidth;
};

#define OV7675_IIC_ADDR 	    0x42 

static unsigned short normal_i2c[] = {OV7675_IIC_ADDR >> 1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data_ov7675 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

static int ov7675_probe(struct i2c_adapter *adapter, int addr, int kind);
static int ov7675_video_probe(struct soc_camera_device *);
static void ov7675_video_remove(struct soc_camera_device *);
static int ov7675_get_control(struct soc_camera_device *, struct v4l2_control *);
static int ov7675_set_control(struct soc_camera_device *, struct v4l2_control *);


static int ov7675_rx_data(struct i2c_client *this_client, char *rxData, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 1,
		 .len = length,
		 .buf = rxData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msgs, 1) < 0) {
		printk(KERN_ERR "ov7675 ov7675_rx_data: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int ov7675_tx_data(struct i2c_client *this_client, char *txData, int length)
{

	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "ov7675 ov7675_tx_data: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int ov7675_init(struct soc_camera_device *icd)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);
    int i;

    for (i = 0; i < sizeof(ov7675_init_data) / 2; i++) {
        if (0 > ov7675_tx_data(ov7675->client, &ov7675_init_data[i][0], 2))
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
    }

    char chekid[2];
    chekid[0] = 0x0a;
    ov7675_rx_data(ov7675->client, &chekid[0], 1);
    printk("\n%s..%s..%d    ******** nzy *********0x%x\n",__FUNCTION__,__FILE__,__LINE__, chekid[0]);

	return 0;
}

static int ov7675_release(struct soc_camera_device *icd)
{
	return 0;
}

static int ov7675_start_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int ov7675_stop_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int bus_switch_request(struct ov7675 *ov7675,
			      struct soc_camera_link *icl)
{
    /*
	unsigned int gpio = icl->gpio;

    int ret = gpio_request(gpio, "ov7675");
    if (ret < 0) {
        dev_err(&ov7675->client->dev, "Cannot get GPIO %u\n",
            gpio);
        return ret;
    }

    ret = gpio_direction_output(gpio, 0);
    if (ret < 0) {
        dev_err(&ov7675->client->dev,
            "Cannot set GPIO %u to output\n", gpio);
        gpio_free(gpio);
        return ret;
    }
	ov7675->switch_gpio = gpio;
    */

    rockchip_mux_api_set(SENSOR_PWDN_IOMUX_PINNAME, SENSOR_PWDN_IOMUX_PINDIR);
    gpio_direction_output(SENSOR_PWDN_IOPIN, GPIO_OUT);
    __gpio_set(SENSOR_PWDN_IOPIN, GPIO_LOW);
    
    
	return 0;
}

static void bus_switch_release(struct ov7675 *ov7675)
{

}

static int ov7675_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);
	int ret;

	return 0;
}

static unsigned long ov7675_query_bus_param(struct soc_camera_device *icd)
{
	/* 0v7675 has all capture_format parameters fixed */
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_SENSOR_UYVY;
}

static int ov7675_set_fmt_cap(struct soc_camera_device *icd,
		__u32 pixfmt, struct v4l2_rect *rect)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);
	int ret;
    int i;
    char *ov7675_win;
    unsigned int ov7675_win_data_len;

    if ((rect->width == icd->width) && (rect->height == icd->height)) { 
        printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__,rect->width,rect->height);
        return 0;
    }

#if 1    
    if ((rect->width <= 320) && (rect->height <= 240)) {
        ov7675_win = ov7675_qvga;
        ov7675_win_data_len = sizeof(ov7675_qvga);
    } else if ((rect->width <= 352) && (rect->height <= 288)) {
        ov7675_win = ov7675_qcif;
        ov7675_win_data_len = sizeof(ov7675_qcif);
    } else if ((rect->width <= 640) && (rect->height <= 480)){
#else
    if ((rect->width <= 640) && (rect->height <= 480)){
#endif    
        ov7675_win = ov7675_vga;
        ov7675_win_data_len = sizeof(ov7675_vga);
    } else {
        ov7675_win = ov7675_sxga;
        ov7675_win_data_len = sizeof(ov7675_sxga);
    }
    
    for (i = 0; i < ov7675_win_data_len / 2; i++) {
        ret = ov7675_tx_data(ov7675->client, ov7675_win + 2*i, 2);
        if (0 > ret) {
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
            return ret;
        }
    }
    
    //printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);

	return 0;
}

static int ov7675_try_fmt_cap(struct soc_camera_device *icd,
			       struct v4l2_format *f)
{
	if (f->fmt.pix.height < 32 + icd->y_skip_top)
		f->fmt.pix.height = 32 + icd->y_skip_top;
	if (f->fmt.pix.height > 480 + icd->y_skip_top)
		f->fmt.pix.height = 480 + icd->y_skip_top;
	if (f->fmt.pix.width < 48)
		f->fmt.pix.width = 48;
	if (f->fmt.pix.width > 640)
		f->fmt.pix.width = 640;
	f->fmt.pix.width &= ~0x01; /* has to be even, unsure why was ~3 */

	return 0;
}

static int ov7675_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_chip_ident *id)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);

	if (id->match_type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match_chip != ov7675->client->addr)
		return -ENODEV;

	id->ident	= ov7675->model;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov7675_get_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != ov7675->client->addr)
		return -ENODEV;

    char reg = reg->reg;
    int ret = ov7675_rx_data(ov7675->client, &reg, 1);
    if (!ret)
        reg->val = reg;

	return ret;
}

static int ov7675_set_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != ov7675->client->addr)
		return -ENODEV;

    char reg[2];
    reg[0] = reg->reg;
    reg[1] = reg->val;
    int ret = ov7675_tx_data(ov7675->client, reg, 2);

	return ret;
}
#endif

static const struct v4l2_queryctrl ov7675_controls[] = {
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain",
		.minimum	= 0,
		.maximum	= 127,
		.step		= 1,
		.default_value	= 64,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= 1,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 255,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Exposure",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}
};

static int ov7675_get_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);
	int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

static struct soc_camera_ops ov7675_ops = {
	.owner			= THIS_MODULE,
	.probe			= ov7675_video_probe,
	.remove			= ov7675_video_remove,
	.init			= ov7675_init,
	.release		= ov7675_release,
	.start_capture		= ov7675_start_capture,
	.stop_capture		= ov7675_stop_capture,
	.set_fmt_cap		= ov7675_set_fmt_cap,
	.try_fmt_cap		= ov7675_try_fmt_cap,
	.set_bus_param		= ov7675_set_bus_param,
	.query_bus_param	= ov7675_query_bus_param,
	.controls		= ov7675_controls,
	.num_controls		= ARRAY_SIZE(ov7675_controls),
	.get_control		= ov7675_get_control,
	.set_control		= ov7675_set_control,
	.get_chip_id		= ov7675_get_chip_id,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= ov7675_get_register,
	.set_register		= ov7675_set_register,
#endif
};

static int ov7675_set_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);
	const struct v4l2_queryctrl *qctrl;
	int data;

	qctrl = soc_camera_find_qctrl(&ov7675_ops, ctrl->id);

	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int ov7675_video_probe(struct soc_camera_device *icd)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);
	s32 data;
	int ret;

	/* We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant. */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

    ov7675->model = V4L2_IDENT_OV7675;
    icd->formats = &ov7675_colour_formats;
    icd->num_formats = ARRAY_SIZE(ov7675_colour_formats);

	/* Now that we know the model, we can start video */
	ret = soc_camera_video_start(icd);
	if (ret)
		goto eisis;

	return 0;

eisis:
ei2c:
	return ret;
}

static void ov7675_video_remove(struct soc_camera_device *icd)
{
	struct ov7675 *ov7675 = container_of(icd, struct ov7675, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", ov7675->client->addr,
		ov7675->icd.dev.parent, ov7675->icd.vdev);
	soc_camera_video_stop(&ov7675->icd);
}

static int ov7675_remove(struct i2c_client *client)
{
	struct ov7675 *ov7675 = i2c_get_clientdata(client);

	soc_camera_device_unregister(&ov7675->icd);
	kfree(ov7675);

	return 0;
}

static int ov7675_detach_client(struct i2c_client *client)
{
	ov7675_remove(client);

	return i2c_detach_client(client);
}

static int ov7675_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data_ov7675, ov7675_probe);
}

static struct i2c_driver ov7675_driver = {
	.driver = {
		.name = "ov7675",
	    },
	.id 	= OV7675_IIC_ADDR,
	.attach_adapter = &ov7675_attach_adapter,
	.detach_client  = &ov7675_detach_client,
};

static struct soc_camera_link iclink = {//nzy add
    .bus_id = 33, /* Must match with the camera ID above */
    .gpio   = 1,
};
    
static int ov7675_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct ov7675 *ov7675;
	struct soc_camera_device *icd;
	struct soc_camera_link *icl;
	struct i2c_client *client = NULL;
	int ret, i;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		ret = -EIO;
        goto exit_check_functionality_failed;
	}

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client) {
		ret = -ENOMEM;
        goto exit_alloc_client_failed;
	}

	strlcpy(client->name, "ov7675", I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &ov7675_driver;
	client->addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 100;	
	ret = i2c_attach_client(client);
	if (ret) {
        goto exit_i2c_attach_client_failed;
	}
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	
	ov7675 = kzalloc(sizeof(struct ov7675), GFP_KERNEL);
	if (!ov7675) {
		ret = -ENOMEM;
        goto exit_alloc_data_failed;
	}

	ov7675->client = client;
	i2c_set_clientdata(client, ov7675);
    //icl = &iclink;//client->dev.platform_data;
    
	/* Second stage probe - when a capture adapter is there */
	icd = &ov7675->icd;
	icd->ops	= &ov7675_ops;
	icd->control	= &client->dev;
	icd->x_min	= 0;
	icd->y_min	= 0;
	icd->x_current	= 0;
	icd->y_current	= 0;
	icd->width_min	= 48;
	icd->width_max	= 640;
	icd->height_min	= 32;
	icd->height_max	= 480;
	icd->y_skip_top	= 0;
	icd->iface	= 33;//icl->bus_id;
	/* Default datawidth - this is the only width this camera (normally)
	 * supports. It is only with extra logic that it can support
	 * other widths. Therefore it seems to be a sensible default. */
	ov7675->datawidth = 8;
	/* Simulated autoexposure. If enabled, we calculate shutter width
	 * ourselves in the driver based on vertical blanking and frame width */
	ov7675->autoexposure = 1;

	ret = bus_switch_request(ov7675, icl);
	if (ret)
		goto exit_bus_switch_request_failed;
		
	ret = soc_camera_device_register(icd);
	if (ret)
        goto exit_soc_camera_device_register_failed;
                
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	return 0;

exit_soc_camera_device_register_failed:
    bus_switch_release(ov7675);
exit_bus_switch_request_failed:
	kfree(ov7675);
exit_alloc_data_failed:
   i2c_detach_client(client);
exit_i2c_attach_client_failed:
	kfree(client);
exit_alloc_client_failed:
exit_check_functionality_failed:
	return ret;
}

static int __init ov7675_mod_init(void)
{
	return i2c_add_driver(&ov7675_driver);
}

static void __exit ov7675_mod_exit(void)
{
	i2c_del_driver(&ov7675_driver);
}

module_init(ov7675_mod_init);
module_exit(ov7675_mod_exit);

MODULE_DESCRIPTION("Micron OV7675 Camera driver");
MODULE_AUTHOR("nzy <kernel@rock-chips>");
MODULE_LICENSE("GPL");
