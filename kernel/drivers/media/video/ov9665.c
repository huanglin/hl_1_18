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
#include <linux/platform_device.h>

#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>
#include <linux/delay.h>


#define _CONS(a,b) a##b
#define CONS(a,b) _CONS(a,b)

#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

/* Sensor Driver Configuration */
#define SENSOR_NAME ov9650
#define SENSOR_V4L2_IDENT V4L2_IDENT_OV9665
#define SENSOR_ID 0x96
#define SENSOR_MIN_WIDTH    176
#define SENSOR_MIN_HEIGHT   144
#define SENSOR_MAX_WIDTH    1280
#define SENSOR_MAX_HEIGHT   1024
#define SENSOR_INIT_WIDTH	320			/* Sensor pixel size for sensor_init_data array */
#define SENSOR_INIT_HEIGHT  240
#define SENSOR_INIT_WINSEQADR sensor_qvga

#define CONFIG_SENSOR_WhiteBalance	1
#define CONFIG_SENSOR_Brightness		0
#define CONFIG_SENSOR_Contrast      	0
#define CONFIG_SENSOR_Saturation    	0
#define CONFIG_SENSOR_Effect         		1	// we will support this!
#define CONFIG_SENSOR_Scene         	1
#define CONFIG_SENSOR_DigitalZoom   	0
#define CONFIG_SENSOR_Focus         		0
#define CONFIG_SENSOR_Exposure      	0
#define CONFIG_SENSOR_Flash         		0
#define CONFIG_SENSOR_Mirror        		0
#define CONFIG_SENSOR_Flip          		0


#define SENSOR_NAME_STRING(a) STR(CONS(SENSOR_NAME, a))
#define SENSOR_NAME_VARFUN(a) CONS(SENSOR_NAME, a)

struct reginfo
{
    u8 reg;
    u8 val;
};
static struct  reginfo ov9665_init_data[] =
{
#if 1
 //ORG
	{0x3e,0xd0},
	//delay,0x5ms
	{0xff,0x0a},
	{0x3e,0xd0},
	//delay,0x5ms
	{0xff,0x0a},
	{0x12,0x80},
	{0xd5,0xff},
	{0xd6,0x3f},
	{0x3d,0x3c},
	{0x11,0x80},
	{0x3a,0xf1},
	{0x3b,0x00},
	{0x3c,0x58},
	{0x3e,0x50},
	{0x71,0x00},
	{0x19,0x01},
	{0x1a,0x82},
	{0x03,0x03},
	{0x2b,0x00},
	{0x0f,0x46},
	{0x55,0xa7},
	{0x58,0x83},
	{0x0c,0x3a},
	{0x59,0x0e},
	{0xdd,0x60},
	{0x56,0x40},
	{0x51,0x00},
	{0x57,0x03},
	{0x59,0x07},
	{0xdd,0xa3},
	{0x59,0x08},
	{0xdd,0x54},
	{0x52,0x30},
	{0x56,0x29},
	{0x53,0x20},
	{0x54,0x30},
	{0x59,0x00},
	{0xdd,0x10},
	{0x59,0x01},
	{0xdd,0x18},
	{0x59,0x0f},
	{0xdd,0x20},
	{0xdd,0x00},
	{0x59,0x10},
	{0xdd,0x7e},
	{0x59,0x0a},
	{0xdd,0x80},
	{0x59,0x0b},
	{0xdd,0x01},
	{0x59,0x0c},
	{0xdd,0x07},
	{0xdd,0x0f},
	{0x59,0x0d},
	{0xdd,0x20},
	{0x59,0x09},
	{0xdd,0x20},
	{0x59,0x02},
	{0xdd,0x80},
	{0x59,0x03},
	{0xdd,0x60},
	{0x59,0x04},
	{0xdd,0xf0},
	{0x59,0x05},
	{0xdd,0x80},
	{0x59,0x06},
	{0xdd,0x04},
	{0x59,0x26},
	{0x63,0x01},
	{0x8a,0x52},
	{0x96,0xf0},
	{0x97,0x06},
	{0x69,0x00},
	{0xab,0xe7},
	{0xb9,0xa0},
	{0xba,0x80},
	{0xbb,0xa0},
	{0xbc,0x80},
	{0xd7,0x00},
	{0x6a,0x24},
	{0x85,0xe7},
	{0x36,0x94},
	{0x65,0x10},
	{0x70,0x04},
	{0x71,0x5c},
	{0x11,0x81},
	{0x64,0x24},
	{0xab,0xe7},
	{0xb0,0x43},
	{0x84,0x86},
	{0xc7,0x10},
	{0xc8,0x02},
	{0xcb,0x38},
	{0xcc,0x38},
	{0xad,0x26},
	{0xd9,0x23},
	{0xda,0x01},
	{0x33,0xc8}, // 0xc0, changed to 0xc8, set reg0x33[3] = 1, jyk
	{0xbd,0x05},
	{0xbe,0x16},
	{0xbf,0x05},
	{0xc0,0x08},
	{0xc1,0x18},
	{0xc2,0x1f},
	{0xc3,0x2b},
	{0xc4,0x2b},
	{0xc5,0x01},
	{0xc6,0x98},
	{0xc7,0x10},
	{0x9b,0x04},
	{0x9c,0x07},
	{0x9d,0x10},
	{0x9e,0x28},
	{0x9f,0x36},
	{0xa0,0x44},
	{0xa1,0x52},
	{0xa2,0x60},
	{0xa3,0x6c},
	{0xa4,0x78},
	{0xa5,0x8c},
	{0xa6,0x9e},
	{0xa7,0xbb},
	{0xa8,0xd2},
	{0xa9,0xe5},
	{0xaa,0x24},
	{0x7d,0x30},
	{0x7e,0x00},
	{0x82,0x03},
	{0x7f,0x00},
	{0x83,0x07},
	{0x80,0x03},
	{0x81,0x03},
	{0x0c,0x3c},
	{0x14,0x48},
	{0x71,0x5f},
	{0x8a,0x50},
	{0x96,0xff},
	{0x97,0x00},
	{0x69,0x00},
	{0x0d,0x82},
	{0x0d,0x80},
	{0x69,0x48},
	{0x24,0x58},// or 48
	{0x25,0x52},//0r 42
	{0x26,0x62},
	{0x70,0x03},
	{0x9b,0x08},
	{0x9c,0x0e},
	{0x9d,0x1e},
	{0x9e,0x40},
	{0x9f,0x54},
	{0xa0,0x66},
	{0xa1,0x78},
	{0xa2,0x86},
	{0xa3,0x92},
	{0xa4,0x9e},
	{0xa5,0xb0},
	{0xa6,0xbe},
	{0xa7,0xd4},
	{0xa8,0xe4},
	{0xa9,0xf0},
	{0xaa,0x15},
	{0x84,0xc4},
	{0x9b,0x0e},
	{0x9c,0x1c},
	{0x9d,0x34},
	{0x9e,0x5a},
	{0x9f,0x68},
	{0xa0,0x76},
	{0xa1,0x82},
	{0xa2,0x8e},
	{0xa3,0x98},
	{0xa4,0xa0},
	{0xa5,0xb0},
	{0xa6,0xbe},
	{0xa7,0xd2},
	{0xa8,0xe2},
	{0xa9,0xee},
	{0xaa,0x18},
	{0x70,0x02},
	{0x5d,0x95},
	{0x5e,0x9b},
	{0x5f,0x21},
	{0xc7,0x90},
	{0xc8,0x06},
	{0xcb,0x50}, 
	{0xcc,0x50}, 
	{0xcf,0x00},
	{0xd0,0x20},
	{0xd1,0x00},
	{0xc7,0x98},
	{0x7c,0x1d},
	{0x65,0x11},
	{0x66,0x00},
	{0x41,0xa0},
	{0x5b,0x20},
	{0x60,0x83},
	{0x05,0x06},
	{0x03,0x03},
	{0xd2,0x10},//84
	{0x74,0xc0},
	{0x88,0xa2},
	{0x64,0x24},
	{0x43,0x00},
	{0x11,0x81},
	{0x2d,0x00},
	{0x2e,0x00},
	{0x0d,0x82},
	{0x0d,0x80},
	{0x0C,0x3C},
	{0x46,0x35},
	{0x4F,0x4F},
	{0x14,0x48},
	{0x5A,0xD7},
	{0x17,0x0c},
	{0x18,0x5d},
	{0x04,0x28}, // 0x28,  set reg0x33[3] = 1 to enable reg0x04 mirror setting, and set reg0x04[7] to 0 to disable mirror
 	{0x15,0x00},


	{0x63 ,0x00},  
	{0x12 ,0x40},  
	{0x17 ,0x0c},  
	{0x18 ,0x5d},  
	{0x19 ,0x02},  
	{0x1a ,0x3f},  
	{0x03 ,0x03},  
	{0x32 ,0xad},  
	{0x5c ,0x80},  
	{0x0d ,0x92},  
	{0x0d ,0x90},  
	{0x36 ,0xb4},  
	{0x65 ,0x11},  
	{0x70 ,0x02},  
	{0x71 ,0x9f},  
	{0x64 ,0xa4},  
	{0xab ,0xef},  
	{0xb9 ,0x50},  
	{0xba ,0x3c},  
	{0xbb ,0x28},  
	{0xbc ,0x1e},  
	{0xad ,0x82},  
	{0xd9 ,0x11},  
	{0xda ,0x00},  
	{0xae ,0x10},   
	{0x85 ,0xef},  
	{0x6a ,0x14},  
	{0xd3 ,0xa0},  

	{0x11, 0x80},
	{0x3d, 0x3c},
	{0x46, 0x6a},
	{0x4f, 0x9d},
	{0x50, 0x83},
	{0x5a, 0x34},
	{0x2a, 0x64},
	//{0x2b, 0x00},
	{0x2d, 0x0 },
	{0x2e, 0x0 },
	{0x00,0x00}		//CAUTIONS: IN ov9665, reg0x00 is set for GAIN control
#else
	//13.4 YUV Setting
	//13.4.1 VGA 
	//OV9663 refernece setting  05212007
	//24MHz  //15FPS
	//VGA  //YUV  8bit output 
	//HREF positive
	//Vsync positive
	//AEC// Auto//
	//AGC//  Auto// 16x ceiling//
	//Banding Filter//Auto//
	//AWB//  Auto//
	//LC  ON//
	//WBC  ON//
	//Gamma  ON//
	//DNS  ON//  
	//Sharpness  ON// 
	//Night mode  off//
	////
	//Reset
	{0x12, 0x80},
	//Add some dealy or wait a few seconds after register reset
	//
	//IO output
	{0xd5, 0xff},
	{0xd6, 0x3f},
	//
	//Clock 24Mhz 15 FPS
	{0x3d, 0x3c},
	{0x11, 0x81},
	{0x2a, 0x00},
	{0x2b, 0x00},
	//
	//Power control
	{0x3a, 0xd9},
	{0x3b, 0x00},
	{0x3c, 0x58},
	{0x3e, 0x50},
	{0x71, 0x00},
	//
	//Sync signal
	{0x15, 0x00},
	// 
	//Data Format YUV
	{0xd7, 0x00},//UYVY 
	{0x6a, 0x24},
	{0x85, 0xe7},
	//
	//Sample Option
	{0x63, 0x00},
	//
	//Windowing
	{0x12, 0x40},
	{0x4d, 0x09},
	{0x17, 0x0c},
	{0x18, 0x5c},
	{0x19, 0x02},
	{0x1a, 0x3f},
	{0x03, 0x03},
	{0x32, 0xb4},
	{0x2b, 0x00},
	{0x5c, 0x80},
	//
	//BLC
	{0x36, 0xb4},
	{0x65, 0x10},
	{0x70, 0x02},
	{0x71, 0x9f},
	{0x64, 0xa4},
	//
	//AEC//  Average//9 zone//
	{0x43, 0x00}, 
	{0x5d, 0x55},
	{0x5e, 0x57},
	{0x5f, 0x21},
	//
	// Brightness
	{0x24, 0x3e},
	{0x25, 0x38},
	{0x26, 0x72},
	//
	//BF auto //60Hz
	//{0x14, 0x68},
	//{0x0c, 0x3a},//38 
	//{0x4f, 0x4f},
	//{0x50, 0x42},
	//{0x5a, 0x67},
	{0x5a, 0x67},
	{0x5a, 0x67},
	//BF 50Hz
	{0x14, 0x68},
	{0x0c, 0x3c},
	{0x4f, 0x4b},
	{0x50, 0x42},
	{0x5a, 0x67},
	{0x2a, 0x00},
	{0x2b, 0x4c},//for 14.3
	//
	//LC enable Largon9306//
	{0x7d, 0x30},
	{0x7e, 0x00},
	{0x82, 0x03},
	{0x7f, 0x00},
	{0x83, 0x07},
	{0x80, 0x03},
	{0x81, 0x04},
	//
	//AWB advance Largon9306//
	{0x96, 0xf0},
	{0x97, 0x00},
	{0x92, 0x02},
	{0x94, 0x12},
	{0x93, 0x29},
	{0x95, 0x39},
	{0x91, 0x8a},
	{0x90, 0xbf},
	{0x8e, 0x41},
	{0x8f, 0x6b},
	{0x8d, 0x16},
	{0x8c, 0x0d},
	{0x8b, 0x0d},
	{0x86, 0x9e}, 
	{0x87, 0x11}, 
	{0x88, 0x22}, 
	{0x89, 0x05},  
	{0x8a, 0x03}, 
	//
	// Gamma enable
	{0x9b, 0x04},
	{0x9c, 0x10},
	{0x9d, 0x23},
	{0x9e, 0x43},
	{0x9f, 0x50},
	{0xa0, 0x5b},
	{0xa1, 0x68},
	{0xa2, 0x75},
	{0xa3, 0x7e},
	{0xa4, 0x88},
	{0xa5, 0x99},
	{0xa6, 0xa6},
	{0xa7, 0xbc},
	{0xa8, 0xd1},
	{0xa9, 0xe6},
	{0xaa, 0x22},
	//
	//De-noise enable auto
	{0xab, 0xe7},
	{0xb0, 0x43},
	{0xac, 0x04},
	{0x84, 0x40}, 
	//
	//Sharpness
	{0xad, 0x82}, 
	{0xd9, 0x11}, 
	{0xda, 0x00}, 
	{0xae, 0x10}, 
	//
	//Scaling
	{0xab, 0xe7},
	{0xb9, 0x50},
	{0xba, 0x3c},
	{0xbb, 0x50},
	{0xbc, 0x3c},
	//
	//CMX
	{0xbd, 0x8 },
	{0xbe, 0x19}, 
	{0xbf, 0x2 },
	{0xc0, 0x8}, 
	{0xc1, 0x2a}, 
	{0xc2, 0x34}, 
	{0xc3, 0x2d}, 
	{0xc4, 0x2d}, 
	{0xc5, 0x0}, 
	{0xc6, 0x98}, 
	{0xc7, 0x18}, 
	{0x69, 0x48},
	//
	//UVave
	{0x74, 0xc0},
	//
	//UVadj
	{0x7c, 0x28},
	{0x65, 0x11},
	{0x66, 0x00},
	{0x41, 0xc0}, 
	{0x5b, 0x24}, 
	{0x60, 0x82}, 
	{0x05, 0x07}, 
	{0x03, 0x03},
	{0xd2, 0x94}, 
	//
	//SAT & Brightness
	{0xc8, 0x06},
	{0xcb, 0x40},
	{0xcc, 0x40},
	{0xcf, 0x00},
	{0xd0, 0x20},
	{0xd1, 0x00},
	{0xc7, 0x18},
	//
	//BLC
	{0x72, 0xc0},
	{0x0d, 0x92},
	{0x0d, 0x90},
	//end

#endif
};

/* 1280X1024 SXGA */
static struct reginfo ov9665_sxga[]=
{
#if  0
    //2009.05.19
    //@@ SXGA 7.5FPS for Rockchip
    //OV9650/9665 refernece setting     05192009
    //24MHz         7.5FPS
    //SXGA      YUV  8bit output 
    //HREF      positive
    //Vsync     positive
    //AEC       Auto    
    //AGC       Auto    16x->8x ceiling
    //Banding Filter Auto   
    //AWB       Auto    
    //LC        ON      
    //WBC       ON 
    //Gamma         ON 
    //DNS       ON   
    //Sharpness     ON 
    //Night mode    off
    
    
    
    //Reset
    {0x12, 0x80},
    //Add some dealy or wait a few seconds after register reset
    //
    //IO output
    {0xd5, 0xff},  
    {0xd6, 0x3f},
    
    //Clock 24Mhz  
    {0x3d, 0x3c},  
    {0x11, 0x81}, //80 for 15fps
    {0x2a, 0x00},
    {0x2b, 0x00},
    // power control
    {0x3a, 0xd9},//f1->d9  
    {0x3b, 0x00},  
    {0x3c, 0x58},  
    {0x3e, 0x50}, 
    {0x71, 0x00}, 
    // 
    //Sync signal
    {0x15, 0x00}, 
    //
    //data format YUV
  //  {0xd7, 0x10}, //00->10 
  //  {0x6a, 0x24}, //2d->24
  //  {0x85, 0xe7},
    {0xd7,0x00},
	{0x6a,0x24},
	{0x85,0xe7},
    //
    //Sample Option
    {0x63, 0x01},
     
    //Windowing
    {0x12, 0x00}, //40->00    
    {0x17, 0x0d}, // C->d  set 0x0c have black line 
    {0x18, 0x5d}, //5d->5c   
    {0x19, 0x01}, //02-01    
    {0x1a, 0x82},  
    {0x03, 0x0f}, //03->0f 
    {0x32, 0x34}, //ad->34
    {0x2b, 0x00}, 
    {0x5c, 0x00}, //80->00 
    
    //BLC 
    {0x36, 0xb4},
    {0x65, 0x10}, //11->10 
    {0x70, 0x02},
    {0x71, 0x9c},
    {0x64, 0x24},
    
    //AEC/Average/9 zone
    {0x43, 0x00},
    {0x5d, 0x55},
    {0x5e, 0x57},
    {0x5f, 0x21},
    
    //Brightness
    {0x24, 0x4e},//0x3e},
    {0x25, 0x48},//0x38},
    {0x26, 0x82},//0x72},
    
    //BF auto/60hz
    {0x14, 0x68}, // 68 for 0ff,banding option on for 60
    {0x0c, 0x3a}, // 38 
    {0x4f, 0x9e}, //4f/9e
    {0x50, 0x84}, //42/84
    {0x5a, 0x67}, //df/67
    //
    //Lc enable /Largan9306
    {0x7d, 0x30},
    {0x7e, 0x00},
    {0x82, 0x03},
    {0x7f, 0x00},
    {0x83, 0x07},
    {0x80, 0x03},
    {0x81, 0x04},
    //
    //AWB advance//
    {0x96, 0xf0},
    {0x97, 0x00},
    {0x92, 0x33},
    {0x94, 0x5a},
    {0x93, 0x3a},
    {0x95, 0x48},
    {0x91, 0xfc},
    {0x90, 0xff},
    {0x8e, 0x4e},
    {0x8f, 0x4e},
    {0x8d, 0x13},
    {0x8c, 0x0c},
    {0x8b, 0x0c},
    {0x86, 0x9e},
    {0x87, 0x11},
    {0x88, 0x22},
    {0x89, 0x05},
    {0x8a, 0x03},
    //
    // Gamma enable
    {0x9b, 0x0e},
    {0x9c, 0x1c},
    {0x9d, 0x34},
    {0x9e, 0x5a},
    {0x9f, 0x68},
    {0xa0, 0x76},
    {0xa1, 0x82},
    {0xa2, 0x8e},
    {0xa3, 0x98},
    {0xa4, 0xa0},
    {0xa5, 0xb0},
    {0xa6, 0xbe},
    {0xa7, 0xd2},
    {0xa8, 0xe2},
    {0xa9, 0xee},
    {0xaa, 0x18},
    
    //De-noise enable auto
    {0xAB, 0xe7},
    {0xb0, 0x43},
    {0xac, 0x04},
    {0x84, 0x40},
    
    //Sharpness
    {0xad, 0x98},//0x84},
    {0xd9, 0x49},//0x24},
    {0xda, 0x02},//00
    {0xae, 0x10},
    //
    //Scaling
    {0xab, 0xe7},
    {0xb9, 0xa0},
    {0xba, 0x80},
    {0xbb, 0xa0},
    {0xbc, 0x80},
    
    //CMX
    {0xbd, 0x8}, 
    {0xbe, 0x19}, 
    {0xbf, 0x2}, 
    {0xc0, 0x8},  //U
    {0xc1, 0x2a}, 
    {0xc2, 0x34}, 
    {0xc3, 0x2d}, //V  2e->2d
    {0xc4, 0x2d}, 
    {0xc5, 0x0}, 
    {0xc6, 0x98}, 
    {0xc7, 0x18}, 
    {0x69, 0x48},
    //
    //UVave
    {0x74, 0xc0},
    
    //UVadj
    {0x7c, 0x28},
    {0x65, 0x11},
    {0x66, 0x00},
    {0x41, 0xc0},
    {0x5b, 0x24}, 
    {0x60, 0x82}, 
    {0x05, 0x07},
    {0x03, 0x0f},
    {0xd2, 0x94}, 
    //
    //SAT & Brightness
    {0xc8, 0x06},
    {0xcb, 0x40},
    {0xcc, 0x40},
    {0xcf, 0x00},
    {0xd0, 0x20},
    {0xd1, 0x00},
    {0xc7, 0x18},
    
    //BLC
    {0x0d, 0x82},
    {0x0d, 0x80}

#else
    //20090519 VGA change to SXGA for rockchip

	{0x63, 0x01},

	//windowing
	{0x12, 0x00},
	{0x4d, 0x11},
	{0x17, 0x0c},
	{0x18, 0x5c},
	{0x19, 0x01},
	{0x1a, 0x82},
	{0x03, 0x0f},

	{0x2b, 0x00},
	{0x32, 0x34},
	{0x5c, 0x00},
	{0x71, 0x9c},
	{0x64, 0x24},
	{0x0c, 0x3a},
	{0x4f, 0x9e},
	{0x50, 0x84},
	{0x5a, 0x67},
	//Sharpness
	{0xad, 0x98},//0x84},
	{0xd9, 0x49},//0x24},
	{0xda, 0x02},//00
	{0xae, 0x10},

	//scaling
	{0xab, 0xe7},
	{0xb9, 0xa0},
	{0xba, 0x80},
	{0xbb, 0xa0},
	{0xbc, 0x80},
	{0x03, 0x0f},
	{0x0d, 0x82},
	{0x0d, 0x80},
	{0x00,0x00}

#endif
};
//=============================

/* 800X600 SVGA*/
static struct reginfo ov9665_svga[] =
{
    {0x0, 0x0},
};

/* 640X480 VGA */
static struct reginfo ov9665_vga[] = 
{
#if 1
    //20090519 SXGA change to vGA for rockchip


	{0x63, 0x00},

	//windowing
	{0x12, 0x40},
	{0x4d, 0x09},
	{0x17, 0x0c},
	{0x18, 0x5d},
	{0x19, 0x02},
	{0x1a, 0x3f},
	{0x03, 0x03},
	{0x32, 0xad},

	{0x5c, 0x80},
	{0x0d ,0x92},
	{0x0d ,0x90},
	{0x36 ,0xb4},
	{0x65 ,0x10},
	{0x70 ,0x02},
	{0x71 ,0x9f},
	{0x64 ,0xa4},
	{0xab, 0xe7},
	{0xb9, 0x50},
	{0xba, 0x3c},
	{0xbb, 0x50},
	{0xbc, 0x3c},
	//Sharpness
	{0xad, 0x82},
	{0xd9, 0x11},
	{0xda, 0x00},
	{0xae, 0x10},
    
    //scaling
    	{0x85 ,0xe7},
	{0x6a ,0x24},
	{0xd3 ,0x06},
	{0x11, 0x80},
	{0x3d, 0x3c},
	{0x46, 0x6a},
	{0x4f, 0x9d},
	{0x50, 0x83},
	{0x5a, 0x34},
	{0x2d, 0x00},
	{0x2e, 0x00},
	{0x00,0x00}

#else
	{0xa8, 0x80},
	{0x0c, 0x04},
	{0x0d, 0x80},
	{0x11, 0x00},
	{0x6b, 0x0a},
	{0x6a, 0x3e},
	{0x12, 0x40},
	{0x18, 0xc7},
	{0x17, 0x27},
	{0x32, 0xbd},
	{0x03, 0x00},
	{0x1a, 0x3d},
	{0x19, 0x01},
	{0x39, 0x50},
	{0x38, 0x92},
	{0x35, 0x81},
	{0x92, 0x00},
	{0x93, 0x00},
	{0x2a, 0x10},
	{0x2b, 0x40}
#endif

};

/* 352X288 CIF */ //jyk, 352x288 is not using, 
static struct reginfo sensor_cif[] =
{
#if 0// sequence not used, jyk
	{0x0c ,0x04},
	{0x0d ,0x80},
	{0x11 ,0x80},
	{0x12 ,0x20},
	{0x13 ,0xe5},
	{0x18 ,0xc7},
	{0x17 ,0x27},
	{0x03 ,0x00},
	{0x1a ,0x3d},
	{0x19 ,0x01},
	{0x39 ,0x50},
	{0x38 ,0x92},
	{0x35 ,0x81},
#endif
	{0x00,0x00}
};

/* 320*240 QVGA */
static  struct reginfo sensor_qvga[] =
{
#if 0 // sequence not used, jyk
	{0x12, 0x10},
    {0xa8, 0x80},
    {0x04, 0x00},
    {0x0c, 0x04},
    {0x0d, 0x80},
    {0x18, 0xc7},
    {0x17, 0x27},
    {0x32, 0xbd},
    {0x03, 0x36},
    {0x1a, 0x1e},
    {0x19, 0x00},
    {0x11, 0x00},
    {0x6b, 0x0a},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x2a, 0x10},
    {0x2b, 0x40},
    {0x6a, 0x3e},
    {0x3b, 0x09},
    {0x00,0x00}
#endif 
    {0x00,0x00}
};

#define MIN(x,y)   ((x<y) ? x: y)
#define MAX(x,y)    ((x>y) ? x: y)

#define OV9695_MIN_WIDTH    176
#define OV9665_MIN_HEIGHT   144
#define OV9665_MAX_WIDTH    1280
#define OV9665_MAX_HEIGHT   1024


#define CONFIG_OV9665_TR      1
#define CONFIG_OV9665_DEBUG	  1

#if (CONFIG_OV9665_TR)
	#define OV9665_TR(format, ...)      printk(format, ## __VA_ARGS__)
	#if (CONFIG_OV9665_DEBUG)
	#define OV9665_DG(format, ...)      printk(format, ## __VA_ARGS__)
	#else
	#define OV9665_DG(format, ...)
	#endif
#else
	#define OV9665_TR(format, ...)
#endif

static const struct soc_camera_data_format ov9665_colour_formats [] = 
{
    {
        /* Order important: first natively supported,
         * second supported with a GPIO extender */
        .name		= "ov9665 YUV420",
        .depth		= 16,	// old depth is 8,  in rk28_caemra, videobuf_setup should change accordingly
        .fourcc		= V4L2_PIX_FMT_YUV420,
     },

    {
        .name = "ov9665 YUV422P",
        .depth = 16,
        .fourcc = V4L2_PIX_FMT_YUV422P,
    },
};


#if 0 // reserved state for future power constrol 
typedef enum ov9665_state_u {
	OV9665_ATTACHED,
	OV9665_DEATTACH
}ov9665_state_t;
#endif 


typedef struct sensor_info_priv_s
{
    int whiteBalance;
    int brightness;
    int contrast;
    int saturation;
    int effect;
    int scene;
    int digitalzoom;
    int focus;
    int flash;
    int exposure;
    unsigned char mirror;                                        /* HFLIP */
    unsigned char flip;                                          /* VFLIP */
    unsigned int winseqe_cur_addr;

} sensor_info_priv_t;


struct ov9665 {
	struct i2c_client *client;

	struct soc_camera_device icd;
	sensor_info_priv_t info_priv;
#if 0	
	int model;	/* V4L2_IDENT_MT9M001* codes from v4l2-chip-ident.h */
	int switch_gpio;
	unsigned char autoexposure;
	unsigned char datawidth;
#endif	
};

#define ov9665_IIC_ADDR 	    0x60  

static unsigned short normal_i2c[] = {ov9665_IIC_ADDR >> 1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data_ov9665 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};



static const struct v4l2_queryctrl ov9665_controls[] = {

	#if CONFIG_SENSOR_WhiteBalance
    {
        .id		= V4L2_CID_DO_WHITE_BALANCE,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "White Balance Control",
        .minimum	= 0,
        .maximum	= 4,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Brightness
	{
        .id		= V4L2_CID_BRIGHTNESS,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Brightness Control",
        .minimum	= -3,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Effect
	{
	        .id		= V4L2_CID_EFFECT,
	        .type		= V4L2_CTRL_TYPE_MENU,
	        .name		= "Effect Control",
	        .minimum	= 0,
	        .maximum	= 5,
	        .step		= 1,
	        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Exposure
	{
        .id		= V4L2_CID_EXPOSURE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Exposure Control",
        .minimum	= 0,
        .maximum	= 6,
        .step		= 1,
        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Saturation
	{
        .id		= V4L2_CID_SATURATION,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Saturation Control",
        .minimum	= 0,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Contrast
	{
        .id		= V4L2_CID_CONTRAST,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Contrast Control",
        .minimum	= -3,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Mirror
	{
        .id		= V4L2_CID_HFLIP,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Mirror Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    },
    #endif

	#if CONFIG_SENSOR_Flip
	{
        .id		= V4L2_CID_VFLIP,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Flip Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    },
    #endif

	#if CONFIG_SENSOR_Scene
    {
        .id		= V4L2_CID_SCENE,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Scene Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_DigitalZoom
    {
        .id		= V4L2_CID_ZOOM_RELATIVE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "DigitalZoom Control",
        .minimum	= -1,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_ZOOM_ABSOLUTE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "DigitalZoom Control",
        .minimum	= 0,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Focus
	{
        .id		= V4L2_CID_FOCUS_RELATIVE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Focus Control",
        .minimum	= -1,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_FOCUS_ABSOLUTE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Focus Control",
        .minimum	= 0,
        .maximum	= 255,
        .step		= 1,
        .default_value = 125,
    },
    #endif

	#if CONFIG_SENSOR_Flash
	{
        .id		= V4L2_CID_FLASH,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Flash Control",
        .minimum	= 0,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
	#endif
};



static int ov9665_probe(struct i2c_adapter *adapter, int addr, int kind);
static int ov9665_video_probe(struct soc_camera_device *);
static void ov9665_video_remove(struct soc_camera_device *);
static int ov9665_get_control(struct soc_camera_device *, struct v4l2_control *);
static int ov9665_set_control(struct soc_camera_device *, struct v4l2_control *);

//jyk  control function
static int ov9665_get_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl);
static int ov9665_set_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl);


#if 1// jyk, old register r/w functions 
static int ov9665_rx_data(struct i2c_client *this_client, char *rxData, int length)
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
		printk(KERN_ERR "ov9665 ov9665_rx_data: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int ov9665_tx_data(struct i2c_client *this_client, char *txData, int length)
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
		printk(KERN_ERR "ov9665 ov9665_tx_data: transfer error\n");
		return -EIO;
	} else
		return 0;
}


static int ov9665_Write(struct soc_camera_device *icd, uint8 *rxData, int length)
{
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
        ov9665_tx_data(ov9665->client, &rxData[0], 2);
	return 0;

}
static int ov9665_Read(struct soc_camera_device *icd, uint8 *rxData)
{
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
	char temp;
	temp = rxData[0];
   	ov9665_rx_data(ov9665->client, &rxData[0], 1);
	rxData[1]=rxData[0];
	rxData[0]=temp;
	return 0;
}
#endif  // old write & read


#if 1  //JYK, NEW OV9665 REG FUNCTIONS
/* sensor register write */
static int sensor_ov9665_write(struct i2c_client *client, u8 reg, u8 val)
{
    int err,cnt;
    u8 buf[2];
    struct i2c_msg msg[1];

    buf[0] = reg & 0xFF;
    buf[1] = val;

    msg->addr = client->addr;
    msg->flags = client->flags;
    msg->buf = buf;
    msg->len = sizeof(buf);
#if 0 	// this members don't exist on 25 kernel, jyk?
    msg->scl_rate = 400*1000;                                        /* ddl@rock-chips.com : 100kHz */
    msg->read_type = I2C_NORMAL;
#endif
    cnt = 3;
    err = -EAGAIN;

    while ((cnt--) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 1);

        if (err >= 0) {
            return 0;
        } else {
            OV9665_TR("\n %s write reg failed, try to write again!\n",SENSOR_NAME_STRING());
            udelay(10);
        }
    }

    return err;
}

/* sensor register read */
static int sensor_ov9665_read(struct i2c_client *client, u8 reg, u8 *val)
{
    int err,cnt;
    u8 buf[1];
    struct i2c_msg msg[1];

    buf[0] = reg & 0xFF;

    msg->addr = client->addr;
    msg->flags = client->flags;
    msg->buf = buf;
    msg->len = sizeof(buf);
//    msg->scl_rate = 400*1000;                                        /* ddl@rock-chips.com : 100kHz */
    i2c_transfer(client->adapter, msg, 1);			// why?? 
    msg->addr = client->addr;
    msg->flags = client->flags|I2C_M_RD;
    msg->buf = buf;
    msg->len = 1;                                     /* ddl@rock-chips.com : 100kHz */

    cnt = 3;
    err = -EAGAIN;
    while ((cnt--) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 1);

        if (err >= 0) {
            *val = buf[0];
            return 0;
        } else {
        	OV9665_TR("\n %s write reg failed, try to read again!\n",SENSOR_NAME_STRING());
            	udelay(10);
         }
    }

    return err;
}
#endif //~JYK, NEW OV9665 REG FUNCTIONS


/* write a array of registers  */
static int sensor_ov9665_write_array(struct i2c_client *client, struct reginfo *regarray)
{
    int err;
    int i = 0;

    while (regarray[i].reg != 0)
    {
        err = sensor_ov9665_write(client, regarray[i].reg, regarray[i].val);
        if (err != 0)
        {
            OV9665_TR("%s..write failed current i = %d\n", SENSOR_NAME_STRING(),i);
            return err;
        }
        i++;
    }
    return 0;
}




static int ov9665_init(struct soc_camera_device *icd)
{
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
    	int i;
	uint8    reg3e[2] = {0x3e,0xd0};
	uint8    reg12[2] = {0x12,0x80};
	uint8    reg13[2] = {0x13,0xe7};
	int ret;
	
/*
    mdelay(100);  
    ov9665_Write(icd, (uint8 *)reg3e, sizeof(reg3e) );
    mdelay(5);
    ov9665_Write(icd, (uint8 *)reg3e, sizeof(reg3e) );
    mdelay(5);
    ov9665_Write(icd, (uint8 *)reg12, sizeof(reg12) );
    mdelay(5);
*/
#if 0
    for (i = 0; i < sizeof(ov9665_init_data) ; i++) {  //  sizeof(ov9665_init_data) /2 
        if (0 > ov9665_tx_data(ov9665->client, &ov9665_init_data[i], 2))
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
    }
    for (i = 0; i < sizeof(ov9665_vga) ; i++) { 	// sizeof( ov9665_vga) /2 
        if (0 > ov9665_tx_data(ov9665->client, &ov9665_vga[i], 2))
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
    }
#endif 


    ret = sensor_ov9665_write_array(ov9665->client, ov9665_init_data);
	    printk("%s..%s..%d    ******** jyk ********* init ret = %d\n",__FUNCTION__,__FILE__,__LINE__, ret);

    ret = sensor_ov9665_write_array(ov9665->client, ov9665_vga);
	    printk("%s..%s..%d    ******** jyk ********* vga ret = %d\n",__FUNCTION__,__FILE__,__LINE__, ret);

    for (i = 0x0a; i < 0x0c; i++) {
	    char chekid[1];
	    chekid[0] = i;
	    ov9665_rx_data(ov9665->client, &chekid[0], 1);
	    printk("%s..%s..%d    ******** nzy *********0x%x\n",__FUNCTION__,__FILE__,__LINE__, chekid[0]);
    }

	//===============jyk 

    icd->x_current = 0;
    icd->y_current = 0;
    icd->width = 800;
    icd->height = 600;

    /* sensor ov9665 information for initialization  */
#if 0
    ov9665->info_priv.whiteBalance = ov9665_controls[0].default_value;
	
    ov9665->info_priv.brightness = ov9665_controls[1].default_value;
    ov9665->info_priv.effect = ov9665_controls[2].default_value;

    ov9665->info_priv.exposure = &icd->exposure;
    *(ov9665->info_priv.exposure) = ov9665_controls[3].default_value;
    ov9665->info_priv.saturation = ov9665_controls[4].default_value;
    ov9665->info_priv.contrast = ov9665_controls[5].default_value;
    ov9665->info_priv.mirror = ov9665_controls[6].default_value;
    ov9665->info_priv.flip = ov9665_controls[7].default_value;
    ov9665->info_priv.scene = ov9665_controls[8].default_value;
    ov9665->info_priv.digitalzoom = ov9665_controls[10].default_value;
	
    /* ddl@rock-chips.com : if sensor support auto focus and flash, programer must run focus and flash code  */
    //ov9665_set_focus();
    //ov9665_set_flash();
    ov9665->info_priv.focus = ov9665_controls[12].default_value;
    ov9665->info_priv.flash = ov9665_controls[13].default_value;
	
#endif
    ov9665->info_priv.effect = ov9665_controls[0].default_value;
    ov9665->info_priv.winseqe_cur_addr  = (int)ov9665_vga;




    OV9665_DG("\n%s..%d *** ddl *** icd->width = %d.. icd->height %d\n",__FUNCTION__,__LINE__,icd->width,icd->height);

    //OV9665_TR("ov9665 digitalzoom is %x\n", ov9665->info_priv.digitalzoom);

    mdelay(300);
	// ~jyk ========================

	return 0;
}



static int ov9665_Set_Mega(struct soc_camera_device *icd)
{
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);

	uint8    reg00[2] = {0,0} , reg04[2] = {0x04,0} , reg10[2] = {0x10,0}
			,reg13[2] = {0x13,0} , reg45[2] = {0x45,0}, reg00_buf[2] = {0x00,0};
	//uint8	,reg111;
	uint32	 i,temp;
	uint32	 PrvGain;
	uint32	 PrvExp;
	uint32	 TgtExp,TgtMaxExp,PrvMaxExp,PrvFps,TgtFps,TgtGain,TgtExpGain;
	int 		ret;
	
	reg13[1] = 0xe2;
	ov9665_Write(icd, (uint8 *)reg13, sizeof(reg13) );
	
	ov9665_Read(icd,reg00);
	ov9665_Read(icd,reg04);
	ov9665_Read(icd,reg10);
	ov9665_Read(icd,reg45);

	PrvGain = reg00[1];
	PrvExp = reg45[1] & 0x3F;
	PrvExp = PrvExp << 8;
	PrvExp = PrvExp | reg10[1];
	PrvExp = PrvExp << 2;
	PrvExp = PrvExp | ( reg04[1] & 0x03);
	
	ov9665_Read(icd,reg00_buf);
	
	reg00_buf[1]=reg00_buf[1]<<1;
	
  	TgtMaxExp =1045;
	PrvMaxExp = 495;
	PrvFps = 25; 
	TgtFps = 7;
	
	TgtExp = (2 * TgtMaxExp* TgtFps * PrvExp) / (PrvMaxExp * PrvFps);
	TgtGain = PrvGain & 0x0F | 0x10;
	
	if (PrvGain & 0x10)
	{
	  TgtGain = TgtGain << 1;
	}
	if (PrvGain & 0x20)
	{
	  TgtGain = TgtGain << 1;
	}
	if (PrvGain & 0x40)
	{
	  TgtGain = TgtGain << 1;
	}
	if (PrvGain & 0x80)
	{
	  TgtGain = TgtGain << 1;
	}

	

	TgtExpGain = TgtExp * TgtGain;
	temp = 1045*TgtFps/100;
	if (TgtExpGain < 16720)
	{
	  TgtExp = TgtExpGain / 16;
	  if(TgtExp == 0)
		while(1);
	  if (TgtExp > temp )//96
	  {//96 is exposure lines in 1/100s. The value = 1045*TgtFps/100;
		TgtExp = TgtExp / temp;//96
		TgtExp = TgtExp * temp;//96
	  }
	}
	else
	{
		//while(1);
	  TgtExp = 1045;
	}
	
	//while(1);
	//if(TgtExp == 0)
	//	while(1);
	TgtGain = (( TgtExpGain * 100 + 50 ) / TgtExp ) / 100;

	//ov9665_Set_Sxga();
#if 0	
    for (i = 0; i < sizeof(ov9665_sxga) ; i++) {  	// sizeof(  ov9665_sxga ) /2
        if (0 > ov9665_tx_data(ov9665->client, &ov9665_sxga[i], 2))
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
    }
	//ov9665_Write(icd, (uint8 *)reg00_buf, sizeof(reg00_buf) );
#else
	  ret = sensor_ov9665_write_array(ov9665->client, ov9665_sxga);
		 printk("\n%s..%s..%d    ******** jyk sxvga ********* ret =%d\n",__FUNCTION__,__FILE__,__LINE__, ret);
#endif
	
	reg04[1] = (reg04[1] & 0xFC ) |(TgtExp & 0x03);
	ov9665_Write(icd, (uint8 *)reg04, sizeof(reg04) );
	reg10[1] = (TgtExp >> 2 )	& 0xFF;
	ov9665_Write(icd, (uint8 *)reg10, sizeof(reg10) );
	
	reg45[1] = TgtExp >> 10;
	ov9665_Write(icd, (uint8 *)reg45, sizeof(reg45) );
	
	PrvGain = 0;
	if( TgtGain > 31)
	{
	  PrvGain = PrvGain | 0x10;
	  TgtGain = TgtGain >> 1;
	}
	if( TgtGain > 31)
	{
	  PrvGain = PrvGain | 0x20;
	  TgtGain = TgtGain >> 1;
	}
	if( TgtGain > 31)
	{
	  PrvGain = PrvGain | 0x40;
	  TgtGain = TgtGain >> 1;
	}
	if( TgtGain > 31)
	{
	  PrvGain = PrvGain | 0x80;
	  TgtGain = TgtGain >> 1;
	}
	if ( TgtGain > 16)
	{
	  PrvGain = PrvGain | ((TgtGain - 16 ) & 0x0F);
	}
	reg00[1] = PrvGain+0x05;
	ov9665_Write(icd, (uint8 *)reg00, sizeof(reg00) );

//	DelayMs_nops(300);
	
//	reg13[1] = 0xe7;
//	ov9665_Write(icd, (uint8 *)reg13, sizeof(reg13) );

//	DelayMs_nops(1000);
//	ov9665_Set_Sxga(); 
//	DelayMs_nops(1000);


	
    return 0;
}

static int ov9665_release(struct soc_camera_device *icd)
{
 uint8 reg3b[2] = {0x3b,0},reg13[2] = {0x13,0xe0},reg39[2] = {0x39,0x6a},regd5[2] = {0xd5,0x0},regd6[2] = {0xd6,0x0};
ov9665_Read(icd ,reg3b); // Set register 0x3b[3] to 0b'0
reg3b[1] &= 0xf7;
ov9665_Write(icd, (uint8 *)reg3b, sizeof(reg3b) );
ov9665_Write(icd, (uint8 *)reg13, sizeof(reg13) );
ov9665_Write(icd, (uint8 *)reg39, sizeof(reg39) );
ov9665_Write(icd, (uint8 *)regd5, sizeof(regd5) );
ov9665_Write(icd, (uint8 *)regd6, sizeof(regd6) );
	return 0;
}

static int ov9665_start_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int ov9665_stop_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int bus_switch_request(struct ov9665 *ov9665 )
							//	,		      struct soc_camera_link *icl)
{
    /*
	unsigned int gpio = icl->gpio;

    int ret = gpio_request(gpio, "ov9665");
    if (ret < 0) {
        dev_err(&ov9665->client->dev, "Cannot get GPIO %u\n",
            gpio);
        return ret;
    }

    ret = gpio_direction_output(gpio, 0);
    if (ret < 0) {
        dev_err(&ov9665->client->dev,
            "Cannot set GPIO %u to output\n", gpio);
        gpio_free(gpio);
        return ret;
    }
	ov9665->switch_gpio = gpio;
*/

	rockchip_mux_api_set(SENSOR_PWDN_IOMUX_PINNAME, SENSOR_PWDN_IOMUX_PINDIR);		/*xxm*/
	gpio_direction_output(SENSOR_PWDN_IOPIN, GPIO_OUT);
	__gpio_set(SENSOR_PWDN_IOPIN, GPIO_LOW);				/*xxm*/
/*
    rockchip_mux_api_set(GPIOH6_IQ_SEL_NAME, IOMUXB_GPIO1_D6);
    gpio_direction_output(GPIOPortH_Pin6, GPIO_OUT);
    __gpio_set(GPIOPortH_Pin6, GPIO_LOW);
*/
    
	return 0;
}

static void bus_switch_release(struct ov9665 *ov9665)
{
	
}

static int ov9665_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
	int ret;

	return 0;
}

static unsigned long ov9665_query_bus_param(struct soc_camera_device *icd)
{
	/* 0v9650 has all capture_format parameters fixed */
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_SENSOR_UYVY;
}

static int ov9665_set_fmt_cap(struct soc_camera_device *icd,
		__u32 pixfmt, struct v4l2_rect *rect)
{
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
	int ret;
    int i;
    char *ov9665_win;
    unsigned int ov9665_win_data_len;

    if ((rect->width == icd->width) && (rect->height == icd->height)) { 
        printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__,rect->width,rect->height);
        return 0;
    }

#if 0    
    if ((rect->width <= 320) && (rect->height <= 240)) {
        ov9665_win = ov9665_qvga;
        ov9665_win_data_len = sizeof(ov9665_qvga);
    } else if ((rect->width <= 352) && (rect->height <= 288)) {
        ov9665_win = ov9665_qcif;
        ov9665_win_data_len = sizeof(ov9665_qcif);
    } else if ((rect->width <= 640) && (rect->height <= 480)){
#else
    if ((rect->width <= 640) && (rect->height <= 480)){
	//return 0;
#endif 
	//ov9665_init(icd);
        ov9665_win = ov9665_vga;
        ov9665_win_data_len = sizeof(ov9665_vga);
    } else {
        ov9665_win = ov9665_sxga;
        ov9665_win_data_len = 0;// ov9665_win_data_len = sizeof(ov9665_sxga);
	ov9665_Set_Mega(icd);
    }
    
   ///* 
#if 0//jyk   
   for (i = 0; i < ov9665_win_data_len / 2; i++) {
        ret = ov9665_tx_data(ov9665->client, ov9665_win + 2*i, 2);
        if (0 > ret) {
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
            return ret;
        }
    }
#else 
	ret = sensor_ov9665_write_array(ov9665->client, ov9665_win);
	
	 printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
#endif

    mdelay(200);
    //*/
    
	return 0;
}

static int ov9665_try_fmt_cap(struct soc_camera_device *icd,
			       struct v4l2_format *f)
{
	if (f->fmt.pix.height < 32 + icd->y_skip_top)
		f->fmt.pix.height = 32 + icd->y_skip_top;
	if (f->fmt.pix.height > 1024 + icd->y_skip_top)
		f->fmt.pix.height = 1024;//f->fmt.pix.height = 1024 + icd->y_skip_top;
	if (f->fmt.pix.width < 48)
		f->fmt.pix.width = 48;
	if (f->fmt.pix.width > 1280)
		f->fmt.pix.width = 1280;
	f->fmt.pix.width &= ~0x01; /* has to be even, unsure why was ~3 */

	printk("\n%s..%s..%d    ******** jyk try_fmt *********<%d, %d>\n",__FUNCTION__,__FILE__,__LINE__, 	f->fmt.pix.width, 	f->fmt.pix.height);
	return 0;
}

static int ov9665_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_chip_ident *id)
{
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);

	if (id->match_type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match_chip != ov9665->client->addr)
		return -ENODEV;

	id->ident	= SENSOR_V4L2_IDENT;
	id->revision	= 0;

	return 0;
}





#if CONFIG_SENSOR_WhiteBalance
static  struct reginfo sensor_WhiteB_Auto[]=
{
	{0x13,0xe7},
	{0x00,0x00}
};

/* INCANDESCENT  */
static  struct reginfo sensor_WhiteB_INCANDESCENT[]=
{
	{0x13,0xe5},
	{0x02,0x46},
	{0x16,0x40},
	{0x01,0x56},
	{0x00,0x00}
};

/* FLUORESCENT  */
static  struct reginfo sensor_WhiteB_FLUORESCENT[]=
{
	{0x13,0xe5},
	{0x02,0x4d},
	{0x16,0x40},
	{0x01,0x4a},
	{0x00,0x00}
};
/* DAYLIGHT  */
static  struct reginfo sensor_WhiteB_DAYLIGHT[]=
{
	{0x13,0xe5},
	{0x02,0x6a},
	{0x16,0x45},
	{0x01,0x3c},
	{0x00,0x00}
};
/* warm FLUORESCENT: CWF ??  */
static  struct reginfo sensor_WhiteB_CWF[]=
{
	{0x13,0xe5},
	{0x02,0x4b},
	{0x16,0x40},
	{0x01,0x4a},
	{0x00,0x00}
};

static struct reginfo *ov9665_WhiteBalanceSeqe[] = {sensor_WhiteB_Auto, sensor_WhiteB_INCANDESCENT, sensor_WhiteB_FLUORESCENT,  sensor_WhiteB_CWF,
	sensor_WhiteB_DAYLIGHT, NULL,
};
#endif



#if CONFIG_SENSOR_Brightness
static  struct reginfo sensor_Brightness0[]=
{
    // Brightness -2
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x49},
    {0x339a, 0x20},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness1[]=
{
    // Brightness -1
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x49},
    {0x339a, 0x10},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness2[]=
{
    //  Brightness 0
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x00},
    {0x3390, 0x41},
    {0x339a, 0x00},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness3[]=
{
    // Brightness +1
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x41},
    {0x339a, 0x10},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness4[]=
{
    //  Brightness +2
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x41},
    {0x339a, 0x20},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness5[]=
{
    //  Brightness +3
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04}, //bit[2] enable
    {0x3390, 0x41}, //bit[3] sign of brightness
    {0x339a, 0x30},
    {0x0000, 0x00}
};
static struct reginfo *sensor_BrightnessSeqe[] = {sensor_Brightness0, sensor_Brightness1, sensor_Brightness2, sensor_Brightness3,
    sensor_Brightness4, sensor_Brightness5,NULL,
};

#endif


#if CONFIG_SENSOR_Effect
static  struct reginfo sensor_Effect_Normal[] =
{
	{0xc7,0x18},
	{0xc8,0x06},
	{0xcd,0x80},
	{0xce,0x80},
	{0x00, 0x00}
};

static  struct reginfo sensor_Effect_WandB[] =
{
	{0xc7,0x18},
	{0xc8,0x1e},
	{0xcd,0x80},
	{0xce,0x80},
	{0x00, 0x00}
};

static  struct reginfo sensor_Effect_Sepia[] =
{
	{0xc7,0x18},
	{0xc8,0x1e},
	{0xcd,0x40},
	{0xce,0xa0},
	{0x00, 0x00}
};

static  struct reginfo sensor_Effect_Negative[] =
{
	{0xc7,0x18},
	{0xc8,0x46},
	
	{0x00, 0x00}
};
static  struct reginfo sensor_Effect_Bluish[] =
{
	{0xc7,0x18},
	{0xc8,0x1e},
	{0xcd,0xa0},
	{0xce,0x40},
	{0x00, 0x00}
};

static  struct reginfo sensor_Effect_Green[] =
{
	{0xc7,0x18},
	{0xc8,0x1e},
	{0xcd,0x60},
	{0xce,0x60},
	{0x00, 0x00}
};
static  struct reginfo sensor_Effect_Red[] =
{
	{0xc7,0x18},
	{0xc8,0x1e},
	{0xcd,0x80},
	{0xce,0xc0},
	{0x00, 0x00}
};

static struct reginfo *ov9665_EffectSeqe[] = {sensor_Effect_Normal, sensor_Effect_WandB, sensor_Effect_Sepia,sensor_Effect_Negative,
    sensor_Effect_Green,NULL,
};
#endif


#if CONFIG_SENSOR_Exposure
static  struct reginfo sensor_Exposure0[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Exposure1[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Exposure2[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Exposure3[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Exposure4[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Exposure5[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Exposure6[]=
{
    {0x00, 0x00}
};

static struct reginfo *sensor_ExposureSeqe[] = {sensor_Exposure0, sensor_Exposure1, sensor_Exposure2, sensor_Exposure3,
    sensor_Exposure4, sensor_Exposure5,sensor_Exposure6,NULL,
};
#endif


#if CONFIG_SENSOR_Saturation
static  struct reginfo sensor_Saturation0[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Saturation1[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Saturation2[]=
{
    {0x00, 0x00}
};
static struct reginfo *sensor_SaturationSeqe[] = {sensor_Saturation0, sensor_Saturation1, sensor_Saturation2, NULL,};

#endif



#if CONFIG_SENSOR_Contrast
static  struct reginfo sensor_Contrast0[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Contrast1[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Contrast2[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Contrast3[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Contrast4[]=
{
    {0x00, 0x00}
};


static  struct reginfo sensor_Contrast5[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_Contrast6[]=
{
    {0x00, 0x00}
};
static struct reginfo *sensor_ContrastSeqe[] = {sensor_Contrast0, sensor_Contrast1, sensor_Contrast2, sensor_Contrast3,
    sensor_Contrast4, sensor_Contrast5, sensor_Contrast6, NULL,
};

#endif
#if CONFIG_SENSOR_Mirror
static  struct reginfo sensor_MirrorOn[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_MirrorOff[]=
{
    {0x00, 0x00}
};
static struct reginfo *sensor_MirrorSeqe[] = {sensor_MirrorOff, sensor_MirrorOn,NULL,};
#endif
#if CONFIG_SENSOR_Flip
static  struct reginfo sensor_FlipOn[]=
{
    {0x00, 0x00}
};

static  struct reginfo sensor_FlipOff[]=
{
    {0x00, 0x00}
};
static struct reginfo *sensor_FlipSeqe[] = {sensor_FlipOff, sensor_FlipOn,NULL,};

#endif
#if CONFIG_SENSOR_Scene
static  struct reginfo sensor_SceneAuto[] = 	//indoor (normal mode)
{
	{0x14,0x48},
	{0x13,0xe7},
	{0x03,0x03},
	{0x0f, 0x46},
	{0x00, 0x00}
};

static  struct reginfo sensor_SceneNight[] =
{
	{0x14,0x68},
	{0x13,0xe7},
	{0x03,0x83},
	{0x0f, 0x4e},		
	{0x00, 0x00}
};
static struct reginfo *ov9665_SceneSeqe[] = {sensor_SceneAuto, sensor_SceneNight,NULL,};

#endif
#if CONFIG_SENSOR_DigitalZoom
static struct reginfo sensor_Zoom0[] =
{
    {0x0, 0x0},
};

static struct reginfo sensor_Zoom1[] =
{
     {0x0, 0x0},
};

static struct reginfo sensor_Zoom2[] =
{
    {0x0, 0x0},
};


static struct reginfo sensor_Zoom3[] =
{
    {0x0, 0x0},
};
static struct reginfo *sensor_ZoomSeqe[] = {sensor_Zoom0, sensor_Zoom1, sensor_Zoom2, sensor_Zoom3, NULL,};
#endif


#if 0
static const struct v4l2_querymenu sensor_menus[] =
{
	#if CONFIG_SENSOR_WhiteBalance
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 0,  .name = "auto",  .reserved = 0, }, {  .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 1, .name = "incandescent",  .reserved = 0,},
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 2,  .name = "fluorescent", .reserved = 0,}, {  .id = V4L2_CID_DO_WHITE_BALANCE, .index = 3,  .name = "daylight", .reserved = 0,},
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 4,  .name = "cloudy-daylight", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Effect
    { .id = V4L2_CID_EFFECT,  .index = 0,  .name = "none",  .reserved = 0, }, {  .id = V4L2_CID_EFFECT,  .index = 1, .name = "mono",  .reserved = 0,},
    { .id = V4L2_CID_EFFECT,  .index = 2,  .name = "negative", .reserved = 0,}, {  .id = V4L2_CID_EFFECT, .index = 3,  .name = "sepia", .reserved = 0,},
    { .id = V4L2_CID_EFFECT,  .index = 4, .name = "posterize", .reserved = 0,} ,{ .id = V4L2_CID_EFFECT,  .index = 5,  .name = "aqua", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Scene
    { .id = V4L2_CID_SCENE,  .index = 0, .name = "auto", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 1,  .name = "night", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Flash
    { .id = V4L2_CID_FLASH,  .index = 0,  .name = "off",  .reserved = 0, }, {  .id = V4L2_CID_FLASH,  .index = 1, .name = "auto",  .reserved = 0,},
    { .id = V4L2_CID_FLASH,  .index = 2,  .name = "on", .reserved = 0,}, {  .id = V4L2_CID_FLASH, .index = 3,  .name = "torch", .reserved = 0,},
    #endif
};
#endif

static const struct v4l2_querymenu ov9665_menus[] =
{
#if CONFIG_SENSOR_WhiteBalance
	    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 0,  .name = "auto",  .reserved = 0, }, {  .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 1, .name = "incandescent",  .reserved = 0,},
	{ .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 2,  .name = "fluorescent", .reserved = 0,},  { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 3,  .name = "warm-fluorescent", .reserved = 0,},
	{  .id = V4L2_CID_DO_WHITE_BALANCE, .index = 4,  .name = "daylight", .reserved = 0,},  
#endif

#if CONFIG_SENSOR_Scene
	    { .id = V4L2_CID_SCENE,  .index = 0, .name = "auto", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 1,  .name = "night", .reserved = 0,},
#endif

#if CONFIG_SENSOR_Effect
            { .id = V4L2_CID_EFFECT,  .index = 0,  .name = "none",  .reserved = 0, }, {  .id = V4L2_CID_EFFECT,  .index = 1, .name = "mono",  .reserved = 0,},
	    {  .id = V4L2_CID_EFFECT, .index = 2,  .name = "sepia", .reserved = 0,},   { .id = V4L2_CID_EFFECT,  .index = 3,  .name = "negative", .reserved = 0,}, 
            { .id = V4L2_CID_EFFECT,  .index = 4,  .name = "aqua", .reserved = 0,},
#endif            
};



static struct soc_camera_ops ov9665_ops = {
	.owner			= THIS_MODULE,
	.probe			= ov9665_video_probe,
	.remove			= ov9665_video_remove,
	.init			= ov9665_init,
	.release		= ov9665_release,
	.start_capture		= ov9665_start_capture,
	.stop_capture		= ov9665_stop_capture,
	.set_fmt_cap		= ov9665_set_fmt_cap,
	.try_fmt_cap		= ov9665_try_fmt_cap,
	.set_bus_param		= ov9665_set_bus_param,
	.query_bus_param	= ov9665_query_bus_param,
	.controls		= ov9665_controls,		// jyk
    .menus                         = ov9665_menus,	//jyk
    .num_controls		= ARRAY_SIZE(ov9665_controls),	//jyk
    .num_menus		= ARRAY_SIZE(ov9665_menus),		//jyk
    
	.get_control		= ov9665_get_control,
	.set_control		= ov9665_set_control,
    .get_ext_control        = ov9665_get_ext_control,		//jyk
    .set_ext_control        = ov9665_set_ext_control,		//jyk
	.get_chip_id		= ov9665_get_chip_id,
	
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= ov9665_get_register,
	.set_register		= ov9665_set_register,
#endif
};


#if CONFIG_SENSOR_Brightness
static int sensor_set_brightness(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_BrightnessSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_BrightnessSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Effect
static int ov9665_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
#if 0
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_EffectSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_EffectSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
#else

    struct ov9665 *ov9665;

    ov9665 = container_of(icd, struct ov9665, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov9665_EffectSeqe[value - qctrl->minimum] != NULL)
        {

#if 0		
            if (ov9665_write_array(ov2655->client, ov2655_EffectSeqe[value - qctrl->minimum]) != 0)
	   {
                OV9665_TR("\n OV9665 WriteReg Fail.. %x   ******** jyk *********\n", __LINE__);
                return -EINVAL;
            }
            OV9665_DG("\n OV9665 Set effect - %x   ******** jyk *********\n", value);
            return 0;
#else
/*
	    for (i = 0; i < sizeof( ov9665_EffectSeqe[value - qctrl->minimum] ) ; i++) {
        		if (0 > ov9665_tx_data(ov9665->client, &ov9665_EffectSeqe[value - qctrl->minimum][i], 2))
		            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
	    }
*/
	    if (sensor_ov9665_write_array(ov9665->client, ov9665_EffectSeqe[value - qctrl->minimum]) != 0)
	   {
                OV9665_TR("\n OV9665 WriteReg Fail.. %x   ******** jyk *********\n", __LINE__);
                return -EINVAL;
            }
            OV9665_DG("\n OV9665 Set effect - %x   ******** jyk *********\n", value);
            return 0;
#endif
        }
    }
    return -EINVAL;
#endif
}
#endif

#if CONFIG_SENSOR_Exposure
static int sensor_set_exposure(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ExposureSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ExposureSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Saturation
static int sensor_set_saturation(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_SaturationSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_SaturationSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Contrast
static int sensor_set_contrast(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ContrastSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ContrastSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Mirror
static int sensor_set_mirror(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_MirrorSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_MirrorSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Flip
static int sensor_set_flip(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_FlipSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_FlipSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Scene
static int ov9665_set_scene(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
#if 0
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_SceneSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_SceneSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
#else

    struct ov9665 *ov9665;

    ov9665 = container_of(icd, struct ov9665, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov9665_SceneSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_ov9665_write_array(ov9665->client, ov9665_SceneSeqe[value - qctrl->minimum]) != 0)
            {
                OV9665_TR("\n OV9665 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV9665_DG("\n OV9655 Set Scene - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    OV9665_TR("\n Scene valure = %d is invalidate..    ******** ddl *********\n", value);
    return -EINVAL;
#endif
}
#endif
#if CONFIG_SENSOR_WhiteBalance
static int ov9665_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
#if 0
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
#endif 

    struct ov9665 *ov9665;

    ov9665 = container_of(icd, struct ov9665, icd);
	
    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov9665_WhiteBalanceSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_ov9665_write_array(ov9665->client, ov9665_WhiteBalanceSeqe[value - qctrl->minimum]) != 0)
            {
                OV9665_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            OV9665_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	OV9665_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif


#if CONFIG_SENSOR_DigitalZoom
static int sensor_set_digitalzoom(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int *value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl_info;
    int digitalzoom_cur, digitalzoom_total;

	qctrl_info = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl_info)
		return -EINVAL;

    digitalzoom_cur = sensor->info_priv.digitalzoom;
    digitalzoom_total = qctrl_info->maximum;

    if ((*value > 0) && (digitalzoom_cur >= digitalzoom_total))
    {
        SENSOR_TR("%s digitalzoom is maximum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if  ((*value < 0) && (digitalzoom_cur <= qctrl_info->minimum))
    {
        SENSOR_TR("%s digitalzoom is minimum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if ((*value > 0) && ((digitalzoom_cur + *value) > digitalzoom_total))
    {
        *value = digitalzoom_total - digitalzoom_cur;
    }

    if ((*value < 0) && ((digitalzoom_cur + *value) < 0))
    {
        *value = 0 - digitalzoom_cur;
    }

    digitalzoom_cur += *value;

    if (sensor_ZoomSeqe[digitalzoom_cur] != NULL)
    {
        if (sensor_write_array(client, sensor_ZoomSeqe[digitalzoom_cur]) != 0)
        {
            SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
            return -EINVAL;
        }
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, *value);
        return 0;
    }

    return -EINVAL;
}
#endif





static int ov9665_get_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
#if  0
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
	int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
#else 
    const struct v4l2_queryctrl *qctrl;
    struct ov9665 *ov9665;
	
    qctrl = soc_camera_find_qctrl(&ov9665_ops, ctrl->id);

    if (!qctrl)
    {
        OV9665_TR("\n%s..%s..%d.. ioctrl is faild    ******** ddl *********\n",__FUNCTION__,__FILE__,__LINE__);
        return -EINVAL;
    }

    ov9665 = container_of(icd, struct ov9665, icd);
	
	//struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
	//int data;

    switch (ctrl->id)
    {
#if 0    
        case V4L2_CID_BRIGHTNESS:
            {
                ctrl->value = ov2655->info_priv.brightness;
                break;
            }
        case V4L2_CID_SATURATION:
            {
                ctrl->value = ov2655->info_priv.saturation;
                break;
            }
        case V4L2_CID_CONTRAST:
            {
                ctrl->value = ov2655->info_priv.contrast;
                break;
            }
#endif

        case V4L2_CID_DO_WHITE_BALANCE:
            {
                ctrl->value = ov9665->info_priv.whiteBalance;
                break;
            }
#if 0		
        case V4L2_CID_EXPOSURE:
            {
                ctrl->value = *(ov2655->info_priv.exposure);
                break;
            }
        case V4L2_CID_HFLIP:
            {
                ctrl->value = ov2655->info_priv.mirror;
                break;
            }
        case V4L2_CID_VFLIP:
            {
                ctrl->value = ov2655->info_priv.flip;
                break;
            }
#endif		
        default :
                break;
    }
    return 0;
#endif
}

static int ov9665_set_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
#if 0
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
	const struct v4l2_queryctrl *qctrl;
	int data;

	qctrl = soc_camera_find_qctrl(&ov9665_ops, ctrl->id);

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
#else
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
	const struct v4l2_queryctrl *qctrl;


	qctrl = soc_camera_find_qctrl(&ov9665_ops, ctrl->id);

    if (!qctrl)
    {
        OV9665_TR("\n OV9665 ioctrl id = %d  is invalidate   ******** ddl *********\n", ctrl->id);
        return -EINVAL;
    }

    ov9665 = container_of(icd, struct ov9665, icd);


    switch (ctrl->id)
    {
#if CONFIG_SENSOR_Brightness    
        case V4L2_CID_BRIGHTNESS:
            {
                if (ctrl->value != ov2655->info_priv.brightness)
                {
                    if (ov2655_set_brightness(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    ov2655->info_priv.brightness = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Exposure
        case V4L2_CID_EXPOSURE:
            {
                if (ctrl->value != *(ov2655->info_priv.exposure))
                {
                    if (ov2655_set_exposure(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    *(ov2655->info_priv.exposure) = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Saturation
        case V4L2_CID_SATURATION:
            {
                if (ctrl->value != ov2655->info_priv.saturation)
                {
                    if (ov2655_set_saturation(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    ov2655->info_priv.saturation = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Contrast 

        case V4L2_CID_CONTRAST:
            {
                if (ctrl->value != ov2655->info_priv.contrast)
                {
                    if (ov2655_set_contrast(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    ov2655->info_priv.contrast = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_WhiteBalance
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                if (ctrl->value != ov9665->info_priv.whiteBalance)
                {
                    if (ov9665_set_whiteBalance(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    ov9665->info_priv.whiteBalance = ctrl->value;
                }
                break;
            }
#endif

#if 0
        case V4L2_CID_HFLIP:
            {
                if (ctrl->value != ov2655->info_priv.mirror)
                {
                    if (ov2655_set_mirror(icd, qctrl,ctrl->value) != 0)
                        return -EINVAL;
                    ov2655->info_priv.mirror = ctrl->value;
                }
                break;
            }
        case V4L2_CID_VFLIP:
            {
                if (ctrl->value != ov2655->info_priv.flip)
                {
                    if (ov2655_set_flip(icd, qctrl,ctrl->value) != 0)
                        return -EINVAL;
                    ov2655->info_priv.flip = ctrl->value;
                }
                break;
            }
#endif 

        default :
            break;
    }
	
#endif

	return 0;
}



static int ov9665_get_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct ov9665 *ov9665;

    qctrl = soc_camera_find_qctrl(&ov9665_ops, ext_ctrl->id);

    if (!qctrl)
    {
        OV9665_TR("\n%s..%s..%d.. ioctrl is faild    ******** ddl *********\n",__FUNCTION__,__FILE__,__LINE__);
        return -EINVAL;
    }

    ov9665 = container_of(icd, struct ov9665, icd);

    switch (ext_ctrl->id)
    {
        case V4L2_CID_SCENE:
            {
                ext_ctrl->value = ov9665->info_priv.scene;
                break;
            }
        case V4L2_CID_EFFECT:
            {
                ext_ctrl->value = ov9665->info_priv.effect;
                break;
            }
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                ext_ctrl->value = ov9665->info_priv.digitalzoom;
                break;
            }
        case V4L2_CID_ZOOM_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                ext_ctrl->value = ov9665->info_priv.focus;
                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FLASH:
            {
                ext_ctrl->value = ov9665->info_priv.flash;
                break;
            }
        default :
            break;
    }
    return 0;
}

static int ov9665_set_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct ov9665 *ov9665;


    qctrl = soc_camera_find_qctrl(&ov9665_ops, ext_ctrl->id);

    if (!qctrl)
    {
        OV9665_TR("\n OV2655 ioctrl id = %d  is invalidate   ******** ddl *********\n", ext_ctrl->id);
        return -EINVAL;
    }

    ov9665 = container_of(icd, struct ov9665, icd);

    switch (ext_ctrl->id)
    {
#if     CONFIG_SENSOR_Scene
        case V4L2_CID_SCENE:
            {
                if (ext_ctrl->value != ov9665->info_priv.scene)
                {
                    if (ov9665_set_scene(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    ov9665->info_priv.scene = ext_ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Effect
	case V4L2_CID_EFFECT:
            {
                if (ext_ctrl->value != ov9665->info_priv.effect)
                {
                    if (ov9665_set_effect(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
	             ov9665->info_priv.effect= ext_ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_DigitalZoom
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                if (ext_ctrl->value != ov2655->info_priv.digitalzoom)
                {
                    ext_ctrl->value -=  ov2655->info_priv.digitalzoom;
                }
                else
                {
                    break;
                }
            }
#endif
#if CONFIG_SENSOR_DigitalZoom
        case V4L2_CID_ZOOM_RELATIVE:
            {
                if (ext_ctrl->value)
                {
                    if (ov2655_set_digitalzoom(icd, qctrl,&ext_ctrl->value) != 0)
                        return -EINVAL;
                    ov2655->info_priv.digitalzoom += ext_ctrl->value;

                    OV2655_DG("ov2655 digitalzoom is %x\n", ov2655->info_priv.digitalzoom);
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Focus
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                if (ext_ctrl->value != ov2655->info_priv.focus)
                {
                    ext_ctrl->value -=  ov2655->info_priv.focus;
                }
                else
                {
                    break;
                }
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                if (ext_ctrl->value)
                {
                    ov2655->info_priv.focus += ext_ctrl->value;

                    OV2655_DG("ov2655 focus is %x\n", ov2655->info_priv.focus);
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Flash
        case V4L2_CID_FLASH:
            {
                ov2655->info_priv.flash = ext_ctrl->value;

                OV2655_DG("ov2655 flash is %x\n", ov2655->info_priv.flash);
                break;
            }

#endif

        default:
            break;
    }

    return 0;
}


/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int ov9665_video_probe(struct soc_camera_device *icd)
{
//	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);
//	s32 data;
	int ret;

	/* We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant. */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

//    ov9665->model = V4L2_IDENT_OV9660;  // jyk, use new ident
    icd->formats = &ov9665_colour_formats;
    icd->num_formats = ARRAY_SIZE(ov9665_colour_formats);

	/* Now that we know the model, we can start video */
	ret = soc_camera_video_start(icd);
	if (ret)
		goto eisis;

	return 0;

eisis:
ei2c:
	return ret;
}

static void ov9665_video_remove(struct soc_camera_device *icd)
{
	struct ov9665 *ov9665 = container_of(icd, struct ov9665, icd);


	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", ov9665->client->addr,
		ov9665->icd.dev.parent, ov9665->icd.vdev);
	soc_camera_video_stop(&ov9665->icd);
}

static int ov9665_remove(struct i2c_client *client)
{
	struct ov9665 *ov9665 = i2c_get_clientdata(client);

	soc_camera_device_unregister(&ov9665->icd);
	kfree(ov9665);

	return 0;
}

static int ov9665_detach_client(struct i2c_client *client)
{
	ov9665_remove(client);

	return i2c_detach_client(client);
}

static int ov9665_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data_ov9665, ov9665_probe);
}

static struct i2c_driver ov9665_driver = {
	.driver = {
		.name = "ov9665",
	    },
	.id 	= ov9665_IIC_ADDR,
	.attach_adapter = &ov9665_attach_adapter,
	.detach_client  = &ov9665_detach_client,
};

extern struct platform_device rk28_device_camera; 
#if 0
static struct soc_camera_link iclink = {//nzy add
    .bus_id = 33, /* Must match with the camera ID above */
    .gpio   = 1,
};
#endif 

static int ov9665_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct ov9665 *ov9665;
	struct soc_camera_device *icd;
//	struct soc_camera_link *icl;
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

	strlcpy(client->name, "ov9665", I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &ov9665_driver;
	client->addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 100;	
	ret = i2c_attach_client(client);
	if (ret) {
        goto exit_i2c_attach_client_failed;
	}
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	
	ov9665 = kzalloc(sizeof(struct ov9665), GFP_KERNEL);
	if (!ov9665) {
		ret = -ENOMEM;
        goto exit_alloc_data_failed;
	}

	ov9665->client = client;
	i2c_set_clientdata(client, ov9665);
    //icl = &iclink;//client->dev.platform_data;
    
	/* Second stage probe - when a capture adapter is there */
	icd = &ov9665->icd;
	icd->ops	= &ov9665_ops;
	icd->control	= &client->dev;
	icd->x_min	= 0;
	icd->y_min	= 0;
	icd->x_current	= 0;
	icd->y_current	= 0;
	icd->width_min	= 176; // ?  should check it 
	icd->width_max	= 1280;
	icd->height_min	= 144;
	icd->height_max	= 1024;
	icd->y_skip_top	= 0;
	icd->iface	= rk28_device_camera.id;//icl->bus_id;
#if 0	 // jyk, new ov9665 struct don't have 
	/* Default datawidth - this is the only width this camera (normally)
	 * supports. It is only with extra logic that it can support
	 * other widths. Therefore it seems to be a sensible default. */
	ov9665->datawidth = 8;
	/* Simulated autoexposure. If enabled, we calculate shutter width
	 * ourselves in the driver based on vertical blanking and frame width */
	ov9665->autoexposure = 1;
#endif
	ret = bus_switch_request(ov9665);//, icl);
	if (ret)
		goto exit_bus_switch_request_failed;
		
	ret = soc_camera_device_register(icd);
	if (ret)
        goto exit_soc_camera_device_register_failed;
                
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	return 0;

exit_soc_camera_device_register_failed:
    bus_switch_release(ov9665);
exit_bus_switch_request_failed:
	kfree(ov9665);
exit_alloc_data_failed:
   i2c_detach_client(client);
exit_i2c_attach_client_failed:
	kfree(client);
exit_alloc_client_failed:
exit_check_functionality_failed:
	return ret;
}

static int __init ov9665_mod_init(void)
{
	return i2c_add_driver(&ov9665_driver);
}

static void __exit ov9665_mod_exit(void)
{
	i2c_del_driver(&ov9665_driver);
}

module_init(ov9665_mod_init);
module_exit(ov9665_mod_exit);

MODULE_DESCRIPTION("Micron ov9665 Camera driver");
MODULE_AUTHOR("nzy <kernel@rock-chips>");
MODULE_LICENSE("GPL");
