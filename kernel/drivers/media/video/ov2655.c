/*
o* Driver for MT9M001 CMOS Image Sensor from Micron
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


/* Sensor Driver Configuration */
#define SENSOR_NAME ov2655
#define SENSOR_V4L2_IDENT V4L2_IDENT_OV2655
#define SENSOR_ID 0x96
#define SENSOR_MIN_WIDTH    176
#define SENSOR_MIN_HEIGHT   144
#define SENSOR_MAX_WIDTH    1600
#define SENSOR_MAX_HEIGHT   1200
#define SENSOR_INIT_WIDTH	320			/* Sensor pixel size for sensor_init_data array */
#define SENSOR_INIT_HEIGHT  240
#define SENSOR_INIT_WINSEQADR sensor_qvga

#define CONFIG_SENSOR_WhiteBalance	1
#define CONFIG_SENSOR_Brightness	0
#define CONFIG_SENSOR_Contrast      0
#define CONFIG_SENSOR_Saturation    0
#define CONFIG_SENSOR_Effect        1
#define CONFIG_SENSOR_Scene         0
#define CONFIG_SENSOR_DigitalZoom   0
#define CONFIG_SENSOR_Focus         0
#define CONFIG_SENSOR_Exposure      0
#define CONFIG_SENSOR_Flash         0
#define CONFIG_SENSOR_Mirror        0
#define CONFIG_SENSOR_Flip          0



struct reginfo{
   u16 reg;
   u8 val;
};

/* init 800X600 SVGA */
static struct reginfo ov2655_init_data[] = {
#if 1// sensor image will mirror if modified as 0
{0x308c, 0x80},
{0x308d, 0x0e},
{0x360b, 0x00},
{0x30b0, 0xff},
{0x30b1, 0xff},
{0x30b2, 0x24},

{0x300e, 0x34},
{0x300f, 0xa6},
{0x3010, 0x81},
{0x3082, 0x01},
{0x30f4, 0x01},
{0x3090, 0x08},//0x33}, // 0x3090[3] = 1: array mirror on, 0: arrary mirror off 
{0x3091, 0xc0},
{0x30ac, 0x42},

{0x30d1, 0x08},
{0x30a8, 0x56},
{0x3015, 0x03},
{0x3093, 0x00},
{0x307e, 0xe5},
{0x3079, 0x00},
{0x30aa, 0x42},
{0x3017, 0x40},
{0x30f3, 0x82},
{0x306a, 0x0c},
{0x306d, 0x00},
{0x336a, 0x3c},
{0x3076, 0x6a},
{0x30d9, 0x8c},
{0x3016, 0x82},
{0x3601, 0x30},
{0x304e, 0x88},
{0x30f1, 0x82},
{0x306f, 0x14},

{0x3012, 0x10},
{0x3011, 0x01},
{0x302A, 0x02},
{0x302B, 0xE6},
{0x3028, 0x07},
{0x3029, 0x93},

{0x3391, 0x06},
{0x3394, 0x38},
{0x3395, 0x38},

{0x3015, 0x02},
{0x302d, 0x00},
{0x302e, 0x00},

{0x3013, 0xf7},
{0x3018, 0x80},
{0x3019, 0x70},
{0x301a, 0xd4},

{0x30af, 0x00},
{0x3048, 0x1f}, 
{0x3049, 0x4e},  
{0x304a, 0x20},  
{0x304f, 0x20},  
{0x304b, 0x02}, 
{0x304c, 0x00},  
{0x304d, 0x02},  
{0x304f, 0x20},  
{0x30a3, 0x10},  
{0x3013, 0xf7}, 
{0x3014, 0x84},  
{0x3071, 0x00},
{0x3070, 0x5d},
{0x3073, 0x00},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},
{0x304d, 0x42},     
{0x304a, 0x40},  
{0x304f, 0x40},  
{0x3095, 0x07},  
{0x3096, 0x16}, 
{0x3097, 0x1d}, 

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x5e},
{0x3088, 0x03},
{0x3089, 0x20},
{0x308a, 0x02},
{0x308b, 0x58},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331a, 0x64},
{0x331b, 0x4b},
{0x331c, 0x00},
{0x331d, 0x38},
{0x3100, 0x00},

{0x3320, 0xfa},
{0x3321, 0x11},
{0x3322, 0x92},
{0x3323, 0x01},
{0x3324, 0x97},
{0x3325, 0x02},
{0x3326, 0xff},
{0x3327, 0x0c},
{0x3328, 0x10}, 
{0x3329, 0x10},
{0x332a, 0x58},
{0x332b, 0x56},
{0x332c, 0xbe},
{0x332d, 0xe1},
{0x332e, 0x3a},
{0x332f, 0x36},
{0x3330, 0x4d},
{0x3331, 0x44},
{0x3332, 0xf8},
{0x3333, 0x0a},
{0x3334, 0xf0},
{0x3335, 0xf0},
{0x3336, 0xf0},
{0x3337, 0x40},
{0x3338, 0x40},
{0x3339, 0x40},
{0x333a, 0x00},
{0x333b, 0x00}, 

{0x3380, 0x28}, 
{0x3381, 0x48}, 
{0x3382, 0x10}, 
{0x3383, 0x22}, 
{0x3384, 0xc0}, 
{0x3385, 0xe2}, 
{0x3386, 0xe2}, 
{0x3387, 0xf2}, 
{0x3388, 0x10}, 
{0x3389, 0x98}, 
{0x338a, 0x00}, 

{0x3340, 0x04},
{0x3341, 0x07},
{0x3342, 0x19},
{0x3343, 0x34},
{0x3344, 0x4a},
{0x3345, 0x5a},
{0x3346, 0x67},
{0x3347, 0x71},
{0x3348, 0x7c},
{0x3349, 0x8c},
{0x334a, 0x9b},
{0x334b, 0xa9},
{0x334c, 0xc0},
{0x334d, 0xd5},
{0x334e, 0xe8},
{0x334f, 0x20},

{0x3350, 0x37},//0x33},  
{0x3351, 0x27},//0x28},  
{0x3352, 0x00}, 
{0x3353, 0x16},	 
{0x3354, 0x00}, 
{0x3355, 0x85},  
{0x3356, 0x35},  
{0x3357, 0x28},  
{0x3358, 0x00}, 
{0x3359, 0x13},  
{0x335a, 0x00}, 
{0x335b, 0x85},  
{0x335c, 0x37},//0x34},  
{0x335d, 0x28},  
{0x335e, 0x00}, 
{0x335f, 0x13},  
{0x3360, 0x00}, 
{0x3361, 0x85},  
{0x3363, 0x70}, 
{0x3364, 0x7f}, 
{0x3365, 0x00}, 
{0x3366, 0x00}, 
{0x3362, 0x90},

{0x3301, 0xff},
{0x338B, 0x11},
{0x338c, 0x10},
{0x338d, 0x40},

{0x3370, 0xd0},
{0x3371, 0x00},
{0x3372, 0x00},
{0x3373, 0x30},
{0x3374, 0x10},
{0x3375, 0x10},
{0x3376, 0x04},
{0x3377, 0x00},
{0x3378, 0x04},
{0x3379, 0x80},

{0x3069, 0x84},
{0x307c, 0x00},//0x10}, // 0x307c[1] : mirror select , 0x307c[0]: flip select, jyk: 0x13 -> 0x10
{0x3087, 0x02},

{0x3300, 0xfc},
{0x3302, 0x11},
{0x3400, 0x02},
{0x3606, 0x20},
{0x3601, 0x30},
{0x30f3, 0x83},
{0x304e, 0x88},

{0x30aa, 0x72},
{0x30a3, 0x80},
{0x30a1, 0x41},

{0x3086, 0x0f},
{0x3086, 0x00},
    
{0x0, 0x0},   //end flag
#else//mirror
{0x308c, 0x80},
{0x308d, 0x0e},
{0x360b, 0x00},
{0x30b0, 0xff},
{0x30b1, 0xff},
{0x30b2, 0x24},

{0x300e, 0x34},
{0x300f, 0xa6},
{0x3010, 0x81},
{0x3082, 0x01},
{0x30f4, 0x01},
{0x3090, 0x33},
{0x3091, 0xc0},
{0x30ac, 0x42},

{0x30d1, 0x08},
{0x30a8, 0x56},
{0x3015, 0x03},
{0x3093, 0x00},
{0x307e, 0xe5},
{0x3079, 0x00},
{0x30aa, 0x42},
{0x3017, 0x40},
{0x30f3, 0x82},
{0x306a, 0x0c},
{0x306d, 0x00},
{0x336a, 0x3c},
{0x3076, 0x6a},
{0x30d9, 0x8c},
{0x3016, 0x82},
{0x3601, 0x30},
{0x304e, 0x88},
{0x30f1, 0x82},
{0x306f, 0x14},

{0x3012, 0x10},
{0x3011, 0x01},
{0x302A, 0x02},
{0x302B, 0xE6},
{0x3028, 0x07},
{0x3029, 0x93},

{0x3391, 0x06},
{0x3394, 0x38},
{0x3395, 0x38},

{0x3015, 0x02},
{0x302d, 0x00},
{0x302e, 0x00},

{0x3013, 0xf7},
{0x3018, 0x80},
{0x3019, 0x70},
{0x301a, 0xd4},

{0x30af, 0x00},
{0x3048, 0x1f}, 
{0x3049, 0x4e},  
{0x304a, 0x20},  
{0x304f, 0x20},  
{0x304b, 0x02}, 
{0x304c, 0x00},  
{0x304d, 0x02},  
{0x304f, 0x20},  
{0x30a3, 0x10},  
{0x3013, 0xf7}, 
{0x3014, 0x84},  
{0x3071, 0x00},
{0x3070, 0x5d},
{0x3073, 0x00},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},
{0x304d, 0x42},     
{0x304a, 0x40},  
{0x304f, 0x40},  
{0x3095, 0x07},  
{0x3096, 0x16}, 
{0x3097, 0x1d}, 

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x5e},
{0x3088, 0x03},
{0x3089, 0x20},
{0x308a, 0x02},
{0x308b, 0x58},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331a, 0x64},
{0x331b, 0x4b},
{0x331c, 0x00},
{0x331d, 0x38},
{0x3100, 0x00},

{0x3320, 0xfa},
{0x3321, 0x11},
{0x3322, 0x92},
{0x3323, 0x01},
{0x3324, 0x97},
{0x3325, 0x02},
{0x3326, 0xff},
{0x3327, 0x0c},
{0x3328, 0x10}, 
{0x3329, 0x10},
{0x332a, 0x58},
{0x332b, 0x56},
{0x332c, 0xbe},
{0x332d, 0xe1},
{0x332e, 0x3a},
{0x332f, 0x36},
{0x3330, 0x4d},
{0x3331, 0x44},
{0x3332, 0xf8},
{0x3333, 0x0a},
{0x3334, 0xf0},
{0x3335, 0xf0},
{0x3336, 0xf0},
{0x3337, 0x40},
{0x3338, 0x40},
{0x3339, 0x40},
{0x333a, 0x00},
{0x333b, 0x00}, 

{0x3380, 0x28}, 
{0x3381, 0x48}, 
{0x3382, 0x10}, 
{0x3383, 0x22}, 
{0x3384, 0xc0}, 
{0x3385, 0xe2}, 
{0x3386, 0xe2}, 
{0x3387, 0xf2}, 
{0x3388, 0x10}, 
{0x3389, 0x98}, 
{0x338a, 0x00}, 

{0x3340, 0x04},
{0x3341, 0x07},
{0x3342, 0x19},
{0x3343, 0x34},
{0x3344, 0x4a},
{0x3345, 0x5a},
{0x3346, 0x67},
{0x3347, 0x71},
{0x3348, 0x7c},
{0x3349, 0x8c},
{0x334a, 0x9b},
{0x334b, 0xa9},
{0x334c, 0xc0},
{0x334d, 0xd5},
{0x334e, 0xe8},
{0x334f, 0x20},

{0x3350, 0x33},  
{0x3351, 0x28},  
{0x3352, 0x00}, 
{0x3353, 0x16},	 
{0x3354, 0x00}, 
{0x3355, 0x85},  
{0x3356, 0x35},  
{0x3357, 0x28},  
{0x3358, 0x00}, 
{0x3359, 0x13},  
{0x335a, 0x00}, 
{0x335b, 0x85},  
{0x335c, 0x34},  
{0x335d, 0x28},  
{0x335e, 0x00}, 
{0x335f, 0x13},  
{0x3360, 0x00}, 
{0x3361, 0x85},  
{0x3363, 0x70}, 
{0x3364, 0x7f}, 
{0x3365, 0x00}, 
{0x3366, 0x00}, 
{0x3362, 0x90},

{0x3301, 0xff},
{0x338B, 0x11},
{0x338c, 0x10},
{0x338d, 0x40},

{0x3370, 0xd0},
{0x3371, 0x00},
{0x3372, 0x00},
{0x3373, 0x30},
{0x3374, 0x10},
{0x3375, 0x10},
{0x3376, 0x04},
{0x3377, 0x00},
{0x3378, 0x04},
{0x3379, 0x80},

{0x3069, 0x84},
{0x307c, 0x10},
{0x3087, 0x02},

{0x3300, 0xfc},
{0x3302, 0x11},
{0x3400, 0x02},
{0x3606, 0x20},
{0x3601, 0x30},
{0x30f3, 0x83},
{0x304e, 0x88},

{0x30aa, 0x72},
{0x30a3, 0x80},
{0x30a1, 0x41},

{0x3086, 0x0f},
{0x3086, 0x00},
    
{0x0, 0x0},   //end flag
#endif
};

/* 1600X1200 UXGA */
static struct reginfo ov2655_uxga[] = {
    
{0x300E, 0x34}, 
{0x3011, 0x01},
{0x3012, 0x00},
{0x302a, 0x05},
{0x302b, 0xCB},
{0x306f, 0x54}, 
{0x3362, 0x80},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x0f},
{0x301d, 0x0f},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x0A},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x04},
{0x3027, 0xbc},
{0x3088, 0x06},
{0x3089, 0x40},
{0x308A, 0x04},
{0x308B, 0xB0},
{0x3316, 0x64},
{0x3317, 0x4B},
{0x3318, 0x00},
{0x3319, 0x6C},
{0x331A, 0x64},
{0x331B, 0x4B},
{0x331C, 0x00},
{0x331D, 0x6C},
{0x3302, 0x01},
    
    {0x0, 0x0}, 
};

/* 1280X1024 SXGA */
static struct reginfo ov2655_sxga[] = {
    
{0x300E, 0x34}, 
{0x3011, 0x01},
{0x3012, 0x00},
{0x302a, 0x05},
{0x302b, 0xCB},
{0x306f, 0x54}, 
{0x3362, 0x80},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x0f},
{0x301d, 0x0f},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x0A},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x04},
{0x3027, 0xbc},
{0x3088, 0x05},
{0x3089, 0x00},
{0x308A, 0x04},
{0x308B, 0x00},
{0x3316, 0x64},
{0x3317, 0x4B},
{0x3318, 0x00},
{0x3319, 0x6C},
{0x331A, 0x50},
{0x331B, 0x40},
{0x331C, 0x00},
{0x331D, 0x6C},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 800X600 SVGA*/
static struct reginfo ov2655_svga[] = {
    
{0x300E, 0x34},
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14},
{0x3362, 0x90},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x5E},
{0x3088, 0x03},
{0x3089, 0x20},
{0x308A, 0x02},
{0x308B, 0x58},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331A, 0x64},
{0x331B, 0x4B},
{0x331C, 0x00},
{0x331D, 0x38},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 640X480 VGA */
static struct reginfo ov2655_vga[] = {
    
{0x300E, 0x34},
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14},
{0x3362, 0x90},

{0x3070, 0x5D},
{0x3072, 0x5D},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x61},
{0x3088, 0x02},
{0x3089, 0x88},
{0x308A, 0x01},
{0x308B, 0xe0},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331A, 0x28},
{0x331B, 0x1e},
{0x331C, 0x08},
{0x331D, 0x38},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 352X288 CIF */
static struct reginfo ov2655_cif[] = {
    
{0x300E, 0x34}, 
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14}, 
{0x3362, 0x90},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x61},
{0x3088, 0x01},
{0x3089, 0x68},
{0x308a, 0x01},
{0x308b, 0x20},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331a, 0x16},
{0x331b, 0x12},
{0x331c, 0x08},
{0x331d, 0x38},
{0x3100, 0x00},
    {0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 320*240 QVGA */
static  struct reginfo ov2655_qvga[] = {
    
{0x300E, 0x34},
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14},
{0x3362, 0x90},

{0x3070, 0x5D},
{0x3072, 0x5D},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x61},
{0x3088, 0x01},
{0x3089, 0x40},
{0x308A, 0x00},
{0x308B, 0xf0},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331A, 0x14},
{0x331B, 0x0f},
{0x331C, 0x00},
{0x331D, 0x38},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 176X144 QCIF*/
static struct reginfo ov2655_qcif[] =
{
	{0x300E, 0x34},
    {0x3011, 0x01},
    {0x3012, 0x10},
    {0x302a, 0x02},
    {0x302b, 0xE6},
    {0x306f, 0x14},
    {0x3362, 0x90},
    {0x3070, 0x5d},
    {0x3072, 0x5d},
    {0x301c, 0x07},
    {0x301d, 0x07},
    {0x3020, 0x01},
    {0x3021, 0x18},
    {0x3022, 0x00},
    {0x3023, 0x06},
    {0x3024, 0x06},
    {0x3025, 0x58},
    {0x3026, 0x02},
    {0x3027, 0x61},
    {0x3088, 0x00},
    {0x3089, 0xb8},
    {0x308a, 0x00},
    {0x308b, 0x90},
    {0x3316, 0x64},
    {0x3317, 0x25},
    {0x3318, 0x80},
    {0x3319, 0x08},
    {0x331a, 0x16},
    {0x331b, 0x12},
    {0x331c, 0x08},
    {0x331d, 0x38},
    {0x3100, 0x00},
    {0x3302, 0x11},

    {0x0, 0x0},
};
#if 0 //jyk_0919
/* 160X120 QQVGA*/
static struct reginfo ov2655_qqvga[] = {
    
{0x300E, 0x34},
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14},
{0x3362, 0x90},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x61},
{0x3088, 0x00},
{0x3089, 0xa0},
{0x308a, 0x00},
{0x308b, 0x78},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331a, 0x0a},
{0x331b, 0x07},
{0x331c, 0x80},
{0x331d, 0x38},
{0x3100, 0x00},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};



static  struct reginfo ov2655_Sharpness_auto[] =
{
    {0x3306, 0x00},
};

static  struct reginfo ov2655_Sharpness1[] =
{
    {0x3306, 0x08},
    {0x3371, 0x00},
};

static  struct reginfo ov2655_Sharpness2[][3] =
{
    //Sharpness 2
    {0x3306, 0x08},
    {0x3371, 0x01},
};

static  struct reginfo ov2655_Sharpness3[] =
{
    //default
    {0x3306, 0x08},
    {0x332d, 0x02},
};
static  struct reginfo ov2655_Sharpness4[]=
{
    //Sharpness 4
    {0x3306, 0x08},
    {0x332d, 0x03},
};

static  struct reginfo ov2655_Sharpness5[] =
{
    //Sharpness 5
    {0x3306, 0x08},
    {0x332d, 0x04},
};
#endif //jyk_0919~ 

#define MIN(x,y)   ((x<y) ? x: y)
#define MAX(x,y)    ((x>y) ? x: y)

#define OV2655_MIN_WIDTH    176
#define OV2655_MIN_HEIGHT   144
#define OV2655_MAX_WIDTH    1600
#define OV2655_MAX_HEIGHT   1200


#define CONFIG_OV2655_TR      1
#define CONFIG_OV2655_DEBUG	  1
#if (CONFIG_OV2655_TR)
	#define OV2655_TR(format, ...)      printk(format, ## __VA_ARGS__)
	#if (CONFIG_OV2655_DEBUG)
	#define OV2655_DG(format, ...)      printk(format, ## __VA_ARGS__)
	#else
	#define OV2655_DG(format, ...)
	#endif
#else
	#define OV2655_TR(format, ...)
#endif
static const struct soc_camera_data_format ov2655_colour_formats [] = 
{
    {
        /* Order important: first natively supported,
         * second supported with a GPIO extender */
        .name		= "ov2655 YUV420",
        .depth		= 16,	// old depth is 8,  in rk28_caemra, videobuf_setup should change accordingly
        .fourcc		= V4L2_PIX_FMT_YUV420,
     },

    {
        .name = "ov2655 YUV422P",
        .depth = 16,
        .fourcc = V4L2_PIX_FMT_YUV422P,
    },
};


typedef enum ov2655_state_u {
	OV2655_ATTACHED,
	OV2655_DEATTACH
}ov2655_state_t;

typedef struct ov2655_info_priv_s
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
    unsigned short *exposure;                               /* ddl@rock-chips.com : struct "soc_camera_device" have exposure element, here is synchronization */
    unsigned char mirror;                                        /* HFLIP */
    unsigned char flip;                                               /* VFLIP */
    unsigned int winseqe_cur_addr;

    unsigned int powerdown_pin;
	unsigned int power_ldo_pin;
	ov2655_state_t state;

} ov2655_info_priv_t;
struct ov2655
{
    struct i2c_client *client;
    struct soc_camera_device icd;
    ov2655_info_priv_t info_priv;
#if 0 //jyk_0919
	int model;	/* V4L2_IDENT_MT9M001* codes from v4l2-chip-ident.h */
	int switch_gpio;
	unsigned char autoexposure;
	unsigned char datawidth;
#endif
};

#define OV2655_IIC_ADDR 	    0x60  

static unsigned short normal_i2c[] = {OV2655_IIC_ADDR >> 1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data_ov2655 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};
//jyk_0919, add controls support
static const struct v4l2_queryctrl ov2655_controls[] =
{
    {
        .id		= V4L2_CID_DO_WHITE_BALANCE,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "White Balance Control",
        .minimum	= 0,
        .maximum	= 4,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_BRIGHTNESS,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Brightness Control",
        .minimum	= -3,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_EFFECT,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Effect Control",
        .minimum	= 0,
        .maximum	= 5,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_EXPOSURE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Exposure Control",
        .minimum	= 0,
        .maximum	= 6,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_SATURATION,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Saturation Control",
        .minimum	= 0,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_CONTRAST,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Contrast Control",
        .minimum	= -3,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_HFLIP,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Mirror Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    }, {
        .id		= V4L2_CID_VFLIP,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Flip Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    }, {
        .id		= V4L2_CID_SCENE,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Scene Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    }, {
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
    }, {
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
    }, {
        .id		= V4L2_CID_FLASH,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Flash Control",
        .minimum	= 0,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    }
};
//~jyk_0919
static int ov2655_probe(struct i2c_adapter *adapter, int addr, int kind);
static int ov2655_video_probe(struct soc_camera_device *);
static void ov2655_video_remove(struct soc_camera_device *);
static int ov2655_get_control(struct soc_camera_device *, struct v4l2_control *);
static int ov2655_set_control(struct soc_camera_device *, struct v4l2_control *);
//jyk  control function
static int ov2655_get_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl);
static int ov2655_set_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl);
#if 0//jyk, power control support,   don't support now
static int ov2655_power(struct soc_camera_device *icd,int on)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

	if (ov2655->info_priv.power_ldo_pin != 0xffffffff) {
		if (on) {										/* ov2655 power on */
			extgpio_set_pinlevel(ov2655->info_priv.power_ldo_pin,Extend_Gpio_High);
			extgpio_set_pindirection(ov2655->info_priv.power_ldo_pin, Extend_Gpio_OUT);

			OV2655_DG("OV2655 LDO On !\n");
		} else {										/* ov2655 power off */
			extgpio_set_pinlevel(ov2655->info_priv.power_ldo_pin,Extend_Gpio_Low);
			extgpio_set_pindirection(ov2655->info_priv.power_ldo_pin, Extend_Gpio_OUT);

			OV2655_DG("OV2655 LDO Off !\n");
		}
	}

	if (ov2655->info_priv.powerdown_pin != 0xffffffff) {
		if (on) {										/* ov2655 power on */
			extgpio_set_pinlevel(ov2655->info_priv.powerdown_pin,Extend_Gpio_Low);
			extgpio_set_pindirection(ov2655->info_priv.powerdown_pin, Extend_Gpio_OUT);

			OV2655_DG("OV2655 Power On !\n");
		} else {										/* ov2655 power off */
			extgpio_set_pinlevel(ov2655->info_priv.powerdown_pin,Extend_Gpio_High);
			extgpio_set_pindirection(ov2655->info_priv.powerdown_pin, Extend_Gpio_OUT);

			OV2655_DG("OV2655 Power Off !\n");
		}
	}
	return 0;
}
//~ 
#endif 

/* ov2655 register write */
static int ov2655_write(struct i2c_client *client, u16 reg, u8 val)
{
    int err,cnt;
    u8 buf[3];
    struct i2c_msg msg[1];

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;
    buf[2] = val;

    msg->addr = client->addr;
    msg->flags = 0;
    msg->buf = buf;
    msg->len = sizeof(buf);

    cnt = 3;
    err = -EAGAIN;

    while ((cnt--) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 1);

        if (err >= 0) {
            return 0;
        } else {
            udelay(100);
        }
    }

   return err;
}

/* ov2655 register read */
static int ov2655_read(struct i2c_client *client, u16 reg, u8 *val)
{
    int err,cnt;
    u8 buf[2];
    struct i2c_msg msg[1];

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;

    msg->addr = client->addr;
    msg->flags |= I2C_M_RD;
    msg->buf = buf;
    msg->len = sizeof(buf);

    cnt = 3;
    err = -EAGAIN;

    while ((cnt--) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 1);

        if (err >= 0) {
            *val = buf[0];
            return 0;
        } else {
            udelay(100);
        }
    }

    return err;
}

/* write a array of registers  */ 
static int ov2655_write_array(struct i2c_client *client, struct reginfo *regarray)
{
   int err;
   int i = 0;

    while (regarray[i].reg != 0)
    {
        err = ov2655_write(client, regarray[i].reg, regarray[i].val);
        if (err != 0)
        {
            OV2655_TR("write failed current i = %d\n", i);
            return err;
        }
        i++;
    }
    return 0;
}

static int ov2655_init(struct soc_camera_device *icd)
{
    struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
    u8 val;
    int ret;
    u16 pid = 0;

	#if 0
    if (GPIOPinNumLast > ov2655->info_priv.powerdown_pin)          /* ov2655 power on */
    {
        OV2655_TR("\n ov2655 Power On. %x   ******** ddl *********\n", __LINE__);
        __gpio_set(ov2655->info_priv.powerdown_pin, GPIO_LOW);
    }

	#else
		// ov2655_power(icd, 1); // jyk, don't support now
	#endif
	
    udelay(20);
	
    /* soft reset */
    ret = ov2655_write(ov2655->client, 0x3012, 0x80);
    if (ret != 0)
    {
        OV2655_TR("soft reset ov2655 failed\n");
		ret = -ENODEV;
		goto OV2655_INIT_ERR;
    }

//    udelay( 5000);  //delay 5 microseconds

    /* check if it is an ov2655 sensor */
    ret = ov2655_read(ov2655->client, 0x300a, &val);
    if (ret != 0)
    {
        OV2655_TR("read chip id high byte failed\n");
        ret = -ENODEV;
		goto OV2655_INIT_ERR;
    }

    pid |= (val << 8);

    ret = ov2655_read(ov2655->client, 0x300b, &val);
    if (ret != 0)
    {
        OV2655_TR("read chip id low byte failed\n");
        ret = -ENODEV;
		goto OV2655_INIT_ERR;
    }
    
    pid |= (val & 0xff);

    OV2655_DG("sensor product id is %x\n",pid);
    if (pid != 0x2656)
    {
        OV2655_TR("error: devicr mismatched \n");
        ret = -ENODEV;
		goto OV2655_INIT_ERR;
    }


    ret = ov2655_write_array(ov2655->client, ov2655_init_data);
    if (ret != 0)
    {
        OV2655_TR("error: ov2655 initial failed\n");
        goto OV2655_INIT_ERR;
    }

    icd->x_current = 0;
    icd->y_current = 0;
    icd->width = 800;
    icd->height = 600;

    /* sensor ov2655 information for initialization  */
    ov2655->info_priv.whiteBalance = ov2655_controls[0].default_value;
    ov2655->info_priv.brightness = ov2655_controls[1].default_value;
    ov2655->info_priv.effect = ov2655_controls[2].default_value;
    ov2655->info_priv.exposure = &icd->exposure;
    *(ov2655->info_priv.exposure) = ov2655_controls[3].default_value;
    ov2655->info_priv.saturation = ov2655_controls[4].default_value;
    ov2655->info_priv.contrast = ov2655_controls[5].default_value;
    ov2655->info_priv.mirror = ov2655_controls[6].default_value;
    ov2655->info_priv.flip = ov2655_controls[7].default_value;
    ov2655->info_priv.scene = ov2655_controls[8].default_value;
    ov2655->info_priv.digitalzoom = ov2655_controls[10].default_value;
    ov2655->info_priv.winseqe_cur_addr  = (int)ov2655_svga;

    /* ddl@rock-chips.com : if sensor support auto focus and flash, programer must run focus and flash code  */
    //ov2655_set_focus();
    //ov2655_set_flash();
    ov2655->info_priv.focus = ov2655_controls[12].default_value;
    ov2655->info_priv.flash = ov2655_controls[13].default_value;

	ov2655->info_priv.state = OV2655_ATTACHED;
    OV2655_DG("\n%s..%d *** ddl *** icd->width = %d.. icd->height %d\n",__FUNCTION__,__LINE__,icd->width,icd->height);

    //OV2655_TR("ov2655 digitalzoom is %x\n", ov2655->info_priv.digitalzoom);

    mdelay(300);
    return 0;
	
OV2655_INIT_ERR:
//	ov2655_power(icd, 0); // jyk,  not supported now
	return ret;	
}

static int ov2655_release(struct soc_camera_device *icd)
{
	return 0;
}

static int ov2655_start_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int ov2655_stop_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int ov2655_gpio_request(struct ov2655 *ov2655)
{
    /*
	unsigned int gpio = icl->gpio;

    int ret = gpio_request(gpio, "ov2655");
    if (ret < 0) {
        dev_err(&ov2655->client->dev, "Cannot get GPIO %u\n",
            gpio);
        return ret;
    }

    ret = gpio_direction_output(gpio, 0);
    if (ret < 0) {
        dev_err(&ov2655->client->dev,
            "Cannot set GPIO %u to output\n", gpio);
        gpio_free(gpio);
        return ret;
    }
	ov2655->switch_gpio = gpio;
    */
#if 0
    rockchip_mux_api_set(GPIOH6_IQ_SEL_NAME, IOMUXB_GPIO1_D6);
    gpio_direction_output(GPIOPortH_Pin6, GPIO_OUT);
    __gpio_set(GPIOPortH_Pin6, GPIO_LOW);
    
#endif    
	rockchip_mux_api_set(SENSOR_PWDN_IOMUX_PINNAME, SENSOR_PWDN_IOMUX_PINDIR);		/*xxm*/
	gpio_direction_output(SENSOR_PWDN_IOPIN, GPIO_OUT);
	__gpio_set(SENSOR_PWDN_IOPIN, GPIO_LOW);				/*xxm*/

//	ov2655->info_priv.powerdown_pin = OV2655_PWRDN_PIN;
//	ov2655->info_priv.power_ldo_pin = OV2655_PWRLDO_PIN;
	
	return 0;
}

static void ov2655_gpio_release(struct ov2655 *ov2655)
{
	#if 0
    if (GPIOPinNumLast > ov2655->info_priv.powerdown_pin)
    {
        __gpio_set(ov2655->info_priv.powerdown_pin, GPIO_HIGH);
        gpio_free(ov2655->info_priv.powerdown_pin);
        ov2655->info_priv.powerdown_pin = 0xffffffff;
    }
    return;
	#else
	ov2655->info_priv.powerdown_pin = 0xffffffff;
	ov2655->info_priv.power_ldo_pin = 0xffffffff;
	#endif
}

static int ov2655_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
       //struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
	

	return 0;
}

static unsigned long ov2655_query_bus_param(struct soc_camera_device *icd)
{
    /* 0v9650 has all capture_format parameters fixed */
    return SOCAM_PCLK_SAMPLE_RISING |
           SOCAM_HSYNC_ACTIVE_HIGH |
           SOCAM_VSYNC_ACTIVE_LOW|
           SOCAM_SENSOR_UYVY;
}

static int ov2655_set_fmt_cap(struct soc_camera_device *icd,
		__u32 pixfmt, struct v4l2_rect *rect)
{
#if 0
    struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
    int ret;
    struct reginfo *ov2655_win;
    
    //printk("rect width = %d, rect height = %d\n", rect->width, rect->height);
#if 1    
    if ((rect->width <= 320) && (rect->height <= 240)) {
       ov2655_win = ov2655_qvga;
    } else if ((rect->width <= 352) && (rect->height <= 288)){
        ov2655_win = ov2655_cif;
    } else if ((rect->width <= 640) && (rect->height <= 480)){
#else
    if ((rect->width <= 640) && (rect->height <= 480)){
#endif    
        ov2655_win = ov2655_vga;
    } else if((rect->width <= 800) && (rect->height <= 600)){
       ov2655_win = ov2655_svga;
    } else if((rect->width <= 1280) && (rect->height <= 1024)) {
        ov2655_win = ov2655_sxga;
    }else {
        ov2655_win = ov2655_uxga;
    }
    
    ret = ov2655_write_array(ov2655->client, ov2655_win); 
    if (ret != 0)
    {
       printk("ov2655 set format capability failed\n");
       return ret;
    }
    mdelay(250);
    //printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__,rect->width,rect->height);

    return 0;
#else

    struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
    int ret, set_w,set_h;
    struct reginfo *winseqe_set_addr;

    set_w = ((rect->left<<1) + rect->width);            /* ddl@rock-chips.com :  Visiarea is in sensor image center  */
    set_h = ((rect->top<<1) + rect->height);

    if ((set_w <= 176) && (set_h <= 144))
	{
		winseqe_set_addr = ov2655_qcif;
        set_w = 176;
        set_h = 144;
	}
	else if ((set_w <= 320) && (set_h <= 240))
    {
        winseqe_set_addr = ov2655_qvga;
        set_w = 320;
        set_h = 240;
    }
	else if ((set_w <= 352) && (set_h<= 288))
    {
        winseqe_set_addr = ov2655_cif;
        set_w = 352;
        set_h = 288;
    }
    else if ((set_w <= 640) && (set_h <= 480))
    {
        winseqe_set_addr = ov2655_vga;
        set_w = 640;
        set_h = 480;
    }
    else if ((set_w <= 800) && (set_h <= 600))
    {
        winseqe_set_addr = ov2655_svga;
        set_w = 800;
        set_h = 600;
    }
    else if ((set_w <= 1280) && (set_h <= 1024))
    {
        winseqe_set_addr = ov2655_sxga;
        set_w = 1280;
        set_h = 1024;
    }
    else if ((set_w <= 1600) && (set_h <= 1200))
    {
        winseqe_set_addr = ov2655_uxga;
        set_w = 1600;
        set_h = 1200;
    }
    else
    {
        winseqe_set_addr = ov2655_qvga;               /* ddl@rock-chips.com : Sensor output smallest size if  isn't support app  */
        set_w = 320;
        set_h = 240;
    }

    if ((int)winseqe_set_addr  != ov2655->info_priv.winseqe_cur_addr)
    {
        ret = ov2655_write_array(ov2655->client, winseqe_set_addr);
        if (ret != 0)
        {
            OV2655_TR("ov2655 set format capability failed\n");
            return ret;
        }

        ov2655->info_priv.winseqe_cur_addr  = (int)winseqe_set_addr;
        mdelay(250);

        OV2655_DG("\n%s..%d *** ddl *** icd->width = %d.. icd->height %d\n",__FUNCTION__,__LINE__,set_w,set_h);
    }
    else
    {
        OV2655_TR("\n .. Current Format is validate *** ddl *** icd->width = %d.. icd->height %d\n",set_w,set_h);
    }

    return 0;
#endif
}

static int ov2655_try_fmt_cap(struct soc_camera_device *icd,
			       struct v4l2_format *f)
{
#if 0 //jyk_0918 
	if (f->fmt.pix.height < 32 + icd->y_skip_top)
		f->fmt.pix.height = 32 + icd->y_skip_top;
	if (f->fmt.pix.height > 1200 + icd->y_skip_top)
		f->fmt.pix.height = 1200 + icd->y_skip_top;
	if (f->fmt.pix.width < 48)
		f->fmt.pix.width = 48;
	if (f->fmt.pix.width > 1600)
		f->fmt.pix.width = 1600;
	f->fmt.pix.width &= ~0x01; /* has to be even, unsure why was ~3 */

	return 0;
#else

    f->fmt.pix.height = MAX(f->fmt.pix.height, icd->height_min);
    if (f->fmt.pix.height + icd->y_skip_top > icd->height_max)
        f->fmt.pix.height = icd->height_max - icd->y_skip_top;

    f->fmt.pix.width = MAX(f->fmt.pix.width, icd->width_min);
    f->fmt.pix.width = MIN(f->fmt.pix.width, icd->width_max);
    f->fmt.pix.width &= ~0x01; /* has to be even, unsure why was ~3 */

    return 0;

#endif
}

static int ov2655_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_chip_ident *id)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);

	if (id->match_type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match_chip != ov2655->client->addr)
		return -ENODEV;

	id->ident	= V4L2_IDENT_OV2655; //ov2655->model;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2655_get_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != ov2655->client->addr)
		return -ENODEV;

    char reg = reg->reg;
    int ret = ov2655_rx_data(ov2655->client, &reg, 1);
    if (!ret)
        reg->val = reg;

	return ret;
}

static int ov2655_set_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != ov2655->client->addr)
		return -ENODEV;

    char reg[2];
    reg[0] = reg->reg;
    reg[1] = reg->val;
    int ret = ov2655_tx_data(ov2655->client, reg, 2);

	return ret;
}
#endif

#define COLOR_TEMPERATURE_CLOUDY_DN  6500
#define COLOR_TEMPERATURE_CLOUDY_UP    8000
#define COLOR_TEMPERATURE_CLEARDAY_DN  5000
#define COLOR_TEMPERATURE_CLEARDAY_UP    6500
#define COLOR_TEMPERATURE_OFFICE_DN     3500
#define COLOR_TEMPERATURE_OFFICE_UP     5000
#define COLOR_TEMPERATURE_HOME_DN       2500
#define COLOR_TEMPERATURE_HOME_UP       3500

static  struct reginfo ov2655_WhiteB_Auto[]=
{
    {0x3306, 0x00},  //AWB auto, bit[1]:0,auto
    {0x0000, 0x00}
};
/* Cloudy Colour Temperature : 6500K - 8000K  */
static  struct reginfo ov2655_WhiteB_Cloudy[]=
{
    {0x3306, 0x82},
    {0x3337, 0x68},
    {0x3338, 0x40},
    {0x3339, 0x4e},
    {0x0000, 0x00}
};
/* ClearDay Colour Temperature : 5000K - 6500K  */
static  struct reginfo ov2655_WhiteB_ClearDay[]=
{
    //Sunny
    {0x3306, 0x02}, //AWB off
    {0x3337, 0x5e},
    {0x3338, 0x40},
    {0x3339, 0x46},
    {0x0000, 0x00}
};
/* Office Colour Temperature : 3500K - 5000K  */
static  struct reginfo ov2655_WhiteB_TungstenLamp1[]=
{
    //Office
    {0x3306, 0x02},
    {0x3337, 0x52},
    {0x3338, 0x40},
    {0x3339, 0x58},
    {0x0000, 0x00}

};
/* Home Colour Temperature : 2500K - 3500K  */
static  struct reginfo ov2655_WhiteB_TungstenLamp2[]=
{
    //Home
    {0x3306, 0x02},
    {0x3337, 0x44},
    {0x3338, 0x40},
    {0x3339, 0x70},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Brightness0[]=
{
    // Brightness -2
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x49},
    {0x339a, 0x20},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Brightness1[]=
{
    // Brightness -1
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x49},
    {0x339a, 0x10},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Brightness2[]=
{
    //  Brightness 0
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x00},
    {0x3390, 0x41},
    {0x339a, 0x00},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Brightness3[]=
{
    // Brightness +1
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x41},
    {0x339a, 0x10},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Brightness4[]=
{
    //  Brightness +2
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x41},
    {0x339a, 0x20},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Brightness5[]=
{
    //  Brightness +3
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04}, //bit[2] enable
    {0x3390, 0x41}, //bit[3] sign of brightness
    {0x339a, 0x30},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Effect_Normal[] =
{
    {0x3391, 0x00},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Effect_WandB[] =
{
    {0x3391, 0x20},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Effect_Sepia[] =
{
    {0x3391, 0x18},
    {0x3396, 0x40},
    {0x3397, 0xa6},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Effect_Negative[] =
{
    //Negative
    {0x3391, 0x40}, //bit[6] negative
    {0x0000, 0x00}
};
static  struct reginfo ov2655_Effect_Bluish[] =
{
    // Bluish
    {0x3391, 0x18},
    {0x3396, 0xa0},
    {0x3397, 0x40},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Effect_Green[] =
{
    //  Greenish
    {0x3391, 0x18},
    {0x3396, 0x60},
    {0x3397, 0x60},
    {0x0000, 0x00}
};
static  struct reginfo ov2655_Exposure0[]=
{
    //-3
    {0x3047, 0x05},
    {0x3018, 0x40},
    {0x3019, 0x30},
    {0x301a, 0x71},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Exposure1[]=
{
    //-2
    {0x3047, 0x05},
    {0x3018, 0x5a},
    {0x3019, 0x4a},
    {0x301a, 0xc2},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Exposure2[]=
{
    //-0.3EV
    {0x3047, 0x05},
    {0x3018, 0x6a},
    {0x3019, 0x5a},
    {0x301a, 0xd4},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Exposure3[]=
{
    //default
    {0x3047, 0x05},
    {0x3018, 0x78},
    {0x3019, 0x68},
    {0x301a, 0xd4},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Exposure4[]=
{
    // 1
    {0x3047, 0x05},
    {0x3018, 0x88},
    {0x3019, 0x78},
    {0x301a, 0xd5},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Exposure5[]=
{
    // 2
    {0x3047, 0x05},
    {0x3018, 0xa8},
    {0x3019, 0x98},
    {0x301a, 0xe6},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Exposure6[]=
{
    // 3
    {0x3047, 0x05},
    {0x3018, 0xc8},
    {0x3019, 0xb8},
    {0x301a, 0xf7},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Saturation0[]=
{
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x02},
    {0x3394, 0x40},
    {0x3395, 0x40},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Saturation1[]=
{
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x02},
    {0x3394, 0x50},
    {0x3395, 0x50},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Saturation2[]=
{
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x02}, //enable color saturation
    {0x3394, 0x70},
    {0x3395, 0x70},
    {0x0000, 0x00}
};


static  struct reginfo ov2655_Contrast0[]=
{
    //Contrast -3
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x45},
    {0x3398, 0x18},
    {0x3399, 0x18},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Contrast1[]=
{
    //Contrast -2
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x45},
    {0x3398, 0x18},
    {0x3399, 0x18},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Contrast2[]=
{
    // Contrast -1
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x45},
    {0x3398, 0x1c},
    {0x3399, 0x1c},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Contrast3[]=
{
    //Contrast 0
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x00},
    {0x3390, 0x41},
    {0x3398, 0x20},
    {0x3399, 0x20},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Contrast4[]=
{
    //Contrast +1
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x45},
    {0x3398, 0x24},
    {0x3399, 0x24},
    {0x0000, 0x00}
};


static  struct reginfo ov2655_Contrast5[]=
{
    //Contrast +2
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04},
    {0x3390, 0x45},
    {0x3398, 0x28},
    {0x3399, 0x28},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_Contrast6[]=
{
    //Contrast +3
    {0x3301, 0xff},//bit[7]:1, enable SDE
    {0x3391, 0x04}, //bit[2] enable contrast/brightness
    {0x3390, 0x45}, //bit[2] Yoffset sign
    {0x3398, 0x2c},
    {0x3399, 0x2c},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_MirrorOn[]=
{
    {0x3069, 0x84},	
    {0x307c, 0x13},	// if this don't work properly, refer to datasheet
    {0x3087, 0x02},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_MirrorOff[]=
{
    {0x3069, 0x84},
    {0x307c, 0x10},	// if this don't work properly, refer to datasheet
    {0x3087, 0x02},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_FlipOn[]=
{
    {0x300e, 0x34},
    {0x300f, 0xa6},
    {0x3010, 0x81},
    {0x3082, 0x01},
    {0x30f4, 0x01},
    {0x3090, 0x3b}, 	// if this don't work properly, refer to datasheet
    {0x3091, 0xc0},
    {0x30ac, 0x42},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_FlipOff[]=
{
    {0x300e, 0x34},
    {0x300f, 0xa6},
    {0x3010, 0x81},
    {0x3082, 0x01},
    {0x30f4, 0x01},
    {0x3090, 0x33},	// if this don't work properly, refer to datasheet
    {0x3091, 0xc0},
    {0x30ac, 0x42},
    {0x0000, 0x00}
};

static  struct reginfo ov2655_SceneAuto[] =
{
#if 0                           /* ddl@rock-chips.com : */
    {0x3014, 0x04},
    {0x3015, 0x00},
    {0x302e, 0x00},
    {0x302d, 0x00},
    {0x0000, 0x00}
#else
    {0x3014, 0x84},
    {0x3015, 0x02},
    {0x302e, 0x00},
    {0x302d, 0x00},
    {0x0000, 0x00}
#endif
};

static  struct reginfo ov2655_SceneNight[] =
{
#if 1
    //30fps ~ 5fps night mode for 60/50Hz light environment, 24Mhz clock input,36Mzh pclk
    {0x300e, 0x34},
    {0x3011, 0x00},
    {0x302c, 0x00},
    {0x3071, 0x00},
    {0x3070, 0xb9},
    {0x301c, 0x02},
    {0x3073, 0x00},
    {0x3072, 0x9a},
    {0x301d, 0x03},
    {0x3014, 0x0c},
    {0x3015, 0x50},//add 5 dummy frame
    {0x302e, 0x00},
    {0x302d, 0x00},
    {0x0000, 0x00}
#else
    //15fps ~ 5fps night mode for 60/50Hz light environment, 24Mhz clock input,18Mhz pclk
    {0x300e, 0x34},
    {0x3011, 0x01},
    {0x302c, 0x00},
    {0x3071, 0x00},
    {0x3070, 0x5d},
    {0x301c, 0x05},
    {0x3073, 0x00},
    {0x3072, 0x4d},
    {0x301d, 0x07},
    {0x3014, 0x0c},
    {0x3015, 0x50},
    {0x302e, 0x00},
    {0x302d, 0x00},
#endif
};


static struct reginfo ov2655_Zoom0[] =
{
    {0x0, 0x0},
};

static struct reginfo ov2655_Zoom1[] =
{
     {0x0, 0x0},
};

static struct reginfo ov2655_Zoom2[] =
{
    {0x0, 0x0},
};


static struct reginfo ov2655_Zoom3[] =
{
    {0x0, 0x0},
};

static struct reginfo *ov2655_ExposureSeqe[] = {ov2655_Exposure0, ov2655_Exposure1, ov2655_Exposure2, ov2655_Exposure3,
    ov2655_Exposure4, ov2655_Exposure5,ov2655_Exposure6,NULL,
};

static struct reginfo *ov2655_EffectSeqe[] = {ov2655_Effect_Normal, ov2655_Effect_WandB, ov2655_Effect_Negative,ov2655_Effect_Sepia,
    ov2655_Effect_Bluish, ov2655_Effect_Green,NULL,
};

static struct reginfo *ov2655_WhiteBalanceSeqe[] = {ov2655_WhiteB_Auto, ov2655_WhiteB_TungstenLamp1,ov2655_WhiteB_TungstenLamp2,
    ov2655_WhiteB_ClearDay, ov2655_WhiteB_Cloudy,NULL,
};

static struct reginfo *ov2655_BrightnessSeqe[] = {ov2655_Brightness0, ov2655_Brightness1, ov2655_Brightness2, ov2655_Brightness3,
    ov2655_Brightness4, ov2655_Brightness5,NULL,
};

static struct reginfo *ov2655_ContrastSeqe[] = {ov2655_Contrast0, ov2655_Contrast1, ov2655_Contrast2, ov2655_Contrast3,
    ov2655_Contrast4, ov2655_Contrast5, ov2655_Contrast6, NULL,
};

static struct reginfo *ov2655_SaturationSeqe[] = {ov2655_Saturation0, ov2655_Saturation1, ov2655_Saturation2, NULL,};

static struct reginfo *ov2655_MirrorSeqe[] = {ov2655_MirrorOff, ov2655_MirrorOn,NULL,};

static struct reginfo *ov2655_FlipSeqe[] = {ov2655_FlipOff, ov2655_FlipOn,NULL,};

static struct reginfo *ov2655_SceneSeqe[] = {ov2655_SceneAuto, ov2655_SceneNight,NULL,};

static struct reginfo *ov2655_ZoomSeqe[] = {ov2655_Zoom0, ov2655_Zoom1, ov2655_Zoom2, ov2655_Zoom3, NULL,};


static const struct v4l2_querymenu ov2655_menus[] =
{
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 0,  .name = "auto",  .reserved = 0, }, {  .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 1, .name = "incandescent",  .reserved = 0,},
            { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 2,  .name = "fluorescent", .reserved = 0,}, {  .id = V4L2_CID_DO_WHITE_BALANCE, .index = 3,  .name = "daylight", .reserved = 0,},
            { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 4,  .name = "cloudy-daylight", .reserved = 0,},
            { .id = V4L2_CID_EFFECT,  .index = 0,  .name = "none",  .reserved = 0, }, {  .id = V4L2_CID_EFFECT,  .index = 1, .name = "mono",  .reserved = 0,},
            { .id = V4L2_CID_EFFECT,  .index = 2,  .name = "negative", .reserved = 0,}, {  .id = V4L2_CID_EFFECT, .index = 3,  .name = "sepia", .reserved = 0,},
            { .id = V4L2_CID_EFFECT,  .index = 4, .name = "posterize", .reserved = 0,} ,{ .id = V4L2_CID_EFFECT,  .index = 5,  .name = "aqua", .reserved = 0,},
            { .id = V4L2_CID_SCENE,  .index = 0, .name = "auto", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 1,  .name = "night", .reserved = 0,},
            { .id = V4L2_CID_FLASH,  .index = 0,  .name = "off",  .reserved = 0, }, {  .id = V4L2_CID_FLASH,  .index = 1, .name = "auto",  .reserved = 0,},
            { .id = V4L2_CID_FLASH,  .index = 2,  .name = "on", .reserved = 0,}, {  .id = V4L2_CID_FLASH, .index = 3,  .name = "torch", .reserved = 0,},
};


static struct soc_camera_ops ov2655_ops = {
	.owner			= THIS_MODULE,
	.probe			= ov2655_video_probe,
	.remove			= ov2655_video_remove,
//    .suspend                     = ov2655_suspend,   // jyk
//    .resume                       = ov2655_resume,	//jyk
	.init			= ov2655_init,
	.release		= ov2655_release,
	.start_capture		= ov2655_start_capture,
	.stop_capture		= ov2655_stop_capture,
	.set_fmt_cap		= ov2655_set_fmt_cap,
	.try_fmt_cap		= ov2655_try_fmt_cap,
	.set_bus_param		= ov2655_set_bus_param,
	.query_bus_param	= ov2655_query_bus_param,
	.controls		= ov2655_controls,		// jyk
    .menus                         = ov2655_menus,	//jyk
    .num_controls		= ARRAY_SIZE(ov2655_controls),	//jyk
    .num_menus		= ARRAY_SIZE(ov2655_menus),		//jyk
    .get_control		= ov2655_get_control,
    .set_control		= ov2655_set_control,
    .get_ext_control        = ov2655_get_ext_control,		//jyk
    .set_ext_control        = ov2655_set_ext_control,		//jyk
	.get_chip_id		= ov2655_get_chip_id,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= ov2655_get_register,
	.set_register		= ov2655_set_register,
#endif
};
static int ov2655_set_brightness(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_BrightnessSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_BrightnessSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("\n OV2655 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("\n OV2655 Set Brightness - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    return -EINVAL;
}
static int ov2655_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_EffectSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_EffectSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("\n OV2655 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("\n OV2655 Set effect - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    return -EINVAL;
}
static int ov2655_set_exposure(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_ExposureSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_ExposureSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("\n OV2655 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("\n OV2655 Set Exposurce - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    return -EINVAL;
}

static int ov2655_set_saturation(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_SaturationSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_SaturationSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("\n OV2655 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("\n OV2655 Set Saturation - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    OV2655_TR("\n Saturation valure = %d is invalidate..    ******** ddl *********\n",value);
    return -EINVAL;
}

static int ov2655_set_contrast(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_ContrastSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_ContrastSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("\n OV2655 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("\n OV2655 Set Contrast - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    OV2655_TR("\n Contrast valure = %d is invalidate..    ******** ddl *********\n", value);
    return -EINVAL;
}

static int ov2655_set_mirror(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_MirrorSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_MirrorSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("\n OV2655 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("\n OV2655 Set Mirror - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    OV2655_TR("\n Mirror valure = %d is invalidate..    ******** ddl *********\n", value);
    return -EINVAL;
}


static int ov2655_set_flip(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_FlipSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_FlipSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("\n OV2655 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("\n OV2655 Set Flip - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    OV2655_TR("\n Flip valure = %d is invalidate..    ******** ddl *********\n", value);
    return -EINVAL;
}

static int ov2655_set_scene(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_SceneSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_SceneSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("\n OV2655 WriteReg Fail.. %x   ******** ddl *********\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("\n OV2655 Set Scene - %x   ******** ddl *********\n", value);
            return 0;
        }
    }
    OV2655_TR("\n Scene valure = %d is invalidate..    ******** ddl *********\n", value);
    return -EINVAL;
}

static int ov2655_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct ov2655 *ov2655;

    ov2655 = container_of(icd, struct ov2655, icd);

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (ov2655_WhiteBalanceSeqe[value - qctrl->minimum] != NULL)
        {
            if (ov2655_write_array(ov2655->client, ov2655_WhiteBalanceSeqe[value - qctrl->minimum]) != 0)
            {
                OV2655_TR("OV2655 WriteReg Fail.. %x\n", __LINE__);
                return -EINVAL;
            }
            OV2655_DG("ov2655_set_whiteBalance - %x\n", value);
            return 0;
        }
    }
    return -EINVAL;
}

static int ov2655_set_digitalzoom(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int *value)
{
    struct ov2655 *ov2655;
    int digitalzoom_cur, digitalzoom_total;

    ov2655 = container_of(icd, struct ov2655, icd);
    digitalzoom_cur = ov2655->info_priv.digitalzoom;
    digitalzoom_total = ov2655_controls[10].maximum;

    if ((*value > 0) && (digitalzoom_cur >= digitalzoom_total))
    {
        OV2655_TR("ov2655 digitalzoom is maximum - %x\n", digitalzoom_cur);
        return -EINVAL;
    }

    if  ((*value < 0) && (digitalzoom_cur <= ov2655_controls[10].minimum))
    {
        OV2655_TR("ov2655 digitalzoom is minimum - %x\n", digitalzoom_cur);
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

    if (ov2655_ZoomSeqe[digitalzoom_cur] != NULL)
    {
        if (ov2655_write_array(ov2655->client, ov2655_ZoomSeqe[digitalzoom_cur]) != 0)
        {
            OV2655_TR("OV2655 WriteReg Fail.. %x\n", __LINE__);
            return -EINVAL;
        }
        OV2655_DG("ov2655_set_digitalzoom - %x\n", *value);
        return 0;
    }

    return -EINVAL;
}


static int ov2655_get_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct ov2655 *ov2655;

    qctrl = soc_camera_find_qctrl(&ov2655_ops, ctrl->id);

    if (!qctrl)
    {
        OV2655_TR("\n%s..%s..%d.. ioctrl is faild    ******** ddl *********\n",__FUNCTION__,__FILE__,__LINE__);
        return -EINVAL;
    }

    ov2655 = container_of(icd, struct ov2655, icd);
	
	//struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
	//int data;

    switch (ctrl->id)
    {
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
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                ctrl->value = ov2655->info_priv.whiteBalance;
                break;
            }
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
        default :
                break;
    }
    return 0;
}


static int ov2655_set_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	//struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
	const struct v4l2_queryctrl *qctrl;
    struct ov2655 *ov2655;

	qctrl = soc_camera_find_qctrl(&ov2655_ops, ctrl->id);

    if (!qctrl)
    {
        OV2655_TR("\n OV2655 ioctrl id = %d  is invalidate   ******** ddl *********\n", ctrl->id);
        return -EINVAL;
    }

    ov2655 = container_of(icd, struct ov2655, icd);


    switch (ctrl->id)
    {
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
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                if (ctrl->value != ov2655->info_priv.whiteBalance)
                {
                    if (ov2655_set_whiteBalance(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    ov2655->info_priv.whiteBalance = ctrl->value;
                }
                break;
            }
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
        default :
            break;
    }
	
	return 0;
}


static int ov2655_get_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct ov2655 *ov2655;

    qctrl = soc_camera_find_qctrl(&ov2655_ops, ext_ctrl->id);

    if (!qctrl)
    {
        OV2655_TR("\n%s..%s..%d.. ioctrl is faild    ******** ddl *********\n",__FUNCTION__,__FILE__,__LINE__);
        return -EINVAL;
    }

    ov2655 = container_of(icd, struct ov2655, icd);

    switch (ext_ctrl->id)
    {
        case V4L2_CID_SCENE:
            {
                ext_ctrl->value = ov2655->info_priv.scene;
                break;
            }
        case V4L2_CID_EFFECT:
            {
                ext_ctrl->value = ov2655->info_priv.effect;
                break;
            }
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                ext_ctrl->value = ov2655->info_priv.digitalzoom;
                break;
            }
        case V4L2_CID_ZOOM_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                ext_ctrl->value = ov2655->info_priv.focus;
                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FLASH:
            {
                ext_ctrl->value = ov2655->info_priv.flash;
                break;
            }
        default :
            break;
    }
    return 0;
}
static int ov2655_set_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct ov2655 *ov2655;


    qctrl = soc_camera_find_qctrl(&ov2655_ops, ext_ctrl->id);

    if (!qctrl)
    {
        OV2655_TR("\n OV2655 ioctrl id = %d  is invalidate   ******** ddl *********\n", ext_ctrl->id);
        return -EINVAL;
    }

    ov2655 = container_of(icd, struct ov2655, icd);

    switch (ext_ctrl->id)
    {
        case V4L2_CID_SCENE:
            {
                if (ext_ctrl->value != ov2655->info_priv.scene)
                {
                    if (ov2655_set_scene(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    ov2655->info_priv.scene = ext_ctrl->value;
                }
                break;
            }
        case V4L2_CID_EFFECT:
            {
                if (ext_ctrl->value != ov2655->info_priv.effect)
                {
                    if (ov2655_set_effect(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    ov2655->info_priv.effect= ext_ctrl->value;
                }
                break;
            }
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

        case V4L2_CID_FLASH:
            {
                ov2655->info_priv.flash = ext_ctrl->value;

                OV2655_DG("ov2655 flash is %x\n", ov2655->info_priv.flash);
                break;
            }



        default:
            break;
    }

    return 0;
}



/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int ov2655_video_probe(struct soc_camera_device *icd)
{
    //struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
    int ret;

    /* We must have a parent by now. And it cannot be a wrong one.
     * So this entire test is completely redundant. */
    if (!icd->dev.parent ||
            to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
        return -ENODEV;

    OV2655_DG("\n%s..%s..%d    ******** ddl *********\n",__FUNCTION__,__FILE__,__LINE__);
    icd->formats = ov2655_colour_formats;
    icd->num_formats = ARRAY_SIZE(ov2655_colour_formats);

	/* Now that we know the model, we can start video */
	ret = soc_camera_video_start(icd);
	if (ret)
		goto eisis;

	return 0;

eisis:

	return ret;
}

static void ov2655_video_remove(struct soc_camera_device *icd)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", ov2655->client->addr,
		ov2655->icd.dev.parent, ov2655->icd.vdev);
	soc_camera_video_stop(&ov2655->icd);
}

static int ov2655_remove(struct i2c_client *client)
{
	struct ov2655 *ov2655 = i2c_get_clientdata(client);

    soc_camera_device_unregister(&ov2655->icd);
//    ov2655_gpio_release(ov2655);	//jyk
    kfree(ov2655);

	return 0;
}

static int ov2655_detach_client(struct i2c_client *client)
{
	ov2655_remove(client);

	return i2c_detach_client(client);
}

static int ov2655_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data_ov2655, ov2655_probe);
}

static struct i2c_driver ov2655_driver =
{
	.driver = {
		.name = "ov2655",
	    },
	.id 	= OV2655_IIC_ADDR,
	.attach_adapter = &ov2655_attach_adapter,
	.detach_client  = &ov2655_detach_client,
};

extern struct platform_device rk28_device_camera;
#if 0
static struct soc_camera_link iclink = {//nzy add
    .bus_id = 33, /* Must match with the camera ID above */
    .gpio   = 1,
};
#endif
    
static int ov2655_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct ov2655 *ov2655;
	struct soc_camera_device *icd;
//	struct soc_camera_link *icl;
	struct i2c_client *client = NULL;
	int ret;
	
    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
    {
	    ret = -EIO;
            goto exit_check_functionality_failed;
	}

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
    if (!client)
    {
	    ret = -ENOMEM;
            goto exit_alloc_client_failed;
	}

	strlcpy(client->name, "ov2655", I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &ov2655_driver;
	client->addressBit = I2C_7BIT_ADDRESS_16BIT_REG; // jyk, tbd 
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 100;	
	ret = i2c_attach_client(client);
	if (ret) {
            goto exit_i2c_attach_client_failed;
	}
    OV2655_DG("\n%s..%s..%d    ******** ddl *********\n",__FUNCTION__,__FILE__,__LINE__);

	ov2655 = kzalloc(sizeof(struct ov2655), GFP_KERNEL);
	if (!ov2655) {
       	    ret = -ENOMEM;
            goto exit_alloc_data_failed;
	}

	ov2655->client = client;
	i2c_set_clientdata(client, ov2655);
        //icl = &iclink;//client->dev.platform_data;

	/* Second stage probe - when a capture adapter is there */
	icd = &ov2655->icd;
	icd->ops	= &ov2655_ops;
	icd->control	= &client->dev;
	icd->x_min	= 0;
	icd->y_min	= 0;
	icd->x_current	= 0;
	icd->y_current	= 0;
    icd->width_min	= OV2655_MIN_WIDTH;
    icd->width_max	= OV2655_MAX_WIDTH;
    icd->height_min	= OV2655_MIN_HEIGHT;
    icd->height_max	= OV2655_MAX_HEIGHT;
    icd->y_skip_top	= 0;
    icd->iface	= rk28_device_camera.id;          /* ddl@rock-chips.com : deault sensor is attach to rk28_vip */
	/* Default datawidth - this is the only width this camera (normally)
	 * supports. It is only with extra logic that it can support
	 * other widths. Therefore it seems to be a sensible default. */
	//ov2655->datawidth = 8; //jyk, 
	
	/* Simulated autoexposure. If enabled, we calculate shutter width
	 * ourselves in the driver based on vertical blanking and frame width */
	 //	ov2655->autoexposure = 1; //jyk, 
		
	 /* sensor ov2655 information for initialization  */
			
    ov2655->info_priv.whiteBalance = ov2655_controls[0].default_value;
    ov2655->info_priv.brightness = ov2655_controls[1].default_value;
    ov2655->info_priv.effect = ov2655_controls[2].default_value;
    ov2655->info_priv.exposure = &icd->exposure;
    *(ov2655->info_priv.exposure) = ov2655_controls[3].default_value;
    ov2655->info_priv.saturation = ov2655_controls[4].default_value;
    ov2655->info_priv.contrast = ov2655_controls[5].default_value;
    ov2655->info_priv.mirror = ov2655_controls[6].default_value;
    ov2655->info_priv.flip = ov2655_controls[7].default_value;
    ov2655->info_priv.scene = ov2655_controls[8].default_value;
    ov2655->info_priv.digitalzoom = ov2655_controls[10].default_value;
    ov2655->info_priv.focus = ov2655_controls[12].default_value;
    ov2655->info_priv.flash = ov2655_controls[13].default_value;

	ret = ov2655_gpio_request(ov2655);// , icl);
	if (ret)
        goto exit_gpio_request_failed;
		
	ret = soc_camera_device_register(icd);
	if (ret)
        goto exit_soc_camera_device_register_failed;

    printk("\n%s..%s..%d    ******** ddl *********\n",__FUNCTION__,__FILE__,__LINE__);
	return 0;

exit_soc_camera_device_register_failed:
    ov2655_gpio_release(ov2655);
exit_gpio_request_failed:
	kfree(ov2655);
exit_alloc_data_failed:
   i2c_detach_client(client);
exit_i2c_attach_client_failed:
	kfree(client);
exit_alloc_client_failed:
exit_check_functionality_failed:
	return ret;
}

static int __init ov2655_mod_init(void)
{
    OV2655_DG("\n%s..%s..%d    ******** ddl *********\n",__FUNCTION__,__FILE__,__LINE__);
	return i2c_add_driver(&ov2655_driver);
}

static void __exit ov2655_mod_exit(void)
{
	i2c_del_driver(&ov2655_driver);
}

module_init(ov2655_mod_init);
module_exit(ov2655_mod_exit);

MODULE_DESCRIPTION("OV2655 Camera sensor driver");
MODULE_AUTHOR("lbt <kernel@rock-chips>");
MODULE_LICENSE("GPL");

