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

#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>
#include <linux/delay.h>
                 
#include <asm/gpio.h>

struct reginfo{
   u16 reg;
   u16 val;
};

/* init 800X600 SVGA */
static struct reginfo MT9D112_init_data[] = {
	 {0xEFFF, 1},	 
	 {0x301A,0x0ACC},	 // RESET_REGISTER
	 {0x3202,0x0008},	 // STANDBY_CONTROL
	 {0xEFFF, 10},		  
	 {0x341E,0x8F09},	 // PLL_CLK_IN_CONTROL
	 {0x341C,0x0218},	 // PLL_DIVIDERS1
	 {0xEFFF, 10},				  
	 {0x341E,0x8F09},	 // PLL_CLK_IN_CONTROL
	 {0x341E,0x8F08},	 // PLL_CLK_IN_CONTROL
	 {0x3044,0x0540},	 // DARK_CONTROL
	 {0x3216,0x02CF},	 // INTERNAL_CLOCK_CONTROL
	 {0x321C,0x0402},	 // OF_CONTROL_STATUS
	 {0x3212,0x0001},	 // FACTORY_BYPASS
	 {0x341E,0x8F09},	 // PLL_CLK_IN_CONTROL
	 {0x341C,0x0120},	 // PLL_DIVIDERS1
	 {0x341E,0x8F09},	 // PLL_CLK_IN_CONTROL
	 {0x341E,0x8F08},	 // PLL_CLK_IN_CONTROL
	 {0x3044,0x0540},	 // DARK_CONTROL
	 {0x3216,0x02CF},	 // INTERNAL_CLOCK_CONTROL
	 {0x321C,0x0402},	 // OF_CONTROL_STATUS
	 {0x3212,0x0001},	 // FACTORY_BYPASS//供品干扰程序利用之前的07005
	 {0x32d4,0x0065},
	 {0x32d6,0x0077},
        {0x32d8,0x0077},
	 {0x32da,0x0053},
	 //[MT9D12 (SOC2020) Register Wizard Defaults]
	 {0x341E,0x8F09},		//PLL/ Clk_in control: BYPASS PLL = 36617
	 {0x341C,0x0120},		//PLL Control 1 = 288
	 {0xEFFF, 1}, 	  // Allow PLL to lock
	 {0x341E,0x8F09},		//PLL/ Clk_in control: PLL ON}, bypassed = 36617
	 {0x341E,0x8F08},		//PLL/ Clk_in control: USE PLL = 36616
	 {0x3044,0x0540},		//Reserved = 1344
	 {0x3216,0x02CF},		//Internal Clock Control = 719
	 {0x321C,0x0402},		//OF Control Status = 1026
	 {0x3212,0x0001},		//Factory Bypass = 1

	 
 //{0x0003},0x0280
	 {0x338C,0x2703},	   
	 {0x3390,0x0280},
 //{0x0005,0x01E0
	 {0x338C,0x2705},	   
	 {0x3390,0x01e0},

	 {0x338C,0x2707},		//Output Width (B)
	 {0x3390,0x0640},		//		= 1600
	 {0x338C,0x2709},		//Output Height (B)
	 {0x3390,0x04B0},		//		= 1200
	 {0x338C,0x270D},		//Row Start (A)
	 {0x3390,0x0000},		//		= 0
	 {0x338C,0x270F},		//Column Start (A)
	 {0x3390,0x0000},		//		= 0
	 {0x338C,0x2711},		//Row End (A)
	 {0x3390,0x04BD},		//		= 1213
	 {0x338C,0x2713},		//Column End (A)
	 {0x3390,0x064D},		//		= 1613
	 {0x338C,0x2715},		//Extra Delay (A) //{0x
	 {0x3390,0x01A8},		//		= 0//{0x0000
	 {0x338C,0x2717},		//Row Speed (A)
	 {0x3390,0x2111},		//		= 8465
	 {0x338C,0x2719},		//Read Mode (A)
	 //{0x330},0x046C},		  //	  = 1132
	 {0x3390,0x046F},		//		= 1132
	 {0x338C,0x271B},		//sensor_sample_time_pck (A)
	 {0x3390,0x024F},		//		= 591
	 {0x338C,0x271D},		//sensor_fine_correction (A)
	 {0x3390,0x0102},		//		= 258
	 {0x338C,0x271F},		//sensor_fine_IT_min (A)
	 {0x3390,0x0279},		//		= 633
	 {0x338C,0x2721},		//sensor_fine_IT_max_margin (A)
	 {0x3390,0x0155},		//		= 341
	 {0x338C,0x2723},		//Frame Lines (A)
	 {0x3390,0x0341},		//		= 659 {0x0293
	 {0x338C,0x2725},		//Line Length (A)
	 {0x3390,0x060F},		//		= 1551
	 {0x338C,0x2727},		//sensor_dac_id_4_5 (A)
	 {0x3390,0x2020},		//		= 8224
	 {0x338C,0x2729},		//sensor_dac_id_6_7 (A)
	 {0x3390,0x2020},		//		= 8224
	 {0x338C,0x272B},		//sensor_dac_id_8_9 (A)
	 {0x3390,0x1020},		//		= 4128
	 {0x338C,0x272D},		//sensor_dac_id_10_11 (A)
	 {0x3390,0x2007},		//		= 8199
	 {0x338C,0x272F},		//Row Start (B)
	 {0x3390,0x0004},		//		= 4
	 {0x338C,0x2731},		//Column Start (B)
	 {0x3390,0x0004},		//		= 4
	 {0x338C,0x2733},		//Row End (B)
	 {0x3390,0x04BB},		//		= 1211
	 {0x338C,0x2735},		//Column End (B)
	 {0x3390,0x064B},		//		= 1611
	 {0x338C,0x2737},		//Extra Delay (B)
	 {0x3390,0x022C},		//		= 556
	 {0x338C,0x2739},		//Row Speed (B)
	 {0x3390,0x2111},		//		= 8465
	 {0x338C,0x273B},		//Read Mode (B)
	 //{0x330},0x0024},		  //	  = 36
	 {0x3390,0x0027},		//		= 36
	 {0x338C,0x273D},		//sensor_sample_time_pck (B)
	 {0x3390,0x0120},		//		= 288
	 {0x338C,0x273F},		//sensor_fine_correction (B)
	 {0x3390,0x00A4},		//		= 164
	 {0x338C,0x2741},		//sensor_fine_IT_min (B)
	 {0x3390,0x0169},		//		= 361
	 {0x338C,0x2743},		//sensor_fine_IT_max_margin (B)
	 {0x3390,0x00A4},		//		= 164
	 {0x338C,0x2745},		//Frame Lines (B)
	 {0x3390,0x0625},		//		= 1573
	 {0x338C,0x2747},		//Line Length (B)
	 {0x3390,0x0824},		//		= 2084
 
	 //CH 2
	 {0x338C,0x2751},		//Crop_X0 (A)
	 {0x3390,0x0000},		//		= 0
	 {0x338C,0x2753},		//Crop_X1 (A)
	 {0x3390,0x0320},		//		= 800
	 {0x338C,0x2755},		//Crop_Y0 (A)
	 {0x3390,0x0000},		//		= 0
	 {0x338C,0x2757},		//Crop_Y1 (A)
	 {0x3390,0x0258},		//		= 600
	 {0x338C,0x275F},		//Crop_X0 (B)
	 {0x3390,0x0000},		//		= 0
	 {0x338C,0x2761},		//Crop_X1 (B)
	 {0x3390,0x0640},		//		= 1600
	 {0x338C,0x2763},		//Crop_Y0 (B)
	 {0x3390,0x0000},		//		= 0
	 {0x338C,0x2765},		//Crop_Y1 (B)
	 {0x3390,0x04B0},		//		= 1200


//[CCM]												  
	 {0x338C,0x2306},	 // MCU_ADDRESS [AWB_CCM_L_0]	   
	 {0x3390,0x0055},	 // MCU_DATA_0	   
	 {0x337C,0x0004},	   //cgz oppo 20081021
	 {0x338C,0x2308},	 // MCU_ADDRESS [AWB_CCM_L_1]	   
	 {0x3390,0xFEC3},	 // MCU_DATA_0					   
	 {0x338C,0x230A},	 // MCU_ADDRESS [AWB_CCM_L_2]	   
	 {0x3390,0x0236},	 // MCU_DATA_0					   
	 {0x338C,0x230C},	 // MCU_ADDRESS [AWB_CCM_L_3]	   
	 {0x3390,0xFE2A},	 // MCU_DATA_0					   
	 {0x338C,0x230E},	 // MCU_ADDRESS [AWB_CCM_L_4]	   
	 {0x3390,0x0229},	 // MCU_DATA_0					   
	 {0x338C,0x2310},	 // MCU_ADDRESS [AWB_CCM_L_5]	   
	 {0x3390,0x014A},	 // MCU_DATA_0					   
	 {0x338C,0x2312},	 // MCU_ADDRESS [AWB_CCM_L_6]	   
	 {0x3390,0xFEB0},	 // MCU_DATA_0					   
	 {0x338C,0x2314},	 // MCU_ADDRESS [AWB_CCM_L_7]	   
	 {0x3390,0xF867},	 // MCU_DATA_0					   
	 {0x338C,0x2316},	 // MCU_ADDRESS [AWB_CCM_L_8]	   
	 {0x3390,0x0ACD},	 // MCU_DATA_0					   
	 {0x338C,0x2318},	 // MCU_ADDRESS [AWB_CCM_L_9]	   
	 {0x3390,0x0024},	 // MCU_DATA_0					   
	 {0x338C,0x231A},	 // MCU_ADDRESS [AWB_CCM_L_10]	   
	 {0x3390,0x003D},	 // MCU_DATA_0					   
	 {0x338C,0x231C},	 // MCU_ADDRESS [AWB_CCM_RL_0]	   
	 {0x3390,0x0353},	 // MCU_DATA_0					   
	 {0x338C,0x231E},	 // MCU_ADDRESS [AWB_CCM_RL_1]	   
	 {0x3390,0xFECA},	 // MCU_DATA_0					   
	 {0x338C,0x2320},	 // MCU_ADDRESS [AWB_CCM_RL_2]	   
	 {0x3390,0xFDE6},	 // MCU_DATA_0					   
	 {0x338C,0x2322},	 // MCU_ADDRESS [AWB_CCM_RL_3]	   
	 {0x3390,0x011A},	 // MCU_DATA_0					   
	 {0x338C,0x2324},	 // MCU_ADDRESS [AWB_CCM_RL_4]	   
	 {0x3390,0x01B2},	 // MCU_DATA_0					   
	 {0x338C,0x2326},	 // MCU_ADDRESS [AWB_CCM_RL_5]	   
	 {0x3390,0xFCE2},	 // MCU_DATA_0					   
	 {0x338C,0x2328},	 // MCU_ADDRESS [AWB_CCM_RL_6]	   
	 {0x3390,0x014A},	 // MCU_DATA_0					   
	 {0x338C,0x232A},	 // MCU_ADDRESS [AWB_CCM_RL_7]	   
	 {0x3390,0x060E},	 // MCU_DATA_0					   
	 {0x338C,0x232C},	 // MCU_ADDRESS [AWB_CCM_RL_8]	   
	 {0x3390,0xF81D},	 // MCU_DATA_0					   
	 {0x338C,0x232E},	 // MCU_ADDRESS [AWB_CCM_RL_9]	   
	 {0x3390,0x0010},	 // MCU_DATA_0					   
	 {0x338C,0x2330},	 // MCU_ADDRESS [AWB_CCM_RL_10]    
	 {0x3390,0xFFEC},	 // MCU_DATA_0					   
	 {0x338C,0xA348},	 // MCU_ADDRESS [AWB_GAIN_BUFFER_SP
	 {0x3390,0x0008},	 // MCU_DATA_0					   
	 {0x338C,0xA349},	 // MCU_ADDRESS [AWB_JUMP_DIVISOR] 
	 {0x3390,0x0002},	 // MCU_DATA_0					   
	 {0x338C,0xA34A},	 // MCU_ADDRESS [AWB_GAIN_MIN]	   
	 {0x3390,0x0059},	 // MCU_DATA_0					   
	 {0x338C,0xA34B},	 // MCU_ADDRESS [AWB_GAIN_MAX]	   
	 {0x3390,0x00A6},	 // MCU_DATA_0					   
	 {0x338C,0xA34F},	 // MCU_ADDRESS [AWB_CCM_POSITION_M
	 {0x3390,0x0000},	 // MCU_DATA_0					   
	 {0x338C,0xA350},	 // MCU_ADDRESS [AWB_CCM_POSITION_M
	 {0x3390,0x007F},	 // MCU_DATA_0					   
	 {0x338C,0xA352},	 // MCU_ADDRESS [AWB_SATURATION]   
	 {0x3390,0x001A},	 // MCU_DATA_0					   
	 {0x338C,0xA353},	 // MCU_ADDRESS [AWB_MODE]		   
	 {0x3390,0x0001},	 // MCU_DATA_0					   
	 {0x338C,0xA35B},	 // MCU_ADDRESS [AWB_STEADY_BGAIN_O
	 {0x3390,0x0078},	 // MCU_DATA_0					   
	 {0x338C,0xA35C},	 // MCU_ADDRESS [AWB_STEADY_BGAIN_O
	 {0x3390,0x0086},	 // MCU_DATA_0					   
	 {0x338C,0xA35D},	 // MCU_ADDRESS [AWB_STEADY_BGAIN_I
	 {0x3390,0x007E},	 // MCU_DATA_0					   
	 {0x338C,0xA35E},	 // MCU_ADDRESS [AWB_STEADY_BGAIN_I
	 {0x3390,0x0082},	 // MCU_DATA_0					   
	 {0x338C,0x235F},	 // MCU_ADDRESS [AWB_CNT_PXL_TH]   
	 {0x3390,0x0040},	 // MCU_DATA_0					   
	 {0x338C,0xA361},	 // MCU_ADDRESS [AWB_TG_MIN0]	   
	 {0x3390,0x00D7},	 // MCU_DATA_0					   
	 {0x338C,0xA362},	 // MCU_ADDRESS [AWB_TG_MAX0]	   
	 {0x3390,0x00F6},	 // MCU_DATA_0					   
	 {0x338C,0xA302},	 // MCU_ADDRESS [AWB_WINDOW_POS]   
	 {0x3390,0x0000},	 // MCU_DATA_0					   
	 {0x338C,0xA303},	 // MCU_ADDRESS [AWB_WINDOW_SIZE]  
	 {0x3390,0x00EF},	 // MCU_DATA_0					   
											
	 {0x338C,0xA364},  // MCU_ADDRESS [AWB_KR_L]		   
	 {0x3390,0x0098},	 // MCU_DATA_0					   
	 {0x338C,0xA365},	 // MCU_ADDRESS [AWB_KG_L]		   
	 {0x3390,0x0096},	 // MCU_DATA_0					   
	 {0x338C,0xA366},	 // MCU_ADDRESS [AWB_KB_L]		   
	 {0x3390,0x0084},	 // MCU_DATA_0					   
	 {0x338C,0xA367},	 // MCU_ADDRESS [AWB_KR_R]		   
	 {0x3390,0x0087},	 // MCU_DATA_0					   
	 {0x338C,0xA368},	 // MCU_ADDRESS [AWB_KG_R]		   
	 {0x3390,0x0080},	 // MCU_DATA_0					   
	 {0x338C,0xA369},	 // MCU_ADDRESS [AWB_KB_R]		   
	 {0x3390,0x0089},	 // MCU_DATA_0		 
	 {0x338C,0xA103},	 // MCU_ADDRESS [SEQ_CMD]		   
	 {0x3390,0x0006},	 // MCU_DATA_0	   
	 {0xEFFF, 100}, 
	   {0x338C,0xA103},	 // MCU_ADDRESS [SEQ_CMD]		   
	   {0x3390,0x0005},	 // MCU_DATA_0	 
	   {0xEFFF, 100}, 
 
	 
	 //[Fix Frame rate]
	 {0x338C,0xA123},	 // MCU_ADDRESS [SEQ_PREVIEW_0_FD]
	 {0x3390,0x0002},	 // MCU_DATA_0
	 {0x338C,0xA404},	 // MCU_ADDRESS [FD_MODE]
	 {0x3390,0x00C2},	 // MCU_DATA_0
	 {0x338C,0xA130},	 // MCU_ADDRESS [SEQ_PREVIEW_2_AE]	  // add by sheree 1008
	 {0x3390,0x0000}, // MCU_DATA_0 
	 {0x338C,0xA103},  // MCU_ADDRESS
	 {0x3390,0x0005},  // MCU_DATA_0
 
 
 
 //[noise reduce setting]
	 {0x338C,0xA115},	 // MCU_ADDRESS [SEQ_LLMODE]
	 {0x3390,0x00EF},	 // MCU_DATA_0
	 {0x338C,0xA118},	 // MCU_ADDRESS [SEQ_LLSAT1]
	 {0x3390,0x0036},	 // MCU_DATA_0
	 {0x338C,0xA119},	 // MCU_ADDRESS [SEQ_LLSAT2]
	 {0x3390,0x0003},	 // MCU_DATA_0
	 {0x338C,0xA11A},	 // MCU_ADDRESS [SEQ_LLINTERPTHRESH1]
	 {0x3390,0x000A},	 // MCU_DATA_0
	 {0x338C,0xA11B},	 // MCU_ADDRESS [SEQ_LLINTERPTHRESH2]
	 {0x3390,0x0020},	 // MCU_DATA_0
	 {0x338C,0xA11C},	 // MCU_ADDRESS [SEQ_LLAPCORR1]
	 {0x3390,0x0002},	 // MCU_DATA_0
	 {0x338C,0xA11D},	 // MCU_ADDRESS [SEQ_LLAPCORR2]
	 {0x3390,0x0000},	 // MCU_DATA_0
	 {0x338C,0xA11E},	 // MCU_ADDRESS [SEQ_LLAPTHRESH1]
	 {0x3390,0x0000},	 // MCU_DATA_0
	 {0x338C,0xA11F},	 // MCU_ADDRESS [SEQ_LLAPTHRESH2]
	 {0x3390,0x0004},	 // MCU_DATA_0
	 {0x338C,0xA13E},	 // MCU_ADDRESS [SEQ_NR_TH1_R]
	 {0x3390,0x0004},	 // MCU_DATA_0
	 {0x338C,0xA13F},	 // MCU_ADDRESS [SEQ_NR_TH1_G]
	 {0x3390,0x000E},	 // MCU_DATA_0
	 {0x338C,0xA140},	 // MCU_ADDRESS [SEQ_NR_TH1_B]
	 {0x3390,0x0004},	 // MCU_DATA_0
	 {0x338C,0xA141},	 // MCU_ADDRESS [SEQ_NR_TH1_OL]
	 {0x3390,0x0004},	 // MCU_DATA_0
	 {0x338C,0xA142},	 // MCU_ADDRESS [SEQ_NR_TH2_R]
	 {0x3390,0x0032},	 // MCU_DATA_0
	 {0x338C,0xA143},	 // MCU_ADDRESS [SEQ_NR_TH2_G]
	 {0x3390,0x000F},	 // MCU_DATA_0
	 {0x338C,0xA144},	 // MCU_ADDRESS [SEQ_NR_TH2_B]
	 {0x3390,0x0032},	 // MCU_DATA_0
	 {0x338C,0xA145},	 // MCU_ADDRESS [SEQ_NR_TH2_OL]
	 {0x3390,0x0032},	 // MCU_DATA_0
	 {0x338C,0xA146},	 // MCU_ADDRESS [SEQ_NR_GAINTH1]
	 {0x3390,0x0005},	 // MCU_DATA_0
	 {0x338C,0xA147},	 // MCU_ADDRESS [SEQ_NR_GAINTH2]
	 {0x3390,0x003A},	 // MCU_DATA_0
	 {0x338C,0xA14F},	 // MCU_ADDRESS [SEQ_CLUSTERDC_TH]
	 {0x3390,0x000D},	 // MCU_DATA_0
	 {0x338C,0xA103},		//Refresh Sequencer Mode
	 {0x3390,0x06  },	  //	  = 6
	 {0xEFFF, 200}, 
	 {0x338C,0xA103},
	 {0x3390,0x0005},
	 {0xEFFF, 200}, 
 //  [test-5_max index change_target change]
	  {0x338C,0xA118},	 // MCU_ADDRESS [SEQ_LLSAT1]
	  {0x3390,0x0020},//{0x0010},	 // MCU_DATA_0
	  {0x338C,0xA119},	 // MCU_ADDRESS [SEQ_LLSAT2]
	  {0x3390,0x0003},	 // MCU_DATA_0
	  {0x338C,0xA206},	 // MCU_ADDRESS [AE_TARGET]
	  //{0x3390},0x0050},  // MCU_DATA_0
	  {0x3390,0x0038},//{0x0031},	 // MCU_DATA_0
	  {0x338C,0xA207},	 // MCU_ADDRESS [AE_GATE]
	  //{0x3390},0x0005},  // MCU_DATA_0
	  {0x3390,0x000B},	 // MCU_DATA_0
	  {0x338C,0xA20C},	 // MCU_ADDRESS [AE_MAX_INDEX]
	  {0x3390,0x000A},	 // MCU_DATA_0
	  {0x338C,0xA109},	 // MCU_ADDRESS [SEQ_AE_FASTBUFF]
	  {0x3390,0x0020},	 //{0x0024 cgz oppo 2008-1020// MCU_DATA_0
	  {0x338C,0xA76D},	 // MCU_ADDRESS [MODE_GAM_CONT_A]
	  {0x3390,0x0003},	 // MCU_DATA_0
	  {0x338C,0xA76F},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_0]
	  {0x3390,0x0000},	 // MCU_DATA_0
	  {0x338C,0xA770},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_1]
	  {0x3390,0x0007},	 // MCU_DATA_0
	  {0x338C,0xA771},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_2]
	  {0x3390,0x0017},	 // MCU_DATA_0
	  {0x338C,0xA772},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_3]
	  {0x3390,0x003B},	 // MCU_DATA_0
	  {0x338C,0xA773},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_4]
	  {0x3390,0x0060},	 // MCU_DATA_0
	  {0x338C,0xA774},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_5]
	  {0x3390,0x007A},	 // MCU_DATA_0
	  {0x338C,0xA775},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_6]
	  {0x3390,0x008F},	 // MCU_DATA_0
	  {0x338C,0xA776},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_7]
	  {0x3390,0x00A0},	 // MCU_DATA_0
	  {0x338C,0xA777},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_8]
	  {0x3390,0x00AE},	 // MCU_DATA_0
	  {0x338C,0xA778},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_9]
	  {0x3390,0x00BA},	 // MCU_DATA_0
	  {0x338C,0xA779},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_10]
	  {0x3390,0x00C5},	 // MCU_DATA_0
	  {0x338C,0xA77A},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_11]
	  {0x3390,0x00CE},	 // MCU_DATA_0
	  {0x338C,0xA77B},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_12]
	  {0x3390,0x00D7},	 // MCU_DATA_0
	  {0x338C,0xA77C},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_13]
	  {0x3390,0x00DF},	 // MCU_DATA_0
	  {0x338C,0xA77D},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_14]
	  {0x3390,0x00E6},	 // MCU_DATA_0
	  {0x338C,0xA77E},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_15]
	  {0x3390,0x00ED},	 // MCU_DATA_0
	  {0x338C,0xA77F},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_16]
	  {0x3390,0x00F3},	 // MCU_DATA_0
	  {0x338C,0xA780},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_17]
	  {0x3390,0x00F9},	 // MCU_DATA_0
	  {0x338C,0xA781},	 // MCU_ADDRESS [MODE_GAM_TABLE_A_18]
	  {0x3390,0x00FF},	 // MCU_DATA_0
	  {0x338C,0xA103},	 // MCU_ADDRESS [SEQ_CMD]
	  {0x3390,0x0006},	 // MCU_DATA_0
	  {0xEFFF, 200}, 
	  {0x338C,0xA103},	 // MCU_ADDRESS [SEQ_CMD]
	  {0x3390,0x0005},	 // MCU_DATA_0
	  {0xEFFF, 200}, 
	  {0x35A4,0x04C3}, //092704c5

	 {0x0,0x0},
};
static struct reginfo MT9D112_640_480_INIT[]={
{0xefff,      1},	    
{0x3386, 0x2500},
 
//[initial]    
{0x301A, 0x0ACC},	 // RESET_REGISTER
{0x3202, 0x0008},	 // STANDBY_CONTROL
{0xefff,     10},		 	  
 
{0x341E, 0x8F08},	 // PLL_CLK_IN_CONTROL
{0xefff,     10},    
{0x341E, 0x8F09},	 // PLL_CLK_IN_CONTROL
{0x341C, 0x0250},   // PLL_DIVIDERS1
{0x341E, 0x8F08},	 // PLL_CLK_IN_CONTROL
{0x3044, 0x0540},	 // DARK_CONTROL
{0x3214, 0x06e6}, 
{0x3216, 0x02CF},	 // INTERNAL_CLOCK_CONTROL
{0x321C, 0x0402},	 // OF_CONTROL_STATUS
{0x3212, 0x0001},	 // FACTORY_BYPASS//1??·?éè?3ìDòà?ó????°μ?07005
 
 
 
//[MT9D112 (SO}C2020) Register Wizard Defaults
{0x341E, 0x8F09},        //PLL/ Clk_in control: BYPASS PLL = 36617
{0x341C, 0x0250},        //PLL Control 1 = 1126
{0xefff, 0x0001},              // Allow PLL to lock
{0x341E, 0x8F09},        //PLL/ Clk_in control: PLL ON, bypassed = 36617
{0x341E, 0x8F08},        //PLL/ Clk_in control: USE PLL = 36616
{0x3044, 0x0542},        //Reserved = 1346
{0x3216, 0x02CF},        //Internal Clock Control = 719
{0x321C, 0x0402},        //OF Control Status = 1026
{0x3212, 0x0001},        //Factory Bypass = 1
{0x338C, 0x2703},        //Output Width (A)
{0x3390, 0x0280},        //      = 640
{0x338C, 0x2705},        //Output Height (A)
{0x3390, 0x01E0},        //      = 480
{0x338C, 0x2707},        //Output Width (B)
{0x3390, 0x0640},        //      = 1600
{0x338C, 0x2709},        //Output Height (B)
{0x3390, 0x04B0},        //      = 1200
{0x338C, 0x270D},        //Row Start (A)
{0x3390, 0x0000},        //      = 0
{0x338C, 0x270F},        //Column Start (A)
{0x3390, 0x0000},        //      = 0
{0x338C, 0x2711},        //Row End (A)
{0x3390, 0x04BD},        //      = 1213
{0x338C, 0x2713},        //Column End (A)
{0x3390, 0x064D},        //      = 1613
{0x338C, 0x2715},        //Extra Delay (A)
{0x3390, 0x009F},        //      = 159
{0x338C, 0x2717},        //Row Speed (A)
{0x3390, 0x2112},        //      = 8466
{0x338C, 0x2719},        //Read Mode (A)
{0x3390, 0x046C},        //      = 1132     mirror
//0x3390, 0x04}6F,
 
{0x338C, 0x271B},        //sensor_sample_time_pck (A)
{0x3390, 0x0122},        //      = 290
{0x338C, 0x271D},        //sensor_fine_correction (A)
//ELD =0x3390}, 0xefff, 0x007B        //      = 123
{0x3390,0x007b}, 
{0x338C, 0x271F},        //sensor_fine_IT_min (A)
//ELD = 0x3390}, 0xefff, 0x013F        //      = 319
{0x3390,0x013f}, 
{0x338C, 0x2721},        //sensor_fine_IT_max_margin (A)
//ELD = 0x3390}, 0xefff, 0x00AB        //      = 171
{0x3390,0x00ab},
{0x338C, 0x2723},       //Frame Lines (A)
{0x3390, 0x0293},       //      = 659
{0x338C, 0x2725},       //Line Length (A)
{0x3390, 0x04E2},       //      = 1250
{0x338C, 0x2727},       //sensor_dac_id_4_5 (A)
//ELD = 0x3390}, 0xefff, 0x1010        //      = 4112
{0x3390, 0x1010}, 
{0x338C, 0x2729},        //sensor_dac_id_6_7 (A)
//ELD = 0x3390}, 0xefff, 0x2010        //      = 8208
{0x3390,0x2010}, 
{0x338C, 0x272B},        //sensor_dac_id_8_9 (A)
//ELD = 0x3390}, 0xefff, 0x1010        //      = 4112
{0x3390,0x1010}, 
{0x338C, 0x272D},        //sensor_dac_id_10_11 (A)
//ELD = 0x3390}, 0xefff, 0x1007        //      = 4103
{0x3390,0x1007}, 
{0x338C, 0x272F},        //Row Start (B)
{0x3390, 0x004 },       //      = 4
{0x338C, 0x2731},        //Column Start (B)
{0x3390, 0x004 },       //      = 4
{0x338C, 0x2733},        //Row End (B)
{0x3390, 0x4BB },       //      = 1211
{0x338C, 0x2735},        //Column End (B)
{0x3390, 0x64B },       //      = 1611
{0x338C, 0x2737},        //Extra Delay (B)
//0x3390, 0x14}7 ,       //      = 327 uxga=0x7.5fps
{0x3390, 0x02f6},         //      =758  uxga=5.11fps
{0x338C, 0x2739},        //Row Speed (B)
{0x3390, 0x2111},        //      = 8465
{0x338C, 0x273B},        //Read Mode (B)
{0x3390, 0x0024},        //      = 36//--------------FLIP
//0x3390, 0x00}27,    ///
 
{0x338C, 0x273D},        //sensor_sample_time_pck (B)
{0x3390, 0x0120},        //      = 288
{0x338C, 0x273F},        //sensor_fine_correction (B)
//ELD = 0x3390}, 0xefff, 0x00A4        //      = 164
{0x3390, 0x00a4},
{0x338C, 0x2741},        //sensor_fine_IT_min (B)
//ELD = 0x3390}, 0xefff, 0x0169        //      = 361
{0x3390,0x0169}, 
{0x338C, 0x2743},        //sensor_fine_IT_max_margin (B)
//ELD = 0x3390}, 0xefff, 0x00A4        //      = 164
{0x3390, 0x00a4},
{0x338C, 0x2745},        //Frame Lines (B)
// 0x3390, 0x0}588,        //      = 1672 -256
//0x3390, 0x06}88,        //      = 1672
{0x3390, 0x095b},        //      = 2395 uxga=5.11fps
{0x338C, 0x2747},        //Line Length (B)
{0x3390, 0x09C4},        //      = 2500
{0x338C, 0x2751},        //Crop_X0 (A)
{0x3390, 0x0000},        //      = 0
{0x338C, 0x2753},        //Crop_X1 (A)
{0x3390, 0x0320},        //      = 800
{0x338C, 0x2755},        //Crop_Y0 (A)
{0x3390, 0x0000},        //      = 0
{0x338C, 0x2757},        //Crop_Y1 (A)
{0x3390, 0x0258},        //      = 600
{0x338C, 0x275F},        //Crop_X0 (B)
{0x3390, 0x0000},        //      = 0
{0x338C, 0x2761},        //Crop_X1 (B)
{0x3390, 0x0640},        //      = 1600
{0x338C, 0x2763},        //Crop_Y0 (B)
{0x3390, 0x0000},        //      = 0
{0x338C, 0x2765},        //Crop_Y1 (B)
{0x3390, 0x04B0},        //      = 1200
{0x338C, 0x222E},        //R9 Step
{0x3390, 0x0066},        //      = 102
{0x338C, 0xA408},        //search_f1_50
{0x3390, 0x18  },      //      = 24
{0x338C, 0xA409},        //search_f2_50
{0x3390, 0x1B  },      //      = 27
{0x338C, 0xA40A},        //search_f1_60
{0x3390, 0x1D  },      //      = 29
{0x338C, 0xA40B},        //search_f2_60
{0x3390, 0x20  },      //      = 32
{0x338C, 0x2411},        //R9_Step_60 (A)
{0x3390, 0x0066},        //      = 102
{0x338C, 0x2413},        //R9_Step_50 (A)
{0x3390, 0x007A},        //      = 122
{0x338C, 0x2415},        //R9_Step_60 (B)
{0x3390, 0x0066},        //      = 102
{0x338C, 0x2417},        //R9_Step_50 (B)
{0x3390, 0x007A},        //      = 122
{0x338C, 0xA40D},        //Stat_min
{0x3390, 0x02  },      //      = 2
{0x338C, 0xA410},        //Min_amplitude
{0x3390, 0x01  },      //      = 1
{0xefff,0x0032}, 
 
{0x338C, 0xA122}, //seq.previewParEnter.ae
{0x3390, 0x0004},	  
{0x338C, 0xA130}, //seq.previewParLeave.ae
{0x3390, 0x0004},	  
{0x338C, 0xA137}, //seq.capParEnter.ae
{0x3390, 0x0002}, //manual	 ￡o2  mdr ￡o4
 
{0x338C, 0xA76e}, //gam_cont_B (RW) Contrast Setting
{0x3390, 0x0002}, //contrast= +0,gramma =0.45
{0x338C, 0xA76d}, //gam_cont_a (RW) Contrast Setting
{0x3390, 0x0012}, //contrast= 125%,0gramma =0.45
 
{0x338C, 0xA215}, //ae.IndexTH23
{0x3390, 0x0005}, //default=0x008
 
{0x338C, 0xA216}, //ae.maxgain23
{0x3390, 0x0078}, //default=0x78
              
////[Low Power Preview 15 FPS]
////---------------------------------------------------
//// Minimum 18.6 FPS (52 ms Int-Time)
////---------------------------------------------------
{0x338C, 0xA20C},	// MCU_ADDRESS [AE_MAX_INDEX]
{0x3390, 0x0005},	// MCU_DATA_0
{0x338C, 0xA20e},  //maxVirtGain
{0x3390, 0x0080},
{0x338C, 0xA214},	// MCU_ADDRESS [AE_MAX_DGAIN_AE2]
{0x3390, 0x0080},	// MCU_DATA_0
//0x338C, 0xA10}3,	// MCU_ADDRESS [SEQ_CMD]
//0x3390, 0x000}5,	// MCU_DATA_0
{0x338C, 0xA206},	 // MCU_ADDRESS [AE_TARGET]
{0x3390, 0x0041},   //0x0031,	 // MCU_DATA_0 
/*************}*******linjinchao add************************/ 
{0x338C, 0xA123},	 // MCU_ADDRESS [SEQ_PREVIEW_0_FD]
{0x3390, 0x0002},	 // MCU_DATA_0    filker cancel-manual
{0x338C, 0xA404},	 // MCU_ADDRESS [50/60 manual select and enable]
{0x3390, 0x00c0},  // MCU_DATA_0	 	[select 50hz environment] 
/*************}*******linjinchao add************************/    
{0x338C, 0xA103},	// MCU_ADDRESS [SEQ_CMD]
{0x3390, 0x0005},	// MCU_DATA_0 
{0x338C, 0xA103},	// MCU_ADDRESS [SEQ_CMD]
{0x3390, 0x0006},	// MCU_DATA_0 	 
////enter into qvga Mode
{0x301a,0x0acc}, 
//0xefff,0x00c}8,
{0x3202,0x0008}, 
{0x338c,0x2703}, 
{0x3390,0x0280}, 
{0x338c,0x2705}, 
{0x3390,0x01E0}, 
{0x338c,0x2751}, 
{0x3390,0x0000}, 
{0x338c,0x2753},//800
{0x3390,0x0320}, 
{0x338c,0x2755}, 
{0x3390,0x0000}, 
{0x338c,0x2757}, 
{0x3a54,0x04c4},
{0x338C, 0xA103},	// MCU_ADDRESS [SEQ_CMD]
{0x3390, 0x0001},	// MCU_DATA_0 
{0x0,0x0}, 
}; 

/* 1600x1200 UXGA */
static struct reginfo MT9D112_uxga[] = {
#if 1
					//CAPTURE  1280*960 OK                                             
			{0x301a,0x0acc},                                                       
			{0x341E,0x8F09},		//PLL/ Clk_in control: BYPASS PLL = 36617          
			{0x341C,0x0150}, //0x0250,		//PLL Control 1 = 288                    
			{0xEFFF,0x0005}, 	   // Allow PLL to lock                              
			{0x341E,0x8F0b},		//PLL/ Clk_in control: PLL ON, bypassed = 36617	   
			{0xefff,0x0020},                                                       
			{0x3210,0x01e8},			                                                 
			{0x3202,0x0008},                                                       
			{0xefff,0x01f4},                                                       
			{0x338c,0x2707},                                                       
			{0x3390,0x0640},                                                       
			{0x338c,0x2709},                                                       
			{0x3390,0x04B0},                                                       
			{0x338c,0x275f},                                                       
			{0x3390,0x0000},                                                       
			{0x338c,0x2761},                                                       
			{0x3390,0x0640},                                                       
			{0x338c,0x2763},                                                       
			{0x3390,0x0000},                                                       
			{0x338c,0x2765},                                                       
			{0x3390,0x04b0},                                                       
			{0xefff,0x01f4},                                                       
			{0x338c,0xa120},                                                       
			{0x3390,0x0072},                                                       
			{0xefff,0x01f4},                                                       
			{0x338c,0xa103},                                                       
			{0x3390,0x0002},                                                       
			{0xefff,0x01f4},                                                       
#else
	{0x338C,0x275F},
	{0x3390,0x0000},
	{0x338C,0x2761},
	{0x3390,0x0640},
	{0x338C,0x2763},
	{0x3390,0x0000},
	{0x338C,0x2765},
	{0x3390,0x04B0},
	{0x338C,0x2707},
	{0x3390,0x0640},
	{0x338C,0x2709},
	{0x3390,0x04B0},
	{0x338C,0xA103},
	{0x3390,0x0005},
#endif
	{0x0,0x0}, 
};

/* 128{0x1024 SXGA */
static struct reginfo MT9D112_sxga[] = {
    
  {0x338C,0x275F},
	{0x3390,0x0000},
	{0x338C,0x2761},
	{0x3390,0x0640},
	{0x338C,0x2763},
	{0x3390,0x0000},
	{0x338C,0x2765},
	{0x3390,0x04B0},
	{0x338C,0x2707},
	{0x3390,0x0500},
	{0x338C,0x2709},
	{0x3390,0x0400},
	{0x338C,0xA103},
	{0x3390,0x0005},	
    
    {0x0,0x0}, 
};

/* 80{0x600 SVGA*/
static struct reginfo MT9D112_svga[] = {
    
	{0x338C,0x275F},
	{0x3390,0x0000},
	{0x338C,0x2761},
	{0x3390,0x0640},
	{0x338C,0x2763},
	{0x3390,0x0000},
	{0x338C,0x2765},
	{0x3390,0x04B0},
	{0x338C,0x2707},
	{0x3390,0x0320},
	{0x338C,0x2709},
	{0x3390,0x0258},
	{0x338C,0xA103},
	{0x3390,0x0005},
    
  {0x0,0x0}, 
};

/* 64{0x480 VGA */
static struct reginfo MT9D112_vga[] = {
    
	{0x338C,0x275F},
	{0x3390,0x0000},
	{0x338C,0x2761},
	{0x3390,0x0640},
	{0x338C,0x2763},
	{0x3390,0x0000},
	{0x338C,0x2765},
	{0x3390,0x04B0},
	{0x338C,0x2707},
	{0x3390,0x0280},
	{0x338C,0x2709},
	{0x3390,0x01E0},
	{0x338C,0xA103},
	{0x3390,0x0005},
    
    {0x0,0x0}, 
};

/* 352X288 CIF */
static struct reginfo MT9D112_cif[] = {
 
    {0x0,0x0}, 
};

/* 320*240 QVGA */
static  struct reginfo MT9D112_qvga[] = {
    
	{0x338C,0x275F},
	{0x3390,0x0000},
	{0x338C,0x2761},
	{0x3390,0x0640},
	{0x338C,0x2763},
	{0x3390,0x0000},
	{0x338C,0x2765},
	{0x3390,0x04B0},
	{0x338C,0x2707},
	{0x3390,0x0140},
	{0x338C,0x2709},
	{0x3390,0x00F0},
	{0x338C,0xA103},
	{0x3390,0x0005},
    
    {0x0,0x0}, 
};

/* 16{0x120 QQVGA*/
static struct reginfo MT9D112_qqvga[] = {

	{0x338C,0x275F},
	{0x3390,0x0000},
	{0x338C,0x2761},
	{0x3390,0x0640},
	{0x338C,0x2763},
	{0x3390,0x0000},
	{0x338C,0x2765},
	{0x3390,0x04B0},
	{0x338C,0x2707},
	{0x3390,0x00a0},
	{0x338C,0x2709},
	{0x3390,0x0078},
	{0x338C,0xA103},
	{0x3390,0x0005},
    
    {0x0, 0x0}, 
};
static struct reginfo MT9D112_preview_mod[] = 
{
	0x338C, 0xA120, 	// MCU_ADDRESS [SEQ_CAP_MODE]
	0x3390, 0x0000, 	// MCU_DATA_0
	0x338C, 0xA103, 	// MCU_ADDRESS [SEQ_CMD]
	0x3390, 0x0001, 	// MCU_DATA_0
};

static struct reginfo MT9D112_capture_mod[] = 
{
	0x338C, 0xA120, 	// MCU_ADDRESS [SEQ_CAP_MODE]
	0x3390, 0x0002, 	// MCU_DATA_0
	0x338C, 0xA103, 	// MCU_ADDRESS [SEQ_CMD]
	0x3390, 0x0002, 	// MCU_DATA_0
};

static const struct soc_camera_data_format MT9D112_colour_formats[] = {
     {
	/* Order important: first natively supported,
	 * second supported with a GPIO extender */
		.name		= "MT9D112 YUV420",
		.depth		= 16,
		.fourcc		= V4L2_PIX_FMT_YUV420,
     },

     {
               .name = "MT9D112 YUV422P",
               .depth = 16,
               .fourcc = V4L2_PIX_FMT_YUV422P,
     },
};


struct MT9D112{
	struct i2c_client *client;
	struct soc_camera_device icd;
	int model;	/* V4L2_IDENT_MT9M001* codes from v4l2-chip-ident.h */
	int switch_gpio;
	unsigned char autoexposure;
	unsigned char datawidth;

};

#define MT9D112_IIC_ADDR 	    0x7a  

static unsigned short normal_i2c[] = {MT9D112_IIC_ADDR >> 1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data_MT9D112 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

static int MT9D112_probe(struct i2c_adapter *adapter, int addr, int kind);
static int MT9D112_video_probe(struct soc_camera_device *);
static void MT9D112_video_remove(struct soc_camera_device *);
static int MT9D112_get_control(struct soc_camera_device *, struct v4l2_control *);
static int MT9D112_set_control(struct soc_camera_device *, struct v4l2_control *);


/* MT9D112 register write */
static int MT9D112_write(struct i2c_client *client, u16 reg, u16 val)
{
   int err;
   u8 buf[4];
   struct i2c_msg msg[1];
if(reg != 0xEFFF)
{
	buf[0] = reg >> 8;     
   	buf[1] = reg & 0xFF;
   	buf[2] = val>> 8; 
   	buf[3] = val & 0xFF;
   
   	msg->addr = client->addr;
   	msg->flags = 0;
   	msg->buf = buf;
   	msg->len = sizeof(buf);

   	err = i2c_transfer(client->adapter, msg, 1);
}else{
	mdelay(val);
	return 0;
}

   if (err >= 0)
   {
      return 0;
   }

   return err;
}

/* MT9D112 register read */
static int MT9D112_read(struct i2c_client *client, u16 reg, u16 *val)
{
   int err;
   u8 buf[2];
   struct i2c_msg msg[1];
   
   buf[0] = reg >> 8; 
   buf[1] = reg & 0xFF;
   
   msg->addr = client->addr;
   msg->flags |= I2C_M_RD;
   msg->buf = buf;
   msg->len = sizeof(buf);

   err = i2c_transfer(client->adapter, msg, 2);

   if(err >= 0)
   {
      *val = (buf[0] << 8)|(buf[1] & 0xFF);
      printk("val = 0x%x,buf = 0x%x%x\n",*val,buf[0],buf[1]); 
      return 0;
   } 
   
   return err;
}

/* write a array of registers  */ 
static int MT9D112_write_array(struct i2c_client *client, struct reginfo *regarray)
{
   int err;
   int i = 0;
   
   while(regarray[i++].reg != 0)
   {
      err = MT9D112_write(client, regarray[i].reg, regarray[i].val);
      if (err != 0)
      {
          printk("write failed current i = %d\n", i);
          return err;
      } 
   }
   return 0; 
}

static struct reginfo MT9D112_test_data[] = {
			{0x301A, 0x0acc},
			{0x3006, 0x0345},
			{0x3008, 0x0678},
			{0,0},
};
		
static int MT9D112_init(struct soc_camera_device *icd)
{
    struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);
    u16 val;
    int ret, i;
    u16 pid = 0;
	

	/* soft reset */
/*    for(i=0x3000;i<0x301b;i+=2){
    	ret = MT9D112_read(MT9D112->client, i, &val);    
    	printk("MT9D112_read0 reg[0x%x] value is [0x%x]\n",i,val); 
    }
    ret = MT9D112_write(MT9D112->client, 0x301A, 0x0acc);
    ret = MT9D112_write(MT9D112->client, 0x3006, 0x0345);
    ret = MT9D112_write(MT9D112->client, 0x3008, 0x0678);

    for(i=0x3000;i<0x301b;i+=2){
    	ret = MT9D112_read(MT9D112->client, i, &val);    
    	printk("MT9D112_read1 reg[0x%x] value is [0x%x]\n",i,val); 
    }*/
//    if (ret != 0)
//    {
//       printk("soft reset MT9D112 failed\n");
//       return -ENODEV;
//    }
////    udelay(5*1000);
//     
//    /* check if it is an MT9D112 sensor */ 
//    ret = MT9D112_read(MT9D112->client, 0x3000, &val);    
//    if (ret != 0)
//    {
//       printk("read chip id high byte failed\n");
//       return -ENODEV;
//    }  

    ret = MT9D112_write_array(MT9D112->client, MT9D112_640_480_INIT);
    if (ret != 0)
    {
       printk("error: MT9D112 initial failed\n");
       return ret;
    } 

    for(i=0x3000;i<0x301b;i+=2){
      ret = MT9D112_read(MT9D112->client, i, &val);    
      printk("MT9D112_read reg[0x%x] value is [0x%x]\n",i,val); 
    }

    return 0;
}

static int MT9D112_release(struct soc_camera_device *icd)
{
	return 0;
}

static int MT9D112_start_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int MT9D112_stop_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int bus_switch_request(struct MT9D112 *MT9D112,
			      struct soc_camera_link *icl)
{
    /*
	unsigned int gpio = icl->gpio;

    int ret = gpio_request(gpio, "MT9D112");
    if (ret < 0) {
        dev_err(&MT9D112->client->dev, "Cannot get GPIO %u\n",
            gpio);
        return ret;
    }

    ret = gpio_direction_output(gpio, 0);
    if (ret < 0) {
        dev_err(&MT9D112->client->dev,
            "Cannot set GPIO %u to output\n", gpio);
        gpio_free(gpio);
        return ret;
    }
	MT9D112->switch_gpio = gpio;
    */
#if 0
    rockchip_mux_api_set(GPIOH6_IQ_SEL_NAME, IOMUXB_GPIO1_D6);
    gpio_direction_output(GPIOPortH_Pin6, GPIO_OUT);
    __gpio_set(GPIOPortH_Pin6, GPIO_LOW);
    
#endif    
	rockchip_mux_api_set(SENSOR_PWDN_IOMUX_PINNAME, SENSOR_PWDN_IOMUX_PINDIR);		/*xxm*/
	gpio_direction_output(SENSOR_PWDN_IOPIN, GPIO_OUT);
	__gpio_set(SENSOR_PWDN_IOPIN, GPIO_LOW);				/*xxm*/

//  pca955x_gpio_direction_output(PCA955X_Pin16, GPIO_HIGH);
//  pca955x_gpio_direction_output(PCA955X_Pin17, GPIO_HIGH);
//
//  msleep(100);
  
  //pca955x_gpio_direction_output(PCA955X_Pin16, GPIO_LOW);
  //pca955x_gpio_direction_output(PCA955X_Pin17, GPIO_LOW);
  
	return 0;
}

static void bus_switch_release(struct MT9D112 *MT9D112)
{

}

static int MT9D112_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
       //struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);
	

	return 0;
}

static unsigned long MT9D112_query_bus_param(struct soc_camera_device *icd)
{
	/* 0v9650 has all capture_format parameters fixed */
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_HIGH|
		SOCAM_SENSOR_UYVY;
}

static int MT9D112_set_fmt_cap(struct soc_camera_device *icd,
		__u32 pixfmt, struct v4l2_rect *rect)
{
    struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);
    int ret;
    struct reginfo *MT9D112_win;
	uint16 val;
#if 1    
    if ((rect->width <= 320) && (rect->height <= 240)) {
       MT9D112_win = MT9D112_qvga;
    } else if ((rect->width <= 352) && (rect->height <= 288)){
        MT9D112_win = MT9D112_cif;
    } else if ((rect->width <= 640) && (rect->height <= 480)){
#else
    if ((rect->width <= 640) && (rect->height <= 480)){
#endif    
    printk("rect width = %d, rect height = %d\n", rect->width, rect->height);
        MT9D112_win = MT9D112_640_480_INIT;
    } else if((rect->width <= 800) && (rect->height <= 600)){
       MT9D112_win = MT9D112_svga;
    } else if((rect->width <= 1280) && (rect->height <= 1024)) {
        MT9D112_win = MT9D112_sxga;
    }else {
    	 MT9D112_win = MT9D112_uxga; 
    	//MT9D112_write_array(MT9D112->client, MT9D112_capture_mod); 
    	//mdelay(50);
    }
    
    ret = MT9D112_write_array(MT9D112->client, MT9D112_win); 
    if (ret != 0)
    {
       printk("MT9D112 set format capability failed\n");
       return ret;
    }
    MT9D112_write(MT9D112->client, 0x338c, 0x2703);
    MT9D112_read(MT9D112->client, 0x3390, &val);    
    printk("MT9D112_read1 reg[0x2703] value is [0x%x]\n",val); 
	
    MT9D112_write(MT9D112->client, 0x338c, 0xa103);
    MT9D112_read(MT9D112->client, 0x3390, &val);    
    printk("MT9D112_read1 reg[0xa103] value is [0x%x]\n",val); 

    mdelay(50);
    printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__,rect->width,rect->height);

    return 0;
}

static int MT9D112_try_fmt_cap(struct soc_camera_device *icd,
			       struct v4l2_format *f)
{
	if (f->fmt.pix.height < 32 + icd->y_skip_top)
		f->fmt.pix.height = 32 + icd->y_skip_top;
	if (f->fmt.pix.height > 1200 + icd->y_skip_top)
		f->fmt.pix.height = 1200 + icd->y_skip_top;
	if (f->fmt.pix.width < 48)
		f->fmt.pix.width = 48;
	if (f->fmt.pix.width > 1600)
		f->fmt.pix.width = 1600;
	f->fmt.pix.width &= ~0x01; /* has to be even, unsure why was ~3 */
    printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__,f->fmt.pix.width,f->fmt.pix.height);
	return 0;
}

static int MT9D112_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_chip_ident *id)
{
	struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);

	if (id->match_type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match_chip != MT9D112->client->addr)
		return -ENODEV;

	id->ident	= MT9D112->model;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int MT9D112_get_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != MT9D112->client->addr)
		return -ENODEV;

    char reg = reg->reg;
    int ret = MT9D112_rx_data(MT9D112->client, &reg, 1);
    if (!ret)
        reg->val = reg;

	return ret;
}

static int MT9D112_set_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != MT9D112->client->addr)
		return -ENODEV;

    char reg[2];
    reg[0] = reg->reg;
    reg[1] = reg->val;
    int ret = MT9D112_tx_data(MT9D112->client, reg, 2);

	return ret;
}
#endif

static const struct v4l2_queryctrl MT9D112_controls[] = {
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

static int MT9D112_get_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	//struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);
	//int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

static struct soc_camera_ops MT9D112_ops = {
	.owner			= THIS_MODULE,
	.probe			= MT9D112_video_probe,
	.remove			= MT9D112_video_remove,
	.init			= MT9D112_init,
	.release		= MT9D112_release,
	.start_capture		= MT9D112_start_capture,
	.stop_capture		= MT9D112_stop_capture,
	.set_fmt_cap		= MT9D112_set_fmt_cap,
	.try_fmt_cap		= MT9D112_try_fmt_cap,
	.set_bus_param		= MT9D112_set_bus_param,
	.query_bus_param	= MT9D112_query_bus_param,
	.controls		= MT9D112_controls,
	.num_controls		= ARRAY_SIZE(MT9D112_controls),
	.get_control		= MT9D112_get_control,
	.set_control		= MT9D112_set_control,
	.get_chip_id		= MT9D112_get_chip_id,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= MT9D112_get_register,
	.set_register		= MT9D112_set_register,
#endif
};

static int MT9D112_set_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	//struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);
	const struct v4l2_queryctrl *qctrl;
	//int data;

	qctrl = soc_camera_find_qctrl(&MT9D112_ops, ctrl->id);

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
static int MT9D112_video_probe(struct soc_camera_device *icd)
{
	struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);
	int ret;

	/* We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant. */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

        MT9D112->model = V4L2_IDENT_MT9D112;
        icd->formats = MT9D112_colour_formats;
        icd->num_formats = ARRAY_SIZE(MT9D112_colour_formats);

	/* Now that we know the model, we can start video */
	ret = soc_camera_video_start(icd);
	if (ret)
		goto eisis;

	return 0;

eisis:

	return ret;
}

static void MT9D112_video_remove(struct soc_camera_device *icd)
{
	struct MT9D112 *MT9D112 = container_of(icd, struct MT9D112, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", MT9D112->client->addr,
		MT9D112->icd.dev.parent, MT9D112->icd.vdev);
	soc_camera_video_stop(&MT9D112->icd);
}

static int MT9D112_remove(struct i2c_client *client)
{
	struct MT9D112 *MT9D112 = i2c_get_clientdata(client);

	soc_camera_device_unregister(&MT9D112->icd);
	kfree(MT9D112);

	return 0;
}

static int MT9D112_detach_client(struct i2c_client *client)
{
	MT9D112_remove(client);

	return i2c_detach_client(client);
}

static int MT9D112_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data_MT9D112, MT9D112_probe);
}

static struct i2c_driver MT9D112_driver = {
	.driver = {
		.name = "MT9D112",
	    },
	.id 	= MT9D112_IIC_ADDR,
	.attach_adapter = &MT9D112_attach_adapter,
	.detach_client  = &MT9D112_detach_client,
};

static struct soc_camera_link iclink = {//nzy add
    .bus_id = 33, /* Must match with the camera ID above */
    .gpio   = 1,
};
    
static int MT9D112_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct MT9D112 *MT9D112;
	struct soc_camera_device *icd;
	struct soc_camera_link *icl;
	struct i2c_client *client = NULL;
	int ret;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
	    ret = -EIO;
            goto exit_check_functionality_failed;
	}

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client) {
	    ret = -ENOMEM;
            goto exit_alloc_client_failed;
	}

	strlcpy(client->name, "MT9D112", I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &MT9D112_driver;
	client->addressBit = I2C_7BIT_ADDRESS_16BIT_REG;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 200;	
	ret = i2c_attach_client(client);
	if (ret) {
            goto exit_i2c_attach_client_failed;
	}
        printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	
	MT9D112 = kzalloc(sizeof(struct MT9D112), GFP_KERNEL);
	if (!MT9D112) {
       	    ret = -ENOMEM;
            goto exit_alloc_data_failed;
	}

	MT9D112->client = client;
	i2c_set_clientdata(client, MT9D112);
        //icl = &iclink;//client->dev.platform_data;

	/* Second stage probe - when a capture adapter is there */
	icd = &MT9D112->icd;
	icd->ops	= &MT9D112_ops;
	icd->control	= &client->dev;
	icd->x_min	= 0;
	icd->y_min	= 0;
	icd->x_current	= 0;
	icd->y_current	= 0;
	icd->width_min	= 48;
	icd->width_max	= 1600;
	icd->height_min	= 32;
	icd->height_max	= 1200;
	icd->y_skip_top	= 0;
	icd->iface	= 33;//icl->bus_id;
	/* Default datawidth - this is the only width this camera (normally)
	 * supports. It is only with extra logic that it can support
	 * other widths. Therefore it seems to be a sensible default. */
	MT9D112->datawidth = 8;
	/* Simulated autoexposure. If enabled, we calculate shutter width
	 * ourselves in the driver based on vertical blanking and frame width */
	MT9D112->autoexposure = 1;

	ret = bus_switch_request(MT9D112, icl);
	if (ret)
		goto exit_bus_switch_request_failed;
		
	ret = soc_camera_device_register(icd);
	if (ret)
        goto exit_soc_camera_device_register_failed;
                
        printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	return 0;

exit_soc_camera_device_register_failed:
    bus_switch_release(MT9D112);
exit_bus_switch_request_failed:
	kfree(MT9D112);
exit_alloc_data_failed:
   i2c_detach_client(client);
exit_i2c_attach_client_failed:
	kfree(client);
exit_alloc_client_failed:
exit_check_functionality_failed:
	return ret;
}

static int __init MT9D112_mod_init(void)
{
	return i2c_add_driver(&MT9D112_driver);
}

static void __exit MT9D112_mod_exit(void)
{
	i2c_del_driver(&MT9D112_driver);
}
late_initcall(MT9D112_mod_init);

//module_init(MT9D112_mod_init);
module_exit(MT9D112_mod_exit);

MODULE_DESCRIPTION("MT9D112 Camera sensor driver");
MODULE_AUTHOR("lbt <kernel@rock-chips>");
MODULE_LICENSE("GPL");

