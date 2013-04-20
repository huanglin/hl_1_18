/* arch/arm/mach-goldfish/board-ROCK28DEMO.c
**
** Copyright (C) 2009 ROCKCHIP, Inc.
**  
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/arch/hw_common.h>
#include <asm/arch/hardware.h>
#include <asm/arch/api_i2c.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/intc.h>
#include <asm/arch/rk28_serial.h>
#include <asm/arch/rk28_fb.h>
#include <asm/arch/rk28_spim.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_i2c.h>
#if defined (CONFIG_RK1000_CONTROL)
#include "../drivers/rk1000/rk1000.h"
#endif
#if defined (CONFIG_SND_SOC_WM8988)
#include "../sound/soc/codecs/wm8988.h"
#endif
#include "../drivers/rtc/rtc-HYM8563.h"
#include <asm/arch/rk28_scu.h>
#include <asm/arch/gpio.h>
/********************************************************
 *	DEBUG
 * ******************************************************/
//#define	RK28_PRINT
#include <asm/arch/rk28_debug.h>
//#include <asm/arch/rk28_ddr.h>

extern 	void __init rk28_irq_init(unsigned int priority[NR_RK28_IRQS]);
extern unsigned int int_priority[NR_RK28_IRQS];
extern struct sys_timer rockchip_timer;
extern   void  rk28_add_device_touchscreen(void);
extern void __init rk28_add_device_mmc(void);
extern 	void rock28_add_device_key(void);
extern 	void __init rk28_add_device_i2c(struct i2c_board_info *devices, int nr_devices);
extern void rk28_adddevice_nand(void);
extern void rk28_add_device_i2s(void);
extern void __init rk28_add_device_camera(void);//nzy add
extern void rk28_add_device_battery(void);
extern void rk28_add_device_backlight(void);
extern void rockchip_add_device_dsp(void);
extern void rk28_add_usb_devices(void);
extern void rk28_add_device_rk1000_control(void);
extern int uart_init(char uartCh, unsigned int baudRate);
extern void __rockchip_scu_init_hw( void );
extern void __init rockchip_timer_clock_source_init( int apb_clk );
extern void __init rk28_kld_init( void );
extern int rk28_restart( int type ) ;
extern void rockchip_add_device_pmem(void);

//IO映射方式描述 ，每个为一段线性连续映射
static struct map_desc rockchip_io_desc[] __initdata = {

	{
		.virtual	= AHB_BASEADD_VA,					//虚拟地址
		.pfn		= __phys_to_pfn(AHB_BASEADD_PA),    //物理地址，须与页表对齐
		.length 	= AHB_SIZE,							//长度
		.type		= MT_DEVICE							//映射方式
	},
	
	{
			.virtual	= APB_BASEADD_VA,
			.pfn		= __phys_to_pfn(APB_BASEADD_PA),
			.length 	= APB_SIZE,
			.type		= MT_DEVICE
	},

                {
			.virtual	= DSP_BASEADD_VA,
			.pfn		= __phys_to_pfn(DSP_BASE_ADDR),
			.length 	= 0x00200000, //APB_SIZE,
			.type		= MT_DEVICE
	},
	#if 0
                {
			.virtual	= 0xff400000,           /* for itcm , vir = phy , for reboot */
			.pfn		= __phys_to_pfn(0xff400000),
			.length 	= APB_SIZE,
			.type		= MT_DEVICE
	}
		#endif
};

 /******************************************************************************************
 * Serial port configuration.
 *    0 .. 1 = USART0 ,USART1
 ******************************************************************************************/
static struct rock_uart_config __initdata rockchip_uart_config = {  
#if defined  (CONFIG_RK2818_A7)
	.console_tty	= 0,  //1,  //0,				//ttyS0
	.nr_tty		= 1,    // only init uart0.
#elif defined  (CONFIG_RK2818_A7_V1_1)
	.console_tty    = 1,  //1,  //0,                                //ttyS1
        .nr_tty         = 2,    // only init uart1.
#else
                .console_tty	= 1,
                .nr_tty		= 2,    // init uart0 and uart1.
#endif
	.tty_map	= { 0, 1 }		/// ttyS0,  ttyS1 
}; 



/*****************************************************************************************
 * SPI devices
 *author: lhh
 *****************************************************************************************/
static struct spi_board_info board_spi_devices[] = {
#if 0
	{	/* net chip */
		.modalias	= "enc28j60",
		.chip_select	= 1,
		.max_speed_hz	= 12 * 1000 * 1000,
		.bus_num	= 0,
	},
#endif
#if defined(CONFIG_WAVEFORM_FROM_EEPROM) || defined(CONFIG_WAVEFORM_FROM_CODE_OR_EEPROM )
    {   /*spi flash add by zyw 2010-07-08*/
        .modalias   = "epd_spi_flash",
        .chip_select    = 0,
        /* yuzhe@rock-chips.com added below */
        .max_speed_hz   = CONFIG_SPI_FLASH_SPEED_MHZ*1000000,
        .bus_num    = 0,        
    },
#endif
  	{	/*spi touscreen (xpt2046) auth wqq 2009-05-16*/
		.modalias	= "xpt2046_ts",
		.chip_select	= 1,
		.max_speed_hz	= 120000,
		.bus_num	= 0,
		.irq = IRQ_GPIO1,
	},


};

/**************************************************************************
* I2C devices
*author :wqq
*date : 2009-5-11
**************************************************************************/

static struct i2c_board_info __initdata board_i2c_devices[] = {
#if defined (CONFIG_RTC_HYM8563)
{
	.driver_name	= "rtc_HYM8563",
	.type			= "RTC",
	.addr			= 0xA2,

},
#endif

#if defined (CONFIG_TOUCHSCREEN_ELAN)
{
	.driver_name	= "elan_touch",
	.type			= "TS",
	.addr			= 0x20,

},
#endif

#if defined (CONFIG_RK28_I2C4310_TS)
{
	.driver_name	= "ra4310name",
	.type			= "TS",
	.addr			= 0x38,

},
#endif

#if defined (CONFIG_RK28_I2C_TS_GTT8205S)
{
	.driver_name  = "gtt8205s",
	.type	=   "TS",
	.addr 	=  0x15,
},
#endif

#if defined (CONFIG_GS_MMA7660)
{
	.driver_name	= "gs_mma7660",
	.type			= "GSENSOR",
	.addr			= 0x98,
},
#endif
#if defined (CONFIG_RK1000_CONTROL)
    {
        .driver_name    = "RK1000_CONTROL",
        .type           = "I2C_CONTROL",
        .addr           = 0x80,
    },
#endif
#if defined (CONFIG_RK1000_TVOUT)
    {
        .driver_name    = "RK1000_TVOUT",
        .type           = "TVOUT",
        .addr           = 0x84,
    },
#endif    
#if defined (CONFIG_SND_SOC_RK1000)
    {
        .driver_name    = "RK1000 I2C Codec",
        .type           = "CODEC",
        .addr           = 0xc0,
    },
#endif
#if defined (CONFIG_RTC_PT7C4337)
	{
		.driver_name	= "rtc_PT7C4337",
		.type			= "RTC",
		.addr			= 0xD0,
	
	},
#endif
#if defined (CONFIG_SOC_CAMERA_OV9650)
{
	.driver_name	= "ov9650",
	.type			= "CSENSOR",
	.addr			= 0xa0,
},
#endif

#if defined(CONFIG_SOC_CAMERA_OV2655_SENSOR)  
{
        .driver_name = "ov2655",
        .type = "CSENSOR",
        .addr = 0xa0,
},
#endif
#if defined(CONFIG_KEYBOARD_SN7326)
{
        .driver_name = "sn7326",
        .type = "keypad",
        .addr = 0xb0,         
},
#endif
 

};

#ifdef CONFIG_RK28_I2C_TS_GTT8205S
static void rk28_gtt8205_init( void )
{
	rockchip_mux_api_set((char *)TOUCH_INT_IOMUX_PINNAME,(unsigned int)TOUCH_INT_IOMUX_PINDIR);
	GPIOSetPinDirection(TOUCH_INT_IOPIN, GPIO_IN);
	GPIOSetPinLevel(TOUCH_INT_IOPIN,GPIO_LOW);
	mdelay(10);
	GPIOSetPinLevel(TOUCH_INT_IOPIN,GPIO_HIGH);
}
#endif


static void rk28_codec_mask_noise(void)
{
	rockchip_mux_api_set((char *)RK1000_RST_IOMUX_PINNAME,(unsigned int)RK1000_RST_IOMUX_PINDIR);
	GPIOSetPinDirection(RK1000_RST_IOPIN,GPIO_OUT);
	GPIOSetPinLevel(RK1000_RST_IOPIN,GPIO_LOW);

	rockchip_mux_api_set((char *)SPK_CTL_IOMUX_PINNAME,(unsigned int)SPK_CTL_IOMUX_PINDIR);
	GPIOSetPinDirection(SPK_CTL_IOPIN,GPIO_OUT);
	GPIOSetPinLevel(SPK_CTL_IOPIN,GPIO_LOW);
	mdelay(5);
}


static void rk28_power_on(void)
{

	rockchip_mux_api_set((char *)DDR_CTL_IOMUX_PINNAME,(unsigned int)DDR_CTL_IOMUX_PINDIR);
	GPIOSetPinLevel(DDR_CTL_IOPIN,GPIO_LOW);
	GPIOSetPinDirection(DDR_CTL_IOPIN,GPIO_OUT);
	GPIOPullUpDown(DDR_CTL_IOPIN,GPIONormal);
	GPIOSetPinLevel(DDR_CTL_IOPIN,GPIO_LOW);

	rockchip_mux_api_set((char *)HOST_ON_IOMUX_PINNAME,(unsigned int)HOST_ON_IOMUX_PINDIR);
	GPIOSetPinLevel(HOST_ON_IOPIN,GPIO_LOW);
	GPIOSetPinDirection(HOST_ON_IOPIN,GPIO_OUT);
	GPIOPullUpDown(HOST_ON_IOPIN,GPIONormal);
	GPIOSetPinLevel(HOST_ON_IOPIN,GPIO_LOW);

	rockchip_mux_api_set((char *)SLEEP_CTL_IOMUX_PINNAME,(unsigned int)SLEEP_CTL_IOMUX_PINDIR);
	GPIOSetPinLevel(SLEEP_CTL_IOPIN,GPIO_HIGH);
	GPIOSetPinDirection(SLEEP_CTL_IOPIN,GPIO_OUT);
	GPIOPullUpDown(SLEEP_CTL_IOPIN,GPIONormal);
	GPIOSetPinLevel(SLEEP_CTL_IOPIN,GPIO_HIGH);

	rockchip_mux_api_set((char *)PWR_ON_IOMUX_PINNAME,(unsigned int)PWR_ON_IOMUX_PINDIR);
	GPIOSetPinLevel(PWR_ON_IOPIN,GPIO_HIGH);
	GPIOSetPinDirection(PWR_ON_IOPIN,GPIO_OUT);
	GPIOPullUpDown(PWR_ON_IOPIN,GPIOPullUp);
	GPIOSetPinLevel(PWR_ON_IOPIN,GPIO_HIGH);

#if defined (CONFIG_SND_SOC_ES8388)
	/*disable audio output */
	rockchip_mux_api_set((char *)GPIOE_SPI1_FLASH_SEL2_NAME, (unsigned int)IOMUXA_GPIO1_A3B7);
	GPIOSetPinLevel(GPIOPortF_Pin7,GPIO_LOW);
	GPIOSetPinDirection(GPIOPortF_Pin7, GPIO_OUT);
	GPIOSetPinLevel(GPIOPortF_Pin7,GPIO_LOW);
#endif
}

void rk28_power_off(void)
{
	local_irq_enable();     /* open for some logs */
	rockchip_mux_api_set((char *)PLAY_ON_IOMUX_PINNAME,(unsigned int)PLAY_ON_IOMUX_PINDIR);
	GPIOPullUpDown(PLAY_ON_IOPIN,GPIOPullDown);
	
	rockchip_mux_api_set((char *)PWR_ON_IOMUX_PINNAME,PWR_ON_IOMUX_PINDIR);
	GPIOSetPinDirection(PWR_ON_IOPIN,GPIO_OUT);
	GPIOSetPinLevel(PWR_ON_IOPIN,GPIO_LOW);
	while( 1 ) {
		rockchip_mux_api_set((char *)PWR_ON_IOMUX_PINNAME,(unsigned int)PWR_ON_IOMUX_PINDIR);
		GPIOSetPinDirection(PWR_ON_IOPIN,GPIO_OUT);
		if(GPIOGetPinLevel(PWR_ON_IOPIN) == GPIO_HIGH)
			GPIOSetPinLevel(PWR_ON_IOPIN,GPIO_LOW);
		if(GPIOGetPinLevel(PLAY_ON_IOPIN) == GPIO_HIGH)
			break;
	}
	rk28_restart( 0 );
}


/* 初始化IO映射表*/
static void __init machine_rk28_mapio(void)
{
	iotable_init(rockchip_io_desc, ARRAY_SIZE(rockchip_io_desc));
	//uart_init(0 , 115200);
	__rockchip_scu_init_hw();
	rk28_kld_init();
	rockchip_timer_clock_source_init( rockchip_clk_get_apb() );
	rockchip_iomux_init();
	/* Setup the serial ports and console*/
	rockchip_init_serial(&rockchip_uart_config);
#ifdef CONFIG_RK28_I2C_TS_GTT8205S
	/*init touchscreen*/
	rk28_gtt8205_init( );
#endif
}

/*初始化IRQ*/
void machine_rk28_init_irq(void)
{	
	rk28_irq_init(int_priority);
}




//初始化各设备
void machine_rk28_board_init(void)
{
	/*Power on*/
	rk28_power_on( );
	pm_power_off = rk28_power_off;
	
/* Serial*/
 
	rockchip_add_device_serial();
	
/*LCD*/
 
 	rk28_add_device_lcdc();    
#ifndef CONFIG_LCD_RK_EINK
	/*camera*/
    rk28_add_device_camera();//nzy add
#endif
/*SDIO*/
	
	rk28_add_device_mmc();
	
/*KEY*/
	
	rock28_add_device_key();
	
 /* SPI */
	
	rockchip_add_device_spi_master(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	
/*NAND*/
	
	rk28_adddevice_nand();
	
/*I2C*/
	
	rk28_add_device_i2c(board_i2c_devices,ARRAY_SIZE(board_i2c_devices));
	
//rockchip_add_device_i2c1(board_i2c1_devices,ARRAY_SIZE(board_i2c1_devices));
	
/*I2S*/
	
	rk28_add_device_i2s();

	
	rk28_add_device_battery();
	
	rk28_add_usb_devices();

	
	rk28_add_device_backlight();
	
	rockchip_add_device_dsp();
#ifndef CONFIG_LCD_RK_EINK
#if defined(CONFIG_ANDROID_PMEM)
	rockchip_add_device_pmem();
#endif	
#endif
}

extern const char linux_banner[];
/* 20100204,HSL@RK, unify format for pc tools analyse:
    "xxxx version x.x.x#addition (product_name) #addition infomation.\n"
*/
const char rockchip_version[] = 
    // "rockchip version v0.02 (Archos) #release first version\n"
//	"rockchip 281x version release v0.03(Archos) # update flash driver"
//	"rockchip 281x version release v0.04(Archos) # add cpufreq"
//	 "rockchip 281x version release v0.05(Archos) # update flash && cpufreq && scu"
//"rockchip 281x version release v0.09(Archos) # alarm && touch wakeup event && sdk code"		/*xxm*/
//"rockchip 281x version release v0.11(Archos)no change dsp frequency # "
//	"rockchip 281x version release v0.12(Archos) switch to base develop version&&release production "
//	"rockchip 281x version release v1.02(Archos)Pmem 32M UI pmem32M alarm bug touch bug "
//"rockchip 281x version release v1.03(Archos)Pmem 26M UI pmem22M &backlight driver&& ddr speed "
//"rockchip 281x version release v1.04(Archos) battery online status"
//"rockchip 281x version release v1.05(Archos) flash driver & usb wakeup & touch default data"
//"rockchip 281x version release v1.06(Archos) flash driver & suspend mode wakeup & sound cut down"
//"rockchip 281x version release v1.08(Archos) update wifi driver && set gpio  reload && scu update"
	"rockchip 281x version release v1.09(Eink) used in Eink, without camera video and touch mabe"
;        

/* 20100202,HSL@RK,for pc tools ,NOT CHANGE!. */
const char system_type[] = 
        "android linux (rockchip).\n"
;
// don`t change the name "MACHINE_ROCK",  -lingzj
MACHINE_START(ROCKCHIP, "RK28board")	
	.phys_io	= 0x18002000,
	
	.io_pg_offst	= (0xFF102000 >> 18) & 0xfffc,
	
	.boot_params	= ROCK_SDRAM_BASE + 0x100,
	
	//以上地址有待确认
	
	.timer		= &rockchip_timer,//此处注册一个系统timer,
	
	.map_io 	= machine_rk28_mapio,
	
	.init_irq	= machine_rk28_init_irq,
	
	.init_machine	= machine_rk28_board_init,
MACHINE_END

