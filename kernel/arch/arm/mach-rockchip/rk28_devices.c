/*
 * arch/arm/mach-rockchip/rockchip_devices.c
 *
 *  Copyright (C) 2005 Thibaut VARENE <varenet@parisc-linux.org>
 *  Copyright (C) 2005 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */  
#include <linux/kernel.h> 
#include <asm/mach/arch.h>  
#include <asm/mach/map.h>

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <asm/arch/board.h>
#include <asm/arch/hardware.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/usb/android.h>
#include <linux/android_pmem.h>

#include <asm/io.h>
#include <asm/arch/hw_common.h>
#include <asm/arch/iomux.h>
#include <asm/arch/api_intc.h>
#include <linux/spi/spi.h>
#include <asm/arch/rk28_serial.h>
#include <asm/arch/rk28_i2c.h>
#include <asm/arch/rk28_fb.h>

#include <asm/arch/rk28_sdmmc.h>
#include <asm/arch/rk28_debug.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_backlight.h>

#include <asm/arch/rk28_debug.h>
#include <asm/arch/rk2818_fb.h>

#define USB_OTG_SIZE 0x00040000

/* --------------------------------------------------------------------
 *  lcdc
 *Author :nzy
 * -------------------------------------------------------------------- */


/*rk2818_fb gpio information*/
static struct rk2818_fb_gpio rk2818_fb_gpio_info = {
#ifdef CONFIG_LCD_RK_EINK
    .lcd_cs     = 0,
    .display_on = 0,
    .lcd_standby = 0,
#else
    .lcd_cs     = 0/*(GPIO_LOW<<16)|GPIOPortB_Pin3*/,
    .display_on = (GPIO_LOW<<16)|LCD_DISP_ON_IOPIN,				/*xxm*/
    .lcd_standby = 0,
    .mcu_fmk_pin = 0,
#endif
};

/*rk2818_fb iomux information*/
static struct rk2818_fb_iomux rk2818_fb_iomux_info = {
    .data16     = GPIOD_LCDC16BIT_SEL_NAME,
    .data18     = GPIOC_LCDC18BIT_SEL_NAME,
    .data24     = GPIOC_LCDC24BIT_SEL_NAME,
    .den        = CXGPIO_LCDDEN_SEL_NAME,
    .vsync      = CXGPIO_LCDVSYNC_SEL_NAME,
    .mcu_fmk    = 0,
};
/*rk2818_fb*/
struct rk2818_fb_mach_info rk2818_fb_mach_info = {
    .gpio = &rk2818_fb_gpio_info,
    .iomux = &rk2818_fb_iomux_info,
};

#define RK2818_LCDC_PHYS             	0x100A4000
#define RK2818_LCDC_SIZE             	SZ_8K

/* rk2818 fb resource */
static struct resource rk2818_fb_resource[] = {
	[0] = {
		.start = RK2818_LCDC_PHYS,
		.end   = RK2818_LCDC_PHYS + RK2818_LCDC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	#if !defined(CONFIG_LCD_RK_EINK)
	[1] = {
		.start = IRQ_NR_LCDC,
		.end   = IRQ_NR_LCDC,
		.flags = IORESOURCE_IRQ,
	},
	#endif
};

/*platform_device*/

struct platform_device rk2818_device_fb = {
	.name		  = "rk2818-fb",
	.id		  = 4,
	.num_resources	  = ARRAY_SIZE(rk2818_fb_resource),
	.resource	  = rk2818_fb_resource,
	.dev            = {
		.platform_data  = &rk2818_fb_mach_info,
	}
};

void __init rk28_add_device_lcdc(void)
{
	
    	platform_device_register(&rk2818_device_fb);
}

/***************************************************************************/
static struct resource rk28_camera_resource[] = {//nzy add
	[0] = {
		.start = VIP_BASE_ADDR,
		.end   = SDMMC1_BASE_ADDR - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VIP,
		.end   = IRQ_VIP,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 rockchip_device_camera_dmamask = 0xffffffffUL;

/*platform_device*/
struct platform_device rk28_device_camera = {//nzy add
	.name		  = "rk28-camera",
	.id		  = 33,
	.num_resources	  = ARRAY_SIZE(rk28_camera_resource),
	.resource	  = rk28_camera_resource,
	.dev            = {
		.dma_mask = &rockchip_device_camera_dmamask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data  = NULL,
	}
};


void __init rk28_add_device_camera(void)//nzy add
{
	
    	///printk("---->%s..%s..:%i\n",__FILE__,__FUNCTION__,__LINE__);
    	platform_device_register(&rk28_device_camera);
}

/***************************************************************************/



// 20101230,HSL@RK,for change serial by machine serial at kld.init.
/*static*/
char android_serial_number[64] = "1122334455";
static struct android_usb_platform_data android_usb_pdata = {
	//below is starndard android usb id
        /*.vendor_id      = 0x0bb4,
        .product_id     = 0x0c01,*/
        
	//below is normal usb mass storage id
        .vendor_id      = 0x05e3,
        .product_id     = 0x0726,

        .adb_product_id = 0x0c02,
        .version        = 0x0100,
        .product_name   = "Android Phone",
        .manufacturer_name = "HTC",
        .nluns = 2,
        .serial_number = android_serial_number,
};

static struct platform_device android_usb_device = {
        .name   = "android_usb",
        .id             = -1,
        .dev            = {
                .platform_data = &android_usb_pdata,
        },
};
//#endif



#ifdef CONFIG_RK2818_HOST11
static struct resource rk2818_host11_resource[] = {
	{
		.start = IRQ_NR_USBHOST,
		.end   = IRQ_NR_USBHOST,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = USB_HOST11_BASE_ADDR,
		.end   = USB_HOST11_BASE_ADDR + USB_OTG_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},

};

struct platform_device rk2818_device_host11 = {
	.name		  = "rk2818_host11",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(rk2818_host11_resource),
	.resource	  = rk2818_host11_resource,
};
#endif

/*DWC_OTG*/
static struct resource dwc_otg_resource[] = {
	[0] = {
		.start = IRQ_OTG,
		.end   = IRQ_OTG,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start = USB_OTG_BASE_ADDR,
		.end   = USB_OTG_BASE_ADDR + USB_OTG_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},

};

struct platform_device rk2818_device_dwc_otg = {
	.name		  = "dwc_otg",
	.id		  = -1,
	#if 0
	.num_resources	  = ARRAY_SIZE(rk2818_host11_resource),
	.resource	  = rk2818_host11_resource,
	#else
	.num_resources	  = ARRAY_SIZE(dwc_otg_resource),
	.resource	  = dwc_otg_resource,
	#endif
};

/* --------------------------------------------------------------------
 *  UART
 *Author :lhh
 * -------------------------------------------------------------------- */
#if defined(CONFIG_ROCK_UART)

int uartportregs		= UART0_BASE_ADDR_VA;
int uartportregs1		= UART1_BASE_ADDR_VA;
static struct resource uart0_resources[] = {
	[0] = {
		.start	= UART0_BASE_ADDR,
		.end	= UART0_BASE_ADDR + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_UART0,
		.end	= IRQ_UART0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rockchip_uart0_device = {
	.name		= "rockchip_usart",  //"rockchip_uart0",
	.id		= 0,
	.resource	= uart0_resources,
	.num_resources	= ARRAY_SIZE(uart0_resources),
};

static struct resource uart1_resources[] = {
	[0] = {
		.start	= UART1_BASE_ADDR,
		.end	= UART1_BASE_ADDR + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_UART1,
		.end	= IRQ_UART1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rockchip_uart1_device = {
	.name		= "rockchip_usart",  //"rockchip_uart0",
	.id		= 1,
	.resource	= uart1_resources,
	.num_resources	= ARRAY_SIZE(uart1_resources),
};

static struct platform_device *rockchip_uarts[NR_PORTS];	/* the UARTs to use */
struct platform_device *rockchip_default_console_device;	/* the serial console device */


void __init __deprecated rockchip_init_serial(struct rock_uart_config *config)
{
	int i;

	/* Fill in list of supported UARTs */
	for (i = 0; i <  config->nr_tty; i++) {
		switch (config->tty_map[i]) {
	    case 0:
				rockchip_mux_api_set(GPIOG1_UART0_MMC1WPT_NAME, IOMUXA_UART0_SOUT);
				rockchip_mux_api_set(GPIOG0_UART0_MMC1DET_NAME, IOMUXA_UART0_SIN);
				rockchip_uarts[i] = &rockchip_uart0_device;
				break;
			case 1:
				rockchip_mux_api_set(GPIOF1_UART1_CPWM1_NAME, IOMUXA_UART1_SOUT);
				rockchip_mux_api_set(GPIOF0_UART1_CPWM0_NAME, IOMUXA_UART1_SIN); 
				rockchip_uarts[i] = &rockchip_uart1_device;
				break;
			default:
				continue;
		}
		rockchip_uarts[i]->id = i;		/* update ID number to mapped ID */
	}
	/* Set serial console device */
	if (config->console_tty<NR_PORTS)
          rockchip_default_console_device = rockchip_uarts[config->console_tty];
	if (!rockchip_default_console_device)
		printk(KERN_INFO "ROCKCHIP: No default serial console defined.\n");
	
}

void __init rockchip_register_uart(unsigned id, unsigned portnr, unsigned pins)
{
	struct platform_device *pdev;


	switch (id) {
		case IRQ_UART0:
			pdev = &rockchip_uart0_device;
			rockchip_mux_api_set(GPIOG1_UART0_MMC1WPT_NAME, IOMUXA_UART0_SOUT);
		  rockchip_mux_api_set(GPIOG0_UART0_MMC1DET_NAME, IOMUXA_UART0_SIN);
			break;
		case IRQ_UART1:
			pdev = &rockchip_uart1_device;
			rockchip_mux_api_set(GPIOF1_UART1_CPWM1_NAME, IOMUXA_UART1_SOUT);
		  rockchip_mux_api_set(GPIOF0_UART1_CPWM0_NAME, IOMUXA_UART1_SIN); 
			break;
		default:
			return;
	}
	pdev->id = portnr;		/* update to mapped ID */
      if (portnr < NR_PORTS) 
         rockchip_uarts[portnr] = pdev;      
	
}
void __init rockchip_set_serial_console(unsigned portnr)
{
	if (portnr < NR_PORTS) 
            rockchip_default_console_device = rockchip_uarts[portnr];
	if (!rockchip_default_console_device)
		printk(KERN_INFO "ROCKCHIP: No default serial console defined.\n");
} 

void __init rockchip_add_device_serial(void)
{
	int i; 
        for (i = 0; i< NR_PORTS; i++){
           if (rockchip_uarts[i])
			platform_device_register(rockchip_uarts[i]);	
			//platform_device_register(rockchip_uarts[1]);			
	}
} 
#else
void __init __deprecated rockchip_init_serial(struct rock_uart_config *config) {}
void __init rockchip_register_uart(unsigned id, unsigned portnr, unsigned pins) {}
void __init rockchip_set_serial_console(unsigned portnr) {}
void __init rockchip_add_device_serial(void) {}
#endif 



/****************************************************************************
*		                     I2C
*Author	: wy
*****************************************************************************/
#if defined(CONFIG_I2C_RK28)

static struct resource i2c_resources[] = {
	[0] = {
		.start	= RK28_BASE_I2C,
		.end	= RK28_BASE_I2C + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= RK28_ID_I2C,
		.end	= RK28_ID_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rk28_i2c_device = {
	.name		= "rk28_i2c",
	.id 	= -1,
	.resource	= i2c_resources,
	.num_resources	= ARRAY_SIZE(i2c_resources),
};

void __init rk28_add_device_i2c(struct i2c_board_info *devices, int nr_devices)
{

	i2c_register_board_info(0, devices, nr_devices);
	platform_device_register(&rk28_i2c_device);

	
}
#else
void __init rk28_add_device_i2c(struct i2c_board_info *devices, int nr_devices) {}

#endif


/******************************************************************************************
*	SDMMC
*Author :xbw
******************************************************************************************/

	 
#if defined(CONFIG_MMC_RK28) || defined(CONFIG_MMC_RK28_MODULE)

      //SDMMC0主要用于SD/MMC卡
        static struct rk28_mmc_data __initdata rk28_sdmmc0_data = {
		//.det_pin	= (u8)GPIOPortF_Pin3,							/*xxm*/
		.det_pin	= (u8)SD_DET_IOPIN,

     #if defined(CONFIG_SDMMC0_BUS_WIDTH_4)
        .wire4		= 1,
     #else
        .wire4		= 0,
     #endif
		 
	//	.wp_pin 	= ... not connected
	//	.vcc_pin	= ... not connected
	};

	
	
	static struct resource sdmmc0_resources[] = {
		[0] = {
			.start = SDMMC0_BASE_ADDR,
			.end   = SDMMC0_BASE_ADDR+ SZ_8K -1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			 .start = IRQ_SDMMC0,  
			.end	= IRQ_SDMMC0, 
			.flags	= IORESOURCE_IRQ,
		},
	
	};
	
	
	static struct platform_device rk28_sdmmc0_device = {
		.name		= "rk28_sdmmc0",
		.id 	= -1,
		.dev		= {
					.platform_data		= &rk28_sdmmc0_data,
		},
		.resource	= sdmmc0_resources,
		.num_resources	= ARRAY_SIZE(sdmmc0_resources),
	};


	//SDMMC1主要用于SDIO设备，如Wifi.
	static struct rk28_mmc_data __initdata rk28_sdmmc1_data = {
        #if defined(CONFIG_SDMMC1_BUS_WIDTH_4)
            .wire4		= 1,
        #else
            .wire4		= 0,
        #endif

		//	.det_pin	= ... not connected
		//	.wp_pin 	= ... not connected
		//	.vcc_pin	= ... not connected
		};
	    
	 static struct resource sdmmc1_resources[] = {
		[0] = {
		    .start = SDMMC1_BASE_ADDR,
		    .end   = SDMMC1_BASE_ADDR + SZ_8K -1,
		    .flags = IORESOURCE_MEM,
		},
		[1] = {
		     .start = IRQ_SDMMC1,  
		    .end    = IRQ_SDMMC1, 
		    .flags  = IORESOURCE_IRQ,
		},
	    
	    
	    };
	    
	 static struct platform_device rk28_sdmmc1_device = {
		.name       = "rk28_sdmmc1",
		.id     = -1,
		.dev        = {
		            .platform_data      = &rk28_sdmmc1_data,
		},
		.resource   = sdmmc1_resources,
		.num_resources  = ARRAY_SIZE(sdmmc1_resources),
	    };
	    

	
	
	void __init rk28_add_device_mmc(void)
	{
		rockchip_mux_api_set(GPIOB1_SMCS1_MMC0PCA_NAME, SD_POWER_IOMUX_PINDIR);
       	 GPIOSetPinDirection(SD_POWER_IOPIN, GPIO_OUT);
		GPIOSetPinLevel(SD_POWER_IOPIN, GPIO_LOW);

		rockchip_mux_api_set(WIFI_PWDN_IOMUX_PINNAME, WIFI_PWDN_IOMUX_PINDIR);
            	GPIOSetPinDirection(WIFI_PWDN_IOPIN, GPIO_OUT);
		GPIOSetPinLevel(WIFI_PWDN_IOPIN, GPIO_LOW);
		GPIOSetPinDirection(WIFI_RST_IOPIN, GPIO_OUT);
		GPIOSetPinLevel(WIFI_RST_IOPIN, GPIO_LOW);
    	   	platform_device_register(&rk28_sdmmc0_device); //sdmmc0 主要用 在SD/MMC card
          	 // printk("\n__init rk28_add_device_mmc:  register  the device of SDIO  *****************xbw******************\n\n");
    	   	platform_device_register(&rk28_sdmmc1_device);//sdmmc1 主要用在SDIO, 如Wifi等

	}
	
#else
	void __init rk28_add_device_mmc(void) {}
#endif
	
	



/****************************************************************************
*							 rk 28 nand
*****************************************************************************/
	
static struct resource nand_resources[] = {
		[0] = {
			.start	= NANDC_BASE_ADDR,
			.end	      = NANDC_BASE_ADDR  + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},

		[1] = {
			.start	= REG_FILE_BASE_ADDR,
			.end	      = REG_FILE_BASE_ADDR  + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
	};
static struct platform_device rk28_nand_device = {
	.name		= "rk28xxnand",
	.id 	= -1,
	.resource	= nand_resources,
	.num_resources	= ARRAY_SIZE(nand_resources),
};


void rk28_adddevice_nand(void)
{
	platform_device_register(&rk28_nand_device);
}



/****************************************************************************
*							 key (adc key)
*****************************************************************************/
#define RK28_ID_ADC 15
static struct resource key_resources[] = {
		[0] = {
			.start	= ADC_BASE_ADDR,
			.end	= ADC_BASE_ADDR + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= RK28_ID_ADC,
			.end	= RK28_ID_ADC,
			.flags	= IORESOURCE_IRQ,
		},
	};

	static struct platform_device rk28_key_device = {
		.name		= "rk28_key_button",
		.id 	= -1,
		.resource	= key_resources,
		.num_resources	= ARRAY_SIZE(key_resources),
	};

static void rk28_keydevice_init_io(void)
{
	rockchip_mux_api_set(PLAY_ON_IOMUX_PINNAME, PLAY_ON_IOMUX_PINDIR);
	GPIOPullUpDown(PLAY_ON_IOPIN,GPIOPullUp);
    GPIOSetPinDirection(PLAY_ON_IOPIN, GPIO_IN);

	rockchip_mux_api_set(VBUS_INT_IOMUX_PINNAME, VBUS_INT_IOMUX_PINDIR);
	GPIOPullUpDown(VBUS_INT_IOPIN, GPIOPullDown);
	GPIOSetPinDirection(VBUS_INT_IOPIN, GPIO_IN);
}
void rock28_add_device_key(void)
{
	rk28_keydevice_init_io();
	platform_device_register(&rk28_key_device);
}

/***********************************************************
*	  ps2 at key board 
*	author :lhh	
*	data:2009-10-30
***************************************************************/
#if defined(CONFIG_KEYBOARD_RK28ATBD)  
struct platform_device rk28_device_at_key = {
		.name	= "Rk28 At Keyboard",
		.id 	= -1,
};
void rk28_add_at_key_device(void)
{
    platform_device_register(&rk28_device_at_key);
}
#else
void rk28_add_at_key_device(void){}
#endif
/***********************************************************
*	  ps2 at key board 
*	author :lhh	
*	data:2009-10-30
***************************************************************/
#if defined(CONFIG_MOUSE_PS2_TOUCH_PAD_RK28)  
struct platform_device rk28_device_mouse = {
		.name	= "Rk28 Mouse",
		.id 	= -1,
};
void rk28_add_mouse_device(void)
{
    platform_device_register(&rk28_device_mouse);
}
#else
void rk28_add_mouse_device(void){}
#endif
/***********************************************************
*	  Battery	 
*	author :colin	
*	data:2009-06-05
***************************************************************/
struct platform_device rk28_device_battery = {
		.name	= "rockchip_battery",
		.id 	= -1,
};


void rk28_add_device_battery(void)
{
	//rockchip_mux_api_set(GPIOB0_SPI0CSN1_MMC1PCA_NAME, IOMUXA_GPIO0_B0);	/*check supply full pin*/
	platform_device_register(&rk28_device_battery);
}


/***********************************************************
*	  backlight	 
*	author :nzy	
*	data:2009-07-21
***************************************************************/
static struct rk28bl_info rk28_bl_info = {
		.pwm_id   = 0,
		//.pw_pin   = GPIO_HIGH | (PWR_ON_IOPIN << 8) ,
		//.pw_pin   = GPIO_HIGH | (GPIOPortH_Pin7 << 8) ,
		.pw_pin   = GPIO_HIGH | (LED_CON_IOPIN << 8) ,
		.bl_ref   = 0,
		.pwm_div  = PWM_DIV2,
		.default_intensity	= 0xa0,
};

static struct platform_device rk28_device_backlight = {
				.name	= "rk28_backlight",
				.id 	= -1,
		.dev	= {
		   .platform_data  = &rk28_bl_info,
		}
};

void rk28_add_device_backlight(void)
{
	platform_device_register(&rk28_device_backlight);
}


/* --------------------------------------------------------------------
 *  SPI
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SPIM_RK28) 

static struct resource spi0_resources[] = {
	[0] = {
		.start	= SPI_MASTER_BASE_ADDR,
		.end	= SPI_MASTER_BASE_ADDR + 0x100,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SPIM,
		.end	= IRQ_SPIM,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rockchip_spi0_device = {
	.name		= "rockchip_spi_master",
	.id		= 0,
	.resource	= spi0_resources,
	.num_resources	= ARRAY_SIZE(spi0_resources),
};


void __init rockchip_add_device_spi_master(struct spi_board_info *devices, int nr_devices)
{
	  int i;

	  /* Choose SPI chip-selects */
	  for (i = 0; i < nr_devices; i++) {
		  switch (i) {
		    case 0:
		      rockchip_mux_api_set(GPIOB_SPI0_MMC0_NAME, IOMUXA_SPI0);
		      rockchip_mux_api_set(GPIOB4_SPI0CS0_MMC0D4_NAME, IOMUXA_GPIO0_B4);
		     // __raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x04) & (~0x40),(GPIO1_BASE_ADDR_VA + 0x04));//gpio1a6
		      //__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x38) | 0x40,(GPIO1_BASE_ADDR_VA + 0x38));
		      //__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x3c) & (~0x40),(GPIO1_BASE_ADDR_VA + 0x3c));		
		      //__raw_writel((__raw_readl(REG_FILE_BASE_ADDR_VA + 0x30) & (~0x3000)) | 0x1000,(REG_FILE_BASE_ADDR_VA + 0x30));	
		      break;
		    case 1:
		      rockchip_mux_api_set(GPIOB_SPI0_MMC0_NAME, IOMUXA_SPI0);
		      rockchip_mux_api_set(GPIOB0_SPI0CSN1_MMC1PCA_NAME, IOMUXA_GPIO0_B0);		   
		      break;
		    }
	  }
	spi_register_board_info(devices, nr_devices);	
	platform_device_register(&rockchip_spi0_device);
}
#else
void __init rockchip_add_device_spi_master(struct spi_board_info *devices, int nr_devices) {}
#endif

void __init rk28_add_usb_devices(void)
{
//#ifdef CONFIG_USB_ANDROID
        platform_device_register(&android_usb_device);
        platform_device_register(&rk2818_device_dwc_otg);
#ifdef CONFIG_RK2818_HOST11
        platform_device_register(&rk2818_device_host11);
#endif
//#endif
}

/* 
 *IIS
 */
#if defined(CONFIG_SND_ROCKCHIP_SOC_IIS)  
 
static struct resource rockchip_iis_resource[] = {
	[0] = {
		.start = I2S_BASE_ADDR,
		.end   = I2S_BASE_ADDR + 0x20,
		.flags = IORESOURCE_MEM,
	}
};

static u64 rockchip_device_iis_dmamask = 0xffffffffUL;

struct platform_device rockchip_device_iis = {
	.name		  = "rockchip-i2s",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(rockchip_iis_resource),
	.resource	  = rockchip_iis_resource,
	.dev              = {
		.dma_mask = &rockchip_device_iis_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

void rk28_add_device_i2s(void)
{
    platform_device_register(&rockchip_device_iis);
}
#else	
void rk28_add_device_i2s(void){}
#endif

static struct resource rockchip_dsp_resource[] = {
        [0] = {
                .start = DSP_BASE_ADDR,
                .end   = DSP_BASE_ADDR + 0x5fffff,
                .flags = IORESOURCE_DMA,
        },
        [1] = {
                .start  = IRQ_PIUCMD,
                .end    = IRQ_PIUCMD,
                .flags  = IORESOURCE_IRQ,
        },
        [2] = {
                .start  = IRQ_DSPSWI,
                .end    = IRQ_DSPSWI,
                .flags  = IORESOURCE_IRQ,
        },
};
static u64 rockchip_device_dsp_dmamask = 0xffffffffUL;
struct platform_device rockchip_device_dsp = {
        .name             = "rk28-dsp",
        .id               = 0,
        .num_resources    = ARRAY_SIZE(rockchip_dsp_resource),
        .resource         = rockchip_dsp_resource,
        .dev              = {
                .dma_mask = &rockchip_device_dsp_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};

void rockchip_add_device_dsp(void)
{
        platform_device_register(&rockchip_device_dsp);
}
#ifndef CONFIG_LCD_RK_EINK 
#if defined(CONFIG_ANDROID_PMEM)

/* start is the end of mem from cmdline!!total sdram 256M. */
//#define PMEM_START              (0x60000000+208*0x100000)
#define PMEM_START              (0x60000000+128*0x100000)
#define PMEM_DSP0_SIZE          (0)//(0x1000000)			/*For froyo UI 32M    xxm*/
//#define PMEM_DSP0_SIZE          (0x1a00000)		
#define PMEM_DSP1_SIZE          (0)//(0x1500000)

static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
	.start = PMEM_START,
	.size =  PMEM_DSP0_SIZE,
};

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct android_pmem_platform_data pmem_pdata_dsp = {
	.name = "pmem-dsp",
	//.no_allocator = 1,			/*xxm	modify for sensor*/
	.no_allocator = 0,
	.cached = 0,
        .start = PMEM_START+PMEM_DSP0_SIZE,
	.size =  PMEM_DSP1_SIZE,
};

static struct platform_device pmem_device_dsp = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_pdata_dsp },
};

void rockchip_add_device_pmem(void)
{
                platform_device_register(&pmem_device);
  	platform_device_register(&pmem_device_dsp);
}

#else
void rockchip_add_device_pmem(void){}
#endif
#endif

/*----------------------------------------------------------------------- */

