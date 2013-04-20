#include <linux/fb.h>
#include <linux/delay.h>
#include <asm/arch/lcdcon.h>
#include <asm/arch/rk28_i2c.h>
#include <asm/arch/rk28_fb.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include "screen.h"

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#define OUT_CLK			27

/* Timing */
#define H_PW			10
#define H_BP			206
#define H_VD			800
#define H_FP			40

#define V_PW			10
#define V_BP			25
#define V_VD			480
#define V_FP			10

/* Other */
#define DCLK_POL		0
#define SWAP_RB			0

#define CS_OUT()        GPIOSetPinDirection(LCD_CS_IOPIN, GPIO_OUT)
#define CS_SET()        GPIOSetPinLevel(LCD_CS_IOPIN, GPIO_HIGH)
#define CS_CLR()        GPIOSetPinLevel(LCD_CS_IOPIN, GPIO_LOW)
#define CLK_OUT()       GPIOSetPinDirection(LCD_CLK_IOPIN, GPIO_OUT)  //I2C1_SCL
#define CLK_SET()       GPIOSetPinLevel(LCD_CLK_IOPIN, GPIO_HIGH)
#define CLK_CLR()       GPIOSetPinLevel(LCD_CLK_IOPIN, GPIO_LOW)
#define TXD_OUT()       GPIOSetPinDirection(LCD_TXD_IOPIN, GPIO_OUT)  //I2C1_SDA
#define TXD_SET()       GPIOSetPinLevel(LCD_TXD_IOPIN, GPIO_HIGH)
#define TXD_CLR()       GPIOSetPinLevel(LCD_TXD_IOPIN, GPIO_LOW)

int init(void);
int standby(u8 enable);

void screen_set_iomux(u8 enable)
{
    int ret=-1;
    if(enable)
    {
        rockchip_mux_api_set(GPIOA4_I2SLRCK_SEL_NAME, 0);
        rockchip_mux_api_set(GPIOE_U1IR_I2C1_NAME, 0);   
    }
    else
    {
         rockchip_mux_api_set(GPIOA4_I2SLRCK_SEL_NAME, 1);
         rockchip_mux_api_set(GPIOE_U1IR_I2C1_NAME, 2);
    }
    
    return ;
}

void set_lcd_info(struct rk28fb_screen *screen)
{
    /* screen type & face */
    screen->type = OUT_TYPE;
    screen->face = OUT_FACE;

    /* Screen size */
    screen->x_res = H_VD;
    screen->y_res = V_VD;

    /* Timing */
    screen->pixclock = OUT_CLK;
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;

	/* Pin polarity */
	screen->pin_hsync = 0;
	screen->pin_vsync = 0;
	screen->pin_den = 0;
	screen->pin_dclk = DCLK_POL;

	/* Swap rule */
    screen->swap_rb = SWAP_RB;
    screen->swap_rg = 0;
    screen->swap_gb = 0;
    screen->swap_delta = 0;
    screen->swap_dumy = 0;
    /* Operation function*/
    screen->init =init;
    screen->standby = standby;
}


void spi_screenreg_set(u32 Addr, u32 Data)
{

#define DRVDelayUs(i)   udelay(i*2)

    u32 i;

    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    DRVDelayUs(2);
    DRVDelayUs(2);

    CS_SET();
    TXD_SET();
    CLK_SET();
    DRVDelayUs(2);

	CS_CLR();
	for(i = 0; i < 6; i++)  //reg
	{
		if(Addr &(1<<(5-i)))
			TXD_SET();
		else
			TXD_CLR();

		// \u6a21\u62dfCLK
		CLK_CLR();
		DRVDelayUs(2);
		CLK_SET();
		DRVDelayUs(2);
	}

	TXD_CLR();  //write

	// \u6a21\u62dfCLK
    CLK_CLR();
    DRVDelayUs(2);
    CLK_SET();
    DRVDelayUs(2);

	TXD_SET();  //highz

	// \u6a21\u62dfCLK
    CLK_CLR();
    DRVDelayUs(2);
    CLK_SET();
    DRVDelayUs(2);


	for(i = 0; i < 8; i++)  //data
	{
		if(Data &(1<<(7-i)))
			TXD_SET();
		else
			TXD_CLR();

		// \u6a21\u62dfCLK
        CLK_CLR();
		DRVDelayUs(2);
		CLK_SET();
		DRVDelayUs(2);
	}

	CS_SET();
	CLK_CLR();
	TXD_CLR();
	DRVDelayUs(2);

}


int init(void)
{    
    screen_set_iomux(1);

    spi_screenreg_set(0x02, 0x07);
    spi_screenreg_set(0x03, 0x5f);
    spi_screenreg_set(0x04, 0x17);
    spi_screenreg_set(0x05, 0x20);
    spi_screenreg_set(0x06, 0x08);
    spi_screenreg_set(0x07, 0x20);
    spi_screenreg_set(0x08, 0x20);
    spi_screenreg_set(0x09, 0x20);
    spi_screenreg_set(0x0a, 0x20);
    spi_screenreg_set(0x0b, 0x22);
    spi_screenreg_set(0x0c, 0x22);
    spi_screenreg_set(0x0d, 0x22);
    spi_screenreg_set(0x0e, 0x10);
    spi_screenreg_set(0x0f, 0x10);
    spi_screenreg_set(0x10, 0x10);

    spi_screenreg_set(0x11, 0x15);
    spi_screenreg_set(0x12, 0xAA);
    spi_screenreg_set(0x13, 0xFF);
    spi_screenreg_set(0x14, 0xb0);
    spi_screenreg_set(0x15, 0x8e);
    spi_screenreg_set(0x16, 0xd6);
    spi_screenreg_set(0x17, 0xfe);
    spi_screenreg_set(0x18, 0x28);
    spi_screenreg_set(0x19, 0x52);
    spi_screenreg_set(0x1A, 0x7c);

    spi_screenreg_set(0x1B, 0xe9);
    spi_screenreg_set(0x1C, 0x42);
    spi_screenreg_set(0x1D, 0x88);
    spi_screenreg_set(0x1E, 0xb8);
    spi_screenreg_set(0x1F, 0xFF);
    spi_screenreg_set(0x20, 0xF0);
    spi_screenreg_set(0x21, 0xF0);
    spi_screenreg_set(0x22, 0x09);

    screen_set_iomux(0);
    return 0;
}

int standby(u8 enable)
{       
    screen_set_iomux(1);
	if(enable) {
		spi_screenreg_set(0x03, 0xde);
	} else {
		spi_screenreg_set(0x03, 0x5f);
	}
    screen_set_iomux(0);
    return 0;
}

