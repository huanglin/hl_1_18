#include <linux/fb.h>
#include <linux/delay.h>
#include <asm/arch/lcdcon.h>
#include <asm/arch/rk28_i2c.h>
#include <asm/arch/rk28_fb.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/anx7150.h>

#include "screen.h"


/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#define DCLK_POL		1 //for test
#define SWAP_RB			0

/* 720p@60Hz Timing */
#define OUT_CLK	    HDMI_CLK_74M
#define H_PW			40
#define H_BP			220
#define H_VD			1280
#define H_FP			110
#define V_PW			5
#define V_BP			20
#define V_VD			720
#define V_FP			5


/* 720p@50Hz Timing */
#define OUT_CLK2	    HDMI_CLK_74M
#define H_PW2			40
#define H_BP2			220
#define H_VD2			1280
#define H_FP2			440
#define V_PW2			5
#define V_BP2			20
#define V_VD2			720
#define V_FP2			5

/* 576p@50Hz Timing */
#define OUT_CLK3		HDMI_CLK_27M
#define H_PW3			64
#define H_BP3			68
#define H_VD3			720
#define H_FP3			12
#define V_PW3			5
#define V_BP3			39
#define V_VD3			576
#define V_FP3			5

int anx7150_init(void);
int anx7150_standby(u8 enable);


void set_hdmi_info(struct rk28fb_screen *screen)
{
    struct rk28fb_screen *screen2 = screen + 1;
	struct rk28fb_screen *screen3 = screen + 2;

    /* ****************** 720p@60Hz ******************* */
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
    screen->init = anx7150_init;
    screen->standby = anx7150_standby;


    /* ****************** 720p@50Hz ******************* */
    /* screen type & face */
    screen2->type = OUT_TYPE;
    screen2->face = OUT_FACE;

    /* Screen size */
    screen2->x_res = H_VD2;
    screen2->y_res = V_VD2;

    /* Timing */
    screen2->pixclock = OUT_CLK2;
	screen2->left_margin = H_BP2;
	screen2->right_margin = H_FP2;
	screen2->hsync_len = H_PW2;
	screen2->upper_margin = V_BP2;
	screen2->lower_margin = V_FP2;
	screen2->vsync_len = V_PW2;

	/* Pin polarity */
	screen2->pin_hsync = 0;
	screen2->pin_vsync = 0;
	screen2->pin_den = 0;
	screen2->pin_dclk = DCLK_POL;

	/* Swap rule */
    screen2->swap_rb = SWAP_RB;
    screen2->swap_rg = 0;
    screen2->swap_gb = 0;
    screen2->swap_delta = 0;
    screen2->swap_dumy = 0;

    /* Operation function*/
    screen2->init = anx7150_init;
    screen2->standby = anx7150_standby;

	/* ****************** 576p@50Hz ******************* */
	/* screen type & face */
	screen3->type = OUT_TYPE;
	screen3->face = OUT_FACE;

	/* Screen size */
	screen3->x_res = H_VD3;
	screen3->y_res = V_VD3;

	/* Timing */
	screen3->pixclock = OUT_CLK3;
	screen3->left_margin = H_BP3;
	screen3->right_margin = H_FP3;
	screen3->hsync_len = H_PW3;
	screen3->upper_margin = V_BP3;
	screen3->lower_margin = V_FP3;
	screen3->vsync_len = V_PW3;

	/* Pin polarity */
	screen3->pin_hsync = 0;
	screen3->pin_vsync = 0;
	screen3->pin_den = 0;
	screen3->pin_dclk = DCLK_POL;

	/* Swap rule */
	screen3->swap_rb = SWAP_RB;
	screen3->swap_rg = 0;
	screen3->swap_gb = 0;
	screen3->swap_delta = 0;
	screen3->swap_dumy = 0;

	/* Operation function*/
	screen3->init = anx7150_init;
	screen3->standby = anx7150_standby;
}


int anx7150_init(void)
{
    return 0;
}

int anx7150_standby(u8 enable)
{
    return 0;
}

