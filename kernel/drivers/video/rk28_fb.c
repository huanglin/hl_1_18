/*
 * linux/drivers/video/rk28fb.c
 *	    RK28 LCD Controller Frame Buffer Driver
 *	    based on skeletonfb.c, s3c2410fb.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/backlight.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/wait.h>

#include <asm/io.h>
#include <asm/div64.h>

#include <asm/mach/map.h>
#include <asm/arch/typedef.h>
#include <asm/arch/rk28_fb.h>
#include <asm/arch/rk28_timer.h>


#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <asm/arch/lcdcon.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>
#include <asm/uaccess.h>

#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#ifdef CONFIG_ANX7150
#include <asm/arch/anx7150.h>
#endif
#ifdef CONFIG_TV_RK1000
#include <asm/arch/rk1000_tv.h>
#endif

#include "display/screen/screen.h"


#define FMK_USE_HARDTIMER       1       //frame mark use hard timer replace high timer
#define WIN1_USE_DOUBLE_BUF     1       //win1 use double buf to accelerate display
#define LANDSCAPE_USE_ROTATE    1       //rotate win1 in landscape with mcu panel

#if 0
	#define fbprintk(msg...)	printk(msg);
#else
	#define fbprintk(msg...)
#endif


#if 0
	#define fbprintk2(msg...)	printk(msg);
#else
	#define fbprintk2(msg...)
#endif


#define LcdReadBit(inf, addr, msk)      ((inf->regbak.addr=inf->preg->addr)&(msk))
#define LcdWrReg(inf, addr, val)        inf->preg->addr=inf->regbak.addr=(val)
#define LcdRdReg(inf, addr)             (inf->preg->addr)
#define LcdSetBit(inf, addr, msk)       inf->preg->addr=((inf->regbak.addr) |= (msk))
#define LcdClrBit(inf, addr, msk)       inf->preg->addr=((inf->regbak.addr) &= ~(msk))
#define LcdMskReg(inf, addr, msk, val)  (inf->regbak.addr)&=~(msk);   inf->preg->addr=(inf->regbak.addr|=(val));

#define CalScale(x, y)	                (uint16)( (x<y) ? ( (((uint32)x*0x10000)/y)+((((uint32)x*0x10000)%y)?1:0)) : (0) )

#define IsMcuLandscape()                ((SCREEN_MCU==inf->cur_screen->type) && ((90==inf->mcu_scandir)||(270==inf->mcu_scandir)))
#define IsMcuUseFmk()                   ( (2==inf->cur_screen->mcu_usefmk) || ((1==inf->cur_screen->mcu_usefmk)&&IsMcuLandscape()) )

struct rk28fb_rgb {
	struct fb_bitfield	red;
	struct fb_bitfield	green;
	struct fb_bitfield	blue;
	struct fb_bitfield	transp;
};

static struct rk28fb_rgb def_rgb_16 = {
     red:    { offset: 11, length: 5, },
     green:  { offset: 5,  length: 6, },
     blue:   { offset: 0,  length: 5, },
     transp: { offset: 0,  length: 0, },
};


struct win0_fmk {
    u8 completed;
    u8 enable;
	u8 format;
	u32 addr_y[2];
	u32 addr_uv[2];
	u16 act_w[2];
	u16 win_w[2];
	u16 win_stx[2];
	u32 addr_y2offset;
	u32 addr_uv2offset;
};

struct win0_par {
	u32 refcount;
	u32	pseudo_pal[16];
	u32 y_offset;
	u32 uv_offset;

	struct win0_fmk fmktmp;
	struct win0_fmk fmk;

    u8 par_seted;
    u8 addr_seted;
};


struct win1_fmk {
    u8 completed;
    u8 enable;
	u32 addr[2];
	u16 win_stx[2];
	u16 act_w[2];
	u32 addr2_offset;
};

struct win1_par {
	u32 refcount;
	u32	pseudo_pal[16];
	int lstblank;

	struct win1_fmk fmktmp;
	struct win1_fmk fmk;
};

struct rk28fb_inf {
    struct fb_info *win0fb;
    struct fb_info *win1fb;

    void __iomem *reg_vir_base;  // virtual basic address of lcdc register
	u32 reg_phy_base;       // physical basic address of lcdc register
	u32 len;               // physical map length of lcdc register

    /* lcdc reg base address and backup reg */
    LCDC_REG *preg;
    LCDC_REG regbak;

	int in_suspend;

    /* variable used in mcu panel */
	int mcu_needflush;
	int mcu_isrcnt;
	u16 mcu_scandir;
	struct timer_list mcutimer;
	int mcu_status;
	int mcu_ebooknew;
	int mcu_usetimer;   // use timer to flush mcu panel for booting
	int mcu_stopflush;
	struct hrtimer htimer;
	int mcu_fmkstate;

    /* external memery */
	char __iomem *screen_base2;
    __u32 smem_len2;
    unsigned long  smem_start2;

    struct rk28fb_screen lcd_info;
    struct rk28fb_screen tv_info[5];
    struct rk28fb_screen hdmi_info[2];
    struct rk28fb_screen *cur_screen;

#ifdef CONFIG_ANDROID_POWER
    android_early_suspend_t early_suspend;
#endif
};

typedef enum _TRSP_MODE
{
    TRSP_CLOSE = 0,
    TRSP_FMREG,
    TRSP_FMREGEX,
    TRSP_FMRAM,
    TRSP_FMRAMEX,
    TRSP_MASK,
    TRSP_INVAL
} TRSP_MODE;

typedef enum _FMK_STATE
{
    FMK_IDLE = 0,
    FMK_INT,
    FMK_PRELEFT,
    FMK_PREUP,
    FMK_LEFT,
    FMK_UP,
    FMK_ENDING

} FMK_STATE;


struct platform_device *g_pdev = NULL;
static int rk28fb_suspend(struct platform_device *pdev, pm_message_t msg);
static int win1fb_set_par(struct fb_info *info);

#if FMK_USE_HARDTIMER
int rk28fb_dohardtimer(void);
#endif

#ifdef CONFIG_ANDROID_POWER
static void rk28fb_early_suspend(android_early_suspend_t *h);
static void rk28fb_early_resume(android_early_suspend_t *h);
#endif


#if 0
#define CHK_SUSPEND(inf)	\
	if(inf->in_suspend)	{	\
		fbprintk(">>>>>> fb is in suspend! return! \n");	\
		return -EPERM;	\
	}
#else
#define CHK_SUSPEND(inf)
#endif

static DECLARE_WAIT_QUEUE_HEAD(wq);
static int wq_condition = 0;

void rgb565_to_gray(uint8_t *gray, uint8_t *rgb, int rgblen)
{
#define rgb_gray4bpp(r,g,b)        ((((30 * r)/31 + (59 * g)/63 + (11 * b)/31)*15)/100)

	int i, j;
	u8 r, g, b, g0, g1;
	u16 *rgb_buf = (u16 *)rgb;
	u16 rgb_data;

#if 0
	for(i = 0; i < rgblen/2; i+=2){
		rgb_data = *rgb_buf++;
		r = (rgb_data>>11)& 0x1F;
		g = (rgb_data>>5)& 0x3F;
		b = rgb_data & 0x1F;
		g0 = rgb_gray4bpp(r, g, b);

		rgb_data = *rgb_buf++;
		r = (rgb_data>>11)& 0x1F;
		g = (rgb_data>>5)& 0x3F;
		b = rgb_data & 0x1F;
		g1 = rgb_gray4bpp(r, g, b);

		*gray++ = (g0 & 0x0F) | ((g1 & 0xF)<<4);
	}
#else
    for(i=599; i>=0; i--) {
        for(j=0; j<800; j+=2) {
            rgb_data = *(rgb_buf+600*j+i);
            r = (rgb_data>>11)& 0x1F;
            g = (rgb_data>>5)& 0x3F;
            b = rgb_data & 0x1F;
            g0 = rgb_gray4bpp(r, g, b);

            rgb_data = *(rgb_buf+600*(j+1)+i);
            r = (rgb_data>>11)& 0x1F;
            g = (rgb_data>>5)& 0x3F;
            b = rgb_data & 0x1F;
            g1 = rgb_gray4bpp(r, g, b);

            *gray++ = (g0 & 0x0F) | ((g1 & 0xF)<<4);
        }
    }
#endif
}


void rgb565_rotate(uint16_t *dst, uint16_t *src, int srcw, int srch)
{
    int i,j,k=0;

    for(i=srcw-1; i>=0; i--)
    {
        for(j=0; j<srch; j++)
        {
            *(dst+k) = *(src+j*srcw+i);
            k++;
        }
    }
}


int mcu_do_refresh(struct rk28fb_inf *inf)
{
    if(inf->mcu_stopflush)  return 0;

    if(SCREEN_MCU!=inf->cur_screen->type)   return 0;

    // use frame mark
    if(IsMcuUseFmk()) {
        if(FMK_IDLE==inf->mcu_fmkstate) {
            inf->mcu_fmkstate = FMK_INT;
        } else {
            inf->mcu_needflush = 1;
            return (1);
        }
        return 0;
    }

    // not use frame mark
    if(LcdReadBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDMODE)) {
        if(!LcdReadBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDED)) {
            inf->mcu_needflush = 1;
        } else {
            if(inf->cur_screen->refresh)    inf->cur_screen->refresh(REFRESH_PRE);
            inf->mcu_needflush = 0;
            inf->mcu_isrcnt = 0;
            LcdSetBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDSIGNAL);
        }
    }
    return 0;
}


void mcutimer_callback(unsigned long arg)
{
    struct rk28fb_inf *inf = platform_get_drvdata(g_pdev);
    static int waitcnt = 0;
    static int newcnt = 0;

    mod_timer(&inf->mcutimer, jiffies + HZ/10);

    switch(inf->mcu_status)
    {
    case MS_IDLE:
        if(OUT_P16BPP4==inf->cur_screen->face) {
            inf->mcu_status = MS_EBOOK;
        } else {
            inf->mcu_status = MS_MCU;
        }
        break;
    case MS_MCU:
        if(inf->mcu_usetimer)   mcu_do_refresh(inf);
        break;
    case MS_EBOOK:
        if(inf->mcu_ebooknew) {
            inf->mcu_ebooknew = 0;
            inf->mcu_status = MS_EWAITSTART;
            newcnt = 0;
        }
        break;
    case MS_EWAITSTART:
        if(inf->mcu_ebooknew) {
            inf->mcu_ebooknew = 0;
            if(newcnt++>10) {
                inf->mcu_status = MS_EWAITEND;
                waitcnt = 0;
            }
        } else {
            inf->mcu_status = MS_EWAITEND;
            waitcnt = 0;
        }
        break;
    case MS_EWAITEND:
        if(0==waitcnt) {
            mcu_do_refresh(inf);
        }
        if(waitcnt++>14) {
            inf->mcu_status = MS_EEND;
        }
        break;
    case MS_EEND:
        inf->mcu_status = MS_MCU;
        break;
    default:
        inf->mcu_status = MS_MCU;
        break;
    }
}


int mcu_refresh(struct rk28fb_inf *inf)
{
    static int mcutimer_inited = 0;

    if(SCREEN_MCU!=inf->cur_screen->type)   return 0;

    if(!mcutimer_inited) {
        mcutimer_inited = 1;
        init_timer(&inf->mcutimer);
        inf->mcutimer.function = mcutimer_callback;
        inf->mcutimer.expires = jiffies + HZ/5;
        inf->mcu_status = 0;
        add_timer(&inf->mcutimer);
    }

    inf->mcu_ebooknew = 1;
    if(MS_MCU==inf->mcu_status)     mcu_do_refresh(inf);

    return 0;
}



void lcdc_soft_rst(void)
{
	volatile unsigned long *scu_softrst_con = (unsigned long *)(SCU_BASE_ADDR_VA + 40);

	*scu_softrst_con |= (1 << 1);  //reset
	udelay(10);
	*scu_softrst_con &= ~(1 << 1); //clean reset
}

int set_lcd_clk(int freq)
{
	if(freq <= 0){
		printk("<3>!!! Invalid lcd clk, %dM Hz.\n", freq);
		return -1;
	}
	
	//printk("<3>lcdc clk: %d M Hz\n", freq);
	
	switch(freq){
	case 26: // hmdi 576p, 27M
		__rockchip_scu_set_parent(SCU_IPID_LCDC, SCU_IPID_ARM, SCU_MODE_NONE, freq);
		__rockchip_clk_set_unit_clock(SCU_IPID_LCDC , 27);
		break;
	case 74: // hdmi 720p, 74.25M
		__rockchip_scu_set_parent(SCU_IPID_LCDC, SCU_IPID_ARM, SCU_MODE_SETDIV, 8);
		break;
	case 75: // PBR 720, 74.25M 
        __rockchip_scu_set_parent(SCU_IPID_LCDC, SCU_IPID_CODEC, SCU_MODE_NONE, freq);
		__rockchip_clk_set_unit_clock(SCU_IPID_LCDC , 74);
		__rockchip_clk_set_unit_clock(SCU_IPID_CODEC, 594);
		break;
	default:
        __rockchip_scu_set_parent(SCU_IPID_LCDC, SCU_IPID_CODEC, SCU_MODE_NONE, freq);
		__rockchip_clk_set_unit_clock(SCU_IPID_LCDC , freq);
		break;
	}

	return 0;
}

int get_lcd_width(void)
{
	struct fb_info *info = registered_fb[1];
	struct rk28fb_inf *inf = info->device->driver_data;

	return inf->lcd_info.x_res;
}

int get_lcd_height(void)
{
	struct fb_info *info = registered_fb[1];
	struct rk28fb_inf *inf = info->device->driver_data;

	return inf->lcd_info.y_res;
}

int win0fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);

void set_lcd_pin(struct platform_device *pdev, int enable)
{
	struct rk28fb_mach_info *mach_info = pdev->dev.platform_data;

	unsigned lcd_cs = mach_info->gpio->lcd_cs&0xffff;
	unsigned display_on = mach_info->gpio->display_on&0xffff;
	unsigned lcd_standby = mach_info->gpio->lcd_standby&0xffff;

	int lcd_cs_pol = (mach_info->gpio->lcd_cs>>16)&0xffff;
	int display_on_pol = (mach_info->gpio->display_on>>16)&0xffff;
	int lcd_standby_pol = (mach_info->gpio->lcd_standby>>16)&0xffff;

	fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);
	fbprintk(">>>>>> lcd_cs(%d) = %d \n", lcd_cs, enable ? lcd_cs_pol : !lcd_cs_pol);
	fbprintk(">>>>>> display_on(%d) = %d \n", display_on, enable ? display_on_pol : !display_on_pol);
	fbprintk(">>>>>> lcd_standby(%d) = %d \n", lcd_standby, enable ? lcd_standby_pol : !lcd_standby_pol);

    // set cs and display_on
    if(mach_info->gpio->lcd_cs) {
        gpio_direction_output(lcd_cs, 0);
		GPIOSetPinLevel(lcd_cs, enable ? lcd_cs_pol : !lcd_cs_pol);
	}
    if(mach_info->gpio->display_on) {
        gpio_direction_output(display_on, 0);
		GPIOSetPinLevel(display_on, enable ? display_on_pol : !display_on_pol);
    }
    if(mach_info->gpio->lcd_standby) {
        gpio_direction_output(lcd_standby, 0);
		GPIOSetPinLevel(lcd_standby, enable ? lcd_standby_pol : !lcd_standby_pol);
    }
}


int init_lcdc(struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    u32 reg1=0, reg2=0, msk=0, clr=0;

	fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

	// eable the lcdc ram buffer
    LcdMskReg(inf, RAM_CEN_CTRL, m_RAM1BUF0_CEN|m_RAM1BUF1_CEN,
        v_RAM1BUF0_CEN(0)|v_RAM1BUF1_CEN(0));

	// set AHB access rule and disable all windows
    LcdMskReg(inf, SYS_CONFIG,
        m_AHB_JUMP | m_AHB_FIELD | m_AHB_INCR | m_AHB_SIZE | m_AHB_WRITE |
        m_AHB_BURST |m_W0_MASTER | m_W1_MASTER | m_W0_ENC |m_W1_A0ENC |
        m_W1_A1ENC | m_W1_A2ENC | m_W1_A3ENC,
        v_AHB_JUMP(0) | v_AHB_FIELD(0) | v_AHB_INCR(31) |
        v_AHB_SIZE(2) | v_AHB_WRITE(0) | v_AHB_BURST(1) |
        v_W0_MASTER(1) | v_W1_MASTER(1) |v_W0_ENC(0) |
        v_W1_A0ENC(0) | v_W1_A1ENC(0) | v_W1_A2ENC(0) | v_W1_A3ENC(0)
        );  // use ahb burst32

	// set all swap rule for every window and set water mark
	reg1 = v_W0_RBSWAP(0) | v_W0YRGB_8SWAP(0) | v_W0YRGB_16SWAP(0) |
        v_W0YRGB_MID8SWAP(0) | v_W0YRGB_LOOPSWAP(0) | v_W0CBR_8SWAP(0) |
        v_W0CBR_16SWAP(0) | v_W0CBR_LOOPSWAP(0);
	reg2 = v_W1A0_RBSWAP(0) | v_W1A1_RBSWAP(0) | v_W1A2_RBSWAP(0) |
        v_W1A3_RBSWAP(0) | v_W1A0_8SWAP(0) | v_W1A1_8SWAP(0) |
        v_W1A2_8SWAP(0) | v_W1A3_8SWAP(0) | v_W1A0_16SWAP(0) |
        v_W1A1_16SWAP(0) | v_W1A2_16SWAP(0) | v_W1A3_16SWAP(0) |
        v_W1A0_LOOPSWAP(0) | v_W1A1_LOOPSWAP(0) | v_W1A2_LOOPSWAP(0) |
        v_W1A3_LOOPSWAP(0) | v_WATER_MARK(0x7f);
	LcdWrReg(inf, WIN1_WATERMARK, reg1 | reg2);

	// disable blank out, black out, tristate out, yuv2rgb bypass
	// and mcu holdmode; and set win1 top.
    LcdMskReg(inf, WIN0_VIR, m_TRISTATE_OUT, v_TRISTATE_OUT(0));
    LcdMskReg(inf, DSP_CTRL_REG0, m_TOPWIN0|m_BLACK_OUT, v_TOPWIN0(0)|v_BLACK_OUT(0));
    LcdMskReg(inf, DSP_CTRL_REG1, m_BLANK_OUT, v_BLANK_OUT(0));
    LcdMskReg(inf, MCU_TIMING_CTRL, m_MCU_HOLDSIGNAL|m_YUV2RGB_BYPASS|m_MCU_HOLDMODE,
        v_MCU_HOLDSIGNAL(0)|v_YUV2RGB_BYPASS(0)|v_MCU_HOLDMODE(1));

    // initialize all interrupt
    clr = v_AHB_ERRCLEAR(1) | v_W1A0_ETYCLEAR(1) | v_W1A1_ETYCLEAR(1) |
        v_W1A2_ETYCLEAR(1) | v_W1A3_ETYCLEAR(1) | v_W0CBR_ETYCLEAR(1) |
        v_W0YRGB_MTYCLEAR(1) | v_HOR_STARTCLEAR(1) | v_FRM_STARTCLEAR(1);
    msk = v_AHB_ERRMASK(1) | v_W1A0_ETYMASK(1) | v_W1A1_ETYMASK(1) |
        v_W1A2_ETYMASK(1) | v_W1A3_ETYMASK(1) | v_W0CBR_ETYMASK(1) |
        v_W0YRGB_MTYMASK(1) | v_HOR_STARTMASK(1) | v_FRM_STARTMASK(0);
    LcdWrReg(inf, INT_LUT, clr | msk);

	// let above to take effect
    LcdWrReg(inf, REG_CFG_DONE, 0x01);

    return 0;

}


void load_screen(struct fb_info *info, bool initscreen)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct rk28fb_screen *screen = inf->cur_screen;
    uint16 face = screen->face;
    uint16 mcu_total, mcu_rwstart, mcu_csstart, mcu_rwend, mcu_csend;
    uint16 right_margin = screen->right_margin, lower_margin = screen->lower_margin;
    uint16 x_res = screen->x_res, y_res = screen->y_res;

	fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

	if(OUT_P16BPP4==face)   face = OUT_P565;

    // set the rgb or mcu
    LcdMskReg(inf, SYS_CONFIG, m_MCU_SELECT, v_MCU_SELECT((SCREEN_MCU==screen->type)?(1):(0)));

	// set out format and mcu timing
    mcu_total  = (screen->mcu_wrperiod*150*1000)/1000000;
    if(mcu_total>31)    mcu_total = 31;
    if(mcu_total<3)     mcu_total = 3;
    mcu_rwstart = (mcu_total+1)/4 - 1;
    mcu_rwend = ((mcu_total+1)*3)/4 - 1;
    mcu_csstart = (mcu_rwstart>2) ? (mcu_rwstart-3) : (0);
    mcu_csend = (mcu_rwend>15) ? (mcu_rwend-1) : (mcu_rwend);

    fbprintk(">> mcu_total=%d, mcu_rwstart=%d, mcu_csstart=%d, mcu_rwend=%d, mcu_csend=%d \n",
        mcu_total, mcu_rwstart, mcu_csstart, mcu_rwend, mcu_csend);
    LcdMskReg(inf, MCU_TIMING_CTRL,
        m_BG_SWAP | m_DIS_FMT1 | m_MCU_CSSTART | m_MCU_CSEND |
        m_MCU_RWSTART | m_MCU_RWEND | m_MCU_TOTAL | m_MCU_HOLDSIGNAL | m_MCU_HOLDMODE,
        v_BG_SWAP(screen->swap_gb) | v_DIS_FMT1(face) | v_MCU_CSSTART(mcu_csstart) |
        v_MCU_CSEND(mcu_csend) | v_MCU_RWSTART(mcu_rwstart) | v_MCU_RWEND(mcu_rwend) |
        v_MCU_TOTAL(mcu_total) | v_MCU_HOLDSIGNAL(0) | v_MCU_HOLDMODE((SCREEN_MCU==screen->type)?(1):(0))
        );

	// set synchronous pin polarity and data pin swap rule
    LcdMskReg(inf, DSP_CTRL_REG0,
        m_DIS_FMT0 | m_H_PINPOL | m_V_PINPOL | m_DEN_PINPOL |
        m_DCLK_PINPOL |m_INTER_LACE | m_RB_SWAP | m_RG_SWAP,
        v_DIS_FMT0((face)>>1) | v_H_PINPOL(screen->pin_hsync) | v_V_PINPOL(screen->pin_vsync) |
        v_DEN_PINPOL(screen->pin_den) | v_DCLK_PINPOL(screen->pin_dclk) | v_INTER_LACE(0) |
        v_RB_SWAP(screen->swap_rb) | v_RG_SWAP(screen->swap_rg)
        );

    LcdMskReg(inf, DSP_CTRL_REG1,
        m_SWAP_DELTA | m_SWAP_DUMY | m_BLACK_RGB,
        v_SWAP_DELTA(screen->swap_delta) | v_SWAP_DUMY(screen->swap_dumy) | v_BLACK_RGB(0)
        );

	// set horizontal & vertical out timing
	if(IsMcuLandscape()) {
        x_res = (screen->mcu_usefmk) ? (screen->y_res/2) : (screen->y_res);
        y_res = screen->x_res;
	}
	if(SCREEN_MCU==inf->cur_screen->type) {
	    right_margin = x_res/6;
	}

    if(right_margin)     right_margin--;
    LcdMskReg(inf, DSP_HS_END, m_BIT11LO, v_BIT11LO(screen->hsync_len));
    LcdMskReg(inf, DSP_HACT_ST, m_BIT11LO, v_BIT11LO(screen->hsync_len + screen->left_margin));
    if(OUT_P16BPP4==screen->face) {    // p16bpp4
        LcdMskReg(inf, DSP_HACT_END, m_BIT11LO, v_BIT11LO(screen->hsync_len + screen->left_margin + x_res/4));
        LcdMskReg(inf, DSP_HTOTAL, m_BIT11LO, v_BIT11LO(screen->hsync_len + screen->left_margin + x_res/4 + right_margin));
    } else {
        LcdMskReg(inf, DSP_HACT_END, m_BIT11LO, v_BIT11LO(screen->hsync_len + screen->left_margin + x_res));
        LcdMskReg(inf, DSP_HTOTAL, m_BIT11LO, v_BIT11LO(screen->hsync_len + screen->left_margin + x_res + right_margin));
    }
    if(lower_margin)     lower_margin--;
    LcdMskReg(inf, DSP_VS_END, m_BIT11LO, v_BIT11LO(screen->vsync_len));
    LcdMskReg(inf, DSP_VACT_ST, m_BIT11LO, v_BIT11LO(screen->vsync_len + screen->upper_margin));
    LcdMskReg(inf, DSP_VACT_END, m_BIT11LO, v_BIT11LO(screen->vsync_len + screen->upper_margin + y_res));
    LcdMskReg(inf, DSP_VTOTAL, m_BIT11LO, v_BIT11LO(screen->vsync_len + screen->upper_margin + y_res + lower_margin));
    LcdMskReg(inf, DSP_VS_ST_F1, m_BIT11LO, v_BIT11LO(0));
    LcdMskReg(inf, DSP_VS_END_F1, m_BIT11LO, v_BIT11LO(0));
    LcdMskReg(inf, DSP_VACT_ST_F1, m_BIT11LO, v_BIT11LO(0));
    LcdMskReg(inf, DSP_VACT_END_F1, m_BIT11LO, v_BIT11LO(0));

	// let above to take effect
    LcdWrReg(inf, REG_CFG_DONE, 0x01);

    // set lcdc clk
    if(SCREEN_MCU==screen->type)    screen->pixclock = 150; //mcu fix to 150 MHz
    set_lcd_clk(screen->pixclock);

	//reset lcdc
	lcdc_soft_rst();
	*inf->preg = inf->regbak;

    // init screen panel
    if(screen->init && initscreen)
    	screen->init();
}

static inline unsigned int chan_to_field(unsigned int chan,
					 struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int fb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	unsigned int val;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseudo-palette */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			val  = chan_to_field(red,   &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue,  &info->var.blue);
			pal[regno] = val;
		}
		break;
	default:
		return -1;	/* unknown type */
	}

	return 0;
}


static int win0fb_blank(int blank_mode, struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct win0_par *par = info->par;

    fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

	CHK_SUSPEND(inf);

    switch(blank_mode)
    {
    case FB_BLANK_UNBLANK:
        LcdMskReg(inf, SYS_CONFIG, m_W0_ENC, v_W0_ENC(1));
        par->fmktmp.enable = 1;
        par->fmktmp.completed = 1;
        break;
    default:
        LcdMskReg(inf, SYS_CONFIG, m_W0_ENC, v_W0_ENC(0));
        par->fmktmp.enable = 0;
        par->fmktmp.completed = 1;
        break;
    }
    LcdWrReg(inf, REG_CFG_DONE, 0x01);

	mcu_refresh(inf);
    return 0;
}



static int win0fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct rk28fb_screen *screen = inf->cur_screen;

    u16 xpos = (var->nonstd>>8) & 0xfff;
    u16 ypos = (var->nonstd>>20) & 0xfff;
    u16 xsize = (var->grayscale>>8) & 0xfff;
    u16 ysize = (var->grayscale>>20) & 0xfff;
    u16 xlcd = screen->x_res;
    u16 ylcd = screen->y_res;
    if(IsMcuLandscape()) {
        xlcd = screen->y_res;
        ylcd = screen->x_res;
    }

    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
/* For HDMI, zyy add 2010.1.10*/
#ifdef CONFIG_ANX7150
	if(anx7150_get_output_status() == HDMI){
		fbprintk(">>>>>> output is hdmi!\n");
		
		switch(anx7150_get_output_resolution()){
		case HDMI_720x576:
			xpos = 0;
			ypos = 0;
			xsize = 720;
			ysize = 576;
			break;
		case HDMI_1280x720:
			xpos = 0;
			ypos = 0;
			xsize = 1280;
			ysize = 720;
			break;
		default:
			fbprintk(">>>>>> unsupported  resloution!\n");
			break;
		}
	}
#endif

/* For TvOut, zyy add 2010.3.4 */
#ifdef CONFIG_TV_RK1000
	switch(rk1000_tv_get_output_status()){
	case Cvbs_NTSC:
		xpos = 0;
		ypos = 0;
		xsize = 720;
		ysize = 480;
		break;
	case Cvbs_PAL:
		xpos = 0;
		ypos = 0;
		xsize = 720;
		ysize = 576;
		break;
	case Ypbpr480:
		xpos = 0;
		ypos = 0;
		xsize = 720;
		ysize = 480;
		break;
	case Ypbpr576:
		xpos = 0;
		ypos = 0;
		xsize = 720;
		ysize = 576;
		break;
	case Ypbpr720:
		xpos = 0;
		ypos = 0;
		xsize = 1280;
		ysize = 720;
		break;
	default:
		break;
	}
#endif

	CHK_SUSPEND(inf);

    if( 0==var->xres_virtual || 0==var->yres_virtual ||
        0==var->xres || 0==var->yres || var->xres<16 ||
        0==xsize || 0==ysize || xsize<16 ||
        ((16!=var->bits_per_pixel)&&(32!=var->bits_per_pixel)) )
    {
        printk(">>>>>> win0fb_check_var fail 1!!! \n");
		printk("0==%d || 0==%d || 0==%d || 0==%d || %d<16 \n ||0==%d || 0==%d || %d<16 ||((16!=%d)&&(32!=%d)) \n",
				var->xres_virtual, var->yres_virtual, var->xres, var->yres, var->xres, xsize, ysize, xsize,
        		var->bits_per_pixel, var->bits_per_pixel);
        return -EINVAL;
    }


    if( (var->xoffset+var->xres)>var->xres_virtual ||
        (var->yoffset+var->yres)>var->yres_virtual ||
        (xpos+xsize)>xlcd || (ypos+ysize)>ylcd )
    {
        printk(">>>>>> win0fb_check_var fail 2!!! \n");
		printk("(%d+%d)>%d || (%d+%d)>%d || (%d+%d)>%d || (%d+%d)>%d \n ",
				var->xoffset, var->xres, var->xres_virtual, var->yoffset, var->yres,
				var->yres_virtual, xpos, xsize, xlcd, ypos, ysize, ylcd);
        return -EINVAL;
    }

    switch(var->nonstd&0xff)
    {
    case 0: // rgb
        switch(var->bits_per_pixel)
        {
        case 16:    // rgb565
            var->xres_virtual = (var->xres_virtual + 0x1) & (~0x1);
            var->xres = (var->xres + 0x1) & (~0x1);
            var->xoffset = (var->xoffset) & (~0x1);
            break;
        default:    // rgb888
            var->bits_per_pixel = 32;
            break;
        }
        break;
    case 1: // yuv422
        var->xres_virtual = (var->xres_virtual + 0x3) & (~0x3);
        var->xres = (var->xres + 0x3) & (~0x3);
        var->xoffset = (var->xoffset) & (~0x3);
        break;
    case 2: // yuv4200
        var->xres_virtual = (var->xres_virtual + 0x3) & (~0x3);
        var->yres_virtual = (var->yres_virtual + 0x1) & (~0x1);
        var->xres = (var->xres + 0x3) & (~0x3);
        var->yres = (var->yres + 0x1) & (~0x1);
        var->xoffset = (var->xoffset) & (~0x3);
        var->yoffset = (var->yoffset) & (~0x1);
        break;
    case 3: // yuv4201
        var->xres_virtual = (var->xres_virtual + 0x3) & (~0x3);
        var->yres_virtual = (var->yres_virtual + 0x1) & (~0x1);
        var->xres = (var->xres + 0x3) & (~0x3);
        var->yres = (var->yres + 0x1) & (~0x1);
        var->xoffset = (var->xoffset) & (~0x3);
        var->yoffset = (var->yoffset) & (~0x1);
        break;
    case 4: // yuv420m
        var->xres_virtual = (var->xres_virtual + 0x7) & (~0x7);
        var->yres_virtual = (var->yres_virtual + 0x1) & (~0x1);
        var->xres = (var->xres + 0x7) & (~0x7);
        var->yres = (var->yres + 0x1) & (~0x1);
        var->xoffset = (var->xoffset) & (~0x7);
        var->yoffset = (var->yoffset) & (~0x1);
        break;
    case 5: // yuv444
        var->xres_virtual = (var->xres_virtual + 0x3) & (~0x3);
        var->xres = (var->xres + 0x3) & (~0x3);
        var->xoffset = (var->xoffset) & (~0x3);
        break;
    default:
        printk(">>>>>> win0fb var->nonstd=%d is invalid! \n", var->nonstd);
        return -EINVAL;
    }

    return 0;
}


static int win0fb_set_par(struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct rk28fb_screen *screen = inf->cur_screen;
    struct fb_var_screeninfo *var = &info->var;
    struct fb_fix_screeninfo *fix = &info->fix;
    struct win0_par *par = info->par;

    u8 format = 0;
    dma_addr_t map_dma;
    u32 y_offset=0, uv_offset=0, cblen=0, crlen=0, map_size=0, smem_len=0;
    u16 xscale_dn, xscale_up, yscale_dn, yscale_up, xres=var->xres;
    u16 xpos = (var->nonstd>>8) & 0xfff;
    u16 ypos = (var->nonstd>>20) & 0xfff;
    u16 xsize = (var->grayscale>>8) & 0xfff;
    u16 ysize = (var->grayscale>>20) & 0xfff;
    u32 win0_en = var->reserved[2];
    u32 y_addr = var->reserved[3];
    u32 uv_addr = var->reserved[4];

    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
/* For HDMI, zyy add 2010.1.10*/
#ifdef CONFIG_ANX7150
	if(anx7150_get_output_status() == HDMI){
		fbprintk(">>>>>> output is hdmi!\n");
		
		switch(anx7150_get_output_resolution()){
		case HDMI_720x576:
			xpos = 0;
			ypos = 0;
			xsize = 720;
			ysize = 576;
			break;
		case HDMI_1280x720:
			xpos = 0;
			ypos = 0;
			xsize = 1280;
			ysize = 720;
			break;
		default:
			fbprintk(">>>>>> unsupported  resloution!\n");
			break;
		}
	}
#endif

/* For TvOut, zyy add 2010.3.4 */
#ifdef CONFIG_TV_RK1000
	switch(rk1000_tv_get_output_status()){
	case Cvbs_NTSC:
		xpos = 0;
		ypos = 0;
		xsize = 720;
		ysize = 480;
		break;
	case Cvbs_PAL:
		xpos = 0;
		ypos = 0;
		xsize = 720;
		ysize = 576;
		break;
	case Ypbpr480:
		xpos = 0;
		ypos = 0;
		xsize = 720;
		ysize = 480;
		break;
	case Ypbpr576:
		xpos = 0;
		ypos = 0;
		xsize = 720;
		ysize = 576;
		break;
	case Ypbpr720:
		xpos = 0;
		ypos = 0;
		xsize = 1280;
		ysize = 720;
		break;
	default:
		break;
	}
#endif
	CHK_SUSPEND(inf);

	/* calculate y_offset,uv_offset,line_length,cblen and crlen  */
    switch(var->nonstd&0xff)
    {
    case 0: // rgb
        switch(var->bits_per_pixel)
        {
        case 16:    // rgb565
            format = 1;
            fix->line_length = 2 * var->xres_virtual;
            y_offset = (var->yoffset*var->xres_virtual + var->xoffset)*2;
            break;
        case 32:    // rgb888
            format = 0;
            fix->line_length = 4 * var->xres_virtual;
            y_offset = (var->yoffset*var->xres_virtual + var->xoffset)*4;
            break;
        default:
            return -EINVAL;
        }
        break;
    case 1: // yuv422
        format = 2;
        fix->line_length = var->xres_virtual;
        y_offset = var->yoffset*var->xres_virtual + var->xoffset;
        uv_offset = var->yoffset*var->xres_virtual + var->xoffset;
        cblen = crlen = (var->xres_virtual*var->yres_virtual)/2;
        break;
    case 2: // yuv4200
        format = 3;
        fix->line_length = var->xres_virtual;
        y_offset = var->yoffset*var->xres_virtual + var->xoffset;
        uv_offset = (var->yoffset/2)*var->xres_virtual + var->xoffset;
        cblen = crlen = (var->xres_virtual*var->yres_virtual)/4;
        break;
    case 3: // yuv4201
        format = 4;
        fix->line_length = var->xres_virtual;
        y_offset = (var->yoffset/2)*2*var->xres_virtual + (var->xoffset)*2;
        uv_offset = (var->yoffset/2)*var->xres_virtual + var->xoffset;
        cblen = crlen = (var->xres_virtual*var->yres_virtual)/4;
        break;
    case 4: // yuv420m
        format = 5;
        fix->line_length = var->xres_virtual;
        y_offset = (var->yoffset/2)*3*var->xres_virtual + (var->xoffset)*3;
        cblen = crlen = (var->xres_virtual*var->yres_virtual)/4;
        break;
    case 5: // yuv444
        format = 6;
        fix->line_length = var->xres_virtual;
        y_offset = var->yoffset*var->xres_virtual + var->xoffset;
        uv_offset = var->yoffset*2*var->xres_virtual + var->xoffset*2;
        cblen = crlen = (var->xres_virtual*var->yres_virtual);
        break;
    default:
        return -EINVAL;
    }

    smem_len = fix->line_length * var->yres_virtual + cblen + crlen;
    map_size = PAGE_ALIGN(smem_len);

    if(y_addr && uv_addr)  // buffer alloced by user
    {
        if (info->screen_base) {
            printk(">>>>>> win0fb unmap memory(%d)! \n", info->fix.smem_len);
            dma_free_writecombine(NULL, PAGE_ALIGN(info->fix.smem_len),info->screen_base, info->fix.smem_start);
	        info->screen_base = 0;
        }
        fix->smem_start = y_addr;
        fix->smem_len = smem_len;
        fix->mmio_start = uv_addr;

        par->addr_seted = ((-1==(int)y_addr)&&(-1==(int)uv_addr)) ? 0 : 1;
    }
    else    // driver alloce buffer
    {
        if ( (smem_len != fix->smem_len) || !info->screen_base )     // buffer need realloc
        {
            fbprintk(">>>>>> win0 buffer size is change! remap memory!\n");
            fbprintk(">>>>>> smem_len %d = %d * %d + %d + %d\n", smem_len, fix->line_length, var->yres_virtual, cblen, crlen);
            fbprintk(">>>>>> map_size = %d\n", map_size);
            LcdMskReg(inf, SYS_CONFIG, m_W0_ENC, v_W0_ENC(0));
            LcdWrReg(inf, REG_CFG_DONE, 0x01);
            msleep(50);
            if (info->screen_base) {
                   fbprintk(">>>>>> win0fb unmap memory(%d)! \n", info->fix.smem_len);
	            dma_free_writecombine(NULL, PAGE_ALIGN(info->fix.smem_len),info->screen_base, info->fix.smem_start);
	            info->screen_base = 0;
	            fix->smem_start = 0;
	            fix->smem_len = 0;
    	    }

    	    info->screen_base = dma_alloc_writecombine(NULL, map_size, &map_dma, GFP_KERNEL);
            if(!info->screen_base) {
                printk(">>>>>> win0fb dma_alloc_writecombine fail!\n");
                return -ENOMEM;
            }
            memset(info->screen_base, 0x00, map_size);
            fix->smem_start = map_dma;
            fix->smem_len = smem_len;
            fix->mmio_start = fix->smem_start + fix->line_length * var->yres_virtual;
            fbprintk(">>>>>> alloc succ, smem_start=%08x, smem_len=%d, mmio_start=%d!\n",
                (u32)fix->smem_start, fix->smem_len, (u32)fix->mmio_start);
        }
    }

    par->y_offset = y_offset;
    par->uv_offset = uv_offset;

	// calculate the display phy address
    y_addr = fix->smem_start + y_offset;
    uv_addr = fix->mmio_start + uv_offset;
    fbprintk("y_addr 0x%08x = 0x%08x + %d\n", y_addr, (u32)fix->smem_start, y_offset);
    fbprintk("uv_addr 0x%08x = 0x%08x + %d\n", uv_addr, (u32)fix->mmio_start , uv_offset);

    // MCU屏横屏时采用左右半屏送数消除斜线
    if( screen->mcu_usefmk && IsMcuLandscape() )
    {
        u16 win_w=0, act_w=0, xlcd=screen->y_res;
        u16 fact_w=0, fact_y=0, fact_uv=0;

        switch(format) {
            case 0:  fact_w=1;  fact_y=4;  fact_uv=0;   break;
            case 1:  fact_w=2;  fact_y=2;  fact_uv=0;   break;
            case 2:  fact_w=4;  fact_y=1;  fact_uv=1;   break;
            case 3:  fact_w=4;  fact_y=1;  fact_uv=1;   break;
            case 4:  fact_w=4;  fact_y=2;  fact_uv=1;   break;
            case 5:  fact_w=4;  fact_y=3;  fact_uv=0;   break;
            case 6:  fact_w=4;  fact_y=1;  fact_uv=2;   break;
            default: fact_w=1;  fact_y=4;  fact_uv=0;   break;
        }

        par->fmktmp.completed = 0;
        par->fmktmp.enable = (win0_en) ? (1) : (0);
        par->fmktmp.format = format;

        if ( xpos >= (xlcd/2) )   //窗口处于右边
        {
            par->fmktmp.addr_y[0] = 0;
            par->fmktmp.addr_y2offset = 0;
            par->fmktmp.addr_y[1] = y_addr;
            par->fmktmp.addr_uv[1] = uv_addr;
            par->fmktmp.act_w[1] = xres;
            par->fmktmp.win_w[1] = xsize;
            par->fmktmp.win_stx[1] = xpos - xlcd/2;
        }
        else if ( (xpos+xsize) <= (xlcd/2) ) //窗口处于左边
        {
            par->fmktmp.addr_y[1] = 0;
            par->fmktmp.addr_y2offset = 0;
            par->fmktmp.addr_y[0] = y_addr;
            par->fmktmp.addr_uv[0] = uv_addr;
            par->fmktmp.act_w[0] = xres;
            par->fmktmp.win_w[0] = xsize;
            par->fmktmp.win_stx[0] = xpos;
        }
        else    //窗口处于左右两边
        {
            win_w = (xlcd/2) - xpos;
            act_w = (xres * win_w) / xsize;
            act_w = (act_w/fact_w) * fact_w;

            if (act_w<win_w)
            {
                xscale_up = CalScale(act_w, win_w);
                while ( (((xscale_up*win_w)/0x2000)%8) != 0 )
                {
                    xscale_up--;
                }
                act_w = (xscale_up*win_w)/0x10000;
            }

            par->fmktmp.addr_y[0] = y_addr;
            par->fmktmp.addr_uv[0] = uv_addr;
            par->fmktmp.act_w[0] = act_w + fact_w;
            par->fmktmp.win_w[0] = win_w;
            par->fmktmp.win_stx[0] = xpos;
            par->fmktmp.addr_y2offset = act_w * fact_y;
            par->fmktmp.addr_uv2offset = act_w * fact_uv;
            par->fmktmp.act_w[1] = xres - act_w;
            par->fmktmp.win_w[1] = xsize - win_w;
            par->fmktmp.win_stx[1] = 0;
            xsize = win_w;
            xres = act_w;
        }
        par->fmktmp.completed = 1;
    }

    if (xres>1280 && xsize>1280) {
        xscale_dn = CalScale(1280, xres);
        xscale_up = CalScale(1280, xsize);
    } else {
        xscale_dn = CalScale(xsize, xres);
        xscale_up = CalScale(xres, xsize);
    }

    yscale_dn = CalScale(ysize, var->yres);
    yscale_up = CalScale(var->yres, ysize);

    if (yscale_dn>0 && yscale_dn<0x2000) {
        return -EINVAL;
    }

    fbprintk(" xscale_dn = 0x%04x xscale_up = 0x%04x \n yscale_dn = 0x%04x yscale_up = 0x%04x \n",
        xscale_dn, xscale_up, yscale_dn, yscale_up);

    xpos += LcdReadBit(inf, DSP_HACT_ST, m_BIT11LO);
    ypos += LcdReadBit(inf, DSP_VACT_ST, m_BIT11LO);

    LcdWrReg(inf, WIN0_YRGB_MST, y_addr);
    LcdWrReg(inf, WIN0_CBR_MST, uv_addr);

    LcdMskReg(inf, SYS_CONFIG, m_W0_ENC | m_W0_INFMT | m_AHB_JUMP,
        v_W0_ENC(win0_en) | v_W0_INFMT(format) | v_AHB_JUMP(0));
    LcdMskReg(inf, WIN0_VIR, m_W0_VIRWIDTH, v_W0_VIRWIDTH(var->xres_virtual));
    LcdMskReg(inf, WIN0_ACT_INFO, m_WORDLO | m_WORDHI, v_WORDLO(xres) | v_WORDHI(var->yres));
    LcdMskReg(inf, DSP_WIN0_ST, m_BIT11LO | m_BIT11HI, v_BIT11LO(xpos) | v_BIT11HI(ypos));
    LcdMskReg(inf, DSP_WIN0_INFO, m_BIT11LO | m_BIT11HI,  v_BIT11LO(xsize) | v_BIT11HI(ysize));
    LcdMskReg(inf, SD_FACTOR, m_WORDLO | m_WORDHI, v_WORDLO(xscale_dn) | v_WORDHI(yscale_dn));
    LcdMskReg(inf, SP_FACTOR, m_WORDLO | m_WORDHI, v_WORDLO(xscale_up) | v_WORDHI(yscale_up));
    LcdMskReg(inf, DSP_CTRL_REG1, m_SCALE_DROPLINE,
        v_SCALE_DROPLINE((yscale_dn>=0x2000 && yscale_dn<=0x8000)?(1):(0)));
    switch(format)
    {
    case 0:
        LcdMskReg(inf, WIN1_WATERMARK, m_W0YRGB_8SWAP | m_W0YRGB_16SWAP | m_W0YRGB_LOOPSWAP | m_W0_RBSWAP | m_W0YRGB_MID8SWAP,
                v_W0YRGB_8SWAP(1) | v_W0YRGB_16SWAP(1) | v_W0YRGB_LOOPSWAP(1) | v_W0_RBSWAP(0)| v_W0YRGB_MID8SWAP(0) );
        break;
    case 1:
        LcdMskReg(inf, WIN1_WATERMARK, m_W0YRGB_8SWAP | m_W0YRGB_16SWAP | m_W0YRGB_LOOPSWAP | m_W0_RBSWAP | m_W0YRGB_MID8SWAP,
                v_W0YRGB_8SWAP(0) | v_W0YRGB_16SWAP(0) | v_W0YRGB_LOOPSWAP(0) | v_W0_RBSWAP(1) | v_W0YRGB_MID8SWAP(0) );
        break;
    case 4:
        LcdMskReg(inf, WIN1_WATERMARK, m_W0YRGB_8SWAP | m_W0YRGB_16SWAP | m_W0YRGB_LOOPSWAP | m_W0_RBSWAP | m_W0YRGB_MID8SWAP,
                v_W0YRGB_8SWAP(0) | v_W0YRGB_16SWAP(0) | v_W0YRGB_LOOPSWAP(0) | v_W0_RBSWAP(0) | v_W0YRGB_MID8SWAP(1) );
        break;
    default:
        LcdMskReg(inf, WIN1_WATERMARK, m_W0YRGB_8SWAP | m_W0YRGB_16SWAP | m_W0YRGB_LOOPSWAP | m_W0_RBSWAP | m_W0YRGB_MID8SWAP,
                v_W0YRGB_8SWAP(0) | v_W0YRGB_16SWAP(0) | v_W0YRGB_LOOPSWAP(0) | v_W0_RBSWAP(0) | v_W0YRGB_MID8SWAP(0) );
        break;
    }

    LcdWrReg(inf, REG_CFG_DONE, 0x01);

    return 0;
}

static int win0fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct fb_var_screeninfo *var0 = &info->var;
    struct fb_fix_screeninfo *fix0 = &info->fix;
    struct win0_par *par = info->par;
    u32 y_offset=0, uv_offset=0, y_addr=0, uv_addr=0;

    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);

	CHK_SUSPEND(inf);

    switch(var0->nonstd&0xff)
    {
    case 0: // rgb
        switch(var0->bits_per_pixel)
        {
        case 16:    // rgb565
            var->xoffset = (var->xoffset) & (~0x1);
            y_offset = (var->yoffset*var0->xres_virtual + var->xoffset)*2;
            break;
        default:    // rgb888
            y_offset = (var->yoffset*var0->xres_virtual + var->xoffset)*4;
            break;
        }
        break;
    case 1: // yuv422
        var->xoffset = (var->xoffset) & (~0x3);
        y_offset = var->yoffset*var0->xres_virtual + var->xoffset;
        uv_offset = var->yoffset*var0->xres_virtual + var->xoffset;
        break;
    case 2: // yuv4200
        var->xoffset = (var->xoffset) & (~0x3);
        var->yoffset = (var->yoffset) & (~0x1);
        y_offset = var->yoffset*var0->xres_virtual + var->xoffset;
        uv_offset = (var->yoffset/2)*var0->xres_virtual + var->xoffset;
        break;
    case 3: // yuv4201
        var->xoffset = (var->xoffset) & (~0x3);
        var->yoffset = (var->yoffset) & (~0x1);
        y_offset = (var->yoffset/2)*2*var0->xres_virtual + (var->xoffset)*2;
        uv_offset = (var->yoffset/2)*var0->xres_virtual + var->xoffset;
        break;
    case 4: // yuv420m
        var->xoffset = (var->xoffset) & (~0x7);
        var->yoffset = (var->yoffset) & (~0x1);
        y_offset = (var->yoffset/2)*3*var0->xres_virtual + (var->xoffset)*3;
        break;
    case 5: // yuv444
        var->xoffset = (var->xoffset) & (~0x3);
        y_offset = var->yoffset*var0->xres_virtual + var->xoffset;
        uv_offset = var->yoffset*2*var0->xres_virtual + var->xoffset*2;
        break;
    default:
        return -EINVAL;
    }

    par->y_offset = y_offset;
    par->uv_offset = uv_offset;

    y_addr = fix0->smem_start + y_offset;
    uv_addr = fix0->mmio_start + uv_offset;

    LcdWrReg(inf, WIN0_YRGB_MST, y_addr);
    LcdWrReg(inf, WIN0_CBR_MST, uv_addr);
    LcdWrReg(inf, REG_CFG_DONE, 0x01);

    if( inf->cur_screen->mcu_usefmk && IsMcuLandscape() ) {
        par->fmktmp.completed = 0;
        if(0==par->fmktmp.addr_y[0] && 0==par->fmktmp.addr_y2offset) {    //right
            par->fmktmp.addr_y[1] = y_addr;
            par->fmktmp.addr_uv[1] = uv_addr;
        } else if(0==par->fmktmp.addr_y[1] && 0==par->fmktmp.addr_y2offset) { //left
            par->fmktmp.addr_y[0] = y_addr;
            par->fmktmp.addr_uv[0] = uv_addr;
        } else {    //middle
            par->fmktmp.addr_y[0] = y_addr;
            par->fmktmp.addr_uv[0] = uv_addr;
        }
        par->fmktmp.completed = 1;
    }

    // enable win0 after the win0 addr is seted
	par->par_seted = 1;
	par->fmktmp.enable = (1==par->addr_seted) ? (1) : (0);
	LcdMskReg(inf, SYS_CONFIG, m_W0_ENC, v_W0_ENC((1==par->addr_seted)?(1):(0)));
	mcu_refresh(inf);

    return 0;
}

int win0fb_open(struct fb_info *info, int user)
{
    struct win0_par *par = info->par;

    fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

    par->par_seted = 0;
    par->addr_seted = 0;

    if(par->refcount) {
        printk(">>>>>> win0fb has opened! \n");
        return -EACCES;
    } else {
        par->refcount++;
        return 0;
    }
}

int win0fb_release(struct fb_info *info, int user)
{
    struct win0_par *par = info->par;
	struct fb_var_screeninfo *var0 = &info->var;

    fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

    if(par->refcount) {
        par->refcount--;

        win0fb_blank(FB_BLANK_POWERDOWN, info);
        // wait for lcdc stop access memory
        msleep(50);

        // unmap memory
        if (info->screen_base) {
            printk(">>>>>> win0fb unmap memory(%d)! \n", info->fix.smem_len);
    	    dma_free_writecombine(NULL, PAGE_ALIGN(info->fix.smem_len),info->screen_base, info->fix.smem_start);
    	    info->screen_base = 0;
    	    info->fix.smem_start = 0;
    	    info->fix.smem_len = 0;
        }

		// clean the var param
		memset(var0, 0, sizeof(struct fb_var_screeninfo));
    }

    return 0;
}


int win0fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct win0_par *par = info->par;
    void __user *argp = (void __user *)arg;

	//fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

	CHK_SUSPEND(inf);

    switch(cmd)
    {
    case FB1_IOCTL_GET_PANEL_SIZE:    //get panel size
        {
            u32 panel_size[2];
            if(IsMcuLandscape()) {
                panel_size[0] = inf->cur_screen->y_res;
                panel_size[1] = inf->cur_screen->x_res;
            } else {
                panel_size[0] = inf->cur_screen->x_res;
                panel_size[1] = inf->cur_screen->y_res;
            }
            if(copy_to_user(argp, panel_size, 8))  return -EFAULT;
        }
        break;

    case FB1_IOCTL_SET_YUV_ADDR:    //set y&uv address to register direct
        {
            u32 yuv_phy[2];
            if (copy_from_user(yuv_phy, argp, 8))
			    return -EFAULT;

            yuv_phy[0] += par->y_offset;
            yuv_phy[1] += par->uv_offset;

            //printk("new y_addr=%08x, new uv_addr=%08x \n", yuv_phy[0], yuv_phy[1]);
            LcdWrReg(inf, WIN0_YRGB_MST, yuv_phy[0]);
            LcdWrReg(inf, WIN0_CBR_MST, yuv_phy[1]);
            LcdWrReg(inf, REG_CFG_DONE, 0x01);

            if( inf->cur_screen->mcu_usefmk && IsMcuLandscape() ) {
                par->fmktmp.completed = 0;
                if(0==par->fmktmp.addr_y[0] && 0==par->fmktmp.addr_y2offset) {    //right
                    par->fmktmp.addr_y[1] = yuv_phy[0];
                    par->fmktmp.addr_uv[1] = yuv_phy[1];
                } else if(0==par->fmktmp.addr_y[1] && 0==par->fmktmp.addr_y2offset) { //left
                    par->fmktmp.addr_y[0] = yuv_phy[0];
                    par->fmktmp.addr_uv[0] = yuv_phy[1];
                } else {    //middle
                    par->fmktmp.addr_y[0] = yuv_phy[0];
                    par->fmktmp.addr_uv[0] = yuv_phy[1];
                }
                par->fmktmp.completed = 1;
            }

            // enable win0 after the win0 par is seted
            par->addr_seted = 1;
            if(par->par_seted) {
                par->fmktmp.enable = 1;
    	        LcdMskReg(inf, SYS_CONFIG, m_W0_ENC, v_W0_ENC(1));
                mcu_refresh(inf);
            }
        }
        break;

    case FB1_TOCTL_SET_MCU_DIR:    //change MCU panel scan direction
        {
            fbprintk(">>>>>> change MCU panel scan direction(%d) \n", (int)arg);

            if(SCREEN_MCU!=inf->cur_screen->type)   return -1;

            switch(arg)
            {
            case 0:
            case 90:
            case 180:
            case 270:
                {
                    if(inf->cur_screen->scandir) {
                        inf->mcu_stopflush = 1;
                        inf->mcu_needflush = 0;
                        inf->mcu_status = FMK_IDLE;
                        while(!LcdReadBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDED)) {
                            msleep(10);
                        }
                        msleep(10);
                        while(!LcdReadBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDED)) {
                            msleep(10);
                        }
                        msleep(10);

                        inf->cur_screen->scandir(arg);
                    }
                    inf->mcu_scandir = arg;
                    load_screen(info, 0);
                    msleep(10);
                    inf->mcu_stopflush = 0;
                    win1fb_set_par(inf->win1fb);
                }
                break;

            default:
                return -1;
            }
        }
        break;
    default:
        break;
    }
    return 0;
}


static struct fb_ops win0fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open    = win0fb_open,
	.fb_release = win0fb_release,
	.fb_check_var	= win0fb_check_var,
	.fb_set_par	= win0fb_set_par,
	.fb_blank	= win0fb_blank,
    .fb_pan_display = win0fb_pan_display,
    .fb_ioctl = win0fb_ioctl,
	.fb_setcolreg	= fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};


static int win1fb_blank(int blank_mode, struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct win1_par *par = info->par;

    fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

	CHK_SUSPEND(inf);

	switch(blank_mode)
    {
    case FB_BLANK_UNBLANK:
        LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC, v_W1_A0ENC(1));
        par->fmktmp.enable = 1;
        par->fmktmp.completed = 1;
        break;
    default:
        LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC, v_W1_A0ENC(0));
        par->fmktmp.enable = 0;
        par->fmktmp.completed = 1;
        break;
    }
    LcdWrReg(inf, REG_CFG_DONE, 0x01);

	mcu_refresh(inf);
    return 0;
}



static int win1fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct rk28fb_screen *screen = inf->cur_screen;

    u16 xpos = (var->nonstd>>8) & 0xfff;
    u16 ypos = (var->nonstd>>20) & 0xfff;
    u16 xlcd = screen->x_res;
    u16 ylcd = screen->y_res;
    u8 trspmode = (var->grayscale>>8) & 0xff;
    u8 trspval = (var->grayscale) & 0xff;

    //fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);

	CHK_SUSPEND(inf);

#if (0==WIN1_USE_DOUBLE_BUF)
    if(var->yres_virtual>ylcd)
        var->yres_virtual = ylcd;
#endif

    if( 0==var->xres_virtual || 0==var->yres_virtual ||
        0==var->xres || 0==var->yres || var->xres<16 ||
        trspmode>5 || trspval>16 ||
        ((16!=var->bits_per_pixel)&&(32!=var->bits_per_pixel)) )
    {
        printk(">>>>>> win1fb_check_var fail 1!!! \n");
        printk(">>>>>> 0==%d || 0==%d ", var->xres_virtual,var->yres_virtual);
        printk("0==%d || 0==%d || %d<16 || ", var->xres,var->yres,var->xres<16);
        printk("%d>5 || %d>16 \n", trspmode,trspval);
        printk("bits_per_pixel=%d \n", var->bits_per_pixel);
        return -EINVAL;
    }

    if( (var->xoffset+var->xres)>var->xres_virtual ||
        (var->yoffset+var->yres)>var->yres_virtual ||
        (xpos+var->xres)>xlcd || (ypos+var->yres)>ylcd )
    {
        printk(">>>>>> win1fb_check_var fail 2!!! \n");
        printk(">>>>>> (%d+%d)>%d || ", var->xoffset,var->xres,var->xres_virtual);
        printk("(%d+%d)>%d || ", var->yoffset,var->yres,var->yres_virtual);
        printk("(%d+%d)>%d || (%d+%d)>%d \n", xpos,var->xres,xlcd,ypos,var->yres,ylcd);
        return -EINVAL;
    }

    switch(var->bits_per_pixel)
    {
    case 16:    // rgb565
        var->xres_virtual = (var->xres_virtual + 0x1) & (~0x1);
        var->xres = (var->xres + 0x1) & (~0x1);
        var->xoffset = (var->xoffset) & (~0x1);
        break;
    default:    // rgb888
        var->bits_per_pixel = 32;
        break;
    }

    return 0;
}


static int win1fb_set_par(struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct fb_var_screeninfo *var = &info->var;
    struct fb_fix_screeninfo *fix = &info->fix;
    struct rk28fb_screen *screen = inf->cur_screen;
    struct win1_par *par = info->par;

    u8 format = 0;
    dma_addr_t map_dma;
    u32 offset=0, addr=0, map_size=0, smem_len=0;
    u16 xpos = (var->nonstd>>8) & 0xfff;
    u16 ypos = (var->nonstd>>20) & 0xfff;
    u16 xres = var->xres, yres = var->yres, xres_virtual = var->xres_virtual;
    // u8 trspmode = TRSP_FMREG; //(var->grayscale>>8) & 0xff;
    // u8 trspmode = TRSP_CLOSE;    // 不使用硬件的半透操作.
    u8 trspmode = TRSP_MASK;        // 将指定的 像素 value (缺省是 黑色(0, 0, 0) ), 设置为 全透.
    u8 trspval = 1;	//(var->grayscale) & 0xff;

    //the below code is not correct, make we can't see alpha picture.
    //u8 trspmode = (var->grayscale>>8) & 0xff;
    //u8 trspval = (var->grayscale) & 0xff;

    //fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);

	CHK_SUSPEND(inf);

    switch(var->bits_per_pixel)
    {
    case 16:    // rgb565
        format = 0;
        fix->line_length = 2 * var->xres_virtual;
        offset = (var->yoffset*var->xres_virtual + var->xoffset)*2;
        break;
    case 32:    // rgb888
    default:
        format = 1;
        fix->line_length = 4 * var->xres_virtual;
        offset = (var->yoffset*var->xres_virtual + var->xoffset)*4;
        break;
    }

    smem_len = fix->line_length * var->yres_virtual;
    map_size = PAGE_ALIGN(smem_len);

#if WIN1_USE_DOUBLE_BUF
    if( var->yres_virtual == 2*screen->y_res ) {
        inf->mcu_usetimer = 0;
    }
    if(0==fix->smem_len) {
        smem_len = smem_len*2;
        map_size = PAGE_ALIGN(smem_len);
        fbprintk(">>>>>> first alloc, alloc double!!! \n ");
    }
#endif

#if WIN1_USE_DOUBLE_BUF
    if (smem_len > fix->smem_len)     // buffer need realloc
#else
    if (smem_len != fix->smem_len)     // buffer need realloc
#endif
    {
        fbprintk(">>>>>> win1 buffer size is change(%d->%d)! remap memory!\n",fix->smem_len, smem_len);
        fbprintk(">>>>>> smem_len %d = %d * %d \n", smem_len, fix->line_length, var->yres_virtual);
        fbprintk(">>>>>> map_size = %d\n", map_size);
        LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC, v_W1_A0ENC(0));
        LcdWrReg(inf, REG_CFG_DONE, 0x01);
        msleep(50);
        if (info->screen_base) {
            printk(">>>>>> win1fb unmap memory(%d)! \n", info->fix.smem_len);
	        dma_free_writecombine(NULL, PAGE_ALIGN(info->fix.smem_len), info->screen_base, info->fix.smem_start);
	        info->screen_base = 0;
	        fix->smem_start = 0;
	        fix->smem_len = 0;
        }

        info->screen_base = dma_alloc_writecombine(NULL, map_size, &map_dma, GFP_KERNEL);
        if(!info->screen_base) {
            printk(">>>>>> win1fb dma_alloc_writecombine fail!\n");
            return -ENOMEM;
        }
        memset(info->screen_base, 0x00, map_size);
        fix->smem_start = map_dma;
        fix->smem_len = smem_len;
        fbprintk(">>>>>> alloc succ, mem=%08x, len=%d!\n", (u32)fix->smem_start, fix->smem_len);

        // malloc external memory
        if(OUT_P16BPP4==screen->face)
        {
            if(inf->screen_base2) {
                printk(">>>>>> win1fb unmap external memory(%d)! \n", inf->smem_len2);
    	        dma_free_writecombine(NULL, PAGE_ALIGN(inf->smem_len2), inf->screen_base2, inf->smem_start2);
    	        inf->screen_base2 = 0;
    	        inf->smem_start2 = 0;
    	        inf->smem_len2 = 0;
            }
            inf->screen_base2 = dma_alloc_writecombine(NULL, PAGE_ALIGN(smem_len/4), (dma_addr_t*)&inf->smem_start2, GFP_KERNEL);
            if(!inf->screen_base2) {
                printk(">>>>>> win1fb external dma_alloc_writecombine fail!\n");
                return -ENOMEM;
            }
            inf->smem_len2 = smem_len/4;
            memset(inf->screen_base2, 0x00, PAGE_ALIGN(inf->smem_len2));
            fbprintk(">>>>>> alloc external succ, mem=%08x, len=%d!\n", (u32)inf->smem_start2, inf->smem_len2);
        }
    }

#if LANDSCAPE_USE_ROTATE
#if WIN1_USE_DOUBLE_BUF
    smem_len = smem_len/2;
#endif
    // malloc external memory2
    static int firstrun = 1;
    if( (IsMcuLandscape() && (inf->smem_len2!=smem_len)) || firstrun )
    {
        if(inf->screen_base2) {
            printk(">>>>>> win1fb unmap external memory(%d)! \n", inf->smem_len2);
	        kfree(inf->screen_base2);
	        inf->screen_base2 = 0;
	        inf->smem_start2 = 0;
	        inf->smem_len2 = 0;
        }
        inf->screen_base2 = kzalloc(PAGE_ALIGN(smem_len), GFP_DMA);
        if(!inf->screen_base2) {
            printk(">>>>>> win1fb external dma_alloc_writecombine fail!\n");
            return -ENOMEM;
        }
        inf->smem_start2 = __pa(inf->screen_base2);
        inf->smem_len2 = smem_len;
        fbprintk(">>>>>> alloc external succ, mem=%08x, len=%d!\n", (u32)inf->smem_start2, inf->smem_len2);
    }
    firstrun = 0;
#endif

    addr = fix->smem_start + offset;

    if(TRSP_FMREGEX==trspmode)     trspmode = TRSP_FMREG;
    if(TRSP_FMRAMEX==trspmode)     trspmode = TRSP_FMRAM;

    if(IsMcuLandscape())
    {
        u16 xlcd_new = screen->y_res;
        u16 ylcd_new = screen->x_res;
        u16 xres_new = yres;
        u16 yres_new = xres;
        u16 xpos_new = ypos;
        u16 ypos_new = ylcd_new - xpos - xres;
#if LANDSCAPE_USE_ROTATE
    #if WIN1_USE_DOUBLE_BUF
        addr = inf->smem_start2;
    #else
        addr = inf->smem_start2 + offset;
    #endif
#endif

    #if WIN1_USE_DOUBLE_BUF
        xres_virtual = var->yres_virtual/2;
    #else
        xres_virtual = var->yres_virtual;
    #endif
        xres = xres_new;
        yres = yres_new;
        xpos = xpos_new;
        ypos = ypos_new;

        if(screen->mcu_usefmk)  // MCU屏横屏时采用左右半屏送数消除斜线
        {
            u16 act_w=0, fact_w=0;

            fact_w = (0==format) ? (2) : (4);

            par->fmktmp.completed = 0;
            par->fmktmp.enable = 1;
            if ( xpos_new >= (xlcd_new/2) )   //窗口处于右边
            {
                par->fmktmp.addr[0] = 0;
                par->fmktmp.addr2_offset = 0;
                par->fmktmp.addr[1] = addr;
                par->fmktmp.act_w[1] = xres_new;
                par->fmktmp.win_stx[1] = xpos_new - xlcd_new/2;
            }
            else if ( (xpos_new + xres_new) <= (xlcd_new/2) ) //窗口处于左边
            {
                par->fmktmp.addr[1] = 0;
                par->fmktmp.addr2_offset = 0;
                par->fmktmp.addr[0] = addr;
                par->fmktmp.act_w[0] = xres_new;
                par->fmktmp.win_stx[0] = xpos_new;
            }
            else    //窗口处于左右两边
            {
                act_w = (xlcd_new/2) - xpos_new;
                par->fmktmp.addr[0] = addr;
                par->fmktmp.act_w[0] = act_w;
                par->fmktmp.win_stx[0] = xpos_new;
                par->fmktmp.addr2_offset = act_w*fact_w;
                par->fmktmp.act_w[1] = xres_new - act_w;
                par->fmktmp.win_stx[1] = 0;
                xres_new = act_w;
            }
            par->fmktmp.completed = 1;
        }
    }

    LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC|m_W1_A0FMT, v_W1_A0ENC(1)|v_W1_A0FMT(format));
    if(OUT_P16BPP4==screen->face) { // p16bpp4
        xpos = LcdReadBit(inf, DSP_HACT_ST, m_BIT11LO) + xpos/4;
        ypos += LcdReadBit(inf, DSP_VACT_ST, m_BIT11LO);
        addr = inf->smem_start2 + offset/4;
        LcdMskReg(inf, WIN1_VIR0, m_WORDLO, v_WORDLO(var->xres_virtual/4));
        LcdWrReg(inf, WIN1_AREA0_MST, addr);
        LcdMskReg(inf, DSP_WIN1_AREA0_ST, m_BIT11LO|m_BIT11HI, v_BIT11LO(xpos)|v_BIT11HI(ypos));
        LcdMskReg(inf, DSP_WIN1_AREA0_INFO, m_BIT11LO|m_BIT11HI, v_BIT11LO(xres/4)|v_BIT11HI(yres));
    } else {
        xpos += LcdReadBit(inf, DSP_HACT_ST, m_BIT11LO);
        ypos += LcdReadBit(inf, DSP_VACT_ST, m_BIT11LO);
        LcdMskReg(inf, WIN1_VIR0, m_WORDLO, v_WORDLO(xres_virtual));
        LcdWrReg(inf, WIN1_AREA0_MST, addr);
        LcdMskReg(inf, DSP_WIN1_AREA0_ST, m_BIT11LO|m_BIT11HI, v_BIT11LO(xpos)|v_BIT11HI(ypos));
        LcdMskReg(inf, DSP_WIN1_AREA0_INFO, m_BIT11LO|m_BIT11HI, v_BIT11LO(xres)|v_BIT11HI(yres));
    }
    LcdMskReg(inf, DSP_CTRL_REG0, m_W1A0_TRSPREG|m_W1A0_TRSPVAL,
        v_W1A0_TRSPREG((TRSP_FMREG==trspmode)||(TRSP_MASK==trspmode))|v_W1A0_TRSPVAL(trspval));
    LcdMskReg(inf, DSP_CTRL_REG1, m_W1A0_TRSPRAM,
        v_W1A0_TRSPRAM((TRSP_FMRAM==trspmode)||(TRSP_MASK==trspmode)));
    if(1==format) {
        LcdMskReg(inf, WIN1_WATERMARK, m_W1A0_8SWAP | m_W1A0_16SWAP | m_W1A0_LOOPSWAP | m_W1A0_RBSWAP,
            v_W1A0_8SWAP(1) | v_W1A0_16SWAP(1) | v_W1A0_LOOPSWAP(1) | v_W1A0_RBSWAP(0) );
    } else {
        LcdMskReg(inf, WIN1_WATERMARK, m_W1A0_8SWAP | m_W1A0_16SWAP | m_W1A0_LOOPSWAP | m_W1A0_RBSWAP,
            v_W1A0_8SWAP(0) | v_W1A0_16SWAP(0) | v_W1A0_LOOPSWAP(0) | v_W1A0_RBSWAP(1) );
    }

	LcdWrReg(inf, REG_CFG_DONE, 0x01);

    if(SCREEN_MCU==inf->cur_screen->type) {
        if(OUT_P16BPP4==inf->cur_screen->face) {
            rgb565_to_gray(inf->screen_base2, inf->win1fb->screen_base, inf->win1fb->fix.smem_len);
        }
#if LANDSCAPE_USE_ROTATE
        if(IsMcuLandscape()) {
        #if WIN1_USE_DOUBLE_BUF
            rgb565_rotate((uint16_t*)inf->screen_base2, (uint16_t*)(((uint32_t)inf->win1fb->screen_base)+offset),
                inf->cur_screen->x_res, inf->cur_screen->y_res);
        #else
            rgb565_rotate((uint16_t*)inf->screen_base2, (uint16_t*)inf->win1fb->screen_base,
                inf->cur_screen->x_res, inf->cur_screen->y_res);
        #endif
        }
#endif
    }

    return 0;
}


static int win1fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct rk28fb_screen *screen = inf->cur_screen;
    struct fb_var_screeninfo *var1 = &info->var;
    struct fb_fix_screeninfo *fix1 = &info->fix;
    struct win1_par *par = info->par;
    u32 offset = 0, addr = 0;

	//fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

	CHK_SUSPEND(inf);

    switch(var1->bits_per_pixel)
    {
    case 16:    // rgb565
        var->xoffset = (var->xoffset) & (~0x1);
        offset = (var->yoffset*var1->xres_virtual + var->xoffset)*2;
        break;
    case 32:    // rgb888
        offset = (var->yoffset*var1->xres_virtual + var->xoffset)*4;
        break;
    default:
        return -EINVAL;
    }

    if(OUT_P16BPP4==screen->face)  // p16bpp4
        addr = inf->smem_start2 + offset/4;
#if LANDSCAPE_USE_ROTATE
    else if(IsMcuLandscape())
    #if WIN1_USE_DOUBLE_BUF
        addr = inf->smem_start2;
    #else
        addr = inf->smem_start2 + offset;
    #endif
#endif
    else
        addr = fix1->smem_start + offset;

    LcdWrReg(inf, WIN1_AREA0_MST, addr);
    LcdWrReg(inf, REG_CFG_DONE, 0x01);

    if( inf->cur_screen->mcu_usefmk && IsMcuLandscape() ) {
        par->fmktmp.completed = 0;
        if(0==par->fmktmp.addr[0] && 0==par->fmktmp.addr2_offset) {    //right
            par->fmktmp.addr[1] = addr;
        } else if(0==par->fmktmp.addr[1] && 0==par->fmktmp.addr2_offset) { //left
            par->fmktmp.addr[0] = addr;
        } else {    //middle
            par->fmktmp.addr[0] = addr;
        }
        par->fmktmp.completed = 1;
    }

	mcu_refresh(inf);

    // flush end when wq_condition=1 in mcu panel, but not in rgb panel
    if(SCREEN_MCU == inf->cur_screen->type) {
        wait_event_interruptible_timeout(wq, wq_condition, HZ/20);
        wq_condition = 0;
    } else {
        wq_condition = 0;
        wait_event_interruptible_timeout(wq, wq_condition, HZ/20);
    }

    return 0;
}


int win1fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct rk28fb_inf *inf = info->device->driver_data;
    struct rk28fb_mach_info *mach_info = info->device->platform_data;

	//fbprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);

	CHK_SUSPEND(inf);

    switch(cmd)
    {
    case FB0_IOCTL_STOP_TIMER_FLUSH:    //stop timer flush mcu panel after android is runing
        if(1==arg) {
            inf->mcu_usetimer = 0;
        }
        break;

    case FB0_IOCTL_SET_PANEL:
        if(arg>7)   return -1;

        /* Black out, because some display device need clock to standby */
		LcdMskReg(inf, DSP_CTRL_REG1, m_BLANK_OUT, v_BLANK_OUT(1));
		#if 0
        LcdMskReg(inf, SYS_CONFIG, m_W0_ENC, v_W0_ENC(0));
        LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC, v_W1_A0ENC(0));
        LcdMskReg(inf, DSP_CTRL_REG1, m_BLACK_RGB, v_BLACK_RGB(0x0000ff));
		#endif
        LcdWrReg(inf, REG_CFG_DONE, 0x01);
        if(inf->cur_screen)
        {
			if(inf->cur_screen->standby)	inf->cur_screen->standby(1);
		}

        /* Load the new device's param */
        switch(arg)
        {
        case 0: inf->cur_screen = &inf->lcd_info;   break;  //lcd
        case 1: inf->cur_screen = &inf->tv_info[0]; break;  //tv ntsc cvbs
        case 2: inf->cur_screen = &inf->tv_info[1]; break;  //tv pal cvbs
        case 3: inf->cur_screen = &inf->tv_info[2]; break;  //tv 480 ypbpr
        case 4: inf->cur_screen = &inf->tv_info[3]; break;  //tv 576 ypbpr
        case 5: inf->cur_screen = &inf->tv_info[4]; break;  //tv 720 ypbpr
        case 6: inf->cur_screen = &inf->hdmi_info[0];  break;  //hdmi 576
        case 7: inf->cur_screen = &inf->hdmi_info[1];  break;  //hdmi 720
        default: break;
        }

		if(arg != 0){
			win1fb_blank(FB_BLANK_NORMAL, inf->win1fb);
		}else{
			win1fb_blank(FB_BLANK_UNBLANK, inf->win1fb);
		}


		// operate the display_on pin to power down the lcd
		if(SCREEN_RGB==inf->cur_screen->type || SCREEN_MCU==inf->cur_screen->type) {
			if(mach_info && mach_info->gpio && mach_info->gpio->display_on) {
				GPIOSetPinLevel(mach_info->gpio->display_on,
					(0!=arg) ? !inf->cur_screen->pin_dispon : inf->cur_screen->pin_dispon);
				gpio_direction_output(mach_info->gpio->display_on, 0);
			}
		}

        load_screen(info, 1);

		if(inf->cur_screen)
		{
			if(inf->cur_screen->standby)	inf->cur_screen->standby(0);
		}

		// hs/vs/den output enable
		LcdMskReg(inf, DSP_CTRL_REG1, m_BLANK_OUT, v_BLANK_OUT(0));
		LcdWrReg(inf, REG_CFG_DONE, 0x01);

		mcu_refresh(inf);
        break;
    default:
        break;
    }
    return 0;
}

static struct fb_ops win1fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= win1fb_check_var,
	.fb_set_par = win1fb_set_par,
	.fb_blank   = win1fb_blank,
	.fb_pan_display = win1fb_pan_display,
    .fb_ioctl = win1fb_ioctl,
	.fb_setcolreg	= fb_setcolreg,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};


static irqreturn_t rk28fb_irq(int irq, void *dev_id)
{
	struct platform_device *pdev = (struct platform_device*)dev_id;
    struct rk28fb_inf *inf = platform_get_drvdata(pdev);
    if(!inf)
        return IRQ_HANDLED;

	LcdMskReg(inf, INT_LUT, m_FRM_STARTCLEAR, v_FRM_STARTCLEAR(1));

	if(SCREEN_MCU == inf->cur_screen->type)
	{
        inf->mcu_isrcnt = !inf->mcu_isrcnt;
        if(inf->mcu_isrcnt)
            return IRQ_HANDLED;

        if(IsMcuUseFmk())
        {
            if(inf->mcu_needflush) {
                if(FMK_IDLE==inf->mcu_fmkstate || FMK_ENDING==inf->mcu_fmkstate) {
                    inf->mcu_fmkstate = FMK_INT;
                    inf->mcu_needflush = 0;
                    fbprintk2("A ");
                } else {
                    return IRQ_HANDLED;
                }
            } else {
                if(FMK_ENDING==inf->mcu_fmkstate) {
                    if(inf->cur_screen->refresh)
                        inf->cur_screen->refresh(REFRESH_END);
                    inf->mcu_fmkstate = FMK_IDLE;
                } else {
                    return IRQ_HANDLED;
                }
            }
        }
        else
        {
            if(inf->mcu_needflush) {
                if(inf->cur_screen->refresh)
                    inf->cur_screen->refresh(REFRESH_PRE);
                inf->mcu_needflush = 0;
                LcdSetBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDSIGNAL);
            } else {
                if(inf->cur_screen->refresh)
                    inf->cur_screen->refresh(REFRESH_END);
            }
        }
	}

	wq_condition = 1;
 	wake_up_interruptible(&wq);

	return IRQ_HANDLED;
}


static irqreturn_t rk28fb_irqfmk(int irq, void *dev_id)
{
	struct platform_device *pdev = (struct platform_device*)dev_id;
    struct rk28fb_inf *inf = platform_get_drvdata(pdev);
    struct rk28fb_screen *screen;
    struct win0_par *w0par;
    struct win1_par *w1par;
    uint16 hact_st = 0;
    static uint8 leap_cnt = 0;
    uint16 tmp = 1;

    if(!inf)    return IRQ_HANDLED;

    screen = inf->cur_screen;
    w0par = inf->win0fb->par;
    w1par = inf->win1fb->par;

    if(0==screen->mcu_usefmk) {
        inf->mcu_fmkstate = FMK_IDLE;
        return IRQ_HANDLED;
    }
	
	

    hact_st = LcdReadBit(inf, DSP_HACT_ST, m_BIT11LO);

    switch(inf->mcu_fmkstate)
    {
    case FMK_INT: // 开定时器(FMK到)
        if(0==irq)    return IRQ_HANDLED;
        if(screen->mcu_usefmk && IsMcuLandscape())
        {
            if(leap_cnt)   { leap_cnt--; return IRQ_HANDLED; }    //竖屏转横屏丢掉2个中断以同步
            if(w0par->fmktmp.completed)   { w0par->fmk = w0par->fmktmp;  w0par->fmktmp.completed = 0; }
            if(w1par->fmktmp.completed)   { w1par->fmk = w1par->fmktmp;  w1par->fmktmp.completed = 0; }
            inf->mcu_fmkstate = FMK_PRELEFT;
        } else if ( (2==screen->mcu_usefmk) && (!IsMcuLandscape()) ) {
            leap_cnt = 2;
            inf->mcu_fmkstate = FMK_PREUP;
        } else {
            leap_cnt = 0;
            inf->mcu_fmkstate = FMK_IDLE;
            break;
        }
#if FMK_USE_HARDTIMER
        rockchip_usertimer_start(500000/screen->mcu_frmrate, rk28fb_dohardtimer);
#else
        hrtimer_start(&inf->htimer,ktime_set(0,(450000000/screen->mcu_frmrate)),HRTIMER_MODE_REL);
#endif
        break;

    case FMK_PRELEFT: // 送左半屏(定时器到)
        if(0!=irq)      return IRQ_HANDLED;
        if(!LcdReadBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDED)) {
            inf->mcu_fmkstate = FMK_IDLE;
            printk("L failed! \n");
            return IRQ_HANDLED;
        }
        if (screen->disparea)    screen->disparea(0);

        if (w0par->fmk.enable && w0par->fmk.addr_y[0]) {    // Win0
            LcdMskReg(inf, SYS_CONFIG, m_W0_ENC|m_W0_INFMT, v_W0_ENC(1)|v_W0_INFMT(w0par->fmk.format));
            LcdWrReg(inf, WIN0_YRGB_MST, w0par->fmk.addr_y[0]);
            LcdWrReg(inf, WIN0_CBR_MST, w0par->fmk.addr_uv[0]);
            LcdMskReg(inf, WIN0_ACT_INFO, m_WORDLO, v_WORDLO(w0par->fmk.act_w[0]));
            LcdMskReg(inf, DSP_WIN0_ST, m_BIT11LO, v_BIT11LO(w0par->fmk.win_stx[0]+hact_st));
            LcdMskReg(inf, DSP_WIN0_INFO, m_BIT11LO,  v_BIT11LO(w0par->fmk.win_w[0]));
            w0par->fmk.addr_y[1] = w0par->fmk.addr_y[0] + w0par->fmk.addr_y2offset;
            w0par->fmk.addr_uv[1] = w0par->fmk.addr_uv[0] + w0par->fmk.addr_uv2offset;
        } else {
            LcdMskReg(inf, SYS_CONFIG, m_W0_ENC,  v_W0_ENC(0));
        }

        if (w1par->fmk.enable && w1par->fmk.addr[0])   // Win1
        {
            LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC, v_W1_A0ENC(1));
            LcdWrReg(inf, WIN1_AREA0_MST, w1par->fmk.addr[0]);
            LcdMskReg(inf, DSP_WIN1_AREA0_ST, m_BIT11LO, v_BIT11LO(w1par->fmk.win_stx[0]+hact_st));
            LcdMskReg(inf, DSP_WIN1_AREA0_INFO, m_BIT11LO, v_BIT11LO(w1par->fmk.act_w[0]));
            w1par->fmk.addr[1] = w1par->fmk.addr[0] + w1par->fmk.addr2_offset;
        } else {
            LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC, v_W1_A0ENC(0));
        }

        LcdSetBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDSIGNAL);
        LcdWrReg(inf, REG_CFG_DONE, 0x01);
        inf->mcu_fmkstate = FMK_LEFT;
        fbprintk2("L ");
        break;

    case FMK_LEFT: // 送右半屏(FMK到)
        if(0==irq)    return IRQ_HANDLED;
        while(!LcdReadBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDED)) {
            udelay(25);
            if(tmp)  printk("X ");
            tmp = 0;
        }
        if(screen->disparea)    screen->disparea(1);

        if (w0par->fmk.enable && w0par->fmk.addr_y[1]) // win0
        {
            LcdMskReg(inf, SYS_CONFIG, m_W0_ENC|m_W0_INFMT,  v_W0_ENC(1)|v_W0_INFMT(w0par->fmk.format));
            LcdWrReg(inf, WIN0_YRGB_MST, w0par->fmk.addr_y[1]);
            LcdWrReg(inf, WIN0_CBR_MST, w0par->fmk.addr_uv[1]);
            LcdMskReg(inf, WIN0_ACT_INFO, m_WORDLO, v_WORDLO(w0par->fmk.act_w[1]));
            LcdMskReg(inf, DSP_WIN0_ST, m_BIT11LO, v_BIT11LO(w0par->fmk.win_stx[1]+hact_st));
            LcdMskReg(inf, DSP_WIN0_INFO, m_BIT11LO, v_BIT11LO(w0par->fmk.win_w[1]));
        } else {
            LcdMskReg(inf, SYS_CONFIG, m_W0_ENC, v_W0_ENC(0));
        }

        if (w1par->fmk.enable && w1par->fmk.addr[1])   // win1
        {
            LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC, v_W1_A0ENC(1));
            LcdWrReg(inf, WIN1_AREA0_MST, w1par->fmk.addr[1]);
            LcdMskReg(inf, DSP_WIN1_AREA0_ST, m_BIT11LO, v_BIT11LO(w1par->fmk.win_stx[1]+hact_st));
            LcdMskReg(inf, DSP_WIN1_AREA0_INFO, m_BIT11LO, v_BIT11LO(w1par->fmk.act_w[1]));
        } else {
            LcdMskReg(inf, SYS_CONFIG, m_W1_A0ENC, v_W1_A0ENC(0));
        }

        inf->mcu_isrcnt = 0;
        inf->mcu_fmkstate = FMK_ENDING;
        LcdSetBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDSIGNAL);
        LcdWrReg(inf, REG_CFG_DONE, 0x01);
        fbprintk2("R ");
        break;

    case FMK_PREUP:     // 送上半屏(定时器到)
        if(0!=irq)      return IRQ_HANDLED;
        if(screen->disparea)    screen->disparea(2);
        inf->mcu_isrcnt = 0;
        inf->mcu_fmkstate = FMK_UP;
        LcdSetBit(inf, MCU_TIMING_CTRL, m_MCU_HOLDSIGNAL);
        LcdWrReg(inf, REG_CFG_DONE, 0x01);
        fbprintk2("U ");
        break;

    case FMK_UP:        // 送下半屏(FMK到)
        if(0==irq)    return IRQ_HANDLED;
        fbprintk2("D ");
        inf->mcu_fmkstate = FMK_ENDING;
        break;

    case FMK_ENDING:
    default:
        inf->mcu_fmkstate = FMK_IDLE;
        break;
    }

	return IRQ_HANDLED;
}


#if FMK_USE_HARDTIMER
int rk28fb_dohardtimer(void)
{
    rk28fb_irqfmk(0, g_pdev);
    return 0;
}
#else
static enum hrtimer_restart rk28fb_dohtimer(struct hrtimer *handle)
{
    rk28fb_irqfmk(0, g_pdev);
    return HRTIMER_NORESTART;
}
#endif


static int __init rk28fb_probe (struct platform_device *pdev)
{
    struct rk28fb_inf *inf = NULL;
    struct resource *res = NULL;
    struct resource *mem = NULL;
    struct rk28fb_mach_info *mach_info = NULL;
    struct rk28fb_screen *screen = NULL;
	int irq = 0;
    int ret = 0;

    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);


    /* Malloc rk28fb_inf and set it to pdev for drvdata */
    fbprintk(">> Malloc rk28fb_inf and set it to pdev for drvdata \n");
    inf = kmalloc(sizeof(struct rk28fb_inf), GFP_KERNEL);
    if(!inf) {
        printk(">> inf kmalloc fail!");
        ret = -ENOMEM;
		goto release_drvdata;
    }
    memset(inf, 0, sizeof(struct rk28fb_inf));
	platform_set_drvdata(pdev, inf);


    /* Fill screen info and set current screen */
    fbprintk(">> Fill screen info and set current screen \n");
    set_lcd_info(&inf->lcd_info);
    set_tv_info(&inf->tv_info[0]);
    set_hdmi_info(&inf->hdmi_info[0]);
    inf->cur_screen = &inf->lcd_info;
    screen = inf->cur_screen;
    if(SCREEN_NULL==screen->type) {
        printk(">> Please select a display device! \n");
        ret = -EINVAL;
		goto release_drvdata;
    }


    /* get virtual basic address of lcdc register */
    fbprintk(">> get virtual basic address of lcdc register \n");
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL) {
        dev_err(&pdev->dev, "failed to get memory registers\n");
        ret = -ENOENT;
		goto release_drvdata;
    }
    inf->reg_phy_base = res->start;
    inf->len = (res->end - res->start) + 1;
    mem = request_mem_region(inf->reg_phy_base, inf->len, pdev->name);
    if (mem == NULL) {
        dev_err(&pdev->dev, "failed to get memory region\n");
        ret = -ENOENT;
		goto release_drvdata;
    }
    inf->reg_vir_base = ioremap(inf->reg_phy_base, inf->len);
    if (inf->reg_vir_base == NULL) {
        dev_err(&pdev->dev, "ioremap() of registers failed\n");
        ret = -ENXIO;
		goto release_drvdata;
    }
    inf->preg = (LCDC_REG*)inf->reg_vir_base;

    /* Prepare win1 info */
    fbprintk(">> Prepare win1 info \n");
   	inf->win1fb = framebuffer_alloc(sizeof(struct win1_par), &pdev->dev);
    if(!inf->win1fb) {
        printk(">> win1fb framebuffer_alloc fail!");
		inf->win1fb = NULL;
        ret = -ENOMEM;
		goto release_win1fb;
    }

    strcpy(inf->win1fb->fix.id, "win1fb");
    inf->win1fb->fix.type        = FB_TYPE_PACKED_PIXELS;
    inf->win1fb->fix.type_aux    = 0;
    inf->win1fb->fix.xpanstep    = 1;
    inf->win1fb->fix.ypanstep    = 1;
    inf->win1fb->fix.ywrapstep   = 0;
    inf->win1fb->fix.accel       = FB_ACCEL_NONE;
    inf->win1fb->fix.visual      = FB_VISUAL_TRUECOLOR;
    inf->win1fb->fix.smem_len    = 0;
    inf->win1fb->fix.line_length = 0;
    inf->win1fb->fix.smem_start  = 0;

    inf->win1fb->var.xres = screen->x_res;
    inf->win1fb->var.yres = screen->y_res;
    inf->win1fb->var.bits_per_pixel = 16;
    inf->win1fb->var.xres_virtual = screen->x_res;
    inf->win1fb->var.yres_virtual = screen->y_res;
    inf->win1fb->var.width = screen->x_res;
    inf->win1fb->var.height = screen->y_res;
    inf->win1fb->var.pixclock = screen->pixclock;
    inf->win1fb->var.left_margin = screen->left_margin;
    inf->win1fb->var.right_margin = screen->right_margin;
    inf->win1fb->var.upper_margin = screen->upper_margin;
    inf->win1fb->var.lower_margin = screen->lower_margin;
    inf->win1fb->var.vsync_len = screen->vsync_len;
    inf->win1fb->var.hsync_len = screen->hsync_len;
    inf->win1fb->var.red    = def_rgb_16.red;
    inf->win1fb->var.green  = def_rgb_16.green;
    inf->win1fb->var.blue   = def_rgb_16.blue;
    inf->win1fb->var.transp = def_rgb_16.transp;

    inf->win1fb->var.nonstd      = 0;  //win1 format & ypos & xpos (ypos<<20 + xpos<<8 + format)
    inf->win1fb->var.grayscale   = 0;  //win1 transprent mode & value(mode<<8 + value)
    inf->win1fb->var.activate    = FB_ACTIVATE_NOW;
    inf->win1fb->var.accel_flags = 0;
    inf->win1fb->var.vmode       = FB_VMODE_NONINTERLACED;

    inf->win1fb->fbops           = &win1fb_ops;
    inf->win1fb->flags           = FBINFO_FLAG_DEFAULT;
    inf->win1fb->pseudo_palette  = ((struct win1_par*)inf->win1fb->par)->pseudo_pal;
    inf->win1fb->screen_base     = 0;

    memset(inf->win1fb->par, 0, sizeof(struct win1_par));


    /* Prepare win0 info */
    fbprintk(">> Prepare win0 info \n");
    inf->win0fb = framebuffer_alloc(sizeof(struct win0_par), &pdev->dev);
    if(!inf->win0fb) {
        printk(">> win0fb framebuffer_alloc fail!");
		inf->win0fb = NULL;
		ret = -ENOMEM;
		goto release_win0fb;
    }

    strcpy(inf->win0fb->fix.id, "win0fb");
	inf->win0fb->fix.type	      = FB_TYPE_PACKED_PIXELS;
	inf->win0fb->fix.type_aux    = 0;
	inf->win0fb->fix.xpanstep    = 1;
	inf->win0fb->fix.ypanstep    = 1;
	inf->win0fb->fix.ywrapstep   = 0;
	inf->win0fb->fix.accel       = FB_ACCEL_NONE;
    inf->win0fb->fix.visual      = FB_VISUAL_TRUECOLOR;
    inf->win0fb->fix.smem_len    = 0;
    inf->win0fb->fix.line_length = 0;
    inf->win0fb->fix.smem_start  = 0;

    inf->win0fb->var.xres = screen->x_res;
    inf->win0fb->var.yres = screen->y_res;
    inf->win0fb->var.bits_per_pixel = 16;
    inf->win0fb->var.xres_virtual = screen->x_res;
    inf->win0fb->var.yres_virtual = screen->y_res;
    inf->win0fb->var.width = screen->x_res;
    inf->win0fb->var.height = screen->y_res;
    inf->win0fb->var.pixclock = screen->pixclock;
    inf->win0fb->var.left_margin = screen->left_margin;
    inf->win0fb->var.right_margin = screen->right_margin;
    inf->win0fb->var.upper_margin = screen->upper_margin;
    inf->win0fb->var.lower_margin = screen->lower_margin;
    inf->win0fb->var.vsync_len = screen->vsync_len;
    inf->win0fb->var.hsync_len = screen->hsync_len;
    inf->win0fb->var.red    = def_rgb_16.red;
    inf->win0fb->var.green  = def_rgb_16.green;
    inf->win0fb->var.blue   = def_rgb_16.blue;
    inf->win0fb->var.transp = def_rgb_16.transp;

    inf->win0fb->var.nonstd      = 0;  //win0 format & ypos & xpos (ypos<<20 + xpos<<8 + format)
    inf->win0fb->var.grayscale   = ((inf->win0fb->var.yres<<20)&0xfff00000) + ((inf->win0fb->var.xres<<8)&0xfff00);//win0 xsize & ysize
    inf->win0fb->var.activate    = FB_ACTIVATE_NOW;
    inf->win0fb->var.accel_flags = 0;
    inf->win0fb->var.vmode       = FB_VMODE_NONINTERLACED;

    inf->win0fb->fbops           = &win0fb_ops;
	inf->win0fb->flags		      = FBINFO_FLAG_DEFAULT;
	inf->win0fb->pseudo_palette  = ((struct win0_par*)inf->win0fb->par)->pseudo_pal;
	inf->win0fb->screen_base     = 0;

    memset(inf->win0fb->par, 0, sizeof(struct win0_par));


 	/* Init all lcdc and lcd before register_framebuffer. */
 	/* because after register_framebuffer, the win1fb_check_par and winfb_set_par execute immediately */
 	fbprintk(">> Init all lcdc and lcd before register_framebuffer \n");
    init_lcdc(inf->win1fb);
	rockchip_scu_register(SCU_IPID_LCDC, SCU_MODE_SETFREQ, 24, NULL);
	mach_info = pdev->dev.platform_data;
	if(mach_info) {
        if( OUT_P888==inf->lcd_info.face ||
            OUT_P888==inf->tv_info[0].face ||
            OUT_P888==inf->hdmi_info[0].face )     // set lcdc iomux
        {
            rockchip_mux_api_set(mach_info->iomux->data24, 1);
        } else {
            rockchip_mux_api_set(mach_info->iomux->data18, 1);
        }
        rockchip_mux_api_set(mach_info->iomux->den, 1);
        rockchip_mux_api_set(mach_info->iomux->vsync, 1);
    }
	set_lcd_pin(pdev, 1);
	mdelay(10);
	g_pdev = pdev;
	inf->mcu_usetimer = 1;
	load_screen(inf->win1fb, 1);


    /* Register framebuffer(win1fb & win0fb) */
    fbprintk(">> Register framebuffer(win1fb & win0fb) \n");
    ret = register_framebuffer(inf->win1fb);
    if(ret<0) {
        printk(">> win1fb register_framebuffer fail!\n");
        ret = -EINVAL;
		goto release_win0fb;
    }

    ret = register_framebuffer(inf->win0fb);
    if(ret<0) {
        printk(">> win0fb register_framebuffer fail!\n");
        ret = -EINVAL;
		goto unregister_win1fb;
    }


#ifdef CONFIG_ANDROID_POWER
    inf->early_suspend.suspend = rk28fb_early_suspend;
    inf->early_suspend.resume = rk28fb_early_resume;
    inf->early_suspend.level= ANDROID_EARLY_SUSPEND_LEVEL_DISABLE_FB;
    android_register_early_suspend(&inf->early_suspend);
#endif

    /* get and request irq */
    fbprintk(">> get and request irq \n");
    irq = platform_get_irq(pdev, 0);
    if (irq < 0) {
        dev_err(&pdev->dev, "no irq for device\n");
        ret = -ENOENT;
        goto release_irq;
    }
    ret = request_irq(irq, rk28fb_irq, IRQF_DISABLED, pdev->name, pdev);
    if (ret) {
        dev_err(&pdev->dev, "cannot get irq %d - err %d\n", irq, ret);
        ret = -EBUSY;
        goto release_irq;
    }

    ret = request_gpio_irq(GPIOPortE_Pin1, (pFunc)rk28fb_irqfmk, GPIOEdgelFalling, pdev);
    if (ret) {
        ret = -EBUSY;
        goto release_irq;
    }

#if (0==FMK_USE_HARDTIMER)
    hrtimer_init(&inf->htimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	inf->htimer.function = rk28fb_dohtimer;
#endif

    return ret;


release_irq:
	if(irq>=0)
    	free_irq(irq, pdev);
unregister_win1fb:
    unregister_framebuffer(inf->win1fb);
release_win0fb:
	if(inf->win0fb)
		framebuffer_release(inf->win0fb);
	inf->win0fb = NULL;
release_win1fb:
	if(inf->win1fb)
		framebuffer_release(inf->win1fb);
	inf->win1fb = NULL;
release_drvdata:
	if(inf && inf->reg_vir_base)
    	iounmap(inf->reg_vir_base);
	if(inf && mem)
    	release_mem_region(inf->reg_phy_base, inf->len);
	if(inf)
    	kfree(inf);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int rk28fb_remove(struct platform_device *pdev)
{
    struct rk28fb_inf *inf = platform_get_drvdata(pdev);
    struct fb_info *info = NULL;
	pm_message_t msg;

	fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);

    if(!inf) {
        printk("inf==0, rk28fb_remove fail! \n");
        return -EINVAL;
    }

#ifdef CONFIG_ANDROID_POWER
    android_unregister_early_suspend(&inf->early_suspend);
#endif

	set_lcd_pin(pdev, 0);

    // blank the lcdc
    if(inf->win0fb)
        win0fb_blank(FB_BLANK_POWERDOWN, inf->win0fb);
    if(inf->win1fb)
        win1fb_blank(FB_BLANK_POWERDOWN, inf->win1fb);

	// suspend the lcdc
	rk28fb_suspend(pdev, msg);

    // unmap memory and release framebuffer
    if(inf->win0fb) {
        info = inf->win0fb;
        if (info->screen_base) {
	        dma_free_writecombine(NULL, PAGE_ALIGN(info->fix.smem_len),info->screen_base, info->fix.smem_start);
	        info->screen_base = 0;
	        info->fix.smem_start = 0;
	        info->fix.smem_len = 0;
        }
        unregister_framebuffer(inf->win0fb);
        framebuffer_release(inf->win0fb);
        inf->win0fb = NULL;
    }
    if(inf->win1fb) {
        info = inf->win1fb;
        if (info->screen_base) {
	        dma_free_writecombine(NULL, PAGE_ALIGN(info->fix.smem_len),info->screen_base, info->fix.smem_start);
	        info->screen_base = 0;
	        info->fix.smem_start = 0;
	        info->fix.smem_len = 0;
        }
        unregister_framebuffer(inf->win1fb);
        framebuffer_release(inf->win1fb);
        inf->win1fb = NULL;
    }

    kfree(inf);
    platform_set_drvdata(pdev, NULL);

    return 0;
}

static int rk28fb_suspend(struct platform_device *pdev, pm_message_t msg)
{
    struct rk28fb_inf *inf = platform_get_drvdata(pdev);

    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);

    if(!inf) {
        printk("inf==0, rk28fb_suspend fail! \n");
        return -EINVAL;
    }

	if(inf->cur_screen->standby)
	{
		fbprintk(">>>>>> power down the screen! \n");
		inf->cur_screen->standby(1);
	}

    LcdMskReg(inf, DSP_CTRL_REG1, m_BLANK_OUT, v_BLANK_OUT(1));
    LcdMskReg(inf, WIN0_VIR, m_TRISTATE_OUT, v_TRISTATE_OUT(1));
    LcdMskReg(inf, SYS_CONFIG, m_W0_MASTER | m_W1_MASTER, v_W0_MASTER(0) | v_W1_MASTER(0) );
  	LcdWrReg(inf, REG_CFG_DONE, 0x01);

	if(!inf->in_suspend)
	{
		fbprintk(">>>>>> diable the lcdc clk! \n");
		msleep(100);
    		rockchip_scu_disableclk(SCU_IPID_LCDC);
		inf->in_suspend = 1;
	}

	set_lcd_pin(pdev, 0);

	return 0;
}


static int rk28fb_resume(struct platform_device *pdev)
{
    struct rk28fb_inf *inf = platform_get_drvdata(pdev);
    struct rk28fb_screen *screen = inf->cur_screen;

    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);

    if(!inf) {
        printk("inf==0, rk28fb_resume fail! \n");
        return -EINVAL;
    }

	set_lcd_pin(pdev, 1);

	if(inf->in_suspend)
	{
	    inf->in_suspend = 0;
    	fbprintk(">>>>>> eable the lcdc clk! \n");
        rockchip_scu_enableclk(SCU_IPID_LCDC);
        __rockchip_clk_set_unit_clock(SCU_IPID_LCDC , screen->pixclock);
        msleep(100);
	}

	LcdMskReg(inf, SYS_CONFIG, m_W0_MASTER | m_W1_MASTER, v_W0_MASTER(1) | v_W1_MASTER(1) );
	LcdMskReg(inf, DSP_CTRL_REG1, m_BLANK_OUT, v_BLANK_OUT(0));
	LcdMskReg(inf, WIN0_VIR, m_TRISTATE_OUT, v_TRISTATE_OUT(0));
	LcdWrReg(inf, REG_CFG_DONE, 0x01);

	if(inf->cur_screen->standby)
	{
		fbprintk(">>>>>> power on the screen! \n");
		inf->cur_screen->standby(0);
	}
    msleep(100);

	return 0;
}


#ifdef CONFIG_ANDROID_POWER
static void rk28fb_early_suspend(android_early_suspend_t *h)
{
	pm_message_t msg;

    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);

    if(g_pdev) {
		rk28fb_suspend(g_pdev, msg);
    } else {
        printk("g_pdev==0, rk28fb_early_suspend fail! \n");
        return;
	}
}
static void rk28fb_early_resume(android_early_suspend_t *h)
{
    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);

    if(g_pdev) {
		rk28fb_resume(g_pdev);
	} else {
        printk("g_pdev==0, rk28fb_early_resume fail! \n");
        return;
    }
}
#endif
static void rk28fb_shutdown(struct platform_device *pdev)
{
       mdelay(300);
	//printk("----------------------------rk28fb_shutdown----------------------------\n");
  	set_lcd_pin(pdev, 0);
    	rockchip_scu_disableclk(SCU_IPID_LCDC);
}

static struct platform_driver rk28fb_driver = {
	.probe		= rk28fb_probe,
	.remove		= rk28fb_remove,
	.driver		= {
		.name	= "rk28-lcdc",
		.owner	= THIS_MODULE,
	},
	.shutdown=rk28fb_shutdown,
};

static int __init rk28fb_init(void)
{
    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
    return platform_driver_register(&rk28fb_driver);
}

static void __exit rk28fb_exit(void)
{
    fbprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
    platform_driver_unregister(&rk28fb_driver);
}

subsys_initcall(rk28fb_init);

//module_init(rk28fb_init);
module_exit(rk28fb_exit);

MODULE_LICENSE("GPL");

