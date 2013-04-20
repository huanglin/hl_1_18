#include <linux/fb.h>
#include <linux/delay.h>
//#include <asm/arch/lcdcon.h>
#include <asm/arch/rk28_i2c.h>
#include <asm/arch/rk2818_fb.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include "screen.h"
#include <asm/arch/typedef.h>

#include "lcd_epd_ctrl.h"
#include "epd/EPD.h"
#include "epd/eink_s.h"
#include <asm/arch/rk28_debug.h>
#include "epd/eink_power.h"
//#define EPD_DBG

#ifdef EPD_DBG
#define epdprintk(fmt, args...)	printk(KERN_INFO "[EPD] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define epdprintk(fmt, args...)
#endif



/* Base */
#define OUT_TYPE		SCREEN_MCU
#define OUT_FACE		OUT_P24BPP4

#define OUT_CLK         64

/* Timing */
#define H_PW			1
#define H_BP		    1

#define H_FP			1

#define V_PW			1
#define V_BP			1

#define V_FP			1

#define USE_FMARK       0               //是否使用FMK (0:不支持 1:横屏支持 2:横竖屏都支持)
#define FRMRATE         60              //MCU屏的刷新率 (FMK有效时用)
//line_sync+line_begin+line_end+301必须为8的倍数
#ifdef CONFIG_V110_EINK_1024X600
#define PANELW          1024
#define PANELH          600
#define P_WR            68//99
#define LINE_SYNC       4
#define LINE_BEGIN      5
#define LINE_END       26
#define FRAME_SYNC       2
#define FRAME_END       30
#elif defined(CONFIG_V220_EINK_800X600)
#define PANELW          800
#define PANELH          600
#define P_WR            50//55
#define LINE_SYNC       4
#define LINE_BEGIN      5
#define LINE_END        18
#define FRAME_SYNC      2
#define FRAME_END       30
#elif defined(CONFIG_V220_EINK_480X800)
#define PANELW          480
#define PANELH          800
#define P_WR            33//55
#define LINE_SYNC       12
#define LINE_BEGIN      21
#define LINE_END        26
#define FRAME_SYNC      2
#define FRAME_END       60
#elif defined(CONFIG_V220_EINK_1024X768)
#define PANELW          1024
#define PANELH          768
#define P_WR            33
#define LINE_SYNC       4
#define LINE_BEGIN      5
#define LINE_END        10
#define FRAME_SYNC      2
#define FRAME_END     50
#elif defined(CONFIG_V110_EINK_800X600)
#define PANELW          800
#define PANELH          600
#define P_WR            68
#define LINE_SYNC     4/// 4
#define LINE_BEGIN     6// 5
#define LINE_END       25//26
#define FRAME_SYNC       2
#define FRAME_END       30
#elif defined(CONFIG_V110_EINK_1024X768)
#define PANELW          1024
#define PANELH          768
#define P_WR            65
#define LINE_SYNC       4
#define LINE_BEGIN      5
#define LINE_END       26
#define FRAME_SYNC       2
#define FRAME_END      20
#elif defined(CONFIG_V110_1200x825_EINK)
#define PANELW         1216
#define PANELH          824
#define P_WR            	55
#define LINE_SYNC       4
#define LINE_BEGIN      5
#define LINE_END        10//26
#define FRAME_SYNC       2
#define FRAME_END      10
#endif

/* Other */
#define DCLK_POL		0
#define SWAP_RB			1


#define  EBOOK_SHR           				GPIOPortC_Pin5
#define  EBOOK_GDRL          				GPIOPortC_Pin4


#if defined(CONFIG_EPD_ROTMODE_0)
u8 Rk_EPD_CTRL_Rotate = EPD_ROTATE_0;
#elif defined CONFIG_EPD_ROTMODE_90
u8 Rk_EPD_CTRL_Rotate = EPD_ROTATE_90;
#elif defined CONFIG_EPD_ROTMODE_180
u8 Rk_EPD_CTRL_Rotate = EPD_ROTATE_180;
#elif defined CONFIG_EPD_ROTMODE_270
u8 Rk_EPD_CTRL_Rotate = EPD_ROTATE_270;
#endif



/*---------EPD 接口信息--------*/
struct EPD_INFO Rk_Epd_Info;

/*--------屏的边框颜色---------*/
u8 Disp_rim = 15;
/*-----------------------------*/

u8 Rk_EPD_CTRL_Mode;
u8 Rk_EPD_CTRL_Force_Mode_en=0;
//u8 Rk_EPD_CTRL_Force_Mode=0;
u8 Rk_EPD_CTRL_Init_failed = 0;
//u8 Rk_EPD_CTRL_Mode_Flag;     //
//u8 Rk_EPD_CTRL_Mode_Nxt;
u8 Rk_EPD_CTRL_Mode_Alpha_En = 0;

typedef enum _IMAGE_FORMAT
{
	EPD_RGB888 = 0,
	EPD_RGB565,
	EPD_GRAY256,
	EPD_GRAY16_N,
	EPD_GRAY16,
} IMAGE_FORMAT;

void Rk_EPD_CTRL_SetDither(unsigned char status)
{
	Rk_Epd_Info.Epd_setdither(status);
}


void Rk_EPD_CTRL_Buf_Switch(u16 **pBuf1,u16 **pBuf2)
{
	u16	*Display_buff_temp;

	Display_buff_temp = *pBuf1;
	*pBuf1 = *pBuf2;
	*pBuf2 = Display_buff_temp;
}

void Rk_EPD_CTRL_SetRotate(u8 rotate)
{
	Rk_EPD_CTRL_Rotate = rotate;
}

int  Rk_EPD_CTRL_GetRotate(void)
{
	return Rk_EPD_CTRL_Rotate;
}
int Rk_EPD_CTRL_SetWin(u32 *SrcY,u32 SrcW, u32 SrcH, u32 virh,u8 Format)
{
	u8 Rk_EPD_CTRL_Format = 0;
	// u8 mode= Rk_EPD_CTRL_Mode;
	epdprintk("SrcY= 0x%x,SrcY= 0x%x,SrcW =0x%x,SrcH= 0x%x, Format=%d,mode=%d\n", SrcY,virt_to_phys(SrcY), SrcW, SrcH,Format,Rk_EPD_CTRL_Mode);
	if(Rk_EPD_CTRL_Init_failed)
		return 1;
	switch (Format)
	{
		case 0: // rgb565
		Rk_EPD_CTRL_Format = EPD_RGB565;
		break;
		case 1:
			Rk_EPD_CTRL_Format = EPD_RGB888;
			break;
		case 3: //yuv
		Rk_EPD_CTRL_Format = EPD_GRAY16;
		break;
		case 4:
		Rk_EPD_CTRL_Format = EPD_GRAY16_N;
		break;
		case 5://hjk
		Rk_EPD_CTRL_Format = EPD_GRAY256;
		break;
		default:
		return FALSE;
	}
	Rk_Epd_Info.Epd_SendData(SrcY, SrcW, SrcH, virh, Rk_EPD_CTRL_Rotate,
	Rk_EPD_CTRL_Format, Disp_rim, NULL);
	
	return 0;
}
void Rk_EPD_CTRL_SetMode(u8 mode)
{
	epdprintk("mode = %d\n",mode);
	Rk_EPD_CTRL_Mode=Rk_Epd_Info.Epd_Eink_mode_setting(mode);
}

int  Rk_EPD_CTRL_Get_Status()
{
	int status;
	
	status=Rk_Epd_Info.Epd_GetStatus();
	if(status==2)
		return 1;//busy
	else
		return 0;//idle
}
void Rk_EPD_CTRL_ReFlush(uint8 mode)
{
	epdprintk(">>>>>> %s : %s mode = %d\n", __FILE__, __FUNCTION__, mode);
	Rk_Epd_Info.Epd_SendData(0, 0, 0, 0, Rk_EPD_CTRL_Rotate,
                              0, Disp_rim, mode);
}

void Rk_EPD_CTRL_SetAlpha(u8 enable)
{
	epdprintk(">>>>>> %s : %s enable = %d\n", __FILE__, __FUNCTION__, enable);

	Rk_EPD_CTRL_Mode_Alpha_En = (enable!=0)?1:0;
}

int Rk_EPD_CTRL_GetMode(void)
{
	epdprintk(">>>>>> %s Rk_EPD_CTRL_Mode = %d\n", __FUNCTION__, Rk_EPD_CTRL_Mode);

	return Rk_Epd_Info.Epd_get_mode();
}

int Rk_EPD_CTRL_Enable_Force(u8 enable)
{
	Rk_EPD_CTRL_Force_Mode_en = enable;
}

int Rk_EPD_CTRL_Send_End(u8 mode)
{
	static int old_mode=0;
	static int same_mode_times=0;
	epdprintk(">>>>>> %s : %s mode = %d\n", __FILE__, __FUNCTION__, mode);
	if(Rk_EPD_CTRL_Force_Mode_en) {
		if(old_mode == mode) {
			same_mode_times++;
		} else {
			same_mode_times = 0;        
		}
		if((same_mode_times >=5)&&(mode == EPD_PART)) {
			same_mode_times=0;
			old_mode = mode;
			return EPD_FULL;
		}  
	}
	old_mode = mode;

	return -1;
}

int Rk_EPD_CTRL_Pan_Init(void)
{
	epdprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);
	if(Rk_Epd_Info.Epd_Eink_Pan_init)
		Rk_Epd_Info.Epd_Eink_Pan_init();
	return 0;
}

int Rk_EPD_CTRL_Pan_Deinit(void)
{
	epdprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);
	if(Rk_Epd_Info.Epd_Eink_Pan_deinit)
		Rk_Epd_Info.Epd_Eink_Pan_deinit();
	return 0;
}

int Rk_EPD_CTRL_Get_Pan_Buf(u32* bufaddr,u32* bufaddrphy, u32* w, u32* h)
{
	epdprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);
	if(Rk_Epd_Info.Epd_Get_Draw_Buf)
		return Rk_Epd_Info.Epd_Get_Draw_Buf(bufaddr,bufaddrphy,w, h);
	else 
		return 0;
}

int Rk_EPD_CTRL_Alpha_Init(u32 bufaddr)
{
	epdprintk(">>>>>> %s : %s \n", __FILE__, __FUNCTION__);
	if(Rk_Epd_Info.Epd_Eink_Alpha_init)
		Rk_Epd_Info.Epd_Eink_Alpha_init(bufaddr);
	return 0;
}

int Rk_EPD_CTRL_Reset(void)
{
	Rk_Epd_Info.Epd_Reset();

	return TRUE;
}
int Rk_EPD_CTRL_Init(void)
{
	pAPB_REG  pAPBRegBase = (pAPB_REG)REG_FILE_BASE_ADDR_VA;
	pAPBRegBase->CPU_APB_REG4 |= 0x248;
	rockchip_mux_api_set(GPIOC_LCDC18BIT_SEL_NAME, 1);
	rockchip_mux_api_set(GPIOC_LCDC24BIT_SEL_NAME, 0);
	GPIOSetPinDirection(EBOOK_SHR, GPIO_OUT);
	GPIOSetPinLevel(EBOOK_SHR,GPIO_LOW);
	GPIOSetPinDirection(EBOOK_GDRL, GPIO_OUT);
	GPIOSetPinLevel(EBOOK_GDRL,GPIO_LOW);
	Eink_S_power_init();
	Rk_EPD_CTRL_Mode = EPD_AUTO;
	if(Rk_Epd_Info.Epd_Init(PANELW, PANELH)) {
		Rk_EPD_CTRL_Init_failed =1;
	}
	return 0;
}
int Rk_EPD_Wave_num(char *buf)
{
	if(Rk_Epd_Info.Get_waveform_num(buf))
		return -1;
	else
		return 0;
}
void RK_EPD_SET_Idle_Time(int time_count)
{
	Rk_Epd_Info.Set_eink_idle_time(time_count);
}
int Rk_EPD_CTRL_Deinit(void)
{
	if(Rk_Epd_Info.Epd_Deinit && Rk_EPD_CTRL_Init_failed !=1)
		Rk_Epd_Info.Epd_Deinit();
	return 0;
}

int Rk_EPD_CTRL_StandBy(u8 Enable)
{
	epdprintk(">>>>>> %s : %s: %d\n", __FILE__, __FUNCTION__, Enable);
	return Rk_Epd_Info.Epd_Standby(Enable);
}

int Rk_EPD_CTRL_ScanDir(u16 Dir)
{
	return 0;
}
int  Get_Waveform_Mode()
{
	#ifdef CONFIG_WAVEFORM_NAND_ADDR
		return CONFIG_WAVEFORM_NAND_ADDR;
	#else
		return 0;
	#endif
}
struct Epd_timing time_temp ;
int pwr_temp = 0;
int panel_setup_set = 0;
static int __init panel_setup(char *options){
	int num;
	switch(*options){
	case 0x31:
	{//v220 800X600
		panel_setup_set = 1;
		time_temp.epd_w=800;
		time_temp.epd_h=600;
		time_temp.line_sync=4; //最小值为1
		time_temp.line_begin=5;//最小值为0
		time_temp.line_end=18;//最小值1
		time_temp.frame_sync_count=2; //最小值为2
		time_temp.frame_end_count=30;//最小值为1
		pwr_temp = 50;
	}
	break;
	case 0x32:
	{ //v220 480x800
		panel_setup_set = 1;
		time_temp.epd_w=480;
		time_temp.epd_h=800;
		time_temp.line_sync=12; //最小值为1
		time_temp.line_begin=21;//最小值为0
		time_temp.line_end=26;//最小值1
		time_temp.frame_sync_count=2; //最小值为2
		time_temp.frame_end_count=60;//最小值为1
		pwr_temp = 33;
	}
	break;
	case 0x33:
	{ //v220 1024x768
		panel_setup_set = 1;
		time_temp.epd_w=1024;
		time_temp.epd_h=768;
		time_temp.line_sync=4; //最小值为1
		time_temp.line_begin=5;//最小值为0
		time_temp.line_end=10;//最小值1
		time_temp.frame_sync_count=2; //最小值为2
		time_temp.frame_end_count=20;//最小值为1
		pwr_temp = 33;
	}
	break;
	case 0x34:
	{ //v110 800x600
		panel_setup_set = 1;
		time_temp.epd_w=800;
		time_temp.epd_h=600;
		time_temp.line_sync=4; //最小值为1
		time_temp.line_begin=6;//最小值为0
		time_temp.line_end=25;//最小值1
		time_temp.frame_sync_count=2; //最小值为2
		time_temp.frame_end_count=30;//最小值为1
		pwr_temp = 68;
	}
	break;
	case 0x35:
	{ //v110 1024x768
		panel_setup_set = 1;
		time_temp.epd_w=1024;
		time_temp.epd_h=768;
		time_temp.line_sync=4; //最小值为1
		time_temp.line_begin=5;//最小值为0
		time_temp.line_end=26;//最小值1
		time_temp.frame_sync_count=2; //最小值为2
		time_temp.frame_end_count=20;//最小值为1
		pwr_temp = 65;
	}
	break;
	case 0x36:
	{ //v110 1200x825
		panel_setup_set = 1;
		time_temp.epd_w=1216;
		time_temp.epd_h=824;
		time_temp.line_sync=4; //最小值为1
		time_temp.line_begin=5;//最小值为0
		time_temp.line_end=10;//最小值1
		time_temp.frame_sync_count=2; //最小值为2
		time_temp.frame_end_count=10;//最小值为1
		pwr_temp = 55;
	}
	break;
	default:
		panel_setup_set = 0;
		break;
}
}
static int __init panel_rotate_setup(char *options){
	int temp;
	temp = simple_strtol(options, NULL, 10);
	switch(temp){
		case 0:
			Rk_EPD_CTRL_Rotate= 0;
			break;
		case 90:
			Rk_EPD_CTRL_Rotate= 1;
			break;
		case 180:
			Rk_EPD_CTRL_Rotate= 2;
			break;
		case 270:
			Rk_EPD_CTRL_Rotate= 3;
			break;
		default:
			Rk_EPD_CTRL_Rotate= 1;
			break;
			
	}
}
int fb_xres = 0;
int fb_yres = 0;
static int __init fb_xres_setup(char *options){
	fb_xres = simple_strtol(options, NULL, 10);
}
static int __init fb_yres_setup(char *options){
	fb_yres = simple_strtol(options, NULL, 10);
	
}

__setup("panel=", panel_setup);
__setup("panel_rotate=", panel_rotate_setup);
__setup("fb_xres=", fb_xres_setup);
__setup("fb_yres=", fb_yres_setup);

void set_lcd_info(struct rk28fb_screen *screen)
{
	struct Epd_timing time;
	
	Rk_Epd_Info.Epd_CallBack = Rk_EPD_CTRL_Send_End;
	Rk_Epd_Info.Epd_Power_on=Eink_s_power_on;
	Rk_Epd_Info.Epd_Power_down=Eink_s_power_down;
	Rk_Epd_Info.Get_Waveform_Mode = Get_Waveform_Mode;
	EPD_REG(&Rk_Epd_Info);

	if(panel_setup_set){
		time.epd_w=time_temp.epd_w;
		time.epd_h=time_temp.epd_h;;
		time.line_sync=time_temp.line_sync;; //最小值为1
		time.line_begin=time_temp.line_begin;//最小值为0
		time.line_end=time_temp.line_end;//最小值1
		time.frame_sync_count=time_temp.frame_sync_count; //最小值为2
		time.frame_end_count=time_temp.frame_end_count;//最小值为1
	}
	else{
		time.epd_w=PANELW;
		time.epd_h=PANELH;
		time.line_sync=LINE_SYNC; //最小值为1
		time.line_begin=LINE_BEGIN;//最小值为0
		time.line_end=LINE_END;//最小值1
		time.frame_sync_count=FRAME_SYNC; //最小值为2
		time.frame_end_count=FRAME_END;//最小值为1
	}
	if(fb_xres)
		screen->x_res = fb_xres;
	else
		screen->x_res = CONFIG_EINK_SCREEN_X;
	if(fb_yres)
		screen->y_res = fb_yres;
	else
		screen->y_res = CONFIG_EINK_SCREEN_Y;
	
	if(Rk_Epd_Info.Epd_Timing_init)
		Rk_Epd_Info.Epd_Timing_init(&time);
	RK_EPD_SET_Idle_Time(CONFIG_EPD_IDLE_TIME);
	/* screen type & face */
	screen->type = OUT_TYPE;
	screen->face = OUT_FACE;

	/* Screen size */
	

	screen->x_epd_res = time.h_count;
	screen->y_epd_res = time.v_count;

	/* Timing */
	if(panel_setup_set){
		screen->pixclock = 10000/pwr_temp;//10000/P_WR;//10000/P_WR;////8000
		screen->mcu_wrperiod = pwr_temp;
	}
	else{
		screen->pixclock = 10000/P_WR;//10000/P_WR;//10000/P_WR;////8000
		screen->mcu_wrperiod = P_WR;
	}
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;
	screen->mcu_usefmk = USE_FMARK;
	screen->mcu_frmrate = FRMRATE;

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
	screen->init = Rk_EPD_CTRL_Init;
	screen->deinit = Rk_EPD_CTRL_Deinit;
	screen->standby = Rk_EPD_CTRL_StandBy;
	screen->refresh = NULL;
	screen->scandir = Rk_EPD_CTRL_ScanDir;
	screen->setpar = Rk_EPD_CTRL_SetWin;
}
