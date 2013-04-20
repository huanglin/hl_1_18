/*
真实可用的模式有以下几种，其余不可用。
#define  EPD_FULL           0
#define  EPD_PART           2
#define  EPD_BLACK_WHITE    4
#define  EPD_AUTO           5
#define  EPD_GU_FULL        7
#define  EPD_TEXT                   9
#define  EPD_AUTO_PART     10
#define  EPD_AUTO_BLACK_WHITE 11
#define  EPD_A2                    12
PVI屏模式应用：
EPD_FULL:全部刷新，16灰阶。优：残影小；缺：刷屏时间长，闪烁感强，视觉效果差。应用于界面切换，消除残影。
EPD_PART：局部刷新，16灰阶。优：闪烁感弱；缺：刷屏时间长，残影大。应用于文本或图片翻页，刷几次后残影会比较大，需用EPD_FULL刷屏一次消除残影。
EPD_BLACK_WHITE：局部刷新，2灰阶。优：闪烁感弱，刷屏时间快。缺：只有2灰阶。应用于菜单栏选择。	
EPD_AUTO：局部刷新，16灰阶。优：屏的反应速度快，用户操作速度快。缺:有呼吸效应，残影大。应用于菜单栏选择、文本、图片翻页等，对残影要求不高的场景。
EPD_TEXT：局部刷新，16灰阶。优：残影小，闪烁感弱；缺：刷屏时间长。应用于文本或图片翻页。
EPD_A2：局部刷新，2灰阶。优：闪烁感弱，刷屏时间最快。缺：只有2灰阶，只有部份2代屏的WAVEFORM才有该模式。应用于文本或图片连续翻页。

奥翼屏模式应用：
PVI屏模式应用：
EPD_FULL:全部刷新，16灰阶。优：残影小；缺：刷屏时间长，闪烁感强，视觉效果差。应用于界面切换，消除残影。
EPD_PART：局部刷新，16灰阶。优：闪烁感弱；缺：刷屏时间长，残影大。应用于文本或图片翻页，刷几次后残影会比较大，需用EPD_FULL刷屏一次消除残影。
EPD_BLACK_WHITE：局部刷新，2灰阶。优：闪烁感弱，刷屏时间快。缺：只有2灰阶。应用于菜单栏选择。	
EPD_AUTO：局部刷新，16灰阶。优：屏的反应速度快，用户操作速度快。缺:有呼吸效应，残影大。应用于菜单栏选择、文本、图片翻页等，对残影要求不高的场景。
EPD_GU_FULL：局部刷新，16灰阶。优：残影小，闪烁感弱；缺：刷屏时间长。应用于文本或图片翻页。
*/

#define  EPD_ROTATE_0       0
#define  EPD_ROTATE_90      1
#define  EPD_ROTATE_180     2
#define  EPD_ROTATE_270     3

#define  EPD_FULL           0
#define  EPD_FULL_WIN       1
#define  EPD_PART           2
#define  EPD_PART_WIN       3
#define  EPD_BLACK_WHITE    4
#define  EPD_AUTO           5
#define  EPD_DRAW_PEN     6
#define  EPD_GU_FULL        7
#define  EPD_GU_PART        8
#define  EPD_TEXT                   9
#define  EPD_AUTO_PART     10
#define  EPD_AUTO_BLACK_WHITE 11
#define  EPD_A2                    12
#define  EPD_A2_DITHER      13
#define  EPD_FULL_DITHER   14
#define  EPD_DISP_WHITE     15
#define  EPD_AUTO_A2        16

typedef struct Epd_timing
{
	u16  epd_w;
    u16  epd_h;
	u8    line_sync;
	u8    line_begin;
	u8    line_end;
	u8    frame_sync_count;
	u8    frame_end_count;
	u16  h_count;
	u16  v_count;
}*p_time;


struct EPD_INFO
{
int   (*Epd_CallBack)(u8 mode);
void   (*Epd_Power_on)(void);
void   (*Epd_Power_down)(void);
int   (*Epd_Init)(u16 PanelW, u16 PanelH);
void   (*Epd_Deinit)(void);
void   (*Epd_Reset)(void);
u8  (*Epd_GetStatus)(void);
void   (*Epd_Next_Frame)(void);
u32 (*Epd_Standby)(u32 Enable);
void   (*Epd_SendData)(u32 *src,int w,int h, int virh,u8 rotate,u8 DataFormat,u8 rim,u8 mode);
u32 (*Epd_Rgb888ToGray)(u16 *graynew, u16 *grayold, u32 *rgb,int w,int h,u8 rotate,u8 Mode);
u32 (*Epd_Gray8bitToGray4bit)(u16 *graynew, u16 *grayold,char *gray8bit, int w,int h,u8 rotate,u8 Mode);
u32 (*Epd_Rgb565ToGray)(u16 *graynew, u16 *grayold, u16 *rgb,int w,int h,u8 rotate,u8 Mode);
void   (*Epd_Data_Init)(void);
void   (*Epd_SetRotate)(u8 rotate);
void   (*Epd_Timing_init)(p_time time);
void (*Epd_Eink_Pan_init)(void);
void (*Epd_Eink_Pan_deinit)(void);
u32 (*Epd_Get_Draw_Buf)(u32* bufaddr,u32* bufaddrphy, u32* w, u32* h);
u32 (*Epd_Eink_Alpha_init)(u32 bufaddr);
u32 (*Epd_Eink_mode_setting)(u32 mode);
int (*Get_waveform_num)(char * buf);
void (*Set_eink_idle_time)(int time_count);
void (*Epd_setdither)(unsigned char status);
int (*Epd_get_mode)();
int (*Get_Waveform_Mode)();

};

extern void EPD_REG(struct EPD_INFO *pInfo);
extern int mcu_ioctl(unsigned int cmd, unsigned long arg);

