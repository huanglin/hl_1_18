/*
��ʵ���õ�ģʽ�����¼��֣����಻���á�
#define  EPD_FULL           0
#define  EPD_PART           2
#define  EPD_BLACK_WHITE    4
#define  EPD_AUTO           5
#define  EPD_GU_FULL        7
#define  EPD_TEXT                   9
#define  EPD_AUTO_PART     10
#define  EPD_AUTO_BLACK_WHITE 11
#define  EPD_A2                    12
PVI��ģʽӦ�ã�
EPD_FULL:ȫ��ˢ�£�16�ҽס��ţ���ӰС��ȱ��ˢ��ʱ�䳤����˸��ǿ���Ӿ�Ч���Ӧ���ڽ����л���������Ӱ��
EPD_PART���ֲ�ˢ�£�16�ҽס��ţ���˸������ȱ��ˢ��ʱ�䳤����Ӱ��Ӧ�����ı���ͼƬ��ҳ��ˢ���κ��Ӱ��Ƚϴ�����EPD_FULLˢ��һ��������Ӱ��
EPD_BLACK_WHITE���ֲ�ˢ�£�2�ҽס��ţ���˸������ˢ��ʱ��졣ȱ��ֻ��2�ҽס�Ӧ���ڲ˵���ѡ��	
EPD_AUTO���ֲ�ˢ�£�16�ҽס��ţ����ķ�Ӧ�ٶȿ죬�û������ٶȿ졣ȱ:�к���ЧӦ����Ӱ��Ӧ���ڲ˵���ѡ���ı���ͼƬ��ҳ�ȣ��Բ�ӰҪ�󲻸ߵĳ�����
EPD_TEXT���ֲ�ˢ�£�16�ҽס��ţ���ӰС����˸������ȱ��ˢ��ʱ�䳤��Ӧ�����ı���ͼƬ��ҳ��
EPD_A2���ֲ�ˢ�£�2�ҽס��ţ���˸������ˢ��ʱ����졣ȱ��ֻ��2�ҽף�ֻ�в���2������WAVEFORM���и�ģʽ��Ӧ�����ı���ͼƬ������ҳ��

������ģʽӦ�ã�
PVI��ģʽӦ�ã�
EPD_FULL:ȫ��ˢ�£�16�ҽס��ţ���ӰС��ȱ��ˢ��ʱ�䳤����˸��ǿ���Ӿ�Ч���Ӧ���ڽ����л���������Ӱ��
EPD_PART���ֲ�ˢ�£�16�ҽס��ţ���˸������ȱ��ˢ��ʱ�䳤����Ӱ��Ӧ�����ı���ͼƬ��ҳ��ˢ���κ��Ӱ��Ƚϴ�����EPD_FULLˢ��һ��������Ӱ��
EPD_BLACK_WHITE���ֲ�ˢ�£�2�ҽס��ţ���˸������ˢ��ʱ��졣ȱ��ֻ��2�ҽס�Ӧ���ڲ˵���ѡ��	
EPD_AUTO���ֲ�ˢ�£�16�ҽס��ţ����ķ�Ӧ�ٶȿ죬�û������ٶȿ졣ȱ:�к���ЧӦ����Ӱ��Ӧ���ڲ˵���ѡ���ı���ͼƬ��ҳ�ȣ��Բ�ӰҪ�󲻸ߵĳ�����
EPD_GU_FULL���ֲ�ˢ�£�16�ҽס��ţ���ӰС����˸������ȱ��ˢ��ʱ�䳤��Ӧ�����ı���ͼƬ��ҳ��
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

