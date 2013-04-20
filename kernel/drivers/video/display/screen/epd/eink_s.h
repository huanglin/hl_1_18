
#include <linux/spi/spi.h>

#define SCREEN_MAX_WIDTH  800//600          //eink.rkl support the width max
#define SCREEN_MAX_HEIGHT 1024

#define  GPIOLCDC             0
#define  GPIOLCDC_GDOE_NOR    1

#define CRC32_CRC32         0x04C10DB7

#define RGBTOY16_16   0x10
#define RGBTOY16_2    0x20
#define Y256TOY16_16  0x30
#define Y256TOY16_2   0x40
#define SENDDATA      0x50
#define SENDREST      0x60
#define SENDPEN       0x90
#define SENDRANDOM       0xa0
#define OTHER_PANLE  0
#define EINKS_PANLE  1
#define OED_PANLE    2
void Eink_Dsp_SetStates(int states);
int   Eink_s_init(u16 PanelW, u16 PanelH);
void   Eink_s_Deinit(void);
int   Eink_s_Reset(void);
void   Eink_s_SendData(u32 *src,int w,int h, int virh,u8 rotate,u8 DataFormat,u8 rim,u8 mode);
#ifdef CONFIG_EINK_BATTERY_CHECK
void Eink_display_init(int *buffer,int tempture,int dsp_pmu_base,int w,int h);
void Eink_s_Reset_in_check(int tempture,int dsp_pmu_base);
void Get_dsp_firmware(void *fw_eink_dsp);
void Set_eink_init_status();
void Quit_eink_init_status();
int Get_einkinit_status();
int Data_Init_in_check(u32 width, u32 height);
u32 Eink_s_waveform(char*pSaveTmp,int length);
#endif
struct epd_sensor_ops {
	int			(*get_value)(s32 *val);
	void			(*release)(void);
	void        	(*wakeup)(void);
	void        	(*shutdown)(void);
	struct module		*owner;
};

struct epd_sensor {
	struct epd_sensor_ops	*ops;
	char			*name;
};

struct spi_flash_ops {
	u32			(*Write)(void *sf, u32 addr, u8 *pData, u32 len);
	u32		(*Read)(void *sf, u32 addr, u8 *pData, u32 len);
	u32		(*Buffer_write)(void *sf, u32 addr, u8 *pData, u32 len);
	struct module		*owner;
};

struct spi_flash {
	struct spi_flash_ops	*ops;
    struct spi_device       *dev;
	char			*name;
};
struct a2_mode_req
{
	char a2_mode_enable; //0:disable a2_mode 1: enable a2_mode
	char a2_start_count;//auto start a2 mode count
	char a2_stop_count;//50ms*A2_stop_count is the time of quit A2 mode when no image refresh
	int    a2_w1;
	int    a2_w2;
	int    a2_h1;
	int    a2_h2;
};
extern int epd_register_sensor(struct epd_sensor *new_sr);
extern int epd_unregister_sensor(struct epd_sensor *sr);
extern int epd_register_flash(struct spi_flash *new_sf);
extern int epd_unregister_flash(struct spi_flash *sf);
extern int epd_spi_flash_read(int addr ,int *buf,int length);
extern int epd_spi_flash_write(int addr ,int *buf,int length);