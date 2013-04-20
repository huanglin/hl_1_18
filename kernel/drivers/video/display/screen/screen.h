typedef enum _SCREEN_TYPE {
    SCREEN_NULL = 0,
    SCREEN_RGB,
	SCREEN_MCU,
    SCREEN_TVOUT,
    SCREEN_HDMI,
} SCREEN_TYPE;

typedef enum _REFRESH_STAGE {
    REFRESH_PRE = 0,
    REFRESH_END,

} REFRESH_STAGE;


typedef enum _MCU_IOCTL {
    MCU_WRCMD = 0,
    MCU_WRDATA,
    MCU_SETBYPASS,

} MCU_IOCTL;


typedef enum _MCU_STATUS {
    MS_IDLE = 0,
    MS_MCU,
    MS_EBOOK,
    MS_EWAITSTART,
    MS_EWAITEND,
    MS_EEND,

} MCU_STATUS;


/* Sceen description */
struct rk28fb_screen {
    /* screen type & out face */
    u16 type;
    u16 face;

	/* Screen size */
	u16 x_res;
	u16 y_res;
 	u16 x_epd_res;
   	 u16 y_epd_res;
    /* Timing */
	u16 pixclock;
	u16 left_margin;
	u16 right_margin;
	u16 hsync_len;
	u16 upper_margin;
	u16 lower_margin;
	u16 vsync_len;

    /* mcu need */
	u8 mcu_wrperiod;
    u8 mcu_usefmk;
    u8 mcu_frmrate;

	/* Pin polarity */
	u8 pin_hsync;
	u8 pin_vsync;
	u8 pin_den;
	u8 pin_dclk;
	u8 pin_dispon;

	/* Swap rule */
    u8 swap_rb;
    u8 swap_rg;
    u8 swap_gb;
    u8 swap_delta;
    u8 swap_dumy;

    /* Operation function*/
    int (*init)(void);
	int (*deinit)(void);
    int (*standby)(u8 enable);
    int (*refresh)(u8 arg);
    int (*scandir)(u16 dir);
    int (*disparea)(u8 area);
    int (*setpar)(u32 *SrcY,u32 SrcW, u32 SrcH, u32 virh,u8 Format);
};

extern void set_lcd_info(struct rk28fb_screen *screen);
extern void set_tv_info(struct rk28fb_screen *screen);
extern void set_hdmi_info(struct rk28fb_screen *screen);


