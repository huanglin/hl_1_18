/* arch/arm/mach-rockchip/rk28_battery.c
 *
 * Copyright (C) 2009 Rockchip Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/mach/time.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/uaccess.h>

#include <asm/arch/rk28_debug.h>
#include <asm/arch/adc.h>
#include "../video/display/screen/epd/eink_s.h"
//#include <linux/wakelock.h>
//#include <asm/gpio.h>

/* Debug */
#define BAT_DBG 0
#if BAT_DBG
#define DBG(x...)	printk(KERN_INFO x)
ktime_t ktime_now,ktime_pre,ktime_start;

#else
#define DBG(x...)
#endif

/*use adc sample battery capacity*/
#define AC_INSERT_VALUE     220
#define BATT_1V2_MODIFY	100
//#define BATT_1V2_VALUE		1338*BATT_1V2_MODIFY/100
#define BATT_1V2_VALUE		1420*BATT_1V2_MODIFY/100
#define BATT_3V05_VALUE		3120//3050
#define BATT_3V00_VALUE		3090//3050

#define BATT_FULL_VALUE	       4200
#define BATT_STEP_FULL_VALUE  4000
#define BATT_EMPTY_VALUE	3500
#define PERCENT				100
#define BATT_LEVEL_FULL		100
#define BATT_LEVEL_EMPTY	0
#define BATT_PRESENT_TRUE	 1
#define BATT_PRESENT_FALSE  0
#define BATT_NOMAL_VOL_VALUE	4000
#define BATT_VOLTAGE_MAX	4200
#define BATT_VOLTAGE_MIN	3400//3300//3500
#define AD_SAMPLE_TIMES	6
#define AC_OFFSET     500
#define PER_MINUTE	600//(60*1000*1000*1000/(TS_POLL_DELAY))
#define CHARGE_FULL_GATE 4140//4100

#define AD_NO_BATT_VALE       200
#define AD_NO_DC_VALE         200

#define TS_POLL_DELAY		(100*1000*1000)
#define SEC_NUM				2  ///8
#define PER_SEC_NUM		20  ///10

#define  BATTERY_SPI_ADDR  0x1e000
static int bat_vol_cnt = 0;  
static int bat_vol_up_cnt = 0; 
static int bat_vol_no_power_cnt = 0;  
static int bat_status =  POWER_SUPPLY_STATUS_UNKNOWN;
static int bat_health = POWER_SUPPLY_HEALTH_GOOD;
static int bat_capacity = BATT_LEVEL_EMPTY;
static int bat_present = BATT_PRESENT_TRUE;
static int bat_voltage =  BATT_NOMAL_VOL_VALUE;
static int ad_sample_current_time = 0;
unsigned int sample_times = 0;			/*count times (report batter status)*/
static int charger_change_cnt = 0;

struct timer_list ac_check_timer;
//static int g_charge_full_cnt = 0;
/*
 * 注意[xjh]:
 * 界面要显示电池容量的百分比，
 * 而不是当前电池电压在整个锂电池工作电压范围的百分比
 * 电池容量和电池电压不成正比
 * 
 * 锂电池的放电特性:
 * 充满电后，一开始放电，前面锂电池的电压下降比较剧烈(
但不代表电池容量也急剧下降)，
 * 然后进入一个稳定期，在稳定期电压下降很不明显
 * 当电量接近放完时，又有一个剧烈的电压下降，如果继续放电，则会毁损电池。
 * 
 * 锂电池的充电特性:
 * 刚开始以恒定电流进行充电，电压上升比较快。
 * 当达到限制电压后，以涓流方式进行恒压充电。
 */

#if 1
//电池放电数组
static int batt_step_table[56]={
    /*
    3400,3420,3430,3440,3450,3465,3480,3495,3510,3520,
	3530,3545,3560,3575,3590,3600,3610,3620,3630,3640,
	3650,3660,3670,3680,3690,3700,3710,3720,3735,3750,
	3765,3780,3790,3805,3820,3835,3850,3865,3880,3895,
	3910,3925,3940,3955,3970,3985,4000,4015,4030,4045,
	4060,4075,4100,4130,4150,4200	
	*/
	//提高关机时的电压
	3490,3510,3520,3530,3545,3560,3575,3590,3600,3610,
	3620,3630,3640,3650,3660,3670,3680,3690,3700,3710,
	3720,3735,3750,3765,3780,3790,3805,3820,3835,3850,
	3865,3880,3895,3910,3925,3940,3955,3970,3985,4000,
	4015,4030,4045,4060,4075,4100,4130,4150,4200,4200,	
	4200,4200,4200,4200,4200,4200
};

//电池充电数组
static int batt_no_current_step_table[56]={
	/*
	3410,3440,3665,3680,3695,3705,3720,3735,3750,3765,
	3780,3795,3810,3820,3830,3840,3850,3860,3870,3880,
	3890,3900,3910,3920,3925,3930,3940,3950,3960,3970,
	3980,3990,4000,4010,4020,4030,4040,4050,4060,4070,
	4080,4090,4100,4110,4120,4125,4130,4135,4140,4145,
	4150,4155,4165,4180,4190,4200
	*/
	
	3490,3510,3765,3780,3795,3810,3820,3830,3840,3850,
	3860,3870,3880,3890,3900,3910,3920,3925,3930,3940,
	3950,3960,3970,3980,3990,4000,4010,4020,4030,4040,
	4050,4060,4070,4080,4090,4100,4105,4110,4115,4120,
	4125,4130,4145,4150,4155,4165,4180,4190,4200,4200,
	4200,4200,4200,4200,4200,4200
};

static int batt_disp_table[56]={
	/*
	 0, 1, 3, 5, 7, 9, 11,13,15,17,
	19,21,23,25,27,29,31,33,35,37,
	39,41,43,45,47,49,51,53,55,57,
	59,61,63,65,67,69,71,73,75,77,
	79,81,83,85,87,89,90,91,93,94,
	95,97,100,100,100,100	
	*/
	
	0, 1, 3, 5, 7, 9, 11,13,15,17,
	19,21,23,25,27,29,31,33,35,37,
	39,41,43,45,47,50,53,56,59,62,
	65,68,71,74,77,80,83,86,89,92,
	95,99,99,99,99,99,99,99,100,100,
	100,100,100,100,100,100	
};

static int batt_disp_table_no_current[56]={
	/*
	 0, 1, 3, 5, 7, 9, 11,13,15,17,
	19,21,23,25,27,29,31,33,35,37,
	39,41,43,45,47,49,51,53,55,57,
	59,61,63,65,67,69,71,73,75,77,
	79,81,83,85,87,90,92,95,97,98,
	99,99,99,99,99,100	
	*/
	 0, 1, 3, 5, 7, 9, 11,13,15,17,
	19,21,23,25,27,29,31,33,35,37,
	39,41,43,45,47,50,53,56,59,62,
	65,68,71,74,77,80,83,86,89,92,
	95,99,99,99,99,99,99,99,100,100,
	100,100,100,100,100,100
};


#else
static int batt_step_table[56]={
/*
    3400,3420,3470,3500,3540,3545,3555,3560,3580,3600,3615,3630,3640,3650,3660,3670,3680,3690,
    3700,3710,3720,3730,3740,3750,3760,3770,3780,3790,3800,3810,3815,3830,3845,3860,3875,3890,
    3900,3910,3920,3930,3940,3950,3960,3970,3985,4000,4005,4010,4015,4020,40300,40400,4050,
    4060,4070,4200
    */
     3400,3420,3470,3500,3540,3545,3555,3560,3580,3600,3615,3630,3640,3650,3660,3670,3680,3690,
    3700,3710,3720,3730,3740,3750,3760,3770,3780,3790,3800,3810,3815,3830,3845,3860,3875,3890,
    3900,3910,3920,3930,3940,3950,3960,3970,3985,4000,4005,4010,4015,4020,4030,4040,4050,
    4060,4070,4200
}; //放电对应数组

static int ac_batt_step_table[56]={
3880,3886,3892,3898,3904,3910,3916,3922,3928,3934,
3940,3946,3952,3958,3964,3970,3976,3982,3988,3994,
4000,4006,4012,4018,4024,4030,4036,4042,4048,4054,
4060,4066,4072,4078,4084,4090,4096,4102,4108,4114,
4120,4126,4132,4138,4144,4150,4156,4162,4168,4174,
4180,4186,4192,4198,4204,4210};

static int batt_no_current_step_table[56]={
	3410,3440,3600,3650,3680,3690,3700,3715,3725,3740,3745,3755,3765,3770,3778,3784,3790,3800,3810,
	3820,3835,3845,3855,3870,3880,3890,3898,3905,3913,3920,3930,3940,3950,3960,3970,3980,3988,3995,
	4002,4010,4018,4025,3035,4042,4051,4060,4065,4070,4075,4080,4085,4090,4095,4100,4105,4200
/*	3980,3986,3992,3998,4004,4010,4016,4022,4028,4034,
4040,4046,4052,4058,4064,4070,4076,4082,4088,4094,
4100,4106,4112,4118,4124,4130,4136,4142,4148,4154,
4160,4166,4172,4178,4184,4190,4196,4202,4208,4214,
4220,4226,4232,4238,4244,4250,4256,4262,4268,4274,
4280,4286,4292,4298,4304,4310*/
};//充电对应数组
static int batt_disp_table[56]={
    0,3,5,8,10,12,14,15,18,20,23,26,28,30,33,37,40,43,47,
    50,52,54,57,60,62,64,66,68,69,70,72,74,76,78,79,
    80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
    96,97,98,99,100	
};
#endif
static u16 g_adcref = 0;
static u16 g_adcbat = 0;	//adccharge;
static int ac_power_off = 0;   // 0 is not power down,1 is power down enter sleep level2
static int full_flag = 0;
static int full_time_cnt = 0;
static int adj_cnt = 0;
static int battery_coefficient=0;
extern	u16 get_rock_adc0(void);	/*battery capacity*/
extern	u16 get_rock_adc1(void);	/*battery charge status*/
extern    u16  get_rock_adc2(void);       /*ac charge status */
extern	u16 get_rock_adc3(void);	/*battery vref*/

int usb_msc_connected = 0;

int get_msc_connect_flag(void)
{
	return(usb_msc_connected);
}
EXPORT_SYMBOL(get_msc_connect_flag);

void set_msc_connect_flag( int connected )
{
	//GPIOSetPinLevel(CHARGE_OK_PIN,GPIO_LOW);
    printk("set usb_msc_connect status = %d 20100803\n" , connected);	
    if( usb_msc_connected == connected )
            return;
        usb_msc_connected = connected;//usb mass storage is ok
#ifdef CONFIG_ANDROID_POWER   
                //20100315 yangkai
                if( !connected ) {
                        rk28_send_wakeup_key();
                }
#endif
}
EXPORT_SYMBOL(set_msc_connect_flag);

struct rk28_battery_data {
	spinlock_t lock;

	struct power_supply 	battery;
	struct power_supply	usb;
	struct power_supply	ac;
	struct hrtimer  timer;	
};


#define APP_BATT_PDEV_NAME		"rockchip_battery"
#define DRV_VER 			"1.0"
typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;


static int rockchip_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int rockchip_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);
static int rockchip_ac_get_property(struct power_supply *psy, 
					enum power_supply_property psp,
					union power_supply_propval *val);
static int get_ac_charge_status(void);


static enum power_supply_property rockchip_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property rockchip_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};


static struct power_supply rockchip_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = rockchip_battery_properties,
		.num_properties = ARRAY_SIZE(rockchip_battery_properties),
		.get_property = rockchip_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = rockchip_power_properties,
		.num_properties = ARRAY_SIZE(rockchip_power_properties),
		.get_property = rockchip_usb_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_AC,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = rockchip_power_properties,
		.num_properties = ARRAY_SIZE(rockchip_power_properties),
		.get_property = rockchip_ac_get_property,
	},
};

static void record_battery_log(char *buf)
{
	struct file *fp;
	char buf1[500]= "battery test";
	mm_segment_t fs;
	fp = filp_open("/flash/battery.txt",O_RDWR|O_APPEND |O_CREAT  ,0644);
	if(IS_ERR(fp)){
		printk("create file error!\n");
		return;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	//vfs_read(fp,buf1,sizeof(buf1),&pos);
	vfs_write(fp,buf,strlen(buf),&fp->f_pos);
	set_fs(fs);
	filp_close(fp,NULL);
	//printk("%s--->%d\n",__FUNCTION__,__LINE__);
	return;
}


static int rockchip_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	
	//todo 
	charger =  CHARGER_USB;
//	DBG("--------%s-->%s-->%d\n",__FILE__,__FUNCTION__,__LINE__);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		#if 1 
		if (psy->type == POWER_SUPPLY_TYPE_AC)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB){
			val->intval = get_msc_connect_flag();
			}
		else
	         val->intval = 0;
		 #else 
		  val->intval = 0;
		 #endif
		break;
	default:
		return -EINVAL;
	}
	return 0;

}
static int rockchip_ac_get_property(struct power_supply *psy, 
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	charger_type_t charger;
	
	//todo 
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE: 
		val->intval = get_ac_charge_status();
		break;
	default:
		return -EINVAL;
	}
	return 0;

}

#if 0
 static void rk28_power_off(void)
{
	GPIOSetPinLevel(GPIOPortF_Pin1,GPIO_LOW);	/*power down*/
}
#else
extern void rk28_power_off(void);

 #endif


static int get_usb_charge_status()
{
	if(get_msc_connect_flag()){
		if(rk2818_get_suspend_flags() == PM_ONELEVEL_SLEEP){
		GPIOSetPinLevel(CHARGE_AC_IOPIN, GPIO_LOW);	 
		GPIOSetPinLevel(CHARGE_USB_IOPIN, GPIO_LOW);
		GPIOSetPinLevel(CHARGE_STANBY_IOPIN, GPIO_HIGH);	
	}
	else if(GPIOGetPinLevel(WIFI_PWDN_IOPIN)){//wifi open the charge current is low
		GPIOSetPinLevel(CHARGE_AC_IOPIN, GPIO_LOW);	 
		GPIOSetPinLevel(CHARGE_USB_IOPIN, GPIO_LOW);
		GPIOSetPinLevel(CHARGE_STANBY_IOPIN, GPIO_LOW);			
	}
	else{
		GPIOSetPinLevel(CHARGE_AC_IOPIN, GPIO_LOW);	 
		GPIOSetPinLevel(CHARGE_USB_IOPIN, GPIO_HIGH);
		GPIOSetPinLevel(CHARGE_STANBY_IOPIN, GPIO_LOW);	
	}
		return 1;
	}	
	else
		return 0;
}
 int ac_charge_status=0;
 int ac_charge_flag=0;
void accheck_callback()
{
	ac_charge_flag=1;
}
 static int get_ac_charge_status(void)
 {

		 if(get_msc_connect_flag())
			return 0;
	 	 if (dwc_vbus_status()) {	
		 		if(ac_charge_status==0){
					 ac_charge_status=1;	
					 ac_check_timer.expires = jiffies + msecs_to_jiffies(1000);
					 add_timer(&ac_check_timer);
					 return 0;
				 }
				 if(ac_charge_flag==0)
				 	return 0;
		 		 GPIOSetPinLevel(CHARGE_AC_IOPIN, GPIO_HIGH);	 
				 GPIOSetPinLevel(CHARGE_USB_IOPIN, GPIO_LOW);
				 GPIOSetPinLevel(CHARGE_STANBY_IOPIN, GPIO_LOW);		 
				 return 1;

		 } else {
		 		ac_charge_status=0;
				ac_charge_flag=0;
		 		GPIOSetPinLevel(CHARGE_AC_IOPIN, GPIO_LOW);	 
				GPIOSetPinLevel(CHARGE_USB_IOPIN, GPIO_LOW);
				GPIOSetPinLevel(CHARGE_STANBY_IOPIN, GPIO_LOW);
				return 0;
		 }
		
 }
static int get_battery_charge_status( void )
{
	if(get_usb_charge_status()||get_ac_charge_status())
		return 1;
	else 
		return 0;
}


extern void rk28_send_wakeup_key( void );
static int rockchip_get_battery_status(void)
{
	//u16 adcref,adcbat;	//adccharge;
	int  current_vol,i;
	int charge_status;
	charge_status=get_battery_charge_status();
	if(charge_status) 
		bat_status =POWER_SUPPLY_STATUS_CHARGING ;
	else 
		bat_status =POWER_SUPPLY_STATUS_DISCHARGING ;	/*no charge*/
	if(full_flag&&charge_status)
		bat_status =POWER_SUPPLY_STATUS_FULL ;//避免充电的符号闪动
	ad_sample_current_time++;
	RockAdcScanning();
    	g_adcbat = (g_adcbat + get_rock_adc0())/2;
   	g_adcref = (g_adcref + get_rock_adc3())/2;
	if(ad_sample_current_time < AD_SAMPLE_TIMES) 
		return 1;
	ad_sample_current_time = 0;		
	if(g_adcbat < AD_NO_BATT_VALE)	/*haven't battery*/ 
	{
		bat_present = BATT_PRESENT_FALSE;	
		goto nobattery;
	}
	bat_present = BATT_PRESENT_TRUE;	/*have battery*/
	/*get charge status*/
	/*get present voltage*/
	current_vol = (g_adcbat * battery_coefficient)/1000;
	bat_voltage = current_vol;
	/*get battery health status*/
	if(batt_step_table[0]>=current_vol)
	{
		if(charge_status)
		{
			bat_health = POWER_SUPPLY_HEALTH_GOOD;	/*current voltage too poor*/
			bat_capacity =  1;
			bat_vol_no_power_cnt = 0;	
		}
		else
		{
			bat_vol_no_power_cnt++;
			if(bat_vol_no_power_cnt< 80){
			    bat_capacity = 1;
			    return 1;
			}
			bat_vol_no_power_cnt = 0;
			bat_health = POWER_SUPPLY_HEALTH_GOOD;	/*current voltage too poor*/
			bat_capacity =	0;   ///9;
			printk("battery is too poor>>power down!!!");
		}
		return 1;
	}
	else if(CHARGE_FULL_GATE <=current_vol)
	{
			if(GPIOGetPinLevel(CHG_OK_IOPIN)) /* current voltage full */							/*xxm*/
			{
				bat_health = POWER_SUPPLY_HEALTH_GOOD;
				bat_vol_no_power_cnt = 0;
				bat_capacity =  BATT_LEVEL_FULL;
				full_flag = 1;
				full_time_cnt = 0;
			}else
				{
					bat_health = POWER_SUPPLY_HEALTH_GOOD;
					bat_vol_no_power_cnt = 0;
					bat_capacity =  99;
				}
		return 1;
	}
	bat_vol_no_power_cnt = 0;
#if 1
	if(charge_status){
		for(i=0; i<55; i++){		
		    if((batt_no_current_step_table[i]<=current_vol)&&(batt_no_current_step_table[i+1]>current_vol))break;		
	    }
		bat_capacity = batt_disp_table_no_current[i];
	}else{
	    for(i=0; i<55; i++){		
		    if((batt_step_table[i]<=current_vol)&&(batt_step_table[i+1]>current_vol))break;		
	    }
		bat_capacity = batt_disp_table[i];
	}
#endif

	bat_health = POWER_SUPPLY_HEALTH_GOOD;
	return 1;
nobattery:
	if( (!get_msc_connect_flag()) || get_ac_charge_status() )	/*the battery charge*/
		bat_status =POWER_SUPPLY_STATUS_CHARGING ;
	else 
		bat_status =POWER_SUPPLY_STATUS_DISCHARGING ;	/*no charge*/
	bat_health = POWER_SUPPLY_HEALTH_GOOD;
	return 0;

}

static int rockchip_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	//DBG("--------%s-->%s-->property_psp%d\n",__FILE__,__FUNCTION__,psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bat_present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bat_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* get power supply */
		val->intval = bat_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* Todo return battery level */	
		val->intval = bat_capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val ->intval = bat_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = BATT_VOLTAGE_MAX;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = BATT_VOLTAGE_MIN;
		break;
	default:		
		return -EINVAL;
	}
	
	return 0;
}


 static enum hrtimer_restart rk28_battery_dostimer(struct hrtimer *handle)
{

	struct rk28_battery_data *data = container_of(handle, struct rk28_battery_data, timer);
	int old_bat_status = bat_status;
	int old_bat_capacity = bat_capacity;
	unsigned long flags;
	#if BAT_DBG
	char bat_info[500];
	
	#endif
	spin_lock_irqsave(&data->lock, flags);
	rockchip_get_battery_status();		/*have battery*/
	full_time_cnt ++;
	if(full_time_cnt >= 2 * PER_MINUTE) 
	{
		full_flag = 0;
		full_time_cnt = 0;
	}
	/*if have usb supply power*/		
	//printk("get_adc2() return %d \n",get_rock_adc2());
	if((bat_present == BATT_PRESENT_TRUE)&&(old_bat_status != bat_status))
	{
		
		charger_change_cnt = 80;
		printk("charge status changed::charger_change_cnt %d old_bat_status %d bat_status %d\n",
			charger_change_cnt,old_bat_capacity,bat_capacity);
		bat_capacity = old_bat_capacity;
		DBG("\n----usbchange----->%s:  old_bat_status==%i  ->bat_status==%i\n",data->battery.name,old_bat_status,bat_status);
		DBG("---->battery adcbat = %d adcref=%d\n",g_adcbat,g_adcref);
		DBG("---->battery present = %d\n",bat_present);
		DBG("---->battery status  = %d\n",bat_status);
		DBG("---->pb0 status = %d pb1 status = %d  usbchargestatus== %d\n",
		GPIOGetPinLevel(GPIOPortB_Pin0),GPIOGetPinLevel(GPIOPortB_Pin1),!get_msc_connect_flag());
		DBG("---->battery current voltage = %d\n",bat_voltage);
		DBG("---->battery capacity = %d\n",bat_capacity);
		power_supply_changed(&data->battery);
		goto next;
	}

	
	if(charger_change_cnt) { //避免拔插适配器电量显示浮动大
		charger_change_cnt--;
		bat_capacity = old_bat_capacity;
		goto update;
	}
	
	//避免任何情况下很大的跳动，比如拨到HOLD键时电池采样电压会高0.2v，这会造成很大的浮动
	if(((old_bat_capacity-bat_capacity)>10) || ((bat_capacity-old_bat_capacity)>10))
	{
		adj_cnt++;
		if(adj_cnt< 80)
			bat_capacity = old_bat_capacity;
		else {
			adj_cnt = 0;
			old_bat_capacity = bat_capacity;
		}
		goto update;	
	}

		
	/*fine set battery capacity*/
	if((get_battery_charge_status())&&(bat_capacity < old_bat_capacity)){	

		if((old_bat_capacity-bat_capacity)<10){
		    bat_capacity = old_bat_capacity;
		    bat_vol_up_cnt = 0;
		}else{
		    bat_vol_up_cnt++;
			if(bat_vol_up_cnt > 80 /*20*/)
			    bat_vol_up_cnt = 0;
			else	
			    bat_capacity = old_bat_capacity;
	    }
	}		
		if((!get_battery_charge_status())&&(bat_capacity > old_bat_capacity)){		
		if((bat_capacity-old_bat_capacity)<10){
			bat_capacity = old_bat_capacity;
			bat_vol_cnt = 0;
		}else{
			bat_vol_cnt++;
			if(bat_vol_cnt > 80)
			    bat_vol_cnt = 0;
			else	
			    bat_capacity = old_bat_capacity;
		}	
	}
update:	
	sample_times ++;						/*count times (report batter status)*/
	if((bat_present == BATT_PRESENT_TRUE)&&(sample_times > SEC_NUM * PER_SEC_NUM) )
	{
		sample_times = 0;
		#if BAT_DBG
		ktime_now = ktime_get();
		DBG("\ntime at %Lu nanosec(interal: %Lu)\n" ,ktime_to_ns( ktime_now),ktime_to_ns( ktime_sub( ktime_now,ktime_pre )) );
		ktime_pre = ktime_now;
		#endif
		DBG("****>battery adcbat = %d adcref=%d\n",g_adcbat,g_adcref);
		DBG("---->battery present = %d\n",bat_present);
		DBG("---->battery status  = %d\n",bat_status);
		DBG("---->pb0 status = %d pb1 status = %d  usbchargestatus== %d\n",
		GPIOGetPinLevel(GPIOPortB_Pin0),GPIOGetPinLevel(GPIOPortB_Pin1),!get_msc_connect_flag());
		DBG("---->battery current voltage = %d\n",bat_voltage);
		DBG("---->battery capacity = %d\n",bat_capacity);
		power_supply_changed(&data->battery);
	}
next:
	spin_unlock_irqrestore(&data->lock, flags);
	handle->expires = ktime_add(handle->expires, ktime_set(0,TS_POLL_DELAY));
	return HRTIMER_RESTART;

 }
static int rockchip_battery_probe(struct platform_device *pdev)
{
	int  rc,i;
	struct rk28_battery_data  *data;
	
	DBG("RockChip battery driver %s\n",DRV_VER);
	/* init power supplier framework */
	rockchip_mux_api_set(CHARGE_STANBY_IOMUX_PINNAME, CHARGE_STANBY_IOMUX_PINDIR);
	GPIOPullUpDown(CHARGE_STANBY_IOPIN, GPIONormal);
	GPIOSetPinDirection(CHARGE_STANBY_IOPIN, GPIO_OUT);
	GPIOSetPinLevel(CHARGE_STANBY_IOPIN, GPIO_LOW);

	rockchip_mux_api_set(CHARGE_USB_IOMUX_PINNAME, CHARGE_USB_IOMUX_PINDIR);
	GPIOPullUpDown(CHARGE_USB_IOPIN, GPIONormal);
	GPIOSetPinDirection(CHARGE_USB_IOPIN, GPIO_OUT);
	GPIOSetPinLevel(CHARGE_USB_IOPIN, GPIO_LOW);

	rockchip_mux_api_set(CHARGE_AC_IOMUX_PINNAME, CHARGE_AC_IOMUX_PINDIR);
	GPIOPullUpDown(CHARGE_AC_IOPIN, GPIONormal);
	GPIOSetPinDirection(CHARGE_AC_IOPIN, GPIO_OUT);
	GPIOSetPinLevel(CHARGE_AC_IOPIN, GPIO_LOW);

	rockchip_mux_api_set(GPIOB3_U0RTSN_SEL_NAME, CHG_OK_IOMUX_PINDIR);
	GPIOPullUpDown(CHG_OK_IOPIN, GPIONormal);
	GPIOSetPinDirection(CHG_OK_IOPIN, GPIO_IN);
	battery_coefficient=6584;
	data=kzalloc(sizeof(*data), GFP_KERNEL);
	spin_lock_init(&data->lock);
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = rk28_battery_dostimer;
	data ->battery = rockchip_power_supplies[0];
	data ->usb	  = rockchip_power_supplies[1];
	data ->ac        = rockchip_power_supplies[2];
	DBG("test %s-->%s-->%s\n",data->battery.name,data->usb.name,data->ac.name);
	 init_timer(&ac_check_timer);
    	ac_check_timer.function = accheck_callback;
    //ac_check_timer.expires = jiffies + msecs_to_jiffies(1500);
    	//add_timer(&ac_check_timer);

	rc = power_supply_register(&pdev->dev, &data ->battery);
	if (rc)
	{
		printk(KERN_ERR "Failed to register battery power supply (%d)\n", rc);
		goto err_battery_fail;
	}

	rc = power_supply_register(&pdev->dev, &data ->usb);
	if (rc)
	{
		printk(KERN_ERR "Failed to register usb power supply (%d)\n", rc);
		goto err_usb_fail;
	}
#if 1
	rc = power_supply_register(&pdev->dev, &data ->ac);
	if (rc)
	{
		printk(KERN_ERR "Failed to register ac power supply (%d)\n", rc);
		goto err_ac_fail;
	}
	DBG("--------cur time:0x%Lx\n",__FILE__,__FUNCTION__,ktime_get() );
#endif
	ADCInit();
   	 bat_vol_no_power_cnt = 81;
	g_adcbat = get_rock_adc0();
	g_adcref = get_rock_adc3();
	/*get originally battery stauts*/
	for(i=0;i<AD_SAMPLE_TIMES;i++)
	{
		rockchip_get_battery_status( );	
		mdelay(15);
	}
	/*low battery low need power down*/
	DBG("---->battery adcbat = %d adcref=%d\n",g_adcbat,g_adcref);
	DBG("---->battery present = %d\n",bat_present);
	DBG("---->battery status  = %d\n",bat_status);
	DBG("---->pb1 status = %d  usbchargestatus== %d\n",GPIOGetPinLevel(GPIOPortB_Pin1),!get_msc_connect_flag());
	DBG("---->battery current voltage = %d\n",bat_voltage);
	DBG("---->battery capacity = %d\n",bat_capacity);
	hrtimer_start(&data->timer,ktime_set(10,TS_POLL_DELAY),HRTIMER_MODE_REL);
	
	return 0;
err_battery_fail:
	power_supply_unregister(&data->battery);
	
err_usb_fail:
	power_supply_unregister(&data->usb);

err_ac_fail:
	power_supply_unregister(&data->ac);
 
	return rc;
		
}

#if 0   //add by cst 
static ssize_t ac_power_mode_show(struct device_driver *_drv,char *_buf)
{
        int count;

        count = sprintf(_buf,"%d",ac_power_off);
        return count;
}

static ssize_t ac_power_mode_store(struct device_driver *_drv,const char *_buf,size_t _count)
{
          
          ac_power_off  = (int)simple_strtol(_buf, NULL, 10);

          return _count;

}

static DRIVER_ATTR(ac_power_off,0666,ac_power_mode_show,ac_power_mode_store);

#endif
#if 1   

static ssize_t get_battery_ad(struct device_driver *_drv,char *_buf)
{
        int count;
	 count = sprintf(_buf,"%d",g_adcbat);
	return count;
}

static ssize_t set_battery_coefficient(struct device_driver *_drv,const char *_buf,size_t _count)
{
	 int value=0;
	 int i;
	 int ret=0;
	 char buf[4]={0};
	 value=simple_strtol(_buf, NULL, 10);
	  //epd_spi_flash_write(BATTERY_SPI_ADDR,&value,4);
	  if(battery_coefficient<5800||battery_coefficient>7000)
		battery_coefficient=6584;
          return _count;
}
static ssize_t get_battery_coefficient(struct device_driver *_drv,char *_buf)
{
	 int count;
	// epd_spi_flash_read(BATTERY_SPI_ADDR,&battery_coefficient,4);
	 count = sprintf(_buf,"%d",battery_coefficient);
	 return count;
}
static ssize_t get_battery_voltage(struct device_driver *_drv,char *_buf)
{
	 int count;
	count=sprintf(_buf,"%d",bat_voltage);
	return count;
}
static DRIVER_ATTR(battery_status,0666,get_battery_ad,set_battery_coefficient);
static DRIVER_ATTR(battery_coefficient,0666,get_battery_coefficient,NULL);
static DRIVER_ATTR(battery_voltage,0666,get_battery_voltage,NULL);
#endif


static struct platform_driver rockchip_battery_driver = {
	.probe	= rockchip_battery_probe,
	.driver	= {
		.name	= APP_BATT_PDEV_NAME,
		.owner	= THIS_MODULE,
	},
};


static int __init rockchip_battery_init(void)
{
	printk("%s::========================================\n",__func__);
	int ret = platform_driver_register(&rockchip_battery_driver);
	if (ret == 0)
	{
		ret = driver_create_file(&rockchip_battery_driver.driver, &driver_attr_battery_status);
		ret = driver_create_file(&rockchip_battery_driver.driver, &driver_attr_battery_coefficient);
		ret = driver_create_file(&rockchip_battery_driver.driver, &driver_attr_battery_voltage);
	}
	return ret;
}
module_init(rockchip_battery_init);
MODULE_DESCRIPTION("Rockchip Battery Driver");
MODULE_LICENSE("GPL");
