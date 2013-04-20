/*
 * Export interface in /sys/class/touchpanel for calibration.
 *
 * Yongle Lai @ Rockchip - 2010-07-26
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>

#include "touchp.h"

/*
 * The sys nodes for touch panel calibration depends on controller's name,
 * such as: /sys/bus/spi/drivers/xpt2046_ts/touchadc
 * If we use another TP controller (not xpt2046_ts), the above path will 
 * be unavailable which will cause calibration to be fail.
 *
 * Another choice is: 
 *   sys/devices/platform/rockchip_spi_master/spi0.0/driver/touchadc
 * this path request the TP controller will be the first device of SPI.
 *
 * To make TP calibration module in Android be universal, we create
 * a class named touchpanel as the path for calibration interfaces.
 */
 
/*
 * TPC driver depended.
 */
extern volatile struct adc_point gADPoint;
#ifdef TS_PRESSURE
extern volatile int gZvalue[3];
#endif

#ifdef CONFIG_TP_800x600
#define XPT2046_TP_LENGTH       600
#define XPT2046_TP_WIDTH        800
#endif


#ifdef CONFIG_TP_1024x600
int screen_x[5] = {50, 974,  50, 974, 512};
int screen_y[5] = {50,  50, 550, 550, 300};
#endif

#ifdef CONFIG_TP_800x600
int screen_x[5] 	= {(XPT2046_TP_LENGTH-40),(XPT2046_TP_LENGTH-40),40,40, (XPT2046_TP_LENGTH/2)};
int screen_y[5] 	= {50, (XPT2046_TP_WIDTH-50), 50, (XPT2046_TP_WIDTH-50), (XPT2046_TP_WIDTH/2)};

#endif

#ifdef CONFIG_TP_800x480
int screen_x[5] = { 50, 750,  50, 750, 400};
int screen_y[5] = { 40,  40, 440, 440, 240};
#endif
#ifdef CONFIG_TP_480x272
int screen_x[5] = { 39-15, 		231+10,  	39-15, 		231+10, 136}; 
int screen_y[5] = { 49-15, 49-15, 20+429, 20+429, 240};//y ƫ���˴��20

#endif

int uncali_x[5] = { 0 };
int uncali_y[5] = { 0 };

#ifdef CONFIG_TP_800x480
int uncali_x_default[5] = { 341, 3758, 328, 3748, 2024 };
int uncali_y_default[5] = { 568, 598, 3509, 3514, 2198 };
#endif

#ifdef CONFIG_TP_800x600

int uncali_x_default[5] 	= {333,3760,333,3760,2046};
int uncali_y_default[5] 	= {450,450,3750,3750,2100};

#endif


#ifdef CONFIG_TP_480x272
int uncali_x_default[5] = { 570, 3470, 644, 3530, 2105 };
int uncali_y_default[5] = { 378, 370, 3318, 3337, 1561 };
#endif
static ssize_t touch_mode_show(struct class *cls, char *_buf)
{
    int count;
    
	count = sprintf(_buf,"TouchCheck:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	                uncali_x[0], uncali_y[0],
	                uncali_x[1], uncali_y[1],
	                uncali_x[2], uncali_y[2],
	                uncali_x[3], uncali_y[3],
	                uncali_x[4], uncali_y[4]);

	printk("buf: %s", _buf);
		
	return count;
}

static ssize_t touch_mode_store(struct class *cls, const char *_buf, size_t _count)
{
    int i, j = 0;
    char temp[5];

    //printk("Read data from Android: %s\n", _buf);
    
    for (i = 0; i < 5; i++)
    {
        strncpy(temp, _buf + 5 * (j++), 4);
        uncali_x[i] = simple_strtol(temp, NULL, 10);
        strncpy(temp, _buf + 5 * (j++), 4);
        uncali_y[i] = simple_strtol(temp, NULL, 10);
        printk("SN=%d uncali_x=%d uncali_y=%d\n", 
                i, uncali_x[i], uncali_y[i]);
    }

    return _count; 
}

//This code is Touch adc simple value
static ssize_t touch_adc_show(struct class *cls,char *_buf)
{
    printk("ADC show: x=%d y=%d\n", gADPoint.x, gADPoint.y);
    
	return sprintf(_buf, "%d,%d\n", gADPoint.x, gADPoint.y);
}

static ssize_t touch_cali_status(struct class *cls, char *_buf)
{
    int ret;
    
    ret = TouchPanelSetCalibration(4, screen_x, screen_y, uncali_x, uncali_y);
    if (ret == 1){
    	memcpy(uncali_x_default, uncali_x, sizeof(uncali_x));
    	memcpy(uncali_y_default, uncali_y, sizeof(uncali_y));
    	ret = sprintf(_buf, "successful\n");
    }
    else{
     	printk("touchpal calibration failed, use default value.\n");
    	ret = TouchPanelSetCalibration(4, screen_x, screen_y, uncali_x_default, uncali_y_default);
    	if (ret == 1){
    		ret = sprintf(_buf, "recovery\n");
    	}
    	else{
    		ret = sprintf(_buf, "fail\n");
   		}
    }
    
    //printk("Calibration status: _buf=<%s", _buf);
    
	return ret;
}
#ifdef TS_PRESSURE
static ssize_t touch_pressure(struct class *cls,char *_buf)
{
	printk("enter %s gADPoint.x==%d,gADPoint.y==%d\n",__FUNCTION__,gADPoint.x,gADPoint.y);
	return sprintf(_buf,"%d,%d,%d\n",gZvalue[0],gZvalue[1],gZvalue[2]);
}
#endif

static struct class *tp_class = NULL;

static CLASS_ATTR(touchcheck, 0666, touch_mode_show, touch_mode_store);
static CLASS_ATTR(touchadc, 0666, touch_adc_show, NULL);
static CLASS_ATTR(calistatus, 0666, touch_cali_status, NULL);
#ifdef TS_PRESSURE
static CLASS_ATTR(pressure, 0666, touch_pressure, NULL);
#endif

static int __init tp_calib_iface_init(void)
{
    int ret = 0;
    int err = 0;
    
    tp_class = class_create(THIS_MODULE, "touchpanel");
    if (IS_ERR(tp_class)) 
    {
        printk("Create class touchpanel failed.\n");
        return -ENOMEM;
    }
    
    err = TouchPanelSetCalibration(4, screen_x, screen_y, uncali_x_default, uncali_y_default);
  	if (err == 1){
		printk("Auto set calibration successfully.\n");
	} else {
		printk("Auto set calibraion failed, reset data again please !");
	}
    
    /*
	 * Create ifaces for TP calibration.
	 */
    ret =  class_create_file(tp_class, &class_attr_touchcheck);
    ret += class_create_file(tp_class, &class_attr_touchadc);
    ret += class_create_file(tp_class, &class_attr_calistatus);
#ifdef TS_PRESSURE
   ret += class_create_file(tp_class, &class_attr_pressure);
#endif
    if (ret)
    {
        printk("Fail to class ifaces for TP calibration.\n");
    }

    return ret;
}

static void __exit tp_calib_iface_exit(void)
{
    class_remove_file(tp_class, &class_attr_touchcheck);
    class_remove_file(tp_class, &class_attr_touchadc);
    class_remove_file(tp_class, &class_attr_calistatus);
#ifdef TS_PRESSURE
    class_remove_file(tp_class, &class_attr_pressure);
#endif
    class_destroy(tp_class);
}

module_init(tp_calib_iface_init);
module_exit(tp_calib_iface_exit);

MODULE_AUTHOR("Yongle Lai");
MODULE_DESCRIPTION("XPT2046 TPC driver @ Rockchip");
MODULE_LICENSE("GPL");
