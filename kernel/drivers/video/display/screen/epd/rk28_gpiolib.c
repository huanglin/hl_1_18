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
int gpio_num;
#define APP_GPIO_PDEV_NAME		"rockchip_gpiolib"
static struct platform_driver rockchip_gpiolib_driver = {
	.driver	= {
		.name	= APP_GPIO_PDEV_NAME,
		.owner	= THIS_MODULE,
	},
};
static ssize_t get_direction(struct device_driver *_drv,char *_buf)
{
	int count;
	int direction;
	direction=GPIOGetPinDirection(gpio_num);
	count = sprintf(_buf,"%d",direction);
	return count;
	
}
static ssize_t set_direction(struct device_driver *_drv,const char *_buf,size_t _count)
{
	int value=0;
	value=simple_strtol(_buf, NULL, 10);
	if(value>1)
		value=0;
	GPIOSetPinDirection(gpio_num, value);
       return _count;
}
static ssize_t get_value(struct device_driver *_drv,char *_buf)
{
	int count;
	int level;
	level=GPIOGetPinLevel(gpio_num);
	count = sprintf(_buf,"%d",level);
	return count;
}
static ssize_t set_value(struct device_driver *_drv,const char *_buf,size_t _count)
{
	
	int value=0;
	value=simple_strtol(_buf, NULL, 10);
	if(value>1)
		value=0;
	GPIOSetPinLevel(gpio_num, value);
	 return _count;
}
static ssize_t get_num(struct device_driver *_drv,char *_buf)
{
	int count;
	count = sprintf(_buf,"%d",gpio_num);
	return count;
}
static ssize_t set_num(struct device_driver *_drv,const char *_buf,size_t _count)
{
	 int value=0;
	 int i;
	 int ret=0;
	 value=simple_strtol(_buf, NULL, 10);
	 gpio_num=value;
        return _count;
}
static ssize_t get_pullupdown(struct device_driver *_drv,char *_buf)
{
	
}
static ssize_t set_pullupdown(struct device_driver *_drv,const char *_buf,size_t _count)
{
	int value=0;
	value=simple_strtol(_buf, NULL, 10);
	if(value>2)
		value=0;
	GPIOPullUpDown(gpio_num, value);
       return _count;
}
static DRIVER_ATTR(direction,0666,get_direction,set_direction);
static DRIVER_ATTR(value,0666,get_value,set_value);
static DRIVER_ATTR(setgpio_num,0666,get_num,set_num);
static DRIVER_ATTR(gpiopullupdown,0666,get_pullupdown,set_pullupdown);

//static DRIVER_ATTR(num,0666,get_battery_voltage,NULL);

static int __init rockchip_gpio_lib_init(void)
{
	int ret = platform_driver_register(&rockchip_gpiolib_driver);
	if (ret == 0)
	{
		ret = driver_create_file(&rockchip_gpiolib_driver.driver, &driver_attr_direction);
		ret = driver_create_file(&rockchip_gpiolib_driver.driver, &driver_attr_value);
		ret = driver_create_file(&rockchip_gpiolib_driver.driver, &driver_attr_gpiopullupdown);
		ret = driver_create_file(&rockchip_gpiolib_driver.driver, &driver_attr_setgpio_num);
	}
	return ret;
}


module_init(rockchip_gpio_lib_init);
MODULE_DESCRIPTION("Rockchip gpio_lib");
MODULE_LICENSE("GPL");

