/*
 * wifi_power.c
 *
 * Power control for WIFI module.
 *
 * There are Power supply and Power Up/Down controls for WIFI typically.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include "wifi_power.h"

#if (WIFI_GPIO_POWER_CONTROL == 1)

extern struct wifi_power power_gpio;
extern struct wifi_power power_save_gpio;
extern struct wifi_power power_reset_gpio;

int wifi_gpio_operate(struct wifi_power *gpio, int flag)
{
	int sensitive;
	
	if (gpio->use_gpio == POWER_NOT_USE_GPIO)
		return 0;
	
	if (gpio->gpio_iomux == POWER_GPIO_IOMUX)
	{
		rockchip_mux_api_set(gpio->iomux_name, gpio->iomux_value);
	}
	
	if (flag == GPIO_SWITCH_ON)
		sensitive = gpio->sensi_level;
	else
		sensitive = 1 - gpio->sensi_level;
		
	if (gpio->use_gpio == POWER_USE_EXT_GPIO)
	{
		wifi_extgpio_operation(gpio->gpio_id, sensitive);
	}
	else
	{
		GPIOSetPinDirection(gpio->gpio_id, GPIO_OUT);
		GPIOSetPinLevel(gpio->gpio_id, sensitive);
	}

	return 0;
}

//MV8686 reset sequence
int wifi_reset_card(void)
{
	if (power_reset_gpio.use_gpio == POWER_NOT_USE_GPIO)
		return 0;

	if (wifi_gpio_operate(&power_reset_gpio, GPIO_SWITCH_ON) != 0)
	{
		printk("Couldn't set GPIO [ON] for reset.\n");
		return -1;
	}
	mdelay(5);

	if (wifi_gpio_operate(&power_reset_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [OFF] for reset.\n");
		return -1;
	}
	mdelay(200);

	if (wifi_gpio_operate(&power_reset_gpio, GPIO_SWITCH_ON) != 0)
	{
		printk("Couldn't set GPIO [ON] for reset.\n");
		return -1;
	}
	mdelay(100);

	return 0;
}

//MV8686 power up sequence
int wifi_turn_on_card(void)
{
	/*
	 * Make sure we are in power off.
	 */
	if (wifi_gpio_operate(&power_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [ON] for power supply.\n");
		return -1;
	}
	
	if (wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [ON] for power up.\n");
		return -1;
	}
	mdelay(5);

	/*
	 * Power on sequence.
	 */
	if (wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_ON) != 0)
	{
		printk("Couldn't set GPIO [ON] for power up.\n");
		return -1;
	}
	mdelay(3);

	if (wifi_gpio_operate(&power_gpio, GPIO_SWITCH_ON) != 0)
	{
		printk("Couldn't set GPIO [ON] for power supply.\n");
		return -1;
	}
	mdelay(5);
	
	/* Reset sequence if necessary. */
	wifi_reset_card();

	return 0;
}

int wifi_turn_off_card(void)
{
	//printk("Turning off SDIO card.\n");
	
	if (wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_ON) != 0)
	{
		printk("Couldn't set GPIO [ON] for power up.\n");
		return -1;
	}
	
	if (wifi_gpio_operate(&power_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [OFF] for power supply.\n");
		return -1;
	}
	mdelay(1);
	
	if (wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [ON] for power up.\n");
		return -1;
	}
	
	if (wifi_gpio_operate(&power_reset_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [ON] for power up.\n");
		return -1;
	}
	
	return 0;
}

void rockchip_wifi_shutdown(void)
{
	printk("rockchip_wifi_shutdown....\n");

	wifi_turn_off_card();
}
EXPORT_SYMBOL(rockchip_wifi_shutdown);

#if 0

#include <linux/init.h>
#include <linux/platform_device.h>

void wifi_power_shutdown(struct platform_device *dev)
{
	printk("==============< Shutdown WiFi hardware >============\n");

	wifi_turn_off_card();
}

static int __devinit wifi_power_probe(struct platform_device *dev)
{
	printk("==============< WiFi power module init >============\n");

	return 0;
}

static struct platform_driver wifi_power_driver = 
{
	.driver		= 
	{
		.name	= "wifi_power",
		.owner	= THIS_MODULE,
	},
	.probe		= wifi_power_probe,
	.shutdown	= wifi_power_shutdown,
};

static struct platform_device *wifi_power_platform_device;

static int wifi_power_init(void)
{
	int error;

	wifi_power_platform_device = platform_device_alloc("wifi_power", -1);
	if (wifi_power_platform_device == NULL)
	{
		printk("Alloc wifi power platform device fail.\n");
		return -ENOMEM;
	}

	error = platform_device_add(wifi_power_platform_device);
	if (error != 0)
	{
		printk("Add wifi power platform device fail.\n");
		goto add_pf_dev_fail;
	}

	error = platform_driver_register(&wifi_power_driver);
	if (error)
	{
		printk("Register wifi_power platform driver fail.\n");
		goto reg_pf_drv_fail;
	}

	return 0;

reg_pf_drv_fail:
	platform_device_del(wifi_power_platform_device);

add_pf_dev_fail:
	platform_device_put(wifi_power_platform_device);

	return error;
}

module_init(wifi_power_init);
#endif

#endif /* WIFI_GPIO_POWER_CONTROL */

