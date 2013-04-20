/*
 * wifi_power.c
 *
 * Yongle Lai @ Rockchip Fuzhou @ 20100303.
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

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,25)
#define OS_IOMUX(name, value) rockchip_mux_api_set((name), (value))
#else
#define OS_IOMUX(name, value) rk29_mux_api_set((name), (value));
#endif

int wifi_gpio_operate(struct wifi_power *gpio, int flag)
{
	int sensitive;
	
	if (gpio->use_gpio == POWER_NOT_USE_GPIO)
		return 0;
	
	if (gpio->gpio_iomux == POWER_GPIO_IOMUX)
	{
		OS_IOMUX(gpio->iomux_name, gpio->iomux_value);
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
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,25)
		gpio_direction_output(gpio->gpio_id, GPIO_OUT);
		__gpio_set(gpio->gpio_id, sensitive);
#else
		gpio_direction_output(gpio->gpio_id, sensitive);
		gpio_set_value(gpio->gpio_id, sensitive);
#endif
	}

	return 0;
}

// WiFi reset sequence
int wifi_reset_card(void)
{
	if (power_reset_gpio.use_gpio == POWER_NOT_USE_GPIO)
		return 0;

	wifi_gpio_operate(&power_reset_gpio, GPIO_SWITCH_ON);
	mdelay(1);

	wifi_gpio_operate(&power_reset_gpio, GPIO_SWITCH_OFF);
	mdelay(3);

	wifi_gpio_operate(&power_reset_gpio, GPIO_SWITCH_ON);
	mdelay(3);

	return 0;
}

int wifi_turn_on_card(int module)
{
	/*
	 * Make sure we are in power off.
	 */
	wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_OFF);
	wifi_gpio_operate(&power_gpio, GPIO_SWITCH_OFF);
	msleep(25);

	wifi_gpio_operate(&power_gpio, GPIO_SWITCH_ON);
	msleep(100);
	
	wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_ON);
	msleep(200);

	/* Reset sequence if necessary. */
	wifi_reset_card();

	return 0;
}

int wifi_turn_off_card(void)
{
	wifi_gpio_operate(&power_reset_gpio, GPIO_SWITCH_OFF);
	wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_OFF);
	mdelay(1);
	wifi_gpio_operate(&power_gpio, GPIO_SWITCH_OFF);
	msleep(10);
	
	return 0;
}

void rockchip_wifi_shutdown(void)
{
	printk("rockchip_wifi_shutdown....\n");

	wifi_turn_off_card();
}
EXPORT_SYMBOL(rockchip_wifi_shutdown);

#endif /* WIFI_GPIO_POWER_CONTROL */

