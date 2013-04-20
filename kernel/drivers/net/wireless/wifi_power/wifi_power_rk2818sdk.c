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

int wifi_default_region = 0x10; // Country / region code

int wifi_external_eeprom = 0;

unsigned long driver_ps_timeout = 2 * 60 * 1000; //2 minutes 

#if (WIFI_GPIO_POWER_CONTROL == 1)

/*
 * GPIO to control LDO/DCDC.
 */
struct wifi_power power_gpio = 
{
	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOH7_HSADCCLK_SEL_NAME,
	IOMUXB_GPIO1_B5, GPIOPortH_Pin7, GPIO_HIGH
};

/*
 * GPIO to control WIFI PowerDOWN/RESET.
 */
struct wifi_power power_save_gpio = 
{
	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOF5_APWM3_DPWM3_NAME,
	IOMUXB_GPIO1_B5, GPIOPortF_Pin5, GPIO_HIGH
};

/*
 * GPIO to reset WIFI. Keep this as NULL normally.
 */
struct wifi_power *power_reset_gpio = &power_save_gpio;
//struct wifi_power *power_reset_gpio = NULL;

/*
 *  Please implement the following 2 functions if 
 *  you are using external GPIO to control WIFI module.
 *
 *  Note: please make variable power_gpio and power_save_gpio
 *  as all 0 in the above.
 */
void wifi_extgpio_turn_on_card(void)
{
}

void wifi_extgpio_turn_off_card(void)
{
}

#endif /* WIFI_GPIO_POWER_CONTROL */

