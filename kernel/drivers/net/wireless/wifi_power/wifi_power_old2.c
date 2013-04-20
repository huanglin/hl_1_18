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

/*
	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOB1_SMCS1_MMC0PCA_NAME,
	IOMUXA_GPIO0_B1, GPIOPortB_Pin1, GPIO_HIGH

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOB3_U0RTSN_SEL_NAME,
	IOMUXA_GPIO0_B1, GPIOPortB_Pin3, GPIO_HIGH

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOB4_SPI0CS0_MMC0D4_NAME,
	IOMUXA_GPIO0_B4, GPIOPortB_Pin4, GPIO_HIGH

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOE_SPI1_SEL_NAME,
	IOMUXA_GPIO1_A1237, GPIOPortE_Pin1, GPIO_HIGH
	
	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOF5_APWM3_DPWM3_NAME,
	IOMUXB_GPIO1_B5, GPIOPortF_Pin5, GPIO_HIGH 

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOE_SPI1_SEL_NAME,
	IOMUXA_GPIO1_A1237, GPIOPortF_Pin7, GPIO_HIGH
*/
unsigned long driver_ps_timeout = 2 * 60 * 1000; //2 minutes 

#if (WIFI_GPIO_POWER_CONTROL == 1)

struct wifi_power power_gpio = 
{
	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOB1_SMCS1_MMC0PCA_NAME,
    IOMUXA_GPIO0_B1, GPIOPortB_Pin1, GPIO_HIGH
};

struct wifi_power power_save_gpio = 
{
 	POWER_USE_GPIO, 0, 0, 0, GPIOPortA_Pin4, GPIO_HIGH
};

#endif /* WIFI_GPIO_POWER_CONTROL */

