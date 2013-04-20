
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <linux/delay.h>
#define  EBOOK_PWRALL        				GPIOPortC_Pin7
#define  EBOOK_VCOM          				GPIOPortC_Pin6
#define  EBOOK_SHR           				GPIOPortC_Pin5
#define  EBOOK_GDRL          				GPIOPortC_Pin4
#define  EBOOK_DACLK         				GPIOPortC_Pin3
#define  EBOOK_DADIO         				GPIOPortC_Pin2
void Eink_S_power_init()
{
	rockchip_mux_api_set(GPIOC_LCDC18BIT_SEL_NAME, 1);
	rockchip_mux_api_set(GPIOC_LCDC24BIT_SEL_NAME, 0);
	rockchip_mux_api_set(EPD_EINK_ON_IOMUX_PINNAME, EPD_EINK_ON_IOMUX_PINDIR);
	gpio_direction_output(EPD_EINK_ON,0);
	GPIOSetPinLevel(EPD_EINK_ON,GPIO_LOW);
	rockchip_mux_api_set(EPD_PWR_ON_IOMUX_PINNAME,EPD_PWR_ON_IOMUX_PINDIR);
	gpio_direction_output(EBOOK_DACLK,0);
	gpio_direction_output(EPD_PWR_ON, 0);
	gpio_direction_output(EBOOK_PWRALL, 0);
	gpio_direction_output(EBOOK_SHR,0);
	gpio_direction_output(EBOOK_GDRL,0);
	gpio_direction_output(EBOOK_VCOM,0);
	gpio_direction_output(EBOOK_DADIO,0);
	GPIOSetPinLevel(EBOOK_DACLK,GPIO_LOW);
	GPIOSetPinLevel(EPD_PWR_ON, GPIO_LOW);
	GPIOSetPinLevel(EBOOK_PWRALL, GPIO_LOW);
	GPIOSetPinLevel(EBOOK_SHR,GPIO_LOW);
	GPIOSetPinLevel(EBOOK_GDRL,GPIO_LOW);
	GPIOSetPinLevel(EBOOK_VCOM,GPIO_LOW);
	GPIOSetPinLevel(EBOOK_DADIO,GPIO_LOW);
}
void   Eink_s_power_on(void)
{
	GPIOSetPinLevel(EPD_PWR_ON, GPIO_HIGH);
	msleep(1);

	GPIOSetPinLevel(EBOOK_PWRALL, GPIO_HIGH);
	GPIOSetPinLevel(EBOOK_SHR,GPIO_HIGH);
	GPIOSetPinLevel(EBOOK_GDRL,GPIO_HIGH);
	msleep(16);

	GPIOSetPinLevel(EBOOK_DACLK,GPIO_HIGH);
	GPIOSetPinLevel(EBOOK_VCOM,GPIO_HIGH);
	msleep(10);

	GPIOSetPinLevel(EBOOK_DADIO,GPIO_HIGH);
	msleep(5);
}

void   Eink_s_power_down(void)
{
	GPIOSetPinLevel(EBOOK_VCOM,GPIO_LOW);
	msleep(5);
	GPIOSetPinLevel(EBOOK_DADIO,GPIO_LOW);
	msleep(6);
	GPIOSetPinLevel(EBOOK_DACLK,GPIO_LOW);
	msleep(16);
	GPIOSetPinLevel(EBOOK_PWRALL,GPIO_LOW);
	GPIOSetPinLevel(EPD_PWR_ON,GPIO_LOW);
}
EXPORT_SYMBOL(Eink_s_power_on);
EXPORT_SYMBOL(Eink_s_power_down);







