
#include <linux/kernel.h>

/*
 * You may set default region here.
 * 0x10 -- China / America
 */
int wifi_default_region  = 0x10; // Country / region code

/*
 * Random MAC will be generated in WiFi driver if
 * this variable is set to 1. - For AR6102.
 */
int wifi_random_mac		 = 1;

/*
 * For AR6102, external eeprom may be used.
 */
int wifi_external_eeprom = 0;

/*
 * When WiFi is IDLE in 2 minutes, we will put WiFi
 * into deep sleep for power saving.
 */
unsigned long driver_ps_timeout = 2 * 60 * 1000; //2 minutes 



