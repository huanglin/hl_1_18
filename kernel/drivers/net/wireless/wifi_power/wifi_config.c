
#include <linux/kernel.h>

/*
 * You may set default region here.
 * 0x10 -- China / America
 */
int wifi_default_region  = 0x10; // Country / region code

/*
 * Please choose the source for mac address.
 *
 *   For the following modules, an eeprom is embedded in module,
 *   so don't care this variable:
 *     01. Samsung SWL-2480
 *   
 *   For the following modules, you need to set the correct value:
 *     01. Atheros AR6102 / AR6122
 */
#define MAC_ADDR_FROM_FILE		0 /* from system/etc/firmware/softmac */
#define MAC_ADDR_FROM_RANDOM	1 /* pseudo-random mac */
#define MAC_ADDR_FROM_EEPROM	2 /* If there is an external eeprom */

int wifi_mac_addr_source = MAC_ADDR_FROM_RANDOM;

/*
 * When WiFi is IDLE in 2 minutes, we will put WiFi
 * into deep sleep for power saving.
 */
unsigned long driver_ps_timeout = 2 * 60 * 1000; //2 minutes 



