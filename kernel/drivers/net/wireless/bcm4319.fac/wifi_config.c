
#include <linux/kernel.h>

/*********************************************************************/
/**************** For every WiFi Chip ********************************/
/*********************************************************************/

/*
 * When WiFi is IDLE with disconnected state as long as
 * wifi_closed_timeout, we will ask Android to turn off WiFi.
 *
 * 0 - means disable this function.
 *
 * Normally, don't set the value less than 6, it maybe cause some
 * unknown issue!
 */
int wifi_turnoff_timeout = 12; //unit is 10 seconds, ex. 12 means 120 seconds

/*
 * When WiFi is IDLE in 2 minutes, we will put WiFi
 * into deep sleep for power saving.
 */
unsigned long driver_ps_timeout = 2 * 60 * 1000; //2 minutes 

/*********************************************************************/
/**************** For MV8686 *****************************************/
/*********************************************************************/

/*
 * You may set default region code here.
 *
 * 0x10 -- America / Canada (Channel 1-11)
 * 0x30 -- Europe (Channel 1-13)
 * 0x31 -- Spain (Channel 10-11)
 * 0x32 -- France (Channel 10-13)
 * 0x33 -- Japan (Channel 1-14)
 */
int wifi_default_region  = 0x30; 

/*********************************************************************/
/**************** For Atheros AR6002 *********************************/
/*********************************************************************/

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
#define MAC_ADDR_FROM_SOFTMAC_FILE	0 /* from system/etc/firmware/softmac */
#define MAC_ADDR_FROM_RANDOM		1 /* pseudo-random mac */
#define MAC_ADDR_FROM_EEPROM		2 /* If there is an external eeprom */
#define MAC_ADDR_FROM_EEPROM_FILE	3 /* from system/etc/firmware/calData_ar6102_15dBm.bin */

int wifi_mac_addr_source = MAC_ADDR_FROM_RANDOM;

/*
 * User customized MAC address in IDB block.
 *
 * Return value: 0 -- not customized, 1 -- customized.
 */
#if 1
int wifi_customized_mac_addr(u8 *mac)
{
	return 0;
}
#else
extern int kld_get_wifi_mac(u8 *mac);

int wifi_customized_mac_addr(u8 *mac)
{
	kld_get_wifi_mac(mac);

	printk("We are using customized MAC: %02X:%02X:%02X:%02X:%02x:%02x\n",
	       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	
	return 1;
}
#endif

/*********************************************************************/
/**************** For Broadcom Modules *******************************/
/*********************************************************************/

/*
 * This definition helps driver to decide which FW and NVRAM should 
 * be used.
 */
#define BCM4319_USI_WMNBM01S         0x100
#define BCM4319_SAMSUNG_SWLB33       0x101
#define BCM4319_CYBERTAN_WC160M      0x102

#define BCM4329_CYBERTAN_NC023S      0x200
#define BCM4329_SAMSUNG_SWLB23       0x201

int bcm_module = BCM4319_CYBERTAN_WC160M;

