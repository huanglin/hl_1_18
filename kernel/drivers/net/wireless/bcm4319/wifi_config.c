
#include <linux/kernel.h>

/*********************************************************************/
/**************** For Broadcom Modules *******************************/
/*********************************************************************/

/*
 * Channel list define. No other options are valid!!
 */
#define CHANNEL_NUMBER_11	11
#define CHANNEL_NUMBER_13	13
#define CHANNEL_NUMBER_14	14
int bcm_channel_number = CHANNEL_NUMBER_13;

/*
 * This definition helps driver to decide which FW and NVRAM should 
 * be used.
 */
#define BCM4319_USI_WMNBM01S         0x100
#define BCM4319_SAMSUNG_SWLB33       0x101
#define BCM4319_CYBERTAN_WC160M      0x102

int bcm_module = BCM4319_USI_WMNBM01S;

#define ANDROID_FW_PATH "/system/etc/firmware/"
#define TEMP_FW_PATH "/flash/"

int bcm4319_set_customer_firmware(char *fw, char *nvram)
{
#if 0
	sprintf(fw, "%s%s", ANDROID_FW_PATH, "bcm4319_fw.bin");
	sprintf(nvram, "%s%s", ANDROID_FW_PATH, "bcm4319_nvram_wc160m.txt");
	
	return 1;
#else
	return 0;
#endif
}

unsigned char wlan_mac_addr[6];

