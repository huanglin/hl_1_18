#ifndef _TPS65180B_H
#define _TPS65180B_H


#define  TPS_TMST_VALUE		0		/*  Thermistor value read by ADC	*/
#define  TPS_ENABLE      		1		/*  Enable/disable bits for regulators	*/
#define  TPS_VP_ADJUST		2		/*  Voltage settings for VPOS, VDDH	*/
#define  TPS_VN_ADJUST		3		/*  Voltage settings for VNEG, VEE	*/

#define  TPS_VCOM_ADJUST	4		/*  Voltage settings for VCOM		*/
#define  TPS_INT_ENABLE1	5		/*  Interrupt enable group1			*/
#define  TPS_INT_ENABLE2	6		/*  Interrupt enable group2			*/
#define  TPS_INT_STATUS1	7		/*  Interrupt status group1			*/
#define  TPS_INT_STATUS2	8		/*  Interrupt status group2			*/
#define  TPS_PWR_SEQ0		9		/*  Power up sequence				*/
#define  TPS_PWR_SEQ1		10		/*  DLY0, DLY1 time set				*/
#define  TPS_PWR_SEQ2		11		/*  DLY2, DLY3 time set				*/

#define  TPS_TMST_CONFIG	12		/*  Thermistor configuration			*/
#define  TPS_TMST_OS		13		/*  Thermistor hot temp set			*/
#define  TPS_TMST_HYST		14		/*  Thermistor cool temp set			*/
#define  TPS_PG_STATUS		15		/*  Power good status each rails		*/
#define  TPS_REVID			16		/*  Device revision ID information	*/
#define  TPS_FIX_READ_POINTER	17		/*  tps65181b only 		*/



extern	int	tps_i2c_set_regs( unsigned char  reg, unsigned char const buf[], unsigned short  len);
extern	int	tps_i2c_read_regs(unsigned char reg, unsigned char buf[], unsigned short len);


#endif	/* _TPS65180B_H */

