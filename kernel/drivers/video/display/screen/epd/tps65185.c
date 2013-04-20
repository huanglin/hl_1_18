/*
 * Papyrus epaper power control HAL
 *
 *      Copyright (C) 2009 Dimitar Dimitrov, MM Solutions
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 * TPS6518x power control is facilitated using I2C control and WAKEUP GPIO
 * pin control. The other VCC GPIO Papyrus' signals must be tied to ground.
 *
 * TODO:
 * 	- Instead of polling, use interrupts to signal power up/down
 * 	  acknowledge.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <asm/arch/iomux.h>
#include "eink_s.h"
//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31))
//#include <linux/i2c/pmic-tps65185-i2c.h>
//#else
//#define PAPYRUS2_1P0_I2C_ADDRESS		0x48
//#define PAPYRUS2_1P1_I2C_ADDRESS		0x68
//extern void papyrus_set_i2c_address(int address);
//#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)
  #include <asm/gpio.h>
#else
  #include <linux/gpio.h>
#endif

#include "pmic.h"

#define TPS65185_I2C_NAME "tps65185"

#define PAPYRUS_VCOM_MAX_MV		0
#define PAPYRUS_VCOM_MIN_MV		-5110

#if 0
#define tps65185_printk(fmt, args...) printk(KERN_INFO "[tps] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define tps65185_printk(fmt, args...) 
#endif

/* After waking up from sleep, Papyrus
   waits for VN to be discharged and all
   voltage ref to startup before loading
   the default EEPROM settings. So accessing
   registers too early after WAKEUP could
   cause the register to be overridden by
   default values */
#define PAPYRUS_EEPROM_DELAY_MS 10//50
/* Papyrus WAKEUP pin must stay low for
   a minimum time */
#define PAPYRUS_SLEEP_MINIMUM_MS 10//110
/* Temp sensor might take a little time to
   settle eventhough the status bit in TMST1
   state conversion is done - if read too early
   0C will be returned instead of the right temp */
#define PAPYRUS_TEMP_READ_TIME_MS 10

/* Powerup sequence takes at least 24 ms - no need to poll too frequently */
#define HW_GET_STATE_INTERVAL_MS 24

struct papyrus_sess {
	struct i2c_adapter *adap;
	struct i2c_client *client;
	uint8_t enable_reg_shadow;
	uint8_t enable_reg;
	uint8_t vadj;
	uint8_t vcom1;
	uint8_t vcom2;
	uint8_t vcom2off;
	uint8_t int_en1;
	uint8_t int_en2;
	uint8_t upseq0;
	uint8_t upseq1;
	uint8_t dwnseq0;
	uint8_t dwnseq1;
	uint8_t tmst1;
	uint8_t tmst2;

	/* Custom power up/down sequence settings */
	struct {
		/* If options are not valid we will rely on HW defaults. */
		bool valid;
		unsigned int dly[8];
	} seq;
	struct timeval standby_tv;
	unsigned int v3p3off_time_ms;

	/* True if a high WAKEUP brings Papyrus out of reset. */
	int wakeup_active_high;
};


#define WAKEUP_GPIO		(GPIOPortG_Pin3)	/* active high */
//#define CPLD_RESET_GPIO		(88)	/* active low */
#define EN_CPLD_POW_GPIO	(GPIOPortG_Pin5)	/* active high */

#define  VCOM_CTRL_GPIO      (GPIOPortC_Pin6)
//#define  SHR_CTRL_GPIO       (GPIOPortC_Pin5)
//#define  GTMODE_CTRL_GPIO       (GPIOPortC_Pin4)

#define PWR_GOOD		       			GPIOPortG_Pin6
#define PWR_GOOD_IOMUX_PINNAME		GPIOG_MMC1_SEL_NAME
#define PWR_GOOD_IOMUX_PINDIR		0

#define PWR_INT		       				GPIOPortG_Pin4
#define PWR_INT_IOMUX_PINNAME		GPIOG_MMC1_SEL_NAME
#define PWR_INT_IOMUX_PINDIR			0


#define PAPYRUS_ADDR_TMST_VALUE		0x00
#define PAPYRUS_ADDR_ENABLE		0x01
#define PAPYRUS_ADDR_VADJ		0x02
#define PAPYRUS_ADDR_VCOM1_ADJUST	0x03
#define PAPYRUS_ADDR_VCOM2_ADJUST	0x04
#define PAPYRUS_ADDR_INT_ENABLE1	0x05
#define PAPYRUS_ADDR_INT_ENABLE2	0x06
#define PAPYRUS_ADDR_INT_STATUS1	0x07
#define PAPYRUS_ADDR_INT_STATUS2	0x08
#define PAPYRUS_ADDR_UPSEQ0		0x09
#define PAPYRUS_ADDR_UPSEQ1		0x0a
#define PAPYRUS_ADDR_DWNSEQ0		0x0b
#define PAPYRUS_ADDR_DWNSEQ1		0x0c
#define PAPYRUS_ADDR_TMST1		0x0d
#define PAPYRUS_ADDR_TMST2		0x0e
#define PAPYRUS_ADDR_PG_STATUS	0x0f
#define PAPYRUS_ADDR_REVID		0x10


#define PAPYRUS_MV_TO_VCOMREG(MV)	((MV) / 10)
#define V3P3_EN_MASK	0x20
#define PAPYRUS_V3P3OFF_DELAY_MS 100

struct papyrus_hw_state {
	uint8_t tmst_value;
	uint8_t int_status1;
	uint8_t int_status2;
	uint8_t pg_status;
};

static uint8_t papyrus2_i2c_addr = 0x68;//PAPYRUS2_1P1_I2C_ADDRESS;
static struct i2c_client *tps65185_client = NULL;
struct pmic_sess pmic_sess_data;
static int papyrus_hw_setreg(struct papyrus_sess *sess, uint8_t regaddr, uint8_t val)
{
	int stat;
	uint8_t txbuf[2] = { regaddr, val };
	struct i2c_msg msgs[] = {
		{
			.addr = tps65185_client->addr,
			.flags = 0,
			.len = 2,
			.buf = txbuf,
		}
	};

	stat = i2c_transfer(tps65185_client->adapter, msgs, ARRAY_SIZE(msgs));
	if (stat < 0){
		pr_err("papyrus: I2C send error: %d\n", stat);
   	}
	else if (stat != msgs->len) {
		pr_err("papyrus: I2C sned N mismatch: %d\n", stat);
		stat = -EIO;
	} else
		stat = 0;
	return stat;
	
}


static int papyrus_hw_getreg(struct papyrus_sess *sess, uint8_t regaddr, uint8_t *val)
{
	int stat;
	struct i2c_msg msg[1];	
	u8 buf[2];
	msg->addr = tps65185_client->addr;
	msg->flags |= I2C_M_RD;
	msg->buf = buf;
	msg->len = 1;
	buf[0] = regaddr;
	stat = i2c_transfer(tps65185_client->adapter, msg, 1);
	if (stat < 0){
		pr_err("papyrus: I2C read error: %d\n", stat);
       	pr_err("Papyrus i2c addr %x %s\n",papyrus2_i2c_addr,__FILE__); 
   	}
	else if (stat != msg->len) {
		pr_err("papyrus: I2C read N mismatch: %d\n", stat);
		stat = -EIO;
	} else
		stat = 0;
	*val = buf[0];
	return stat;
}


static void papyrus_hw_get_pg(struct papyrus_sess *sess,
							  struct papyrus_hw_state *hwst)
{
	int stat;

	stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_PG_STATUS, &hwst->pg_status);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
}

/*
static void papyrus_hw_get_state(struct papyrus_sess *sess, struct papyrus_hw_state *hwst)
{
	int stat;

	stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_TMST_VALUE, &hwst->tmst_value);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_INT_STATUS1, &hwst->int_status1);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_INT_STATUS2, &hwst->int_status2);
	stat |= papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_PG_STATUS, &hwst->pg_status);
	if (stat)
		pr_err("papyrus: I2C error: %d\n", stat);
}
*/

static void papyrus_hw_send_powerup(struct papyrus_sess *sess)
{
	int stat = 0;

	/* enable CPLD */
	//gpio_direction_output(CPLD_RESET_GPIO, 0);
	//gpio_direction_output(EN_CPLD_POW_GPIO, GPIO_HIGH);
	GPIOSetPinLevel(EN_CPLD_POW_GPIO,GPIO_HIGH);
	//gpio_direction_output(CPLD_RESET_GPIO, 1);

	/* set VADJ */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_VADJ, sess->vadj);
	//printk("sess->upseq0=%x,stat=%d\n",sess->upseq0,stat);
	/* set UPSEQs */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ0, sess->upseq0);
	//printk("sess->vadj=%x,stat=%d\n",sess->vadj,stat);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_UPSEQ1, sess->upseq1);
	//printk("sess->upseq1=%x\n,stat=%d\n",sess->upseq1,stat);
	/* set DWNSEQs */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ0, sess->dwnseq0);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_DWNSEQ1, sess->dwnseq1);
	/* set TMSTs */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_TMST2, sess->tmst2);
	/* set INT_ENABLEs */
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_INT_ENABLE1, sess->int_en1);
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_INT_ENABLE2, sess->int_en2);
	/* Enable 3.3V switch to the panel */
	sess->enable_reg_shadow |= V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);
	/* switch to active mode, VCOM buffer disabled */
	sess->enable_reg_shadow = 0xaf;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);
	if (stat)
		pr_err("papyrus_hw_send_powerup: I2C error: %d\n", stat);
}


static void papyrus_hw_send_powerdown(struct papyrus_sess *sess)
{
	int stat = 0;

	/* keep XXX_PWR_EN signals enabled and activate STANDBY */
	sess->enable_reg_shadow = 0x6f;
	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);

	msleep(sess->v3p3off_time_ms);

	/* 3.3V switch must be turned off last */
	sess->enable_reg_shadow &= ~V3P3_EN_MASK;
	stat |= papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);

	/* disable CPLD */
	//gpio_direction_output(CPLD_RESET_GPIO, 0);
	//gpio_direction_output(EN_CPLD_POW_GPIO,GPIO_LOW);
	GPIOSetPinLevel(EN_CPLD_POW_GPIO,GPIO_LOW);

	if (stat)
		pr_err("papyrus_hw_send_powerup: I2C error: %d\n", stat);

	do_gettimeofday(&sess->standby_tv);
}

static int papyrus_hw_get_revid(struct papyrus_sess *sess)
{
	int stat;
	uint8_t revid;
	papyrus_hw_getreg(sess, PAPYRUS_ADDR_REVID, &revid);
	if (stat) {
		pr_err("papyrus: I2C error: %d\n", stat);
	}
}

void papyrus_set_i2c_address(int address)
{

}

static int papyrus_hw_arg_init(struct papyrus_sess *sess)
{
	sess->vadj = 0x03;

	sess->upseq0 = 0xe4;
	sess->upseq1 = 0x55;

	sess->dwnseq0 = 0x1e;
	sess->dwnseq1 = 0xe0;

	return 0;
}


static int papyrus_hw_init(struct papyrus_sess *sess, const char *chip_id)
{
	int stat = 0;


	
	rockchip_mux_api_set(GPIOC_LCDC18BIT_SEL_NAME, 1);
	rockchip_mux_api_set(GPIOC_LCDC24BIT_SEL_NAME, 0);


	rockchip_mux_api_set(PWR_GOOD_IOMUX_PINNAME, PWR_GOOD_IOMUX_PINDIR);
	GPIOPullUpDown(PWR_GOOD, GPIONormal);
	GPIOSetPinDirection(PWR_GOOD, GPIO_IN);
	GPIOPullUpDown(PWR_INT, GPIONormal);
	GPIOSetPinDirection(PWR_INT, GPIO_IN);
	


	
	GPIOSetPinDirection(VCOM_CTRL_GPIO, GPIO_OUT);
	GPIOSetPinLevel(VCOM_CTRL_GPIO,GPIO_HIGH);
	
	
	
	
	GPIOSetPinDirection(EN_CPLD_POW_GPIO, GPIO_OUT);
	GPIOSetPinLevel(EN_CPLD_POW_GPIO,GPIO_LOW);
	sess->wakeup_active_high = 1;
	GPIOSetPinDirection(WAKEUP_GPIO, GPIO_OUT);
	GPIOSetPinLevel(WAKEUP_GPIO,GPIO_LOW);
	/* wait to reset papyrus */
	msleep(PAPYRUS_SLEEP_MINIMUM_MS);
	GPIOSetPinLevel(WAKEUP_GPIO,GPIO_HIGH);
	msleep(PAPYRUS_EEPROM_DELAY_MS);
	//stat = papyrus_hw_arg_init(sess);
	stat |= papyrus_hw_get_revid(sess);
	pr_info("papyrus: detected device with ID=%02x (TPS6518%dr%dp%d)\n",
			stat, stat & 0xF, (stat & 0xC0) >> 6, (stat & 0x30) >> 4);
	return stat;
}

static int papyrus_hw_read_temperature(struct pmic_sess *pmsess, int *t)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	int stat;
	int ntries = 50;
	uint8_t tb;

	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_TMST1, 0x80);
	do {
		stat = papyrus_hw_getreg(sess,
				PAPYRUS_ADDR_TMST1, &tb);
	} while ( !stat &&ntries-- && (((tb & 0x20) == 0) || (tb & 0x80)));
	if (stat)
		return stat;
	msleep(PAPYRUS_TEMP_READ_TIME_MS);
	stat = papyrus_hw_getreg(sess, PAPYRUS_ADDR_TMST_VALUE, &tb);
	*t = (int)(int8_t)tb;
	tps65185_printk("current temperature is %d\n",*t);

	return stat;
}


static void papyrus_hw_power_req(struct pmic_sess *pmsess, bool up)
{
	struct papyrus_sess *sess = pmsess->drvpar;

	pr_debug("papyrus: i2c pwr req: %d\n", up);
	if (up)
		papyrus_hw_send_powerup(sess);
	else
		papyrus_hw_send_powerdown(sess);
}


static bool papyrus_hw_power_ack(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	struct papyrus_hw_state hwst;
	int st;
	int retries_left = 10;

	do {
		papyrus_hw_get_pg(sess, &hwst);

		pr_debug("hwst: tmst_val=%d, ist1=%02x, ist2=%02x, pg=%02x\n",
				hwst.tmst_value, hwst.int_status1,
				hwst.int_status2, hwst.pg_status);
		hwst.pg_status &= 0xfa;
		if (hwst.pg_status == 0xfa)
			st = 1;
		else if (hwst.pg_status == 0x00)
			st = 0;
		else {
			st = -1;	/* not settled yet */
			msleep(HW_GET_STATE_INTERVAL_MS);
		}
		retries_left--;
	} while ((st == -1) && retries_left);

	if ((st == -1) && !retries_left)
		pr_err("papyrus: power up/down settle error (PG = %02x)\n", hwst.pg_status);

	return !!st;
}


static void papyrus_hw_cleanup(struct papyrus_sess *sess)
{
}


/* -------------------------------------------------------------------------*/

static int papyrus_set_enable(struct pmic_sess *pmsess, int enable)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->enable_reg = enable;
	return 0;
}

static int papyrus_set_vcom_voltage(struct pmic_sess *pmsess, int vcom_mv)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG(-vcom_mv) & 0x00FF);
	sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG(-vcom_mv) & 0x0100) >> 8);
	return 0;
}

static int papyrus_set_vcom1(struct pmic_sess *pmsess, uint8_t vcom1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vcom1 = vcom1;
	return 0;
}

static int papyrus_set_vcom2(struct pmic_sess *pmsess, uint8_t vcom2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	// TODO; Remove this temporary solution to set custom vcom-off mode
	//       Add PMIC setting when this is to be a permanent feature
	pr_debug("papyrus_set_vcom2 vcom2off 0x%02x\n", vcom2);
	sess->vcom2off = vcom2;
	return 0;
}

static int papyrus_set_vadj(struct pmic_sess *pmsess, uint8_t vadj)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->vadj = vadj;
	return 0;
}

static int papyrus_set_int_en1(struct pmic_sess *pmsess, uint8_t int_en1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->int_en1 = int_en1;
	return 0;
}

static int papyrus_set_int_en2(struct pmic_sess *pmsess, uint8_t int_en2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->int_en2 = int_en2;
	return 0;
}

static int papyrus_set_upseq0(struct pmic_sess *pmsess, uint8_t upseq0)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->upseq0 = upseq0;
	return 0;
}

static int papyrus_set_upseq1(struct pmic_sess *pmsess, uint8_t upseq1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->upseq1 = upseq1;
	return 0;
}

static int papyrus_set_dwnseq0(struct pmic_sess *pmsess, uint8_t dwnseq0)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->dwnseq0 = dwnseq0;
	return 0;
}

static int papyrus_set_dwnseq1(struct pmic_sess *pmsess, uint8_t dwnseq1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->dwnseq1 = dwnseq1;
	return 0;
}

static int papyrus_set_tmst1(struct pmic_sess *pmsess, uint8_t tmst1)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->tmst1 = tmst1;
	return 0;
}

static int papyrus_set_tmst2(struct pmic_sess *pmsess, uint8_t tmst2)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	sess->tmst2 = tmst2;
	return 0;
}

static int papyrus_vcom_switch(struct pmic_sess *pmsess, bool state)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	int stat;

	sess->enable_reg_shadow &= ~((1u << 4) | (1u << 6) | (1u << 7));
	sess->enable_reg_shadow |= (state ? 1u : 0) << 4;

	stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_ENABLE,
						sess->enable_reg_shadow);

	/* set VCOM off output */
	if (!state && sess->vcom2off != 0) {
		stat = papyrus_hw_setreg(sess, PAPYRUS_ADDR_VCOM2_ADJUST,
						sess->vcom2off);
	}

	return stat;
}

static bool papyrus_standby_dwell_time_ready(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;
	struct timeval current_tv;
	long total_secs;

	do_gettimeofday(&current_tv);
mb();
	total_secs = current_tv.tv_sec - sess->standby_tv.tv_sec;

	if (total_secs < PAPYRUS_STANDBY_DWELL_TIME)
		return false;

	return true;
}

static void papyrus_pm_sleep(struct pmic_sess *sess)
{
#if !defined(FB_OMAP3EP_PAPYRUS_PM_STANDBY)
	struct papyrus_sess *s = sess->drvpar;
	//gpio_direction_output(WAKEUP_GPIO, !s->wakeup_active_high);
	GPIOSetPinLevel(WAKEUP_GPIO, !s->wakeup_active_high);
	//GPIOSetPinLevel(EN_CPLD_POW_GPIO,GPIO_LOW);
	msleep(PAPYRUS_SLEEP_MINIMUM_MS);
#endif
	pr_debug("%s\n", __func__);
}

static void papyrus_pm_resume(struct pmic_sess *sess)
{
#if !defined(FB_OMAP3EP_PAPYRUS_PM_STANDBY)
	struct papyrus_sess *s = sess->drvpar;
	//gpio_direction_output(WAKEUP_GPIO, s->wakeup_active_high);
	GPIOSetPinLevel(WAKEUP_GPIO, s->wakeup_active_high);
	//GPIOSetPinLevel(EN_CPLD_POW_GPIO,GPIO_HIGH);
	msleep(PAPYRUS_EEPROM_DELAY_MS);
#endif
	pr_debug("%s\n", __func__);
}

static int papyrus_probe(struct pmic_sess *pmsess,struct i2c_client *client)
{
	struct papyrus_sess *sess;
	int stat;
	sess = kzalloc(sizeof(*sess), GFP_KERNEL);
	if (!sess)
		return -ENOMEM;
	sess->vcom1 = (PAPYRUS_MV_TO_VCOMREG(2800) & 0x00FF);
	sess->vcom2 = ((PAPYRUS_MV_TO_VCOMREG(2800) & 0x0100) >> 8);
	//papyrus_hw_arg_init(sess);
	if (pmsess->v3p3off_time_ms == -1)
		sess->v3p3off_time_ms = PAPYRUS_V3P3OFF_DELAY_MS;
	else
		sess->v3p3off_time_ms = pmsess->v3p3off_time_ms;

	do_gettimeofday(&sess->standby_tv);

	stat = papyrus_hw_init(sess, pmsess->drv->id);
	if (stat)
		goto free_sess;
	pmsess->drvpar = sess;
	pmsess->revision = papyrus_hw_get_revid(sess);
	return 0;

free_sess:
	kfree(sess);

	return stat;
}

static void papyrus_remove(struct pmic_sess *pmsess)
{
	struct papyrus_sess *sess = pmsess->drvpar;

	papyrus_hw_cleanup(sess);

	kfree(sess);
	pmsess->drvpar = 0;
}

const struct pmic_driver pmic_driver_tps65185_i2c = {
	.id = "tps65185-i2c",

	.vcom_min = PAPYRUS_VCOM_MIN_MV,
	.vcom_max = PAPYRUS_VCOM_MAX_MV,
	.vcom_step = 10,

	.hw_read_temperature = papyrus_hw_read_temperature,
	.hw_power_ack = papyrus_hw_power_ack,
	.hw_power_req = papyrus_hw_power_req,

	.set_enable = papyrus_set_enable,
	.set_vcom_voltage = papyrus_set_vcom_voltage,
	.set_vcom1 = papyrus_set_vcom1,
	.set_vcom2 = papyrus_set_vcom2,
	.set_vadj = papyrus_set_vadj,
	.set_int_en1 = papyrus_set_int_en1,
	.set_int_en2 = papyrus_set_int_en2,
	.set_upseq0 = papyrus_set_upseq0,
	.set_upseq1 = papyrus_set_upseq1,
	.set_dwnseq0 = papyrus_set_dwnseq0,
	.set_dwnseq1 = papyrus_set_dwnseq1,
	.set_tmst1 = papyrus_set_tmst1,
	.set_tmst2 = papyrus_set_tmst2,

	.hw_vcom_switch = papyrus_vcom_switch,

	.hw_init = papyrus_probe,
	.hw_cleanup = papyrus_remove,

	.hw_standby_dwell_time_ready = papyrus_standby_dwell_time_ready,
	.hw_pm_sleep = papyrus_pm_sleep,
	.hw_pm_resume = papyrus_pm_resume,
};


static unsigned short normal_i2c[] = {0x68, I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static int tps65185_probe(struct i2c_adapter *adapter, int addr, int kind);
static struct i2c_client_address_data addr_data_tps65185 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

static int epd_tps65185_attach(struct i2c_adapter *adapter)
{
    return i2c_probe(adapter, &addr_data_tps65185, tps65185_probe);
}

static int epd_tps65185_detach(struct i2c_client *client)
{
	
}

static void tps65185_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pmic_driver_tps65185_i2c.hw_pm_sleep((struct pmic_sess *)&pmic_sess_data);
}
static int tps65185_resume(struct i2c_client *client)
{
	pmic_driver_tps65185_i2c.hw_pm_resume((struct pmic_sess *)&pmic_sess_data);
}
 int save_vcom_voltage(int voltage)
{
	struct papyrus_sess sess ;
	char revid;
	char status = 0;
	int stat;
	int ntries = 50;
	GPIOSetPinDirection(EN_CPLD_POW_GPIO, GPIO_OUT);
	GPIOSetPinLevel(EN_CPLD_POW_GPIO,GPIO_LOW);
	GPIOSetPinDirection(WAKEUP_GPIO, GPIO_OUT);
	GPIOSetPinLevel(WAKEUP_GPIO,GPIO_LOW);
	msleep(PAPYRUS_SLEEP_MINIMUM_MS);
	GPIOSetPinLevel(WAKEUP_GPIO,GPIO_HIGH);
	msleep(PAPYRUS_EEPROM_DELAY_MS);
	
	sess.vcom1 = (PAPYRUS_MV_TO_VCOMREG(voltage) & 0x00FF);
	sess.vcom2 = ((PAPYRUS_MV_TO_VCOMREG(voltage) & 0x0100) >> 8)|0x40;
	
	stat = papyrus_hw_setreg(&sess, PAPYRUS_ADDR_VCOM1_ADJUST,
							  sess.vcom1);
	stat |=papyrus_hw_setreg(&sess, PAPYRUS_ADDR_VCOM2_ADJUST,
							 sess.vcom2);
	if(stat){
		printk("save_vcom_voltage I2C ERR,return\n");
		return -1;
	}
	papyrus_hw_getreg(&sess, PAPYRUS_ADDR_INT_STATUS1, &status);
	while(((status&0x01) == 0) && (ntries--)){
		printk("wait program bit ,status =%d\n",status);
		papyrus_hw_getreg(&sess, PAPYRUS_ADDR_INT_STATUS1, &status);
		mdelay(100);
	}
	if(ntries == 0){
		printk("vcom program err,return\n");
		return -1;
	}
	GPIOSetPinDirection(WAKEUP_GPIO, GPIO_OUT);
	GPIOSetPinLevel(WAKEUP_GPIO,GPIO_LOW);
	msleep(PAPYRUS_SLEEP_MINIMUM_MS);
	return 0;
}

EXPORT_SYMBOL(save_vcom_voltage);	

static struct i2c_driver tps65185_driver = {
	.driver = {
		.name	  = TPS65185_I2C_NAME,
		.owner	  = THIS_MODULE,
	},
	.suspend        = tps65185_suspend,
	.resume         = tps65185_resume,
	.attach_adapter	= epd_tps65185_attach,
	.detach_client	= epd_tps65185_detach,
};

static int tps65185_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	int temp;
	int rc = 0;
	struct i2c_client *client = NULL;
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) 
	{
		printk("I2C check functionality failed.");
		return -ENODEV;
	}
	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (client == NULL) {
		rc = -ENOMEM;
		goto failout;
	}
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->addr = addr;
	client->adapter = adapter;
	client->speed = 200;
	client->driver = &tps65185_driver;
	strlcpy(client->name, "tps65185", I2C_NAME_SIZE);

	rc = i2c_attach_client(client);
	if (rc < 0)
		goto failout;
	tps65185_client = client;
	//save_vcom_voltage();
	return 0;
	failout:
	kfree(client);
	return rc;
}

static int tps65185_remove(struct i2c_client *client)
{
	pmic_driver_tps65185_i2c.hw_cleanup((struct pmic_sess *)&pmic_sess_data);
	memset(&pmic_sess_data,0,sizeof(struct pmic_sess));
	return 0;
}


static int __init tps65185_init(void)
{
	return i2c_add_driver(&tps65185_driver);
}

static void __exit tps65185_exit(void)
{
	return i2c_del_driver(&tps65185_driver);
}

subsys_initcall(tps65185_init);
module_exit(tps65185_exit);

MODULE_DESCRIPTION("ti tps65185 pmic");
MODULE_LICENSE("GPL");
struct epd_sensor tps65185_sensor;




void temp_get(int *value)
{
	int stat = 0;
	stat = papyrus_hw_read_temperature((struct pmic_sess *)&pmic_sess_data,value);
	if(stat){
		printk("read temperture err,set temperture = 0 \n");
		*value = 0;
	}
}
void temp_release()
{
}
static struct epd_sensor_ops tps65182_sensor_ops = {
	.get_value	= temp_get,
	.release		= temp_release,
};
void Eink_S_power_init()
{
	if(0 != pmic_driver_tps65185_i2c.hw_init((struct pmic_sess *)&pmic_sess_data,tps65185_client))
	{
		printk("pmic_driver_tps65185_i2c hw_init failed.");
		return -ENODEV;
	}	
	pmic_sess_data.is_inited = 1;
	tps65185_sensor.ops =  &tps65182_sensor_ops;
	epd_register_sensor(&tps65185_sensor);
	tps65185_printk("tps65185_probe_after_ebc ok.\n");
}
void   Eink_s_power_on(void)
{
	if(pmic_sess_data.is_inited)
		pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,1);
}

void   Eink_s_power_down(void)
{
	if(pmic_sess_data.is_inited)
		pmic_driver_tps65185_i2c.hw_power_req((struct pmic_sess *)&pmic_sess_data,0);
}
EXPORT_SYMBOL(Eink_s_power_on);
EXPORT_SYMBOL(Eink_s_power_down);









