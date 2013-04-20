/*
 * Windfarm PowerMac thermal control. LM75 sensor
 *
 * (c) Copyright 2005 Benjamin Herrenschmidt, IBM Corp.
 *                    <benh@kernel.crashing.org>
 *
 * Released under the term of the GNU GPL v2.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/i2c.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/sections.h>

#include "eink_s.h"
//#include <asm/arch/rk28_define.h>


#if 0
#define lmprintk(args...)	printk(args)
#else
#define lmprintk(args...)	do { } while(0)
#endif


struct epd_lm75_sensor {
	int			ds1775 : 1;
	int			inited : 1;
	struct 	i2c_client	i2c;
	struct  epd_sensor	sens;
};

#define epd_to_lm75(c) container_of(c, struct epd_lm75_sensor, sens)
#define i2c_to_lm75(c) container_of(c, struct epd_lm75_sensor, i2c)

struct epd_sensor *pgepd_lm75 = NULL;

static int epd_lm75_attach(struct i2c_adapter *adapter);
static int epd_lm75_detach(struct i2c_client *client);

static struct i2c_driver epd_lm75_driver = {
	.driver = {
		.name	= "epd_lm75",
	},
	.attach_adapter	= epd_lm75_attach,
	.detach_client	= epd_lm75_detach,
};

static int epd_lm75_shut_down()
{
	struct epd_lm75_sensor *lm = epd_to_lm75(pgepd_lm75);
	struct i2c_client* client = &(lm->i2c);   
	int  ret;
	u8 buf[2];
	struct i2c_msg msg[1];

	lmprintk("in epd_lm75_shut_down\n ");
	msg->addr = client->addr;
	msg->flags = 0;
	msg->buf = buf;
	msg->len = sizeof(buf);
	buf[0] = 0x01;
	buf[1] = 0x01;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret != msg->len)
	{
		lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return 1;
	}
	return 0;
}
static int epd_lm75_wake_up()
{
	struct epd_lm75_sensor *lm = epd_to_lm75(pgepd_lm75);
	struct i2c_client* client = &(lm->i2c);   
	int  ret;
	u8 buf[2];
	struct i2c_msg msg[1];	

	lmprintk("in epd_lm75_wake_up\n ");
	msg->addr = client->addr;
	msg->flags = 0;
	msg->buf = buf;
	msg->len = sizeof(buf);
	buf[0] = 0x01;
	buf[1] = 0x00;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret != msg->len)
	{
		lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return 1;
	}
	return 0;
}
static int epd_lm75_get(s32 *value)
{
	struct epd_lm75_sensor *lm = epd_to_lm75(pgepd_lm75);
	struct i2c_client* client = &(lm->i2c);   
	int  ret;
	u8 buf[2];
	struct i2c_msg msg[1];	

	msg->addr = client->addr;
	msg->flags |= I2C_M_RD;
	msg->buf = buf;
	msg->len = sizeof(buf);
	buf[0] = 0;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret != msg->len)
	{
		lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return 1;
	}
	
	*value = buf[0]>>1;
	lmprintk("\n*************epd_lm75_get:reg=%d,value=%d\n",0,*value);
	return 0;
}

static void epd_lm75_release(void)
{
	struct epd_lm75_sensor *lm = epd_to_lm75(pgepd_lm75);

	/* check if client is registered and detach from i2c */
	if (lm->i2c.adapter) {
		i2c_detach_client(&lm->i2c);
		lm->i2c.adapter = NULL;
	}

	kfree(lm);
}

static struct epd_sensor_ops epd_lm75_ops = {
	.get_value	= epd_lm75_get,
	.release		= epd_lm75_release,
	.shutdown 	= epd_lm75_shut_down,
	.wakeup 		= epd_lm75_wake_up,
	.owner		= THIS_MODULE,
};

static struct epd_lm75_sensor *epd_lm75_create(struct i2c_adapter *adapter,
					     u8 addr, int ds1775,
					     const char *loc)
{
	struct epd_lm75_sensor *lm;
	int rc;

	lmprintk("epd_lm75: creating  %s device at address 0x%x\n",
		ds1775 ? "ds1775" : "lm75", addr);

	lm = kzalloc(sizeof(struct epd_lm75_sensor), GFP_KERNEL);
	if (lm == NULL)
		return NULL;

	lm->sens.name = "epd_lm75";
	lm->inited = 1;
	lm->sens.ops = &epd_lm75_ops;
	lm->ds1775 = ds1775;
	lm->i2c.addr = addr;
	lm->i2c.adapter = adapter;
	lm->i2c.driver = &epd_lm75_driver;
	lm->i2c.Channel = I2C_CH1;//
	lm->i2c.speed = 200;
	lm->i2c.mode = NORMALMODE;
    
	strncpy(lm->i2c.name, lm->sens.name, I2C_NAME_SIZE-1);
	rc = i2c_attach_client(&lm->i2c);
	if (rc) {
		lmprintk(KERN_ERR "failed to attach %s %s to i2c,"
		       " err %d\n", ds1775 ? "ds1775" : "lm75",
		       lm->i2c.name, rc);
		goto fail;
	}

	if (epd_register_sensor(&lm->sens)) {
		i2c_detach_client(&lm->i2c);
		goto fail;
	}
    pgepd_lm75 = &lm->sens;

	return lm;
 fail:
	kfree(lm);
	return NULL;
}

static unsigned short normal_i2c[] = {0x90 >> 1, I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;

static struct i2c_client_address_data addr_data_lm75 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};



static int epd_lm75_probe(struct i2c_adapter *adapter, int addr, int kind)
{   
	int ret;
	
	lmprintk("epd_lm75: epd_lm75_probe at address 0x%x\n",addr);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
	    ret = -EIO;
            return ret;
	}
	epd_lm75_create(adapter, addr, 0, NULL);
	
	return 0;
}

static int epd_lm75_attach(struct i2c_adapter *adapter)
{
    lmprintk("epd_lm75: epd_lm75_attach\n");
    return i2c_probe(adapter, &addr_data_lm75, epd_lm75_probe);
}

static int epd_lm75_detach(struct i2c_client *client)
{
	struct epd_lm75_sensor *lm = i2c_to_lm75(client);

	lmprintk("epd_lm75: i2c detatch called for %s\n", lm->sens.name);

	/* Mark client detached */
	lm->i2c.adapter = NULL;

	/* release sensor */
	epd_unregister_sensor(&lm->sens);

	return 0;
}

static int __init epd_lm75_sensor_init(void)
{
	 lmprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
	return i2c_add_driver(&epd_lm75_driver);
}

static void __exit epd_lm75_sensor_exit(void)
{
	i2c_del_driver(&epd_lm75_driver);
}

subsys_initcall(epd_lm75_sensor_init);

//fs_initcall_sync(epd_lm75_sensor_init);
module_exit(epd_lm75_sensor_exit);

MODULE_AUTHOR("Benjamin Herrenschmidt <benh@kernel.crashing.org>");
MODULE_DESCRIPTION("LM75 sensor objects for PowerMacs thermal control");
MODULE_LICENSE("GPL");

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_lm_show(struct seq_file *s, void *v)
{
    u32 value=0;
    epd_lm75_get(&value);
	//seq_printf(s, "\nTemperature is:");
	seq_printf(s, "%d\n", value);

	return 0;
}

static int proc_lm_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_lm_show, NULL);
}

static const struct file_operations proc_lm_fops = {
	.open		= proc_lm_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init lm_proc_init(void)
{
	proc_create("lm75", 0, NULL, &proc_lm_fops);
	return 0;

}
late_initcall(lm_proc_init);
#endif /* CONFIG_PROC_FS */

