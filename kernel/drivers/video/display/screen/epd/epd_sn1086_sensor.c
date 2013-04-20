/*
 * Windfarm PowerMac thermal control. sn1086 sensor
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


struct epd_sn1086_sensor {
	int			ds1775 : 1;
	int			inited : 1;
	struct 	i2c_client	i2c;
	struct  epd_sensor	sens;
};

#define epd_to_sn1086(c) container_of(c, struct epd_sn1086_sensor, sens)
#define i2c_to_sn1086(c) container_of(c, struct epd_sn1086_sensor, i2c)

struct epd_sensor *pgepd_sn1086 = NULL;

static int epd_sn1086_attach(struct i2c_adapter *adapter);
static int epd_sn1086_detach(struct i2c_client *client);

static struct i2c_driver epd_sn1086_driver = {
	.driver = {
		.name	= "epd_sn1086",
	},
	.attach_adapter	= epd_sn1086_attach,
	.detach_client	= epd_sn1086_detach,
};

static int epd_sn1086_shut_down()
{
	struct epd_sn1086_sensor *lm = epd_to_sn1086(pgepd_sn1086);
	struct i2c_client* client = &(lm->i2c);   
	int  ret;
	u8 buf[2];
	struct i2c_msg msg[1];

	lmprintk("in epd_sn1086_shut_down\n ");
	msg->addr = client->addr;
	msg->flags = 0;
	msg->buf = buf;
	msg->len = sizeof(buf);
	buf[0] = 0x09;
	buf[1] = 0x40;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret != msg->len)
	{
		lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return 1;
	}
	return 0;
}
static int epd_sn1086_wake_up()
{
	struct epd_sn1086_sensor *lm = epd_to_sn1086(pgepd_sn1086);
	struct i2c_client* client = &(lm->i2c);   
	int  ret;
	u8 buf[2];
	struct i2c_msg msg[1];	

	lmprintk("in epd_sn1086_wake_up\n ");
	msg->addr = client->addr;
	msg->flags = 0;
	msg->buf = buf;
	msg->len = sizeof(buf);
	buf[0] = 0x09;
	buf[1] = 0x00;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret != msg->len)
	{
		lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return 1;
	}
	return 0;
}
#if 1
static int epd_sn1086_status()
{
	struct epd_sn1086_sensor *lm = epd_to_sn1086(pgepd_sn1086);
	struct i2c_client* client = &(lm->i2c);   
	int  ret;
	u8 buf[2];
	struct i2c_msg msg[1];
	
	msg->addr = client->addr;
	msg->flags |= I2C_M_RD;
	msg->buf = buf;
	msg->len = sizeof(buf);
	buf[0]=0x02;
	ret = i2c_transfer(client->adapter, msg, 1);
	if(ret != msg->len)
	{
		lmprintk("%s: i2c_transfer ERR ret = %d",__FUNCTION__, ret);
		return 1;
	}
	lmprintk("\n*************epd_sn1086_get:reg=%d,value=%d\n",0,buf[0]);
}
#endif
static int epd_sn1086_get(s32 *value)
{
	struct epd_sn1086_sensor *lm = epd_to_sn1086(pgepd_sn1086);
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
	lmprintk("\n*************epd_sn1086_get:reg=%d,value=%d\n",0,buf[0]);
	*value = buf[0];
	
	return 0;
}

static void epd_sn1086_release(void)
{
	struct epd_sn1086_sensor *lm = epd_to_sn1086(pgepd_sn1086);

	/* check if client is registered and detach from i2c */
	if (lm->i2c.adapter) {
		i2c_detach_client(&lm->i2c);
		lm->i2c.adapter = NULL;
	}

	kfree(lm);
}

static struct epd_sensor_ops epd_sn1086_ops = {
	.get_value	= epd_sn1086_get,
	.release		= epd_sn1086_release,
	.shutdown 	= epd_sn1086_shut_down,
	.wakeup 		= epd_sn1086_wake_up,
	.owner		= THIS_MODULE,
};

static struct epd_sn1086_sensor *epd_sn1086_create(struct i2c_adapter *adapter,
					     u8 addr, int ds1775,
					     const char *loc)
{
	struct epd_sn1086_sensor *lm;
	int rc;

	lmprintk("epd_sn1086: creating  %s device at address 0x%x\n",
		ds1775 ? "ds1775" : "sn1086", addr);

	lm = kzalloc(sizeof(struct epd_sn1086_sensor), GFP_KERNEL);
	if (lm == NULL)
		return NULL;

	lm->sens.name = "epd_sn1086";
	lm->inited = 1;
	lm->sens.ops = &epd_sn1086_ops;
	lm->ds1775 = ds1775;
	lm->i2c.addr = addr;
	lm->i2c.adapter = adapter;
	lm->i2c.driver = &epd_sn1086_driver;
	lm->i2c.Channel = I2C_CH1;//
	lm->i2c.speed = 200;
	lm->i2c.mode = NORMALMODE;
    
	strncpy(lm->i2c.name, lm->sens.name, I2C_NAME_SIZE-1);
	rc = i2c_attach_client(&lm->i2c);
	if (rc) {
		lmprintk(KERN_ERR "failed to attach %s %s to i2c,"
		       " err %d\n", ds1775 ? "ds1775" : "sn1086",
		       lm->i2c.name, rc);
		goto fail;
	}

	if (epd_register_sensor(&lm->sens)) {
		i2c_detach_client(&lm->i2c);
		goto fail;
	}
    pgepd_sn1086 = &lm->sens;

	return lm;
 fail:
	kfree(lm);
	return NULL;
}

static unsigned short normal_i2c[] = {0x98 >> 1, I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;

static struct i2c_client_address_data addr_data_sn1086 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};



static int epd_sn1086_probe(struct i2c_adapter *adapter, int addr, int kind)
{   
	int ret;
	
	lmprintk("epd_sn1086: epd_sn1086_probe at address 0x%x\n",addr);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
	    ret = -EIO;
            return ret;
	}
	epd_sn1086_create(adapter, addr, 0, NULL);
	
	return 0;
}

static int epd_sn1086_attach(struct i2c_adapter *adapter)
{
    lmprintk("epd_sn1086: epd_sn1086_attach\n");
    return i2c_probe(adapter, &addr_data_sn1086, epd_sn1086_probe);
}

static int epd_sn1086_detach(struct i2c_client *client)
{
	struct epd_sn1086_sensor *lm = i2c_to_sn1086(client);

	lmprintk("epd_sn1086: i2c detatch called for %s\n", lm->sens.name);

	/* Mark client detached */
	lm->i2c.adapter = NULL;

	/* release sensor */
	epd_unregister_sensor(&lm->sens);

	return 0;
}

static int __init epd_sn1086_sensor_init(void)
{
	 lmprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
	return i2c_add_driver(&epd_sn1086_driver);
}

static void __exit epd_sn1086_sensor_exit(void)
{
	i2c_del_driver(&epd_sn1086_driver);
}

subsys_initcall(epd_sn1086_sensor_init);

//fs_initcall_sync(epd_sn1086_sensor_init);
module_exit(epd_sn1086_sensor_exit);

MODULE_AUTHOR("Benjamin Herrenschmidt <benh@kernel.crashing.org>");
MODULE_DESCRIPTION("sn1086 sensor objects for PowerMacs thermal control");
MODULE_LICENSE("GPL");

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_lm_show(struct seq_file *s, void *v)
{
    u32 value=0;
    epd_sn1086_get(&value);
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
	proc_create("sn1086", 0, NULL, &proc_lm_fops);
	return 0;

}
late_initcall(lm_proc_init);
#endif /* CONFIG_PROC_FS */

