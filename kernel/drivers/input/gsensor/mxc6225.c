/*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <asm/arch/gpio.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include "mxc6225.h"

#if 1
#define printk_d(fmt, args...)  printk(KERN_INFO "[gsensor] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define printk_d(fmt, args...)
#endif

#define printk_e(fmt, args...)  printk(KERN_ERR  "[gsensor] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#define printk_i(fmt, args...)  printk(KERN_INFO "[gsensor] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)


#define MXC6225_GPIO_INT     GSENSOR_INT_IOPIN
static int mxc6225_probe(struct i2c_adapter *adapter, int addr, int kind);


/* Addresses to scan -- protected by sense_data_mutex */
static char sense_data[RBUFF_SIZE + 1];
static struct i2c_client *this_client;
static unsigned short normal_i2c[] = {MXC6225_IIC_ADDR >> 1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data_mxc6225 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static atomic_t data_ready;
static android_early_suspend_t mxc6225_early_suspend;
static int revision = -1;

/* AKM HW info */
static ssize_t gsensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "AK8976A_%#x\n", revision);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, gsensor_vendor_show, NULL);

static struct kobject *android_gsensor_kobj;

static int gsensor_sysfs_init(void)
{
	int ret ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL) {
		printk_e("subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk_e("sysfs_create_group failed\n");
		goto err4;
	}

	return 0 ;
err4:
	kobject_del(android_gsensor_kobj);
err:
	return ret ;
}


static int mxc6225_rx_data(struct i2c_client *client, char *rxData, int length)
{
	char reg2[2] = {rxData[0], 0};

	if (i2c_master_send(this_client, reg2, 2) != 2)	{
		printk_e("Error rxData[0]=0x%x \n", rxData[0]);
		return -EIO;
	}


	if (i2c_master_recv(this_client, rxData, length) != length) {
		printk_e("Error rxData[0]=0x%x \n", rxData[0]);
		return -EIO;
	}

	return 0;
}


static int mxc6225_tx_data(struct i2c_client *client, char *txData, int length)
{
	if (i2c_master_send(this_client, txData, length) != length)	{
		printk_e("Error txData[0]=0x%x \n", txData[0]);
		return -EIO;
	}

	return 0;
}



static int mxc6225_start_dev(struct i2c_client *client)
{
	char buffer[MXC6225_REG_LEN];
	int  ret = 0;

	buffer[0] = MXC6225_REG_DETECTION;
	buffer[1] = 0x2a;
	ret = mxc6225_tx_data(client, buffer, 2);
	
   GPIOClrearInmarkIntr(MXC6225_GPIO_INT);

#ifdef MX6225_DATA_FILTER
	mx6225_adjust.count = 0;
#endif

	return ret;
}


static int mxc6225_start(struct i2c_client *client)
{ 
    struct mxc6225_data *mxc6225 = (struct mxc6225_data *)i2c_get_clientdata(client);
    
    if (mxc6225->status == MXC6225_OPEN) {
        return 0;      
    }
   
	mxc6225->status = MXC6225_OPEN;

    return mxc6225_start_dev(client);
}


static int mxc6225_close_dev(struct i2c_client *client)
{    	
	char buffer[2];

    GPIOInmarkIntr(MXC6225_GPIO_INT);

	buffer[0] = MXC6225_REG_DETECTION;
	buffer[1] = 0xaa;
	
	return mxc6225_tx_data(client, buffer, 2);
}


static int mxc6225_close(struct i2c_client *client)
{
	struct mxc6225_data *mxc6225 = (struct mxc6225_data *)i2c_get_clientdata(client);
	mxc6225->status = MXC6225_CLOSE;

    return mxc6225_close_dev(client);
}


static int mxc6225_reset(struct i2c_client *client)
{
	int ret = 0;

	ret = mxc6225_close_dev(client);
	ret = mxc6225_start_dev(client);
    
	return ret ;
}


static inline int mxc6225_convert_to_int(char value)
{
    int result;
	
	if (value >128) 
		result = 191 - value;
	else
		result =value;  
	
	return result;
}


static void mxc6225_report_value(struct i2c_client *client, struct mxc6225_axis *axis)
{
    struct mxc6225_data *mxc6225 = i2c_get_clientdata(client);

#ifdef MX6225_DATA_FILTER
	int i = 0, axis_x=0, axis_y=0, axis_z=0;
#endif

	axis->x=axis->x*MXC6225_GRAVITY_STEP;
	axis->y=axis->y*MXC6225_GRAVITY_STEP;

#ifdef MX6225_DATA_FILTER
	for(i = 0; i < MX6225_TEMP_SIZE; i++) {
		if((MX6225_TEMP_SIZE-1) == i) {
			mx6225_adjust.axis_x[i] = axis->x;
			mx6225_adjust.axis_y[i] = axis->y;
			//mx6225_adjust.axis_z[i] = axis->z;
		}else{
			mx6225_adjust.axis_x[i] = mx6225_adjust.axis_x[i+1];
			mx6225_adjust.axis_y[i] = mx6225_adjust.axis_y[i+1];
			//mx6225_adjust.axis_z[i] = mx6225_adjust.axis_z[i+1];
		}
	}
	for(i = 0; i < MX6225_TEMP_SIZE; i++){
		axis_x += mx6225_adjust.axis_x[i];
		axis_y += mx6225_adjust.axis_y[i];
		//axis_z += mx6225_adjust.axis_z[i];
	}

	if(++mx6225_adjust.count > MX6225_TEMP_SIZE){
		axis->x = axis_x/MX6225_TEMP_SIZE;
		axis->y = axis_y/MX6225_TEMP_SIZE;
		//axis->z = axis_z/MX6225_TEMP_SIZE;
	}
#endif

    input_report_abs(mxc6225->input_dev, ABS_X, axis->x);
    input_report_abs(mxc6225->input_dev, ABS_Y, axis->y);
    input_sync(mxc6225->input_dev);

    printk_d("Gsensor x==%d  y==%d \n",axis->x,axis->y);
}


static int mxc6225_get_data(struct i2c_client *client)
{
	char buffer[3];
	int ret;
	struct mxc6225_axis axis;

	memset(buffer, 0, 3);
	buffer[0] = MXC6225_REG_X_OUT;
	ret = mxc6225_rx_data(client, &buffer[0], 3);
	if (ret < 0)
		return ret;
	
	axis.y =-mxc6225_convert_to_int(buffer[MXC6225_REG_X_OUT]);
	axis.x =-mxc6225_convert_to_int(buffer[MXC6225_REG_Y_OUT]);
	
	if (axis.x>0) 
		axis.x = 64-axis.x;
	
	if (axis.y<0)  
		axis.y = -64-axis.y;

	mxc6225_report_value(client, &axis);

	return 0;
}

/*
static void mxc6225_test()
{
	char buffer[4];
	int ret;

	mxc6225_start_dev(this_client);
	mdelay(1000);

	while(1)
	{
		memset(buffer, 0, 3);
		buffer[0] = MXC6225_REG_X_OUT;
		ret = mxc6225_rx_data(this_client, &buffer[0], 3);
	
		printk_d("addr=0x%x, ret=%d, buffer[0]=0x%x buffer[1]=0x%x buffer[2]=0x%x\n", this_client->addr, ret, buffer[0], buffer[1], buffer[2]);
		mdelay(1000);
	}
}
*/

static int mxc6225_trans_buff(char *rbuf, int size)
{
	wait_event_interruptible(data_ready_wq, atomic_read(&data_ready));

	atomic_set(&data_ready, 0);
	memcpy(rbuf, &sense_data[0], size);

	return 0;
}


static int mxc6225_open(struct inode *inode, struct file *file)
{
	printk_i("\n");

	return 0;
}

static int mxc6225_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int mxc6225_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	char msg[RBUFF_SIZE + 1];
	int  ret = -1;
	char rate;

	printk_i("cmd: %d\n", cmd);

	switch (cmd) {
	case ECS_IOCTL_APP_SET_RATE:
		if (copy_from_user(&rate, argp, sizeof(rate)))
			return -EFAULT;
		break;

	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_START:
		ret = mxc6225_start(this_client);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_CLOSE:
		ret = mxc6225_close(this_client);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_APP_SET_RATE:
		ret = mxc6225_reset(this_client);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GETDATA:
		ret = mxc6225_trans_buff(msg, RBUFF_SIZE);
		if (ret < 0)
			return ret;
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_GETDATA:
		if (copy_to_user(argp, &msg, sizeof(msg)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}


static void mxc6225_work_func(struct work_struct *work)
{
	if (mxc6225_get_data(this_client) < 0) 
		printk_e( "MXC6225 mma_work_func: Get data failed\n");
		
    GPIOClrearInmarkIntr(MXC6225_GPIO_INT);
}


static void  mxc6225_delaywork_func(struct work_struct  *work)
{
	printk_d("Enter\n");

#if 1  //test
	static int count = 0 ;
	if (count == 0)
	{
		mxc6225_start(this_client);
		count = 1;
		return;
	}
#endif

	if (mxc6225_get_data(this_client) < 0) {
		printk_e("Get data failed\n");
	}
	

	GPIOClrearInmarkIntr(MXC6225_GPIO_INT);
}


static irqreturn_t mxc6225_interrupt(int irq, void *dev_id)
{
	struct mxc6225_data *data = dev_id;

	printk_d("Enter\n");

	GPIOInmarkIntr(MXC6225_GPIO_INT);

	schedule_delayed_work(&data->delaywork,msecs_to_jiffies(30));
	
	return IRQ_HANDLED;
}


static struct file_operations mxc6225_fops = {
	.owner	 = THIS_MODULE,
	.open	 = mxc6225_open,
	.release = mxc6225_release,
	.ioctl	 = mxc6225_ioctl,
};


static struct miscdevice mxc6225_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mxc6225_daemon",
	.fops = &mxc6225_fops,
};


static int mxc6225_remove(struct i2c_client *client)
{
	struct mxc6225_data *mma = i2c_get_clientdata(client);

#ifdef CONFIG_ANDROID_POWER
    android_unregister_early_suspend(&mxc6225_early_suspend);
#endif        
	free_gpio_irq(MXC6225_GPIO_INT);
	input_unregister_device(mma->input_dev);
	i2c_detach_client(client);
	kfree(mma);

	return 0;
}


static int mxc6225_detach_client(struct i2c_client *client)
{
	struct gsensor_device *gs = i2c_get_clientdata(client);

	mxc6225_remove(this_client);

	return i2c_detach_client(client);
}


static int mxc6225_attach_adapter(struct i2c_adapter *adap)
{
	printk_d("Enter\n");
	return i2c_probe(adap, &addr_data_mxc6225, mxc6225_probe);
}


#ifdef CONFIG_ANDROID_POWER
static int mxc6225_suspend(android_early_suspend_t *h)
{
	printk_i("enter\n");
	return mxc6225_close_dev(this_client);
}

static int mxc6225_resume(android_early_suspend_t *h)
{
    struct mxc6225_data *mma = (struct mxc6225_data *)i2c_get_clientdata(this_client);
	
	printk("enter!\n");
	
	return mxc6225_start_dev(this_client);
}


#endif

static struct i2c_driver mxc6225_driver = {
	.driver = {
		.name = "gs_mma7660",
	    },
	.id 	= MXC6225_IIC_ADDR,
	.attach_adapter = &mxc6225_attach_adapter,
	.detach_client  = &mxc6225_detach_client,
	.suspend = mxc6225_suspend,
	.resume = mxc6225_resume,
};


static int mxc6225_init_client(struct i2c_client *client)
{
	struct mxc6225_data *data;
	int ret;

	data = i2c_get_clientdata(client);
#if 0
	GPIOPullUpDown(MXC6225_GPIO_INT,GPIOPullUp);
	
	ret = request_gpio_irq(MXC6225_GPIO_INT, mxc6225_interrupt, GPIOEdgelRising, data);		
	if (ret < 0) {
		printk_e( "request irq failed\n");
        return ret;
	}
#endif
	init_waitqueue_head(&data_ready_wq);

#if 0  
        mma7660_start(MMA7660_RATE_16);
#endif    

	return 0;
}


static int mxc6225_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct mxc6225_data *mma;
	struct i2c_client *client = NULL;
	int err;

	printk_d("Enter\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client) {
		err = -ENODEV;
		goto exit_alloc_client_failed;
	}

	strlcpy(client->name, "gs_mma7660", I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &mxc6225_driver;
	client->addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 200;	
	err = i2c_attach_client(client);
	if (err < 0) {
		err = -ENODEV;
		goto exit_i2c_attach_client_failed;
	}

	mma = kzalloc(sizeof(struct mxc6225_data), GFP_KERNEL);
	if (!mma) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	INIT_WORK(&mma->work, mxc6225_work_func);
	INIT_DELAYED_WORK(&mma->delaywork, mxc6225_delaywork_func);
	
	i2c_set_clientdata(client, mma);

	this_client = client;

	err = mxc6225_init_client(client);
	if (err < 0) {
		printk_e("mxc6225_init_client failed\n");
		goto exit_request_gpio_irq_failed;
	}
		
	mma->input_dev = input_allocate_device();
	if (!mma->input_dev) {
		err = -ENOMEM;
		printk_e("Failed to allocate input device\n");
		goto exit_input_allocate_device_failed;
	}

	set_bit(EV_ABS, mma->input_dev->evbit);

	/* x-axis acceleration */
	input_set_abs_params(mma->input_dev, ABS_X, -1500, 1500, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(mma->input_dev, ABS_Y, -1500, 1500, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(mma->input_dev, ABS_Z, -1500, 1500, 0, 0);


	mma->input_dev->name = "compass";

	err = input_register_device(mma->input_dev);
	if (err < 0) {
		printk_e("Unable to register input device: %s\n", mma->input_dev->name);
		goto exit_input_register_device_failed;
	}

	err = misc_register(&mxc6225_device);
	if (err < 0) {
		printk_e("mmad_device register failed\n");
		goto exit_misc_device_register_mxc6225_device_failed;
	}

	err = gsensor_sysfs_init();
	if (err < 0) {
		printk_e("gsensor sysfs init failed\n");
		goto exit_gsensor_sysfs_init_failed;
	}
	
#if (0)//#ifdef CONFIG_ANDROID_POWER
    mxc6225_early_suspend.suspend = mxc6225_suspend;
    mxc6225_early_suspend.resume = mxc6225_resume;
    mxc6225_early_suspend.level = 0x2;
    android_register_early_suspend(&mxc6225_early_suspend);
#endif

//	mxc6225_test();
//	schedule_delayed_work(&mma->delaywork,msecs_to_jiffies(30));

	return 0;

exit_gsensor_sysfs_init_failed:
  __misc_deregister(&mxc6225_device, 1);
exit_misc_device_register_mxc6225_device_failed:
   input_unregister_device(mma->input_dev);
exit_input_register_device_failed:
	input_free_device(mma->input_dev);
exit_input_allocate_device_failed:
    free_gpio_irq(MXC6225_GPIO_INT);
exit_request_gpio_irq_failed:
	kfree(mma);	
exit_alloc_data_failed:
   i2c_detach_client(client);
exit_i2c_attach_client_failed:
	kfree(client);
exit_alloc_client_failed:
exit_check_functionality_failed:
	return err;
}


static int __init mxc6225_i2c_init(void)
{
	printk_i(KERN_INFO "MXC6225 driver: init\n");
	return i2c_add_driver(&mxc6225_driver);
}

static void __exit mxc6225_i2c_exit(void)
{
	i2c_del_driver(&mxc6225_driver);
}

module_init(mxc6225_i2c_init);
module_exit(mxc6225_i2c_exit);



