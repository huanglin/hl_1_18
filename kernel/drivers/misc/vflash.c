
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>




#if 1
#define DBG(x...)	printk(KERN_INFO  x)
#else
#define DBG(x...)
#endif

char GetSNSectorInfo(char * pbuf);

int print_buf(char *buf,int size)
{
	int i,j,mo=size%16,line = size/16;
	char *pbuf = buf;

	if(line>0)
	{
		for(i=0;i<line;i++)
		{
			for(j=0;j<16;j++)
			{
				printk("0x%02x ",*pbuf);
				pbuf++;
			}
			printk("\n");
		}
	}
	
	for(j=0;j<mo;j++)
	{
		printk("0x%02x ",*pbuf);
		pbuf++;
	}
	return 0;
}



#define GET_IDBDATA  0x01
#define IDBLOCK_SIZE  512

#define VERSION "0.1"
static int minor = MISC_DYNAMIC_MINOR;

int vflash_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    if(NULL == argp)
        return -EFAULT;
    DBG("%s ioctl:%d\n", __FUNCTION__, cmd);
        
    switch(cmd)
    {
        case GET_IDBDATA:
        {
            char data[IDBLOCK_SIZE];

            memset(data, 0, IDBLOCK_SIZE);
            GetSNSectorInfo(data);
            if(copy_to_user(argp, data, IDBLOCK_SIZE))
                return -EFAULT;
        }
        break;

	default:
		break;
	}
	return 0;
}


static int vflash_open(struct inode *inode, struct file *file)
{
    DBG("%s\n", __FUNCTION__);
	return 0;
}

static int vflash_release(struct inode *inode, struct file *file)
{
    DBG("%s\n", __FUNCTION__);
	return 0;
}


static const struct file_operations vflash_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= vflash_ioctl,
	.open		= vflash_open,
	.release	= vflash_release,
};



static struct miscdevice vflash_miscdev= {
	.name		= "vflash",
	.fops		= &vflash_fops,
};


static int vflash_init(void)
{
	vflash_miscdev.minor = minor;

	if (misc_register(&vflash_miscdev) < 0) {
		printk(KERN_ERR"Can't register misc device with minor %d", minor);
		return -EIO;
	}
	return 0;
}

static void vflash_exit(void)
{
	if (misc_deregister(&vflash_miscdev) < 0)
		printk(KERN_ERR"Can't unregister misc device with minor %d", minor);
}


module_init(vflash_init);
module_exit(vflash_exit);

module_param(minor, int, 0444);

MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");

