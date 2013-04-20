/****************************************************************************************
 * 
 *Copyright 	:ROCKCHIP  Inc
 *Author	: zhongyw 
 *Date		: 2010-07-08
 *
 ********************************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/arch/typedef.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <asm/uaccess.h>


#include "eink_s.h"
#include "spi_flash.h"

#define SPI_PAGE_SIZE   16    //一次写16B
#define FLASH_PAGE_SIZE  65536//65536  //spi flash 一页的大小 256B
#define FLASH_GLOBAl_SIZE 32*65536
#define FLASH_PAGE_EARSE 4096
#if 0
	#define sfprintk(msg...)	printk(msg);
#else
	#define sfprintk(msg...)
#endif

struct spi_flash *gflashdev = NULL;

struct class *my_class;
struct cdev *gp_cdev;

void Delay100cyc(u32 count)
{
    u16 i;

    while (count--)
        for (i = 0; i < 23; i++);
}


u32 SpiFlashWaitBusy(struct spi_flash *flashdev)
{
	u8 cmd[1];
	u8 status=0xff;
	u32 i;
	for (i=0; i<500000; i++)
	{
		Delay100cyc(100);
		cmd[0] = 0x05;
        spi_write_then_read(flashdev->dev, cmd, 1, &status, 1);
		if ((status & 0x01) == 0)		
		return 0;
	}
	return 1;
}


/*
--------------------------------------------------------------------------------
  Function name : SPIFlashReadID
  Author        : chenfen
  Description   : 
                  
  Input         : 
  Return        : 

  History:     <author>         <time>         <version>       
             chenfen        2009/1/10         Ver1.0
  desc:         ORG
--------------------------------------------------------------------------------
*/
u32 SPIFlashReadID(struct spi_flash *flashdev, u8 * pdata)
{
    u8 cmd[4]; 
    cmd[0] = READ_AD;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
     
    spi_write_then_read(flashdev->dev, cmd, 4, pdata, 2);
    return 0;
}

/*
--------------------------------------------------------------------------------
  Function name : SPIFlashRead
  Author        : chenfen
  Description   : 
                  
  Input         : 
  Return        : 

  History:     <author>         <time>         <version>       
             chenfen        2009/1/10         Ver1.0
  desc:         ORG
  int32 SPIMRead(void *pdata, SPI_DATA_WIDTH dataWidth, u32 length)
--------------------------------------------------------------------------------
*/
u32 SPIFlashRead(void *flashdev, u32 addr, u8 *pData, u32 len) 
{    
    u8 cmd[4];
    u32 ReadLen;
    u32 ret = 0;
    u32 data = (u32)pData;
   
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
 
    while (len > 0)
    {
        ReadLen = (len > SPI_PAGE_SIZE)? SPI_PAGE_SIZE : len;     
        
        cmd[0] = READ_DATA;
        cmd[1] = addr>>16 & 0xff;
        cmd[2] = addr>>8 & 0xff;
        cmd[3] = addr & 0xff;
        ret = spi_write_then_read(((struct spi_flash*)flashdev)->dev, cmd, 4, (u8*)data, ReadLen);
        if( ret )
        {
            printk("%s [%d] spi_write_then_read err\n",__FUNCTION__, __LINE__);
            return 1;
        }
        data += ReadLen;
        len -= ReadLen;       
        addr += ReadLen;
    }

    return 0;
}

/*
--------------------------------------------------------------------------------
  Function name : SPIFlashWrite
  Author        : chenfen
  Description   : 
                  
  Input         : 
  Return        : 

  History:     <author>         <time>         <version>       
             chenfen        2009/1/10         Ver1.0
  desc:         ORG
   SPIMWrite(void *pdata, SPI_DATA_WIDTH dataWidth, u32 length)
--------------------------------------------------------------------------------
*/
u32 SPIFlashWrite(void *flashdev, u32 addr, u8 *pData, u32 len) 
{   
    u8 data[20];
	u32 writeLen; 
	u32 ret=0;
    
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
   
	while (len > 0)      
	{      
        writeLen = SPI_PAGE_SIZE - (addr % SPI_PAGE_SIZE);	
        writeLen = (len > writeLen)? writeLen : len;
	    data[0] = WRITE_ENABLE;    //write enable	    
       
        ret = spi_write_then_read(((struct spi_flash*)flashdev)->dev, (u8*)data, 1, NULL, 0);

        if (0!=SpiFlashWaitBusy(flashdev))
        {
           printk("%s [%d] SpiFlashWaitBusy err\n",__FUNCTION__, __LINE__);
           ret=1;
        }
        
	    data[0] = BYTE_WRITE;    //byte program
	    data[1] = addr>>16 & 0xff;
	    data[2] = addr>>8 & 0xff;
	    data[3] = addr & 0xff;

        memcpy(&data[4], pData, writeLen);
        
        ret = spi_write_then_read(((struct spi_flash*)flashdev)->dev, (u8*)data, writeLen+4, NULL, 0 );      
	    if(ret)
        {
            printk("%s [%d] spi_write_then_read err\n",__FUNCTION__, __LINE__);
            return 1;
        }
	    pData = (u8*)((u32)pData + writeLen);
	    addr = addr+writeLen;      
	    len -= writeLen;

        Delay100cyc(30);  //大于100ns  4.333us*30=130us
        if (0!=SpiFlashWaitBusy(flashdev))
        {
           printk("%s [%d] SpiFlashWaitBusy err\n",__FUNCTION__, __LINE__);
           ret=1;
        }
	}

	data[0] = WRITE_DISABLE;    //write disable
	spi_write_then_read(((struct spi_flash*)flashdev)->dev, data, 1, NULL, 0);
	return 0;
}

u32 Sector_Erase(void *flashdev, u32 addr) 
{    
    u8 data[4];

    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__); 
    
    data[0] = 0x06;    //write enable     
    spi_write_then_read(((struct spi_flash*)flashdev)->dev, data, 1, NULL, 0);
    
    data[0] = 0xd8;   // 块擦除
    data[1] = addr>>16 & 0xff;
    data[2] = addr>>8 & 0xff;
    data[3] = addr & 0xff;
    
    spi_write_then_read(((struct spi_flash*)flashdev)->dev, data, 4, NULL, 0 );
    
    if (0!=SpiFlashWaitBusy(flashdev))
    {
       printk("%s [%d] SpiFlashWaitBusy err\n",__FUNCTION__, __LINE__);
       return 1;
    }

    data[0] = 0x04;    //write disable
	spi_write_then_read(((struct spi_flash*)flashdev)->dev, data, 1, NULL, 0);

    return 0;

}

u32 Chiperase(struct spi_flash *flashdev)
{
	u8 cmd[4]; 
	u32 ret=0;
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
	cmd[0] = 0x06;    //write enable		
	spi_write_then_read(flashdev->dev, cmd, 1, NULL, 0);
    cmd[0] = 0xc7;
    spi_write_then_read(flashdev->dev, cmd, 1, NULL, 0);
    
	if (0!=SpiFlashWaitBusy(flashdev))
	{
       printk("%s [%d] SpiFlashWaitBusy err\n",__FUNCTION__, __LINE__);
	   ret=1;
	}

	cmd[0] = 0x04;    //write disable
	spi_write_then_read(flashdev->dev, cmd, 1, NULL, 0);
    return ret;
}
int spi_write_data(void *flashdev, int addr,char *buf,int len)
{
	char *pagedata=NULL;
	int earse_count=0;
   	u32 pageaddr=0;
	u32 pagelen=0;
	int temp_addr;
	int ret;
	pagedata = (char*)kmalloc(FLASH_PAGE_SIZE, GFP_KERNEL);
	    if(pagedata<0)
	    {
	        printk("spi_flash_write kmalloc failed 1 \n");
	        return -1;
	    } 
	while(len>=FLASH_PAGE_SIZE){  	
		pagelen = addr % FLASH_PAGE_SIZE;
		if(pagelen)
			pageaddr=addr - pagelen;
		else
			pageaddr=addr;
		temp_addr=pageaddr;
		ret = SPIFlashRead(gflashdev, pageaddr, pagedata, FLASH_PAGE_SIZE);
	        if(ret != 0)
	        {
	           printk("%s SPIFlashRead err\n",__FUNCTION__);  
		    return -1;
	    	}
		for(earse_count=0;earse_count<FLASH_PAGE_SIZE/FLASH_PAGE_EARSE;earse_count++){
			Sector_Erase(gflashdev, pageaddr);  
			pageaddr=pageaddr+FLASH_PAGE_EARSE;
		}    
	        SPIFlashWrite(gflashdev, addr, (u8*)buf, FLASH_PAGE_SIZE); 
		 if(pagelen)	
		 	SPIFlashWrite(gflashdev, temp_addr, pagedata, pagelen); 
		 buf=buf+FLASH_PAGE_SIZE;
	        len -= FLASH_PAGE_SIZE;
	        addr = temp_addr+FLASH_PAGE_SIZE;        
     	}	
	if(len){ 	
		pagelen = addr % FLASH_PAGE_SIZE;
		if(pagelen)
			pageaddr=addr -pagelen;
		else
			pageaddr=addr;
		temp_addr=pageaddr;
		ret = SPIFlashRead(gflashdev, pageaddr, pagedata, FLASH_PAGE_SIZE);
	        if(ret != 0)
	        {
	           printk("%s SPIFlashRead err\n",__FUNCTION__);    
		    return -1;
	    	}
		for(earse_count=0;earse_count<FLASH_PAGE_SIZE/FLASH_PAGE_EARSE;earse_count++){
			Sector_Erase(gflashdev, pageaddr);  
			pageaddr=pageaddr+FLASH_PAGE_EARSE;
		} 
		 SPIFlashWrite(gflashdev, addr, (u8*)buf, len); 
		 if(pagelen)	
		 	SPIFlashWrite(gflashdev, temp_addr, pagedata, pagelen); 
		 SPIFlashWrite(gflashdev, addr+len, pagedata+pagelen+len, FLASH_PAGE_SIZE-pagelen-len); 
		
	}
	  kfree(pagedata);
	  return 0;
}
/*
--------------------------------------------------------------------------------
  Function name : SPIFlashInit
  Author        : chenfen
  Description   : 
                  
  Input         : 
  Return        : 

  History:     <author>         <time>         <version>       
             chenfen        2009/1/10         Ver1.0
  desc:         ORG
--------------------------------------------------------------------------------
*/


static struct spi_flash_ops  gspi_flash_ops = {
	.Write = SPIFlashWrite,
	.Read = SPIFlashRead,
	.Buffer_write = spi_write_data,
	.owner = THIS_MODULE
};

static int spi_flash_open(struct inode *inode, struct file *file)
{
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
    file->f_pos = 0;
    return 0;
}
static ssize_t spi_flash_write(struct file *file, const char __user *data,
			      size_t len, loff_t * ppos)
{
    int ret =0;
    char *write_data=NULL;
    u32 addr = 0;

	if(ppos)
	addr = *ppos;   
	write_data = (char*)kmalloc(len, GFP_KERNEL);
	
	if(copy_from_user(write_data, data, len))
	{
		kfree(write_data);
		return -EFAULT;
	}
	spi_write_data(gflashdev,addr,(char*)write_data,len);
		
	kfree(write_data);
	return len;

}
static ssize_t spi_flash_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
    u32 addr = 0;
    char* kbuf = NULL;

    //if(ppos)addr = *ppos;   
    sfprintk(">>>>>> %s : %s, %x\n", __FILE__, __FUNCTION__, addr);

    kbuf = (char*)kmalloc(count, GFP_KERNEL);
    if(SPIFlashRead(gflashdev, addr ,kbuf , count))
    {
        kfree(kbuf);
        return -1;
    }
    else
    {
        if(copy_to_user(buf, (char *)kbuf, count))
        {
            kfree(kbuf);
            return -EFAULT;
        }
        return count;
    }
    return 0;
}


/* seek文件定位函数 */
static loff_t spi_flash_seek(struct file *filp, loff_t offset, int orig)
{
  loff_t ret = 0;
  switch (orig)
  {
    case 0:   /*相对文件开始位置偏移*/
      if (offset < 0)
      {
        ret =  - EINVAL;
        break;
      }
      if ((unsigned int)offset > FLASH_GLOBAl_SIZE)
      {
        ret =  - EINVAL;
        break;
      }
      filp->f_pos = (unsigned int)offset;
      ret = filp->f_pos;
      break;
    case 1:   /*相对文件当前位置偏移*/
      if ((filp->f_pos + offset) > FLASH_GLOBAl_SIZE)
      {
        ret =  - EINVAL;
        break;
      }
      if ((filp->f_pos + offset) < 0)
      {
        ret =  - EINVAL;
        break;
      }
      filp->f_pos += offset;
      ret = filp->f_pos;
      break;
    default:
      ret =  - EINVAL;
      break;
  }
  return ret;
}



static struct file_operations spi_flash_fops = {
    .owner	= THIS_MODULE,
    .open   = spi_flash_open,
    .write  = spi_flash_write,
    .read   = spi_flash_read,
    .llseek = spi_flash_seek
};

//static int  __devinit spi_flash_probe(struct spi_device *spi)
static int  __devinit spi_flash_probe(struct spi_device *spi)
{        
    int16 flash_major = 0;
    dev_t devno; 
    int err =0;
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
    gflashdev = kmalloc(sizeof(struct spi_flash), GFP_KERNEL);
    if(gflashdev==NULL)
    {
        return 1;
    }
    
    gflashdev->ops = &gspi_flash_ops;
    gflashdev->dev = spi;
    epd_register_flash(gflashdev);
  //  register_chrdev(0, "spi_flash", &spi_flash_fops);
   /* create your own class under /sysfs */
     devno = MKDEV(flash_major, 0);
     if(flash_major)
     {
         register_chrdev_region(devno, 1, "spi_flash");
     }
     else
     {
         alloc_chrdev_region(&devno,0, 1, "spi_flash");
         flash_major = MAJOR(devno);
     }
     
     gp_cdev = NULL;
     gp_cdev = kmalloc(sizeof(struct cdev), GFP_KERNEL);
     if(gp_cdev == NULL)
     {
         printk("Error adding SPI FLASH\n");
         return 1;
     }
     cdev_init(gp_cdev, &spi_flash_fops);
     gp_cdev->owner = THIS_MODULE;
     gp_cdev->ops = &spi_flash_fops;
     err = cdev_add(gp_cdev, devno, 1);
     if(err)
        printk(KERN_NOTICE "Error adding SPI FLASH\n");
     
     my_class = class_create(THIS_MODULE, "spi_flash");
     if(IS_ERR(my_class)) 
     {
          printk("Err: failed in creating spi flash class.\n");
          return -1; 
     } 
     device_create(my_class, NULL, devno, "spi_flash"); 
     
	return 0;
}

static  int __devexit spi_flash_remove(struct spi_device *pdev)
{  
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
    if(gflashdev)
    {
        kfree(gflashdev);
        gflashdev = NULL;
    }
    
    //unregister_chrdev(0, "spi_flash");
    if(gp_cdev)
    {
        cdev_del(gp_cdev);
        kfree(gp_cdev);
    }
    if(my_class)
    {
        device_destroy(my_class, 0);         //delete device node under /dev
        class_destroy(my_class);             //delete class created by us
    }
	return 0;
}

#ifdef CONFIG_PM
int spi_flash_suspend(struct spi_device *spi, pm_message_t state)
{
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
     
    return 0;
}

int spi_flash_resume(struct spi_device *spi)
{
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
  
    return 0;
}
#endif 



static struct spi_driver spi_flash_driver = 
{
	.driver = {
		.name = "epd_spi_flash",
		.bus	  = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = spi_flash_probe,
	.remove = __devexit_p(spi_flash_remove),
#ifdef CONFIG_PM
	.suspend = spi_flash_suspend,
	.resume = spi_flash_resume
#endif
};

static int __init spi_flash_init(void)
{
	int ret = spi_register_driver(&spi_flash_driver);
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
    // ret = driver_create_file(&spi_flash_driver.driver, &driver_attr_touchcheck);
	
	return ret;
}

static void __exit spi_flash_exit(void)
{
    sfprintk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
    
  //  driver_remove_file(&spi_flash_driver.driver, &driver_attr_touchcheck);
   
	return spi_unregister_driver(&spi_flash_driver);
}

//device_initcall_sync(spi_flash_init);

//fs_initcall_sync(spi_flash_init);
subsys_initcall(spi_flash_init);
module_exit(spi_flash_exit);
MODULE_AUTHOR("zyw,zyw@rockchip.com");
MODULE_DESCRIPTION("rockchip rk28chip extern spi flash");
MODULE_LICENSE("GPL");

