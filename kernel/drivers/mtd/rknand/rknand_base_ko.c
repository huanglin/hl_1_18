/*
 *  linux/drivers/mtd/rknand/rknand_base.c
 *
 *  Copyright (C) 2005-2009 Fuzhou Rockchip Electronics
 *  ZYF <zyf@rock-chips.com>
 *
 *   
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/reboot.h>
#include <asm/io.h>
#include <asm/mach/flash.h>
//#include "api_flash.h"
#include "rknand_base.h"


#define DRIVER_NAME	"rk28xxnand"
const char rknand_base_version[] = "rknand_base.c version: 4.30 20111009";
#define NAND_DEBUG_LEVEL0 0
#define NAND_DEBUG_LEVEL1 1
#define NAND_DEBUG_LEVEL2 2
#define NAND_DEBUG_LEVEL3 3

int g_num_partitions = 0;
unsigned long SysImageWriteEndAdd = 0;
struct mtd_info		rknand_mtd;  
struct mtd_partition *rknand_parts;
struct rknand_info * gpNandInfo;

#ifdef CONFIG_MTD_NAND_RK29XX_DEBUG
static int s_debug = CONFIG_MTD_NAND_RK29XX_DEBUG_VERBOSE;
#undef NAND_DEBUG
#define NAND_DEBUG(n, format, arg...) \
	if (n <= s_debug) {	 \
		printk(format,##arg); \
	}
#else
#undef NAND_DEBUG
#define NAND_DEBUG(n, arg...)
static const int s_debug = 0;
#endif

#include <linux/proc_fs.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26))
#define NANDPROC_ROOT  (&proc_root)
#else
#define NANDPROC_ROOT  NULL
#endif

#define     DATA_LEN            (1024*8*2/4)              //数据块单位word
#define     SPARE_LEN           (32*8*2/4)               //校验数据长度
#define     PAGE_LEN            (DATA_LEN+SPARE_LEN)    //每个数据单位的长度
#define     MAX_BUFFER_SIZE     (long)(2048 * 4) //sector
long grknand_buf[MAX_BUFFER_SIZE * 512/4] __attribute__((aligned(4096)));
long grknand_dma_buf[PAGE_LEN*4*5] __attribute__((aligned(4096)));

static struct proc_dir_entry *my_proc_entry;
extern int rkNand_proc_ftlread(char *page);
extern int rkNand_proc_bufread(char *page);
static int rkNand_proc_read(char *page,
			   char **start,
			   off_t offset, int count, int *eof, void *data)
{
	char *buf = page;
	int step = offset;
	*(int *)start = 1;
	if(step == 0)
	{
        buf += sprintf(buf, "%s\n", rknand_base_version);
        if(gpNandInfo->proc_ftlread)
            buf += gpNandInfo->proc_ftlread(buf);
        if(gpNandInfo->proc_bufread)
            buf += gpNandInfo->proc_bufread(buf);
    }
	return buf - page < count ? buf - page : count;
}

static void rk28nand_create_procfs(void)
{
    /* Install the proc_fs entry */
    my_proc_entry = create_proc_entry("rk28xxnand",
                           S_IRUGO | S_IFREG,
                           NANDPROC_ROOT);

    if (my_proc_entry) {
        my_proc_entry->write_proc = NULL;
        my_proc_entry->read_proc = rkNand_proc_read;
        my_proc_entry->data = NULL;
    } 
}

void printk_write_log(long lba,int len, const u_char *pbuf)
{
    char debug_buf[100];
    int i;
    for(i=0;i<len;i++)
    {
        sprintf(debug_buf,"%lx :",lba+i);
        print_hex_dump(KERN_WARNING, debug_buf, DUMP_PREFIX_NONE, 16,4, &pbuf[512*i], 8, 0);
    }
}

static int rk28xxnand_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	int ret = 0;
	int sector = len;
	int LBA = (int)(from);
	//if(rknand_debug)
    //   printk("rk28xxnand_read: from=%x,sector=%x,\n",(int)LBA,sector);
    if(sector && gpNandInfo->ftl_read)
    {
		ret = gpNandInfo->ftl_read(LBA, sector, buf);
    }
	*retlen = len;
	return 0;//ret;
}

static int rk28xxnand_write(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, const u_char *buf)
{
	int ret = 0;
	int sector = len;
	int LBA = (int)(from);
	//printk("*");
	//if(rknand_debug)
    //    printk(KERN_NOTICE "write: from=%lx,sector=%x\n",(int)LBA,sector);
    //printk_write_log(LBA,sector,buf);
	if(sector && gpNandInfo->ftl_write)// cmy
	{
		if(LBA < SysImageWriteEndAdd)//0x4E000)
		{
			NAND_DEBUG(NAND_DEBUG_LEVEL0,">>> FtlWriteImage: LBA=0x%08X  sector=%d\n",LBA, sector);
            ret = gpNandInfo->ftl_write(LBA, sector, (void *)buf,1);
        }
		else
        {
            ret = gpNandInfo->ftl_write(LBA, sector, (void *)buf,0);
        }
	}
	*retlen = len;
	return 0;
}

static int rk28xxnand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int ret = 0;
    if (instr->callback)
		instr->callback(instr);
	return ret;
}

static void rk28xxnand_sync(struct mtd_info *mtd)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_sync: \n");
    if(gpNandInfo->ftl_sync)
        gpNandInfo->ftl_sync();
}

extern void FtlWriteCacheEn(int);
static int rk28xxnand_panic_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	int sector = len;
	int LBA = (int)(to);

	if (sector && gpNandInfo->ftl_write_panic) {
	    if(gpNandInfo->ftl_cache_en)
		    gpNandInfo->ftl_cache_en(0);
		gpNandInfo->ftl_write_panic(LBA, sector, (void *)buf);
	    if(gpNandInfo->ftl_cache_en)
		    gpNandInfo->ftl_cache_en(1);
	}
	*retlen = len;
	return 0;
}


int GetIdBlockSysData(char * buf, int Sector)
{
    if(gpNandInfo->GetIdBlockSysData)
	   return( gpNandInfo->GetIdBlockSysData( buf,  Sector));
    return 0;
}

char GetSNSectorInfo(char * pbuf)
{
    if(gpNandInfo->GetSNSectorInfo)
	   return( gpNandInfo->GetSNSectorInfo( pbuf));
    return 0;
}

char GetChipSectorInfo(char * pbuf)
{
    if(gpNandInfo->GetChipSectorInfo)
	   return( gpNandInfo->GetChipSectorInfo( pbuf));
    return 0;
}

int  GetParamterInfo(char * pbuf , int len)
{
    int ret = -1;
	int sector = (len)>>9;
	int LBA = 0;
	if(sector && gpNandInfo->ftl_read)
	{
		ret = gpNandInfo->ftl_read(LBA, sector, pbuf);
	}
	return ret?-1:(sector<<9);
}

int  GetflashDataByLba(int lba,char * pbuf , int len)
{
    int ret = -1;
	int sector = (len)>>9;
	int LBA = lba;
	if(sector && gpNandInfo->ftl_read)
	{
		ret = gpNandInfo->ftl_read(LBA, sector, pbuf);
	}
	return ret?-1:(sector<<9);
}


static int rk28xxnand_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	return 0;
}

static int rk28xxnand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	return 0;
}

static int rk28xxnand_init(struct rknand_info *nand_info)
{
	struct mtd_info	   *mtd = &rknand_mtd;
	struct rknand_chip *rknand = &nand_info->rknand;  

	rknand->state = FL_READY;
	rknand->rknand_schedule_enable = 1;
	rknand->pFlashCallBack = NULL;
	init_waitqueue_head(&rknand->wq);

    mtd->oobsize = 0;
    mtd->oobavail = 0;
    mtd->ecclayout = 0;
    mtd->erasesize = 32;
    mtd->writesize = 8;

	// Fill in remaining MTD driver data 
	mtd->type = MTD_NANDFLASH;
	mtd->flags = (MTD_WRITEABLE|MTD_NO_ERASE);//
	mtd->erase = rk28xxnand_erase;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = rk28xxnand_read;
	mtd->write = rk28xxnand_write;
	mtd->read_oob = NULL;
	mtd->write_oob = NULL;
	mtd->panic_write = rk28xxnand_panic_write;

	mtd->sync = rk28xxnand_sync;
	mtd->lock = NULL;
	mtd->unlock = NULL;
	mtd->suspend = NULL;
	mtd->resume = NULL;
	mtd->block_isbad = rk28xxnand_block_isbad;
	mtd->block_markbad = rk28xxnand_block_markbad;
	mtd->owner = THIS_MODULE;
    return 0;
}


/*
 * CMY: 增加了对命令行分区信息的支持
 *		若cmdline有提供分区信息，则使用cmdline的分区信息进行分区
 *		若cmdline没有提供分区信息，则使用默认的分区信息(rk28_partition_info)进行分区
 */

#ifdef CONFIG_MTD_CMDLINE_PARTS
const char *part_probes[] = { "cmdlinepart", NULL }; 
#endif 

static int rk29xxnand_add_partitions(struct rknand_info *nand_info)
{
#ifdef CONFIG_MTD_CMDLINE_PARTS
    int num_partitions = 0; 

	// 从命令行解析分区的信息
    num_partitions = parse_mtd_partitions(&(rknand_mtd), part_probes, &rknand_parts, 0); 
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"num_partitions = %d\n",num_partitions);
    if(num_partitions > 0) { 
    	int i;
		g_num_partitions = num_partitions;
		return add_mtd_partitions(&(rknand_mtd), rknand_parts, num_partitions);
    } 
#endif 
	return 0;
}

int add_rknand_device(struct rknand_info * prknand_Info)
{
    struct mtd_partition *parts;
    int i;
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"add_rknand_device: \n");
    
    rknand_mtd.size = (uint64_t)gpNandInfo->nandCapacity;
    
    rk29xxnand_add_partitions(prknand_Info);
 
    parts = rknand_parts;
    for(i=0;i<g_num_partitions;i++)
    {
        //printk(">>> part[%d]: name=%s offset=0x%012llx\n", i, parts[i].name, parts[i].offset);
        if(strcmp(parts[i].name,"backup") == 0)
        {
            SysImageWriteEndAdd = (unsigned long)(parts[i].offset + parts[i].size);//sector
            //printk(">>> SysImageWriteEndAdd=0x%lx\n", SysImageWriteEndAdd);
            break;
        }
    }

    gpNandInfo->SysImageWriteEndAdd = SysImageWriteEndAdd;
    return 0;
}

int get_rknand_device(struct rknand_info ** prknand_Info)
{
    *prknand_Info = gpNandInfo;
    return 0;    
}

EXPORT_SYMBOL(get_rknand_device);

static int rknand_probe(struct platform_device *pdev)
{
	struct rknand_info *nand_info;
	int err = 0;
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_probe: \n");
	gpNandInfo = kzalloc(sizeof(struct rknand_info), GFP_KERNEL);
	if (!gpNandInfo)
		return -ENOMEM;
    
    nand_info = gpNandInfo;

    memset(gpNandInfo,0,sizeof(struct rknand_info));

    gpNandInfo->bufSize = MAX_BUFFER_SIZE * 512;
    gpNandInfo->pbuf = (char *)grknand_buf;
    gpNandInfo->pdmaBuf = (char *)grknand_dma_buf;
    //printk(" gpNandInfo->pdmaBuf=0x%x\n",  gpNandInfo->pdmaBuf); 
#ifdef CONFIG_MTD_EMMC_CLK_POWER_SAVE
    gpNandInfo->emmc_clk_power_save_en = 1;
#endif

	rknand_mtd.name = pdev->dev.bus_id;//dev_name(&pdev->dev);
	rknand_mtd.priv = &nand_info->rknand;
	rknand_mtd.owner = THIS_MODULE;
    
	if(rk28xxnand_init(nand_info))
	{
		err = -ENXIO;
		goto  exit_free;
	}
	
	nand_info->add_rknand_device = add_rknand_device;
	nand_info->get_rknand_device = get_rknand_device;

	rk28nand_create_procfs();
	return 0;

exit_free:
	if(nand_info)
      	kfree(nand_info);

	return err;
}

static int rknand_suspend(struct platform_device *pdev, pm_message_t state)
{
    gpNandInfo->rknand.rknand_schedule_enable = 0;
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rknand_suspend: \n");
	return 0;
}

static int rknand_resume(struct platform_device *pdev)
{
    gpNandInfo->rknand.rknand_schedule_enable = 1;
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rknand_resume: \n");
	return 0;
}

void rknand_shutdown(struct platform_device *pdev)
{
    printk("rknand_shutdown...\n");
    gpNandInfo->rknand.rknand_schedule_enable = 0;
    if(gpNandInfo->rknand_buffer_shutdown)
        gpNandInfo->rknand_buffer_shutdown();    
}

static struct platform_driver rknand_driver = {
	.probe		= rknand_probe,
	.suspend	= rknand_suspend,
	.resume		= rknand_resume,
	.shutdown   = rknand_shutdown,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};


MODULE_ALIAS(DRIVER_NAME);

static int __init rknand_init(void)
{
	int ret;
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rknand_init: \n");
	ret = platform_driver_register(&rknand_driver);
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"platform_driver_register:ret = %x \n",ret);
	return ret;
}

static void __exit rknand_exit(void)
{
    platform_driver_unregister(&rknand_driver);
}

module_init(rknand_init);
module_exit(rknand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ZYF <zyf@rock-chips.com>");
MODULE_DESCRIPTION("rknand driver.");


