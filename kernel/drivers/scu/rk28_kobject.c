/*
 *  rkscu/rk_kobject.c
 *
 * (C) Copyright hsl 2009
 *	Released under GPL v2.
 *
 * 
 * log:
 *      for scu unit sysfs 
 *      20070716, REMOVE attr max_clk,CAN NOT BE change.
 *      20090716,USE init12m active for kernel , IO mem dump.
 *      20090717,use init24m actiev for kernel function test.
 *      
 */


#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/wait.h>

#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/spinlock_types.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/kallsyms.h>
#include <linux/rtc.h>
#include <asm/atomic.h>
#include <asm/signal.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>

#include <asm/arch/rk28_scu.h>

#define WHOWME "SCUK"
#define S_LEVEL  S_L_WARN
#include <asm/arch/rk28_debug.h>

#if SCU_KOJB_SUPPORT

static struct kset *scu_kset;
#define KTO_SCU_UINT( x )        container_of(x, struct rockchip_scu_unit, kobj)

extern int __rk28_scu_parse_cmd( char * cdb );
extern int __rockchip_clk_get_uint_clk_hw( struct rockchip_scu_unit *p  );

static char scu_init24m_cmd_write[128];
static ssize_t __scu_init24m_show_active(struct rockchip_scu_unit *p ,
        char *buf , size_t count)
{
        unsigned long flags;
        char    cmd[128];
        int      ret = -1;
        
        local_irq_save(flags);
        if( scu_init24m_cmd_write[0] ) {
                strcpy( cmd , scu_init24m_cmd_write );
        } else {
                cmd[0] = 0;
        }
        local_irq_restore(flags);

        if( cmd[0] ){
                ret = __rk28_scu_parse_cmd( cmd );
        }
        if( buf )
                ret = sprintf( buf ,"CMD:%s,ret=0x%x\n", scu_init24m_cmd_write  , ret );
        if( ret == 0 ) /* 0 meas try again!! */
                ret = 0XEEEE0000;
        return ret;
        
}

static ssize_t __scu_init24m_set_active(struct rockchip_scu_unit *p ,
        const char *buf ,size_t count)
{
        int len = strlen( buf );

        if( len < 3 )
                return -EIO;
        if( len > 127 )
                len = 127;
        while( buf[len-1] == '\n' || buf[len-1] == ' ' ) {
                len--;
        }
        memcpy( scu_init24m_cmd_write , buf , len );
        scu_init24m_cmd_write[len] = 0;
        len = __scu_init24m_show_active( p , NULL , count );
        return count;
}

#if 0
/*20100621 WQQ for test cpu change freq
*test case
*   1.change freq
*   2.record log
*/

static void record_changecpufreq_log(char *buf)
{
	struct file *fp;
	char buf1[500];
	mm_segment_t fs;
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	//printk("%s--->%d\n",__FUNCTION__,__LINE__);
	if((strncmp(buf,"start",5)==0)||(strncmp(buf,"stop",4) == 0) ||strncmp(buf,"error", 5)==0)
		snprintf(buf1,sizeof(buf1)," browser %s -->(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",buf ,
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	else
		snprintf(buf1,sizeof(buf1),"Set arm freq %s -->(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",buf ,
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	fp = filp_open("/flash/changecpufreqlog.txt",O_RDWR|O_APPEND |O_CREAT  ,0644);
	if(IS_ERR(fp)){
		printk("create file error!\n");
		return;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	//vfs_read(fp,buf1,sizeof(buf1),&pos);
	vfs_write(fp,buf1,strlen(buf1),&fp->f_pos);
	set_fs(fs);
	filp_close(fp,NULL);
	printk("%s--->%d\n",__FUNCTION__,__LINE__);
	return;
}
static pid_t rk28_find_browserpid(char *buf)
{
	  struct task_struct *g, *p;
	  if(!buf){
	  	printk("%s->%d browser common name is null\n",__FUNCTION__,__LINE__);
	  }
	  do_each_thread(g, p) {
	  	//printk("%s->%d find task common name is %s(%d)\n",__FUNCTION__,__LINE__,p->comm,p->pid);
          	if( strcmp(buf,p->comm) == 0 ) 
			return p->pid;
        } while_each_thread(g, p);
	  return -1;
}

/* 20100617,HSL@RK.for test vdd,cpu freq. 
 *  test case : 
 *  1. 1.4v, whick freq is statable,
 *  2. what the min vdd for all kind freq(max , low, normal).
*/
#define SET_ARM_600MHz		0
#define SET_ARM_630MHz		1
#define SET_ARM_660MHz		2
#define SET_ARM_690MHz		3
#define SET_ARM_720MHz		4
#define SET_ARM_750MHz		5
#define SET_ARM_540MHz		6

static unsigned int  scu_changefrq_mode  = SET_ARM_540MHz;   
static ssize_t __scu_12m_store_active(struct rockchip_scu_unit *p , const char *buf , size_t count)
{

	 static pid_t	browser_pid;
        int n = strlen(buf);
	 count -= 1;	/*clear enter char*/
        printk("%s::get cmd %s size =%d  scu_changefrq_mode==%d\n" , __func__ , buf ,count,scu_changefrq_mode);
        if(strncmp(buf, "start",count)== 0) { /* start a browser */
		printk("%s-->%d get cmd start",__FUNCTION__,__LINE__);
		record_changecpufreq_log("start");
		//if(scu_changefrq_mode == SET_ARM_540MHz)
			scu_changefrq_mode = SET_ARM_660MHz;
		//else
			//scu_changefrq_mode = SET_ARM_540MHz;
		switch(scu_changefrq_mode){
			case SET_ARM_600MHz :
									record_changecpufreq_log("600Mhz");
									rockchip_clk_set_arm(600);
									record_changecpufreq_log("start OK");
									break;
			case SET_ARM_630MHz :
									record_changecpufreq_log("630Mhz");
									rockchip_clk_set_arm(630);
									break;
			case SET_ARM_660MHz :
									record_changecpufreq_log("660Mhz-0");
									rockchip_clk_set_arm(660);
									record_changecpufreq_log("660Mhz-1");
									break;
			case SET_ARM_690MHz : 
									record_changecpufreq_log("690Mhz");
									rockchip_clk_set_arm(690);
									break;
			case SET_ARM_720MHz : 
									record_changecpufreq_log("720Mhz");
									rockchip_clk_set_arm(720);
									break;
			case SET_ARM_750MHz :
									record_changecpufreq_log("750Mhz");
									rockchip_clk_set_arm(750);
									break;
						default   :
									record_changecpufreq_log("default540Mhz");
									rockchip_clk_set_arm(540);
									break;
				
		}
        } 
	else if(strncmp(buf, "stop",count)== 0) {  /* stop the browser */
	                char *bro_comm = "android.browser";
		printk("%s-->%d get cmd stop",__FUNCTION__,__LINE__);
		record_changecpufreq_log("stop");
		browser_pid = rk28_find_browserpid(bro_comm);
		if(browser_pid < 0){
			printk("get browser pid error,comm=%s!!\n" , bro_comm );
			record_changecpufreq_log("error get browser pid error!!");
			rockchip_clk_set_arm(540);
			return n;
		}
        	if(kill_proc(browser_pid, SIGKILL, 1) < 0){
			record_changecpufreq_log("error stop browser service fail!!");
			printk("stop browser service fail !!!\n");
			rockchip_clk_set_arm(540);
			return n;
        	}
		rockchip_clk_set_arm(540);
		record_changecpufreq_log("stop OK");
        } 
	else {
        	  printk("%s-->%d no cmd parameter",__FUNCTION__,__LINE__);
                return -EIO;
        }
        return n;
        
}
#else
#define __scu_12m_store_active(p , buf ,count)  -EIO
#endif
/* default kobject attribute operations */
static ssize_t scu_kobj_attr_show(struct kobject *kobj, struct attribute *attr,
			      char *buf)
{
        ssize_t ret = -EIO;
        struct rockchip_scu_unit *p = KTO_SCU_UINT( kobj );

        if ( attr == &p->attr_cur_clk ) {
                int clk = __rockchip_clk_get_uint_clk_hw(p);
                return sprintf(buf, "%d,%06d\n", clk/1000000 , 
                        (clk -(clk/1000000)*1000000)  );
//        } else if ( attr == &p->attr_max_clk ) {
//                return sprintf(buf, "%d\n", p->max_clk );
        } else if ( attr == &p->attr_active ) {
                if( p->id == SCU_IPID_24M)
                        return __scu_init24m_show_active(p , buf , 0);
                //if( p->id == SCU_IPID_12M)
                //        return __scu_init12m_show_active(p , buf );
                return sprintf(buf, "cur_clk:%d,max_clk:%d,prot:%lx,lock:%d,"
                        "tol_child:%d,act_child:%d,div=%d\n" ,
                        p->cur_clk,p->max_clk,p->propt,atomic_read(&p->lock_cnt),
                        p->total_child , p->active_child , p->cur_div );
        } else {
                S_INFO("unknow attr %s" , attr->name );
        }
        return ret;
}
static int scu_kobj_set_clk( struct rockchip_scu_unit *p , int tmp )
{
        int ret= -EIO;
        if( p->id == SCU_IPID_ARM) {
#ifndef CONFIG_RK2818_HOST11
                ret = rockchip_clk_set_arm( tmp );
#endif
        } else if( p->id == SCU_IPID_DSP || p->id == SCU_IPID_CODEC ) {
                ret = rockchip_clk_set_codecdsp( p->id , tmp );
        } else {
                //S_WARN("no support!!\n");
                ret = __rockchip_clk_set_unit_clock( p->id  , tmp );
       }
       return ret;
}
static ssize_t scu_kobj_attr_store(struct kobject *kobj, struct attribute *attr,
			       const char *buf, size_t count)
{
        ssize_t ret = -EIO;
        int     tmp =0;
        struct rockchip_scu_unit *p = KTO_SCU_UINT( kobj );

        sscanf(buf, "%d", &tmp );
        //SCU_BUG("scu %s kojb set:%s to %d,buf=%s" , SCU_SYSFS_NAME(p) , attr->name , tmp , buf); 
        if ( attr == &p->attr_cur_clk ) {
                ret = scu_kobj_set_clk( p , tmp );
//        } else if ( attr == &p->attr_max_clk ) {
//                ret = __rockchip_scu_change_mode( p->id , SCU_MODE_FREQ , tmp );
        } else if ( attr == &p->attr_active ) {
                if( p->id == SCU_IPID_12M)
                        return __scu_12m_store_active(p , buf ,count);
                if( p->id == SCU_IPID_24M)
                        return __scu_init24m_set_active( p , buf , count);
                // 20100711,HSL@RK,ADD lock ctrol.
                switch( tmp ) {
                case 0:
                        ret = rockchip_scu_disableclk( p->id );
                        break;
                case 1:
                        ret = rockchip_scu_enableclk( p->id );
                        break;
                case 2:
                        ret = rockchip_clk_lock_pll( p->id );
                        break;
                case 3:
                        ret = rockchip_clk_unlock_pll( p->id );
                        break;
                default:
                        #if 1 // 20100712,HSL@RK,for test only.
                        if( tmp > 100 ) {
                                rockchip_clk_unlock_pll( p->id );
                                ret = scu_kobj_set_clk( p , tmp );
                                rockchip_clk_lock_pll( p->id );
                        } else {
                                S_INFO("unknow opt %d at %s" ,tmp , attr->name );
                        }
                        #endif
                        break;
                }
        } else {
                S_INFO("unknow attr %s" , attr->name );
        }
        if( ret >= 0 )
                ret = count;
        return ret;
}

struct sysfs_ops scu_sysfs_ops = {
	.show	= scu_kobj_attr_show,
	.store	= scu_kobj_attr_store,
};


static void scu_kset_release(struct kobject *kobj)
{
        S_INFO("kobject: '%s' (%p): %s\n",
        kobject_name(kobj), kobj, __FUNCTION__);
        
}

static struct kobj_type scu_kset_ktype = {
	.sysfs_ops	= &scu_sysfs_ops,
	.release = scu_kset_release,
	.default_attrs = NULL,
};
static void __rockchip_scu_node_attr_init(struct attribute *attr , char * name )
{
        attr->name = name;
        // 20100617,HSL@RK,for test,set to 777.
        attr->mode = 0777;
        //attr->mode = 0644;
        attr->owner = NULL;
}

int __rockchip_scu_node_kobject_init(struct rockchip_scu_unit *p )
{
        struct attribute *scu_def_attrs[4];
        int     result;

        if( !scu_kset )
                return 0;
        __rockchip_scu_node_attr_init(&p->attr_cur_clk , "cur_clk");
        scu_def_attrs[0] = &p->attr_cur_clk;
        __rockchip_scu_node_attr_init(&p->attr_active, "active");
        scu_def_attrs[1] = &p->attr_active;
        
        //__rockchip_scu_node_attr_init(&p->attr_max_clk, "max_clk");
        //scu_def_attrs[2] = &p->attr_max_clk;
        
        scu_def_attrs[2] = NULL;
        scu_kset_ktype.default_attrs = &scu_def_attrs[0];
        memset(&p->kobj,0,sizeof(struct kobject));
        p->kobj.kset = scu_kset;
        kobject_init( &p->kobj , &scu_kset_ktype );
        if ( p->parent )
                result = kobject_add( &p->kobj , &p->parent->kobj, "%s", p->name );
        else 
                result = kobject_add( &p->kobj , NULL , "%s", p->name );
                //result = kobject_add( &p->kobj , kernel_kobj , "%s", p->name );
        if( result < 0 ) {
                S_INFO("add kobject: '%s' failed!", p->name );
                kobject_put( &p->kobj );
                return -ENOMEM;
                }
        return 0;
}

int __rockchip_scu_kset_init( void )
{
        scu_kset = kset_create_and_add("scu", NULL, NULL ); /*kernel_kobj*/
        if (!scu_kset)
                return -ENOMEM;
        return 0;
}

void __rockchip_scu_kset_exit( void )
{
        kset_unregister(scu_kset);
}

#endif  /* SCU_KOJB_SUPPORT */

