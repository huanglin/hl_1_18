/*
 * Gadget Driver for rockchip usb
 *
 * Copyright (C) 2009 Rochchip, Inc.
 * Author: Hsl
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

#include "rk28_usb.h"
#include <linux/syscalls.h>


int     adb_support = 0;
#ifdef RKUSB_SUPPORT
extern int kernel_getlog( char ** start  , int * offset , int* len );
extern int kernel_getlog_start_length( int* start  , int * len );
extern int android_main_getlog( char ** start  , int * offset , int* len );
extern int android_main_getlog_start_length( int* start  , int * len );
extern int __rk28_scu_parse_cmd( char * cdb );
extern volatile int rk28_pm_status ;
extern void rockchip_timer_freeze(int freeze ); /* freeze timer while at break point */
extern void panic(const char * fmt, ...);
void __rk28_force_signal(struct task_struct *tsk, unsigned long addr,
		unsigned int sig, int code );
extern int rk28_msc_switch( int action );
extern int kld_get_tag_data( unsigned int tag , void** data );
extern int rk28_restart( int type ) ;
extern void rk28_panic_reset(void);

static int rkusb_do_thread_cmd( struct rkusb_dev *dev );
static int rkusb_command( struct rkusb_dev *dev );

#define __at_break_point_freeze()                (rk28_pm_status == PMS_SLEEP+4 )
#define  __rkusb_read_enable( dev )             (1) // for msc ext,must can read .(rk28_system_crash>=2)
#define  __rkusb_write_enable( dev )            (rk28_system_crash>=RKDBG_WRITE)
char    rkusb_en_write[16]= "@rkusb_all";
static struct rkusb_dev *_rkusb_dev;
extern struct __except_content rkusb_except_content;
char    rkusb_version[]=RKUSB_KVERSION;
#include "rk28_usb_io.c"        /* 20100204,split for tx/rx packet. */


extern const char linux_banner[];
extern const char rockchip_version[];
extern const char system_type[];
extern char *saved_command_line;

extern void adb_function_enable(int enable);
extern void * rkh_begin;

#if MSC_OEXT_SUPPORT
#define PC_COMPATIBLE           1
#else
#define PC_COMPATIBLE           1
#endif
/* get verion , return size and sdram addr(lba) */
static int rkusb_ucmd_get_version( struct rkusb_dev *dev )
{
        int r = RKUSB_CB_OK_NONE;
        unsigned int off_len[2];
        
        switch( DEV_LUN(dev) ) {
        case LUN_USCM_VER_TAG:
                off_len[0] = (unsigned int)rkh_begin;
                off_len[1] = 0X2000;
                break;
        case LUN_USCM_VER_LOADER:
                {
                /* 20100415,loader,image version get from cmdline. */
                /* 20100517,HSL@RK,for PC DM tools compalite.
                  * cmdline format:bootver=2010-05-15#3.27 .
                  * need data format: loader version X.XX YYYYMMDD.
                */
                #if PC_COMPATIBLE
                        char *buf = __rkusb_rwbuffer_start( dev );
                        char *p = strstr( saved_command_line , "bootver=" );
                        if( !p || p[18] != '#' ) {
                                r = RKUSB_CB_FAILD;
                        } else {
                                strcpy( buf , "loader version "); // 15 bytes
                                memcpy( buf+15 , p+19 , 5 );  // 'x.xx '.
                                memcpy( buf+20 , p+8 ,  4);   // year
                                memcpy( buf+24 , p+13 , 2 ); // month
                                memcpy( buf+26 , p+16 , 2 ); // day
                                buf[28] = 0;
                                off_len[0] = (unsigned int)buf;
                                off_len[1] = 29;
                                //printk("%s:%s\n" , __func__ , buf );
                        }
                #else
                r = RKUSB_CB_FAILD;
                #endif
                }
                break;
        case LUN_USCM_VER_KERNEL:
                off_len[0] = (unsigned int)linux_banner;
                off_len[1] = strlen(linux_banner)+1;
                break;
        case LUN_USCM_VER_ANDROID:
                off_len[0] = (unsigned int)rockchip_version;
                off_len[1] = strlen(rockchip_version)+1;
                break;
        case LUN_USCM_VER_OPENCORE:
        default:
                r = RKUSB_CB_FAILD;
                break;
        }
        if( r == RKUSB_CB_OK_NONE )
                rkusb_normal_data_xfer_onetime(dev , off_len );
        return r;
}

/*
 * 20100111,HSL@RK,use LUN to dicide call at task or irq.
 * lun==0: at irq , other at task.
 * 20100420,HSL@RK,define some special ARGV,like current,task(pid).
 * start with"#".
 */
static int rkusb_function_call_cb( struct rkusb_dev *dev )
{
         int     ret;
         rk28printk("%s::cmd=%s,crash=%d\n" , __func__ , (char*) dev->req_out->buf , rk28_system_crash );
         if( !__rkusb_write_enable() ) {
                //if( !strcmp((char*) dev->req_out->buf , rkusb_en_read) ) {
                //        rk28_system_crash = 2;
                //        rkusb_send_csw_result( dev , rk28_system_crash);
                //        return RKUSB_CB_OK_NONE;
                //}
                if( !strcmp((char*) dev->req_out->buf , rkusb_en_write) ) {
                        rk28_system_crash = RKDBG_WRITE;
                        rkusb_send_csw_result( dev , rk28_system_crash);
                        return RKUSB_CB_OK_NONE;
                }
                return RKUSB_CB_FAILD;
        }
        if( DEV_LUN(dev) == 0 ) {
                ret = __rk28_scu_parse_cmd((char*) dev->req_out->buf);
                rk28printk("cmd ret = 0x%x(%d)" , ret ,ret );
                rkusb_send_csw_result( dev , ret );
        } else {
                rkusb_wakeup_thread( dev );
        }
        return RKUSB_CB_OK_NONE;
}

static int rkusb_function_call( struct rkusb_dev *dev )
{
        rkusb_normal_data_xfer( dev , rkusb_function_call_cb );   
        return RKUSB_CB_OK_NONE;
}

static void __rksub_copy_task_info( TASK_INFO *ti , struct task_struct * p)
{
        get_task_comm(ti->comm , p);
        ti->size = sizeof (*ti );
        ti->state = p->state;
        ti->stack = p->stack;
        ti->flags = p->flags;
        ti->prio = p->prio;
        ti->static_prio = p->static_prio;
        ti->normal_prio = p->normal_prio;
        ti->sched_class = (void*)p->sched_class;
        ti->sum_exec_runtime = p->se.sum_exec_runtime;
        ti->mm = p->mm ;
        ti->active_mm = p->active_mm;
        ti->pid = p->pid;
        ti->rt_priority = p->rt_priority;
        ti->utime = p->utime;
        ti->stime = p->stime;
        ti->this_ptr = (unsigned long)p;
        if( p->files ) {
                void * pt = p->files->fd_array;
                memcpy( ti->fd_array , pt , sizeof (ti->fd_array) );
        } else
                memset( ti->fd_array , 0 , sizeof ti->fd_array );
                
//        strcpy( ti->cmd , ti->comm );
//        if( !__system_crashed() && !__at_break_point_freeze() )
//                rkusb_get_task_cmdline( p , ti->cmd , sizeof( ti->cmd) );
                
        ti->tgid = p->tgid;
        ti->start_time = timespec_to_ns( &p->start_time);
        
        ti->uid = p->uid;
        ti->euid = p->euid ; 
        ti->suid = p->suid;
        ti->fsuid = p->fsuid;

        ti->gid = p->gid;
        ti->egid = p->egid ; 
        ti->sgid = p->sgid;
        ti->fsgid = p->fsgid;
        
        ti->policy = p->policy;
}

/* 20100420,HSL@RK,support offset.return task from lba.
*/
static int rksub_get_sym_tasks( struct rkusb_dev *dev )
{
        ALL_TASK                 at;
        TASK_INFO               *ti;
        struct task_struct *g, *p;
        int     buf_full = 0;
        int     tn , tntoal;
        int start = DEV_OFFSET(dev);
        
        ti = (TASK_INFO*)__rkusb_rwbuffer_start( dev );
        at.size = sizeof( at );
        at.start_ts = ti;
        at.ti_size = sizeof(*ti);
        tntoal = 0;
        tn = 0;
        read_lock(&tasklist_lock);
        do_each_thread(g, p) {
                if( !buf_full && tntoal >=  start) {
                        if( ti+1 > (TASK_INFO*)__rkusb_rwbuffer_end( dev ) ) {
                                buf_full = 1;
                                tn = tntoal;
                        } else {
                                __rksub_copy_task_info( ti , p );
                                ti++;
                        }
                }
                tntoal ++;
        } while_each_thread(g, p);
        read_unlock(&tasklist_lock);
        /* 20100504,HSL@RK,copy idle task(pid=0) */
        if( ti+1 < (TASK_INFO*)__rkusb_rwbuffer_end( dev ) ) {
                p = &init_task;
                __rksub_copy_task_info( ti , p );
                tntoal++;
        }
        if( !tn )
                tn = tntoal;
        at.task_num = tn;
        at.task_total_num = tntoal;
        at.now = ktime_to_ns(ktime_get() );
        rkusb_normal_data_xfer_onetime( dev , &at );
        return RKUSB_CB_OK_NONE;
}

static char* __rksub_copy_task_vma(struct rkusb_dev *dev ,
        TASK_VAM *tm , struct task_struct * p , char *dest )
{
        TASK_VMA_INFO   *tv;
        struct mm_struct *mm;
        struct vm_area_struct * mmap = NULL;
        int       buf_full = 0;
        int       n,ntotal;

        tv = (TASK_VMA_INFO*)dest; 
        memset( tm , 0 , sizeof(*tm) );
        mm = p->mm;
        tm->size = sizeof(*tm);
        tm->flag = (1<<0);
        tm->pid = p->pid;
        if( !mm ) {
                mm = p->active_mm;
                tm->flag &= ~(1<<0);
        }
        /* 20100409,HSL@RK, kernel thread mm(active_mm) may be NULL. */
        if( mm ) {
                tm->start_code = mm->start_code;
                tm->end_code= mm->end_code;
                tm->start_data= mm->start_data;
                tm->end_data = mm->end_data;
                tm->start_brk = mm->start_brk;
                tm->brk = mm->brk;
                tm->start_stack = mm->start_stack;
                tm->arg_start = mm->arg_start;
                tm->arg_end = mm->arg_end;
                tm->env_start = mm->env_start;
                tm->env_end = mm->env_end;
                mmap = mm->mmap;
        }
        tm->vi = tv;
        tm->vi_size = sizeof(TASK_VMA_INFO);
        n = ntotal = 0;
        while( mmap ) {
                if( buf_full ) 
                        goto loop_none;
                tv->size = sizeof(TASK_VMA_INFO);
                tv->start = mmap->vm_start;
                tv->end = mmap->vm_end;
                tv->flags = mmap->vm_flags;
                tv->pgoff = mmap->vm_pgoff;  /* 20100226,HSL@RK,add offset in file*/
                tv->file = mmap->vm_file;
                memset( tv->mape_filename , 0 , sizeof(tv->mape_filename) );
                if( tv->file ) {
                        if( strlen( FILENAME(mmap->vm_file)  ) >= sizeof(tv->mape_filename) )
                                strncpy(tv->mape_filename,FILENAME(mmap->vm_file)+
                                        strlen( FILENAME(mmap->vm_file)  )-sizeof(tv->mape_filename)+1,
                                        sizeof(tv->mape_filename)-1 );
                        else
                                strcpy(tv->mape_filename,FILENAME(mmap->vm_file) );
                }
                tv++;
                if( tv >= (TASK_VMA_INFO*)__rkusb_rwbuffer_end( dev ) ) {
                        buf_full = 1;
                        n = ntotal+1;
                }
loop_none:                        
                ntotal++;
                mmap = mmap->vm_next;
                
        }
        if( !n )
                n = ntotal;
        tm->vi_xfer = n;
        tm->vi_tolnum = ntotal;
        rk28printk("%s::task=%s,pid=%d,total vma=%d\n", __func__ , p->comm , p->pid , ntotal );
        return (char*)tv;
}

/* lab : = pid .*/
static int rkusb_get_task_mm( struct rkusb_dev *dev )
{
        TASK_VAM        tm;
        struct task_struct *p;
        int     pid = DEV_OFFSET(dev);
        if( pid == 0 )
                p = current ;
        else {
                p = find_task_by_pid( pid );
                if( !p )
                        return RKUSB_CB_FAILD;
        }
        __rksub_copy_task_vma( dev , &tm , p , __rkusb_rwbuffer_start( dev ));
        rkusb_normal_data_xfer_onetime( dev , &tm );
        return RKUSB_CB_OK_NONE;
}

static int rkusb_get_kernel_symbols( struct rkusb_dev *dev )
{
        struct __kernel_symbol ks;
        ks.size = sizeof( ks );
        ks._stext = _stext;
        ks._text = _text;
        ks._etext = _etext;
        ks._data = __data_start ; //_data;
        ks._edata = _edata;
        ks.__bss_start = __bss_start;
        ks._end = _end;

        ks.kallsyms_start = (unsigned char*)kallsyms_addresses;
        ks.total_syms_size = (unsigned char*)__start_rodata - ks.kallsyms_start;
        ks._kallsyms_num_syms = kallsyms_num_syms;
        ks._kallsyms_addresses = (unsigned long*)kallsyms_addresses;
        ks._kallsyms_markers = (unsigned long*)kallsyms_markers;
        ks._kallsyms_names = (unsigned char*)kallsyms_names;
        ks._kallsyms_token_index = (unsigned short*)kallsyms_token_index;
        ks._kallsyms_token_table = (unsigned char*)kallsyms_token_table;
        rkusb_normal_data_xfer_onetime( dev , &ks );
        rk28printk("symbols addres=0x%p,names=0x%p,syms=0x%lx\n",
                ks._kallsyms_addresses,ks._kallsyms_names,ks._kallsyms_num_syms);
        return RKUSB_CB_OK_NONE;
}

struct __except_content rkusb_except_content;
static pid_t           except_upid;
static char            except_ucomm[TASK_COMM_LEN];
unsigned long 	       except_addr; /* for data abort addr. */

/*
    if we have multi vma: like PV author vmas:
15:start=0x4040a000,end=0x40509000,flag=0x73,off=0x4040a,file=
16:start=0x40509000,end=0x4050a000,flag=0x70,off=0x40509,file=
17:start=0x4050a000,end=0x40609000,flag=0x73,off=0x4050a,file=
18:start=0x40609000,end=0x4060a000,flag=0x70,off=0x40609,file=
19:start=0x4060a000,end=0x40709000,flag=0x73,off=0x4060a,file=
20:start=0x40709000,end=0x4070a000,flag=0x70,off=0x40709,file=
21:start=0x4070a000,end=0x40711000,flag=0x73,off=0x4070a,file=
22:start=0x40711000,end=0x40712000,flag=0x70,off=0x40711,file=
23:start=0x40712000,end=0x40811000,flag=0x73,off=0x40712,file=
    when sp = 0x40508b98 , the stack may be from 0x4040a000 -- 0x40811000,
    not 0x4040a000 -- 0x40509000.
*/
static unsigned long __rksub_find_task_sp_vma(struct task_struct * p,
        unsigned long sp )
{
        struct mm_struct *mm;
        struct vm_area_struct * mmap;
        unsigned long sp_end = 0 ;
        mm = p->mm;
        if( !mm ) {
                mm = p->active_mm;
        }
        if( mm ) {
               mmap = mm->mmap;
		while( mmap ) {
		        if( !sp_end ) {
                        	if( mmap->vm_start <= sp && sp <= mmap->vm_end ) {
                        	        sp_end = mmap->vm_end;
                        	}
                       } else {
                            if( mmap->vm_start == sp_end ) {
                                    sp_end = mmap->vm_end;
                            } else {
                                    break;
                            }
                       }
                	mmap = mmap->vm_next;
        	}
        }
        return sp_end;
}

static int __rksub_get_value_around_reg(struct task_struct * p,
        TASK_CRASH      *cr ,  struct pt_regs *regs)
{
        struct mm_struct *mm;
        struct vm_area_struct * mmap;
        int     i;
        int     cp = sizeof(cr->reg_value[0]);
        mm = p->mm;
        if( !mm ) {
                mm = p->active_mm;
        }
        cr->reg_flag = 0;
        printk("sizeof reg_value=%d\n" , cp );
        if( mm ) {
               for( i = 0 ; i < 16 ; i++ ) {
                      unsigned long sp = (unsigned long)regs->uregs[i];
                      mmap = mm->mmap;
                      sp &= ~3; /* 4 aligned */
        		while( mmap ) {
                            	if( mmap->vm_start <= sp && sp <= mmap->vm_end ) {
                            	        cr->reg_flag |= 1<<i;
                            	        cp = copy_from_user( cr->reg_value[i], (const void __user *)(sp-0x10), sizeof(cr->reg_value[i]));
                            	        break;
                            	}
                        	mmap = mmap->vm_next;
                	}
        	}
        }
        return cp;
}
/*   20100316,HSL@RK,copy info of crash task for 
  *   later debug.things: copy task_struct,copy vma,copy some stack.
  *   data put into __rkusb_rwbuffer_start buffer.
  *   20100331,HSL@RK,for TRACE-GETREG function,remain the first 8K space.
  */
static void rkusb_record_crash_task( unsigned long sp ) 
{
        struct rkusb_dev *dev = _rkusb_dev;
        struct task_struct  *t = current;
        char            *buf = __rkusb_rwbuffer_start(dev)+CRASH_OFFSET;
        TASK_CRASH      *crash = (TASK_CRASH*)buf;
        TASK_INFO *ti;
        TASK_VAM        *tm;
        int             stack,left ;
	 unsigned long  sp_end;
        if( crash->magic == CRASH_MAGIC && crash->pid == t->pid )
                return;
	 printk(KERN_EMERG "record excp task %s(%d)\n" ,t->comm , t->pid);
        crash->size = sizeof(*crash);
        crash->magic = CRASH_MAGIC;
        crash->ktime = ktime_get();
        crash->pid = t->pid;
        crash->mode = rkusb_except_content.mode;
        crash->addr = rkusb_except_content.addr;
        
        ti = (TASK_INFO*)(buf+crash->size);
        __rksub_copy_task_info(ti,t);
        crash->task = sizeof(*ti);

        tm = (TASK_VAM*)((char*)ti+crash->task);
        buf = __rksub_copy_task_vma( dev , tm , t , (char*)(tm+1));
        crash->mm = sizeof(*tm);
        crash->vma = buf - (char*)(tm+1);
        __rksub_get_value_around_reg( t , crash , (struct pt_regs *)rkusb_except_content.uregs );
        stack = __rkusb_rwbuffer_end(dev) -buf;
        rk28printk("task %s crash,pid=%d,sp=0x%lx,stack=0x%x\n" , t->comm ,
                t->pid , sp , stack  );
        if( stack > 0 ) {
		sp_end = __rksub_find_task_sp_vma(t , sp );
		if( sp_end )
			left = sp_end-sp; /* get the real stack length */
		else 	
			left = stack;	/* 20100510,default len */
                if( stack < left ){	
			printk("used stack 0x%x > left space 0x%x\n" , left ,stack );
                        left = stack;
		}
               stack = copy_from_user( buf , (const void __user *)sp, left);
               crash->stask = left-stack;
        } else {        /* no space for stack */
                crash->stask = 0;
                rk28printk("%s::task %s crash,no buffer space for stack\n" ,__func__,  t->comm );
        }
}

/* 20100426,HSL@RK,give condition for whitch user task can panic kernel.
*/
void set_except_regs(struct pt_regs *new_regs , int mode , 
        unsigned long addr, unsigned int fsr)
{
        rkusb_except_content.addr = addr ;
        rkusb_except_content.fsr = fsr;
        rkusb_except_content.mode = mode;
        rkusb_except_content.pid = current->pid;
        memcpy(rkusb_except_content.uregs,new_regs , sizeof(struct pt_regs) );
        if( user_mode(new_regs ) ) {
                if( __rkusb_debug_mod() ) {
                        if( except_upid && current->pid != except_upid )
                                goto just_recored;
                        if(  except_ucomm[0] && strncmp(except_ucomm,current->comm , strlen(except_ucomm) ) )
                                goto just_recored;
			if( except_addr && addr != except_addr )
				goto just_recored;	
                        printk(KERN_EMERG "Kernel panic: panic by user mode fault(%d,%s)\n" ,except_upid ,except_ucomm );
                        rk28_panic_reset();
                        return ;
                        //panic("panic by user mode fault\n"); /* 20100407,HSL@RK,panic NO_RETURN !!*/
                } 
just_recored:                
                rkusb_record_crash_task( new_regs->ARM_sp);
        }
}

extern int profile_check( void *pc_buf , int len , int* kernel , int *user );
static int rkusb_get_symbol( struct rkusb_dev *dev )
{
        switch ( DEV_FUNC(dev) ) {
        case FUNC_GSYM_KERNEL:
                return rkusb_get_kernel_symbols( dev );
        case FUNC_GSYM_GETTASKS:
                return rksub_get_sym_tasks( dev );
        case FUNC_GSYM_GETTASKVM:
                return rkusb_get_task_mm( dev );
        case FUNC_GSYM_PROFILE:
                {
                char * buf = __rkusb_rwbuffer_start(dev);
                PROFILE_INFO   pe;
                pe.size = sizeof( pe );
                pe.total_len = profile_check( buf , THREAD_SIZE , &pe.npc , &pe.unpc );
                pe.buf = (unsigned long)buf;
                pe.now = ktime_to_ns( ktime_get() );
                pe.ksize = pe.usize = 8; /* keep the size of struct prof_entry define at profile.c */
                rk28printk("%s::profile n=%d,un=%d\n" , __func__ , pe.npc , pe.unpc );
                rkusb_normal_data_xfer_onetime( dev , &pe );
                return 0;
                }
        default:
                break;
        }
        return RKUSB_CB_FAILD;
}

#include "rk28_msc_ext.c"

static int rkusb_usb_command( struct rkusb_dev *dev )
{
        int r = RKUSB_CB_OK_CSW;
        unsigned int off_len[2];
        rk28printk("%s::func=0x%x,len=%d\n" , __func__ , DEV_FUNC(dev),DEV_LENGTH(dev));
        switch( DEV_FUNC(dev) ) {
                case FUNC_UCMD_DEVREADY:
                        break;
                case FUNC_UCMD_DISCONNECTMSC:
                        rk28_msc_switch( 0 );
                        break;
                case FUNC_UCMD_SYSTEMINFO:
                        {
                        unsigned int off_len[2];
                        off_len[0] = (unsigned int)system_type;
                        off_len[1] = strlen(system_type)+1;
                        rkusb_normal_data_xfer_onetime(dev , off_len );
                        r = RKUSB_CB_OK_NONE;
                        }
                        break;
                case FUNC_UCMD_RESTART:
                        rk28printk("get restart cmd,lun=%d\n" , DEV_LUN(dev));
                        /* 0: normal restart , 1:to loader rkusb*/
                        rk28_restart(DEV_LUN(dev));
                        break;
                case FUNC_UCMD_GETVERSION:
                        r = rkusb_ucmd_get_version( dev );
                        break;
                case FUNC_UCMD_GETCHIPINFO:
                        {
                        /* 20100515,HSL@RK,use read tag sdram instead.*/
                        #if PC_COMPATIBLE
                        off_len[1] = (unsigned int)kld_get_tag_data(0X524B0005,(void**)&off_len[0]);
                        rkusb_normal_data_xfer_onetime(dev , off_len );
                        r = RKUSB_CB_OK_NONE;
                        #else
                        r = RKUSB_CB_FAILD;
                        #endif
                        }
                        break;
                case FUNC_UCMD_GETSN:
                        {
                        #if PC_COMPATIBLE
                        off_len[1] = (unsigned int)kld_get_tag_data(0X524B0006,(void**)&off_len[0]);
                        rk28printk("get sn:off=0x%x,len=%d\n" , off_len[0],off_len[1] );
                        rkusb_normal_data_xfer_onetime(dev , off_len );
                        r = RKUSB_CB_OK_NONE;
                        #else
                        r = RKUSB_CB_FAILD;
                        #endif
                        }
                        break;
                case FUNC_UCMD_DEBUGINFO:
                        {
                        RKDEBUG_INFO      *dbi = DEV_FAST_ALLOC(dev,RKDEBUG_INFO);
                        memset( dbi , 0 , sizeof(RKDEBUG_INFO) );
                        dbi->size = sizeof(RKDEBUG_INFO);
                        dbi->adb_support = adb_support;
                        dbi->dbg_level = __rkusb_write_enable(dev)?2:(__rkusb_read_enable(dev)?1:0);
                        dbi->tr_length = dev->req_buf_length;
                        dbi->save_length = dev->buf_length;
                        dbi->save_buf = (unsigned long)__rkusb_rwbuffer_start(dev);
                        strcpy(dbi->cur_device,"all");
                        
                        strcpy( dbi->cur_version,rkusb_version);
                        dbi->addr_crash = (unsigned long)&rk28_system_crash;
                        dbi->crash_offset = CRASH_OFFSET;
                        dbi->addr_upid = (unsigned long)&except_upid;
                        dbi->addr_ucomm = (unsigned long)except_ucomm;
                        #if 0
                        dbi->cmds[0]=K_FW_SDRAM_READ_10;
                        dbi->cmds[1]=K_FW_SDRAM_WRITE_10;
                        dbi->cmds[2]=K_FW_TRACE;
                        dbi->cmds[3]=K_FW_GETLOG;
                        dbi->cmds[4]=K_FW_FUNCALL;
                        dbi->cmds[5]=K_FW_GETSYMB;
                        dbi->cmds[6]=K_FW_GETRESULT;
                        dbi->cmds[7]=K_FW_USBCMD;
                        #endif
                        // 20100712,HSL@RK,add log info.only support prinkt and log main current.
                        dbi->klog_addr = (unsigned long)dev->log[0].va_start;
                        dbi->klog_size = (unsigned long)dev->log[0].total_len;
                        dbi->alogm_addr= (unsigned long)dev->log[1].va_start;
                        dbi->alogm_size = (unsigned long)dev->log[1].total_len;
                        rkusb_normal_data_xfer_onetime(dev , dbi );
                        r = RKUSB_CB_OK_NONE;
                        }
                        break;
                 case FUNC_UCMD_OPENADB:
                        {
                        /* lun:0 close,1: open */
                        adb_function_enable(DEV_LUN(dev));
                        dwc_otg_set_vbus_status( 0 ); /* force to reconnect! */
                        }
                        break;
                 case FUNC_UCMD_UNBOOT:
                        if( DEV_LUN(dev) ) {
                                system_state = SYSTEM_RUNNING;
                        } else {
                                system_state = SYSTEM_HALT;
                        }
                        rk28printk("%s::sys state=%d\n" , __func__ , system_state );
                case FUNC_UCMD_RKUSBVER:
                        {
                                if( DEV_OFFSET(dev) > 0 ) {
                                        r = rkusb_omsc_ext_cmd( dev );
                                        if( r != RKUSB_CB_UNHANDLE )
                                                break;
                                } 
                                off_len[0] = (unsigned int)rkusb_version;
                                off_len[1] = sizeof(rkusb_version);
                                rkusb_normal_data_xfer_onetime(dev , off_len );
                                r = RKUSB_CB_OK_NONE;
                        }
                        break;
                default:
                        break ;
        }
        return r;
}


#include "rk28_trace.c"
#include "rk28_xfile.c"

static int rkusb_get_lastcmd_result( struct rkusb_dev *dev )
{
        if( DEV_LENGTH(dev) != RKUSB_RESULT_LEN )
                return RKUSB_CB_FAILD;
        rkusb_normal_data_xfer_onetime( dev , &dev->cr );
        return RKUSB_CB_OK_NONE;
}
static int rkusb_command( struct rkusb_dev *dev )
{
        int r= RKUSB_CB_FAILD;
        //dev->cb.DataTransferLength = __be32_to_cpu( dev->cb.DataTransferLength );
        //dev->cb.Lba = __be32_to_cpu( dev->cb.Lba );
        //dev->cb.pid = __be16_to_cpu( dev->cb.pid );

        rkusb_print_cb( dev );
        if( DEV_LENGTH(dev) > dev->req_buf_length ){
                rk28printk("data length[0x%x] > limit[0x%x]\n" , DEV_LENGTH(dev),dev->req_buf_length);
                DEV_LENGTH(dev) = dev->req_buf_length;
        }
        //dev->thread_task->mm = NULL;
        dev->slave_task = NULL;
        switch ( DEV_CMD(dev) )
        {
        case K_FW_GETRESULT:
                r = rkusb_get_lastcmd_result( dev );
                break;
        case K_FW_TRACE:
               r = rkusb_do_trace( dev );
               break;
        case K_FW_USBCMD:
               r = rkusb_usb_command( dev );
               break;
        case K_FW_XFERFILE:
                r = rkusb_do_xfer_file( dev );
                break;
        case K_FW_FUNCALL:
                r = rkusb_function_call( dev );
                break;
        case K_FW_SDRAM_READ_10:
                r = rkusb_read_sdram( dev );
                break;
        case K_FW_SDRAM_WRITE_10:
                r = rkusb_write_sdram( dev );
                break;
        case K_FW_GETSYMB:
                r = rkusb_get_symbol( dev );
                break;
        case K_FW_TASKFUN:
                r = rkusb_task_cmd( dev );
                break;
        default:
                return RKUSB_NOT_HANDLED;
        } 
        if( RKUSB_CB_FAILD == r ) {
                /* 20100330,HSL@RK,for failed cmd,must handle it */
                if( DEV_LENGTH(dev) ) {
                        DEV_OFFSET(dev) = 0;
                        rkusb_normal_data_xfer( dev , rkusb_failed_cb ); 
                        return 0 ; 
                } else {
                        r = RKUSB_CB_FAILD_CSW;
                }
        } 
        if( RKUSB_CB_OK_CSW == r )
                rkusb_send_csw( dev , RKUSB_STATUS_PASS );
        else if( RKUSB_CB_FAILD_CSW == r )
                rkusb_send_csw( dev , RKUSB_STATUS_FAIL);
        return 0;
}

#ifdef LOG_SHELL

static int rkusb_shell_w_off = 0;
static int rkusb_shell_r_off = 0;
/* for user application STDOUT */
#define LOG_SHELL_GETOFFSET( p , off )        (off&(p->total_len-1))
static int rkusb_shell_writelog( struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
        struct log_buffer *p = &_rkusb_dev->log[2];
        int ret = 0;
        int written = 0;
        size_t size;
        if( !(p->property & LOG_PROT_MAYWRITE ) )
                return 0;
        if( count == 2 && buf[0] == '#' && buf[1] == ' ' ) {
                //rk28printk("%s:found last statement\n" ,__func__ );
                return count ;
        }
        
        //rkusb_print_bin( (char*)buf , count );
        if( !p->va_start )
                return 0;
        for (;;) {
	         size = count;	
                         if( size + LOG_SHELL_GETOFFSET(p,rkusb_shell_w_off) > p->total_len )
                                size = p->total_len - LOG_SHELL_GETOFFSET(p,rkusb_shell_w_off);
		memcpy(p->va_start+LOG_SHELL_GETOFFSET(p,rkusb_shell_w_off) , buf, size );
		written += size;
		buf += size;
		count -= size;
                         spin_lock( &_rkusb_dev->lock );
                         rkusb_shell_w_off += size;
                         spin_unlock( &_rkusb_dev->lock );
                         ret = written;
	        if (!count)
		break;
                        ret = -ERESTARTSYS;
                        if (signal_pending(current))
                	break;
                        cond_resched();
	}
        return ret;
}

extern void * console_register_direct_write( void * write );
extern struct console *console_drivers;
static int rkusb_shell_initlog( struct log_buffer *p  )
{
        int     indx;
        struct tty_driver *tty_drv = console_drivers->device( console_drivers , &indx );
        struct tty_struct *tty_s = tty_drv->ttys[indx ];
        if( p->private_data )
                return 0;
        if( !tty_s )
                return -EIO;
        
        p->total_len = 4096;
        p->va_start = kmalloc(p->total_len , GFP_ATOMIC );
        if( !p->va_start ) {
                p->total_len = 0;
                p->property = 0;
                p->private_data = NULL;
                return -ENOMEM;
        }
        rk28printk("%s:register shell write\n" ,__func__ );
        p->offset = p->len = 0;
        p->private_data = tty_s;
        console_register_direct_write( rkusb_shell_writelog );
        return 0;
}

/*
 *   return : = count ok, else failed 
 */
static int rkusb_shell_log_write( struct log_buffer *p , char *buf, int count )
{
        char *pt = buf;
        struct tty_struct *tty_s = (struct tty_struct *)p->private_data;
        int     i = 0;
        int flag = TTY_NORMAL;

        buf[count] = 0;
        rk28printk("%s:log=%s,buf=%s,count=%d,tty_s=0x%p\n" ,__func__ , p->name , pt , count , tty_s );
        if( !tty_s ) {
                return 0;
        }
        #if 0 /* add at pc tools */
        if ( buf[count-1]  != '\n' ) {
                buf[count++] = '\n';
                buf[count] = 0;
        }
        #endif
                
        while( *pt && i < count ) {
                tty_insert_flip_char(tty_s, *pt , flag);
                pt++;
                i++;
        }
        tty_flip_buffer_push( tty_s );
        p->property |= LOG_PROT_MAYWRITE;
        return count;
}

static int rkusb_shell_getlog( char ** start  , int * offset , int* len )
{
        return 0;
}

static int rkusb_shell_getlog_start_length( int * start , int* len )
{
        struct log_buffer *p = &_rkusb_dev->log[2];
        *start = LOG_SHELL_GETOFFSET(p,rkusb_shell_r_off);
        *len = rkusb_shell_w_off - rkusb_shell_r_off;
        rkusb_shell_r_off = rkusb_shell_w_off;
        p->property &= ~LOG_PROT_MAYWRITE;      /* can write until next shell cmd */
        return 0;
}
#endif


static int __init rkusb_init_internal( void )
{
               struct rkusb_dev *dev;
               struct log_buffer *p ;
        	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
        	if (!dev)
		        return -ENOMEM;
		/* 20100416,HSL@RK,found max tasks=223,need 58K. */
		/* 20101123,HSL@RK,set to 96K for more stack size when record crash task. */
            dev->buf_length = 96*1024;
            dev->req_buf_length = 4*1024;
            dev->buf = kmalloc(dev->buf_length+dev->req_buf_length*2,GFP_KERNEL); 
            if( !dev->buf ) {
                    kfree( dev );
                    return -ENOMEM;
            }

             dev->f[0].buf_in = dev->buf+dev->buf_length;
             dev->f[1].buf_in = dev->f[0].buf_in+dev->req_buf_length;
             __rkusb_bk_init_stack(dev->buf ,dev->buf_length);
             //rkusb_trace_fileop("sys_unlink","delete.txt");

             dev->luns = ARRAY_SIZE( dev->log );
             p = &dev->log[0];
             kernel_getlog( &p->va_start,&p->offset,&p->total_len);
             p++;
             android_main_getlog( &p->va_start,&p->offset,&p->total_len);             
#ifdef LOG_SHELL
             P++;
             strcpy( p->name , "shell log" );
             p->property = LOG_PROT_READ|LOG_PROT_WRITE; /* read and write */
             p->getlog = rkusb_shell_getlog;
             p->getlog_start_length = rkusb_shell_getlog_start_length;
             p->setlog = rkusb_shell_log_write;
#endif 

             //spin_lock_init(&dev->lock);
             dev->cr.Signature= RKUSB_RESULT_SIG;
             dev->cs.Signature = RKUSB_BULK_CS_SIG;
             INIT_WORK(&dev->theadwork,__rkusb_thread_work);
             register_undef_hook(&rkusb_arm_break_hook);
             _rkusb_dev=dev;

#ifdef RK28_PRINT       /* 20100324,HSL@RK,open while debug. */
             //rk28_system_crash = RKDBG_CUSTOMER0;
#endif             

             /*XXXX:20100324,HSL@RK,open debug here for 产线测试!!*/
             /* 正常情况下，应该关闭!*/
             //rk28_system_crash = RKDBG_CUSTOMER0;
             //strcpy(except_ucomm,"Binder Thread #"); /* only crash for binder. */
             //except_addr = 0xdeadd00d;	/* devAbort */
             return 0;
}

usb_complete_cb    gadget_out_cb = NULL;
static int rkusb_reinit_req( struct usb_func  *f,struct usb_ep* ep_in,struct usb_ep* ep_out ) 
{
            if( f->in && f->req_in ) {
                    usb_ep_free_request(f->in , f->req_in);
                    f->req_in = NULL;
                    f->in = NULL;
            }
            if( !ep_in )
                    return 0;
            f->req_in = usb_ep_alloc_request( ep_in , GFP_ATOMIC);
            if( !f->req_in )
                     return -ENOMEM;
            f->in = ep_in;
            f->req_in->buf = f->buf_in;
            f->req_in->complete = rkusb_complete_in;
            f->out = ep_out;
            gadget_out_cb = rkusb_complete_out;
            return 0;
}
/*
*  init call at msc or adb set alt.use buf_length to distinguish.
*  adb first!
* __attribute__((weak))
*/
int  rkusb_init(struct usb_ep* in_ep , struct usb_ep* out_ep , int fsg )
{
        if( _rkusb_dev->disable ){
                gadget_out_cb = NULL;
                return 0;
        }
        rk28printk("%s::build[0002,gcc %d.%d.%d]\n" , __func__ , 
                __GNUC__ ,__GNUC_MINOR__,__GNUC_PATCHLEVEL__);
                
        /*
         * 4 init case :
         * 0: only fsg , 1:only adb , 2:fsg first,adb second , 3:adb first,fsg second.
         * we want:if have adb,select adb ,then select fsg.
         * todo:: may selecy both?. fsg==0: adb first, 1:fsg first.
         * 20101229,both support!!fsg=1: msc , fsg=0: adb.
        */
        S_INFO("%s:fsg=%d,ep in=0x%p,ep out=0x%p\n" , __func__ , fsg , in_ep , out_ep );
        if( fsg < 2 ) {
                rkusb_reinit_req( &_rkusb_dev->f[fsg] ,in_ep,out_ep);
        }
        return 0;
}

int rkusb( int en )
{
        _rkusb_dev->disable = 0;
        switch( en ){
        case 0:
                gadget_out_cb = NULL;
                break;
        case 1:
                gadget_out_cb = rkusb_complete_out;
                break;
        default:
                gadget_out_cb = NULL;
                _rkusb_dev->disable = 1;
                break;
        }
        return en;
}

fs_initcall(rkusb_init_internal);
//module_init(rkusb_init_internal);
#else
void profile_irq_check( int irq )
{
        return ;
}
void set_except_regs(struct pt_regs *new_regs , int mode , 
        unsigned long addr, unsigned int fsr)
{
        return ;
}
#endif


