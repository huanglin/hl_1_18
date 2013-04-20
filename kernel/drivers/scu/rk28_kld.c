/*
 * function about kernel and loader 
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/bug.h>
#include <asm/memory.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/rk28_debug.h>
#include <asm/arch/rk28_scu.h>

#else
#include <string.h>
//#include "linux_types.h"
//#include "ctypes.h"
#include "typedef.h"
#define ENOMEM		12
#define virt_to_phys( x )	((unsigned int)x)
#define local_irq_disable()	
#define SCU_BASE_ADDR_VA	0
#endif

#define ALIGN4( size )          ((((size)+3)>>2)<<2)
/*
 * system restart test function. use WDT .when kernel panic,restart system.
 * 20090919,add for other fault.
 * 20091011,test for communicate with loader.
 * 20100202,HSL@RK,not changed the tag value, use hardcode other place.
 */

struct rktag_header {
             __u32 tag;
             __u32 size;
	
};
#define RKTAG_NONE                      0X00000000      /* loader to kernel */
#define KRTAG_NONE                      0X00000000      /* kernel to loader */
#define NEXT_TAG( h )                   ((struct rktag_header*)((char*)h+h->size) )

#define RKTAG_IDTAG                     0X524B4B52      // loader to kernel
#define KRTAG_IDTAG                     0X4B52524B      // kernel to loader
#define RKTAG_IDSIZE                    sizeof(struct rktag_header)

/* chip info , when change,need to change pc tools. */  
#define RKTAG_CHIP                       0X524B0000       
struct rktag_chip {
                __u32 chip_sn0;
                __u32 chip_xxx;
	__u32 peripheral;
	__u32 flt_version;
	
	/* 20100202,HSL@RK,info for id block sector 0 */
	__u16  disk0_size_mb;
	__u16  disk1_size_mb;
	
	__u32  machine_id;  
	
	__u32  flash_sectors;
	__u8    flash_at;       /* access time */
	__u8    flash_page_sectors;
	__u8    flash_ecc_bits;
	__u8    flash_pad;
	__u16  flash_block_sectors;
	
	__u16  total_reserved_blocks;
	
	__u16  first_ext_block;
	__u16  last_ext_block;
	__u16  first_reserved_block;
	__u16  last_reserved_block;
};
#define RKTAG_RUNTIME                0X524B0001         /* loader run time */
struct rktag_runtime {
             __u32 total_ms;
             __u32 timer00;
             __u32 timer01;
             __u32 timer1;
};
/* 20100413,HSL@RK,LOADER VERSION PASS TO SYSTEM BY CMDLINE.
 *  RKTAG_CHECKCODE unuse now.
 */
#if 0
/* 20100202,HSL@RK,for compatible to linux,use string instead
 * loader version X.XX YYYYMMDD
 */
#define RKTAG_VERSION                0X524B0002         /* loader versio info */
struct rktag_version {
             #if 0
             __u32 date;        /* 20091023 */
             __u16 main_version;        /* 2.6 = 0x0206 */
             __u16 min_version;         /* 27 = 0x0027 */
             #else
              __u8    version[512];
             #endif
};
#define RKTAG_CHECKCODE           0X524B0003         /* run code */
struct rktag_checkcode {
             __u32 va_start;
             __u32 ph_start;
             __u32 length;
};
#endif


#define RKTAG_BOOTFLAG             0X524B0004         /* normal boot ,recover boot,function switch,as luns... */
struct rktag_bootflag {
             __u32 flag;            /* bit0: 0normal boot , 1:recover ,bit1:0:normal , 1:from rockusb , bit2: maskrom usb  */
             __u32 function0;  /* bit32:1 rkusb disable  */  
             __u32 function1;  /* bit value?  */  
};

#define RKTAG_IDBLK_CHIPINFO             0X524B0005         /* chip info at sector 2 */
struct rktag_idblk_chipinfo {
             __u16  size;
             __u8    info[510]; /* depend on size */
};

#define RKTAG_IDBLK_SERIAL             0X524B0006         /* machine serial number at sector 3 */
struct rktag_idblk_serial {
             __u16  size;
             __u8    sn[30]; /* depend on size */
};


#define RKTAG_IDBLK_UID             0X524B0009         /* UID at sector 3 */
struct rktag_idblk_uid {
             __u8   size;
             __u8   uid[30]; /* depend on size */
};

#define RKTAG_IDBLK_MAC             0X524B000A         /* MAC at sector 3 */
struct rktag_idblk_mac {
             __u8   size;
             __u8   mac[6]; /* depend on size */
};

#define RKTAG_IDBLK_BLUETOOTH        0X524B000B         /* MAC at sector 3 */
struct rktag_idblk_bt {
             __u8   size;
             __u8   bt[6]; /* depend on size */
};



#define RKTAG_TAGINFO             0X524B0007         /* tag addr and size */
struct rktag_tag {
             __u32  save_addr;
             __u32  size; /* total size */
};
/* 
 * cmy@20100409: 
 */
#define RKTAG_PRINT_BUFFER                0X524B0008         /* loader versio info */
struct rktag_print_buffer {
    __u8    *pt;
    __u8    buffer[1024*3];
};
/*******************KERNEL TO LOADER******************************/
/*
 * FLAG kernel to load start from here .
 */
#define KRTAG_RESET_FLAG           0X4B520000     /* system reset type,loader boot type */    
struct krtag_reset_flag {
             __u16 reset_type;
             __u16 boot_type;
             __u16 xboot_type;   
             __u16 xreset_type;  /* = ^reset_type */
};
#define RESET_PANIC                     0XAEEA
#define RESET_HARDRESET             0XA55A
#define RESET_SOFTRESET              0X5A5A
#define RESET_COLD                      0XE5E5  /* unnkow*/

enum {
BOOT_NORMAL=                  0,
BOOT_LOADER,
BOOT_MASKROM,
BOOT_RECOVER,
BOOT_NORECOVER,
BOOT_WINCE, /* FOR OTHER SYSTEM */
BOOT_WIPEDATA, /* enter recover and wipe data. */
BOOT_WIPEALL,
BOOT_MAX  /* MAX VALID BOOT TYPE.*/
};
#define HARD_RESET                      2
#define PANIC_RESET                      1
#define SOFT_RESET                       0

#define TAG_BY_REG                       0
#ifdef __KERNEL__ 
#define BOOT_BASE_REGISTER      (PWM_BASE_ADDR_VA) // pwm1 reg.
#else
#define BOOT_BASE_REGISTER      (PWM_BASE_ADDR) // pwm1 reg.
#endif
#define BOOT_SET_START()             do{io_write(BOOT_BASE_REGISTER+0x1c,0);}while(0) // disable pwm.        
#define BOOT_SET_BOOT(n)             do{io_write(BOOT_BASE_REGISTER+0x14,n);}while(0) // set boot at hrc.        
#define BOOT_SET_RESET(n)           do{io_write(BOOT_BASE_REGISTER+0x18,n);}while(0) // set reset at lrc..        
#define BOOT_GET_BOOT()             io_read(BOOT_BASE_REGISTER+0x14) // set reset at lrc..        
#define XOR_SHIFT( n )         (((n)<<16)|((~(n))&0xffff))

#define KRTAG_GET_LOG                0X4B520001    /* system log infomatio */
struct log_info {
             __u32 ph_start;
             __u32 total_len;
             __u32 offset;   
             __u32 len;
};
struct krtag_get_log {
             struct log_info log[3];    /* print,android ..*/
};

#if 0
#define KRTAG_VERSION                0X4B520002    /* kernel version infomatio,sdk version */
struct krtag_version {
             struct rktag_version sdk_version;
             __u32 size;       /* version string len */
             char linux[4];
};
#define KRTAG_CHECKDATA           0X4B520003    /* kernel data for check code ,for soft sn .. */
struct rktag_checkdata {
             __u32 data0;        
             __u32 data1;        
             __u32 data2;        
             __u32 data3;        
};
#endif

struct kld_tag {
	struct rktag_header hdr;
	union {
		struct rktag_chip		chip;
		struct rktag_runtime            rt;
		//struct rktag_version            lver;
		//struct rktag_checkcode       ck;
		struct rktag_bootflag          bf;
                                struct rktag_idblk_chipinfo   id_chip;
                                struct rktag_idblk_serial       sn;
                                struct rktag_tag                     tg;
                                
		struct krtag_reset_flag	reset;
		struct krtag_get_log           log;
		//struct krtag_version           kver;
		//struct rktag_checkdata       ckdata;
		struct rktag_print_buffer     print_buffer;

		
		struct rktag_idblk_uid          uid;
		
		struct rktag_idblk_mac          mac;
		
		struct rktag_idblk_bt          bt;

	} u;
};

#ifdef __KERNEL__ 
/* 20100413,HSL@RK,do not move tag until restart. */
//#define __uninit_bss __section(.uninit.bss )
//__uninit_bss static struct rktag_header     rkld_param[512];     /* 512 * 8 = 4K */
struct rktag_header * rkh_begin ;//= (struct rktag_header*)&rkld_param[0];
struct rktag_header * rkh_end  ;//= (struct rktag_header*)&rkld_param[512];
#define RKTAG_POSITION          //(struct rktag_header *)(0xC0780000)
#else
/*
#pragma arm section [section_sort_list]

其中：

section_sort_list

    指定要用于后续函数或对象的节名称的可选列表。section_sort_list 的语法为：

    section_type[[=]"name"] [,section_type="name"]*

有效的节类型是：


      code

      rodata

      rwdata

      zidata。
*/
#pragma arm section zidata = "PARAM"
struct rktag_header *param_addr;
#pragma arm section zidata
static struct rktag_header * rkh_begin = (struct rktag_header*)0x60002000;        /* start point */
static struct rktag_header * rkh_end = (struct rktag_header*)0x60004000;           /* end point */
#endif
static struct rktag_header * rkh_current;       /* current to insert */

static struct rktag_header * kld_find_tag( __u32 tag )
{
        struct rktag_header *h = rkh_begin;
        
        /* find loader to kernel */
        while( h < rkh_current ) {
                if( h->tag ==  tag )
                        return h;
                if( h->tag == RKTAG_NONE )
                        break;
                h = NEXT_TAG( h );
        }
        return NULL;
}

/* 
 * for kernel set tag to loader or loader set tag to kernel.
 * FIXME:need to disable irq ??
 */
int kld_set_tag( struct rktag_header * h ) 
{
        struct rktag_header *h0 = kld_find_tag( h->tag );
        if( h0 ) {
                memcpy( h0 , h , h->size );
                return 0;
        }
        if( (char*)rkh_current+h->size > (char*)rkh_end )
                return -ENOMEM;
        memcpy( rkh_current , h , h->size );
        rkh_current = NEXT_TAG(rkh_current);
        return 0;
}

#ifdef __KERNEL__
/* get tag data,return data size and ptr , 0:failed */
int kld_get_tag_data( unsigned int tag , void** data )
{
        struct rktag_header *h0 = kld_find_tag( tag );
        if( h0 ) {
                *data = h0+1;
                return h0->size - sizeof(struct rktag_header);
        }
        return 0;
}
#if 0 /* log enable */
extern int kernel_getlog( char ** start  , int * offset , int* len );
extern int kernel_getlog_start_length( int* start  , int * len );
extern int android_main_getlog( char ** start  , int * offset , int* len );
extern int android_main_getlog_start_length( int* start  , int * len );
void kld_tag_getlog( void )
{
        struct kld_tag  tg;
        char    *start;
        tg.hdr.tag = KRTAG_GET_LOG;
        tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct krtag_get_log);
        kernel_getlog( &start, &tg.u.log.log[0].offset, &tg.u.log.log[0].total_len);
        kernel_getlog_start_length( &tg.u.log.log[0].offset, &tg.u.log.log[0].len);
        tg.u.log.log[0].ph_start = virt_to_phys( start );

        android_main_getlog( &start, &tg.u.log.log[1].offset, &tg.u.log.log[1].total_len);
        android_main_getlog_start_length( &tg.u.log.log[1].offset, &tg.u.log.log[1].len);
        tg.u.log.log[1].ph_start = virt_to_phys( start );

        tg.u.log.log[2].total_len = 0;
        kld_set_tag( &tg.hdr );
}
#else
#define kld_tag_getlog()
#endif

void (*rk28_arch_reset)( int mod);
extern void(*rk28_restart_mmu)(void ) ;
void rk28_soft_restart( void )
{   
        //struct rktag_header *  tg = RKTAG_POSITION;
        struct rktag_tag        *t;
        if( kld_get_tag_data(RKTAG_TAGINFO,(void**)&t) ){
                memcpy( phys_to_virt(t->save_addr) , rkh_begin , t->size );
        }
        #ifndef CONFIG_CHIP_RK2818
        *((int*)(REG_FILE_BASE_ADDR_VA+0x14)) = 0;        /* un  remap , set 0 at maskrom */
        rockchip_clk_set_arm( 24 ) ; /* for loader , maskrom , AHB 1:1--do it at rk281x_reboot */
        #else
         scu_set_clk_for_reboot( );   // MUST slow down ddr freq.
        #endif
        rk28_restart_mmu();
}

int     rk28_system_crash = 0; /* 0:normal , 0xf:real crash,>=2:debug mode */
extern void machine_restart(char * __unused);
extern  void rk28_usb( void ) ;
/* 
 * reset: 0 : normal reset 1: panic , 2: hard reset.
 * boot : 0: normal , 1: loader , 2: maskrom , 3:recovery
 * 20100514,HSL@RK,for 281x,DATA at sdram will lost after mask rom.
 * so use some register ro record.
 */
void kld_set_reset_flag( int reset , int boot )
{
        unsigned int rt;
        unsigned int bt = boot;
        switch ( reset ) {
                case 0:
                        rt = RESET_SOFTRESET;
                        break;
                case 1:
                        rt = RESET_PANIC;
                        kld_tag_getlog();
                        break;
                case 2:
                default:
                        rt = RESET_HARDRESET;
                        break;
        }
#if TAG_BY_REG        
        if( bt >= BOOT_MAX )
                bt = BOOT_NORMAL;
        BOOT_SET_START();
        rt = XOR_SHIFT(rt);
        bt = XOR_SHIFT(bt);
        BOOT_SET_BOOT(bt);
        BOOT_SET_RESET(rt);
#endif
        // 20100607,HSL@RK,always by sdram tag.
        {
        struct kld_tag  tg;
        tg.hdr.tag = KRTAG_RESET_FLAG;
        tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct krtag_reset_flag);
        tg.u.reset.reset_type = rt;
        tg.u.reset.boot_type = bt;
        tg.u.reset.xboot_type = ~tg.u.reset.boot_type;
        tg.u.reset.xreset_type = ~tg.u.reset.reset_type;
        kld_set_tag( &tg.hdr );
        }
}
#if 0
int kld_get_reboot( void )
{
        __u32 boot = BOOT_GET_BOOT();
        int     real_boot = boot>>16;
        printk("%s::boot=0x%08x,real boot=0x%04x\n" ,__func__, boot , real_boot);
        if( real_boot !=( (~boot)&0xffff ) ) {
                printk("INVALID BOOT FLAGS!\n");
                return BOOT_NORMAL;
        }
        return real_boot;
}
#endif
#include <linux/cpu.h>
#include <asm/processor.h>
#include <asm/system.h>
#include <asm/arch/system.h>

extern void setup_mm_for_reboot(char mode);
static void kld_reboot( int reset , int boot )
{
        local_irq_disable();
        kld_set_reset_flag(reset , boot );
        cpu_proc_fin();
        setup_mm_for_reboot('r');
        rk28_soft_restart();
}
/* 
 * reset: 0 : normal reset 1: panic , 2: hard reset.
 * type-->boot : 0: normal , 1: loader , 2: maskrom , 3:recovery
 *
 */

int rk28_restart( int type ) 
{
	if( type < BOOT_MAX )
		kld_reboot( 0 , type );
	else {
	        switch ( type ) {
	        case 0x10:
	                *(int*)(0xfe04c0fa) = 0xe5e6e700;
	                break;
	        default:
	                {
	                void(*deader)(void) = (void(*)(void))0xc600c400;
	                deader();
	                }
	                break;
	        }
	}
        return 0x24;
}

extern void rockchip_timer_freeze(int freeze );
//extern void adb_function_enable(int enable );
//extern int adb_enabled( void );
extern int dwc_otg_reset( void ) ;
extern void rk28_usb_force_resume( void );

void rk28_usb( void )
{
        if( get_msc_connect_flag() /*&&  adb_enabled()*/ ){
                return;
        }
        //adb_function_enable(1);
        rk28_usb_force_resume();
        dwc_otg_reset();
}

/*
 * function for disable all irq call at pc tool for bk.
*/
unsigned long __rkusb_bk_save( int off_irq )
{
        unsigned long flags = 0;
        if( off_irq )
                local_irq_save(flags);
        rockchip_timer_freeze(1);
        disable_irq_nosync( IRQ_NR_LCDC );
        disable_irq_nosync( IRQ_NR_WDT );
        disable_irq_nosync( IRQ_NR_GPIO0 );
        disable_irq_nosync( IRQ_NR_GPIO1 );
        return flags;
}
void __rkusb_bk_restore( unsigned long flags )
{
        rockchip_timer_freeze(0);
        enable_irq( IRQ_NR_LCDC );
        enable_irq( IRQ_NR_WDT );
        enable_irq( IRQ_NR_GPIO0 );
        enable_irq( IRQ_NR_GPIO1 );
        if( flags )
                local_irq_restore(flags);
}

void rk28_panic_reset(void)
{
        int at_debug = rk28_system_crash;
        printk("%s::crash=%d\n" , __func__ , rk28_system_crash);
        if( __system_crashed() ) 
                return ;
        rk28_system_crash |= (1<<16); /* 20091206,HSL@RK,must set flag ! */      
        if( at_debug >= RKDBG_LOADER ) {
                kld_reboot(1,1);
        } else if( at_debug >= RKDBG_CUSTOMER0) {
                //rk28_system_crash |= RKDBG_CUSTOMER1;
                __rkusb_bk_save( 0 );
                local_irq_enable();     /* for get log by usb */
                rk28_usb();
                while( 1 ) {
                        if( need_resched() && at_debug >= RKDBG_CUSTOMER1 ) {
                                //debug_print("schedule after panic\n");
                                schedule();
                        }
                        if((rk28_system_crash&0xffff) > 0x20 ){
                                rk28_system_crash = RKDBG_WRITE;
                                __rkusb_bk_restore( 0 );
                                return;
                        }
                }
        }
        kld_reboot(1,0);
}

static void rk28_kld_reset( int mod )
{
        printk("%s::reboot system,mod=%d\n" ,__func__ , mod );
        rk28_restart( mod );
}

#if 0
static void rk28_set_check_data( __u32 data0 , __u32 data1 ,
        __u32 data2 , __u32 data3 )
{
       struct kld_tag  tg;
        tg.hdr.tag = KRTAG_CHECKDATA;
        tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct rktag_checkdata);
       
        tg.u.ckdata.data0 = data0;
        tg.u.ckdata.data1 = data1;
        tg.u.ckdata.data2 = data2;
        tg.u.ckdata.data3 = data3;
        kld_set_tag( &tg.hdr );
}
#endif

char rk28_hex2bcd( char hex  ) 
{
        int h = (hex & 0xf);
        if(  h > 9  )
                return 'A'+(h-10);
        return '0'+h;
}
extern char android_serial_number[ ];
/* 20100413,HSL@RK,do  nothing if not tag found.
 *  20101230,HSL@RK,update serial for usb.
*/
void __init rk28_kld_init( void )
{      
        struct rktag_header * h;
        int     get_tag = 0;
        int     get_serial = 0;
        rk28_arch_reset = rk28_kld_reset ; //rk28_enable_wdt_to_restat;
        
        h = (struct rktag_header *)0xc0002000;
        rkh_begin =rkh_current = rkh_end = h;
        if( h->tag == RKTAG_IDTAG && h->size == RKTAG_IDSIZE ) {
                h->tag = KRTAG_IDTAG;
        } else /* if( h->tag != KRTAG_IDTAG || h->size != RKTAG_IDSIZE )*/ {
                //printk("%s:: found invalid tag=0x%x,size=0x%x at 0x%p\n" , __func__ , 
                //        h->tag , h->size , h );
                //rkh_current = h;
                //goto kld_set;
                return;
        }
        
        //memcpy( rkh_begin , h , sizeof rkld_param );
        //h = rkh_begin;
        if( h->tag == KRTAG_IDTAG && h->size == RKTAG_IDSIZE ) {
                h = NEXT_TAG( h );
                // && h <  rkh_end
                while( h->tag != RKTAG_NONE  ) {
                        #if 0
                        struct kld_tag  *tg = (struct kld_tag  *)h;
                        switch (h->tag) {
                        case RKTAG_CHIP:
                                {
                                struct rktag_chip * p = (struct rktag_chip*)(h+1);
                                printk("%s::get RKTAG_CHIP,size=0x%x,chipxxx=0x%x,reserved blk=%d,rkh_begin=0x%p\n" , __func__ ,
                                        h->size-8 ,p->chip_xxx,p->total_reserved_blocks,rkh_begin);
                                }
                                break;
                        case RKTAG_RUNTIME:
                                printk("%s::load timer=%d ms\n" , __func__ , tg->u.rt.total_ms );
                                break;
                        case RKTAG_VERSION:
                                printk("%s::load date=%x , version=%x.%x\n" , __func__ , tg->u.lver.date , tg->u.lver.main_version , tg->u.lver.min_version );
                                break;
                        case RKTAG_CHECKCODE:
                         //       printk("%s::code start=%x , size=%d\n" , __func__ , tg->u.ck.va_start , tg->u.ck.length );
                                break;
                        case RKTAG_BOOTFLAG:
                        //        printk("%s::boot flag=%x\n" , __func__ , tg->u.bf.flag );
                                break;
                        case RKTAG_IDBLK_CHIPINFO:
                                break;
                        case RKTAG_IDBLK_SERIAL:
                                break;
                        default:
                                printk("%s::unknow tag=0x%x\n" , __func__ , h->tag );
                                break;
                        }
                        #else
                        if( h->tag == RKTAG_TAGINFO ) {
                                struct rktag_tag        *t = (struct rktag_tag*)(h+1);
                                rkh_end = (struct rktag_header*)((char*)rkh_begin+t->size);
                                get_tag = 1;
                                //printk("tag addr=0x%x,total size=0x%x,begin=0x%p\n" , t->save_addr,t->size , rkh_begin );
                        } else if ( h->tag == RKTAG_IDBLK_SERIAL ){
                                struct rktag_idblk_serial        *t = (struct rktag_idblk_serial*)(h+1);
                                // if we have serial??
                                if( t->size && t->sn[0]  ) {
                                        memcpy( android_serial_number ,t->sn ,  t->size );
                                        printk("%s: serial=%s\n" , __func__ , t->sn );
                                        android_serial_number[t->size] = 0;
                                        get_serial = 1;
                                }
                        } else if( RKTAG_IDBLK_UID == h->tag ) {
                                 // we have uid?
                                 struct rktag_idblk_uid *t = (struct rktag_idblk_uid*)(h+1);
                                 if( t->size && t->uid[0] && !get_serial ) {
                                        int i  = 0;
                                        char *p = android_serial_number;
                                        while( i < t->size ) {
                                                *p++ = rk28_hex2bcd(t->uid[i]>>4);
                                                *p++ = rk28_hex2bcd(t->uid[i]);
                                                i++;
                                        }
                                        *p = 0;
                                        //memcpy( android_serial_number ,t->uid ,  t->size );
                                        printk("%s: uid=%s,android serila=%s\n" , __func__ , t->uid , android_serial_number);
                                }
                        } else if(RKTAG_CHIP == h->tag ) {
                                struct rktag_chip * p = (struct rktag_chip*)(h+1);
                                printk("%s::get RKTAG_CHIP,size=0x%x,sn=0x%x,chipxxx=0x%x,reserved blk=%d\n" , __func__ ,
                                        h->size-8,p->chip_sn0,p->chip_xxx,p->total_reserved_blocks);
                        }
                        //printk("%s::get tag=0x%x,size=0x%x,rkh_begin=0x%p\n" , __func__ ,h->tag,
                        //                h->size-8 , rkh_begin);
                        #endif
                        h = NEXT_TAG( h );
                }
                rkh_current = h;
                if( !get_tag ) /* loader not support RKTAG_TAGINFO*/
                        rkh_end = (struct rktag_header*)((char*)rkh_begin+0x2000);
        }
        //rk28_set_check_data((__u32)rk28_panic_reset , (__u32)RKTAG_POSITION ,
        //        0 , 0); 
        //kld_set_reset_flag( HARD_RESET , BOOT_NORMAL ); /* 0: normal , 1:loader usb */
}

#else
#define phy_addr      ((struct rktag_header *)(0x607f0000))

extern void SET_ROM_SP( void );
void loader_tag_boot_maskrom_usb( void )
{
        //RkPrintf("boot ro maksrom usb...\n" );
        SET_ROM_SP();
}

static struct rktag_header * kld_get_current( void )
{
        struct rktag_header *h = rkh_begin;
        while( h->tag != RKTAG_NONE  ) {
                if( NEXT_TAG( h ) > rkh_end )
                        break;
                h = NEXT_TAG( h );
                
        }
        return h;
}

/*
 * 20100204,HSL@RK,because loader_tag_set_chip will be call many time,
 * and will lost other info.so do only first time.
*/

void loader_tag_set_chip( __u32 chip_sn , __u32 per , __u32 flt_version )
{
        struct kld_tag  tg;
        struct rktag_header *h0 = kld_find_tag( RKTAG_CHIP );
        if( h0 )
                return ;
        tg.hdr.tag = RKTAG_CHIP;
        tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct rktag_chip );
        tg.u.chip.chip_sn0 = chip_sn; /* RK2808*/
        #if 0
        tg.u.chip.chip_xxx = 0x524b2877;        // 28xx
        if( (chip_sn & RK28xx_CHIPMASK) == RK2808_CHIPSIGN)
                tg.u.chip.chip_xxx = 0x524b2808; // 2808 
        if( (chip_sn & RK28xx_CHIPMASK) == RK2800_CHIPSIGN)
                tg.u.chip.chip_xxx = 0x524b2808;
        if( (chip_sn & RK28xx_CHIPMASK) == RK2806_CHIPSIGN)
                tg.u.chip.chip_xxx = 0x524b2806;                
        if( (chip_sn & RK28xx_CHIPMASK) == RK2806PLUS_CHIPSIGN)
                tg.u.chip.chip_xxx = 0x524b2807;   // 2807
        #else
        tg.u.chip.chip_xxx = 0x524b2800|(chip_sn&RK28xx_CHIPMASK);        // 28xx
        #endif
        tg.u.chip.peripheral = per;             /* how to use ? */
        tg.u.chip.flt_version = flt_version; /* 01.00 */
        kld_set_tag( &tg.hdr );
}

/* t0:the timer0 count , t1:the timer1 count */
void loader_tag_set_timer( __u32 t0 , __u32 t1)
{
        
        struct rktag_header *h0 = kld_find_tag( RKTAG_RUNTIME );
        if( h0 ) {
                int     apb_kclk = RKLD_APB_FREQ; // 50M 
                struct kld_tag  *tg = (struct kld_tag *)h0;
                tg->u.rt.timer01 = t0; 
                tg->u.rt.total_ms = (tg->u.rt.timer00-tg->u.rt.timer01) / (apb_kclk); 
                tg->u.rt.timer1 = t1; 
        } else {
                struct kld_tag  tg;
                tg.hdr.tag = RKTAG_RUNTIME;
                tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct rktag_runtime);
                tg.u.rt.total_ms = 0; 
                tg.u.rt.timer00 = t0;            
                tg.u.rt.timer01 = t0;
                tg.u.rt.timer1 = t1; 
                kld_set_tag( &tg.hdr );
        }
}
#if 0
/*  
 * 20100202,HSL@RK,change to string format.
*/
void loader_tag_set_version( __u32 date , __u16 maj_v , __u16 min_v )
{
        #if 0
        struct kld_tag  tg;
        tg.hdr.tag = RKTAG_VERSION;
        tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct rktag_version);
        tg.u.lver.date = date;                   /* 20091114 */
        tg.u.lver.main_version = maj_v;             /*2.6.27: 0206 */
        tg.u.lver.min_version = min_v;          /* 0027  */
        kld_set_tag( &tg.hdr );
        #else
        char    *ver;
        int       len;
        struct kld_tag  tg;
        tg.hdr.tag = RKTAG_VERSION;
        ver = (char*)&tg.u.lver;
        // loader version X.XX YYYYMMDD
        len = sprintf(ver,"loader version %x.%x %x\n" , maj_v,min_v,date )+1;
        //RkPrintf("%s",ver );
        tg.hdr.size = sizeof( struct rktag_header ) + ALIGN4(len);
        kld_set_tag( &tg.hdr );
        #endif
}

void loader_tag_set_checkcode( __u32 code_start , __u32 va_start ,  __u32 length  )
{
        struct kld_tag  tg;
        struct rktag_header *h0;
        struct rktag_checkcode *ck;
        tg.hdr.tag = RKTAG_CHECKCODE;
        tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct rktag_checkcode)+length ;
        tg.u.ck.va_start = va_start;        
        tg.u.ck.length = length;         
        kld_set_tag( &tg.hdr );
        h0 = kld_find_tag( RKTAG_CHECKCODE );
        if( !h0 ) {
                return ; /* no free space */
        }
        ck = (struct rktag_checkcode *)(h0+1);
        ck->ph_start = (__u32)(ck+1);
        memcpy((void*)ck->ph_start , (void*)code_start , length );
}
#endif

/* get tag data,return data size and ptr , 0:failed */
int kld_get_tag_data( unsigned int tag , void** data )
{
        struct rktag_header *h0 = kld_find_tag( tag );
        if( h0 ) {
                *data = h0+1;
                return h0->size - sizeof(h0);
        }
        return 0;
}

// cmy@20100409
void loader_tag_set_print_buffer( void )
{
    struct rktag_print_buffer *p;
    struct kld_tag  tg;
    tg.hdr.tag = RKTAG_PRINT_BUFFER;
    tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct rktag_print_buffer);
    kld_set_tag( &tg.hdr );
    kld_get_tag_data(RKTAG_PRINT_BUFFER, (void**)&p);
    p->pt = p->buffer;
}

void loader_tag_set_bootflag( __u32 flag , __u32 f0 , __u32 f1 )
{
        struct rktag_header *h0 = kld_find_tag( RKTAG_BOOTFLAG );
        if( h0 ) {
                struct kld_tag  *tg = (struct kld_tag *)h0;
                tg->u.bf.flag |= flag;                    
                tg->u.bf.function0 |= f0;            
                tg->u.bf.function1 |= f1;  
        } else {
                struct kld_tag  tg;
                tg.hdr.tag = RKTAG_BOOTFLAG;
                tg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct rktag_bootflag );
                tg.u.bf.flag = flag;                    
                tg.u.bf.function0 = f0;            
                tg.u.bf.function1 = f1;           
                kld_set_tag( &tg.hdr );
        }
}

void loader_tag_chipinfo_update( char* sector1 )
{
        struct rktag_header *h0 = kld_find_tag( RKTAG_CHIP );
        if( h0 ) {
                struct rktag_chip       *chip = (struct rktag_chip*)(h0+1);
                chip->total_reserved_blocks = *((__u16*)(sector1));
                chip->disk0_size_mb = *((__u16*)(sector1+2));
                chip->disk1_size_mb = *((__u16*)(sector1+4));
                memcpy( &chip->machine_id,sector1+14,4);
                sector1 += 484;
                chip->flash_sectors= *((__u32*)(sector1));
                chip->flash_at = *((__u8*)(sector1+489-484));
                chip->flash_page_sectors = *((__u8*)(sector1+492-484));
                chip->flash_ecc_bits = *((__u8*)(sector1+493-484));
                chip->flash_block_sectors = *((__u16*)(sector1+490-484));
                chip->first_ext_block = *((__u16*)(sector1+494-484));
                chip->last_ext_block = *((__u16*)(sector1+496-484));
                chip->first_reserved_block = *((__u16*)(sector1+498-484));
                chip->last_reserved_block = *((__u16*)(sector1+500-484));
        } else {
                //RkPrintf("chip info tag not founed!\n");
        }
}

void loader_tag_idblk_chipinfo( char *sector2 )
{
        
        struct kld_tag  tg;
        struct rktag_idblk_chipinfo     *id_chip = (struct rktag_idblk_chipinfo*)sector2;
        if(  id_chip->size > 510 ||id_chip->size == 0 )
                return;
        tg.hdr.tag = RKTAG_IDBLK_CHIPINFO;
        tg.hdr.size = sizeof( struct rktag_header ) + ALIGN4(id_chip->size+sizeof(id_chip->size));
        memcpy( &tg.u.id_chip , id_chip , id_chip->size+sizeof(id_chip->size) );
        kld_set_tag( &tg.hdr );
}


void loader_tag_idblk_serial( char *sector3 )
{
        struct kld_tag  tg;
        struct rktag_idblk_serial *sn = (struct rktag_idblk_serial*)sector3;
#if 0
        RkPrintf("sn->size = %d\n", sn->size);
        if(  sn->size > 30  ||sn->size == 0 )
        {
            sn->size = 30;
            memcpy(sn->sn, "123456781234567812345678123456", 30);
        }
#else
        if(  sn->size > 30  ||sn->size == 0 )
                return;
#endif
        tg.hdr.tag = RKTAG_IDBLK_SERIAL;
        tg.hdr.size = sizeof( struct rktag_header ) + ALIGN4(sn->size+sizeof(sn->size));
        memcpy( &tg.u.sn , sn , sn->size+sizeof(sn->size) );
        kld_set_tag( &tg.hdr );
}

void loader_tag_idblk_uid( char *sector3 )
{
        struct kld_tag  tg;
        struct rktag_idblk_uid *uid = (struct rktag_idblk_uid*)(sector3+467);
#if 0
        RkPrintf("uid->size = %d\n", uid->size);
        if(  uid->size > 30  ||uid->size == 0 )
        {
            uid->size = 30;
            memcpy(uid->uid, "123456781234567812345678123456", 30);
        }
#else
        if(  uid->size > 30  ||uid->size == 0 )
                return;
#endif
        tg.hdr.tag = RKTAG_IDBLK_UID;
        tg.hdr.size = sizeof( struct rktag_header ) + ALIGN4(uid->size+sizeof(uid->size));
        memcpy( &tg.u.uid , uid , uid->size+sizeof(uid->size) );
        kld_set_tag( &tg.hdr );
}

void loader_tag_idblk_mac( char *sector3 )
{
    struct kld_tag  tg;
    struct rktag_idblk_mac *mac = (struct rktag_idblk_mac*)(sector3+505);
    if(  mac->size > 6 || mac->size == 0 )
        return;

    tg.hdr.tag = RKTAG_IDBLK_MAC;
    tg.hdr.size = sizeof( struct rktag_header ) + ALIGN4(mac->size+sizeof(mac->size));
    memcpy( &tg.u.mac , mac , mac->size+sizeof(mac->size) );
    kld_set_tag( &tg.hdr );
}

void loader_tag_idblk_bt( char *sector3 )
{
    struct kld_tag  tg;
    struct rktag_idblk_bt *bt = (struct rktag_idblk_bt*)(sector3+498);
    if(  bt->size > 6 || bt->size == 0 )
        return;
    tg.hdr.tag = RKTAG_IDBLK_BLUETOOTH;
    tg.hdr.size = sizeof( struct rktag_header ) + ALIGN4(bt->size+sizeof(bt->size));
    memcpy( &tg.u.bt , bt, bt->size+sizeof(bt->size) );
    kld_set_tag( &tg.hdr );
}



void loader_tag_set_taginfo( void )
{
        struct rktag_header *h0 = kld_find_tag( RKTAG_CHIP );
        if( !h0 ) {
                struct kld_tag  ktg;
                ktg.hdr.tag = RKTAG_TAGINFO;
                ktg.hdr.size = sizeof( struct rktag_header ) + sizeof(struct rktag_tag);
                ktg.u.tg.save_addr = (__u32)phy_addr;
                ktg.u.tg.size = (char*)rkh_end-(char*)rkh_begin;
                kld_set_tag( &ktg.hdr );
        }
}
#if 0
static volatile int loader_stop = 0;
static void rk28_kld_dead( void )
{
        RkPrintf("dead for jtag......\n" );
        while( loader_stop == 0 );
}
#endif

void rk28_kld_loader_init( void )
{      
        struct rktag_header  *tg;
        int             len = (char*)rkh_end-(char*)rkh_begin;
        tg = phy_addr;
        memset(rkh_begin,0,len);        /* only one way,kernel to loader may use pwm reg.*/
        tg = rkh_begin;
        tg->tag = RKTAG_IDTAG;  /* SET OR RESET THE TAG and SIZE. */
        tg->size = RKTAG_IDSIZE;

        rkh_current = kld_get_current();
        loader_tag_set_taginfo();
}

int loader_tag_checkboot( void ) 
{
        __u32 boot = BOOT_GET_BOOT();
        int     real_boot = boot>>16;
        RkPrintf("loader_tag_checkboot ,boot=0x%08x\n" , boot );
        if( real_boot != ((~boot)&0xffff)  ) {
                return BOOT_NORMAL;
        }

        /* 20091116,HSL@RK,clear for next */
        BOOT_SET_BOOT( BOOT_NORMAL );
        if( real_boot == BOOT_MASKROM ) {
                loader_tag_set_bootflag(0x4 , 0 , 0);
                loader_tag_boot_maskrom_usb();
        } 
        return real_boot;
}
struct rktag_print_buffer* g_print_buf = NULL;

int write_print_buffer( const char *fmt, ... )
{
//    struct rktag_print_buffer* print_buf;

    int len;
    va_list ap;
    char buf[1024];
    int remain_size = 0;

    if(g_print_buf == NULL)
    {
        if( 0 == kld_get_tag_data(RKTAG_PRINT_BUFFER, (void**)&g_print_buf) )
            return 0;
    }

    remain_size = (char*)(g_print_buf+1)- (char*)g_print_buf->pt;
    if(remain_size <= 5)
        return 0;

    va_start(ap, fmt);
    len = vsprintf(buf, fmt, ap);
    va_end(ap);

    if(len > remain_size)
        len = remain_size;
        
    memcpy(g_print_buf->pt, buf, len);
    g_print_buf->pt += len;
    
    return len;
}

#endif

