/*
 *  rk_scu/rk_scu.c
 *
 * (C) Copyright hsl 2009
 *	Released under GPL v2.
 *
 * 
 * 
 *      
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
#include <linux/delay.h>

#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/spinlock_types.h>

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/kallsyms.h>
#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/bug.h>

#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#define SHOWME  "SCU"
#define S_LEVEL  S_L_WARN
#include <asm/arch/rk28_debug.h>

#include <asm/arch/rk28_scu.h>
#include "../video/display/screen/lcd_epd_ctrl.h"
MODULE_LICENSE("Dual BSD/GPL");

// 20100625,HSL@RK,change active_child by clk enable/disable.
#define  SCU_ADD_NODE( p )      do {  list_add_tail( &p->list , &p->parent->child_list ); \
                                                p->parent->total_child++;\
                                                } while( 0 )

#define  SCU_DEL_NODE( p )      do {  list_del( &p->list ); \
                                                p->parent->total_child--;\
                                                } while( 0 )                                                
#define SCU_INPUT_CLK_HZ( clk )     ( clk < SCU_CLK_MHZ )                                                
/* interface for scu_hw.c*/
void __rockchip_scu_init_hw( void );
int __rockchip_scu_setdiv_hw( struct rockchip_scu_unit * p );
int __rockchip_scu_enable_hw( ip_id  id , struct rockchip_scu_unit * p);
int __rockchip_scu_disable_hw( ip_id  id , struct rockchip_scu_unit * p);
int __rockchip_scu_set_pllclk_hw( struct rockchip_scu_unit * p  , int stage);
//int __rockchip_scu_set_armpll_hw( struct rockchip_scu_unit * p  , int stage);
int __rockchip_scu_set_48m_hw( struct rockchip_scu_unit * p  , int stage);
int __sdram_change_ahbclk( struct rockchip_scu_unit * p  , int stage );

int __init __rockchip_scu_get_ahb_clk_hw( void );
int __init __rockchip_scu_get_apb_clk_hw( void );

int rockchip_chg_parent_all(struct rockchip_scu_unit *p,
        struct rockchip_scu_unit *np );
 int __rockchip_get_unit_div_hw( struct rockchip_scu_unit *ipinfo );
 int __rockchip_scu_get_pll_clk_hw( ip_id id );

 extern	void    FlashTimingCfg(unsigned int AHBnKHz);
 #if defined(CONFIG_RTL8192CU) || defined(CONFIG_RTL8188EUS)
 extern int rtl8188_power_state(void);
 #else
 int rtl8188_power_state(void){return true;}
 #endif


/* 20100702,HSL@RK,for change flash timing. */
static int scu_change_ahbclk( ip_id ip , int input_clk , int stage )
{
        if( stage == STAGE_DOING_CHG ) {
                FlashTimingCfg( input_clk / 1000 );
        }
        return 0;
}
/* interface for rk_kobject.c */
#if SCU_KOJB_SUPPORT                
#define __rockchip_scu_kobj_put( obj )        kobject_put( obj )
int __rockchip_scu_kset_init( void );
void __rockchip_scu_kset_exit( void );
int __rockchip_scu_node_kobject_init(struct rockchip_scu_unit *p );
#else
#define __rockchip_scu_kobj_put( obj )       
#define __rockchip_scu_kset_init( )                              0
#define __rockchip_scu_kset_exit( )
#define __rockchip_scu_node_kobject_init( p )      0
#endif

#if (LCDC_PLL_SOURCE == 0x0u)
#define LCDC_PARENT_CLK     SCU_IPID_ARM
#elif (LCDC_PLL_SOURCE == 0x2u)
#define LCDC_PARENT_CLK     SCU_IPID_CODEC
#else
#define LCDC_PARENT_CLK     SCU_IPID_DSP
#endif
/* 
* return value: 
*       0: ok to finish travel .
*       1: continue to travel.
*       -1: failed to finish travel.
*/
typedef int (*travel_function )(struct rockchip_scu_unit * , unsigned long );
#define SCU_TRV_RETURN_OK                       0
#define SCU_TRV_RETURN_CONTINUE            1
#define SCU_TRV_RETURN_FAIL                    (-1)

#define SCU_ROOT_ID                             14
#define RKSCU_INIT_IPTABLE_OFN( ip , name ,propt,parent,divbit_start , \
                divbit_end ,reg_idx, ifn ,cpfn,max_clk, idx , ofn )       \
        {ip,# name, propt, parent , divbit_start ,divbit_end ,reg_idx , \
        ifn,cpfn,max_clk,LIST_HEAD_INIT(scu_ip_info_table[idx].list), \
        LIST_HEAD_INIT(scu_ip_info_table[idx].child_list) ,ofn, 1,1}

#define RKSCU_INIT_IPTABLE_ALL( ip , name ,propt,parent,divbit_start , \
                divbit_end ,reg_idx, ifn ,cpfn,max_clk, idx  )       \
                RKSCU_INIT_IPTABLE_OFN( ip , name ,propt,parent,divbit_start , \
                divbit_end ,reg_idx, ifn ,cpfn,max_clk, idx , NULL )
static struct rockchip_scu_unit       scu_ip_info_table[]  = {
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_SDMMC0, sdmmc0 ,SCU_PROPT_USEMAX,
                SCU_IPID_HCLK, 4 , 6 , 0 , NULL , NULL,0, 0 ),
        // 20100608,HSL@RK,lcdc from arm for ddr and video bug.
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_LCDC, lcdc ,0/*SCU_PROPT_USEMAX*/,
                LCDC_PARENT_CLK, 8 , 15 ,0 , NULL , rockchip_chg_parent_all,0,1),
                
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_USBPHY, usbphy ,SCU_PROPT_USEMAX,
                SCU_IPID_ARM, 4, 7, 2 , /*__rockchip_scu_set_48m_hw*/NULL , NULL,0, 2 ),
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_SENSOR, sensor ,0,
                SCU_IPID_ARM, 20, 22,0 , __rockchip_scu_set_48m_hw,rockchip_chg_parent_all,0,3),
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_SDMMC1, sdmmc1 ,SCU_PROPT_USEMAX,
                SCU_IPID_HCLK, 8, 10, 2 , NULL ,NULL,0, 4 ),
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_LSADC, lsadc , 0,
                SCU_IPID_PCLK, 8, 15, 1 , NULL ,NULL,0, 5 ),
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_DDR, ddr , SCU_PROPT_MAXFREQ|SCU_PROPT_USEMAX,
                SCU_IPID_CODEC, 30, 31, 0 , NULL ,rockchip_chg_parent_all,400*SCU_CLK_MHZ, 6 ),

        /* global clk */
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_PCLK, apb ,SCU_PROPT_PARERNT,
                SCU_IPID_HCLK, 2 , 3 , 0 , NULL , NULL,0,7 ),
                
        RKSCU_INIT_IPTABLE_OFN(SCU_IPID_HCLK, ahb ,SCU_PROPT_PARERNT,
                SCU_IPID_ARM, 0 , 1 , 0 , NULL,NULL,250*1000*1000,8 , NULL/*scu_change_ahbclk*/ ),
                
        /* MAX HCLK<=155 , MAX ARM = 155*3=465 */        
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_ARM, armclk ,SCU_PROPT_PARERNT|SCU_PROPT_MAXFREQ|SCU_PROPT_HAVEPLL,
                SCU_IPID_24M, 0 , 0 ,INVALID_REG_INDEX, __rockchip_scu_set_pllclk_hw,NULL ,750*SCU_CLK_MHZ, 9 ),

        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_DSP, dsp ,SCU_PROPT_PARERNT|SCU_PROPT_MAXFREQ|SCU_PROPT_HAVEPLL,
                SCU_IPID_24M, 0 , 0 ,INVALID_REG_INDEX , __rockchip_scu_set_pllclk_hw , NULL,600*SCU_CLK_MHZ,10 ),
                
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_CODEC, codec ,SCU_PROPT_PARERNT|SCU_PROPT_MAXFREQ|SCU_PROPT_HAVEPLL,
                SCU_IPID_24M, 0, 0, INVALID_REG_INDEX , __rockchip_scu_set_pllclk_hw ,NULL,450*SCU_CLK_MHZ, 11 ),
                
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_12M, 12m ,SCU_PROPT_PARERNT|SCU_PROPT_ROOT,
                SCU_IPID_24M, 0, 0, INVALID_REG_INDEX ,NULL,NULL,12*SCU_CLK_MHZ, 12 ),
                
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_I2S, i2s ,0,
                SCU_IPID_12M, 3, 7, 1 , NULL , rockchip_chg_parent_all,0,13), 

        /* MUST INIT cur_clk.keep at the index of scu_system_clkinfo .*/
        {SCU_IPID_24M,"24m",SCU_PROPT_PARERNT|SCU_PROPT_ROOT,SCU_IPID_MAX,
        0,0,INVALID_REG_INDEX,NULL,NULL,24*SCU_CLK_MHZ,
        LIST_HEAD_INIT(scu_ip_info_table[SCU_ROOT_ID].list), LIST_HEAD_INIT(scu_ip_info_table[SCU_ROOT_ID].child_list),
        NULL,1,1,24*1000000,0},
};
#define SCU_TABLE_LASTIP        SCU_IPID_24M

struct rockchip_scu_system
{
        struct rockchip_scu_unit        *root;
        spinlock_t                spinlock;     /* lock for this struct. spin_lock_init */ 
};
static int loop_jiffies_arm_clk_MHz=0;        
static unsigned long  bake_loops_per_jiffy;

static struct rockchip_scu_system  scu_system_clkinfo =
{
         &scu_ip_info_table[SCU_ROOT_ID],        /* 24m clk as root , not support extern clk */
};

static android_early_suspend_t ddr_suspend;

static struct rockchip_scu_unit * __rockchip_find_unit_byipid( ip_id id )
{
        struct rockchip_scu_unit       *p = &scu_ip_info_table[0];
        while(1){
                if( p->id == id )
                        return p;
                if( p->id == SCU_TABLE_LASTIP )
                        break;
                p++;
        }
        return NULL;
}

struct rockchip_scu_unit * __rockchip_find_unit_at_node( struct rockchip_scu_unit *node , ip_id id )
{
        struct rockchip_scu_unit *t;
        struct list_head *pos;
        struct rockchip_scu_unit *pt;

        list_for_each(pos, &node->child_list) {
        	t = list_entry(pos, struct rockchip_scu_unit, list);
        	if( t->id == id )
        		return t;
             if( t->propt & SCU_PROPT_PARERNT ){
                pt = __rockchip_find_unit_at_node( t , id );
                if( pt )
                        return pt;
             }
        }
        return NULL;
}

static inline struct rockchip_scu_unit *__rockchip_get_clkset_parent( struct rockchip_scu_unit * p )
{
        while( !(p->propt & (SCU_PROPT_HAVEPLL|SCU_PROPT_ROOT))  )
                p = p->parent;
        BUG_ON( p == NULL );
        return p;
}

/* two way: parent first or child first .
* return : 
*       SCU_TRV_RETURN_FAIL: failed 
*       SCU_TRV_RETURN_OK or SCU_TRV_RETURN_CONTINUE: ok.
*/
static int __rockchip_scu_travel_tree( struct rockchip_scu_unit *node ,int parent_first, travel_function tvf , unsigned long arg )
{
        struct rockchip_scu_unit *t;
        struct list_head *pos;
        int      ret;
        list_for_each(pos, &node->child_list) {
                t = list_entry(pos, struct rockchip_scu_unit, list);
                if( parent_first )  {
                     ret = tvf( t, arg ) ;
                     if( ret == SCU_TRV_RETURN_OK )
                	        return ret;
                     if( ret == SCU_TRV_RETURN_FAIL)
                	        return ret;
                     if( t->propt & SCU_PROPT_PARERNT ){
                        ret = __rockchip_scu_travel_tree( t ,parent_first , tvf , arg );
                        if( ret == SCU_TRV_RETURN_OK )
                	        return ret;
                        if( ret == SCU_TRV_RETURN_FAIL)
                	        return ret;
                     } 
                } else  {                        
                        if( t->propt & SCU_PROPT_PARERNT ){
                        ret = __rockchip_scu_travel_tree( t ,parent_first, tvf , arg );
                        if( ret == SCU_TRV_RETURN_OK )
                	        return ret;
                        if( ret == SCU_TRV_RETURN_FAIL)
                	        return ret;
                        } 
                        ret = tvf( t, arg ) ;
                        if( ret == SCU_TRV_RETURN_OK )
                	        return ret;
                        if( ret == SCU_TRV_RETURN_FAIL)
                	        return ret;
                }
        }
        return SCU_TRV_RETURN_CONTINUE; /* 递归调用下，需要返回该值*/
}

/* FIXME! may need enable p->parent , parent's parent ...? */
static int __rockchip_scu_enable_clk( struct rockchip_scu_unit * p  )
{
        if( p->propt & SCU_PROPT_DISABLE ) {
                if( p->parent  ) {
                        spin_lock( &scu_system_clkinfo.spinlock );
                        p->parent->active_child++;
                        spin_unlock( &scu_system_clkinfo.spinlock );
                        if( (p->parent->propt & SCU_PROPT_DISABLE) )
                                __rockchip_scu_enable_clk( p->parent );
                }
                S_INFO("scu enable clk %s\n" , SCU_SYSFS_NAME(p) );
                p->propt &=~ SCU_PROPT_DISABLE;
                __rockchip_scu_enable_hw( p->id , p );
        }
        return 0;
}

static int __rockchip_scu_disable_clk( struct rockchip_scu_unit * p  )
{
        if( !(p->propt & SCU_PROPT_DISABLE) ) {
                S_INFO("scu disable clk %s\n" , SCU_SYSFS_NAME(p) );
                p->propt |= SCU_PROPT_DISABLE;
                __rockchip_scu_disable_hw( p->id , p );
                if( p->parent ) {
                        int ac ;
                        spin_lock( &scu_system_clkinfo.spinlock );
                        ac = --p->parent->active_child;
                        spin_unlock( &scu_system_clkinfo.spinlock );
                        if( ac == 0 )
                                __rockchip_scu_disable_clk( p->parent );
                }
        }
        return 0;
}

/* set the real clk ,include pll , div , user callback 
* 20100622.HSL@RK,use arg as stage (before change / after change) 
*/

static int __rockchip_clk_set_unit_clk_single( struct rockchip_scu_unit *p , unsigned long arg)
{
        p->propt &= ~SCU_PROPT_CLKSET;
        /*
         * 20090721,maybe tmp_clk == cur_clk,but tmp_div != cur_div ,such as adc , fix output 1MHZ.
         */
         if( SCU_UNIT_HAVEDIV(p) &&  p->tmp_div != p->cur_div ) {
                if ( __rockchip_scu_setdiv_hw( p  ) < 0 )
                        return -SCU_TRV_RETURN_FAIL; 
                p->cur_div = p->tmp_div;
        }
        //S_INFO("set clk single:set %s clk from %d to %d\n" , p->name , p->cur_clk , p->tmp_clk );
        if( p->tmp_clk != p->cur_clk ) {
                if( p->ifn )
                        p->ifn( p , arg );
                p->cur_clk = p->tmp_clk;
        }  
        /* always call back.this time ,clk may not set!! */
        if( p->ofn )
                p->ofn( p->id , p->cur_clk , arg ); 
        return SCU_TRV_RETURN_CONTINUE;
}

/* check if we can set the new clk from parent .
*  set tmp_div , tmp_clk.
* have SCU_PROPT_FIXDIV property need do nothing.
* 20100621,HSL@RK,if the new clk < max_clk,we do nothing.
* 20100622.HSL@RK,use arg as stage (before change / after change)
*/
static int __rockchip_clk_checkforset_unit_clk_single( struct rockchip_scu_unit *p , unsigned long arg)
{
        //S_INFO("check ot change %s clk,ofn=0x%p\n" , p->name , p->ofn );
        if( !(SCU_PROPT_CLKSET & p->propt ) ) {
                p->tmp_div = p->cur_div;
                if( SCU_UNIT_HAVEDIV(p) ) {
                        if( !(SCU_PROPT_FIXDIV&p->propt) ) {
                                if ( (SCU_PROPT_USEMAX&p->propt) ||
                                ( (p->propt & SCU_PROPT_MAXFREQ)   
                                //&& p->parent->tmp_clk>p->cur_div*p->max_clk) ) {
                                && p->parent->tmp_clk/p->cur_div > p->max_clk) ) {
                                                p->tmp_div = p->parent->tmp_clk/p->max_clk ;
                                                if( p->tmp_div  * p->max_clk < p->parent->tmp_clk )
                                                        p->tmp_div += 1;
                                                if( p->tmp_div > SCU_DIV_MAXVALUE(p) ) {
                                                        S_WARN("XXX:set [%s] div=%d , max=%d\n" , p->name , p->tmp_div , SCU_DIV_MAXVALUE(p) );
                                                        p->tmp_div = SCU_DIV_MAXVALUE(p);
                                                }
                                }
                        } 
                }
        }
        /* get the tmp clk for its childs */
        p->tmp_clk =  p->parent->tmp_clk / p->tmp_div;
        // for fix div,may happen tmp_clk > max_clk ,we warn here.
        if( p->tmp_clk > p->max_clk ) {
                S_WARN("XXX:set %s clk=%d , max=%d\n" , p->name , p->tmp_clk , p->max_clk );
        }
         if( p->tmp_clk != p->cur_clk ) {
                if( p->ifn )
                        p->ifn( p , arg );
        }     
        /* always call back. */
        if( p->ofn ) {
                if( p->ofn( p->id , p->tmp_clk , arg ) )
                        return SCU_TRV_RETURN_FAIL;
        }
        return SCU_TRV_RETURN_CONTINUE;
}

/* 20100624,HSL@RK,now ,the clk have set and stable.
 *  so we use cur_clk.
*/
static int __rockchip_clk_after_setting( struct rockchip_scu_unit *p , unsigned long arg)
{
        /* always call back. */
        if( p->ofn ) {
                if( p->ofn( p->id , p->cur_clk , STAGE_AFTER_CHG ) )
                        return SCU_TRV_RETURN_FAIL;
        }
        return SCU_TRV_RETURN_CONTINUE;
}
int __rockchip_scu_set_div( struct rockchip_scu_unit *p, int new_div )  
{
        if( p->cur_div == new_div ) 
                return 0;
        /* LCDC max=200*1000000 and div = 24,new_div * p->max_clk will overflow.*/
        //if( p->parent->cur_clk> new_div * p->max_clk ) {
        if( p->parent->cur_clk/new_div > p->max_clk ) {
                S_WARN("set %s div=%d to low,parent clk=%d,max_clk=%d\n",p->name , new_div,
                        p->parent->cur_clk,p->max_clk);
                //return -EINVAL;
                new_div = p->parent->cur_clk/p->max_clk +1;
        }
        p->tmp_div = new_div;
        p->tmp_clk = p->parent->cur_clk / p->tmp_div;
        if( __rockchip_clk_set_unit_clk_single( p , STAGE_AFTER_CHG ) == SCU_TRV_RETURN_FAIL )
                return -EIO;
        return 0;
}

int __rockchip_scu_set_unit_div( ip_id id , int new_div ) 
{
        struct rockchip_scu_unit *p ;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p ) {               /* for find error , can not register again */
                S_ERR("%s::failed to find ip[%d]" ,__func__, id );
                return -EIO;
        }
        if( SCU_UNIT_HAVEDIV(p) )
                return __rockchip_scu_set_div( p , new_div );
        return -EINVAL;
}
int __rockchip_clk_set_div_clock( struct rockchip_scu_unit *p , int new_clk )    /* SCU_IPID_ARM */
{
        int div;
        div = p->parent->cur_clk / new_clk;
        /* 20100911,HSL@RK,keep real clk <= new_clk. */
        if( div *new_clk < p->parent->cur_clk ) 
            div += 1; 
        return __rockchip_scu_set_div( p , div );
}

int __rockchip_clk_set_pll_clock( struct rockchip_scu_unit *p, int new_clk )    /* SCU_IPID_ARM */
{
        struct rockchip_scu_unit *parent = p;
        int parent_first;
        int     res = 0;
        
#if SCU_SEM                 
        down_interruptible( &p->clk_sem );
#endif        
        p->tmp_clk = new_clk;
        p->propt |=  SCU_PROPT_CLKSET;
        
        //S_INFO("%s:: check for change  %s clk\n" , __func__ , p->name );
        /* we chang clk from parent */
        if( __rockchip_scu_travel_tree( parent , 1, __rockchip_clk_checkforset_unit_clk_single , STAGE_BEFORE_CHG ) 
                < 0 /*== SCU_TRV_RETURN_FAIL*/) {
                res = -EIO;
                goto clearup_clkset;
        }
        res = 0;
        /* clk decrease , parent first , clk inscrease , child first */
        parent_first = (parent->tmp_clk < parent->cur_clk);
        if( parent_first )
                __rockchip_clk_set_unit_clk_single( parent , STAGE_DOING_CHG );

        /*XXX: can not be failed */
        __rockchip_scu_travel_tree( parent ,parent_first , __rockchip_clk_set_unit_clk_single , STAGE_DOING_CHG);

        if( !parent_first )
                __rockchip_clk_set_unit_clk_single( parent , STAGE_DOING_CHG );

        __rockchip_scu_travel_tree( parent , 1, __rockchip_clk_after_setting , STAGE_AFTER_CHG );
clearup_clkset:
        while( p->propt&SCU_PROPT_CLKSET ) {
                p->propt &= ~SCU_PROPT_CLKSET;
                p = p->parent;
        }
#if SCU_SEM                         
        up( &p->clk_sem ) ;
#endif        
        return res;
}

/* XXX:20100720,HSL@RK,the new_clk may be MHZ(<1000) or HZ.*/
int __rockchip_clk_set_unit_clock( ip_id id , int new_clk )    /* SCU_IPID_ARM */
{
        struct rockchip_scu_unit *p ;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p ) {               /* for find error , can not register again */
                S_ERR("%s::failed to find ip[%d]" ,__func__, id );
                return -EIO;
        }
        if( SCU_INPUT_CLK_HZ(new_clk) )    // < 1000 , the unit is MHZ.
                new_clk *= SCU_CLK_MHZ; // MHZ to HZ.
        S_INFO("set %s to clk %d,cur clk=%d Hz\n" , p->name , new_clk , p->cur_clk );
        if( p->cur_clk == new_clk )
                return 0;
        if( p->propt & SCU_PROPT_ROOT )
                return -EINVAL;
        /* check lock & sem */
        if( atomic_read( &p->lock_cnt ) > 0 ){
                S_INFO("%s clk locked!!\n" , p->name );
                return -EACCES ; // 20100917,HSL@RK.-EINTR;
        }

        if( new_clk > p->max_clk ) {
                S_ERR("XXX:unable to set %s clk=%d , max=%d\n" , p->name , p->tmp_clk , p->max_clk );
                return -EINVAL;
        }
        if( p->propt & SCU_PROPT_HAVEPLL )
                return __rockchip_clk_set_pll_clock( p , new_clk );
        if( SCU_UNIT_HAVEDIV(p) )
                return __rockchip_clk_set_div_clock( p , new_clk );
         return -EINVAL;               
}

int rockchip_scu_disableclk( ip_id id )
{
        struct rockchip_scu_unit *p ;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        //S_INFO("scu disable clk,id= %d,node=%p" , id , p);
        if( p ) 
                __rockchip_scu_disable_clk( p );
        else
                __rockchip_scu_disable_hw( id , p);
        return 0;
}
int  rockchip_scu_enableclk( ip_id id )
{
        struct rockchip_scu_unit *p ;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        //S_INFO("scu enable clk,id= %d,node=%p" , id , p);
        if( p ) 
                __rockchip_scu_enable_clk( p );
        else
                __rockchip_scu_enable_hw( id , p );        
        return 0;
}

/* check if the clk be locked? */
int scu_clk_locked( ip_id id )
{
        struct rockchip_scu_unit *p ;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p && atomic_read( &p->lock_cnt ) > 0 ) {
            return 1;
        }
        return 0;
}

/*
 * can not lock more then one time.
 * 20100916,HSL@RK,return the original lock status for saving.
 * return : 0: unlock before the function. >0: alread locked. < 0:error.
 * 20100917,HSL@RK,use lock count for multi lock,unlock.
 * 20100921,HSL@RK, 考虑到计数会带来不匹配的问题，所以对非跟节点
 * 不使用计数，根节点才使用计数。
*/
int rockchip_clk_lock_pll( ip_id id )
{
        int lc;
        struct rockchip_scu_unit *p,*root ;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p ) {
            S_WARN("%s: not found scu id %d for lock\n" , __func__ , id );
            return -EINVAL;
        }
        root =  __rockchip_get_clkset_parent( p );
        if( p != root ) {
            if( atomic_read(&p->lock_cnt))   // already lock.
                    return 1;
            atomic_set( &p->lock_cnt , 1);
        }
        lc = atomic_inc_return( &root->lock_cnt );
        //S_WARN("lock clk %s by %s,lc=%d\n" , root->name , p->name , lc );
        return lc-1;
}
/* return original lock state:>0:lock , 0:unlock. <0 :error.*/
int rockchip_clk_unlock_pll( ip_id id )
{
        int lc;
        struct rockchip_scu_unit *p ,*root;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p ) {
            S_WARN("%s: not found scu id %d for unlock\n" , __func__ , id );
            return -EINVAL;
        }
        root =  __rockchip_get_clkset_parent( p );
        if( p != root ) {
            if( !atomic_read(&p->lock_cnt) )   // not unlock.
                    return 0;
            atomic_set( &p->lock_cnt , 0);
        }
        lc = atomic_dec_return( &root->lock_cnt ); /* if p== root? so dec p first. */
        //S_WARN("unlock clk %s by %s,lc=%d\n" , root->name , p->name , lc );
        return lc+1;
}

int scu_unlock_arm_force( void )
{
        int locks;
        struct rockchip_scu_unit *p;        
        unsigned long flags;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , SCU_IPID_ARM );
        local_irq_save(flags);
        locks = p->lock_cnt.counter;
        p->lock_cnt.counter = 0;
        local_irq_restore(flags);
        return locks;
}

void scu_lock_arm_force( int locks )
{
        struct rockchip_scu_unit *p;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , SCU_IPID_ARM );
        atomic_set( &p->lock_cnt , locks);
}

#if SCU_SEM                 
int rockchip_clk_semlock_pll( ip_id id )
{
        struct rockchip_scu_unit *p ;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p ) {
                p =  __rockchip_get_clkset_parent( p );
                down_interruptible( &p->clk_sem );
                p->lock_ipid = id; /* for bug info */
        }
        return 0;
      
}
int rockchip_clk_semunlock_pll( ip_id id )
{
        struct rockchip_scu_unit *p ;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p ) {
                p =  __rockchip_get_clkset_parent( p );
                /* 20090717,for unit unlock many times */
                if( p->lock_ipid != SCU_IPID_MAX ) {
                        p->lock_ipid = SCU_IPID_MAX;
                        up( &p->clk_sem );
                }
        }
        return 0;
        
}
#endif
int __rockchip_clk_get_uint_clk_hw( struct rockchip_scu_unit *p  )
{
        int     clk_hz,div;
        if( !p )
                return 0;
        if( p->propt & SCU_PROPT_HAVEPLL ) {
                clk_hz = __rockchip_scu_get_pll_clk_hw( p->id );
        } else if( SCU_UNIT_HAVEDIV(p) ) {
                clk_hz = __rockchip_clk_get_uint_clk_hw( p->parent );
                div = __rockchip_get_unit_div_hw( p );
                clk_hz /= div;
        } else {
                clk_hz = p->cur_clk;
        }
        //S_INFO("%s cache clk=%d,real clk=%d\n" ,p->name , p->cur_clk , clk_hz );
        if( clk_hz !=  p->cur_clk ) {
                S_WARN("%s:%s have error cache clk=%d,real clk=%d\n" , __func__ , 
                        p->name , p->cur_clk , clk_hz );
                p->cur_clk = clk_hz;
        }
        return clk_hz;
}

int __rockchip_clk_get_uint_clk( ip_id id )
{
        int     clk_hz;
        struct rockchip_scu_unit *p ; //,  *semParent;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p )
                return 0;
        clk_hz = p->cur_clk;    /* pclk will be call many times,use cache value.*/
        return clk_hz;
}
int rockchip_clk_get_ahb( void )
{
        int a = __rockchip_clk_get_uint_clk( SCU_IPID_HCLK );     
        if( !a )
                a = __rockchip_scu_get_ahb_clk_hw();
        return a;
}
int rockchip_clk_get_apb( void )
{
       int a = __rockchip_clk_get_uint_clk( SCU_IPID_PCLK );
       if( !a )
                a = __rockchip_scu_get_apb_clk_hw();
        return a;
}
int rockchip_clk_get_arm( void )
{
        int a = __rockchip_clk_get_uint_clk( SCU_IPID_ARM );
        if( !a )
                a = __rockchip_scu_get_pll_clk_hw( SCU_IPID_ARM );
        return a;
}

#if SCU_SEM                 
int rockchip_clk_get_ipsource( ip_id id )
{
        int     clk;
        struct rockchip_scu_unit *p ,  *semParent;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p )
                return 0;
        
        semParent =  __rockchip_get_clkset_parent( p );
        down_interruptible( &semParent->clk_sem );
        clk = p->parent->cur_clk ;
        up( &semParent->clk_sem );
        return clk; 
}
#endif

/* 20100904,HSL#RK,test tell me,364M is unstable when change ddr_freq frequently. */

//#define CHANGE_DDR_WHEN_SLEEP   1       // 1 : change ddr t0 140 when lcd off. 0 : change nothing.
/* 20100702,HSL@RK,hdiv and pdiv and delay:
  * pdiv & (1<<0) : pll for dsp, pdiv & (1<<1) : pll for codec.
  * hdiv != 0 : special clk, search only equl. else search closely.
  * dly !=0: we need to change vdd: 1:v1.2 ,2:1.3v.
 */
struct rockchip_pll_set         codec_pll[] = {
        // clk_hz = 24*clkf/(clkr*clkod)     clkr  clkf   clkod  hdiv  pdiv dly
        //{600*SCU_CLK_MHZ ,                   6 , 150 ,    1 ,    0,       1},        // pdiv & (1<<0) == dsp.
        {594*SCU_CLK_MHZ ,                   4 , 99 ,      1,     0 ,    1},    // HDMI.
        //{560*SCU_CLK_MHZ ,                   6 , 140 ,    1 ,    0,       1},        // pdiv & (1<<0) == dsp.
        {500*SCU_CLK_MHZ ,                   6 , 125 ,    1 ,    0,       1}, 
        {450*SCU_CLK_MHZ ,                   4 , 75 ,     1 ,     0,       1 }, 
        //{400*SCU_CLK_MHZ ,                   6 , 100 ,    1 ,    0,       1 , 1}, 
        {SCU_CLK_122880_32,                      13 , 213 ,      1 ,    1,        2}, // pll = 393230769 HZ / 32 =  12.288461
        {SCU_CLK_122880_30,                      25 , 384 ,      1 ,    1,        2}, // pll = 368640000 HZ / 30 =  12.288
        {330*SCU_CLK_MHZ ,                  4 , 55 ,      1 ,     0,        2 ,   2},
        //307.2M, 12.280M = 48k*256 = (24/5) * (64) / 1(od) / 25(div)
		    {SCU_CLK_122880,                      5 , 64 ,      1 ,    1,        2},
		    {297*SCU_CLK_MHZ ,                   8 , 99 ,      1 ,    1,        2 ,   2}, // for TV OUT.
		    //282.24M, 11.2896M = 44.1k*256 = (24/25) * ((441/3)*2) / 1(od) / 25(div)
        {SCU_CLK_112896 ,                    25 ,147*2 , 1,     1 ,       2},   // 282.24.for hdmi
        {280*SCU_CLK_MHZ ,                   6 , 70 ,      1 ,    0,        2 ,   2},
        {165*SCU_CLK_MHZ ,                  4 , 55 ,      2 ,     0,        2 ,   0},
        {136*SCU_CLK_MHZ ,                  6 , 34*2 ,     2 ,    0 ,       2},
        // pll power down.
         {24*SCU_CLK_MHZ ,                    4 , 50 ,       1 ,    0,       3}, // POWER down,by the real clk = 300M
};

int rockchip_clk_set_codecdsp( ip_id id , int new_clk )
{
        struct rockchip_pll_set *ps,*pt;
        int     r;
        int     bit = id - SCU_IPID_DSP;

        struct rockchip_scu_unit *p =  __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id);
        if( !p )
                return -EPERM;
        if( SCU_INPUT_CLK_HZ(new_clk) )    // < 1000 , the unit is MHZ.
                new_clk *= SCU_CLK_MHZ; // MHZ to HZ.
        /* found the rockchip_pll_set we want. */
        ps = pt = &codec_pll[0];
        while( 1 ){
                if( pt->clk_hz == new_clk ) {
                        ps = pt;
                        break;
                }
                if( !(pt->apb_div & (1<<bit )) || pt->ahb_div){
                        pt ++;
                        continue;
                }
                
                // we are sorted,and ps->clk_hz > pt->clk_hz.
                if((pt->clk_hz > new_clk ||
                        (new_clk-pt->clk_hz < ps->clk_hz-new_clk)) )
                        ps = pt;
                if( pt->clk_hz < new_clk  || pt ->clk_hz == 24*SCU_CLK_MHZ )
                        break;
                pt ++;
        }
        S_INFO("%s::set %s to %d Hz,cur clk=%d Hz\n" , __func__ ,p->name, ps->clk_hz , p->cur_clk);
        if( ps->clk_hz == p->cur_clk )
                return 0;
        p->pll_set = ps;
        r = __rockchip_clk_set_unit_clock(id/*SCU_IPID_CODEC*/ , ps->clk_hz);
        return r;
}

static void scu_reduce_codec_pll( void )
{
        int clk =__rockchip_clk_get_uint_clk(SCU_IPID_CODEC) ;
        if( clk >= DDR_NORMAL_CLK*SCU_CLK_MHZ ) {
            ddr_change_mode( 0 );
            clk >>= 1;
            S_INFO("%s::change ddr freq=%d HZ\n" , __func__ , clk );
            rockchip_clk_set_codecdsp( SCU_IPID_CODEC , clk );
            mdelay(10);
            clk = 140;
            rockchip_clk_set_codecdsp( SCU_IPID_CODEC , clk );
            S_INFO("%s::change ddr freq=%d MHZ\n" , __func__ , clk );
        }
        //debug_gpio_reverse(); // low-->140M.
}

static int dsp_videio_on = 0;
void scu_set_vedio_flag( int on )
{
        dsp_videio_on = on;
}

#if 0
int rk28_video_is_on( void )
{
        return dsp_videio_on;
}
#endif

static void scu_restore_codec_pll( void )
{
        int clk ;
        if( __rockchip_clk_get_uint_clk(SCU_IPID_CODEC) <= 150*SCU_CLK_MHZ ) {
                clk = DDR_NORMAL_CLK/2;
                //debug_gpio_reverse();
                rockchip_clk_set_codecdsp( SCU_IPID_CODEC , clk );
                mdelay(10);
                S_INFO("%s::change ddr freq=%d MHZ\n" , __func__ , clk );
                clk = DDR_NORMAL_CLK;
                rockchip_clk_set_codecdsp( SCU_IPID_CODEC , clk );
                //mdelay( 10 );
                S_INFO("%s::change ddr freq=%d MHZ\n" , __func__ , clk );
                ddr_change_mode( 1 );
        }
        //debug_gpio_reverse(); // low-->140M.
}

static void scu_ddr_early_suspend(android_early_suspend_t *h)
{
        //S_INFO("%s::usb=%d,video=%d\n" , __func__ , get_msc_connect_flag(),rk28_video_is_on());
        
        /* 20100708,HSL@RK,if we do usb copy,do not change ddr freq. 
          * 20100914,HSL@RK,if arm was locked,we change nothing.
          * 20101011,HSL@RK,h==NULL for scu_set_clk_for_reboot,change clk anyway.
          * .
        */		
        #if CHANGE_DDR_WHEN_SLEEP
        if( (!scu_clk_locked(SCU_IPID_ARM) /*&& !get_msc_connect_flag()*/ 
             && !rk28_video_is_on())&& (!Rk_EPD_CTRL_Get_Status()) \
			 && rtl8188_power_state() \
			 || !h  ){

                int arm_normal_clk = rockchip_clk_get_arm();
                if( arm_normal_clk > 500*SCU_CLK_MHZ ) {
                        /* change arm clk first!! */
                        S_INFO("%s:set arm clk from %dHz\n",__func__ , arm_normal_clk);
                        rockchip_clk_set_arm( arm_normal_clk>>1  );  // two step to decrease arm clk. 
                        mdelay( 5 );
                        rockchip_clk_set_arm( arm_normal_clk>>2 );  // 600 -> 150..
                        S_INFO(" to %dHz\n" , arm_normal_clk>>2 );
                }
       	        scu_reduce_codec_pll(); 
        }
        #else
         int arm_normal_clk = rockchip_clk_get_arm();
        if( arm_normal_clk > 500*SCU_CLK_MHZ ) {
                    /* change arm clk first!! */
                    S_INFO("%s:set arm clk from %dHz\n",__func__ , arm_normal_clk);
                    rockchip_clk_set_arm( arm_normal_clk>>1  );  // two step to decrease arm clk. 
                    mdelay( 5 );
                    rockchip_clk_set_arm( arm_normal_clk>>2 );  // 600 -> 150..
                    S_INFO(" to %dHz\n" , arm_normal_clk>>2 );
       }
        #endif
}

static void scu_ddr_early_resume(android_early_suspend_t *h)
{
        #if CHANGE_DDR_WHEN_SLEEP
        int arm_normal_clk;
        scu_restore_codec_pll();
        if( (arm_normal_clk = rockchip_clk_get_arm()) < 200*SCU_CLK_MHZ ){
                int locks;
                /*restore arm clk!!*/
                locks = scu_unlock_arm_force();
                rockchip_clk_set_arm( arm_normal_clk<<1  );  // two step to increasee arm clk. 
                S_INFO("%s: set arm clk\n",__func__ );
                //rockchip_clk_set_arm(600);	/*20100829@wqq 300->660 cpu dead*/                 
                rockchip_clk_set_arm( arm_normal_clk<<2 );
                S_INFO("to %dHz\n" , arm_normal_clk<<2 );
                scu_lock_arm_force( locks );
        }
        #else
        int arm_normal_clk;
        if( (arm_normal_clk = rockchip_clk_get_arm()) < 200*SCU_CLK_MHZ ){
                int locks;
                /*restore arm clk!!*/
                locks = scu_unlock_arm_force();
                rockchip_clk_set_arm( arm_normal_clk<<1  );  // two step to increasee arm clk. 
                S_INFO("%s: set arm clk\n",__func__ );
                //rockchip_clk_set_arm(600);	/*20100829@wqq 300->660 cpu dead*/                 
                rockchip_clk_set_arm( arm_normal_clk<<2 );
                S_INFO("to %dHz\n" , arm_normal_clk<<2 );
                scu_lock_arm_force( locks );
        }
        #endif
}

/* 20100622,we only need to test change freq between 660 and 540. 
 *  change CLK MUST CHANGE clkr,clkf,clkod!!!
 *  20100710,HSL@RK,dly & 1 meas special clk,not use by cpu freq and normal set.
 *  20100720,HSL@RK,high ahb freq may save power.
 * 20100806,HSL@RK,keep apb stay at 50M for CMMB(when arm at 600M).
*/
struct rockchip_pll_set         arm_pll[] = {
              // clk_hz = 24*clkf/(clkr*clkod)    clkr clkf clkod  hdiv pdiv dly (pdiv=1,2,4,no 3!!)
#if MAYCHG_VDD    
        //{660*SCU_CLK_MHZ ,                             4 , 110 , 1 ,      3 ,    4 ,  1},   //
#endif

	{624*SCU_CLK_MHZ ,							 4 , 104 , 1 ,		3 ,    4 ,	0},   // for host1.1 48m. 
	{594*SCU_CLK_MHZ ,							 4 , 99 ,  1,		 3 ,	4 ,  1},	// HDMI.
	//{480*SCU_CLK_MHZ ,							 6 , 120 ,1,	   3 ,	  4 ,  1},	  // host1.1 and audio eq.
	{312*SCU_CLK_MHZ ,							 4 , 104 ,2 ,	   2 ,	  2 ,  0},	 // 
	{156*SCU_CLK_MHZ ,							   4 , 104 ,4 , 	 1 ,	2 ,  0},   // 
	// last item,pll power down.set real clk == 300M
	{24*SCU_CLK_MHZ ,                               4 , 50 ,  1,        3 ,   2,   0 }, // POWER down
};
//extern unsigned long loops_per_jiffy;

/* choice the closely freq. */
/* 20100914,HSL@RK,we support new_clk unit Hz and MHz. */
int rockchip_clk_set_arm( int new_clk )
{
        struct rockchip_pll_set *ps,*pt;
        struct rockchip_scu_unit *ahb,*apb;
        int     r;
        struct rockchip_scu_unit *p =  __rockchip_find_unit_at_node( scu_system_clkinfo.root  , SCU_IPID_ARM );
        if( !p )
                return -EPERM;
        /* found the rockchip_pll_set we want. */
        if( SCU_INPUT_CLK_HZ(new_clk) ) /* new_clk unit is MHz?? */
            new_clk *= SCU_CLK_MHZ; // MHZ to HZ.
        ps = pt = &arm_pll[0];
        while( 1 ){
                if( pt->clk_hz == new_clk ) {
                        ps = pt;
                        break;
                }
                // special clk ,exact match.
                if( (pt->delay&1) ){
                        pt ++;
                        continue;
                }
                // we are sorted,and ps->clk_hz > pt->clk_hz.
                if( (pt->clk_hz > new_clk ||
                        (new_clk-pt->clk_hz < ps->clk_hz-new_clk) ) )
                        ps = pt;
                if(pt->clk_hz <new_clk || pt ->clk_hz == 24*SCU_CLK_MHZ )
                        break;
                pt ++;
        }
        //S_INFO("%s::set arm to %d Hz,cur clk=%d Hz\n" , __func__ , ps->clk_hz , p->cur_clk);
        if( ps->clk_hz == p->cur_clk )
                return 0;
        p->pll_set = ps;
        WARN_ON( ps->apb_div == 3 );
        
        ahb = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , SCU_IPID_HCLK);
        if( ahb ) {
                /* make tmp div not change by __rockchip_clk_checkforset_unit_clk_single*/
                ahb->propt |= SCU_PROPT_CLKSET; 
                /* mak div not change by  __rockchip_clk_set_unit_clk_single
                 *  we change ahb,apb div when change arm pll.
                 *  not change cur_div for error case (need restore.).
                */
                ahb->tmp_div /*= ahb->cur_div*/ = ps->ahb_div;
        }
        apb = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , SCU_IPID_PCLK);
        if( apb ) {
                apb->propt |= SCU_PROPT_CLKSET;
                apb->tmp_div /*= apb->cur_div*/ = ps->apb_div;
        }
        r = __rockchip_clk_set_unit_clock(SCU_IPID_ARM , ps->clk_hz );
        __rockchip_clk_get_uint_clk_hw(apb); /* for update and check cache value.*/
        //S_INFO("%s:set arm to %d(%d) Hz\n" , __func__ ,ps->clk_hz, p->cur_clk );
        // adjust loops_per_jiffy by new clk.
        if( !r && loop_jiffies_arm_clk_MHz ) {
                loops_per_jiffy = bake_loops_per_jiffy*(p->cur_clk/SCU_CLK_MHZ) /loop_jiffies_arm_clk_MHz;
                //S_INFO("%s::new lpj=%ld,new arm clk=%d\n",__func__ ,loops_per_jiffy, p->cur_clk);
        }
    #if CHANGE_DDR_WHEN_SLEEP
        /* XXX:20100708,if usb in and enter one level sleep,we do not change ddr freq.
         * when pull out the usb cable,system will 
         * enter deep sleep ( not call scu_ddr_early_suspend ,it call normal suspend).we have no normal
         * suspend callback ,so decrease ddr freq at here. 
        */
        if( ps->clk_hz == 24*SCU_CLK_MHZ ) {
                //S_WARN("%s::reduce ddr freq when set arm to 24M,new clk=%d.\n" , __func__ , new_clk);
                S_INFO("%s::reduce ddr freq when set arm to 24M,new clk=%d.\n" , __func__ , new_clk);
                scu_reduce_codec_pll( );
        }
    #endif

        return r;
}

void scu_set_clk_for_reboot( void )
{
        scu_unlock_arm_force(); /* for force change freq. */
        //scu_ddr_early_suspend( NULL );

        //把scu_ddr_early_suspend中用于降低ARM频率的操作保留，
        //而用于降低DDR频率的部分去除掉。
        //因为复位是随时随地可能有的，可能这时候DDR还在被读写，但是用户却要求复位
        //如果有降低DDR频率的操作，则需要DDR空闲，否则在DDR忙的时候变频，会导致DDR的PLL失锁，读写出错

        int arm_normal_clk = rockchip_clk_get_arm();
        if( arm_normal_clk > 500*SCU_CLK_MHZ ) {
                    /* change arm clk first!! */
                    S_INFO("%s:set arm clk from %dHz\n",__func__ , arm_normal_clk);
                    rockchip_clk_set_arm( arm_normal_clk>>1  );  // two step to decrease arm clk. 
                    mdelay( 5 );
                    rockchip_clk_set_arm( arm_normal_clk>>2 );  // 600 -> 150..
                    S_INFO(" to %dHz\n" , arm_normal_clk>>2 );
        }

        //ddr_change_mode( 1 );
        //通过这个函数将arm变到24MHz的时候，还会去变DDR的频率，因此把它去掉，移到rk28_ddr.c的rk281x_reboot函数，直接让CPU进入slow mode
        //rockchip_clk_set_arm( 24 ) ; /* for loader , maskrom , AHB 1:1--do it at rk281x_reboot */
}

int __rockchip_change_mode_ipinfo( struct rockchip_scu_unit *p , 
    unsigned char scu_mode , int parm )
{
        S_INFO("change %s to Mode %d,parm=%d\n" , SCU_SYSFS_NAME(p) , scu_mode , parm );
        if( scu_mode == SCU_MODE_DIV ){
                if( (p->propt & SCU_PROPT_FIXDIV) && p->cur_div == parm )
                        return 0;
                if(  !SCU_UNIT_HAVEDIV(p) ) { /* NO REAL DIV CTRL */
                        S_INFO("set ip[%s] div=%d ,but no div ctrl!" , SCU_SYSFS_NAME(p), parm);
                        return -EPERM;
                }
                p->propt |= SCU_PROPT_FIXDIV;
                p->tmp_div = parm;
                /* we need to calc tmp_clk here for __rockchip_clk_set_unit_clk_single */
                p->tmp_clk = p->parent->cur_clk / p->tmp_div;
        } else if ( scu_mode == SCU_MODE_FREQ){
                // 20101109,HSL,add Hz support
                if( parm < 1000 )
                        parm*=SCU_CLK_MHZ;
                p->propt |= SCU_PROPT_MAXFREQ;
                p->max_clk = parm;
                if( p->parent ) {
                        p->parent->tmp_clk = p->parent->cur_clk;
                        __rockchip_clk_checkforset_unit_clk_single( p , STAGE_AFTER_CHG );
                }
       }
        
        if ( p->tmp_div == 0 )
                p->tmp_div = 1;

        if( !(p->propt & SCU_PROPT_MAXFREQ) ) {
                if( (p->propt & SCU_PROPT_FIXDIV) ) 
                        p->max_clk = p->parent->max_clk / p->tmp_div;
                else
                        p->max_clk = p->parent->max_clk;        /* 1:1 from parent */
        } 
        
        /* need to check div valid */
        if( SCU_UNIT_HAVEDIV(p) && p->tmp_div > SCU_DIV_MAXVALUE(p) ) {
                S_ERR("set ip[%s] div=%d , max=%d" , SCU_SYSFS_NAME(p),
                        p->tmp_div , SCU_DIV_MAXVALUE(p) );
                return -EPERM; 
        }

        /* need to set tmp clk to the right value */
        /* 20100316,HSL@RK,only SCU_MODE_SETFREQ will set UNIT clk = param.
         *  other ,not change parent clk,only change div.
         */
        if( scu_mode == SCU_MODE_NONE ) {
                /* HSL@RK,20090905 ,if init clk > set max clk */
                if( p->cur_clk > p->max_clk ) {  
                        p->tmp_clk = p->max_clk;
                        __rockchip_clk_set_unit_clk_single( p , STAGE_AFTER_CHG);        
                } 
        } else /*if( scu_mode == SCU_MODE_DIV ||scu_mode == SCU_MODE_SETDIV )*/{
                __rockchip_clk_set_unit_clk_single( p , STAGE_AFTER_CHG);     
        } 
        return 0;
}

int __rockchip_scu_change_mode( ip_id id , unsigned char scu_mode , int parm )
{
        struct rockchip_scu_unit *p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p )
            return -EINVAL;
        return __rockchip_change_mode_ipinfo(p,scu_mode,parm);
}
static int __rockchip_scu_register_node(struct rockchip_scu_unit *p, unsigned char scu_mode , int parm  , change_clk fn )
{
        if( p->propt & SCU_PROPT_HAVEPLL ) {
        #if SCU_SEM                 
                sema_init( &p->clk_sem , 1 );
        #endif
                p->lock_ipid = SCU_IPID_MAX;
        } 
       if( __rockchip_scu_node_kobject_init( p ) < 0 ) {
                S_INFO("scu %s create kobject failed\n" , p->name );
                return -EPERM; 
       }
       if( p->propt & SCU_PROPT_ROOT ) {
                p->cur_clk = p->max_clk;
                SCU_ADD_NODE( p );
                goto not_need_set;
        }
        //if( fn ) {
        //        printk("%s::%s,",__func__ , p->name );
        //        __print_symbol("fn=%s\n" , (unsigned long)fn );
        //}
        spin_lock( &scu_system_clkinfo.spinlock );

        /* add tail , for fast search */
        SCU_ADD_NODE( p );
        p->ofn = fn;
        if( SCU_UNIT_HAVEDIV(p) ) {
                p->cur_div = __rockchip_get_unit_div_hw( p );  
                p->cur_clk = p->parent->cur_clk / p->cur_div;
        }else if(p->propt&SCU_PROPT_HAVEPLL) {
                p->cur_clk = __rockchip_scu_get_pll_clk_hw(p->id);
        } else {
                p->cur_clk = p->parent->cur_clk;
        }
        S_INFO("scu %s ,read cur clk=%d Hz,div=%d\n" , p->name , p->cur_clk , p->cur_div);
        if( __rockchip_change_mode_ipinfo( p , scu_mode , parm ) )
                goto set_failed;
        /* make it enable , ifn meas it can enable itself */
not_need_set:            
        if ( !p->ifn ) {
                p->propt |= SCU_PROPT_DISABLE;  // force to enable clk.
                __rockchip_scu_enable_clk( p ); /* hw enable */
        } else {
                p->parent->active_child++;      // ifn must may p be active.
        }
        __rockchip_clk_after_setting( p , 0 );
        return 0;
set_failed:        
        S_INFO("register %s failed at change node mode\n" , p->name );
        spin_unlock( &scu_system_clkinfo.spinlock );
        kobject_del(&p->kobj);
        SCU_DEL_NODE( p );
        return -EINVAL;
}
 
/* 20100903,HSL@RK,for unit at table scu_ip_info_table,we 
  * do not change ofn if fn==NULL !!
*/
int rockchip_scu_register( ip_id id , unsigned char scu_mode , int parm  , change_clk fn )
{
        struct rockchip_scu_unit *parent = NULL;
        struct rockchip_scu_unit *p ;

        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p ) {               /* for find error , can not register again */
                S_INFO("ip[%d] register again\n" , id );
                return -EPERM;
        }
        p = __rockchip_find_unit_byipid( id );
        if( !p ) {
                S_WARN("register scu failed:can not find ip[%d]\n" , id );
                return -ENODEV;
        }
        if( scu_system_clkinfo.root->id == p->id_parent )
                parent = scu_system_clkinfo.root;
        else 
                parent = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , p->id_parent );
        if( !parent ) {
                S_WARN("must register %s parent first" , p->name );
                return -EPERM;
        }
        
        /* need to check parent valid */
        if( !(parent->propt & SCU_PROPT_PARERNT) ) {
                S_WARN("can not register %s to %s" , p->name /*SCU_SYSFS_NAME(p) */, 
                        SCU_SYSFS_NAME(parent));
                return -EPERM;
        }
        
        p->parent = parent ;
        if( !fn ){      // 20100903,HSL@RK,add.
                fn = p->ofn;
        }
        return __rockchip_scu_register_node(p , scu_mode , parm , fn );
        
}

int rockchip_scu_apbunit_register( ip_id id , char* name , change_clk fn )
{
        struct rockchip_scu_unit *p ,*parent;
        int     res;
        
        if( id >= SCU_IPID_GATE_MAX )
                return -EPERM;
        
        p = kzalloc( sizeof(*p), GFP_KERNEL );
        if( !p )
                return -ENOMEM;
        S_INFO("register scu unit %s\n" , name );
        parent = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , SCU_IPID_PCLK );
        BUG_ON( !parent);      /* must have parent */
        p->parent = parent;
        p->id = id ;
        p->name = name ;
        INIT_LIST_HEAD(&p->child_list);
        INIT_LIST_HEAD(&p->list);
        p->propt = 0;
        p->cur_div = 1;
        p->max_clk = p->parent->max_clk;
        p->divreg_index = INVALID_REG_INDEX;
        res =  __rockchip_scu_register_node(p , SCU_MODE_NONE, 0 , fn );
        if( res < 0 ) {
                S_ERR("scu register %s FAILED\n" , name );
                kfree(p);
        }
        return res;
}

//  , int idmode , int parm
int __rockchip_scu_set_parent( ip_id id ,  ip_id id_parent )
{
        int ret;
        struct rockchip_scu_unit *p ;
        struct rockchip_scu_unit *nparent ;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p || !p->cpfn )
                return -EINVAL; /* not support currently */
        if( p->parent && p->parent->id == id_parent )
                return 0;
        nparent = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id_parent );
        if( !nparent )
                return -EINVAL;
        spin_lock( &scu_system_clkinfo.spinlock );
        if(p->cpfn( p , nparent ) ) {
                ret = -EIO;
                goto out_here;
        }
        ret = kobject_move( &p->kobj , &nparent->kobj );
        if( ret )
                goto out_here;
        SCU_DEL_NODE( p );
        p->parent = nparent;
        SCU_ADD_NODE(p);

        /* 20100208,HSL@RK,reset the max_clk. */
        p->max_clk = p->parent->max_clk;        /* 1:1 from parent */
        /* 20100112,HSL@RK,update new clk for the new parent.*/
        if( SCU_UNIT_HAVEDIV(p) ) {
                p->cur_div = __rockchip_get_unit_div_hw( p );  
                p->cur_clk = p->parent->cur_clk / p->cur_div;
                if( (p->propt & SCU_PROPT_FIXDIV) ) 
                        p->max_clk = p->parent->max_clk/ p->cur_div;
        }else {
                p->cur_clk = p->parent->cur_clk;
        }
        #if 0
        if( idmode != SCU_MODE_NONE )
                ret = __rockchip_change_mode_ipinfo( p , idmode , parm );
        #endif
out_here:        
        spin_unlock( &scu_system_clkinfo.spinlock );
        return ret;
}

change_clk scu_register_ofn( ip_id id , change_clk fn )
{
        struct rockchip_scu_unit *p ;
        change_clk old;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p->ofn ){
                S_WARN("scu uint %s ofn exit\n" , p->name );
        }
        old = p->ofn;
        p->ofn = fn;
        return old;
}
extern int rockchip_timer_change_pclk( ip_id ip , int input_clk , int stage );
extern int rk28_delay( int us  );
extern void rk28_wakeup_force( void ) ;
//void __tcmfunc open_dsp_power_at_tcm( void );
void open_dsp_power_at_ddr( void );
 void dsp_power_off( void );


#if 0  /* code for test change clk.need function rk28_wakeup_force at ad_button.c */
static struct timer_list change_cpu_freq_timer;
static int scu_test_way;
static void scu_cpu_freq_timer( unsigned long pdata )
{
#if 0
        int cur_clk = rockchip_clk_get_arm();
        int nclk;
        if( cur_clk != 600*SCU_CLK_MHZ ) {
                mod_timer( &change_cpu_freq_timer , jiffies + (6*HZ));
                rk28_send_wakeup_key();
                nclk = 600;
        } else {
                mod_timer( &change_cpu_freq_timer , jiffies + (4*HZ));
                nclk = 660;
        }
        rockchip_clk_set_arm( nclk );
        S_WARN("change arm freq from %d Mhz to %d Mhz\n" , cur_clk /SCU_CLK_MHZ,
                rockchip_clk_get_arm()/SCU_CLK_MHZ);
#else
        rk28_wakeup_force();
        mod_timer( &change_cpu_freq_timer , jiffies + (HZ*3));
        S_INFO("WAKEUP/SLEEP at %lld\n" , ktime_to_ns(ktime_get()) );
        #if 0
        if( scu_test_way )
                open_dsp_power_at_tcm();
        else 
                open_dsp_power_at_ddr();
        S_WARN("after DSP power on/off,way=%d\n" ,scu_test_way );
        dsp_power_off();
        #endif
        
#endif
}
static void scu_init_timer( void )
{
        init_timer( &change_cpu_freq_timer );
        change_cpu_freq_timer.function = scu_cpu_freq_timer;
        change_cpu_freq_timer.data = (unsigned long)&change_cpu_freq_timer;
        //change_cpu_freq_timer.expires = jiffies + (3*HZ);     // man test!
        change_cpu_freq_timer.expires = jiffies + (80*HZ); // auto test!!start time become long.
        add_timer( &change_cpu_freq_timer );
}
android_suspend_lock_t scu_wake_lock;
void scu_init_lock( int way )
{
        scu_wake_lock.name = "scu";
        android_init_suspend_lock(&scu_wake_lock);
        android_lock_suspend( &scu_wake_lock );
        S_INFO("%s:: init cpu freq timer\n" , __func__ );
        scu_test_way = way;
        //rockchip_clk_set_codecdsp(SCU_IPID_DSP,24); // low power for ddr.
        //rockchip_clk_lock_pll( SCU_IPID_DSP );
        
        scu_init_timer();
}
void scu_stop( void )
{       
        android_unlock_suspend( &scu_wake_lock );
        del_timer( &change_cpu_freq_timer );
}
#else 
#define scu_init_lock(way)
#endif
void rk28_scu_ver( void )
{
        printk("=======RK2818 SCU VERSION=20111204,V1.12=======\n");
}
__init
int __rockchip_scu_tree_init( void )
{

	 rk28_scu_ver();
        spin_lock_init( &scu_system_clkinfo.spinlock );      
        if( __rockchip_scu_kset_init() < 0 )
                return -ENOMEM;
        __rockchip_scu_node_kobject_init(scu_system_clkinfo.root);
        //__rockchip_scu_init_hw( ); /* 这个函数也可以放到 baord init 里面调用*/
        /* not change arm clk here .*/
        rockchip_scu_register( SCU_IPID_ARM , SCU_MODE_NONE, 0 , NULL );
        rockchip_scu_register( SCU_IPID_CODEC, SCU_MODE_NONE , 0 , NULL );
        rockchip_scu_register( SCU_IPID_DSP, SCU_MODE_NONE , 0 , NULL );
        rockchip_scu_register( SCU_IPID_12M, SCU_MODE_NONE, 0 , NULL );
        
        //rockchip_scu_register( SCU_IPID_HCLK , SCU_MODE_FREQ, SCU_MAX_AHB_CLK , NULL );
        rockchip_scu_register( SCU_IPID_HCLK , SCU_MODE_NONE, 0 , NULL );
        rockchip_scu_register( SCU_IPID_PCLK , SCU_MODE_NONE, 0 , NULL );
        
        /* 20100208,HSL@RK,set div = 25 for hdmi */
        rockchip_scu_register(SCU_IPID_I2S , SCU_MODE_DIV , 25 , NULL);
        
        // 20100705,HSL@RK,register lcdc for 281x.max freq = 27M.
        // 20100705,HSL@RK,not change ddr when lcd on,so may set fix div.
        //rockchip_scu_register(SCU_IPID_LCDC, SCU_MODE_FREQ, 27 , NULL);
        //rockchip_scu_register(SCU_IPID_LCDC, SCU_MODE_DIV, DDR_NORMAL_CLK/SCU_LCDC_CLK +1, NULL);
        #ifdef CONFIG_LCD_RK_EINK
       	 rockchip_scu_register(SCU_IPID_LCDC, SCU_MODE_FREQ, 400 , NULL);//add by hl for eink
	#else
	 	rockchip_scu_register(SCU_IPID_LCDC, SCU_MODE_FREQ, 200 , NULL);
	 #endif
        // 20100719,HSL@RK,register for test ahb clk and arm clk.
        //rockchip_scu_register( SCU_IPID_SDMMC0, SCU_MODE_DIV, 8, NULL);
        
        // register usb phy for host1.1 48M,need arm clk = 576/1/2/3/4
        rockchip_scu_register(SCU_IPID_USBPHY,SCU_MODE_FREQ,48,NULL);
        rockchip_scu_apbunit_register( SCU_IPID_TIMER , "timer" , rockchip_timer_change_pclk );        
        rockchip_scu_register( SCU_IPID_DDR, SCU_MODE_NONE , 0 , NULL );   

        rockchip_clk_set_arm( ARM_NORMAL_CLK>>2 ); 

        rockchip_clk_set_codecdsp(SCU_IPID_DSP,24 ); // low power for ddr.
        rockchip_clk_set_codecdsp( SCU_IPID_CODEC ,DDR_NORMAL_CLK/2); // two step.
        rockchip_clk_set_codecdsp( SCU_IPID_CODEC , DDR_NORMAL_CLK /*388*/ /*364*/ /*312*/);

        rockchip_clk_set_arm( ARM_NORMAL_CLK>>1 ); 
        rockchip_clk_set_arm( ARM_NORMAL_CLK );
        
#ifdef CONFIG_RK2818_HOST11
        rockchip_clk_lock_pll( SCU_IPID_ARM );        
#endif
         

#if (LCDC_PLL_SOURCE == 0x1u)
        rockchip_clk_set_codecdsp(SCU_IPID_DSP,594 );
        rockchip_clk_lock_pll( SCU_IPID_DSP );
#endif
        // 100 HZ.
        /* (lpj=1495040,armclk=600000000) */
        /* (lpj=1646592,armclk=660000000) */
        calibrate_delay();
        loop_jiffies_arm_clk_MHz = rockchip_clk_get_arm()/SCU_CLK_MHZ;
        bake_loops_per_jiffy = loops_per_jiffy;
        //rk28_delay( 3 /*793*/ /* 1345 */ );   // for test ticks and lpj.

        /* last suspend,first resume. */
        ddr_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_DISABLE_FB+20;
        ddr_suspend.suspend = scu_ddr_early_suspend;
        ddr_suspend.resume = scu_ddr_early_resume;
        android_register_early_suspend(&ddr_suspend);

        //scu_init_lock( 0 );
        /* 20100903,HSL@RK,scu initilizing before flash!! */
        scu_register_ofn( SCU_IPID_HCLK,scu_change_ahbclk);
        S_INFO("scu init finish,arm clk=%dMHZ,ddr clk=%dMHz,%s change ddr when sleep\n" , 
                loop_jiffies_arm_clk_MHz,__rockchip_clk_get_uint_clk(SCU_IPID_CODEC)/SCU_CLK_MHZ,
                CHANGE_DDR_WHEN_SLEEP?"":"not");
        return 0;
}

EXPORT_SYMBOL(ddr_pll_delay ); // ddr_pll_delay

arch_initcall(__rockchip_scu_tree_init);

/************************************************************************/
MODULE_AUTHOR("Huang SL");
MODULE_DESCRIPTION("rk28 scu manage");
MODULE_ALIAS("RK28 SCU");


