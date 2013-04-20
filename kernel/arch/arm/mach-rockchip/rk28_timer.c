/* linux/arch/arm/mach-msm/timer.c
 *
 * Copyright (C) 2007 Google, Inc.
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
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
//#include <linux/delay.h>

#include <asm/mach/time.h>

#include <asm/io.h>
#include <asm/div64.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/rk28_irqs.h>

#define SHOWME  "RKTIMER"
#define S_LEVEL  S_L_WARN
#include <asm/arch/rk28_debug.h>

typedef volatile unsigned int   io_reg;      
struct rockchip_timer_reg_hw
{
    io_reg load_count;
    io_reg current_value;
    io_reg control_reg;
    io_reg timer_eoi;
    io_reg int_status;
};
 
static struct rockchip_timer_reg_hw *rockchip_timer_regs_base = 
        (struct rockchip_timer_reg_hw *)(TIMER_BASE_ADDR_VA);

#define WDT_DISABLE()                        do { *( (io_reg*)WDT_BASE_ADDR_VA)=0; }while( 0 )

#define TIMER_MODE_USER                 (0X1<<1)        /* load load_count */
#define TIMER_MODE_FREE                 (0X0<<1)        /* load 0XFFFFFFFF */

#define RK_TIMER_ENABLE( n )                    do { rockchip_timer_regs_base[n].control_reg |= 1 ; } while(0)
#define RK_TIMER_DISABLE( n )                   do { rockchip_timer_regs_base[n].control_reg &= ~1 ; } while(0)

#define RK_TIMER_SETMODE( n  , mode )      do { \
                                                                        if( mode ) rockchip_timer_regs_base[n].control_reg |= mode ;\
                                                                        else rockchip_timer_regs_base[n].control_reg &= ~TIMER_MODE_USER ;} while(0)

#define RK_TIMER_SETCOUNT( n  , count )    do { rockchip_timer_regs_base[n].load_count = count ; } while(0)     
#define RK_TIMER_GETCOUNT( n  )    (rockchip_timer_regs_base[n].load_count )     

#define RK_TIMER_READVALUE( n  )        ( rockchip_timer_regs_base[n].current_value )
#define RK_TIMER_INT_CLEAR( n  )         readl( &rockchip_timer_regs_base[n].timer_eoi )
#define RK_TIMER_INTS_READ( n  )        ( rockchip_timer_regs_base[n].int_status )
#define RK_TIMER_INT_DISABLE( n  )       do { rockchip_timer_regs_base[n].control_reg |= (0x1<<2) ; } while(0)
#define RK_TIMER_INT_ENABLE( n  )     do { rockchip_timer_regs_base[n].control_reg &= ~(0x1<<2) ; } while(0)
#define RK_TIMER_SETMODE_USER( n  )     RK_TIMER_SETMODE( n  , TIMER_MODE_USER )

#define RK_TIMER_GETMODE( n  )     ( rockchip_timer_regs_base[n].control_reg)
#define RK_TIMER_ENABLED( n )       (rockchip_timer_regs_base[n].control_reg&1 )

/*
 * 20100104,HSL@RK,for more free req,set MIN PCLK=10K(LIKE 0.25M).
 */
#define TIMER_MIN_PCLK                  (25*10000U)     /*how many of one second(HZ), ts 250k = 4us */        
#define TIME_MS2COUNT                   (TIMER_MIN_PCLK/MSEC_PER_SEC)

#define CHECK_VBUS_MS                   (1000)     /* ms */
#define RK_CHECK_VBUS_COUNT      (CHECK_VBUS_MS*TIME_MS2COUNT)     /* ms */
#define TIMER_CLKEVT_CYCLE        0XFFFFFFFFU

#if !defined(CONFIG_CHIP_RK2818)
#error "do you want to build rk2808 product?"
#endif
#define TIMER_DBG               0
#if defined(CONFIG_CHIP_RK2818) && !TIMER_DBG 
#define CHIP_RK281X             1       
#else
#define CHIP_RK281X             0
#endif

/*
*  timer0 for clock event to gen timer interrupt.
*  timer1 for clock source to read the current nano time, about interrupt every 57s.
*  (0xffffffff/(75*1000000)) ---MAX APB CLK = 75M.
* the min unit is RK_TIMER_MIN_PCLK.
* 20100623,HSL@RK,use timer2--24M input as clksource.
*/
#define TIMER_CLKEVT                     0      /* system jiffies timer */
#if CHIP_RK281X
#define TIMER_SCR_CLK                   (24*1000*1000)
#define TIMER_CLKSRC                     2     /* monotic timer,24M input */
#define TIMER_CLKUSB                     1      /* usb detect timer , at system suspend status */
#else
#define TIMER_SCR_CLK                   TIMER_MIN_PCLK
#define TIMER_CLKSRC                     1     /* monotic timer,apb input. */
#define TIMER_CLKUSB                     2      /* usb detect timer , at system suspend status */
#endif

// we can debug timer inverse.
#if !TIMER_DBG 
        #if CHIP_RK281X
        #define TIMER_SCR_MASK                0XFFFFFFFF        // 32bit reg max value.
        #define TIMER_SET_ROUNDUS()        RK_TIMER_SETCOUNT(TIMER_CLKSRC,0xFFFFFFFF)
        #else
        #define TIMER_SCR_MASK               0XFFFFF // (apb=75M,rockchip_apb_clk=300,0xffffffff/300=0xda740d,so max=0xfffff )
        #define TIMER_SET_ROUNDUS()        RK_TIMER_SETCOUNT(TIMER_CLKSRC,TIMER_SCR_MASK*rockchip_apb_clk)
        #endif

        
#else   // debug mode
        #define TIMER_SCR_MASK               0XFFFFF // 55*1000*1000,4s.
        /* max:APB 55M(rockchip_apb_clk=220),min:4M. so max mask = 0XFFFFFFFF/220=0X129E412
          * we set MAX mask = 0xFFFFFF.
        */
        #define TIMER_SET_ROUNDUS()        RK_TIMER_SETCOUNT(TIMER_CLKSRC,TIMER_SCR_MASK*rockchip_apb_clk)
#endif

unsigned int      rockchip_apb_clk; /* value of apb clk , unit = TIMER_MIN_PCLK HZ */
static unsigned long  clk_source_suspend_ms ;  // MS.
static unsigned long  clk_source_suspend_second ;  // MS.

static spinlock_t rockchip_timer_spinlock;      //spin_lock_init( &scu_system_clkinfo.spinlock );   

struct rockchip_clock {
	struct clock_event_device   clockevent;
	struct clocksource          clocksource;
                struct irqaction            irq_event;
	struct irqaction            irq_source;
	struct irqaction            irq_user;
             
};
static struct rockchip_clock rockchip_clocks;
static int clk_source_inited;
/*
 *  XXX:may be call at interrupt.
 * 20091116,HSL@RK,bug:if TIMER_CLKSRC down to 0 and set to max, and irq was disabled
 * 
 */
cycle_t rockchip_timer_read(void)
{
#if CHIP_RK281X
        unsigned int t = RK_TIMER_READVALUE(TIMER_CLKSRC);
        return ~t;
#else
        unsigned int t = RK_TIMER_READVALUE(TIMER_CLKSRC);
        unsigned int c = RK_TIMER_GETCOUNT(TIMER_CLKSRC);
        static unsigned int last_cycle; // for debug cycle reverse.
        t = (c-t);
        last_cycle = t;
        return t/rockchip_apb_clk;
#endif                
}

#if !CHIP_RK281X
irqreturn_t clock_source_interrupt(int irq, void *dev_id)
{
        //int int_pending ;
        //rockchip_timer_read();
        //int_pending = RK_TIMER_INTS_READ(TIMER_CLKSRC);
        RK_TIMER_INT_CLEAR(TIMER_CLKSRC);
        //S_INFO("%s::count=0x%x,int=%d\n" , __func__ , RK_TIMER_GETCOUNT(TIMER_CLKSRC),int_pending );
        return IRQ_HANDLED;
}
#endif
/* system need !, unit is second. 
*/
unsigned long read_persistent_clock(void)
{
        return (clk_source_suspend_second); // ms to second.  
}

/* 20091128,HSL@RK,system need !, returns current time in nanosec units */
unsigned long long sched_clock(void)
{
       if( clk_source_inited /*RK_TIMER_ENABLED( TIMER_CLKSRC )*/ ) {
                ktime_t now = ktime_get();
                return ktime_to_ns(now);
       } else  // XXX:if not else ,will stop at itcm init.
                return 0 ; // (unsigned long long)jiffies * (NSEC_PER_SEC / HZ);
}

unsigned long printk_clock( void )
{
        if( clk_source_inited ) {
                struct timespec ts;
                #if 0
                getnstimeofday( &ts );
                getboottime(&ts );
                #else
                ktime_get_ts( &ts );
                #endif
                return ts.tv_sec+clk_source_suspend_second;
                
        }
        return 0;
}

/* for calibrate_delay_direct. */
int read_current_timer(unsigned long *timer_val)
{
        cycle_t  now;
        if( !clk_source_inited ) {
                *timer_val = 0;
                return -1;
        }
        now = rockchip_timer_read();
        *timer_val = now &0xffffffff;
        return 0;
}
int rockchip_timer_set_next_event(unsigned long cycles,
				    struct clock_event_device *evt)
{
        RK_TIMER_DISABLE(TIMER_CLKEVT);
        RK_TIMER_SETCOUNT(TIMER_CLKEVT,cycles*rockchip_apb_clk);
        RK_TIMER_ENABLE(TIMER_CLKEVT);
        return 0;       /* 0: OK */
}

irqreturn_t rockchip_timer_clockevent_interrupt(int irq, void *dev_id)
{
        struct clock_event_device *evt = dev_id;
        //debug_gpio_reverse();
        RK_TIMER_INT_CLEAR(TIMER_CLKEVT);
        if( evt->mode != CLOCK_EVT_MODE_PERIODIC )
                RK_TIMER_DISABLE(TIMER_CLKEVT);
        evt->event_handler(evt);
        return IRQ_HANDLED;
}

int (*g_user_callback)(void);
irqreturn_t rockchip_timer_clockuser_interrupt(int irq, void *dev_id)
{
    RK_TIMER_INT_CLEAR(TIMER_CLKUSB);
    RK_TIMER_DISABLE(TIMER_CLKUSB);
    if(g_user_callback)   g_user_callback();
    return IRQ_HANDLED;
}
int rockchip_usertimer_start(unsigned long usecs, int (*callback)(void))
{
    g_user_callback = callback;
    RK_TIMER_DISABLE(TIMER_CLKUSB);
    RK_TIMER_SETCOUNT(TIMER_CLKUSB, usecs*rockchip_apb_clk);
    RK_TIMER_ENABLE(TIMER_CLKUSB);
	return 0;
}


int rockchip_timer_change_pclk( ip_id ip , int input_clk , int stage )
{
        int clk;
        //unsigned int cur_cyl0;
        //S_INFO("%s::new clk=%d,old rockchip_apb_clk=%d\n" ,__func__,input_clk, rockchip_apb_clk);
        #ifdef CONFIG_CHIP_RK2818
        if( stage != STAGE_AFTER_CHG )
                return 0;
        #endif
        clk = input_clk/TIMER_MIN_PCLK;
        if( clk == rockchip_apb_clk )
                return 0;
         spin_lock( &rockchip_timer_spinlock );
        rockchip_apb_clk = clk ;
        spin_unlock( &rockchip_timer_spinlock );

        TIMER_SET_ROUNDUS();
        S_INFO("%s::apb clk=%d,rockchip_apb_clk=%d\n" ,__func__ , 
                input_clk , rockchip_apb_clk);
        return 0;
}

static void rktimer_update_suspend_ms( unsigned int ms )
{
        clk_source_suspend_ms += ms;
        while( clk_source_suspend_ms >= 1000 ){
                clk_source_suspend_second++;
                clk_source_suspend_ms-= 1000;
        }
}
/*
 * 20091120,HSL@RK,disable irq for enable timer and set count . 
 *
 */
void rockchip_timer_clocksource_suspend_resume(int suspend )
{
	unsigned long flags;
	local_irq_save( flags );
	
	RK_TIMER_DISABLE(TIMER_CLKEVT);
#if 0
	if( suspend ) {
		RK_TIMER_SETCOUNT(TIMER_CLKEVT,RK_CHECK_VBUS_COUNT*rockchip_apb_clk); 
		RK_TIMER_ENABLE(TIMER_CLKEVT);
	} else {
		int cyl = RK_TIMER_GETCOUNT(TIMER_CLKEVT) -
		RK_TIMER_READVALUE(TIMER_CLKEVT);
		rktimer_update_suspend_ms( CHECK_VBUS_MS*cyl / RK_TIMER_GETCOUNT(TIMER_CLKEVT) );
	}
	S_INFO("r/s timer,load cnt=0x%x,clk_source_suspend=%ld\n" , 
	RK_TIMER_GETCOUNT(TIMER_CLKEVT), clk_source_suspend_second);
#endif
	local_irq_restore(flags);
}

static unsigned int clock_event_freeze_count;
static int clock_source_freezed;
void rockchip_timer_freeze(int freeze )
{
        unsigned long flags;
        local_irq_save( flags );
        if( freeze ) {
                clock_event_freeze_count = RK_TIMER_READVALUE(TIMER_CLKSRC);
                 RK_TIMER_DISABLE(TIMER_CLKSRC);
                 RK_TIMER_DISABLE(TIMER_CLKEVT);
        } else {
                RK_TIMER_SETCOUNT(TIMER_CLKSRC,clock_event_freeze_count);
                RK_TIMER_ENABLE(TIMER_CLKSRC);
                RK_TIMER_ENABLE(TIMER_CLKEVT);
        }
        clock_source_freezed = freeze;
        local_irq_restore(flags);
}

/* 
 * return 1:need to quit suspend , 0 :reenter suspend 
 * 20091010,由于拔掉USB会产生我们不希望的中断，因此USB只能通过
 * VBUS的变化来判断，不能通过中断判断.
 * 关于VBUS的判断，统一放到 dwc_otg_pcd.c 文件里面.
 */
int rockchip_timer_clocksource_irq_checkandclear( void )
{
        unsigned int  t0,t;
        volatile unsigned int *intc_reg = (volatile unsigned int *)(INTC_BASE_ADDR_VA);      // +0x28
#if 0
        printk("intc regs0:\n0x%08x 0x%08x 0x%08x 0x%08x\n"
                "0x%08x 0x%08x 0x%08x 0x%08x\n"
                "0x%08x 0x%08x 0x%08x 0x%08x\n"
                "0x%08x 0x%08x\n"
                ,
                intc_reg[0],intc_reg[1],intc_reg[2],intc_reg[3],
                intc_reg[4],intc_reg[5],intc_reg[6],intc_reg[7],
                intc_reg[8],intc_reg[9],intc_reg[10],intc_reg[11],
                intc_reg[12],intc_reg[13]
                );
        t0 = intc_reg[IRQ_REG_MASKSTATUS_L/4];
        t = intc_reg[IRQ_REG_MASKSTATUS_H/4];

        //debug_gpio_reverse();
        printk("irq0=0x%x!,irq1=0x%x(%d)!\n" ,
                 t0 , t , (t>>IRQ_NR_TIMER2)&1 );
        if( t0 == 0 && t == 0 ) {
                intc_reg[IRQ_REG_INTEN_L/4] = (1<<IRQ_NR_GPIO1);
                intc_reg[IRQ_REG_INTEN_H/4] = 0;
         }
        printk("intc regs1:\n0x%08x 0x%08x 0x%08x 0x%08x\n"
                "0x%08x 0x%08x 0x%08x 0x%08x\n"
                "0x%08x 0x%08x 0x%08x 0x%08x\n"
                "0x%08x 0x%08x\n"
                ,
                intc_reg[0],intc_reg[1],intc_reg[2],intc_reg[3],
                intc_reg[4],intc_reg[5],intc_reg[6],intc_reg[7],
                intc_reg[8],intc_reg[9],intc_reg[10],intc_reg[11],
                intc_reg[12],intc_reg[13]
                );
#else
        t0 = intc_reg[IRQ_REG_MASKSTATUS_L/4];
        t = intc_reg[IRQ_REG_MASKSTATUS_H/4];
#endif
        S_INFO("wake up,t0=0x%x,t1=0x%x\n" , t0,t);
        /* clock source irq */
        if( t0&(1<<(IRQ_NR_TIMER1+TIMER_CLKEVT)) ) {
                RK_TIMER_INT_CLEAR(TIMER_CLKEVT);
                rktimer_update_suspend_ms(CHECK_VBUS_MS);
                t0 &= ~(1<<(IRQ_NR_TIMER1+TIMER_CLKEVT)); // remove for test deep sleep and wakeup.
        }

        /* 20091103,HSL@RK,all irq must be handle !*/
        t0 |= (t&0xffff);
        return t0; 
}
void rockchip_timer_set_mode(enum clock_event_mode mode,
			      struct clock_event_device *evt)
{
//	struct rockchip_clock *clock = container_of(evt, struct rockchip_clock, clockevent);
	switch (mode) {
	case CLOCK_EVT_MODE_RESUME:
	          RK_TIMER_ENABLE(TIMER_CLKEVT);
                          break;
	case CLOCK_EVT_MODE_PERIODIC:
                          
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	           //RK_TIMER_DISABLE(TIMER_CLKEVT);
                        //RK_TIMER_INT_ENABLE( TIMER_CLKEVT );
                        //RK_TIMER_SETMODE_USER( TIMER_CLKEVT );
                        //RK_TIMER_ENABLE(TIMER_CLKEVT);
		break;
	case CLOCK_EVT_MODE_UNUSED:
                         break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		RK_TIMER_DISABLE(TIMER_CLKEVT);
		break;
	}
}


void __init rockchip_timer_clock_source_init( int apb_clk )
{
        int             v =  apb_clk / TIMER_MIN_PCLK;

        /* 20101008,HSL,do not warn for 2818.*/
        //WARN_ON(v * TIMER_MIN_PCLK != apb_clk);  /* have bug */

        S_INFO("%s::apb=%d,timer apb=%d\n",__func__,apb_clk,v);
        spin_lock( &rockchip_timer_spinlock );
        rockchip_apb_clk = v ;
        
        RK_TIMER_DISABLE(TIMER_CLKSRC);
        TIMER_SET_ROUNDUS();
        #if !CHIP_RK281X
        RK_TIMER_INT_ENABLE( TIMER_CLKSRC );
        #else
        RK_TIMER_INT_DISABLE( TIMER_CLKSRC );
        #endif
        RK_TIMER_SETMODE_USER( TIMER_CLKSRC );

        RK_TIMER_DISABLE(TIMER_CLKEVT);
        RK_TIMER_INT_ENABLE( TIMER_CLKEVT );
        RK_TIMER_SETMODE_USER( TIMER_CLKEVT );

        spin_unlock( &rockchip_timer_spinlock );
}

#if CHIP_RK281X 
/* 20100706,HSL@RK,for wifi lib use!! */
extern void __udelay(unsigned long usecs);
void udelay( unsigned long usecs )
{
        return __udelay( usecs );
}
#else
void udelay( unsigned long usecs )
{
        uint64_t later ;
        uint64_t begin,offset;
        if( clock_source_freezed )
                return __udelay( usecs );
                        
        begin = rockchip_timer_read();
        
        later = (rockchip_apb_clk*TIMER_MIN_PCLK);
        later *= ((uint64_t)usecs);
        do_div(later,USEC_PER_SEC);
        if( later > TIMER_SCR_MASK ){
                printk("%s::too much long delay=%ld us\n",__func__ , usecs );
                return __udelay( usecs );
        }
        while( 1 ) {
                nop();
                offset =  (rockchip_timer_read( )-begin) & TIMER_SCR_MASK ; 
                if( offset >= later) {
                        break;
                }
        }
}
#endif
EXPORT_SYMBOL(udelay);

#if 1  /* test delay work or not. */
/* result:
   [0] Calibrating delay loop... 299.00 BogoMIPS (lpj=1495040,armclk=600000000)
   [0] clock source mult=0x29ab , shift=8
   
   [0] one jiffies delay,s=0x190c1f4,e=0x1946cc5,ns=10014354( 10ms , 100HZ )
   [0] delay us=1234,s=0x195437d,e=0x195b701,ns=1232205

   [0] one jiffies delay,s=0x190c39a,e=0x1946e58,ns=10013562
   [0] delay us=584,s=0x19544ec,e=0x1957bb6,ns=584434
   delay us=11,s=0x171b8c0,e=0x171b9fb,ns=13125
   
   ddr_pll_delay:
   arm clk(600,000,000hz),s=0x1199726,e=0xf6d124d,ns=10014981250
   arm clk(60,000,000hz),s=0x119bcfa,e=0x2887f9c,ns=1001531380
   arm clk(40,000,000hz),s=0x119b766,e=0x20e3b5b,ns=667711073
   arm clk(6,000,000hz),s=0x1734eec,e=0x197fd0f,ns=100164588
 */
int rk28_delay( int us  )
{
        struct rockchip_clock *clock = &rockchip_clocks;
        struct clocksource *cs = &clock->clocksource;
        cycle_t         s,e;
        unsigned long ticks;
        int     clk = rockchip_clk_get_arm();
        printk("clock source mult=0x%x , shift=%d\n" , 
                cs->mult , cs->shift );
        
        ticks = jiffies;
        while( jiffies < ticks+1 );
        s = cs->read();
        ticks = jiffies;
        while( jiffies < ticks+1 );
        e = cs->read();
        printk("one jiffies delay,s=0x%Lx,e=0x%Lx,ns=%Ld\n" , s , e , cyc2ns(cs , e-s ) );
        s = cs->read();
        udelay( us );
        e = cs->read();
        printk("delay us=%d,s=0x%Lx,e=0x%Lx,ns=%Ld\n" , us ,
                s , e , cyc2ns(cs , e-s ) );
        // for set ddr_pll_delay
        clk /= 100;
        s = cs->read();
        ddr_pll_delay( clk );
        e = cs->read();
        printk("ddr_pll_delay arm clk(%dhz),s=0x%Lx,e=0x%Lx,ns=%Ld\n" , clk ,
                s , e , cyc2ns(cs , e-s ) );
        return 0x20;
}
#endif

/* code for test apb clk 
 * apb clk = 示波器读数 nHz * 2 * (150*1000) HZ.
 * if nHz == 250 , the apb clk = 250*2*150*1000 = 75 000 000 = 75M 
 */
#if 0
irqreturn_t rockchip_timer3_interrupt(int irq, void *dev_id)
{
        debug_gpio_reverse();
        RK_TIMER_INT_CLEAR(TIMER_CLKUSB);
        return IRQ_HANDLED;
}

struct irqaction timer3_irqa = {
                .name    = "timer3",
                .flags   = 0 ,
                .handler = rockchip_timer3_interrupt,
                .dev_id  = NULL,
                .irq     = IRQ_NR_TIMER3
};
void rochchip_init_timer3( void )
{
        if( setup_irq(IRQ_NR_TIMER3,&timer3_irqa) ) {
                printk("request irq timer3 failed\n");
                BUG();
        }
        RK_TIMER_DISABLE(TIMER_CLKUSB);
        RK_TIMER_SETCOUNT(TIMER_CLKUSB, 150*1000 );
        RK_TIMER_INT_ENABLE( TIMER_CLKUSB );
        RK_TIMER_SETMODE_USER( TIMER_CLKUSB );
        RK_TIMER_ENABLE(TIMER_CLKUSB);
}
#else
#define rochchip_init_timer3()
#endif


static struct rockchip_clock rockchip_clocks = {
		.clockevent = {
			.name           = "timer0",
			.features       = CLOCK_EVT_FEAT_ONESHOT,
			.shift          = 32,
			.rating         = 200,
			.set_next_event = rockchip_timer_set_next_event,
			.set_mode       = rockchip_timer_set_mode,
		},
		.clocksource = {
			.name           = "timer2",
			.rating         = 300,
			.read           = rockchip_timer_read,
			.mask           = TIMER_SCR_MASK,
			.shift          = 8,
			.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
		},
		.irq_event = {
			.name    = "timer0",
			.flags   = IRQF_DISABLED | IRQF_TIMER ,
			.handler = rockchip_timer_clockevent_interrupt,
			.dev_id  = &rockchip_clocks.clockevent,
			.irq     = IRQ_NR_TIMER1+TIMER_CLKEVT
		},
		#if !CHIP_RK281X
		.irq_source = {
			.name    = "timer2",
			.flags   = IRQF_DISABLED ,
			.dev_id  = &rockchip_clocks.clocksource,
			.handler = clock_source_interrupt,
			.irq = IRQ_NR_TIMER1+TIMER_CLKSRC
		},
		#endif
		.irq_user = {
		    .name    = "timer1",
			.flags   = IRQF_DISABLED ,
			.handler = rockchip_timer_clockuser_interrupt,
			.dev_id  = &rockchip_clocks.irq_user,
			.irq     = IRQ_NR_TIMER1+TIMER_CLKUSB
		}
};
static void __init rockchip_timer_init(void)
{
	int res;
	
	struct rockchip_clock *clock = &rockchip_clocks;
	struct clock_event_device *ce = &clock->clockevent;
	struct clocksource *cs = &clock->clocksource;

              /* 
               * init at  machine_rk28_mapio for udelay early use.
               * must after __rockchip_scu_init_hw to use rockchip_clk_get_apb.
               */
             //rockchip_timer_clock_source_init(  rockchip_clk_get_apb() ); 
             
                cs->mult = clocksource_hz2mult(TIMER_SCR_CLK , cs->shift);
	res = clocksource_register(cs);
	if (res)
		BUG();  /* have bug */
                RK_TIMER_ENABLE(TIMER_CLKSRC);  // start clk source.
                clk_source_inited = 1;
                
	ce->mult = div_sc( TIMER_MIN_PCLK , NSEC_PER_SEC, ce->shift);
	ce->max_delta_ns =  
		clockevent_delta2ns( TIMER_CLKEVT_CYCLE , ce);
                #ifdef CONFIG_HIGH_RES_TIMERS
                ce->min_delta_ns = NSEC_PER_SEC/HZ;
                #else
	ce->min_delta_ns = clockevent_delta2ns(2, ce);
                #endif
                
	ce->cpumask = cpumask_of_cpu(0);
                clockevents_register_device(ce);

            #if !CHIP_RK281X /* 20100623,NO NEED to use irq(debug).*/
                res = setup_irq(clock->irq_source.irq, &clock->irq_source);
	if (res)
		BUG();  /* have bug */
            #endif
                rochchip_init_timer3();
                
                res = setup_irq(clock->irq_event.irq, &clock->irq_event);
	if (res)
		BUG();  /* have bug */
                res = setup_irq(clock->irq_user.irq, &clock->irq_user);
	if (res)
		BUG();  /* have bug */


}


struct sys_timer rockchip_timer = {
	.init = rockchip_timer_init
};
