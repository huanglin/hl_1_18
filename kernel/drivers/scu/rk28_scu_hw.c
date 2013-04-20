/*
 *  scu/rk_scu_hw.c
 *
 * (C) Copyright hsl 2009
 *	Released under GPL v2.
 *
 * 
 * log:
 *      basic scu register function.
 *      20090518,change uniform to set io register.
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

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <asm/atomic.h>

#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/tcm.h>
#include <asm/cacheflush.h>

#include <asm/arch/rk28_scu.h>
/* 20100615,HSL@RK,FOR pwm3 change vdd core.*/
#include <asm/io.h>
#include <asm/arch/rk28_backlight.h>

#define SHOWME  "SCUHW"
#define S_LEVEL  S_L_WARN
#include <asm/arch/rk28_debug.h>
#include <asm/arch/rk28_dma.h>

/*SCU PLL CON*/
#define PLL_TEST        (0x01u<<25)
#define PLL_SAT         (0x01u<<24)
#define PLL_FAST        (0x01u<<23)
#define PLL_PD          (0x01u<<22)
#define PLL_CLKR(i)     (((i)&0x3f)<<16)
#define PLL_CLKF(i)     (((i)&0x0fff)<<4)
#define PLL_CLKOD(i)    (((i)&0x07)<<1)
#define PLL_BYPASS      (0X01)

/*SCU MODE CON*/
#define SCU_INT_CLR         (0x01u<<8)
#define SCU_WAKEUP_POS      (0x00u<<7)
#define SCU_WAKEUP_NEG      (0x01u<<7)
#define SCU_ALARM_WAKEUP_DIS (0x01u<<6)
#define SCU_EXT_WAKEUP_DIS   (0x01u<<5)
#define SCU_STOPMODE_EN     (0x01u<<4)

#define SCU_CPUMODE_MASK    (0x03u<<2)
#define SCU_CPUMODE_SLOW    (0x00u<<2)
#define SCU_CPUMODE_NORMAL  (0x01u<<2)
#define SCU_CPUMODE_DSLOW   (0x02u<<2)

#define SCU_DSPMODE_MASK    (0x03u<<0)
#define SCU_DSPMODE_SLOW    (0x00u<<0)
#define SCU_DSPMODE_NORMAL  (0x01u<<0)
#define SCU_DSPMODE_DSLOW   (0x02u<<0)

/*SCU PMU MODE*/
#define PMU_SHMEM_PWR_STAT  (0x01u<<8)
#define PMU_DEMOD_PWR_STAT  (0x01u<<7)
#define PMU_CPU_PWR_STAT    (0x01u<<6)
#define PMU_DSP_PWR_STAT    (0x01u<<5)

#define PMU_EXT_SWITCH_PWR  (0x01u<<4)

#ifdef CONFIG_CHIP_RK2818
#define PMU_LCDC_PD        (0x01u<<3)
#define PMU_DDR_PD        (0x01u<<2)
#define PMU_CPU_PD          (0x01u<<1)
#define PMU_DSP_PD          (0x01u<<0)
#else
#define PMU_SHMEM_PD        (0x01u<<3)
#define PMU_DEMOD_PD        (0x01u<<2)
#define PMU_CPU_PD          (0x01u<<1)
#define PMU_DSP_PD          (0x01u<<0)
#endif

/*SCU SOFTWARE RESET CON*/
#define CLK_RST_SDRAM       (1<<28)
#define CLK_RST_SHMEM1      (1<<27)
#define CLK_RST_SHMEM0      (1<<26)
#define CLK_RST_DSPA2A      (1<<25)
#define CLK_RST_SDMMC1      (1<<24)
#define CLK_RST_ARM         (1<<23)
#define CLK_RST_DEMODGEN    (1<<22)
#define CLK_RST_PREFFT      (1<<21)
#define CLK_RST_RS          (1<<20)
#define CLK_RST_BITDITL     (1<<19)
#define CLK_RST_VITERBI     (1<<18)
#define CLK_RST_FFT         (1<<17)
#define CLK_RST_FRAMEDET    (1<<16)
#define CLK_RST_IQIMBALANCE (1<<15)
#define CLK_RST_DOWNMIXER   (1<<14)
#define CLK_RST_AGC         (1<<13)
#define CLK_RST_USBPHY      (1<<12)
#define CLK_RST_USBC        (1<<11)
#define CLK_RST_DEMOD       (1<<10)
#define CLK_RST_SDMMC0      (1<<9)
#define CLK_RST_DEBLK       (1<<8)
#define CLK_RST_LSADC       (1<<7)
#define CLK_RST_I2S         (1<<6)
#define CLK_RST_DSPPER      (1<<5)
#define CLK_RST_DSP         (1<<4)
#define CLK_RST_NANDC       (1<<3)
#define CLK_RST_VIP         (1<<2)
#define CLK_RST_LCDC        (1<<1)
#define CLK_RST_USBOTG      (1<<0)

/*SCU CLK SEL0 CON*/
#define CLK_SDMMC1_SHFT     25
#define CLK_SDMMC1_MASK     (0x07u<<25)
#define CLK_SDMMC1_DIV(i)   (((i-1)&0x07u)<<25)

#define CLK_SENSOR_SHFT     23
#define CLK_SENSOR_MASK     (0x03u<<23)
#define CLK_SENSOR_24M      (0x00u<<23)
#define CLK_SENSOR_27M      (0x01u<<23)
#define CLK_SENSOR_48M      (0x02u<<23)

#define CLK_48M_SHFT     20
#define CLK_48M_MASK        (0x07u<<20)
#define CLK_48M_DIV(i)      (((i-1)&0x07u)<<20)


#define CLK_USBPHY_SHFT     18
#define CLK_USBPHY_MASK     (0x03u<<18)
#define CLK_USBPHY_24M      (0x00u<<18)
#define CLK_USBPHY_12M      (0x01u<<18)
#define CLK_USBPHY_48M      (0x01u<<18)

#define CLK_LCDC_ARMPLL     (0x00u<<16)//
#define CLK_LCDC_DSPPLL     (0x01u<<16)//
#define CLK_LCDC_CODPLL     (0x02u<<16)//

#define CLK_LCDC_SHFT     8
#define CLK_LCDC_MASK       (0x0ffu<<8)
#define CLK_LCDC_DIV(i)     (((i-1)&0xffu)<<8)

#define CLK_LCDC_DIVOUT     (0x00<<7)//
#define CLK_LCDC_27M        (0X01<<7)//

#define CLK_SDMMC0_SHFT     4
#define CLK_SDMMC0_MASK     (0x07u<<4)
#define CLK_SDMMC0_DIV(i)   (((i-1)&0x07u)<<4)

#define CLK_PCLK_SHFT     2
#define CLK_PCLK_MASK       (0x03u<<2)
#define CLK_HCLK_PCLK_11    (0x00u<<2)
#define CLK_HCLK_PCLK_21    (0x01u<<2)
#define CLK_HCLK_PCLK_41    (0x02u<<2)

#define CLK_HCLK_SHFT     0
#define CLK_HCLK_MASK       (0x03u<<0)
#define CLK_ARM_HCLK_11     (0x00u<<0)
#define CLK_ARM_HCLK_21     (0x01u<<0)
#define CLK_ARM_HCLK_31     (0x02u<<0)
#define CLK_ARM_HCLK_41     (0x03u<<0)

/*SCU CLK SEL1 CON*/
#define CLK_SHMEM1_SHFT     30
#define CLK_SHMEM1_MASK     (0x01u<<30)
#define CLK_SHMEM1_DEMODCLK (0x00u<<30)
#define CLK_SHMEM1_ARMCLK   (0x01u<<30)

#define CLK_SHMEM0_SHFT     29
#define CLK_SHMEM0_MASK     (0x01u<<29)
#define CLK_SHMEM0_DEMODCLK (0x00u<<29)
#define CLK_SHMEM0_ARMCLK   (0x01u<<29)

#define CLK_HSADCO_SHFT     28
#define CLK_HSADCO_MASK      (0x01u<<28)
#define CLK_HSADCO_NORMAL    (0x00u<<28)
#define CLK_HSADCO_INVERT    (0x01u<<28)

#define CLK_GPS_SHFT     27
#define CLK_GPS_MASK        (0x01u<<27)
#define CLK_GPS_DEMODCLK    (0x00u<<27)
#define CLK_GPS_TUNER_INPUT (0x01u<<27)

#define CLK_DEMOD_INTCLK    (0x00u<<26)//
#define CLK_DEMOD_EXTCLK    (0x01u<<26)//

#define CLK_DEMOD_ARMPLL    (0x00u<<24)//
#define CLK_DEMOD_DSPPLL    (0x01u<<24)//
#define CLK_DEMOD_CODPLL    (0x02u<<24)//

#define CLK_DEMOD_SHFT     16
#define CLK_DEMOD_MASK      (0x0ffu<<16)
#define CLK_DEMOD_DIV(i)    (((i-1)&0x0ffu)<<16)

#define CLK_LSADC_SHFT     8
#define CLK_LSADC_MASK      (0x0ffu<<8)
#define CLK_LSADC_DIV(i)    (((i-1)&0x0ffu)<<8)

#define CLK_CODEC_SHFT     3
#define CLK_CODEC_MASK      (0x1fu<<3)
#define CLK_CODEC_DIV(i)    (((i-1)&0x1fu)<<3)

#define CLK_CODEC_CPLLCLK   (0x00u<<2)//
#define CLK_CODEC_12M       (0x01u<<2)//

#define CLK_CPLL_SLOW       (0x00u<<0)//
#define CLK_CPLL_NORMAL     (0x01u<<0)//
#define CLK_CPLL_DSLOW      (0x02u<<0)//

/********************************************************************
**                          结构定义                                *
********************************************************************/
typedef volatile unsigned int   io_reg;      
struct rockchip_scu_reg_hw
{
    io_reg scu_pll_config[3];//[3];//0:arm 1:dsp 2:codec
    io_reg scu_mode_config;
    io_reg scu_pmu_config;
    io_reg scu_clksel0_config;
    io_reg scu_clksel1_config;
    io_reg scu_clkgate0_config;
    io_reg scu_clkgate1_config;
    io_reg scu_clkgate2_config;
    io_reg scu_softreset_config;
    io_reg scu_chipcfg_config;
    io_reg scu_cuppd;    /* arm power down */
        io_reg scu_clksel2_config;    /* 281x,clksell2. */
};

struct rockchip_grf_reg_hw
{
    io_reg  CPU_APB_REG0;
    io_reg  CPU_APB_REG1;
    io_reg  CPU_APB_REG2;
    io_reg  CPU_APB_REG3;
    io_reg  CPU_APB_REG4;
    io_reg  CPU_APB_REG5;
    io_reg  CPU_APB_REG6;
    io_reg  CPU_APB_REG7;
    io_reg  IOMUX_A_CON;
    io_reg  IOMUX_B_CON;
    io_reg  GPIO0_AB_PU_CON;
    io_reg  GPIO0_CD_PU_CON;
    io_reg  GPIO1_AB_PU_CON;
    io_reg  GPIO1_CD_PU_CON;
    io_reg  OTGPHY_CON0;
    io_reg  OTGPHY_CON1;
} ;


#define ARM_PLL_DELAY           800  // loop.ARM run at 24M,
// 20100719,HSL@RK , from (200*200) to 300*200 for change ddr failed.
#define OTHER_PLL_DELAY        (300*200)  // loop .ARM run at normal.

struct pll_vdd{
        int     max_hz;
        int     vdd;
};

// vdd a > vdd b, like VDD_BIGER(VDD_130,VDD_120) == TRUE.
#define VDD_BIGER( a , b )      ( (a) < (b) )       // vdd a > b ?

struct pll_vdd  scu_hw_vdd_table[] = {
#if MAYCHG_VDD              
        {660*SCU_CLK_MHZ, VDD_140},
        {630*SCU_CLK_MHZ, VDD_135},
        {600*SCU_CLK_MHZ, VDD_130},
        {560*SCU_CLK_MHZ, VDD_125},     // 1.2 == 1.25 
        {500*SCU_CLK_MHZ, VDD_125/*VDD_120*/},
        {300*SCU_CLK_MHZ, VDD_120/*VDD_115*/}, 
        {200*SCU_CLK_MHZ, VDD_115},
        {140*SCU_CLK_MHZ, VDD_110},
        {24*SCU_CLK_MHZ,   VDD_095}, // we not change vdd here,change at rk28_do_halt
#else        
        {540*SCU_CLK_MHZ, VDD_125},
        {24*SCU_CLK_MHZ,   VDD_125}, 
#endif        
} ;

int     ddr_min_vdd=VDD_105;                    // ddr special vdd,not the same for pll.
int     scu_hw_cur_pll_clk[3];

#ifndef CONFIG_LCD_RK_EINK
#define PWM3_REG_BASE_VA                        (PWM_BASE_ADDR_VA+0X30)
#define PWM3_REG_IOMUX                            (REG_FILE_BASE_ADDR_VA+0X24)
#define PWM3_ALWAYS_AS_IO                          1
static int __tcmdata current_vdd = -1;
#endif
int __tcmdata ddr_disabled = 0;


/* we get the correct vdd by all pll clk(max freq) and ddr request. 
*/
static struct pll_vdd * scu_hw_get_vdd( int id , struct rockchip_pll_set *pset )
{
        struct pll_vdd *ps,*pt;
        int effect_clk=0;
        int     i;

        if( pset ) {
                scu_hw_cur_pll_clk[id] = pset->clk_hz;
        }
        
        // chose max freq.
        for( i = 0 ; i < 3 ; i++ ){
                if( scu_hw_cur_pll_clk[i] > effect_clk )
                        effect_clk = scu_hw_cur_pll_clk[i];
        }
  
        ps = pt = &scu_hw_vdd_table[0];
        while( 1 ){
                if( pt->max_hz == effect_clk ) {
                        ps = pt;
                        break;
                }
                if( VDD_BIGER( ddr_min_vdd, pt->vdd ) ) {
                	if( VDD_BIGER( ddr_min_vdd, ps->vdd ) ) {
                	        if( pt > &scu_hw_vdd_table[0] )
                        	                ps = pt-1;
                        	        else 
                        	                ps = pt;
                        	}
                        break;
                }
                // we are sorted,and ps->clk_hz > pt->clk_hz.
                if(pt->max_hz > effect_clk ||
                        (effect_clk-pt->max_hz < ps->max_hz-effect_clk) )
                        ps = pt;
                 
                if(pt->max_hz <effect_clk || pt ->max_hz == 24*SCU_CLK_MHZ )
                        break;
                pt ++;
        }
        S_INFO("vdd:clk=%d,vdd=%d,clk:arm=%d,dsp=%d,codec=%d,ddr vdd=%d\n" , 
                effect_clk,ps->vdd,scu_hw_cur_pll_clk[0],
                scu_hw_cur_pll_clk[1],scu_hw_cur_pll_clk[2],ddr_min_vdd);
        return ps;
}
/* note: delay how long? outside ctrl.
 * 20100622,we use gpio--input--pulldown to set vdd=1.2(default setting).
 * so we always set pwm3 as gpio.
 * about 10 ms to be stable.
 * 20100707,change return value to indicate wether need delay:
 * 1: vdd up,need delay,0:vdd down or not change,do not need delay.
*/
#ifndef CONFIG_LCD_RK_EINK
int __tcmfunc rk28_change_vdd( int vdd )
{
        int     lrc = PMW3_PRESCALE;
        int     hrc = vdd;
        volatile unsigned long *reg;
        unsigned long val;
        int r = 0;
#if !MAYCHG_VDD              
        return 0;
#endif        
        if( current_vdd == vdd )
                return 0;
        #if 0
        // can not change!for lock pll enter 2 level sleep.
        if( rockchip_clk_get_arm() >= 300*SCU_CLK_MHZ  && 
                VDD_BIGER(VDD_110 ,vdd) ) {
                return 0;
        }
        #endif
        if( VDD_BIGER(vdd ,current_vdd) ) {
                /*  电压跨度 >= 0.2V. 10 = 0.2/0.476 = 40% = 10 / 25.
                 *  may crash the core.we warn here for debug.
                */
                if( current_vdd - vdd > 10 && !ddr_disabled) {
                        S_WARN("vdd raise from %d to %d\n" , current_vdd , vdd );
                }
                r = 1;
        }
        if( !ddr_disabled ) {
                S_INFO("%s::lrc=%d,hrc=%d\n" , __func__ , lrc , hrc );
                //dump_stack();
        }
        current_vdd = vdd;
        if( hrc == VDD_140  ) {
                // gpio , low , 1.4v
                #if !PWM3_ALWAYS_AS_IO
                reg = (volatile unsigned long *)PWM3_REG_IOMUX;
                *reg &= ~(0x3<<14); // mux pwm3 to gpio--1 pb 5 , pf5.
                #endif 
                
                // pf5,direction.
                reg = (volatile unsigned long *)(GPIO1_BASE_ADDR_VA+0x10);
                *reg |= (0x1<<5) ; // 1: output.

                // pf5,data.
                reg = (volatile unsigned long *)(GPIO1_BASE_ADDR_VA+0x0c);
	*reg &= ~(0x1<<5) ; // 0: low
	
        } else if( hrc == VDD_095 ) {
                // gpio , high.0.95v
                #if  !PWM3_ALWAYS_AS_IO
                reg = (volatile unsigned long *)PWM3_REG_IOMUX;
                *reg &= ~(0x3<<14); // mux pwm3 to gpio--1 pb 5 , pf5.
                #endif
                
                // direction.
                reg = (volatile unsigned long *)(GPIO1_BASE_ADDR_VA+0x10);
                *reg |= (0x1<<5) ; // 1: output.

                // data.
                reg = (volatile unsigned long *)(GPIO1_BASE_ADDR_VA+0x0c);
	*reg |= (0x1<<5) ; // 1: high
        } else if(hrc == VDD_125){
                // gpio , input ,pull down. 1.2v.
                #if !PWM3_ALWAYS_AS_IO
                reg = (volatile unsigned long *)PWM3_REG_IOMUX;
                *reg &= ~(0x3<<14); // mux pwm3 to gpio--1 pb 5 , pf5.
                #endif
                
                // direction.input
                reg = (volatile unsigned long *)(GPIO1_BASE_ADDR_VA+0x10);
                *reg &= ~(0x1<<5) ; // 0: input.

                // pull down.GPIO1_AB_PU_CON
                reg = (volatile unsigned long *)(REG_FILE_BASE_ADDR_VA+0x30);
                val = *reg;
                val &= ~(0x3 << (16+5*2) );
                val |= (0x2<<(16+5*2)) ; // 10: pull down.
	*reg = val;
        } 
        #if !PWM3_ALWAYS_AS_IO
        else {
                // pwm3,
                 reg = (volatile unsigned long *)PWM3_REG_IOMUX;
                *reg |= (0x1<<14); // mux to pwm3 .
                
                __raw_writel(PWM_DIV|PWM_RESET,PWM3_REG_BASE_VA+PWM_REG_CTRL);
                __raw_writel(lrc, PWM3_REG_BASE_VA+PWM_REG_LRC);
                __raw_writel(hrc, PWM3_REG_BASE_VA+PWM_REG_HRC);
                __raw_writel(0, PWM3_REG_BASE_VA+PWM_REG_CNTR);
                __raw_writel(PWM_DIV|PWM_ENABLE|PWM_TIME_EN,
                        PWM3_REG_BASE_VA+PWM_REG_CTRL);
        }
        #endif
        return r;
}
#else
int rk28_change_vdd( int vdd )
{
	return 0;
}
#endif


//static struct rockchip_scu_reg_hw *scu_register_base = (struct rockchip_scu_reg_hw *)(SCU_BASE_ADDR_VA);
//static struct rockchip_grf_reg_hw* scu_reg_file_base = (struct rockchip_grf_reg_hw*)REG_FILE_BASE_ADDR_VA;
#define scu_register_base               ( (struct rockchip_scu_reg_hw *)SCU_BASE_ADDR_VA)
#define scu_reg_file_base               ( (struct rockchip_grf_reg_hw *)REG_FILE_BASE_ADDR_VA)

int __rockchip_clk_set_unit_clock( ip_id id , int new_clk ) ;
struct rockchip_scu_unit * __rockchip_find_unit_at_node( struct rockchip_scu_unit *node , ip_id id );
static void __tcmfunc __rockchip_scu_pll_slowmod_hw( ip_id id  , int enter )
{
        
        if( enter ) {
                if( id == SCU_IPID_ARM )  {
                        scu_register_base->scu_mode_config &= ~SCU_CPUMODE_MASK;
                } else if( id == SCU_IPID_DSP )
                        scu_register_base->scu_mode_config &= ~SCU_DSPMODE_MASK;
                else if( id == SCU_IPID_CODEC )        
                        scu_register_base->scu_clksel1_config &= ~(0X03);
        } else {
                if( id == SCU_IPID_ARM ) {   
                        scu_register_base->scu_mode_config |= SCU_CPUMODE_NORMAL;
                } else if( id == SCU_IPID_DSP )
                        scu_register_base->scu_mode_config |= SCU_DSPMODE_NORMAL;
                else if( id == SCU_IPID_CODEC )        
                        scu_register_base->scu_clksel1_config |= (0X01);
        }
}

static inline io_reg   *scu_hw_get_selreg( struct rockchip_scu_unit * p )
{
        io_reg   *reg = &scu_register_base->scu_clksel0_config;
        if( p->divreg_index == 1 )
                reg++;
        else if( p->divreg_index == 2 )
                reg = &scu_register_base->scu_clksel2_config;
        return reg;
}

static void __rochchip_scu_setdiv_reg( struct rockchip_scu_unit * p , io_reg *reg ,
        int div )
{
        unsigned int val = __raw_readl(reg);
        val &= ~( ( ( 1<<(p->divbit_end-p->divbit_start+1) ) - 1 )<<p->divbit_start );
        val |= (div-1) << p->divbit_start;
        __raw_writel( val , reg );
}

/* set register to the value div 
  * 20100702,HSL@RK,for ddr ,can not change div here,we do with change pll together.
  * 20100716,HSL@RK,the same with hdiv,pdiv.
*/
int __rockchip_scu_setdiv_hw( struct rockchip_scu_unit * p )
{
        int     div = p->tmp_div;
        io_reg   *reg = scu_hw_get_selreg( p );
        if( p->id !=  SCU_IPID_DDR && p->id !=  SCU_IPID_HCLK
                && p->id !=  SCU_IPID_PCLK )
                __rochchip_scu_setdiv_reg( p , reg , div );
        S_INFO("scu hw set %s div=%d ,reg=0x%p\n" , p->name , p->tmp_div , reg );
        //printk("scu hw set %s div=%d ,reg=0x%p\n" , p->name , p->tmp_div , reg );
        return 0;
}

/* set register clk gate to enable this ip clk 
*  if enable arm/dsp,codec, return normal state 
*/
int __rockchip_scu_enable_hw( ip_id  id , struct rockchip_scu_unit * p)
{
        io_reg   *reg ;

        if( p && p->ifn )
                {
                S_INFO("enable %s,bak=%d\n" , p->name , p->tmp_clk );
                __rockchip_clk_set_unit_clock(id,p->tmp_clk );
                return 0;
                }
        
        if( id >= SCU_IPID_GATE_MAX )         
                return 0;
        
        reg = &scu_register_base->scu_clkgate0_config;
        while( id > 31 )
                {
                id -= 32;
                reg++;
                }
        (*reg) &= ~(1<<id );    // clear bit 
        return 0;
}

/* set register clk gate to disable this ip clk 
*  if disable arm/dsp,codec, enter slow mode 
*  20100710,HSL@RK,return the original state.
*/
int __rockchip_scu_disable_hw( ip_id  id , struct rockchip_scu_unit * p)
{
        io_reg  *reg ;
        int en;
                
        if( p && (p->propt&SCU_PROPT_HAVEPLL) )  {
                /* XXX:if want to change hclk div = 1 , tmp change SCU_IBIP_HCLK propt 
                  * 20100204,HSL@RK, div SCU_CLK_MHZ2KHZ for enable.
                */
                int oclk = p->cur_clk/(SCU_CLK_MHZ);
                __rockchip_clk_set_unit_clock(id,24);
                p->tmp_clk = oclk;
                return 0; //__rockchip_clk_set_unit_clock( id , 24 );
                }
        
        if( id >= SCU_IPID_GATE_MAX )  
                return 0;
        reg = &scu_register_base->scu_clkgate0_config;
        while( id > 31 )
                {
                id -= 32;
                reg++;
                }
        en = (*reg)&(1<<id)?0:1;
        (*reg) |= (1<<id );    // set_bit
        return en;
}

static void __tcmfunc scu_hw_pll_wait_lock( int id, int delay )
{
        io_reg * wr = &scu_reg_file_base->CPU_APB_REG0 ; 
        int bit = 1<< (7+id-SCU_IPID_ARM);
        while( delay > 0 ){
                if( *wr & bit )
                        break;
                delay --;
        }
        if( delay == 0 && !ddr_disabled ){
                S_WARN("%s::wait pll bit 0x%x time out!\n" ,__func__,bit );
        }
}

void __tcmfunc SDRAM_AfterUpdateFreq(uint32 SDRAMoldKHz, uint32 DDRnewKHz);
void __tcmfunc SDRAM_BeforeUpdateFreq(uint32 SDRAMnewKHz, uint32 DDRnewKHz);

static void __tcmfunc noinline scuhw_ddr_change_freq(struct rockchip_pll_set *ps, 
        io_reg *pll , int id , int ddr_div)
{
	int arm_clk = 180*1000;         //KHZ,no use.
	uint32 DDR_MHz = ps->clk_hz; // we change at scu_hw_for_change_ddr.
	S_INFO("change ddr to %ldMHZ,dly 1200 ",DDR_MHz);
	ddr_disabled = 1;
	SDRAM_BeforeUpdateFreq( arm_clk,  DDR_MHz);
	__rockchip_scu_pll_slowmod_hw( id , 1 );
        	(*pll) = PLL_SAT|PLL_FAST|
                         (PLL_CLKR(ps->clkr-1))|(PLL_CLKF(ps->clkf-1))|(PLL_CLKOD(ps->clkod- 1));
                //scu_register_base->scu_clksel0_config &= ~(0x3 << 30);
                //scu_register_base->scu_clksel0_config |= (ddr_div-1) << 30;
                //scu_hw_pll_wait_lock(id , OTHER_PLL_DELAY );  
                //scu_hw_pll_wait_lock这个函数等待lock不准，在CPU为156MHz时，大概延时400us
                ddr_pll_delay( 24000 );
	__rockchip_scu_pll_slowmod_hw( id , 0 );
	SDRAM_AfterUpdateFreq( arm_clk,  DDR_MHz);
	S_INFO(" OK!\n" );
	ddr_disabled = 0;
}

static void noinline scu_hw_for_change_ddr(struct rockchip_pll_set *ps, 
        io_reg *pll , int id , int ddr_div )
{
        struct rockchip_pll_set ps_stack = *ps;
        flush_cache_all();      // 20100615,HSL@RK.
        __cpuc_flush_user_all();
        scuhw_ddr_change_freq( &ps_stack ,pll , id , ddr_div);
        //S_INFO("%s::change ddr clk ok\n",__func__ );
}

/* for change sp. if in a function, some param will be pass by fp,ip, not sp. */
static void noinline scu_hw_wrap_for_ddr(struct rockchip_pll_set *ps, 
        io_reg *pll , int id , int ddr_div )
{
#if 0 //XXX: will crash,we need do all work before.
        unsigned long flags;
        local_irq_save(flags);
        /* XXX:if have irq, 4K SRAM is enougth?? */
        DDR_SAVE_SP;
        local_irq_restore(flags);
        scu_hw_for_change_ddr( ps , pll , id , ddr_div);
        local_irq_save(flags);
        DDR_RESTORE_SP;
        local_irq_restore(flags);
#else
        unsigned long flags;
        local_irq_save(flags);
        /* XXX:if have irq, 4K SRAM is enougth?? */
        DDR_SAVE_SP;
        scu_hw_for_change_ddr( ps , pll , id , ddr_div);
        DDR_RESTORE_SP;
        local_irq_restore(flags);
#endif 
}

static void scu_hw_set_ahb_apb_div( int ahb_div , int apb_div )
{
        io_reg  reg_val;
        reg_val = scu_register_base->scu_clksel0_config;
        reg_val  &= ~0xf;
        /* ahb div 2(0b10) == 3, apb div 2(0b10) == 4*/
        reg_val |= ((apb_div<<1)&0xc)|(ahb_div-1);
        scu_register_base->scu_clksel0_config = reg_val;
}

/* 20100622,if only change clkod,we do not need delay and slow mod.
  * 20100626,HSL@RK,check for pll lock status.
*/
static void __rockchip_scu_change_pll_hw( io_reg   *regpll ,
        struct rockchip_scu_unit * p , struct rockchip_pll_set *ps,
        struct rockchip_scu_unit * ddr , int vdd, int delay )
{
        int clkr,clkf;
        io_reg  reg_val = *regpll;
        int     change_pll = 1;
        unsigned long flags;
        clkr =  (((reg_val>>16)&0x3f)+1);
        clkf =  ((reg_val>>4)&0xfff)+1;

#if 0   // 20100726,HSL@RK,we change ahb ,so need to enter slow mod.
        if( clkr == ps->clkr && clkf == ps->clkf && !(reg_val & PLL_PD)) {
                S_INFO("only change %s clkod,cur reg=0x%x\n" , p->name , reg_val);
                change_pll = 0;  
         }
#endif         
         // we delay here ,not at slow mod.
        if( p->cur_clk < p->tmp_clk ) {// increase freq.
                if( rk28_change_vdd( vdd ) ) {
                        mdelay( 10 );
                }
        }

        /*20100726,HSL@RK,close irq for change ahb = ahb =1.*/
        local_irq_save(flags);
        if( change_pll && !ddr )
                __rockchip_scu_pll_slowmod_hw( p->id , 1 );
        /* if change arm pll,we change vdd core and ahb,apb div. */
        if(  p->id == SCU_IPID_ARM ) {
                        if( change_pll && !ddr) // we at slow mod herer!set ahb = apb = 24m.
                                scu_register_base->scu_clksel0_config &= ~0xf;
                        else {
                        /* if only change od,we not at slow mode.if increase arm freq ,
                          *  we must set ahb,apb first.
                         */
                                if( p->cur_clk < p->tmp_clk ) {
                                        scu_hw_set_ahb_apb_div( ps->ahb_div , ps->apb_div );
                                }
                        }
        }
        local_irq_restore(flags);
        if( (reg_val & PLL_PD) && !ddr  ) {
                (*regpll) &= ~PLL_PD;
                S_INFO("%s::%s pll power on,set to %d,delay=%d\n" , __func__ ,p->name , ps->clk_hz , delay );
                scu_hw_pll_wait_lock(p->id , delay*2 );
        }
        /* XXX:delay for pll state , for 0.3ms , clkf will lock clkf*/
        if( ddr ){
                struct rockchip_pll_set ps_stack = *ps;
                /* 20100820,HSL@RK,when change ddr freq,DDR can not be access,we puase all
                  * possible DMA channel.(lcd already off)we close change 0,1,2 here.
                */
                io_reg *i2c_dma_cfg = (io_reg*)(DW_DMA_BASE_ADDR_VA+DWDMA_CFGL(RK28_DMA_CH0));
                int     one_channel_offset = 0x58/sizeof(io_reg);
                //int     en ;
                ps_stack.clk_hz /= (1000000*ddr->cur_div) ; // cal ddr clk from hz to MHZ.
                //ps_stack.clk_hz /= (1000*ddr->cur_div) ; // cal ddr clk from hz to KHZ.
                S_INFO("%s::ddr clk =%d Mhz\n",__func__ , ps_stack.clk_hz );
                *i2c_dma_cfg |= (1<<8); // suspend dma channe0 --SDMMC0
                i2c_dma_cfg += one_channel_offset;
                *i2c_dma_cfg |= (1<<8); // suspend dma channel -- SDMMC1
                i2c_dma_cfg += one_channel_offset;
                *i2c_dma_cfg |= (1<<8); // suspend dma channe2 -- I2S.
                //en = __rockchip_scu_disable_hw(SCU_IPID_DMA , NULL );
                scu_hw_wrap_for_ddr( &ps_stack , regpll , p->id , ddr->cur_div);
                
                //if( en )  __rockchip_scu_enable_hw(SCU_IPID_DMA , NULL );
                *i2c_dma_cfg &= ~(1<<8); // resume dma channe2 
                i2c_dma_cfg -= one_channel_offset;
                *i2c_dma_cfg &= ~(1<<8); 
                i2c_dma_cfg -= one_channel_offset;
                *i2c_dma_cfg &= ~(1<<8); 
        } else {
                /* 可能屏幕会倒动,如果不修改 clkr和clkf则不用进入slow mode .*/
                (*regpll) = PLL_SAT|PLL_FAST|
                        (PLL_CLKR(ps->clkr-1))|(PLL_CLKF(ps->clkf-1))|(PLL_CLKOD(ps->clkod- 1));
               #if 0
               if( change_pll )
                        scu_hw_pll_wait_lock(p->id , delay );
               #else // 20100725,HSL@RK,we not at slow mode,so can delay here.
               scu_hw_pll_wait_lock(p->id , delay );
               #endif
        }
        if(  p->id == SCU_IPID_ARM ) {
                S_INFO("%s::set ahb=%d,apb=%d\n" , __func__ , ps->ahb_div , ps->apb_div);
                scu_hw_set_ahb_apb_div( ps->ahb_div , ps->apb_div );
        }
        if( p->cur_clk > p->tmp_clk ) {// decrease freq.
                rk28_change_vdd( vdd );
        }
        if( !ddr )
        __rockchip_scu_pll_slowmod_hw(p->id , 0 );
}

/*
* return value :
*       0: need parent change to set as p->tmp_clk .
*       1: set ok , no need parent change.
*       -1: can not change to special CLK.
*       XXX:when arm pll down ,need set ahb div = 1, ahb = 24M? apb ?
*/
int __rockchip_scu_set_pllclk_hw( struct rockchip_scu_unit * p  , int stage)
{
        int pll_i =p->id - SCU_IPID_ARM;
        register io_reg   *regpll = &scu_register_base->scu_pll_config[pll_i];
        register struct rockchip_pll_set *ps = p->pll_set;
        struct rockchip_scu_unit * pddr;
         int  delay[3] = {ARM_PLL_DELAY,OTHER_PLL_DELAY,OTHER_PLL_DELAY};
        struct pll_vdd * vdd;
        if( !ps )
                return -1;
        if( stage == STAGE_BEFORE_CHG ) {      /* not real set clk , just require */
                return 0;
        }
        
        pddr = __rockchip_find_unit_at_node( p, SCU_IPID_DDR);
        if( pddr ){
                if( ps->delay == 2 ) {
                        ddr_min_vdd = VDD_130;
                } else if( ps->delay == 1 ) {
                        ddr_min_vdd = VDD_125;
                } else {
                        ddr_min_vdd = VDD_105;
                }
        }
        vdd = scu_hw_get_vdd( pll_i,ps);
        
        if( p->tmp_clk == 24*SCU_CLK_MHZ && !pddr ) {
                //struct rockchip_scu_unit ahb = __rockchip_find_unit_at_node();
                __rockchip_scu_pll_slowmod_hw( p->id , 1 );
                /* 20100615,HSL@RK,when power down,set pll out=300 M.*/
                (*regpll) = PLL_SAT|PLL_FAST|(PLL_CLKR(ps->clkr-1))
                        |(PLL_CLKF(ps->clkf-1))|(PLL_CLKOD(ps->clkod- 1));
                //p->tmp_clk = 24*1000*1000;
                S_INFO("set 24M clk ,%s power down\n", p->name );
                __udelay( delay[pll_i] );   /* pll stabe ?then power down. */
                (*regpll) |= PLL_PD;
                if( p->id == SCU_IPID_ARM ){
                        scu_hw_set_ahb_apb_div( ps->ahb_div , ps->apb_div );
                        /* 20100919,HSL@RK,if sysrestart,we change vdd to io ctrl,not pwm. */
                        if(system_state == SYSTEM_RESTART ){
                            S_INFO("%s::before restart,change vdd=125\n" , __func__ );
                            rk28_change_vdd( VDD_125 );
                        }
                } else {
                        /* 20100723,HSL@RK,change arm to 24m,we change vdd at rk28_do_halt.
                          * other,may happen arm clk down,but dsp is on.so we change vdd when dsp off.
                        */
                        rk28_change_vdd( vdd->vdd );
                }
        } else {
                S_INFO("set %s pll,clkr=%d,clkf=%d,clkod=%d\n", p->name ,ps->clkr,ps->clkf,ps->clkod);
                __rockchip_scu_change_pll_hw(regpll , p , ps , pddr , vdd->vdd ,delay[pll_i] );
        }
        return 0;
}

/* for sensor , usb phy 48M */
int __rockchip_scu_set_48m_hw( struct rockchip_scu_unit * p  , int stage)
{
#if 0
        int     start_bit ;
        int     mask_bit;

        /* not support externel clk */
        if( p->tmp_clk != 24*SCU_CLK_MHZ2KHZ && p->tmp_clk != 48*SCU_CLK_MHZ2KHZ )
                return SCU_SETCLK_IMPOSSIBLE;
        if( stage == 0 ) {       /* not real set clk , just require */
                if( p->tmp_clk == 48 ) {
                        p->tmp_div = p->parent->max_clk / p->tmp_clk;
                        if( p->tmp_div > SCU_DIV_MAXVALUE(p) )
                                p->tmp_div = SCU_DIV_MAXVALUE(p);
                        p->parent->tmp_clk = p->tmp_div * p->tmp_clk ;
                        S_INFO("set %s to 48M,so parent %s to %d", 
                                p->name , p->parent->name , p->parent->tmp_clk );
                        return SCU_SETCLK_PARENT; /* need parent change !*/
                }
                return 0;
        }

        start_bit = 23; /* sensor , 23-24 */
        mask_bit = (0x03)<<23;
        if( p->id == SCU_IPID_USBPHY ) {
                start_bit = 18; /* usb phy , 18-19 */
                mask_bit = (0x03)<<18;
        }
                
        if( p->tmp_clk == 24 ) {
                S_INFO("set 24M clk ,%s power down", p->name );
                scu_register_base->scu_clksel0_config &= ~mask_bit;
        } else {
                scu_register_base->scu_clksel0_config |= (2)<<start_bit;
        }
#endif        
        return 0;  
}

/* 20100702,HSL@RK,if change ddr parent,we need the new clk == old clk. */
 int rockchip_chg_parent_all(struct rockchip_scu_unit *p,
        struct rockchip_scu_unit *np )
 {
        /* bit 17:16 of clk sel0 */
        int     start_bit;
        int     mask = 0x3; /* 2bits */
        int     val ;
        io_reg   *regsel = &scu_register_base->scu_clksel0_config;

        if( p->id == SCU_IPID_LCDC )
            start_bit = 16;
        else if( p->id == SCU_IPID_I2S ) {
            start_bit = 2;
            regsel++;   /* clk sel 1*/
            mask = 1;
        }
        else if( p->id == SCU_IPID_SENSOR)
            start_bit = 23;  
        else
            return -1;
        if( p->id == SCU_IPID_LCDC ) {
            switch( np->id ){
            case SCU_IPID_ARM:
                val = 0;
                break;
            case SCU_IPID_CODEC:
                val = 2;
                break;
            case SCU_IPID_DSP:
                val = 1;
                break;
            default:
                return -2;
            }
        } else if (p->id == SCU_IPID_I2S ) {
            switch( np->id ){
            case SCU_IPID_CODEC:
                val = 0;
                break;
            case SCU_IPID_12M:
                val = 1;
                break;
            default:
                return -2;
            }
        } else if( p->id == SCU_IPID_DDR) {
                if( p->parent->cur_clk != np->cur_clk )
                        return -3;
                start_bit = 28;
            switch( np->id ){
            case SCU_IPID_ARM:
                val = 1;
                break;
            case SCU_IPID_CODEC:
                val = 0;
                break;
            case SCU_IPID_DSP:
                val = 2;
                break;
            default:
                return -2;
            }
        } else { /* SCU_IPID_SENSOR */
            switch( np->id ){
            case SCU_IPID_ARM:  /* ARM PLL FOR 48M */
                val = 2;
                break;
            case SCU_IPID_24M:
                val = 0;
                break;
            default:    /* extern 27M not support */
                return -2;
            }
        }

        /* 20100208,HSL@RK,change to one instr! */
        *regsel = ((*regsel) & (~(mask<<start_bit)) )|(val<<start_bit);
        //*regsel &= ~(mask<<start_bit);
        //*regsel |= (val<<start_bit);
        return 0;
 }

/*
 *  20100203,HSL@RK,msleep can not use at irq or softirq.
 *  index: datasheet , SOFTRESET REG.
 */
int rockchip_scu_reset_unit( int index )
{
        int val = 1;
        if( index >= 32 )       
                return 0;
        if( index == 11 || index == 12 ){
                index = 11;
                val = 3;
        }
        scu_register_base->scu_softreset_config |= (val<<index);
        if( !in_interrupt() && !__system_crashed() )
                msleep(10);
        else 
                mdelay( 5 );
        scu_register_base->scu_softreset_config &= ~(val<<index);
        if( !in_interrupt() && !__system_crashed())
                msleep(10);
        else 
                mdelay( 5 );
        return 0;
}
int __rockchip_get_unit_div_hw( struct rockchip_scu_unit *p )
{
        int     div;
        io_reg   *regsel = scu_hw_get_selreg( p );
                
        //div = 1+ ( ((*regsel)>>ipinfo->divbit_start )&((1<<(ipinfo->divbit_end-ipinfo->divbit_start+2))-1) );
        div = 1+ ( ((*regsel)>>p->divbit_start )&((1<<(p->divbit_end-p->divbit_start+1))-1) );
        if( p->id == SCU_IPID_PCLK && div == 3 )
                div = 4;
        S_INFO("get %s div=%d,reg=0x%p,regval=0x%08x\n" , p->name , div ,regsel, *regsel );
        return div;
}

int __rockchip_scu_get_pll_clk_hw( ip_id id )
{
        int     clk = 24*1000;
        unsigned int reg_val ;
        io_reg   *regpll = &scu_register_base->scu_pll_config[0];
        if( id == SCU_IPID_DSP ) {
                regpll++;
        }
        if( id == SCU_IPID_CODEC ) {
                regpll+= 2;
        }
        reg_val = *regpll;
        if( !(reg_val & PLL_PD) ) {
                /* 20101018,HSL@RK,if clk=24M,we may overflow when 24*clkf(clkf==294) */
                clk = clk * (((reg_val>>4)&0xfff)+1) /( (((reg_val>>16)&0x3f)+1)  * (((reg_val>>1)&0x7)+1));
        }
        clk *= 1000;
        S_INFO("read pll config 0x%p,reg val=0x%08x,clk=%dHz\n" , regpll , reg_val , clk );
        S_INFO("clkr=%d,clkf=%d,clkod=%d\n", (((reg_val>>16)&0x3f)+1) ,(((reg_val>>4)&0xfff)+1),(((reg_val>>1)&0x7)+1));
        return clk;
}

int  __rockchip_scu_get_ahb_clk_hw( void )
{
        int div = (scu_register_base->scu_clksel0_config&0x3)+1;
       // return __rockchip_scu_get_pll_clk_hw(SCU_IPID_ARM)/div;
        return __rockchip_scu_get_pll_clk_hw(0)/div;
}

int  __rockchip_scu_get_apb_clk_hw( void )
{
        int div =1<< ( (scu_register_base->scu_clksel0_config>>2)&0x3);
        //if( div == 3 )
        //        div = 4;
        return __rockchip_scu_get_ahb_clk_hw()/div;
}


int *(*rk28_idle)(void ) ;
void __init __rockchip_scu_init_hw( void )
{        
        //int * p;
        unsigned long reg_value,sfr;
    
        S_INFO("%s:: init scu hardware\n" , __func__ );
        /* reset dps for softreboot unstable state.*/
        sfr = scu_register_base->scu_softreset_config;
        scu_register_base->scu_softreset_config |= 0
                                    | (1<<4)    // dsp core
                                    | (1 <<5)   // dsp pherial
                                    | (1 << 8 ) // deblocking 
                                    | ( 1 << 25 ); // dsp A2A.
        mdelay( 5 );     
        scu_register_base->scu_softreset_config = sfr;
        // power domain.
         scu_register_base->scu_mode_config = SCU_INT_CLR|SCU_WAKEUP_POS|SCU_ALARM_WAKEUP_DIS\
                             |SCU_EXT_WAKEUP_DIS|SCU_CPUMODE_NORMAL
                             |SCU_DSPMODE_SLOW;
                             //|SCU_DSPMODE_NORMAL;
        scu_register_base->scu_pmu_config = 0x0;
        scu_register_base->scu_pmu_config |=  PMU_DSP_PD;
        reg_value = scu_register_base->scu_clksel0_config;
        reg_value &= ~((0x3<<16)); // |(0xf)
        reg_value |=  0
                        // (CLK_SDMMC1_DIV(4)       //sdmmc1 divider div (4)
                        //| CLK_SENSOR_24M        //sensor clock select 24MHz
                        //| CLK_48M_DIV(4)        //48MHz divider div (4)
                        //| CLK_USBPHY_24M        //USB PHY clock select 24MHz
                        //| CLK_LCDC_CODPLL       //lcdc clock divide from codecpll
                        //| CLK_LCDC_ARMPLL       //lcdc clock divide from armpll
                        | (LCDC_PLL_SOURCE<<16)
                        | CLK_LCDC_DIV(20)       //lcdc divider div (8)
                        //| CLK_LCDC_DIVOUT       //lcdc clock from divider out
                        //| CLK_SDMMC0_DIV(8)     //sdmmc0 divder div (4)
                        //| CLK_HCLK_PCLK_21       //hclk:pclk = 2:1
                        //| CLK_ARM_HCLK_41       //arm clk:hclk = 4:1
                        |0;
        scu_register_base->scu_clksel0_config = reg_value;
        /* not set gate0,usb failed,sdmmc1 ok*/
        /*not set gate1,usb failed,sdmmc1 ok*/
        /*not set gate2,usb OK and sdmmc1 ok*/
        #if 1
        //clock gate初始设置，0x00为打开，0x01为关闭
        scu_register_base->scu_clkgate0_config = 0
             //            |(0x01u<<SCU_IPID_SDMMC1)    // sdio failed.
                         |(0x01u<<SCU_IPID_UART3)  
                         | (0x01u<<SCU_IPID_UART2)
             //            |(0x01u<<SCU_IPID_LSADC)   
                         | (0X01u<<SCU_IPID_RTC) 
                         |(0x01u<<SCU_IPID_WDT)     
            //             | (0x01u<<SCU_IPID_PWM)  // 20100626,A7,如果关闭，唤醒会白屏
                         |(0x01u<<SCU_IPID_SPI1)    
                         | (0x01u<<SCU_IPID_SPI0)
                         |(0x01u<<SCU_IPID_I2C1)    
                         | (0x01u<<SCU_IPID_I2C0)
              //           |(0x01u<<SCU_IPID_UART1)   
              //           |(0x01u<<SCU_IPID_UART0)//串口调试输出
              //           |(0x01u<<SCU_IPID_GPIO1)  
              //           | (0x01u<<SCU_IPID_GPIO0)
              //           |(0x01u<<SCU_IPID_EBROM)       // mask rom,if close ,read data error.audio.
              //           |(0x01u<<SCU_IPID_SDMMC0) 
              //           | (0x01u<<SCU_IPID_I2S)
               //         | (0x01u<<SCU_IPID_VIP)		/*add by xxm 2010-7-15*/
                         |(0x01u<<SCU_IPID_DEBLK) 
                         |(0x01u<<SCU_IPID_HIF)     
                         | (0x01u<<SCU_IPID_SRAMDSP)
              //           |(0x01u<<SCU_IPID_SRAMARM) 
             //            | (0x01u<<SCU_IPID_DMA)      /* HSL@RK,20090516,can not close*/
                         |(0x01u<<SCU_IPID_DSPc);
        #endif
        #if 1
        scu_register_base->scu_clkgate1_config  = 0
     
                         | (0x01u<<(SCU_IPID_USBHOST&0x1f))
                         |(0x01u<<(SCU_IPID_DSPmib&0x1f))  
                         | (0x01u<<(SCU_IPID_DSPsib&0x1f))
                         |(0x01u<<(SCU_IPID_DSPt&0x1f))
                         | (0x00u<<(SCU_IPID_DDRAXI&0x1f)) //!!!: low disable.
                         |(0x01u<<(SCU_IPID_SDRAM&0x1f)) 
                         | (0x01u<<(SCU_IPID_MCDMA&0x1f))
                         |(0x01u<<(SCU_IPID_GPU&0x1f))
                         |(0x01u<<(SCU_IPID_DBLh&0x1f))
                   //      |(0x01u<<(SCU_IPID_LCDCh&0x1f))       
                   //      | (0x01u<<(SCU_IPID_LCDCs&0x1f))
                         |(0x01u<<(SCU_IPID_MSDRAM_CTRL&0x1f))
                         |(0x01u<<(SCU_IPID_SDRAM_CTRL&0x1f))
                         | (0x01u<<(SCU_IPID_MSDRAM_COM&0x1f))
                         |(0x010u<<(SCU_IPID_HSADC&0x1f))
                         |0;
        #endif
        #if 1
        scu_register_base->scu_clkgate2_config  = 0
                          |(0x01u<<(SCU_IPID_DSPBUS&0x1f))
                //          |(0x01u<<(SCU_IPID_EXPBUS&0x1f))  // HSL@RK,usb will failed if close.
                          |(0x01u<<(SCU_IPID_EFUSE&0x1f))
                          |(0x01u<<(SCU_IPID_DTCM1&0x1f))
                          |(0x01u<<(SCU_IPID_DTCM0&0x1f))
                          |(0x01u<<(SCU_IPID_VAHB&0x1f))
                          |0;
        #endif
}
#if 0
void __tcmfunc open_dsp_power_at_tcm( void )
{
        unsigned long flags;
        printk("%s!!\n",__func__);
        local_irq_save(flags);
        /* arm enter slow mode */
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0xc) &(~0xc)) , SCU_BASE_ADDR_VA+0xc);
        /* dsp subsys power on 0x21*/
        ddr_pll_delay(24);	//开之前也得加,避免总线还在访问
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x10) & (~0x21)) , SCU_BASE_ADDR_VA+0x10);
        ddr_pll_delay(240);	//关中断时间不能太长 (6000大概为1us)
        /* arm enter normal mode */
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0xc) |(1<<2)) , SCU_BASE_ADDR_VA+0xc);
        local_irq_restore(flags);
        mdelay(10);
}

void open_dsp_power_at_ddr(  void )
{
        unsigned long flags;
        printk("%s!!\n",__func__);
        local_irq_save(flags);
        /* dsp subsys power on 0x21*/
        ddr_pll_delay(6000);	//开之前也得加,避免总线还在访问
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x10) & (~0x21)) , SCU_BASE_ADDR_VA+0x10);
        ddr_pll_delay(6000);	//关中断时间不能太长 (6000大概为1us)
        local_irq_restore(flags);
        mdelay(10);
}
 void dsp_power_off( void )
 {
        mdelay(10);
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x10) | (0x21)) , SCU_BASE_ADDR_VA+0x10);
}
#endif
