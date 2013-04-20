/* arch/arm/mach-msm/pm.c
 *
 * Goldfish Power Management Routines
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <asm/arch/system.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/rk28_irqs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/rk28_scu.h>
#include <asm/irq_regs.h>
#include <linux/android_power.h>
#include <asm/arch/rk28_debug.h>

extern void rockchip_timer_clocksource_suspend_resume(int suspend );
extern int rockchip_timer_clocksource_irq_checkandclear( void );
extern u64 rockchip_timer_read(void);
extern void rockchip_timer_freeze(int freeze );

volatile int     rk28_pm_status = 0 ; // 0: normal , 1 : one level suspend , 2: two level suspend ,
extern void rk28_halt_at_tcm( void );

extern int rk28_usb_suspend( int exitsuspend );
extern int rk28_usb_check_vbus_change( void );
extern int rk28_usb_check_connectid_change(void);


/* 
 * 20100309,HSL@RK,test fun . open when need.
*/
#if 0

void rk28_print_scu_reg( void )
{
        int *reg = (int *)(SCU_BASE_ADDR_VA);   
        debug_print("0x%08x 0x%08x 0x%08x 0x%08x\n"
                            "0x%08x 0x%08x 0x%08x 0x%08x\n"
                          //  "0x%08x 0x%08x 0x%08x 0x%08x\n" ,
                            ,reg[0],reg[1],reg[2],reg[3],
                            reg[4],reg[5],reg[6],reg[7]);
}

/*
 * 20100309,HSL@RK,test fun for caclulate latency from deep sleep to irq action.
 * some driver need handle irq quickly as possible.
 */
void halt_latency( void )
{
        u64 now=rockchip_timer_read() ;
        printk("wakeup time cycle=%Ld,now cycle=%Ld,delay=%Ld us\n" ,
                rk28_up_timer_cycle ,now,(now-rk28_up_timer_cycle)*4 );
}

/*
 * 20100309,HSL@RK,check fun for system no action,and softlookup no effective,
 * but irq can go.
*/
static unsigned long last_jn;
void rk28_check_jiffies_at_irq( void )
{
        struct pt_regs *new_regs = get_irq_regs();
        int             print = 0;

        if( !last_jn )
                last_jn = jiffies;
        printk("last jiffies=%ld,now jiffies=%ld,goes=%ld\n" , last_jn , jiffies ,jiffies -last_jn);
        if( jiffies -last_jn < 25 || print ) {
                
                if (new_regs)
		show_regs(new_regs);
	else
		dump_stack();
        }
        last_jn = jiffies;
}
#endif

void pm_dump_scu_grf(void)
{
	int regvalue;
	printk("_______________________SCU Regs________________________________\n");
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x00));printk("SCU_APLL_CON:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x04));printk("SCU_DPLL_CON:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x08));printk("SCU_CPLL_CON:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x0c));printk("SCU_MODE_CON:     0x%08x\n",regvalue);
	
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x10));printk("SCU_PMU_MODE:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x14));printk("SCU_CLKSEL0_CON:  0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x18));printk("SCU_CLKSEL1_CON:  0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x1c));printk("SCU_CLKGATE0_CON: 0x%08x\n",regvalue);
	
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x20));printk("SCU_CLKGATE1_CON: 0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x24));printk("SCU_CLKGATE2_CON: 0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x28));printk("SCU_SOFTRST_CON:  0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x2c));printk("SCU_CHIPCFG_CON:  0x%08x\n",regvalue);
	
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x30));printk("SCU_CPUPD: 0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(SCU_BASE_ADDR_VA+0x34));printk("SCU_CLKSEL2_CON: 0x%08x\n",regvalue);
	printk("_______________________SCU Regs________________________________\n");

	printk("_______________________GRF Regs________________________________\n");
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0000));printk("CPU_APB_REG0:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0004));printk("CPU_APB_REG1:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0008));printk("CPU_APB_REG2:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x000c));printk("CPU_APB_REG3:     0x%08x\n",regvalue);
	
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0010));printk("CPU_APB_REG4:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0014));printk("CPU_APB_REG5:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0018));printk("CPU_APB_REG6:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x001c));printk("CPU_APB_REG7:     0x%08x\n",regvalue);
	
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0020));printk("IOMUX_A_CON:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0024));printk("IOMUX_B_CON:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0028));printk("GPIO0_AB_PU_CON:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x002c));printk("GPIO0_CD_PU_CON:     0x%08x\n",regvalue);
	
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0030));printk("GPIO1_AB_PU_CON:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0034));printk("GPIO1_CD_PU_CON:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x0038));printk("OTGPHY_CON0:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(REG_FILE_BASE_ADDR_VA+0x003c));printk("OTGPHY_CON1:     0x%08x\n",regvalue);
	printk("_______________________GRF Regs________________________________\n");
	
	printk("_______________________GPIO0 Regs________________________________\n");
	regvalue = readl((uint32_t *)(GPIO0_BASE_ADDR_VA+0x0000));printk("GPIO_SWPORTA_DR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO0_BASE_ADDR_VA+0x0004));printk("GPIO_SWPORTA_DDR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO0_BASE_ADDR_VA+0x000c));printk("GPIO_SWPORTB_DR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO0_BASE_ADDR_VA+0x0010));printk("GPIO_SWPORTB_DDR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO0_BASE_ADDR_VA+0x0018));printk("GPIO_SWPORTC_DR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO0_BASE_ADDR_VA+0x001c));printk("GPIO_SWPORTC_DDR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO0_BASE_ADDR_VA+0x0024));printk("GPIO_SWPORTD_DR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO0_BASE_ADDR_VA+0x0028));printk("GPIO_SWPORTD_DDR:     0x%08x\n",regvalue);
	printk("_______________________GPIO0 Regs________________________________\n");
	
	printk("_______________________GPIO1 Regs________________________________\n");
	regvalue = readl((uint32_t *)(GPIO1_BASE_ADDR_VA+0x0000));printk("GPIO_SWPORTA_DR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO1_BASE_ADDR_VA+0x0004));printk("GPIO_SWPORTA_DDR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO1_BASE_ADDR_VA+0x000c));printk("GPIO_SWPORTB_DR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO1_BASE_ADDR_VA+0x0010));printk("GPIO_SWPORTB_DDR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO1_BASE_ADDR_VA+0x0018));printk("GPIO_SWPORTC_DR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO1_BASE_ADDR_VA+0x001c));printk("GPIO_SWPORTC_DDR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO1_BASE_ADDR_VA+0x0024));printk("GPIO_SWPORTD_DR:     0x%08x\n",regvalue);
	regvalue = readl((uint32_t *)(GPIO1_BASE_ADDR_VA+0x0028));printk("GPIO_SWPORTD_DDR:     0x%08x\n",regvalue);
	printk("_______________________GPIO1 Regs________________________________\n");

}

static int rk28_pm_enter(suspend_state_t state)
{
	int arm_clk = rockchip_clk_get_arm();
	int dsp_clk = __rockchip_clk_get_uint_clk( SCU_IPID_DSP );
	int arm_lock;

	if( rk28_pm_status )
		return 0;

	rockchip_clk_unlock_pll( SCU_IPID_ARM );
	arm_lock = scu_unlock_arm_force();
	if( arm_clk > 500*SCU_CLK_MHZ ) {
		rockchip_clk_set_arm( arm_clk>>1  ); 
		rockchip_clk_set_arm( arm_clk>>2 );
	}
	rockchip_clk_set_arm(24);	

#if (LCDC_PLL_SOURCE == 0x1u)
	if( dsp_clk != 24*SCU_CLK_MHZ ){
		rockchip_clk_unlock_pll( SCU_IPID_DSP );
		rockchip_clk_set_codecdsp( SCU_IPID_DSP, 24 );
	}
#endif

	rockchip_timer_clocksource_suspend_resume( 1 );
	rk28_pm_status = 1;
	rk2818_set_suspend_flags(PM_TWOLEVEL_SLEEP);
	mdelay(2);
	rk28_usb_suspend( 0 );
	pr_info("PM: system sleep\n" );
	rk28_halt_at_tcm();
	pr_info("PM: system wake\n");
	rk28_usb_suspend( 1 );
	udelay(400);
	rockchip_timer_clocksource_suspend_resume( 0 );
	
	if( arm_clk > 500*SCU_CLK_MHZ ) {
		rockchip_clk_set_arm( arm_clk>>2 );
		rockchip_clk_set_arm( arm_clk>>1  ); 
	}
	rockchip_clk_set_arm( arm_clk  );
	rockchip_clk_lock_pll( SCU_IPID_ARM );
	scu_lock_arm_force(arm_lock);
	
#if (LCDC_PLL_SOURCE == 0x1u)
	if( dsp_clk != 24*SCU_CLK_MHZ ){
		rockchip_clk_set_codecdsp(SCU_IPID_DSP,dsp_clk );
		rockchip_clk_lock_pll( SCU_IPID_DSP );
	}
#endif
	return 0;
}

static void rk28_pm_finish( void )
{
        if( rk28_pm_status == 1 )
                rk28_pm_status = 0;
}

static struct platform_suspend_ops rk28_pm_ops = {
	.enter	= rk28_pm_enter,
	.valid	= suspend_valid_only_mem,
	.finish	= rk28_pm_finish,
};

static int __init rk28_pm_init(void)
{
	suspend_set_ops(&rk28_pm_ops);
	return 0;
}

__initcall(rk28_pm_init);

