/* arch/arm/mach-rockchip/rk28_backlight.c
 *
 * Copyright (C) 2009 Rockchip Corporation.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include <asm/io.h>
#include <asm/arch/typedef.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/rk28_backlight.h>
#ifdef CONFIG_ANX7150
#include <asm/arch/anx7150.h>
#endif

//#define RK28_PRINT 
#include <asm/arch/rk28_debug.h>

/*
 * Debug
 */
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif


#define write_pwm_reg(id, addr, val)        __raw_writel(val, addr+(PWM_BASE_ADDR_VA+id*0x10)) 
#define read_pwm_reg(id, addr)              __raw_readl(addr+(PWM_BASE_ADDR_VA+id*0x10))    
#define mask_pwm_reg(id, addr, msk, val)    write_dma_reg(id, addr, (val)|((~(msk))&read_dma_reg(id, addr)))


static struct rk28_backlight *rk28_bl = NULL;
#define BACKLIGHT_SEE_MINVALUE	53

static s32 rk28_bl_get_brightness(struct backlight_device *dev)
{
	struct rk28_backlight *bl = dev_get_drvdata(&dev->dev);
    bl->div_total = read_pwm_reg(bl->id, PWM_REG_LRC);
    bl->divh = read_pwm_reg(bl->id, PWM_REG_HRC);
    if (bl->pdata->bl_ref) {
        return BL_STEP * bl->divh / bl->div_total;
    } else {
        return BL_STEP - (BL_STEP * bl->divh / bl->div_total);
    }
}

static s32 rk28_bl_update_status(struct backlight_device *dev)
{
	struct rk28_backlight *bl = dev_get_drvdata(&dev->dev);
    bl->div_total = read_pwm_reg(bl->id, PWM_REG_LRC);
	if(bl->suspend_status)
		return 0;
    if (bl->pdata->bl_ref) {
	 	DBG(">>>%s-->%d   dev->props.brightness == %d\n",__FUNCTION__,__LINE__,dev->props.brightness);
        bl->divh = bl->div_total*(dev->props.brightness)/BL_STEP;
    } else {
     	 	DBG(">>>%s-->%d   dev->props.brightness == %d\n",__FUNCTION__,__LINE__,dev->props.brightness);
			if(dev->props.brightness < BACKLIGHT_SEE_MINVALUE)/*avoid set black screen then can't operation*/
				dev->props.brightness = BACKLIGHT_SEE_MINVALUE;
        	bl->divh = bl->div_total * (BL_STEP - dev->props.brightness) / BL_STEP;
    }
    write_pwm_reg(bl->id, PWM_REG_HRC, bl->divh);
    DBG(">>>>>> %s : %s  dev->props.brightness==%d\n", __FILE__, __FUNCTION__,dev->props.brightness);
    return 0;
}

static struct backlight_ops rk28_bl_ops = {
	.update_status = rk28_bl_update_status,
	.get_brightness = rk28_bl_get_brightness,
};


static int rk28_bl_change_clk(ip_id ip , int input_clk)
{
    u32 divl, divh, tmp,brightness;
	if (!rk28_bl) {
        printk(KERN_WARNING "%s: backlight device does not exist \n",__func__); 
		return -ENODEV;		
    }
    divl = read_pwm_reg(rk28_bl->id, PWM_REG_LRC);
    divh = read_pwm_reg(rk28_bl->id, PWM_REG_HRC);

    tmp = input_clk/PWM_APB_PRE_DIV;
    tmp >>= (1 + (rk28_bl->pwm_div >> 9));
    
    rk28_bl->div_total = (tmp) ? tmp : 1;
    rk28_bl->divh = rk28_bl->div_total*divh/divl;
    
    write_pwm_reg(rk28_bl->id, PWM_REG_LRC,  rk28_bl->div_total);
    write_pwm_reg(rk28_bl->id, PWM_REG_HRC,  rk28_bl->divh );    
    write_pwm_reg(rk28_bl->id, PWM_REG_CNTR, 0);    
	brightness = rk28_bl_get_brightness(rk28_bl->backlight_dev);
    DBG(">>>>>> %s : %s After change frequecy current brightness==%d\n", __FILE__, __FUNCTION__,brightness);
    return 0;
}   

#ifdef CONFIG_ANDROID_POWER
 void rk28_bl_suspend(android_early_suspend_t *h)
{
	u32 tmp_divh;
	if (!rk28_bl) 
        printk(KERN_WARNING "%s: backlight device does not exist \n",__func__); 
	if(rk28_bl->suspend_status)
		del_timer(&rk28_bl->timer);
    rk28_bl->div_total = read_pwm_reg(rk28_bl->id, PWM_REG_LRC);
    rk28_bl->divh = read_pwm_reg(rk28_bl->id, PWM_REG_HRC);
    if(rk28_bl->pdata->bl_ref) {
        tmp_divh = 0;
    } else {
        tmp_divh = rk28_bl->div_total;
    }
    write_pwm_reg(rk28_bl->id, PWM_REG_HRC, tmp_divh);
	rk28_bl->suspend_status = 1;
    DBG(">>>>>> %s : %s  current brightness==%d  current_intensity==%d\n", __FILE__, __FUNCTION__,rk28_bl->backlight_dev->props.brightness,rk28_bl->current_intensity);
	DBG("%s--->%s--->suspend_status==%d\n",__FILE__,__FUNCTION__,rk28_bl->suspend_status);
}

static void rk28_delaybacklight_timer(unsigned long data)
{
	struct rk28_backlight *bl = (struct rk28_backlight *)data;
    
	DBG(">>>>>> %s : %s brightness==%d current_intensity==%d\n", __FILE__, __FUNCTION__,bl->backlight_dev->props.brightness,bl->current_intensity);
    bl->div_total = read_pwm_reg(bl->id, PWM_REG_LRC);
	bl->current_intensity = bl->backlight_dev->props.brightness;
	if(bl->current_intensity < BACKLIGHT_SEE_MINVALUE)/*avoid set black screen then can't operation*/
		bl->current_intensity = BACKLIGHT_SEE_MINVALUE;
    if (bl->pdata->bl_ref) {
        bl->divh = bl->div_total*(bl->current_intensity)/BL_STEP;
    } else {
        bl->divh = bl->div_total*(BL_STEP - bl->current_intensity)/BL_STEP;
    }
    write_pwm_reg(bl->id, PWM_REG_HRC, bl->divh);
	bl->suspend_status = 0;
	DBG("%s--->%s--->suspend_status==%d\n",__FILE__,__FUNCTION__,bl->suspend_status);
}

 void rk28_bl_resume(android_early_suspend_t *h)
{
#ifdef CONFIG_ANX7150
	if(anx7150_get_output_status() == HDMI)
		return;
#endif

	del_timer(&rk28_bl->timer);
	rk28_bl->timer.expires  = jiffies + 30;
	add_timer(&rk28_bl->timer);
	DBG(">>>>>> %s : %s brightness==%d\n", __FILE__, __FUNCTION__,rk28_bl->current_intensity);
}

static android_early_suspend_t bl_early_suspend;
#endif
 
static char *pwm_iomux[] = {
     GPIOF2_APWM0_SEL_NAME,
     GPIOF3_APWM1_MMC0DETN_NAME,
     GPIOF4_APWM2_MMC0WPT_NAME,
     GPIOF5_APWM3_DPWM3_NAME,
};

static int rk28_backlight_probe(struct platform_device *pdev)
{		
    struct rk28_backlight *bl;
	struct backlight_device *dev;
    struct rk28bl_info *pdata = pdev->dev.platform_data;
	u32 ret;
	if (!pdata)
		return -ENXIO;
	bl = kzalloc(sizeof(struct rk28_backlight), GFP_KERNEL);
	if (unlikely(!bl))
		return -ENOMEM;
	dev = backlight_device_register("rk28_bl", &pdev->dev, bl, &rk28_bl_ops);
	if (IS_ERR(dev)) {
		kfree(bl);
		printk(KERN_WARNING "%s: backlight device register error\n", __func__); 
		return PTR_ERR(dev);
	}
	
    bl->pin =  (pdata->pw_pin >> 8) & 0xff;
    bl->lev =  pdata->pw_pin & 0xf;
    bl->id  =  pdata->pwm_id;
	bl->pwm_div = pdata->pwm_div;
	bl->pdata = pdata;
	bl->dev = &pdev->dev;
	bl->backlight_dev = dev;
	platform_set_drvdata(pdev, dev);
	
    bl->div_total = rockchip_clk_get_apb()/PWM_APB_PRE_DIV;
    bl->div_total >>= (1 + (bl->pwm_div >> 9));
    bl->div_total = (bl->div_total) ? bl->div_total : 1;
    if(bl->pdata->bl_ref) {
        bl->divh = 0;
    } else {
        bl->divh = bl->div_total / 2;
    }

    /*init timer to dispose workqueue */
    setup_timer(&bl->timer, rk28_delaybacklight_timer, (unsigned long)bl);

    write_pwm_reg(bl->id, PWM_REG_CTRL, bl->pwm_div|PWM_RESET);
    write_pwm_reg(bl->id, PWM_REG_LRC, bl->div_total);
    write_pwm_reg(bl->id, PWM_REG_HRC, bl->divh);
    write_pwm_reg(bl->id, PWM_REG_CNTR, 0x0);
    write_pwm_reg(bl->id, PWM_REG_CTRL, bl->pwm_div|PWM_ENABLE|PWM_TIME_EN);
    ret = rockchip_scu_apbunit_register(SCU_IPID_PWM, "pwm0", rk28_bl_change_clk);
    if (ret < 0) {
        printk(KERN_WARNING "%s: scu apbunit register error\n",__func__); 
        backlight_device_unregister(dev);
        return ret;
    }
	dev->props.power = FB_BLANK_UNBLANK;
	dev->props.fb_blank = FB_BLANK_UNBLANK;
	dev->props.max_brightness = BL_STEP;
	dev->props.brightness = pdata->default_intensity;

#ifdef CONFIG_ANDROID_POWER
    bl_early_suspend.suspend = rk28_bl_suspend;
    bl_early_suspend.resume = rk28_bl_resume;
    bl_early_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    android_register_early_suspend(&bl_early_suspend);
#endif

    rockchip_mux_api_set(pwm_iomux[bl->id], 1);
    gpio_direction_output(bl->pin, 0);
    GPIOSetPinLevel(bl->pin, bl->lev);
    rk28_bl_update_status(dev);
	rk28_bl = bl;
	return 0;
}

static int rk28_backlight_remove(struct platform_device *pdev)
{		
	if (rk28_bl) {
		backlight_device_unregister(rk28_bl->backlight_dev);
#ifdef CONFIG_ANDROID_POWER
       android_unregister_early_suspend(&bl_early_suspend);
#endif        
        return 0;
    } else {
        rk28printk(KERN_CRIT "%s: no backlight device has registered\n", __func__); 
        return -ENODEV;      
    }
}
static void rk28_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct rk28_backlight *bl = dev_get_drvdata(&dev->dev);
	bl->current_intensity = dev->props.brightness; 
	bl->current_intensity/=2;
	bl->div_total = read_pwm_reg(bl->id, PWM_REG_LRC);   
	if (bl->pdata->bl_ref) {
			bl->divh = bl->div_total*(bl->current_intensity)/BL_STEP;
	} else {
			bl->divh = bl->div_total*(BL_STEP-bl->current_intensity)/BL_STEP;
	}
	write_pwm_reg(bl->id, PWM_REG_HRC, bl->divh);
	mdelay(10);
	bl->current_intensity/=2;
	if (bl->pdata->bl_ref) {
		bl->divh = bl->div_total*(bl->current_intensity)/BL_STEP;
	} else {
		bl->divh = bl->div_total*(BL_STEP-bl->current_intensity)/BL_STEP;
	}
	write_pwm_reg(bl->id, PWM_REG_HRC, bl->divh); 
    mdelay(10);
	/*set  PF1=1 PF2=1 for close backlight*/	
	rockchip_mux_api_set(LED_CON_IOMUX_PINNAME, LED_CON_IOMUX_PINDIR);
	GPIOSetPinDirection(LED_CON_IOPIN,GPIO_OUT);
	GPIOSetPinLevel(LED_CON_IOPIN,GPIO_HIGH);
	mdelay(10);

}

static struct platform_driver rk28_backlight_driver = {
	.probe	= rk28_backlight_probe,
	.remove = rk28_backlight_remove,
	.driver	= {
		.name	= "rk28_backlight",
		.owner	= THIS_MODULE,
	},
	.shutdown=rk28_backlight_shutdown,
};


static int __init rk28_backlight_init(void)
{
	rk28printk("%s::========================================\n",__func__);
	platform_driver_register(&rk28_backlight_driver);
	return 0;
}
//#ifndef CONFIG_LCD_RK_EINK
rootfs_initcall(rk28_backlight_init);
//#endif
//late_initcall(rk28_backlight_init);
//module_init(rk28_backlight_init);
