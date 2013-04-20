/*
 *  linux/arch/arm/mach-integrator/cpu.c
 *
 *  Copyright (C) 2001-2002 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * CPU support functions
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/io.h>

/* ************
 *	  DEBUG
 **************/

#define SHOWME  "CPUFREQ"
#define S_LEVEL  S_L_WARN
#include <asm/arch/rk28_debug.h>
#include <asm/arch/rk28_scu.h>

static struct cpufreq_driver rk2818_cpufreq_driver;

#if defined(CPUFREQ_CHANGFRQ_TABLE)
static struct cpufreq_frequency_table rk2818_freq_table[] = {	
                { 0, 630000 },
	{ 1, 540000 },	
	{ 2, 270000 },	
	{ 3, 180000 },	
	{ 4, 135000 },	
	{ 0, CPUFREQ_TABLE_END },
};
#endif
unsigned int find_freq_table_target(struct cpufreq_policy *policy,unsigned int  target_freq, unsigned int relation)
{
	int i;
	for (i=0; (rk2818_freq_table[i].frequency != CPUFREQ_TABLE_END); i++) {
		if(target_freq == rk2818_freq_table[i].frequency)		
			break;
	}
	if(rk2818_freq_table[i].frequency == CPUFREQ_TABLE_END)
	{
		printk("invail freq value!!");
		return target_freq;
	}
	switch(relation) {
		case CPUFREQ_RELATION_H:
			if(policy->max == rk2818_freq_table[i].frequency)
				break;	/*already highest freq needn't increase*/
			i --;
			break;
		case CPUFREQ_RELATION_L:
			if(policy->min == rk2818_freq_table[i].frequency)
				break;	/*already lowest freq needn't decrease*/
			i ++;
			break;
	}
	S_INFO("get next target freq is (%u kHz, %u)\n",  rk2818_freq_table[i].frequency,rk2818_freq_table[i].index);
	return rk2818_freq_table[i].frequency;
}

/*
 * Validate the speed policy.
 */
static int rk2818_verify_policy(struct cpufreq_policy *policy)
{
	
	S_INFO("%s->%s->line =%d \n" , __FILE__,__FUNCTION__,__LINE__);
	cpufreq_verify_within_limits(policy, 
				     policy->cpuinfo.min_freq, 
				     policy->cpuinfo.max_freq);

	policy->cur = rockchip_clk_get_arm()/1000;

	cpufreq_verify_within_limits(policy, 
				     policy->cpuinfo.min_freq, 
				     policy->cpuinfo.max_freq);
	S_INFO("%s-> policy->cur =%d \n",__FUNCTION__,policy->cur);
	return 0;
}


static int rk2818_set_target(struct cpufreq_policy *policy,
				 unsigned int target_freq,
				 unsigned int relation)
{
	cpumask_t cpus_allowed;
	int cpu = policy->cpu;
	struct cpufreq_freqs freqs;
	S_INFO("%s->%s->line =%d \n" , __FILE__,__FUNCTION__,__LINE__);
	/*
	 * Save this threads cpus_allowed mask.
	 */
	cpus_allowed = current->cpus_allowed;

	/*
	 * Bind to the specified CPU.  When this call returns,
	 * we should be running on the right CPU.
	 */
	set_cpus_allowed(current, cpumask_of_cpu(cpu));
	BUG_ON(cpu != smp_processor_id());
	freqs.old = rockchip_clk_get_arm() /1000;
	freqs.cpu = policy->cpu;
	freqs.new = find_freq_table_target(policy,target_freq, relation);
	if(freqs.new < 0){
		set_cpus_allowed(current, cpus_allowed);
		return 0;
	}
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	rockchip_clk_set_arm(freqs.new/1000);
	/*
	 * Restore the CPUs allowed mask.
	 */
	set_cpus_allowed(current, cpus_allowed);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static unsigned int rk2818_get(unsigned int cpu)
{
	unsigned int current_freq;
	cpumask_t cpus_allowed;
	S_INFO("%s->%s->line =%d \n" , __FILE__,__FUNCTION__,__LINE__);
	cpus_allowed = current->cpus_allowed;
	set_cpus_allowed(current, cpumask_of_cpu(cpu));
	current_freq = rockchip_clk_get_arm()/1000; /* current freq */
	S_INFO("%s->%s->line =%d ->cpu = %d current_freq=%d\n" , __FILE__,__FUNCTION__,__LINE__,cpu,current_freq);
	set_cpus_allowed(current, cpus_allowed);
	return current_freq;
}

static int rk2818_cpufreq_init(struct cpufreq_policy *policy)
{
	S_INFO("%s->%s->line =%d \n" , __FILE__,__FUNCTION__,__LINE__);
	/* set default policy and cpuinfo */
	policy->cpuinfo.max_freq = policy->max = 630000;
	policy->cpuinfo.min_freq = policy->min =  135000;
	policy->cpuinfo.transition_latency = 20*1000*1000; /* 10s */
	policy->cur  =  rk2818_get(policy->cpu);

	return 0;
}

static struct cpufreq_driver rk2818_cpufreq_driver = {
	.flags          = CPUFREQ_STICKY,
	.verify		= rk2818_verify_policy,
	.target		= rk2818_set_target,
	.get		= rk2818_get,
	.init		= rk2818_cpufreq_init,
	.name		= "rk2818_cpufreq",
};

static int __init rk2818_cpu_init(void)
{
	S_INFO("%s->%s->line =%d \n" , __FILE__,__FUNCTION__,__LINE__);
	return cpufreq_register_driver(&rk2818_cpufreq_driver);
}

//static void __exit rk2818_cpu_exit(void)
//{
//	cpufreq_unregister_driver(&rk2818_driver);
//}

MODULE_AUTHOR ("WQQ");
MODULE_DESCRIPTION ("cpufreq driver for ARM rockchip CPUs");
MODULE_LICENSE ("GPL");

module_init(rk2818_cpu_init);
//module_exit(rk2818_cpu_exit);
