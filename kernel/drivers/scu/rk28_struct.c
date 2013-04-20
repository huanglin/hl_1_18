/* 20100324,HSL@RK,
 * for get kernel struct size and element offset!.
 * 
*/
#ifdef __KERNEL__
#include <linux/module.h>
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/binfmts.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/tty.h>
#include <linux/console.h>
#include <linux/tty_flip.h>  
#include <linux/mm.h>  
#include <linux/file.h>  
#include <linux/dma-mapping.h>
#include <linux/seq_file.h> 

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include <asm/traps.h>
#include <asm/cacheflush.h>

#include<asm/types.h>
#include<asm/ptrace.h>
#include<asm/irq_regs.h>

#include<asm/arch/gpio.h>
#include<asm/arch/rk28_debug.h>
#include<asm/arch/rk28_msc_ext.h>

/*
 * entirely copy from sched.c.
 */
struct task_group_cpy {
#ifdef CONFIG_CGROUP_SCHED
	struct cgroup_subsys_state css;
#endif

#ifdef CONFIG_FAIR_GROUP_SCHED
	/* schedulable entities of this group on each cpu */
	struct sched_entity **se;
	/* runqueue "owned" by this group on each cpu */
	struct cfs_rq **cfs_rq;
	unsigned long shares;
#endif

#ifdef CONFIG_RT_GROUP_SCHED
	struct sched_rt_entity **rt_se;
	struct rt_rq **rt_rq;

	u64 rt_runtime;
#endif

	struct rcu_head rcu;
	struct list_head list;
};
struct cfs_rq_cpy {
	struct load_weight load;
	unsigned long nr_running;

	u64 exec_clock;
	u64 min_vruntime;

	struct rb_root tasks_timeline;
	struct rb_node *rb_leftmost;
	struct rb_node *rb_load_balance_curr;
	/* 'curr' points to currently running entity on this cfs_rq.
	 * It is set to NULL otherwise (i.e when none are currently running).
	 */
	struct sched_entity *curr, *next;

	unsigned long nr_spread_over;

#ifdef CONFIG_FAIR_GROUP_SCHED
	struct rq *rq;	/* cpu runqueue to which this cfs_rq is attached */

	/*
	 * leaf cfs_rqs are those that hold tasks (lowest schedulable entity in
	 * a hierarchy). Non-leaf lrqs hold other higher schedulable entities
	 * (like users, containers etc.)
	 *
	 * leaf_cfs_rq_list ties together list of leaf cfs_rq's in a cpu. This
	 * list is used during load balance.
	 */
	struct list_head leaf_cfs_rq_list;
	struct task_group *tg;	/* group that "owns" this runqueue */
#endif
};
struct rt_prio_array_cpy {
	DECLARE_BITMAP(bitmap, MAX_RT_PRIO+1); /* include 1 bit for delimiter */
	struct list_head queue[MAX_RT_PRIO];
};

struct rt_rq_cpy {
	struct rt_prio_array_cpy active;
	unsigned long rt_nr_running;
#if defined CONFIG_SMP || defined CONFIG_RT_GROUP_SCHED
	int highest_prio; /* highest queued rt task prio */
#endif
#ifdef CONFIG_SMP
	unsigned long rt_nr_migratory;
	int overloaded;
#endif
	int rt_throttled;
	u64 rt_time;

#ifdef CONFIG_RT_GROUP_SCHED
	unsigned long rt_nr_boosted;

	struct rq *rq;
	struct list_head leaf_rt_rq_list;
	struct task_group *tg;
	struct sched_rt_entity *rt_se;
#endif
};

struct rq_cpy {
	/* runqueue lock: */
	spinlock_t lock;

	/*
	 * nr_running and cpu_load should be in the same cacheline because
	 * remote CPUs use both these fields when doing load calculation.
	 */
	unsigned long nr_running;
	#define CPU_LOAD_IDX_MAX 5
	unsigned long cpu_load[CPU_LOAD_IDX_MAX];
	unsigned char idle_at_tick;
#ifdef CONFIG_NO_HZ
	unsigned char in_nohz_recently;
#endif
	/* capture load from *all* tasks on this cpu: */
	struct load_weight load;
	unsigned long nr_load_updates;
	u64 nr_switches;

	struct cfs_rq_cpy cfs;
	struct rt_rq_cpy rt;
	u64 rt_period_expire;
	int rt_throttled;

#ifdef CONFIG_FAIR_GROUP_SCHED
	/* list of leaf cfs_rq on this cpu: */
	struct list_head leaf_cfs_rq_list;
#endif
#ifdef CONFIG_RT_GROUP_SCHED
	struct list_head leaf_rt_rq_list;
#endif

	/*
	 * This is part of a global counter where only the total sum
	 * over all CPUs matters. A task can increase this counter on
	 * one CPU and if it got migrated afterwards it may decrease
	 * it on another CPU. Always updated under the runqueue lock:
	 */
	unsigned long nr_uninterruptible;

	struct task_struct *curr, *idle;
	unsigned long next_balance;
	struct mm_struct *prev_mm;

	u64 clock, prev_clock_raw;
	s64 clock_max_delta;

	unsigned int clock_warps, clock_overflows, clock_underflows;
	u64 idle_clock;
	unsigned int clock_deep_idle_events;
	u64 tick_timestamp;

	atomic_t nr_iowait;

#ifdef CONFIG_SMP
	struct root_domain *rd;
	struct sched_domain *sd;

	/* For active balancing */
	int active_balance;
	int push_cpu;
	/* cpu of this runqueue: */
	int cpu;

	struct task_struct *migration_thread;
	struct list_head migration_queue;
#endif

#ifdef CONFIG_SCHED_HRTICK
	unsigned long hrtick_flags;
	ktime_t hrtick_expire;
	struct hrtimer hrtick_timer;
#endif

#ifdef CONFIG_SCHEDSTATS
	/* latency stats */
	struct sched_info rq_sched_info;

	/* sys_sched_yield() stats */
	unsigned int yld_exp_empty;
	unsigned int yld_act_empty;
	unsigned int yld_both_empty;
	unsigned int yld_count;

	/* schedule() stats */
	unsigned int sched_switch;
	unsigned int sched_count;
	unsigned int sched_goidle;

	/* try_to_wake_up() stats */
	unsigned int ttwu_count;
	unsigned int ttwu_local;

	/* BKL stats */
	unsigned int bkl_count;
#endif
	struct lock_class_key rq_lock_key;
};

/*
 * 20100324,HSL@RK,for get data size,offset at kernel.
*/
struct kernel_struct {
        char    name[60];       /* struct name  or element name */
        int       size;
};

struct kernel_offset {
        char    name[80];       /* struct name  or element name */
        int       size;
};

#define offset_of(s,e)  __compiler_offsetof( s, e )
#define STRUCT( s )     { #s, sizeof( s ) }
#define OFFSET( s , e )     {#s "." #e, offset_of(s,e) }
/*static__init const*/ struct kernel_struct __rkusb_kernel_struct_info[] = {
        STRUCT(struct kernel_struct),   // 0
        STRUCT(struct task_struct),     // 0x2e0
        STRUCT(struct sched_entity),
        STRUCT(struct sched_class),     
        STRUCT(struct sched_rt_entity),
        STRUCT(struct cfs_rq_cpy),
        STRUCT(struct rt_rq_cpy),
        STRUCT(struct rq_cpy),
        
        STRUCT(struct file),                    // 0x118
        STRUCT(struct mm_struct),       // 0x15c
        STRUCT(struct vm_area_struct), // 0x54
        STRUCT(struct linux_binfmt),
        STRUCT(struct user_struct),
        STRUCT(struct thread_struct ),
        STRUCT(struct path),
        STRUCT(struct fs_struct),       // 10
        STRUCT(struct fdtable),
        STRUCT(struct files_struct),
        STRUCT(struct rb_node),         
        STRUCT(struct seq_file),           // 0x38 ,14
        STRUCT(struct dentry),
        
};

/*static __init const*/ struct kernel_offset __rkusb_kernel_element_info[] = {
        OFFSET(struct kernel_offset ,size ), // 0
        OFFSET(struct task_struct,tasks),
        OFFSET(struct task_struct,se),
        OFFSET(struct task_struct,mm),
        OFFSET(struct task_struct,parent),
        OFFSET(struct task_struct,user),
        OFFSET(struct task_struct,thread),
        OFFSET(struct task_struct,bio_list),
        
        OFFSET(struct mm_struct,pgd),           // 8
        OFFSET(struct vm_area_struct,vm_ops),

        OFFSET(struct seq_file,lock), // 0x20 
        
        OFFSET(struct file,private_data),
        OFFSET(struct dentry,d_name),
};

#endif /* __kernel */
