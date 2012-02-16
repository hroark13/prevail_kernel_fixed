/*
 *  linux/arch/arm/kernel/irq.c
 *
 *  Copyright (C) 1992 Linus Torvalds
 *  Modifications for ARM processor Copyright (C) 1995-2000 Russell King.
 *
 *  Support for Dynamic Tick Timer Copyright (C) 2004-2005 Nokia Corporation.
 *  Dynamic Tick Timer written by Tony Lindgren <tony@atomide.com> and
 *  Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This file contains the code used by various IRQ handling routines:
 *  asking for different IRQ's should be done through these routines
 *  instead of just grabbing them. Thus setups with different IRQ numbers
 *  shouldn't result in any weird surprises, and installing new handlers
 *  should be easier.
 *
 *  IRQ's are in fact implemented a bit like signal handlers for the kernel.
 *  Naturally it's not a 1:1 relation, but there are similarities.
 */
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/random.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/kallsyms.h>
#include <linux/proc_fs.h>

#include <asm/system.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <asm/perftypes.h>

/*
 * Recorder for GAF v3.0
 */

#include <linux/gaf_rec.h>

struct GAF_RECORD_0 gaf_record_0 = {
	.version = 0x1,
	.lows = NUM_OF_LOWS_0,
	.columns = NUM_OF_COLUMNS_0,
	.column_title = {"time%t", "pid%p", "r0%h", "r1%h", "r2%h", "r3%h", "r4%h", "r5%h", "r6%h", "r7%h", "r8%h", "r9%h", "r10%h", "r11%h", "r12%h", "r13%h", "r14%h", "r15%h", "org_r0%h", "over%h", "__T_T_TR%h", "FN%a", "LR%a", "PC%a", }
};

inline void REC_do_IRQ(unsigned int irq, struct pt_regs *old_regs)
{
	unsigned int cpu;
	unsigned int q_index;
	struct irq_desc *p_irq_desc;
	struct REC_RECORD *p_record;
	cpu = smp_processor_id();
	q_index = gaf_record_0.save_index[cpu];

	if(old_regs) {
		memcpy (gaf_record_0.column[cpu][q_index].data, old_regs, sizeof(struct pt_regs));
	} else {
		gaf_record_0.column[cpu][q_index].data[0] = 0xf0f0f0f0;
	}

	gaf_record_0.column[cpu][q_index].time = cpu_clock(cpu);
	gaf_record_0.column[cpu][q_index].pid = current->pid;
	p_irq_desc = irq_to_desc(irq);

	if(p_irq_desc) {
		if(p_irq_desc->action) {
			gaf_record_0.column[cpu][q_index].data[19] = (void*) p_irq_desc->action->handler;
		} else {
			gaf_record_0.column[cpu][q_index].data[19] = 0x0;
		}
	}

	gaf_record_0.column[cpu][q_index].data[20] = gaf_record_0.column[cpu][q_index].data[14];
	gaf_record_0.column[cpu][q_index].data[21] = gaf_record_0.column[cpu][q_index].data[15];

	gaf_record_0.save_index[cpu]=(++q_index)&(NUM_OF_LOWS_0-1);
	return;
}


/*
 * No architecture-specific irq_finish function defined in arm/arch/irqs.h.
 */
#ifndef irq_finish
#define irq_finish(irq) do { } while (0)
#endif

void (*init_arch_irq)(void) __initdata = NULL;
unsigned long irq_err_count;

int show_interrupts(struct seq_file *p, void *v)
{
	int i = *(loff_t *) v, cpu;
	struct irqaction * action;
	unsigned long flags;

	if (i == 0) {
		char cpuname[12];

		seq_printf(p, "    ");
		for_each_present_cpu(cpu) {
			sprintf(cpuname, "CPU%d", cpu);
			seq_printf(p, " %10s", cpuname);
		}
		seq_putc(p, '\n');
	}

	if (i < NR_IRQS) {
		raw_spin_lock_irqsave(&irq_desc[i].lock, flags);
		action = irq_desc[i].action;
		if (!action)
			goto unlock;

		seq_printf(p, "%3d: ", i);
		for_each_present_cpu(cpu)
			seq_printf(p, "%10u ", kstat_irqs_cpu(i, cpu));
		seq_printf(p, " %10s", irq_desc[i].chip->name ? : "-");
		seq_printf(p, "  %s", action->name);
		for (action = action->next; action; action = action->next)
			seq_printf(p, ", %s", action->name);

		seq_putc(p, '\n');
unlock:
		raw_spin_unlock_irqrestore(&irq_desc[i].lock, flags);
	} else if (i == NR_IRQS) {
#ifdef CONFIG_FIQ
		show_fiq_list(p, v);
#endif
#ifdef CONFIG_SMP
		show_ipi_list(p);
		show_local_irqs(p);
#endif
		seq_printf(p, "Err: %10lu\n", irq_err_count);
	}
	return 0;
}

/*
 * do_IRQ handles all hardware IRQ's.  Decoded IRQs should not
 * come via this function.  Instead, they should provide their
 * own 'handler'
 */
asmlinkage void __exception asm_do_IRQ(unsigned int irq, struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);

	perf_mon_interrupt_in();
/* 
 * Recorder 1.0 for GAFv3.0
 */

	//{{ ss.sec
	/* RECORD read the information about interrupt ans store it. */
	REC_do_IRQ(irq, old_regs);
	//}}

	irq_enter();

	/*
	 * Some hardware gives randomly wrong interrupts.  Rather
	 * than crashing, do something sensible.
	 */
	if (unlikely(irq >= NR_IRQS)) {
		if (printk_ratelimit())
			printk(KERN_WARNING "Bad IRQ%u\n", irq);
		ack_bad_irq(irq);
	} else {
		generic_handle_irq(irq);
	}

	/* AT91 specific workaround */
	irq_finish(irq);

	irq_exit();
	set_irq_regs(old_regs);
	perf_mon_interrupt_out();
}

void set_irq_flags(unsigned int irq, unsigned int iflags)
{
	struct irq_desc *desc;
	unsigned long flags;

	if (irq >= NR_IRQS) {
		printk(KERN_ERR "Trying to set irq flags for IRQ%d\n", irq);
		return;
	}

	desc = irq_desc + irq;
	raw_spin_lock_irqsave(&desc->lock, flags);
	desc->status |= IRQ_NOREQUEST | IRQ_NOPROBE | IRQ_NOAUTOEN;
	if (iflags & IRQF_VALID)
		desc->status &= ~IRQ_NOREQUEST;
	if (iflags & IRQF_PROBE)
		desc->status &= ~IRQ_NOPROBE;
	if (!(iflags & IRQF_NOAUTOEN))
		desc->status &= ~IRQ_NOAUTOEN;
	raw_spin_unlock_irqrestore(&desc->lock, flags);
}

void __init init_IRQ(void)
{
	int irq;

	for (irq = 0; irq < NR_IRQS; irq++)
		irq_desc[irq].status |= IRQ_NOREQUEST | IRQ_NOPROBE;

	init_arch_irq();
}

#ifdef CONFIG_HOTPLUG_CPU

static void route_irq(struct irq_desc *desc, unsigned int irq, unsigned int cpu)
{
	pr_debug("IRQ%u: moving from cpu%u to cpu%u\n", irq, desc->node, cpu);

	raw_spin_lock_irq(&desc->lock);
	desc->chip->set_affinity(irq, cpumask_of(cpu));
	raw_spin_unlock_irq(&desc->lock);
}

/*
 * The CPU has been marked offline.  Migrate IRQs off this CPU.  If
 * the affinity settings do not allow other CPUs, force them onto any
 * available CPU.
 */
void migrate_irqs(void)
{
	unsigned int i, cpu = smp_processor_id();

	for (i = 0; i < NR_IRQS; i++) {
		struct irq_desc *desc = irq_desc + i;

		if (desc->node == cpu) {
			unsigned int newcpu = cpumask_any_and(desc->affinity,
							      cpu_online_mask);
			if (newcpu >= nr_cpu_ids) {
				if (printk_ratelimit())
					printk(KERN_INFO "IRQ%u no longer affine to CPU%u\n",
					       i, cpu);

				cpumask_setall(desc->affinity);
				newcpu = cpumask_any_and(desc->affinity,
							 cpu_online_mask);
			}

			route_irq(desc, i, newcpu);
		}
	}
}
#endif /* CONFIG_HOTPLUG_CPU */
