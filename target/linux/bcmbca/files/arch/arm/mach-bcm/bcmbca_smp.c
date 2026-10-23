// SPDX-License-Identifier: GPL-2.0-only
/*
 * SMP support for BCMBCA Cortex-A7 SoCs (BCM47622 / BCM6755 and similar)
 * booted by Broadcom CFE without PSCI firmware.
 *
 * CFE keeps the secondary cores in a WFI loop. Writing the physical entry
 * address to the "cpu-release-addr" register (GPIO GeneralPurpose register
 * on BCM47622) and sending a wakeup SGI makes the core jump to that address.
 *
 * Based on arch/arm/mach-versatile/platsmp.c
 */

#include <linux/cache.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp.h>
#include <asm/smp_plat.h>

#include "bcmbca_smp.h"

volatile int bcmbca_cpu_release __cacheline_aligned = -1;

static void bcmbca_write_cpu_release(int val)
{
	bcmbca_cpu_release = val;
	smp_wmb();
	sync_cache_w(&bcmbca_cpu_release);
}

static void bcmbca_smp_secondary_init(unsigned int cpu)
{
	/* Let the primary processor know we're out of the pen */
	bcmbca_write_cpu_release(-1);
}

static int bcmbca_smp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	bcmbca_write_cpu_release(cpu_logical_map(cpu));

	/* Wake the core up from the CFE WFI loop */
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (bcmbca_cpu_release == -1)
			break;

		udelay(10);
	}

	if (bcmbca_cpu_release != -1) {
		bcmbca_write_cpu_release(-1);
		return -ENOSYS;
	}

	return 0;
}

static void __init bcmbca_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *np = NULL;
	void __iomem *release;
	u32 addr = 0;

	for_each_of_cpu_node(np) {
		if (!of_property_read_u32(np, "cpu-release-addr", &addr)) {
			of_node_put(np);
			break;
		}
	}

	if (!addr) {
		pr_err("bcmbca-smp: no cpu-release-addr, secondary CPUs won't start\n");
		return;
	}

	release = ioremap(addr, sizeof(u32));
	if (!release) {
		pr_err("bcmbca-smp: failed to map cpu-release-addr\n");
		return;
	}

	writel(__pa_symbol(bcmbca_secondary_startup), release);
	iounmap(release);
}

static const struct smp_operations bcmbca_smp_ops __initconst = {
	.smp_prepare_cpus	= bcmbca_smp_prepare_cpus,
	.smp_secondary_init	= bcmbca_smp_secondary_init,
	.smp_boot_secondary	= bcmbca_smp_boot_secondary,
};

CPU_METHOD_OF_DECLARE(bcmbca_smp, "brcm,bca-smp", &bcmbca_smp_ops);
