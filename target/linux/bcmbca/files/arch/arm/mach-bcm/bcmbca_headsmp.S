/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Holding pen for BCMBCA Cortex-A7 SoCs booted by Broadcom CFE.
 *
 * CFE parks the secondary cores in WFI. Once woken up by an SGI they jump
 * to the address stored in the GPIO "GeneralPurpose" register. Each core
 * then waits here until bcmbca_cpu_release matches its MPIDR.
 *
 * Based on arch/arm/mach-versatile/headsmp.S
 */
#include <linux/linkage.h>
#include <asm/assembler.h>

ENTRY(bcmbca_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5
	bic	r0, #0xff000000
	adr_l	r6, bcmbca_cpu_release		@ PC relative, the MMU is off
pen:	ldr	r7, [r6]
	cmp	r7, r0
	bne	pen

	b	secondary_startup
ENDPROC(bcmbca_secondary_startup)
