/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __BCMBCA_SMP_H
#define __BCMBCA_SMP_H

extern volatile int bcmbca_cpu_release;

void bcmbca_secondary_startup(void);

#endif /* __BCMBCA_SMP_H */
