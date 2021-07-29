/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 * Common Header for WOKOO machines
 */

#ifndef __ARCH_ARM_MACH_WOKOO_COMMON_H
#define __ARCH_ARM_MACH_WOKOO_COMMON_H

#ifdef CONFIG_PM_SLEEP
void wokoo_cpu_resume(void);
void wokoo_cpu_suspend(void __iomem *ocram_vbase);
void wokoo_pm_init(void);
#else
static inline void wokoo_pm_init(void) {}
void wokoo_cpu_suspend(void __iomem *ocram_vbase) {}
#endif

#endif /* __ARCH_ARM_MACH_WOKOO_COMMON_H */
