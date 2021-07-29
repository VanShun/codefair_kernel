/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */ 
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/irqchip.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include <asm/mach-types.h>

#include "common.h"

#if 0
static int __init wokoo_fdt_map_sys(unsigned long node, const char *uname,
					int depth, void *data)
{
	return 1;
}

/* Not register for earlyprintk */
static void __init wokoo_dt_map_io(void)
{
}
#endif

static void __init wokoo_init_irq(void)
{
	irqchip_init();
}

static void __init wokoo_dt_init_late(void)
{
	platform_device_register_simple("wokoo-cpufreq", -1, NULL, 0);
	wokoo_pm_init();
}

static char const *const wokoo_dt_compat[] __initconst = {
	"codefair,wokoo",
	NULL
};

DT_MACHINE_START(WOKOO_DT, "CodeFair WOKOO-based board")
	.dt_compat = wokoo_dt_compat,
	.init_irq	= wokoo_init_irq,
//	.map_io = wokoo_dt_map_io,
	.init_late = wokoo_dt_init_late,
MACHINE_END
