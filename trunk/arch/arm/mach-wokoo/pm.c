/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 * WOKOO - Power Management support
 *
 * Based on arch/arm/mach-wokoo/pm.c
 */ 

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <asm/fncpy.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>

#include "common.h"




#define WOKOO_SLPCTL0            0x000		/* Sleep control register 0 */
#define WOKOO_SLPST              0x00c		/* Sleep start register */
#define WOKOO_INT_A7_MK0         0x1e4		/* Cortex-A7 interrupt input mask register 0 */
#define WOKOO_INT_A7_MK1         0x1e8		/* Cortex-A7 interrupt input mask register 1 */
#define WOKOO_INT_A7_EN          0x1f8		/* Cortex-A7 interrupt enable register */
#define WOKOO_LP_CTL             0x2e4		/* Low power control register */

#define WOKOO_INT_A7_MK0_RTC     BIT(5)
#define WOKOO_INT_A7_MK0_GPIO    BIT(16)

#define WOKOO_SLP_RAM_MODE_SHIT  16
#define WOKOO_SLP_RAM_MODE_MASK  0x3
#define WOKOO_SLP_RAM_MODE_01    0x1
#define WOKOO_SLP_RAM_MODE_02    0x2

#define WOKOO_SLP_DDR_AXI_SHIT   10

#define WOKOO_SUSPEND_OCRAM_SIZE 0x1000		/* SRAM size for suspend code */

/* These enum are option of low power mode */
enum {
	WOKOO_ARM_OFF_LOGIC_NORMAL = 0,
	WOKOO_ARM_OFF_LOGIC_DEEP = 1,
};

struct wokoo_pm_base {
	phys_addr_t pbase;
	void __iomem *vbase;
};


struct wokoo_cpu_pm_info {
	phys_addr_t pbase; /* The physical address of pm_info. */
	phys_addr_t resume_addr; /* The physical resume address for asm code */
	u32 ddr_type;
	u32 pm_info_size; /* Size of pm_info. */
	struct wokoo_pm_base mem_pwr_base;
	struct wokoo_pm_base mem_ctl_base;
	struct wokoo_pm_base mem_phy_base;
	struct wokoo_pm_base ap_pwr_base;
} __aligned(8);




static void __iomem *ap_pwr_base;
static void __iomem *mem_ctl_base;
static void __iomem *mem_phy_base;
static void __iomem *mem_pwr_base;
static void __iomem *suspend_ocram_base;
static void (*wokoo_suspend_in_ocram_fn)(void __iomem *ocram_vbase);




static void wokoo_slp_mode_set_resume(void)
{
}

static int wokoo_lpmode_enter(unsigned long arg)
{
	if (!wokoo_suspend_in_ocram_fn) {
		cpu_do_idle();
	} else {
		wokoo_suspend_in_ocram_fn(suspend_ocram_base);
	}

	return 1;
}

/*
 * RAM sleep mode
 */
void wokoo_set_int_mem_clk_lpm(int mode)
{
	u32 val = readl_relaxed(ap_pwr_base + WOKOO_LP_CTL);

	val &= ~((WOKOO_SLP_RAM_MODE_MASK << WOKOO_SLP_RAM_MODE_SHIT) & (0x1 << WOKOO_SLP_DDR_AXI_SHIT));
	if (mode)
		val |= (mode << WOKOO_SLP_RAM_MODE_SHIT) | (0x1 << WOKOO_SLP_DDR_AXI_SHIT);
	writel_relaxed(val, ap_pwr_base + WOKOO_LP_CTL);
}

/*
 * Suspend operations.
 */
static int wokoo_suspend_enter(suspend_state_t state)
{
	u32  reg_mk0 = 0;
	u32  reg_mk1 = 0;
	u32  reg_slp = 0;
	u32  reg_int = 0;

	reg_slp = readl(ap_pwr_base + WOKOO_SLPCTL0);
	/* Let cpu and jtag goto suspend */
	writel(0xFFFFFFFF, ap_pwr_base + WOKOO_SLPCTL0);
	reg_int = readl(ap_pwr_base + WOKOO_INT_A7_EN);
	/* Disable A7 interupt */
	writel(0x0, ap_pwr_base + WOKOO_INT_A7_EN);
	reg_mk0 = readl(ap_pwr_base + WOKOO_INT_A7_MK0);
	reg_mk1 = readl(ap_pwr_base + WOKOO_INT_A7_MK1);
	/* Disable modules interupt but gpio and rtc */
	writel(~(WOKOO_INT_A7_MK0_RTC | WOKOO_INT_A7_MK0_GPIO),
		   ap_pwr_base + WOKOO_INT_A7_MK0);
	reg_mk0 = readl(ap_pwr_base + WOKOO_INT_A7_MK0);
	writel(0x0, ap_pwr_base + WOKOO_INT_A7_MK1);

	switch (state) {
	case PM_SUSPEND_STANDBY:
		//wokoo_set_int_mem_clk_lpm(WOKOO_SLP_RAM_MODE_01);
		cpu_do_idle();
		break;

	case PM_SUSPEND_MEM :
		//wokoo_set_int_mem_clk_lpm(WOKOO_SLP_RAM_MODE_02);
		cpu_suspend(0, wokoo_lpmode_enter);
		break;
	}

	/* Restore A7 interupt register  */
	writel(reg_int, ap_pwr_base + WOKOO_INT_A7_EN);
	/* Restore SLPCTL0 register  */
	writel(reg_slp | 0xFFFF0000, ap_pwr_base + WOKOO_SLPCTL0);
	/* Restore modules interupt mask register  */
	writel(reg_mk0, ap_pwr_base + WOKOO_INT_A7_MK0);
	writel(reg_mk1, ap_pwr_base + WOKOO_INT_A7_MK1);
	//wokoo_set_int_mem_clk_lpm(0);
	wokoo_slp_mode_set_resume();

	return 0;
}

static int wokoo_suspend_prepare(void)
{
	return regulator_suspend_prepare(PM_SUSPEND_MEM);
}

static void wokoo_suspend_finish(void)
{
	if (regulator_suspend_finish())
		pr_err("%s: Suspend finish failed\n", __func__);
}

static int wokoo_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY || state == PM_SUSPEND_MEM);
}

static const struct platform_suspend_ops wokoo_suspend_ops = {
	.enter		= wokoo_suspend_enter,
	.prepare	= wokoo_suspend_prepare,
	.finish		= wokoo_suspend_finish,
	.valid		= wokoo_pm_valid,
};


static const struct of_device_id wokoo_clk_of_device_ids[] __initconst = {
	{
		.compatible = "wokoo,wokoo-clks",
	},
	{ /* sentinel */ },
};

static const struct of_device_id wokoo_memctl_of_device_ids[] __initconst = {
	{
		.compatible = "wokoo,wokoo-memctl",
	},
	{ /* sentinel */ },
};

static const struct of_device_id wokoo_memphy_of_device_ids[] __initconst = {
	{
		.compatible = "wokoo,wokoo-memphy",
	},
	{ /* sentinel */ },
};

static const struct of_device_id wokoo_mempwr_of_device_ids[] __initconst = {
	{
		.compatible = "wokoo,wokoo-mempwr",
	},
	{ /* sentinel */ },
};

/*
 * Syscore operations used to delay restore of certain registers.
 */
static void wokoo_pm_resume(void)
{
}

static struct syscore_ops wokoo_pm_syscore_ops = {
	.resume		= wokoo_pm_resume,
};

/*
 * Initialization entry point.
 */
void __init wokoo_pm_init(void)
{
	struct wokoo_cpu_pm_info *pm_info;
	const struct of_device_id *match;
	struct device_node *np;
	struct gen_pool *sram_pool;
	unsigned long sram_base;
	phys_addr_t sram_pbase;
	struct platform_device *pdev;

	/* AP_PWR base */
	np = of_find_matching_node_and_match(NULL, wokoo_clk_of_device_ids,
						&match);
	if (!match) {
		pr_err("Failed to find clks node\n");
		return;
	}

	ap_pwr_base = of_iomap(np, 0);
	of_node_put(np);
	WARN_ON(!ap_pwr_base);

	/* MEMCTL base */
	np = of_find_matching_node_and_match(NULL, wokoo_memctl_of_device_ids,
						&match);
	if (!match) {
		pr_err("Failed to find memctl node\n");
		return;
	}

	mem_ctl_base = of_iomap(np, 0);
	of_node_put(np);

	/* MEMCTL PHY base */
	np = of_find_matching_node_and_match(NULL, wokoo_memphy_of_device_ids,
						&match);
	if (!match) {
		pr_err("Failed to find memphy node\n");
		return;
	}

	mem_phy_base = of_iomap(np, 0);
	of_node_put(np);

	/* DDR_PWR base */
	np = of_find_matching_node_and_match(NULL, wokoo_mempwr_of_device_ids,
						&match);
	if (!match) {
		pr_err("Failed to find mempwr node\n");
		return;
	}

	mem_pwr_base = of_iomap(np, 0);
	of_node_put(np);

	/* SRAM base */
	np = of_find_compatible_node(NULL, NULL, "mmio-sram");
	if (!np) {
		pr_warn("%s: failed to find ocram node!\n", __func__);
		return;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		pr_warn("%s: failed to find sram device!\n", __func__);
		goto put_node;
	}

	/* Set SRAM to ram pool */
	sram_pool = gen_pool_get(&pdev->dev, NULL);
	if (!sram_pool) {
		pr_warn("%s: sram pool unavailable!\n", __func__);
		goto put_node;
	}

	/* Get ram pool for suspend */
	sram_base = gen_pool_alloc(sram_pool, WOKOO_SUSPEND_OCRAM_SIZE);
	if (!sram_base) {
		pr_warn("%s: unable to alloc ocram!\n", __func__);
		goto put_node;
	}

	sram_pbase = gen_pool_virt_to_phys(sram_pool, sram_base);

	suspend_ocram_base = __arm_ioremap_exec(sram_pbase,
		WOKOO_SUSPEND_OCRAM_SIZE, false);

	memset(suspend_ocram_base, 0, sizeof(*pm_info));
	pm_info = suspend_ocram_base;
	pm_info->pbase = sram_pbase;
	pm_info->resume_addr  = __pa_symbol(wokoo_cpu_resume);
	pm_info->pm_info_size = sizeof(*pm_info);

	pm_info->mem_ctl_base.vbase = mem_ctl_base;
	pm_info->mem_phy_base.vbase = mem_phy_base;
	pm_info->mem_pwr_base.vbase = mem_pwr_base;
	pm_info->ap_pwr_base.vbase  = ap_pwr_base;

	/* Copy suspend code to SRAM */
	wokoo_suspend_in_ocram_fn = fncpy(
		suspend_ocram_base + sizeof(*pm_info),
		&wokoo_cpu_suspend,
		WOKOO_SUSPEND_OCRAM_SIZE - sizeof(*pm_info));

	register_syscore_ops(&wokoo_pm_syscore_ops);
	suspend_set_ops(&wokoo_suspend_ops);
put_node:
	of_node_put(np);
}
