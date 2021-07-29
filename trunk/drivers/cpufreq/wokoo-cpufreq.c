// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 * CPU frequency scaling for WOKOO
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>




/* Use 144MHz when entering sleep mode */
#define SLEEP_FREQ_INDEX     L13
#define WOKOO_A7_CLK_MAX_DIV (L15 +1)


/* Tracks if cpu freqency can be updated anymore */
static bool no_cpufreq_access;
static struct device *cpu_dev;
/* A7 clock */
static struct clk	 *clk_a7;
/* Pll_out clock */
static struct clk	 *clk_pll;
/* A7 axi main clock */
static struct clk	 *clk_a7_axi;

static DEFINE_MUTEX(set_freq_lock);

/* Freqency index */
enum perf_level {
	L0, L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11, L12, L13, L14, L15
};

/* Freqency table */
#if 0
static struct cpufreq_frequency_table wokoo_freq_table[] = {
	{0, L0, 768000},
	{0, L1, 720000},
	{0, L2, 672000},
	{0, L3, 624000},
	{0, L4, 576000},
	{0, L5, 528000},
	{0, L6, 480000},
	{0, L7, 432000},
	{0, L8, 384000},
	{0, L9, 336000},
	{0, L10, 288000},
	{0, L11, 240000},
	{0, L12, 192000},
	{0, L13, 144000},
	//{0, L14, 96000},
	//{0, L15, 48000},
	{0, 0, CPUFREQ_TABLE_END},
};
#else
static struct cpufreq_frequency_table wokoo_freq_table[] = {
	{0, L0, 0},
	{0, L1, 0},
	{0, L2, 0},
	{0, L3, 0},
	{0, L4, 0},
	{0, L5, 0},
	{0, L6, 0},
	{0, L7, 0},
	{0, L8, 0},
	{0, L9, 0},
	{0, L10, 0},
	{0, L11, 0},
	{0, L12, 0},
	{0, L13, 0},
	//{0, L14, 0}, /* Not use L14, L15, need dynamic frequency for a7_axi */
	//{0, L15, 0},
	{0, 0, CPUFREQ_TABLE_END},
};
#endif

/* 
 * wokoo_target - set target frequency
 */
static int wokoo_target(struct cpufreq_policy *policy, unsigned int index)
{
	unsigned long new_freq;
	unsigned int  priv_index;
	int ret = 0;

	mutex_lock(&set_freq_lock);

	if (no_cpufreq_access) {
		pr_err("Denied access to %s as it is disabled temporarily\n",
				__func__);
		ret = -EINVAL;
		goto exit;
	}

	priv_index = cpufreq_table_find_index_h(policy, policy->cur);
	new_freq   = wokoo_freq_table[index].frequency * 1000;
	//pr_debug("wokoo_target freq = %ld\n", new_freq);

	/* Can`t dynamic frequency for a7_axi */
 #if 0
	if (index > L13) {
		if (priv_index <= L13) {
			clk_set_rate(clk_a7_axi, 32000000);
			//pr_debug("wokoo_target get a7_axi = %ld, set = 16M\n", clk_get_rate(clk_a7_axi));
		}
	} else {
		if (priv_index > L13) {
			clk_set_rate(clk_a7_axi, 64000000);
			//pr_debug("wokoo_target get a7_axi = %ld, set = 64M\n", clk_get_rate(clk_a7_axi));
		}
	}
#endif

	clk_set_rate(clk_a7, new_freq);

	pr_debug("Perf changed[L%d]\n", index);

exit:
	mutex_unlock(&set_freq_lock);
	return ret;
}

/* 
 * wokoo_cpu_init - init frequency table
 */
static int wokoo_cpu_init(struct cpufreq_policy *policy)
{
	int ret, i;
	unsigned long pll_rate_step;

	if (policy->cpu != 0) {
		ret = -EINVAL;
		return ret;
	}

	policy->clk   = clk_a7;
	pll_rate_step = clk_get_rate(clk_pll) / 1000 / WOKOO_A7_CLK_MAX_DIV;

	/* Get frequency table */
	for (i = 0; i < L14; i++) {
		wokoo_freq_table[i].frequency = pll_rate_step * (L15 + 1 - i);
	}

	policy->suspend_freq = wokoo_freq_table[SLEEP_FREQ_INDEX].frequency;
	cpufreq_generic_init(policy, wokoo_freq_table, 61036);
	return 0;
}

static struct cpufreq_driver wokoo_driver = {
	.flags		= CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= wokoo_target,
	.get		= cpufreq_generic_get,
	.init		= wokoo_cpu_init,
	.name		= "wokoo-cpufreq",
	.attr		= cpufreq_generic_attr,
	.suspend	= cpufreq_generic_suspend,
	.resume		= cpufreq_generic_suspend, /* We need to set SLEEP FREQ again */
};

static int wokoo_cpufreq_probe(struct platform_device *pdev)
{
	int ret;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		return -ENODEV;
	}

	clk_a7 = devm_clk_get(cpu_dev, "a7");
	if (IS_ERR(clk_a7)) {
		ret = PTR_ERR(clk_a7);
		dev_err(cpu_dev, "failed to get a7 clk: %d\n", ret);
		return ret;
	}

	clk_pll = devm_clk_get(cpu_dev, "pll");
	if (IS_ERR(clk_pll)) {
		ret = PTR_ERR(clk_pll);
		dev_err(cpu_dev, "failed to get pll clk: %d\n", ret);
		return ret;
	}

	clk_a7_axi = devm_clk_get(cpu_dev, "a7_axi");
	if (IS_ERR(clk_a7_axi)) {
		ret = PTR_ERR(clk_a7_axi);
		dev_err(cpu_dev, "failed to get pll clk: %d\n", ret);
		return ret;
	}

	return cpufreq_register_driver(&wokoo_driver);
}

static struct platform_driver wokoo_cpufreq_platdrv = {
	.driver = {
		.name	= "wokoo-cpufreq",
	},
	.probe = wokoo_cpufreq_probe,
};
builtin_platform_driver(wokoo_cpufreq_platdrv);
