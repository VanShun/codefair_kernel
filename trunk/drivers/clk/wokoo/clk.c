/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */ 
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/reboot.h>
#include "clk.h"




/* Spinclock define for ccm */
DEFINE_SPINLOCK(wokoo_ccm_lock);




/* Earlycon */
static int wokoo_keep_uart_clocks;
static struct clk ** const *wokoo_uart_clocks;




/**
 * wokoo_keep_uart_clocks_param - init keep uart clocks
 * @str: string for setup param
 *
 */
static int __init wokoo_keep_uart_clocks_param(char *str)
{
	wokoo_keep_uart_clocks = 1;

	return 0;
}

__setup_param("earlycon", wokoo_keep_uart_earlycon,
			wokoo_keep_uart_clocks_param, 0);
__setup_param("earlyprintk", wokoo_keep_uart_earlyprintk,
			wokoo_keep_uart_clocks_param, 0);

/**
 * wokoo_register_uart_clocks - register uart clocks
 * @clk: clocks
 *
 */
void wokoo_register_uart_clocks(struct clk ** const clks[])
{
	if (wokoo_keep_uart_clocks) {
		int i;

		wokoo_uart_clocks = clks;
		for (i = 0; wokoo_uart_clocks[i]; i++)
			clk_prepare_enable(*wokoo_uart_clocks[i]);
	}
}

/**
 * wokoo_clk_disable_uart - disable uart clocks
 *
 */
static int __init wokoo_clk_disable_uart(void)
{
	if (wokoo_keep_uart_clocks && wokoo_uart_clocks) {
		int i;

		for (i = 0; wokoo_uart_clocks[i]; i++)
			clk_disable_unprepare(*wokoo_uart_clocks[i]);
	}

	return 0;
}
late_initcall_sync(wokoo_clk_disable_uart);


static void __iomem *rst_base;
static unsigned int reg_restart;
static void (*cb_restart)(void);
/**
 * wokoo_restart_notify - restart machine
 * @this
 * @mode
 * @cmd
 */
static int wokoo_restart_notify(struct notifier_block *this,
				   unsigned long mode, void *cmd)
{
	if (cb_restart)
		cb_restart();

	/* Write 0xff to restart reg */
	writel(0xff, rst_base + reg_restart);
	return NOTIFY_DONE;
}

static struct notifier_block wokoo_restart_handler = {
	.notifier_call = wokoo_restart_notify,
	.priority = 128,
};

/**
 * wokoo_register_restart_notifier - restart funtion register
 * @base: restart reg base
 * @reg: restart reg
 * @cb: callback
 */
void __init
wokoo_register_restart_notifier(void __iomem *base,
					       unsigned int reg, void (*cb)(void))
{
	int ret;

	rst_base    = base;
	reg_restart = reg;
	cb_restart  = cb;

	ret = register_restart_handler(&wokoo_restart_handler);
	if (ret)
		pr_err("%s: cannot register restart handler, %d\n",
				__func__, ret);
}

