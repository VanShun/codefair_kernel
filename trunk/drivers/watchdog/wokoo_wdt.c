/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>




#define DRIVER_NAME "wokoo-wdt"

#define WOKOO_WDT_CR            0x00 /* Control register */
#define WOKOO_WDT_TORR          0x04 /* Timeout range register */
#define WOKOO_WDT_CCVR          0x08 /* Current count register */
#define WOKOO_WDT_CRR           0x0c /* Counter restart register */
#define WOKOO_WDT_STAT          0x10 /* Status register */
#define WOKOO_WDT_ICR           0x14 /* Interupt clear register */

#define WOKOO_WDT_CR_ENABLE     BIT(0)
#define WOKOO_WDT_CR_RMOD       BIT(1)
#define WOKOO_WDT_CR_RPL        (0x7 << 2)
#define WOKOO_WDT_TORR_TOUT		(0xF << 0)
#define WOKOO_WDT_CRR_RLOAD     0x76
#define WOKOO_WDT_ICR_ICR       BIT(0)

#define WOKOO_WDT_MAX_TIME      66u
#define WOKOO_WDT_DEFAULT_TIME  9




/**
 * struct wokoo_wdt_device - wokoo wathch device
 * @clk: gate clcok
 * @regmap: base address
 * @wdog: wathdog device
 * @ext_reset: ext reset
 */
struct wokoo_wdt_device {
	struct clk				*clk;
	void __iomem			*base;
	struct watchdog_device	wdog;
	bool 					ext_reset;
	unsigned int			max_time;
};




static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");


static unsigned timeout;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default="
				__MODULE_STRING(WOKOO_WDT_DEFAULT_TIME) ")");

static const struct watchdog_info wokoo_wdt_info = {
	.identity = "wokoo+ watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
};

static const struct watchdog_info wokoo_wdt_pretimeout_info = {
	.identity = "wokoo+ watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE |
		   WDIOF_PRETIMEOUT,
};



/*
 * wokoo_wdt_sec_to_count - calc count reg value
 */
static unsigned int wokoo_wdt_sec_to_count(unsigned int sec, unsigned int maxsec)
{
	unsigned int  val = 0;

	if (sec > maxsec) {
		val = 0xf;
	} else if (sec > (maxsec + 1) / 2) {
		val = 0xe;
	} else if (sec > (maxsec + 1) / 4) {
		val = 0xd;
	} else if (sec > (maxsec + 1) / 8) {
		val = 0xc;
	} else if (sec > (maxsec + 1) / 16) {
		val = 0xb;
	} else if (sec > (maxsec + 1) / 32) {
		val = 0xa;
	} else if (sec > (maxsec + 1) / 64) {
		val = 0x9;
	} else if (sec >= (maxsec + 1) / 128) {
		val = 0x8;
	} else {
		val = 0x7;
	}

	return val;
}

/*
 * wokoo_wdt_set_bits - set reg bits
 */
static void wokoo_wdt_set_bits(struct wokoo_wdt_device *wdev, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(wdev->base + reg);
	val |= bits;
	writel_relaxed(val, wdev->base + reg);
}

#if 0
/*
 * wokoo_wdt_clr_bits - clear reg bits
 */
static void  wokoo_wdt_clr_bits(struct wokoo_wdt_device *wdev, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(wdev->base + reg);
	val &= ~bits;
	writel_relaxed(val, wdev->base + reg);
}
#endif

/*
 * wokoo_wdt_restart - restart watchdog
 */
static int wokoo_wdt_restart(struct watchdog_device *wdog, unsigned long action,
							void *data)
{
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);

	wokoo_wdt_set_bits(wdev, WOKOO_WDT_CR, WOKOO_WDT_CR_ENABLE);
	wokoo_wdt_set_bits(wdev, WOKOO_WDT_CRR, WOKOO_WDT_CRR_RLOAD);

	/* wait for reset to assert... */
	mdelay(500);

	return 0;
}

/*
 * wokoo_wdt_setup - watchdog setup
 */
static inline void wokoo_wdt_setup(struct watchdog_device *wdog)
{
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);
	u32 regval = 0;
	u32 cntval = 0;


	/* Set the watchdog's Time-Out value */
	cntval = wokoo_wdt_sec_to_count(wdog->timeout, wdev->max_time);

	regval = readl_relaxed(wdev->base + WOKOO_WDT_TORR);
	regval = (regval & ~0xf) + (cntval & 0xf);
	writel_relaxed(regval, wdev->base + WOKOO_WDT_TORR);

	/* Set watchdog mode */
	wokoo_wdt_set_bits(wdev, WOKOO_WDT_CR, WOKOO_WDT_CR_RMOD);

	wokoo_wdt_set_bits(wdev, WOKOO_WDT_CR, WOKOO_WDT_CR_ENABLE);
}

/*
 * wokoo_wdt_is_running - is running
 */
static inline bool wokoo_wdt_is_running(struct wokoo_wdt_device *wdev)
{
	u32 val;

	val = readl_relaxed(wdev->base + WOKOO_WDT_CR);

	return val & WOKOO_WDT_CR_ENABLE;
}

/*
 * wokoo_wdt_ping - restart watchdog and clear interupt
 */
static int wokoo_wdt_ping(struct watchdog_device *wdog)
{
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);
	u32 val;

	val = readl_relaxed(wdev->base + WOKOO_WDT_CCVR);
	writel_relaxed(WOKOO_WDT_CRR_RLOAD, wdev->base + WOKOO_WDT_CRR);
	return 0;
}

/*
 * __wokoo_wdt_set_timeout - new timeout value
 */
static void __wokoo_wdt_set_timeout(struct watchdog_device *wdog,
				   unsigned int new_timeout)
{
	u32 regval = 0;
	u32 cntval = 0;
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);

	cntval = wokoo_wdt_sec_to_count(new_timeout, wdev->max_time);
	regval = readl_relaxed(wdev->base + WOKOO_WDT_TORR);
	regval = (regval & ~0xf) + (cntval & 0xf);
	writel_relaxed(regval, wdev->base + WOKOO_WDT_TORR);
}

static int wokoo_wdt_set_timeout(struct watchdog_device *wdog,
				unsigned int new_timeout)
{
	unsigned int actual;
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);

	actual = min(new_timeout, wdev->max_time);
	__wokoo_wdt_set_timeout(wdog, actual);
	wdog->timeout = new_timeout;
	return 0;
}

/*
 * wokoo_wdt_set_pretimeout - pre timeout value
 */
static int wokoo_wdt_set_pretimeout(struct watchdog_device *wdog,
				   unsigned int new_pretimeout)
{
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);

	if (new_pretimeout >= wdev->max_time)
		return -EINVAL;

	wdog->pretimeout = new_pretimeout;
	return 0;
}

/*
 * wokoo_wdt_isr - interupt handler
 */
static irqreturn_t wokoo_wdt_isr(int irq, void *wdog_arg)
{
	struct watchdog_device  *wdog = wdog_arg;
	//struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);
	//unsigned int  val = 0;

	/* Don`t clear irq, next timeout reset system */
	//val = readl_relaxed(wdev->base + WOKOO_WDT_ICR);

	watchdog_notify_pretimeout(wdog);

	return IRQ_HANDLED;
}

/*
 * wokoo_wdt_start - start watchdog
 */
static int wokoo_wdt_start(struct watchdog_device *wdog)
{
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);

	if (wokoo_wdt_is_running(wdev))
		wokoo_wdt_set_timeout(wdog, wdog->timeout);
	else
		wokoo_wdt_setup(wdog);

	set_bit(WDOG_HW_RUNNING, &wdog->status);

	return wokoo_wdt_ping(wdog);
}

static const struct watchdog_ops wokoo_wdt_ops = {
	.owner = THIS_MODULE,
	.start = wokoo_wdt_start,
	.ping  = wokoo_wdt_ping,
	.set_timeout    = wokoo_wdt_set_timeout,
	.set_pretimeout = wokoo_wdt_set_pretimeout,
	.restart = wokoo_wdt_restart,
};

static int __init wokoo_wdt_probe(struct platform_device *pdev)
{
	struct wokoo_wdt_device  *wdev;
	struct watchdog_device   *wdog;
	struct clk               *clk;
	int ret;

	wdev = devm_kzalloc(&pdev->dev, sizeof(*wdev), GFP_KERNEL);
	if (!wdev)
		return -ENOMEM;

	wdev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdev->base))
		return PTR_ERR(wdev->base);

	wdev->clk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(wdev->clk)) {
		dev_err(&pdev->dev, "can't get Watchdog clock\n");
		return PTR_ERR(wdev->clk);
	}

	clk = devm_clk_get(&pdev->dev, "sclk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "can't get Watchdog source clock\n");
		return PTR_ERR(clk);
	}

	wdev->max_time  = 0x7fffffff / clk_get_rate(clk);
	wdog			= &wdev->wdog;
	wdog->info		= &wokoo_wdt_info;
	wdog->ops		= &wokoo_wdt_ops;
	wdog->parent	= &pdev->dev;
	wdog->timeout	= wdev->max_time / 2;
	wdog->min_timeout			= 1;
	wdog->max_hw_heartbeat_ms	= wdev->max_time * 1000;

	ret = platform_get_irq(pdev, 0);
	if (ret > 0)
		if (!devm_request_irq(&pdev->dev, ret, wokoo_wdt_isr, 0,
						dev_name(&pdev->dev), wdog))
			wdog->info = &wokoo_wdt_pretimeout_info;

	ret = clk_prepare_enable(wdev->clk);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, wdog);
	watchdog_set_drvdata(wdog, wdev);
	watchdog_set_nowayout(wdog, nowayout);
	watchdog_set_restart_priority(wdog, 128);
	watchdog_init_timeout(wdog, timeout, &pdev->dev);

	if (wokoo_wdt_is_running(wdev)) {
		wokoo_wdt_set_timeout(wdog, wdog->timeout);
		set_bit(WDOG_HW_RUNNING, &wdog->status);
	}


	ret = watchdog_register_device(wdog);
	if (ret)
		goto disable_clk;

	dev_info(&pdev->dev, "timeout %d sec (nowayout=%d)\n",
		 wdog->timeout, nowayout);

	return 0;

disable_clk:
	clk_disable_unprepare(wdev->clk);
	return ret;
}

static int __exit wokoo_wdt_remove(struct platform_device *pdev)
{
	struct watchdog_device  *wdog = platform_get_drvdata(pdev);
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);

	watchdog_unregister_device(wdog);

	if (wokoo_wdt_is_running(wdev)) {
		wokoo_wdt_ping(wdog);
		dev_crit(&pdev->dev, "Device removed: Expect reboot!\n");
	}

	clk_disable_unprepare(wdev->clk);
	return 0;
}

static void wokoo_wdt_shutdown(struct platform_device *pdev)
{
	struct watchdog_device  *wdog = platform_get_drvdata(pdev);
	struct wokoo_wdt_device *wdev = watchdog_get_drvdata(wdog);

	if (wokoo_wdt_is_running(wdev)) {
		/*
		 * We are running, configure max timeout before reboot
		 * will take place.
		 */
		wokoo_wdt_set_timeout(wdog, wdev->max_time);
		wokoo_wdt_ping(wdog);
		dev_crit(&pdev->dev, "Device shutdown: Expect reboot!\n");
	}
}

#ifdef CONFIG_PM_SLEEP
/* Disable watchdog if it is active or non-active but still running */
static int wokoo_wdt_suspend(struct device *dev)
{
	struct wokoo_wdt_device *wdev = dev_get_drvdata(dev);

	clk_disable_unprepare(wdev->clk);
	return 0;
}

/* Enable watchdog and configure it if necessary */
static int wokoo_wdt_resume(struct device *dev)
{
	int ret = 0;
	struct wokoo_wdt_device *wdev = dev_get_drvdata(dev);

	ret = clk_prepare_enable(wdev->clk);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(wokoo_wdt_pm_ops, wokoo_wdt_suspend,
			 wokoo_wdt_resume);

static const struct of_device_id wokoo_wdt_dt_ids[] = {
	{ .compatible = "wokoo,wokoo-wdt", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wokoo_wdt_dt_ids);

static struct platform_driver wokoo_wdt_driver = {
	.remove		= __exit_p(wokoo_wdt_remove),
	.shutdown	= wokoo_wdt_shutdown,
	.driver		= {
		.name	= DRIVER_NAME,
		.pm		= &wokoo_wdt_pm_ops,
		.of_match_table = wokoo_wdt_dt_ids,
	},
};

module_platform_driver_probe(wokoo_wdt_driver, wokoo_wdt_probe);

MODULE_ALIAS("platform:wokoo-wdt");
MODULE_DESCRIPTION("Wokoo SoC watchdog driver");
MODULE_AUTHOR("GuangHuang Lin<linguanghuang@codefairsemi.com >");
MODULE_LICENSE("GPL v2");
