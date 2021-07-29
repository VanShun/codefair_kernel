/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <soc/imx/timer.h>




#define TIMER_NAME            "wokoo_apt"

/* Timer load value */
#define WOKOO_TIMER_TLC        0x00
/* Timer read current value */
#define WOKOO_TIMER_TCV        0x04
/* Timer control reg */
#define WOKOO_TIMER_TCR        0x08
/* Timer clear irq reg */
#define WOKOO_TIMER_TIC        0x0C
/* Timer irq status reg */
#define WOKOO_TIMER_TIS        0x10

/* 1 - disable interput, 0 - enable */
#define WOKOO_TIMER_TCR_INT_EN BIT(2)
/* 1 - custom mode, 0 - auto mode */
#define WOKOO_TIMER_TCR_MODE   BIT(1)
#define WOKOO_TIMER_TCR_CNT_EN BIT(0)




/**
 * struct wokoo_timer - WOKOO timer
 * @base: base address of timer registers
 * @irq: timer irq number
 * @compen: count compensation value
 * @clk_sclk: source clock for timer
 * @clk_pclk: gate clock for timer 
 * @ced: clock event device 
 * @act: irq action 
 * @freq: timer freq 
 *
 */
struct wokoo_timer {
	void __iomem *base;
	int          irq;
	/* Count compensation value */
	u32          compen;
	struct clk   *clk_sclk;
	struct clk   *clk_pclk;
	struct clock_event_device ced;
	struct irqaction          act;
	unsigned long             freq;
	unsigned long             intevt;
};




/*
 * set bits
 */
static void wokoo_set_bits(struct wokoo_timer *wokootm, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(wokootm->base + reg);
	val |= bits;
	writel_relaxed(val, wokootm->base + reg);
}

/*
 * clear bits
 */
static void  wokoo_clr_bits(struct wokoo_timer *wokootm, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(wokootm->base + reg);
	val &= ~bits;
	writel_relaxed(val, wokootm->base + reg);
}

static inline struct wokoo_timer *to_wokoo_timer(struct clock_event_device *ced)
{
	return container_of(ced, struct wokoo_timer, ced);
}

/*
 * irq disable
 */
static void wokoo_apt_irq_disable(struct wokoo_timer *wokootm)
{
	wokoo_set_bits(wokootm, WOKOO_TIMER_TCR, WOKOO_TIMER_TCR_INT_EN);
}

/*
 * timer disable
 */
static void wokoo_apt_timer_disable(struct wokoo_timer *wokootm)
{
	wokoo_clr_bits(wokootm, WOKOO_TIMER_TCR, WOKOO_TIMER_TCR_CNT_EN);
}

/*
 * timer enable
 */
static void wokoo_apt_timer_enable(struct wokoo_timer *wokootm)
{
	wokoo_set_bits(wokootm, WOKOO_TIMER_TCR, WOKOO_TIMER_TCR_CNT_EN);
}

/*
 * irq unmask
 */
static void wokoo_apt_irq_enable(struct wokoo_timer *wokootm)
{
	wokoo_clr_bits(wokootm, WOKOO_TIMER_TCR, WOKOO_TIMER_TCR_INT_EN);
}

/*
 * irq acknowledge, clear irq
 */
static void wokoo_apt_irq_acknowledge(struct wokoo_timer *wokootm)
{
	readl_relaxed(wokootm->base + WOKOO_TIMER_TIC);
}

static void __iomem *sched_clock_reg;

/*
 * ticks must be continuous
 */
static u64 totaltick = 0;
static u64 notrace wokoo_read_sched_clock(void)
{
	return totaltick + ~readl_relaxed(sched_clock_reg);
}

#if 1
static u64 notrace wokoo_clocksource_mmio_readl(struct clocksource *c)
{
	return totaltick + ~readl_relaxed(sched_clock_reg);
}
#endif

/* Do not use it, system can calc MIPS */
#if defined(CONFIG_ARM11)
static struct delay_timer wokoo_delay_timer;

/*
 * read current timer ticks
 */
static unsigned long wokoo_read_current_timer(void)
{
	return ~readl_relaxed(sched_clock_reg);
}
#endif

/*
 * update timer counter
 */
static void wokoo_apt_update_counter(unsigned long cycles,
				    struct wokoo_timer *wokootm)
{
	writel_relaxed(cycles, wokootm->base + WOKOO_TIMER_TLC);
}

/*
 * clocksource init
 */
static int __init wokoo_clocksource_init(struct wokoo_timer *wokootm)
{
	unsigned long c = clk_get_rate(wokootm->clk_sclk);
	/* 64M for PLL_OUT = 768M */
	int ret = 0;

	void __iomem *reg = wokootm->base + WOKOO_TIMER_TCV;

	wokootm->freq   = c;
	wokoo_apt_update_counter(0xffffffff, wokootm);
	wokoo_apt_timer_enable(wokootm);

	/* To provide accurate counting, we need register_current_timer_delay */
#if defined(CONFIG_ARM11)
	wokoo_delay_timer.read_current_timer = &wokoo_read_current_timer;
	wokoo_delay_timer.freq = wokootm->freq;
	register_current_timer_delay(&wokoo_delay_timer);
#endif

	sched_clock_reg = reg;

#if 1 /* Not need this part of code, sched_clock_register can provide date data */
	ret = clocksource_mmio_init(reg, TIMER_NAME, wokootm->freq, 250, 32,
			wokoo_clocksource_mmio_readl);

	if (ret) {
		pr_err("Failed to register clocksource\n");
		goto out_clocksource;
	}
#endif

	sched_clock_register(wokoo_read_sched_clock, 32, wokootm->freq);
	return ret;
#if 1
out_clocksource:
	return ret;
#endif
}

/*
 * wokoo_apt_set_next_event_ext
 */
static int wokoo_apt_set_next_event_ext(struct clock_event_device *ced)
{
	struct wokoo_timer *wokootm   = to_wokoo_timer(ced);

	totaltick += wokootm->intevt;
	wokoo_apt_timer_disable(wokootm);
	wokoo_apt_update_counter(wokootm->intevt, wokootm);
	wokoo_apt_timer_enable(wokootm);

	return 0;
}

/*
 * set_next_event
 */
static int wokoo_apt_set_next_event(unsigned long evt,
			      struct clock_event_device *ced)
{
#if 0
	struct wokoo_timer *wokootm   = to_wokoo_timer(ced);

	/* Values must be continuous */
	evt       -= wokootm->compen;
	totaltick += evt;

	wokoo_apt_timer_disable(wokootm);
	wokoo_apt_update_counter(evt, wokootm);
	wokoo_apt_timer_enable(wokootm);
#endif

	return 0;
}

/*
 * shutdown
 */
static int wokoo_shutdown(struct clock_event_device *ced)
{
	struct wokoo_timer *wokootm = to_wokoo_timer(ced);

	/* Disable interrupt in APT module */
	wokoo_apt_irq_disable(wokootm);

	wokoo_apt_timer_disable(wokootm);

	/* Clear pending interrupt */
	wokoo_apt_irq_acknowledge(wokootm);

	//printk(KERN_INFO "%s: changing mode\n", __func__);

	return 0;
}

/*
 * set oneshout
 */
static int wokoo_set_oneshot(struct clock_event_device *ced)
{
	struct wokoo_timer *wokootm = to_wokoo_timer(ced);

	/* Disable interrupt in APT module */
	wokoo_apt_irq_disable(wokootm);

	if (!clockevent_state_oneshot(ced)) {
		/* Clear pending interrupt */
		wokoo_apt_irq_acknowledge(wokootm);
	}

	//printk("%s: changing mode\n", __func__);

	wokoo_apt_timer_enable(wokootm);
	wokoo_apt_irq_enable(wokootm);

	return 0;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t wokoo_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *ced     = dev_id;
	struct wokoo_timer        *wokootm = to_wokoo_timer(ced);

	wokoo_apt_irq_acknowledge(wokootm);

	if (likely(clockevent_state_oneshot(ced)))
		wokoo_apt_timer_disable(wokootm);

#if 0
	ced->event_handler(ced);
#else
	/* Loader counter peridoic */
	wokoo_apt_set_next_event_ext(ced);
	ced->event_handler(ced);
#endif

	return IRQ_HANDLED;
}

/*
 * clockevent init for the timer
 */
static int __init wokoo_clockevent_init(struct wokoo_timer *wokootm)
{
	struct clock_event_device *ced = &wokootm->ced;
	/* 64M(bus_clk) / 20(timer_clk) for PLL_OUT = 768M */
	unsigned long c = clk_get_rate(wokootm->clk_sclk);
	int          ret = -EINVAL;

	ced->name     = TIMER_NAME;
	ced->features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_DYNIRQ;
	ced->set_state_shutdown = wokoo_shutdown;
	ced->set_state_oneshot  = wokoo_set_oneshot;
	ced->tick_resume        = wokoo_shutdown;
	ced->set_next_event     = wokoo_apt_set_next_event;
	ced->rating             = 200;
	ced->cpumask            = cpu_possible_mask;
	ced->irq                = wokootm->irq;
	wokootm->freq           = c;
	wokootm->compen         = c / HZ / 320;
	/* freq / 100 = 10ms tick */
	wokootm->intevt         = c / 100;

	ret = request_irq(wokootm->irq, wokoo_timer_interrupt, IRQF_TIMER,
			  TIMER_NAME, ced);
	if (ret) {
		pr_err("Failed to initialize '%s': %d\n",
			TIMER_NAME, ret);
		goto out_irq;
	}

	/* Ticks must > 4 for wokoo timer */
	clockevents_config_and_register(ced, wokootm->freq, 4, 0xfffffffe);
	return 0;

out_irq:
	return ret;
}

/*
 * timer init for the timer
 */
static int __init _wokoo_timer_init(struct wokoo_timer *wokootm)
{
	int ret;

	if (IS_ERR(wokootm->clk_sclk)) {
		pr_err("WOKOO timer: unable to get clk\n");
		return PTR_ERR(wokootm->clk_sclk);
	}

	if (!IS_ERR(wokootm->clk_pclk))
		clk_prepare_enable(wokootm->clk_pclk);

	clk_prepare_enable(wokootm->clk_sclk);

	/* Disable interrupt in APT module */
	wokoo_apt_irq_disable(wokootm);

	wokoo_apt_timer_disable(wokootm);

	/* Select custom mode */
	wokoo_clr_bits(wokootm, WOKOO_TIMER_TCR, WOKOO_TIMER_TCR_MODE);

	/* init and register the timer to the framework */
	ret = wokoo_clockevent_init(wokootm);
	if (ret)
		return ret;

	return wokoo_clocksource_init(wokootm);
}

/*
 * timer dt init for the timer
 */
static int __init wokoo_timer_init_dt(struct device_node *np)
{
	struct wokoo_timer *wokootm;
	static int initialized;
	int ret;

	/* Support one instance only */
	if (initialized)
		return 0;

	wokootm = kzalloc(sizeof(*wokootm), GFP_KERNEL);
	if (!wokootm)
		return -ENOMEM;

	wokootm->base = of_iomap(np, 0);
	if (!wokootm->base)
		return -ENXIO;

	wokootm->irq = irq_of_parse_and_map(np, 0);
	if (wokootm->irq <= 0)
		return -EINVAL;

	wokootm->clk_pclk = of_clk_get_by_name(np, "pclk");

	/* Try osc_per first, and fall back to per otherwise */
	wokootm->clk_sclk = of_clk_get_by_name(np, "sclk");
	if (IS_ERR(wokootm->clk_sclk))
		wokootm->clk_sclk = of_clk_get_by_name(np, "pclk");

	ret = _wokoo_timer_init(wokootm);
	if (ret)
		return ret;

	initialized = 1;

	return 0;
}

TIMER_OF_DECLARE(wokoo_timer, "wokoo,wokoo-apt", wokoo_timer_init_dt);
