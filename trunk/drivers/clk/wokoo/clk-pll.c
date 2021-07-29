/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include "clk.h"




#define WOKOO_PLL_CTL1_OFFSET           0x04
#define WOKOO_PLL_FREQ_CTL_OFFSET       0x30

#define WOKOO_PLL_LOCK		            BIT(24)
#define WOKOO_PLL_FRAC_SHIFT		    0
#define WOKOO_PLL_FRAC_MASK             0xffffff
#define WOKOO_PLL_DSMPD_SHIFT           28
#define WOKOO_PLL_DSMPD_MASK            0x1
#define WOKOO_PLL_POSTDIV1_SHIFT        20
#define WOKOO_PLL_POSTDIV1_MASK         0x7
#define WOKOO_PLL_POSTDIV2_SHIFT        24
#define WOKOO_PLL_POSTDIV2_MASK         0x7
#define WOKOO_PLL_FBDIV_SHIFT           8
#define WOKOO_PLL_FBDIV_MASK            0xfff
#define WOKOO_PLL_REFDIV_SHIFT          0
#define WOKOO_PLL_REFDIV_MASK           0x3f
#define WOKOO_PLL_FREQ_MASK_SHIFT       BIT(1)
#define WOKOO_PLL_FREQ_START_SHIFT      BIT(0)
#define WOKOO_PLL_FREQ_MASK_EN_WRITE    BIT(17)
#define WOKOO_PLL_FREQ_START_EN_WRITE   BIT(16)

/**
 * struct wokoo_clk_pll - WOKOO PLL clock
 * @clk_hw: clock source
 * @base: base address of PLL registers
 * @ref_clock: refer clock
 * @rate_count: rate count
 * @rate_table: table of divider pll rate
 *
 * WOKOO PLL clock, found on codefair series.  Divider for pll
 * is actually a multiplier, and always sits at bit 0.
 */
struct wokoo_clk_pll {
	struct clk_hw	hw;
	void __iomem	*base;
	unsigned long	ref_clock;
	unsigned int	rate_count;
	const struct wokoo_pll_rate_table *rate_table;
};




#define to_wokoo_clk_pll(_hw) container_of(_hw, struct wokoo_clk_pll, hw)




static int wokoo_clk_pll_wait_lock(struct wokoo_clk_pll *pll)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(10);

	/* Wait for PLL to lock */
	do {
		if (readl_relaxed(pll->base + WOKOO_PLL_CTL1_OFFSET) & WOKOO_PLL_LOCK)
			break;
		if (time_after(jiffies, timeout))
			break;
		usleep_range(50, 500);
	} while (1);

	return readl_relaxed(pll->base + WOKOO_PLL_CTL1_OFFSET) & WOKOO_PLL_LOCK ? 0 : -ETIMEDOUT;
}

static const struct wokoo_pll_rate_table *wokoo_clk_get_pll_settings(
				struct wokoo_clk_pll *pll, unsigned long rate)
{
	const struct wokoo_pll_rate_table  *rate_table = pll->rate_table;
	int i;

	for (i = 0; i < pll->rate_count; i++) {
		if (rate == rate_table[i].rate)
			return &rate_table[i];
	}

	return NULL;
}

static long wokoo_clk_pll_round_rate(struct clk_hw *hw,
				unsigned long drate, unsigned long *prate)
{
	struct wokoo_clk_pll *pll = to_wokoo_clk_pll(hw);
	const struct wokoo_pll_rate_table *rate_table = pll->rate_table;
	int i;

	/* Assumming rate_table is in descending order */
	for (i = 0; i < pll->rate_count; i++) {
		if (drate >= rate_table[i].rate)
			return rate_table[i].rate;
	}

	/* return minimum supported value */
	return rate_table[i - 1].rate;
}

static void wokoo_clk_pll_get_params(struct wokoo_clk_pll *pll,
					struct wokoo_pll_rate_table *rate)
{
	u32 pllcon0, pllcon1;

	pllcon0 = readl_relaxed(pll->base);
	rate->fbdiv = ((pllcon0 >> WOKOO_PLL_FBDIV_SHIFT)
				& WOKOO_PLL_FBDIV_MASK);

	rate->refdiv   = ((pllcon0 >> WOKOO_PLL_REFDIV_SHIFT)
				& WOKOO_PLL_REFDIV_MASK);
	rate->postdiv1 = ((pllcon0 >> WOKOO_PLL_POSTDIV1_SHIFT)
				& WOKOO_PLL_POSTDIV1_MASK);
	rate->postdiv2 = ((pllcon0 >> WOKOO_PLL_POSTDIV2_SHIFT)
				& WOKOO_PLL_POSTDIV2_MASK);

	pllcon1 = readl_relaxed(pll->base + WOKOO_PLL_CTL1_OFFSET);
	rate->frac = ((pllcon1 >> WOKOO_PLL_FRAC_SHIFT)
				& WOKOO_PLL_FRAC_MASK);

	rate->dsmpd = ((pllcon0 >> WOKOO_PLL_DSMPD_SHIFT) & (WOKOO_PLL_DSMPD_MASK));

#if 0
	pr_debug("%s: get params %x/%x fbdiv: %d, postdiv1: %d, refdiv: %d, postdiv2: %d, dsmpd: %d, frac: %d\n",
		__func__, pllcon0, pllcon1, rate->fbdiv, rate->postdiv1, rate->refdiv,
		rate->postdiv2, rate->dsmpd, rate->frac);
#endif
}

static unsigned long wokoo_clk_pll_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct wokoo_clk_pll *pll = to_wokoo_clk_pll(hw);
	struct wokoo_pll_rate_table cur;
	u64 rate64 = parent_rate;

	wokoo_clk_pll_get_params(pll, &cur);

	rate64 *= cur.fbdiv;
	do_div(rate64, cur.refdiv);

	if (cur.dsmpd == 0) {
		/* fractional mode */
		u64 frac_rate64 = parent_rate * cur.frac;

		do_div(frac_rate64, cur.refdiv);
		rate64 += frac_rate64 >> 24;
	}

	do_div(rate64, cur.postdiv1);
	do_div(rate64, cur.postdiv2);

	return (unsigned long)rate64;
}

static int wokoo_clk_pll_set_params(struct wokoo_clk_pll *pll,
				const struct wokoo_pll_rate_table *rate)
{
	struct wokoo_pll_rate_table cur;
	u32 pllcon0, pllcon1;
	int ret;

	pr_debug("%s: rate settings for %lu fbdiv: %d, postdiv1: %d, refdiv: %d, postdiv2: %d, dsmpd: %d, frac: %d\n",
		__func__, rate->rate, rate->fbdiv, rate->postdiv1, rate->refdiv,
		rate->postdiv2, rate->dsmpd, rate->frac);

	wokoo_clk_pll_get_params(pll, &cur);
	cur.rate = 0;


	pllcon0  = (rate->fbdiv &  WOKOO_PLL_FBDIV_MASK) << WOKOO_PLL_FBDIV_SHIFT;
	pllcon0 |= (rate->postdiv1 &  WOKOO_PLL_POSTDIV1_MASK) << WOKOO_PLL_POSTDIV1_SHIFT;
	pllcon0 |= (rate->postdiv2 &  WOKOO_PLL_POSTDIV2_MASK) << WOKOO_PLL_POSTDIV2_SHIFT;
	pllcon0 |= (rate->refdiv &  WOKOO_PLL_REFDIV_MASK) << WOKOO_PLL_REFDIV_SHIFT;

	/* update pll values */
	writel_relaxed(pllcon0, pll->base);

	pllcon1  = (rate->frac &  WOKOO_PLL_FRAC_MASK) << WOKOO_PLL_FRAC_SHIFT;
	pllcon1 |= (rate->dsmpd &  WOKOO_PLL_DSMPD_MASK) << WOKOO_PLL_DSMPD_SHIFT;

	writel_relaxed(pllcon1, pll->base + WOKOO_PLL_CTL1_OFFSET);


	/* wait for the pll to lock */
	ret = wokoo_clk_pll_wait_lock(pll);
	if (ret) {
		pr_warn("%s: pll update unsuccessful, trying to restore old params\n",
			__func__);
		wokoo_clk_pll_set_params(pll, &cur);
	}

	return ret;
}

static int wokoo_clk_pll_set_rate(struct clk_hw *hw, unsigned long drate,
					unsigned long prate)
{
	struct wokoo_clk_pll *pll = to_wokoo_clk_pll(hw);
	const struct wokoo_pll_rate_table *rate;

	pr_debug("%s: changing %s to %lu with a parent rate of %lu\n",
		 __func__, __clk_get_name(hw->clk), drate, prate);

	/* Get required rate settings from table */
	rate = wokoo_clk_get_pll_settings(pll, drate);
	if (!rate) {
		pr_err("%s: Invalid rate : %lu for pll clk %s\n", __func__,
			drate, __clk_get_name(hw->clk));
		return -EINVAL;
	}

	return wokoo_clk_pll_set_params(pll, rate);
}

static int wokoo_clk_pll_enable(struct clk_hw *hw)
{
	struct wokoo_clk_pll *pll = to_wokoo_clk_pll(hw);
	u32  pllfreq = 0;


	pllfreq  = readl(pll->base + WOKOO_PLL_FREQ_CTL_OFFSET);
	pllfreq &= ~WOKOO_PLL_FREQ_MASK_SHIFT;
	pllfreq |=  WOKOO_PLL_FREQ_MASK_EN_WRITE;
	writel(pllfreq, pll->base + WOKOO_PLL_FREQ_CTL_OFFSET);

	return 0;
}

static void wokoo_clk_pll_disable(struct clk_hw *hw)
{
	struct wokoo_clk_pll *pll = to_wokoo_clk_pll(hw);
	u32  pllfreq = 0;


	pllfreq  = readl(pll->base + WOKOO_PLL_FREQ_CTL_OFFSET);
	pllfreq |= WOKOO_PLL_FREQ_MASK_SHIFT | WOKOO_PLL_FREQ_MASK_EN_WRITE;
	writel(pllfreq, pll->base + WOKOO_PLL_FREQ_CTL_OFFSET);
}

static int wokoo_clk_pll_is_enabled(struct clk_hw *hw)
{
	struct wokoo_clk_pll *pll = to_wokoo_clk_pll(hw);
	u32 pllcon = readl(pll->base + WOKOO_PLL_FREQ_CTL_OFFSET);

	return !(pllcon & WOKOO_PLL_FREQ_MASK_SHIFT);
}

static void wokoo_clk_pll_init(struct clk_hw *hw)
{
	struct wokoo_clk_pll *pll = to_wokoo_clk_pll(hw);
	const struct wokoo_pll_rate_table *rate;
	struct wokoo_pll_rate_table cur;
	unsigned long drate;


	drate = clk_hw_get_rate(hw);
	rate  = wokoo_clk_get_pll_settings(pll, drate);

	//printk("wokoo_clk_pll_init rate = %ld\n", drate);

	/* when no rate setting for the current rate, rely on clk_set_rate */
	if (!rate)
		return;

	wokoo_clk_pll_get_params(pll, &cur);

	pr_debug("%s: pll %s@%lu: Hz\n", __func__, __clk_get_name(hw->clk),
		 drate);
	pr_debug("old - fbdiv: %d, postdiv1: %d, refdiv: %d, postdiv2: %d, dsmpd: %d, frac: %d\n",
		 cur.fbdiv, cur.postdiv1, cur.refdiv, cur.postdiv2,
		 cur.dsmpd, cur.frac);
	pr_debug("new - fbdiv: %d, postdiv1: %d, refdiv: %d, postdiv2: %d, dsmpd: %d, frac: %d\n",
		 rate->fbdiv, rate->postdiv1, rate->refdiv, rate->postdiv2,
		 rate->dsmpd, rate->frac);

	if (rate->fbdiv != cur.fbdiv || rate->postdiv1 != cur.postdiv1 ||
		rate->refdiv != cur.refdiv || rate->postdiv2 != cur.postdiv2 ||
		rate->dsmpd != cur.dsmpd ||
		(!cur.dsmpd && (rate->frac != cur.frac))) {
		struct clk *parent = clk_get_parent(hw->clk);

		if (!parent) {
			pr_warn("%s: parent of %s not available\n",
				__func__, __clk_get_name(hw->clk));
			return;
		}

		pr_debug("%s: pll %s: rate params do not match rate table, adjusting\n",
			 __func__, __clk_get_name(hw->clk));
		wokoo_clk_pll_set_params(pll, rate);
	}
}

static const struct clk_ops wokoo_clk_pll_ops = {
	.recalc_rate = wokoo_clk_pll_recalc_rate,
	.round_rate	 = wokoo_clk_pll_round_rate,
	.set_rate	 = wokoo_clk_pll_set_rate,
	.enable		 = wokoo_clk_pll_enable,
	.disable	 = wokoo_clk_pll_disable,
	.is_enabled	 = wokoo_clk_pll_is_enabled,
	.init		 = wokoo_clk_pll_init,
};

/**
 * wokoo_clk_hw_pll - register pll clock 
 * @type: pll type
 * @name: name of this clock
 * @parent_name: name of this clock's parent
 * @base: register address to control gating of this clock
 * @wokoo_pll_rate_table: pll rate table
 */
struct clk_hw *wokoo_clk_hw_pll(enum wokoo_pll_type type, const char *name,
				const char *parent_name, void __iomem *base,
				struct wokoo_pll_rate_table *rate_table)
{
	struct wokoo_clk_pll *pll;
	const struct clk_ops *ops;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	if (rate_table) {
		int len;

		/* find count of rates in rate_table */
		for (len = 0; rate_table[len].rate != 0; )
			len++;

		pll->rate_count = len;
		pll->rate_table = kmemdup(rate_table,
					pll->rate_count *
					sizeof(struct wokoo_pll_rate_table),
					GFP_KERNEL);
		WARN(!pll->rate_table,
			"%s: could not allocate rate table for %s\n",
			__func__, name);
	}

	switch (type) {
	default:
		ops = &wokoo_clk_pll_ops;
	}
	pll->base = base;

	init.name = name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;
	hw = &pll->hw;

	ret = clk_hw_register(NULL, hw);
	if (ret) {
		kfree(pll);
		return ERR_PTR(ret);
	}

	return hw;
}
