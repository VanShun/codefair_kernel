/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */
#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include "clk.h"




/**
 * struct clk_mult_div
 * @hw: clk_hw 
 * @mult: multiplier value 
 * @div: divider value
 * @flags: CLK_SET_RATE_NO_WRITEABLE/CLK_SET_RATE_MUL_2
 * @base: clock reg base
 * @div_mask: divider bits mask
 * @div_shift: divider bit shift
 * @mult_mask: multiplier bits mask
 * @mult_shift: multiplier bit shift
 * 
 */
struct clk_mult_div {
	struct clk_hw	hw;
	unsigned int	mult;
	unsigned int	div;
	unsigned long	flags;
	void __iomem	*base;
	u32		div_mask;
	u32		div_shift;
	u32		mult_mask;
	u32		mult_shift;
};




//#define WOKOO_DEBUG    1
#define to_clk_mult_div(_hw) container_of(_hw, struct clk_mult_div, hw)


/**
 * DOC: basic fixed multiplier and divider clock that cannot gate
 *
 * Traits of this clock:
 * prepare - clk_prepare only ensures that parents are prepared
 * enable - clk_enable only ensures that parents are enabled
 * rate - rate is fixed.  clk->rate = parent->rate / div * mult
 * parent - fixed parent.  No clk_set_parent support
 */

static unsigned long clk_factor_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_mult_div *fix = to_clk_mult_div(hw);
	unsigned long long int rate;
	u32 reg;

	reg       = readl_relaxed(fix->base);
	fix->mult = (reg & (fix->mult_mask << fix->mult_shift)) >> fix->mult_shift;
	fix->div  = (reg & (fix->div_mask  << fix->div_shift )) >> fix->div_shift;
	if (fix->flags & CLK_SET_RATE_MUL_2) {
		fix->div  <<= 1;
	}

	rate = (unsigned long long int)parent_rate * fix->mult;
	do_div(rate, fix->div);
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_mult_div clk_factor_recalc_rate reg = 0x%x, fix->div = %d, fix->mult = %d\n", reg, fix->div, fix->mult);
	printk(KERN_DEBUG "clk_factor_recalc_rate rate = %lld, parent_rate = %ld\n", rate, parent_rate);
#endif
	return (unsigned long)rate;
}

static long clk_factor_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
#if 0
	struct clk_mult_div *fix = to_clk_mult_div(hw);
	u32 reg;

	reg       = readl_relaxed(fix->base);
	fix->mult = (reg & (fix->mult_mask << fix->mult_shift)) >> fix->mult_shift;
	fix->div  = (reg & (fix->div_mask  << fix->div_shift )) >> fix->div_shift;
	fix->div  <<= 1;


	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		unsigned long best_parent;

		best_parent = (rate / fix->mult) * fix->div;
		*prate = clk_hw_round_rate(clk_hw_get_parent(hw), best_parent);
	}

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_mult_div clk_factor_round_rate reg = 0x%x, fix->div = %d, fix->mult = %d\n", reg, fix->div, fix->mult);
	printk(KERN_DEBUG "clk_factor_round_rate rate = %ld, parent_rate = %ld\n", (*prate / fix->div) * fix->mult, *prate);
#endif
	return (*prate / fix->div) * fix->mult;
#else
	return rate;
#endif
}

/**
 *  Get the greatest common divisor
 */
static unsigned long getMax(unsigned long n, unsigned long m) {
	unsigned long tp, p;

	if (n < m){
		tp = n;
		n = m;
		m = tp;
	}

	p = n % m;
	while (p != 0) {
		n = m;
		m = p;
		p = n % m;
	}

	return m;
}

/**
 *  Get the divisor and multiplier
 */
static int getDiv(unsigned long *n, unsigned long *m) {
	unsigned long p;

	if (*m == 0)
		return -1;

	if (*n == 0)
		return 0;

	p = getMax(*n, *m);
	*n = *n / p;
	*m = *m / p;

	return 0;	
}

static int clk_factor_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_mult_div *fix = to_clk_mult_div(hw);
	unsigned long rate0, rate1;
	u32 reg;

	if (rate == 0) {
		return -1;
	}

	rate0 = rate;
	if (fix->flags & CLK_SET_RATE_MUL_2) {
		rate0 = rate0 << 1;
	}
	rate1 = parent_rate;
	getDiv(&rate0, &rate1);
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_mult_div clk_factor_set_rate rate0 = %ld, rate1 = %ld\n", rate0, rate1);
#endif
	fix->mult  = rate0;
	fix->div   = rate1;
	reg  = readl_relaxed(fix->base);
	reg &= ~(fix->mult_mask << fix->mult_shift);
	reg &= ~(fix->div_mask << fix->div_shift);
	reg |= fix->mult << fix->mult_shift;
	reg |= fix->div << fix->div_shift;
	if (!(fix->flags & CLK_SET_RATE_NO_WRITEABLE)) {
		reg |= 1 << (fix->mult_shift + 16);
		reg |= 1 << (fix->div_shift  + 16);
	}
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_mult_div clk_factor_set_rate reg = 0x%x, fix->div = %d, fix->mult = %d\n", reg, fix->div, fix->mult);
#endif
	writel_relaxed(reg, fix->base);
	return 0;
}

static const struct clk_ops clk_mul_div_ops = {
	.round_rate = clk_factor_round_rate,
	.set_rate = clk_factor_set_rate,
	.recalc_rate = clk_factor_recalc_rate,
};

/**
 * wokoo_hw_register_mult_divider - register multiplier
 * and divider clocks.
 * @dev: clock device
 * @name: clock name
 * @parent_name: parent clock name
 * @flags: CLK_SET_RATE_NO_WRITEABLE/CLK_SET_RATE_MUL_2
 * @reg: clock reg
 * @mult_shift: multiplier bit shift
 * @mult_mask: multiplier bits mask
 * @div_shift: divider bit shift
 * @div_mask: divider bits mask
 * @write_shift: writeable bit shift
 * @mult: multiplier value(no used)
 * @div: divider value(no used)
 *
 *
 * Returns success clk_hw.
 */
struct clk_hw *
wokoo_hw_register_mult_divider(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 mult_shift, u32 mult_mask, u32 div_shift, u32 div_mask,
		unsigned int mult, unsigned int div)
{
	struct clk_mult_div *fix;
	struct clk_init_data init = { };
	struct clk_hw *hw;
	int ret;

	fix = kmalloc(sizeof(*fix), GFP_KERNEL);
	if (!fix)
		return ERR_PTR(-ENOMEM);

	/* struct clk_fixed_factor assignments */
	fix->mult  = mult;
	fix->div   = div;
	fix->flags = flags;
	fix->mult_mask  = mult_mask;
	fix->mult_shift = mult_shift;
	fix->div_mask   = div_mask;
	fix->div_shift  = div_shift;
	fix->base       = reg;
	fix->hw.init    = &init;

	init.name = name;
	init.ops = &clk_mul_div_ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name: NULL);
	init.num_parents  = (parent_name ? 1 : 0);

	hw = &fix->hw;
	ret = clk_hw_register(NULL, hw);

	if (ret) {
		kfree(fix);
		hw = ERR_PTR(ret);
	}

	return hw;
}

