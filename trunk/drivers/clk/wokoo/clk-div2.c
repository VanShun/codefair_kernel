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
 * struct clk_div2
 * @hw: clk_hw
 * @div1: divider value
 * @div2: divider value 
 * @div1_start: div1 starting value 
 * @div2_start: div2 starting value
 * @flags: CLK_SET_RATE_NO_WRITEABLE
 * @base: clock reg base
 * @div1_mask: divider bits mask 
 * @div1_shift: divider bit shift
 * @div1_write_shift: writeable bit shift
 * @div2_mask: divider bits mask 
 * @div2_shift: divider bit shift
 * @div2_write_shift: writeable bit shift 
 */
struct clk_div2 {
	struct clk_hw	hw;
	unsigned int	div1;
	unsigned int	div2;
	unsigned int	div1_start;
	unsigned int	div2_start;
	unsigned long	flags;
	void __iomem	*base;
	u32		div1_mask;
	u32		div1_shift;
	u32		div1_write_shift;
	u32		div2_mask;
	u32		div2_shift;
	u32		div2_write_shift;
};




//#define WOKOO_DEBUG    1
#define to_clk_div2(_hw) container_of(_hw, struct clk_div2, hw)




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
	struct clk_div2 *fix = to_clk_div2(hw);
	unsigned long long int rate;
	u32 reg;

	reg       = readl_relaxed(fix->base);
	fix->div1 = (reg & (fix->div1_mask << fix->div1_shift)) >> fix->div1_shift;
	fix->div2 = (reg & (fix->div2_mask << fix->div2_shift)) >> fix->div2_shift;

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_recalc_rate reg = 0x%x, fix->div1 = %d, fix->div2 = %d\n", reg, fix->div1, fix->div2);
#endif

	rate = (unsigned long long int)parent_rate;

	/* First frequency division freq1 = prate * div1 / 8 */
	fix->div1 = 8 / fix->div1;
	if (fix->flags & CLK_SET_RATE_ADD_1_DIV) {
		fix->div2 = fix->div2 + 1;
	} else {
		/* Second frequency division freq2 = freq1 / (div2 + 0x03) */
		fix->div2 = fix->div2 + fix->div2_start;
	}

	do_div(rate, fix->div1);
	do_div(rate, fix->div2);
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_recalc_rate rate = %lld\n", rate);
#endif
	return (unsigned long)rate;
}

static long clk_factor_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
#if 0
	struct clk_div2 *fix = to_clk_div2(hw);
	u32 reg;

	reg       = readl_relaxed(fix->base);
	fix->div1 = (reg & (fix->div1_mask << fix->div1_shift)) >> fix->div1_shift;
	fix->div2 = (reg & (fix->div2_mask << fix->div2_shift)) >> fix->div2_shift;

	printk(KERN_DEBUG "clk_factor_round_rate reg = 0x%x, fix->div1 = %d, fix->div2 = %d\n", reg, fix->div1, fix->div2);

	/* First frequency division freq1 = prate * div1 / 8 */
	fix->div1 = 8 / fix->div1;
	/* Second frequency division freq2 = freq1 / (div2 + 0x03) */
	fix->div2 = fix->div2 + fix->div2_start;


	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		unsigned long best_parent;

		best_parent = (rate * fix->div1) * fix->div2;
		*prate = clk_hw_round_rate(clk_hw_get_parent(hw), best_parent);
	}

	printk(KERN_DEBUG "clk_factor_recalc_rate rate = %ld\n", (*prate / fix->div1) / fix->div2);
	return (*prate / fix->div1) / fix->div2;
#else
	return rate;
#endif
}

static int getMul(unsigned long *n, unsigned long *m, unsigned long min) {
	unsigned long p, i;

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "getMul n = %ld, m = %ld\n", *n, *m);
#endif
	if (*n > *m)
		return -1;

	/* Not allowed decimal */
	if (*m % *n)
		return -1;

	if (min == 0) {
		min = 1;
	}

	p = *m / *n;
	
	/* Not allowed decimal */
	for (i = 8; i > 0; i--) {
		if (p % i == 0) {
			*n = i;
			*m = p / i;
			if (*m >= min && !(8 % *n)) {
				break;
			}
		}
	}

	return 0;
}

static int clk_factor_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_div2 *fix = to_clk_div2(hw);
	unsigned long rate0, rate1;
	u32 reg;
	int ret = -1;

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_set_rate rate = %ld, parent_rate = %ld\n", rate, parent_rate);
#endif
	if (rate == 0) {
		return ret;
	}

	rate0 = rate;
	rate1 = parent_rate;
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_set_rate rate0 = %ld, rate1 = %ld\n", rate0, rate1);
#endif
	ret = getMul(&rate0, &rate1, fix->div2_start);
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "getMul rate0 = %ld, rate1 = %ld\n", rate0, rate1);
#endif
	if (ret == -1) {
		return ret;
	}

	fix->div1 = rate0;
	fix->div2 = rate1;

	/* First frequency division freq1 = prate * div1 / 8 */
	fix->div1 = 8 / fix->div1;
	if (fix->flags & CLK_SET_RATE_ADD_1_DIV) {
		fix->div2 = fix->div2 - 1;
	} else {
		/* Second frequency division freq2 = freq1 / (div2 + 0x03) */
		fix->div2 = fix->div2 - fix->div2_start;
	}

	reg  = readl_relaxed(fix->base);
	reg &= ~(fix->div1_mask << fix->div1_shift);
	reg &= ~(fix->div2_mask << fix->div2_shift);
	reg |= fix->div1 << fix->div1_shift;
	reg |= fix->div2 << fix->div2_shift;
	if (!(fix->flags & CLK_SET_RATE_NO_WRITEABLE)) {
		reg |= 1 << fix->div1_write_shift;
		reg |= 1 << fix->div2_write_shift;
	}
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_set_rate reg = 0x%x, fix->div1 = %d, fix->div2 = %d\n", reg, fix->div1, fix->div2);
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
 * wokoo_hw_register_divider2 - register divider2 clocks.
 * @dev: clock device
 * @name: clock name
 * @parent_name: parent clock name
 * @flags: CLK_SET_RATE_PARENT/CLK_SET_RATE_NO_WRITEABLE
 * @reg: clock reg
 * @div1_shift: divider bit shift
 * @div1_mask: divider bits mask
 * @div1_write_shift: writeable bit shift
 * @div2_shift: divider bit shift
 * @div2_mask: divider bits mask
 * @div2_write_shift: writeable bit shift
 *
 *
 * Returns success clk_hw.
 */
struct clk_hw *
wokoo_hw_register_divider2(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 div1_shift, u32 div1_mask, u32 div1_write_shift,
		u32 div2_shift, u32 div2_mask, u32 div2_write_shift,
		unsigned int div1_start, unsigned int div2_start)
{
	struct clk_div2 *fix;
	struct clk_init_data init = { };
	struct clk_hw *hw;
	int ret;

	fix = kmalloc(sizeof(*fix), GFP_KERNEL);
	if (!fix)
		return ERR_PTR(-ENOMEM);

	/* struct clk_fixed_factor assignments */
	fix->div1_start = div1_start;
	fix->div2_start = div2_start;
	fix->flags      = flags;
	fix->div1_mask  = div1_mask;
	fix->div1_shift = div1_shift;
	fix->div2_mask  = div2_mask;
	fix->div2_shift = div2_shift;
	fix->base       = reg;
	fix->hw.init    = &init;

	fix->div1_write_shift = div1_write_shift;
	fix->div2_write_shift = div2_write_shift;

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

