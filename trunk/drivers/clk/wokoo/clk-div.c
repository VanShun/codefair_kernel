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
 * struct clk_div
 * @hw: clk_hw
 * @div: divider value
 * @flags: CLK_SET_RATE_PARENT/CLK_SET_RATE_DIV_ADD_1
 *         CLK_SET_RATE_DIV_ADD_1/CLK_SET_RATE_MUL_2
 * @base: clock reg base
 * @div_mask: divider bits mask
 * @div_shift: divider bit shift
 * @write_shift: writeable bit shift
 * @div: divider value
 * 
 */
struct clk_div {
	struct clk_hw	hw;
	unsigned int	div;
	unsigned long	flags;
	void __iomem	*base;
	u32		div_mask;
	u32		div_shift;
	u32		write_shift;
};





//#define WOKOO_DEBUG    1
#define to_clk_div(_hw) container_of(_hw, struct clk_div, hw)




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
	struct clk_div *fix = to_clk_div(hw);
	unsigned long long int rate;
	u32 reg;

	reg      = readl_relaxed(fix->base);
	fix->div = (reg & (fix->div_mask << fix->div_shift)) >> fix->div_shift;

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_recalc_rate reg = %x, fix->div = %d\n", reg, fix->div);
#endif

	if (fix->flags & CLK_SET_RATE_DIV_ADD_1) {
		fix->div = (fix->div_mask + 1) / (fix->div + 1);
	} else if (fix->flags & CLK_SET_RATE_ADD_1_DIV) {
		/* Frequency division freq = (div + 1) */
		fix->div = (fix->div + 1);
	} else if (fix->flags & CLK_SET_RATE_MUL_2) {
		/* Frequency division freq = 2 * (div + 1) */
		fix->div = 2 * (fix->div + 1);
	} else {
		if (fix->div == 0) {
			return 0;
		} else {
			/* Frequency division freq = prate * div / 8 */
			//fix->div = (fix->div_mask + 1) / 2 / fix->div;
			rate = (parent_rate / 1000) * fix->div / ((fix->div_mask + 1) / 2);
			return (unsigned long)rate * 1000;
		}
	}

	rate = (unsigned long long int)parent_rate;
	do_div(rate, fix->div);
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_recalc_rate rate = %lld, parent_rate = %ld, fix->div = %d\n", rate, parent_rate, fix->div);
#endif
	return (unsigned long)rate;
}

static int clk_factor_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_div *fix = to_clk_div(hw);
	u32 reg;
	int ret = -1;

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_set_rate parent_rate = %ld, rate = %ld\n", parent_rate, rate);
#endif
	if (rate == 0) {
		return ret;
	}

	fix->div = parent_rate / rate;

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_set_rate fix->div = %d\n", fix->div);
#endif
	if (fix->div == 0) {
		return ret;
	}

	if (fix->flags & CLK_SET_RATE_DIV_ADD_1) {
		fix->div = ((fix->div_mask + 1) / fix->div) - 1;
	} else if (fix->flags & CLK_SET_RATE_ADD_1_DIV) {
		/* Frequency division freq = (div + 1) */
		fix->div = (fix->div - 1);
	} else if (fix->flags & CLK_SET_RATE_MUL_2) {
		/* Frequency division freq = 2 * (div + 1) */
		fix->div = (fix->div / 2) - 1;
	} else {
		/* Frequency division freq = prate * div / 8 */
		//fix->div = (fix->div_mask + 1) / 2 / fix->div;
		unsigned long ratex = (rate / 1000) * (fix->div_mask + 1) / 2;
		fix->div = ratex / (parent_rate / 1000);
	}

	reg  = readl_relaxed(fix->base);
	reg &= ~(fix->div_mask << fix->div_shift);
	reg |= fix->div << fix->div_shift;
	if (!(fix->flags & CLK_SET_RATE_NO_WRITEABLE)) {
		reg |= 1 << fix->write_shift;
	}
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_set_rate reg = 0x%x, fix->div = %d\n", reg, fix->div);
#endif
	writel_relaxed(reg, fix->base);
	return 0;
}

static long clk_factor_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
#if 0
	struct clk_div *fix = to_clk_div(hw);
	u32 reg;

	reg      = readl_relaxed(fix->base);
	fix->div = (reg & (fix->div_mask << fix->div_shift)) >> fix->div_shift;
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_round_rate reg = 0x%x, fix->div = %d\n", reg, fix->div);
#endif

	if (fix->flags & CLK_SET_RATE_DIV_ADD_1) {
		/* Frequency division freq = (max + 1) / (div + 1) */
		fix->div = (fix->div_mask + 1) / (fix->div + 1);
	} else if (fix->flags & CLK_SET_RATE_ADD_1_DIV) {
		/* Frequency division freq = (div + 1) */
		fix->div = (fix->div + 1);
	} else if (fix->flags & CLK_SET_RATE_MUL_2) {
		/* Frequency division freq = 2 * (div + 1) */
		fix->div = 2 * (fix->div + 1);
	} else {
		if (fix->div == 0) {
			return *prate;
		}
		/* Frequency division freq = prate * div / 8 */
		unsigned long ratex = (rate / 1000) * (fix->div_mask + 1) / 2;
		fix->div = ratex / (parent_rate / 1000);
	}


	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		unsigned long best_parent;

		best_parent = (rate * fix->div);
		*prate = clk_hw_round_rate(clk_hw_get_parent(hw), best_parent);
	}

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "clk_factor_round_rate rate = %ld, parent_rate = %ld, fix->div = %d\n", rate, *prate, fix->div);
#endif
	return (*prate / fix->div);
#else
	return rate;
#endif
}

static const struct clk_ops clk_div_ops = {
	.round_rate = clk_factor_round_rate,
	.set_rate = clk_factor_set_rate,
	.recalc_rate = clk_factor_recalc_rate,
};


/**
 * wokoo_hw_register_divider - register divider clocks.
 * @dev: clock device
 * @name: clock name
 * @parent_name: parent clock name
 * @flags: CLK_SET_RATE_PARENT/CLK_SET_RATE_DIV_ADD_1
 *         CLK_SET_RATE_DIV_ADD_1/CLK_SET_RATE_MUL_2
 * @reg: clock reg
 * @div_shift: divider bit shift
 * @div_mask: divider bits mask
 * @write_shift: writeable bit shift
 * @div: divider value
 *
 *
 * Returns success clk_hw.
 */
struct clk_hw *
wokoo_hw_register_divider(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 div_shift, u32 div_mask,
		unsigned int write_shift, unsigned int div)
{
	struct clk_div *fix;
	struct clk_init_data init = { };
	struct clk_hw *hw;
	int ret;

	fix = kmalloc(sizeof(*fix), GFP_KERNEL);
	if (!fix)
		return ERR_PTR(-ENOMEM);

	/* struct clk_fixed_factor assignments */
	fix->div   = div;
	fix->flags = flags;
	fix->div_mask  = div_mask;
	fix->div_shift = div_shift;
	fix->write_shift = write_shift;
	fix->base       = reg;
	fix->hw.init    = &init;

	init.name = name;
	init.ops = &clk_div_ops;
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

