// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2010-2011 Canonical Ltd <jeremy.kerr@canonical.com>
 * Copyright (C) 2011-2012 Mike Turquette, Linaro Ltd <mturquette@linaro.org>
 *
 * Gated clock implementation
 */
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/string.h>
#include "clk.h"




/**
 * struct wokoo_clk_gate
 * @hw: clk_hw 
 * @reg: clock reg base
 * @bit_idx: bit shift
 * @bit_w_mask: writeable bit shift
 * @flags: CLK_GATE_SET_TO_DISABLE 
 *         CLK_GATE_HIWORD_MASK
 * @lock: spin lock
 * 
 */
struct wokoo_clk_gate {
	struct clk_hw hw;
	void __iomem	*reg;
	u8		bit_idx;
	u8		bit_w_mask;
	u8		flags;
	spinlock_t	*lock;
	const char *name;
};



//#define WOKOO_DEBUG    1
#define to_wokoo_clk_gate(_hw) container_of(_hw, struct wokoo_clk_gate, hw)

/**
 * DOC: basic gatable clock which can gate and ungate it's ouput
 *
 * Traits of this clock:
 * prepare - clk_(un)prepare only ensures parent is (un)prepared
 * enable - clk_enable and clk_disable are functional & control gating
 * rate - inherits rate from parent.  No clk_set_rate support
 * parent - fixed parent.  No clk_set_parent support
 */

static inline u32 clk_gate_readl(struct wokoo_clk_gate *gate)
{
	return readl(gate->reg);
}

static inline void clk_gate_writel(struct wokoo_clk_gate *gate, u32 val)
{
	writel(val, gate->reg);
}

/*
 * It works on following logic:
 *
 * For enabling clock, enable = 1
 *	set2dis = 1	-> clear bit	-> set = 0
 *	set2dis = 0	-> set bit	-> set = 1
 *
 * For disabling clock, enable = 0
 *	set2dis = 1	-> set bit	-> set = 1
 *	set2dis = 0	-> clear bit	-> set = 0
 *
 * So, result is always: enable xor set2dis.
 */
static void wokoo_clk_gate_endisable(struct clk_hw *hw, int enable)
{
	struct wokoo_clk_gate *gate = to_wokoo_clk_gate(hw);
	int set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	unsigned long uninitialized_var(flags);
	u32 reg;

	set ^= enable;

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);
	else
		__acquire(gate->lock);

	reg = clk_gate_readl(gate);
	if (gate->flags & CLK_GATE_HIWORD_MASK) {
		reg |= BIT(gate->bit_w_mask);
	}

	if (set)
		reg |= BIT(gate->bit_idx);
	else
		reg &= ~BIT(gate->bit_idx);

	clk_gate_writel(gate, reg);
#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "wokoo_clk_gate_endisable %s, set = %d, reg = %x\n", gate->name, set, reg);
#endif

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
	else
		__release(gate->lock);
}

static int wokoo_clk_gate_enable(struct clk_hw *hw)
{
	wokoo_clk_gate_endisable(hw, 1);

	return 0;
}

static void wokoo_clk_gate_disable(struct clk_hw *hw)
{
	wokoo_clk_gate_endisable(hw, 0);
}

static int wokoo_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct wokoo_clk_gate *gate = to_wokoo_clk_gate(hw);

	reg = clk_gate_readl(gate);

	/* if a set bit disables this clk, flip it before masking */
	if (gate->flags & CLK_GATE_SET_TO_DISABLE)
		reg ^= BIT(gate->bit_idx);

	reg &= BIT(gate->bit_idx);

#ifdef WOKOO_DEBUG
	printk(KERN_DEBUG "wokoo_clk_gate_is_enabled reg = %d\n", reg);
#endif

	return reg ? 1 : 0;
}

static struct clk_ops wokoo_clk_gate_ops = {
	.enable = wokoo_clk_gate_enable,
	.disable = wokoo_clk_gate_disable,
	.is_enabled = wokoo_clk_gate_is_enabled,
};

/**
 * wokoo_clk_hw_register_gate - register a gate clock with the 
 * clock framework 
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of this clock's parent
 * @flags: framework-specific flags for this clock
 * @reg: register address to control gating of this clock
 * @bit_idx: which bit in the register controls gating of this clock 
 * @bit_w_mask: which bit controls writeable of this clock
 * @clk_gate_flags: gate-specific flags for this clock
 * @lock: shared register lock for this clock
 */
struct clk_hw *wokoo_clk_hw_register_gate(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 bit_idx, u8 bit_w_mask,
		u8 clk_gate_flags, spinlock_t *lock)
{
	struct wokoo_clk_gate *gate;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	/* allocate the gate */
	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &wokoo_clk_gate_ops;
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	/* struct clk_gate assignments */
	gate->reg = reg;
	gate->bit_idx = bit_idx;
	gate->bit_w_mask = bit_w_mask;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->hw.init = &init;
	gate->name = name;

	hw = &gate->hw;
	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(gate);
		hw = ERR_PTR(ret);
	}

	return hw;
}


