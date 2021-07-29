/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>
#include "clk.h"




/**
 * struct wokoo_softrst - soft reset
 * @rcdev: reset controller device
 * @reg_base: base address of reset registers
 * @num_regs: number regs
 * @num_per_reg: number devices of regs
 * @flags: WOKOO_SOFTRST_HIWORD_MASK
 * @lock: spin clock
 *
 */
struct wokoo_softrst {
	struct reset_controller_dev	rcdev;
	void __iomem    *reg_base;
	int				num_regs;
	int				num_per_reg;
	u8				flags;
	spinlock_t		lock;
};




/** 
 * wokoo_softrst_assert - soft reset assert
 */
static int wokoo_softrst_assert(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct wokoo_softrst *softrst = container_of(rcdev,
							struct wokoo_softrst,
							rcdev);
	int bank = id / softrst->num_per_reg;
	int offset = id % softrst->num_per_reg;

	if (softrst->flags & WOKOO_SOFTRST_HIWORD_MASK) {
		writel(BIT(offset) | (BIT(offset) << 16),
				softrst->reg_base + (bank * 4));
	} else {
		unsigned long flags;
		u32 reg;

		spin_lock_irqsave(&softrst->lock, flags);

		reg = readl(softrst->reg_base + (bank * 4));
		writel(reg | BIT(offset), softrst->reg_base + (bank * 4));

		spin_unlock_irqrestore(&softrst->lock, flags);
	}

	return 0;
}

/** 
 * wokoo_softrst_deassert - soft reset deassert
 */
static int wokoo_softrst_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct wokoo_softrst *softrst = container_of(rcdev,
							struct wokoo_softrst,
							rcdev);
	int bank = id / softrst->num_per_reg;
	int offset = id % softrst->num_per_reg;

	if (softrst->flags & WOKOO_SOFTRST_HIWORD_MASK) {
		writel((BIT(offset) << 16), softrst->reg_base + (bank * 4));
	} else {
		unsigned long flags;
		u32 reg;

		spin_lock_irqsave(&softrst->lock, flags);

		reg = readl(softrst->reg_base + (bank * 4));
		writel(reg & ~BIT(offset), softrst->reg_base + (bank * 4));

		spin_unlock_irqrestore(&softrst->lock, flags);
	}

	return 0;
}

static const struct reset_control_ops wokoo_softrst_ops = {
	.assert		= wokoo_softrst_assert,
	.deassert	= wokoo_softrst_deassert,
};


/** 
 * wokoo_register_softrst - soft reset deassert 
 * @np: device node 
 * @num_regs: number of regs 
 * @base: base of regs 
 * @flag: WOKOO_SOFTRST_HIWORD_MASK
 */
void __init wokoo_register_softrst(struct device_node *np,
					unsigned int num_regs,
					void __iomem *base, u8 flags)
{
	struct wokoo_softrst *softrst;
	int ret;

	softrst = kzalloc(sizeof(*softrst), GFP_KERNEL);
	if (!softrst)
		return;

	spin_lock_init(&softrst->lock);

	softrst->reg_base = base;
	softrst->flags = flags;
	softrst->num_regs = num_regs;
	softrst->num_per_reg = (flags & WOKOO_SOFTRST_HIWORD_MASK) ? 16 : 32;

	softrst->rcdev.owner = THIS_MODULE;
	softrst->rcdev.nr_resets =  num_regs * softrst->num_per_reg;
	softrst->rcdev.ops = &wokoo_softrst_ops;
	softrst->rcdev.of_node = np;
	ret = reset_controller_register(&softrst->rcdev);
	if (ret) {
		pr_err("%s: could not register reset controller, %d\n",
				__func__, ret);
		kfree(softrst);
	}
};
