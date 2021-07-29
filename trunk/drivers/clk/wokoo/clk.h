/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */
#ifndef __MACH_WOKOO_CLK_H
#define __MACH_WOKOO_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>





extern spinlock_t wokoo_ccm_lock;

void wokoo_check_clocks(struct clk *clks[], unsigned int count);
void wokoo_check_clk_hws(struct clk_hw *clks[], unsigned int count);
void wokoo_unregister_clocks(struct clk *clks[], unsigned int count);





/* Set reg no need to set writeable bit */
#define CLK_SET_RATE_NO_WRITEABLE       BIT(24)
/* Set clock rate : div = (MASK + 1) / (reg_value + 1)  */
#define CLK_SET_RATE_DIV_ADD_1          BIT(25)
/* Set clock rate : div = reg_value + 1  */
#define CLK_SET_RATE_ADD_1_DIV          BIT(26)
/* Set clock rate : mul = 2 * (reg_value + 1)  */
#define CLK_SET_RATE_DIV_MUL_2          BIT(27)
#define CLK_SET_RATE_MUL_2              CLK_SET_RATE_DIV_MUL_2
/* Reset machine reg */
#define WOKOO_GLB_SFRST_CTL             0x180
/* Reset devices reg */
#define WOKOO_SOFTRST_CON               0x184
#define WOKOO_SOFTRST_NUM               14

#define WOKOO_SOFTRST_HIWORD_MASK       BIT(0)





/* NOTE: Rate table should be kept sorted in descending order. */
struct wokoo_pll_rate_table {
	unsigned long rate;
	unsigned int nr;
	unsigned int nf;
	unsigned int no;
	unsigned int nb;
	unsigned int fbdiv;
	unsigned int postdiv1;
	unsigned int refdiv;
	unsigned int postdiv2;
	unsigned int dsmpd;
	unsigned int frac;
};

enum wokoo_pll_type {
	WOKOO_PLL_GENERIC,
	WOKOO_PLL_SYS,
	WOKOO_PLL_USB,
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
				struct wokoo_pll_rate_table *rate_table);

struct clk *wokoo_clk_register_gate(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 bit_idx,
		u8 bit_w_mask, u8 clk_gate_flags, spinlock_t *lock);

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
struct clk_hw *wokoo_hw_register_mult_divider(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 mult_shift, u32 mult_mask, u32 div_shift, u32 div_mask,
		unsigned int mult, unsigned int div);

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
struct clk_hw *wokoo_hw_register_divider2(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 div1_shift, u32 div1_mask, u32 div1_write_shift,
		u32 div2_shift, u32 div2_mask, u32 div2_write_shift,
		unsigned int div1_start, unsigned int div2_start);

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
struct clk_hw *wokoo_hw_register_divider(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 div_shift, u32 div_mask,
		unsigned int write_shift, unsigned int div);

/**
 * wokoo_hw_register_fixup_mult_divider - register fixup
 * multiplier and divider clocks.
 * @dev: clock device
 * @name: clock name
 * @parent_name: parent clock name
 * @flags: CLK_SET_RATE_NO_WRITEABLE
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
struct clk_hw *clk_hw_register_fixup_mult_divider(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 mult_shift, u32 mult_mask, u32 div_shift, u32 div_mask,
		unsigned int mult, unsigned int div);

#define WOKOO_PLL_RATE(_rate, _refdiv, _fbdiv, _postdiv1,	\
			_postdiv2, _dsmpd, _frac)		\
{								\
	.rate	= _rate##U,					\
	.fbdiv = _fbdiv,					\
	.postdiv1 = _postdiv1,					\
	.refdiv = _refdiv,					\
	.postdiv2 = _postdiv2,					\
	.dsmpd = _dsmpd,					\
	.frac = _frac,						\
}

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
		u8 clk_gate_flags, spinlock_t *lock);

static inline struct clk_hw *wokoo_clk_hw_fixed_factor(const char *name,
		const char *parent, unsigned int mult, unsigned int div)
{
	return clk_hw_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

static inline struct clk_hw *wokoo_clk_hw_gate(const char *name, const char *parent,
							void __iomem *reg, u8 shift, u8 w_shift)
{
	return wokoo_clk_hw_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
					shift, w_shift, CLK_GATE_HIWORD_MASK, &wokoo_ccm_lock);
}

static inline struct clk_hw *wokoo_clk_hw_mux(const char *name, void __iomem *reg,
			u8 shift, u8 width, const char * const *parents,
			int num_parents)
{
	return clk_hw_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0, &wokoo_ccm_lock);
}

/**
 * wokoo_register_restart_notifier - restart funtion register
 * @base: restart reg base
 * @reg: restart reg
 * @cb: callback
 */
void wokoo_register_restart_notifier(void __iomem *base, unsigned int reg, void (*cb)(void));

#ifdef CONFIG_RESET_CONTROLLER
/** 
 * wokoo_register_softrst - soft reset deassert 
 * @np: device node 
 * @num_regs: number of regs 
 * @base: base of regs 
 * @flag: WOKOO_SOFTRST_HIWORD_MASK
 */
void __init wokoo_register_softrst(struct device_node *np,
					unsigned int num_regs,
					void __iomem *base, u8 flags);
#else
static inline void wokoo_register_softrst(struct device_node *np,
					unsigned int num_regs,
					void __iomem *base, u8 flags)
{
}
#endif

#endif
