/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */
#include <dt-bindings/clock/wokoo-clock.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include "clk.h"





static struct clk_hw **hws;
static struct clk_hw_onecell_data *clk_hw_data;

/* pll_out rate table */
static struct wokoo_pll_rate_table wokoo_pll_rates[] = {
	/* _mhz, _refdiv, _fbdiv, _postdiv1, _postdiv2, _dsmpd, _frac */
	WOKOO_PLL_RATE(1000000000, 1, 125, 1, 1, 1, 0),
	WOKOO_PLL_RATE(960000000, 1, 120, 1, 1, 1, 0),
	WOKOO_PLL_RATE(800000000, 1, 100, 1, 1, 1, 0),
	WOKOO_PLL_RATE(768000000, 1, 96, 1, 1, 1, 0),
	WOKOO_PLL_RATE(560000000, 1, 70, 1, 1, 1, 0),
	WOKOO_PLL_RATE(480000000, 1, 60, 1, 1, 1, 0),
	WOKOO_PLL_RATE(400000000, 1, 50, 1, 1, 1, 0),
	WOKOO_PLL_RATE(320000000, 1, 40, 1, 1, 1, 0),
	WOKOO_PLL_RATE(160000000, 1, 20, 2, 1, 1, 0),
	{ /* sentinel */ },
};

/* timer parent clock table */
static const char *timer_pre_sels[]  = { "bus_mclk", "pll_mclk", "ctl_pclk", "func_mclk", "osc32k"};
/* clock out parent clock table */
static const char *clkout_pre_sels[] = { "osc32k", "func_mclk", "pll_mclk", "bus_mclk"};




/*
 * wokoo_check_clk_hws - check clks
 */
void wokoo_check_clk_hws(struct clk_hw *clks[], unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		if (IS_ERR(clks[i]))
			pr_err("WOKOO clk %u: register failed with %ld\n",
					i, PTR_ERR(clks[i]));
}

/*
 * wokoo_clocks_init - clks register
 */
static void __init wokoo_clocks_init(struct device_node *np)
{
	void __iomem *base;

	clk_hw_data = kzalloc(struct_size(clk_hw_data, hws,
						WOKOO_CLK_END), GFP_KERNEL);
	if (WARN_ON(!clk_hw_data))
		return;

	clk_hw_data->num = WOKOO_CLK_END;
	hws = clk_hw_data->hws;

	hws[WOKOO_CLK_DUMMY]  = clk_hw_register_fixed_rate(NULL, "dummy", NULL, 0, 0);
	hws[WOKOO_CLK_OSC32K] = __clk_get_hw(of_clk_get_by_name(np, "osc32k"));
	hws[WOKOO_CLK_OSC16M] = __clk_get_hw(of_clk_get_by_name(np, "osc16m"));

	base = of_iomap(np, 0);
	of_node_put(np);
	WARN_ON(!base);

	/* pll out 768M, u-boot setting value */
	hws[WOKOO_CLK_PLLOUT] = wokoo_clk_hw_pll(WOKOO_PLL_GENERIC,	 "pll_out", "osc16m", base + 0x30, wokoo_pll_rates);

	/* A7_CLK, A7_CLK_GR[0~4] = 0x2, div = 16 / A7_CLK_GR = 8, write able = BIT(16)*/
	hws[WOKOO_SCLK_A7]     = wokoo_hw_register_divider(NULL, "a7_clk", "pll_out", 0, base + 0x44, 0, 0x1f, 16, 8);
	/* A7_PCLK_DBG, A7_PCLKDBG_GR[8~11] = 0x2, div = 8 / A7_CLK_GR = 4, write able = BIT(17)*/
	hws[WOKOO_PCLK_A7_DBG] = wokoo_hw_register_divider(NULL, "a7_pclk_dbg", "a7_clk", 0, base + 0x44, 8, 0xf, 17, 4);
	hws[WOKOO_ATCLK_A7]    = wokoo_hw_register_divider(NULL, "a7_atclk", "a7_clk", 0, base + 0x44, 12, 0xf, 18, 4);

	/* A7_AXI, CLK_GR[0~4] = 0x4 write bit = BIT(16), CLK_DR[4~5] = 0x3, write bit = BIT(17), div1_start = 0, div2_start = 0x03 */
	hws[WOKOO_SCLK_A7_AXI] = wokoo_hw_register_divider2(NULL, "a7_axi_clk", "pll_out", 0, base + 0x4c, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);
	/* A7_AXI_GPV_CLK_EN[8], BIT_WE=18 */
	hws[WOKOO_CLK_A7_AXI_GPV] = wokoo_clk_hw_gate("a7_axi_gpv_clk", "a7_axi_clk", base + 0x4c, 8, 18);

	/* BUS_MCLK */
	hws[WOKOO_MCLK_BUS]	= wokoo_hw_register_divider2(NULL, "bus_mclk", "pll_out", 0, base + 0x50, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);
#if 0
	/* If not used, kernel exec clk_disable() default, so can`t config those clks */
	hws[WOKOO_HCLK_CTL]         = wokoo_clk_hw_gate("ctl_hclk", "bus_mclk", base + 0x54, 12, 28);
	hws[WOKOO_CLK_AXI_AHB]      = wokoo_clk_hw_gate("axi_ahb_clk", "bus_mclk", base + 0x54, 4, 20);
	hws[WOKOO_CLK_PERI_BUS_GPV] = wokoo_clk_hw_gate("peri_bus_gpv_clk", "bus_mclk", base + 0x54, 1, 17);
	hws[WOKOO_CLK_PERI_BUS]     = wokoo_clk_hw_gate("peri_bus_clk", "bus_mclk", base + 0x54, 0, 16);
	hws[WOKOO_ACLK_A7_AXI_PERI] = wokoo_clk_hw_gate("a7_axi_peri_aclk", "bus_mclk", base + 0x58, 11, 27);
	hws[WOKOO_HCLK_PBR]         = wokoo_clk_hw_gate("pbr_hclk", "bus_mclk", base + 0x58, 9, 25);
#endif

	hws[WOKOO_CLK_AP_DMAG]      = wokoo_clk_hw_gate("ap_dmag_clk", "bus_mclk", base + 0x54, 11, 27);
	hws[WOKOO_CLK_AP_DMA_BUS]   = wokoo_clk_hw_gate("ap_dma_bus_clk", "bus_mclk", base + 0x54, 5, 21);
	hws[WOKOO_CLK_DMA_AXI]      = wokoo_clk_hw_gate("dma_axi_clk", "bus_mclk", base + 0x54, 2, 18);

	hws[WOKOO_HCLK_UAC]         = wokoo_clk_hw_gate("uac_hclk", "bus_mclk", base + 0x58, 8, 24);
	hws[WOKOO_HCLK_LCDC]        = wokoo_clk_hw_gate("lcdc_hclk", "bus_mclk", base + 0x58, 7, 23);
	hws[WOKOO_HCLK_BOOTROM]     = wokoo_clk_hw_gate("bootrom_hclk", "bus_mclk", base + 0x58, 6, 22);
	hws[WOKOO_HCLK_CIPHER]      = wokoo_clk_hw_gate("cipher_hclk", "bus_mclk", base + 0x58, 5, 21);
	//hws[WOKOO_HCLK_SEC_RAM]     = wokoo_clk_hw_gate("sec_ram_hclk", "bus_mclk", base + 0x58, 4, 20);
	hws[WOKOO_HCLK_SECURITY]    = wokoo_clk_hw_gate("security_hclk", "bus_mclk", base + 0x58, 3, 19);
	hws[WOKOO_HCLK_USBOTG]      = wokoo_clk_hw_gate("usbotg_hclk", "bus_mclk", base + 0x58, 0, 16);
	hws[WOKOO_CLK_ISP_BUS_LP]   = wokoo_clk_hw_gate("isp_bus_lp_clk", "bus_mclk", base + 0x5c, 12, 28);
	//hws[WOKOO_CLK_DDR_AXI_BUS_LP] = wokoo_clk_hw_gate("ddr_axi_bus_lp_clk", "bus_mclk", base + 0x5c, 11, 27);
	//hws[WOKOO_CLK_A7_AXI_BUS_LP]  = wokoo_clk_hw_gate("a7_axi_bus_lp_clk", "bus_mclk", base + 0x5c, 8, 24);
	//hws[WOKOO_CLK_SYS_BUS_LP]     = wokoo_clk_hw_gate("sys_bus_lp_clk", "bus_mclk", base + 0x5c, 7, 23);

	/* PLL_MCLK = pll_out / 4 */
	hws[WOKOO_MCLK_PLL]	  = wokoo_clk_hw_fixed_factor("pll_mclk", "pll_out", 1, 4);

	/* FUNC_MCLK */
	hws[WOKOO_MCLK_FUNC]  = wokoo_clk_hw_fixed_factor("func_mclk", "osc16m", 1, 1);

	/* APB_CLK */
	hws[WOKOO_PCLK_CTL_WORK] = wokoo_hw_register_divider(NULL, "ctl_pclk", "bus_mclk", CLK_SET_RATE_DIV_ADD_1, base + 0x64, 0, 0x7, 16, 2);
	hws[WOKOO_PCLK_CTL_IDLE] = wokoo_hw_register_divider(NULL, "ctl_pclk_idle", "bus_mclk", CLK_SET_RATE_DIV_ADD_1, base + 0x64, 4, 0x7, 17, 2);
	hws[WOKOO_PCLK_BP147]    = wokoo_clk_hw_gate("bp147_pclk", "ctl_pclk", base + 0xb4, 15, 31);
	hws[WOKOO_PCLK_RTC]      = wokoo_clk_hw_gate("rtc_pclk", "ctl_pclk", base + 0xb4, 14, 30);
	hws[WOKOO_PCLK_TPZCTL]   = wokoo_clk_hw_gate("tpzctl_pclk", "ctl_pclk", base + 0xb4, 13, 29);
	hws[WOKOO_PCLK_QSPI]     = wokoo_clk_hw_gate("qspi_pclk", "ctl_pclk", base + 0xb4, 12, 28);
	hws[WOKOO_PCLK_GPIO0]    = wokoo_clk_hw_gate("gpio0_pclk", "ctl_pclk", base + 0xb4, 11, 27);
	hws[WOKOO_PCLK_I2S1]     = wokoo_clk_hw_gate("i2s1_pclk", "ctl_pclk", base + 0xb4, 10, 26);
	hws[WOKOO_PCLK_TIMER]    = wokoo_clk_hw_gate("timer_pclk", "ctl_pclk", base + 0xb4, 9, 25);
	hws[WOKOO_PCLK_WDT0]     = wokoo_clk_hw_gate("wdt0_pclk", "ctl_pclk", base + 0xb4, 8, 24);
	hws[WOKOO_PCLK_PWM]      = wokoo_clk_hw_gate("pwm_pclk", "ctl_pclk", base + 0xb4, 7, 23);
	hws[WOKOO_PCLK_UART2]    = wokoo_clk_hw_gate("uart2_pclk", "ctl_pclk", base + 0xb4, 6, 22);
	hws[WOKOO_PCLK_UART1]    = wokoo_clk_hw_gate("uart1_pclk", "ctl_pclk", base + 0xb4, 5, 21);
	hws[WOKOO_PCLK_UART0]    = wokoo_clk_hw_gate("uart0_pclk", "ctl_pclk", base + 0xb4, 4, 20);
	hws[WOKOO_PCLK_SSI0]     = wokoo_clk_hw_gate("ssi0_pclk", "ctl_pclk", base + 0xb4, 3, 19);
	hws[WOKOO_PCLK_I2C1]     = wokoo_clk_hw_gate("i2c1_pclk", "ctl_pclk", base + 0xb4, 2, 18);
	hws[WOKOO_PCLK_I2C0]     = wokoo_clk_hw_gate("i2c0_pclk", "ctl_pclk", base + 0xb4, 1, 17);
	hws[WOKOO_PCLK_I2S0]     = wokoo_clk_hw_gate("i2s0_pclk", "ctl_pclk", base + 0xb4, 0, 16);
	hws[WOKOO_PCLK_ISP]      = wokoo_clk_hw_gate("isp_pclk", "ctl_pclk", base + 0xb8, 4, 20);
	/* Do not register DDR clock */
	//hws[WOKOO_PCLK_DDR_PWR]  = wokoo_clk_hw_gate("ddr_pwr_pclk", "ctl_pclk", base + 0xb8, 3, 19);
	hws[WOKOO_PCLK_MUXPIN]   = wokoo_clk_hw_gate("muxpin_pclk", "ctl_pclk", base + 0xb8, 2, 18);
	//hws[WOKOO_PCLK_MCTL_DDR] = wokoo_clk_hw_gate("ddr_mctl_pclk", "ctl_pclk", base + 0xb8, 1, 17);
	//hws[WOKOO_PCLK_MPHY_DDR] = wokoo_clk_hw_gate("ddr_mphy_pclk", "ctl_pclk", base + 0xb8, 0, 16);
	hws[WOKOO_MCLK_PWM]      = wokoo_clk_hw_gate("pwm_mclk", "ctl_pclk", base + 0xc4, 4, 12);
	hws[WOKOO_MCLK_I2C1]     = wokoo_clk_hw_gate("i2c1_mclk", "ctl_pclk", base + 0xc4, 2, 10);
	hws[WOKOO_MCLK_I2C0]     = wokoo_clk_hw_gate("i2c0_mclk", "ctl_pclk", base + 0xc4, 1, 9);

	/* UAC_RNG_SLOW_CLK */
	hws[WOKOO_SCLK_UAC_RNG_SLOW] = wokoo_hw_register_divider2(NULL, "uac_rng_slow_clk", "pll_mclk", 0, base + 0x70, 0, 0xf, 16, 4, 0x1f, 17, 0, 0x01);
	/* UAC_CLK */
	hws[WOKOO_SCLK_UAC_SUB]      = wokoo_hw_register_divider2(NULL, "uac_sub_clk", "pll_out", 0, base + 0x74, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);
	hws[WOKOO_CLK_UAC_RNG_SLOW]  = wokoo_clk_hw_gate("uac_rng_slow_clk_en", "uac_rng_slow_clk", base + 0x70, 12, 18);
	hws[WOKOO_CLK_UAC_SM4]       = wokoo_clk_hw_gate("uac_sm4_clk", "uac_sub_clk", base + 0x74, 15, 25);
	hws[WOKOO_CLK_UAC_SM3]       = wokoo_clk_hw_gate("uac_sm3_clk", "uac_sub_clk", base + 0x74, 14, 24);
	hws[WOKOO_CLK_UAC_SM1]       = wokoo_clk_hw_gate("uac_sm1_clk", "uac_sub_clk", base + 0x74, 13, 23);
	hws[WOKOO_CLK_UAC_PKI]       = wokoo_clk_hw_gate("uac_pki_clk", "uac_sub_clk", base + 0x74, 12, 22);
	hws[WOKOO_CLK_UAC_HRNG]      = wokoo_clk_hw_gate("uac_hrng_clk", "uac_sub_clk", base + 0x74, 11, 21);
	hws[WOKOO_CLK_UAC_DES]       = wokoo_clk_hw_gate("uac_des_clk", "uac_sub_clk", base + 0x74, 10, 20);
	hws[WOKOO_CLK_UAC_ALG]       = wokoo_clk_hw_gate("uac_alg_clk", "uac_sub_clk", base + 0x74, 9, 19);
	hws[WOKOO_CLK_UAC_AES]       = wokoo_clk_hw_gate("uac_aes_clk", "uac_sub_clk", base + 0x74, 8, 18);

	/* ARITH_CLK */
	hws[WOKOO_MCLK_ARITH] = wokoo_hw_register_divider2(NULL, "arith_mclk", "pll_out", 0, base + 0x74, 0, 0xf, 16, 4,  0x3, 17, 0, 0x03);
	hws[WOKOO_HCLK_ARITH] = wokoo_hw_register_divider2(NULL, "arith_hclk", "pll_out", 0, base + 0x74, 8, 0xf, 18, 12, 0x3, 19, 0, 0x03);
	hws[WOKOO_CLK_ARITH]  = wokoo_clk_hw_gate("arith_clk", "arith_mclk", base + 0x7c, 4, 12);

	/* DDR_AXI_CLK */
	hws[WOKOO_CLK_DDR_AXI]      = wokoo_hw_register_divider2(NULL, "ddr_axi_clk", "pll_out", 0, base + 0x80, 0, 0xf, 16, 8, 0x3, 18, 0, 0x03);
	hws[WOKOO_CLK_DDR_AXI_IDLE] = wokoo_hw_register_divider2(NULL, "ddr_axi_clk_idle", "pll_out", 0, base + 0x80, 4, 0xf, 17, 12, 0x3, 18, 0, 0x03);

	/* USB_12M */
	hws[WOKOO_MCLK_PLL_DIV]    = wokoo_hw_register_divider(NULL, "pll_mclk_div", "pll_out", CLK_SET_RATE_ADD_1_DIV | CLK_SET_RATE_NO_WRITEABLE, base + 0x90, 3, 0xf, 0, 8);
	hws[WOKOO_SCLK_USBOTG_12M] = wokoo_hw_register_divider(NULL, "usb_12m", "pll_mclk_div", CLK_SET_RATE_ADD_1_DIV | CLK_SET_RATE_NO_WRITEABLE, base + 0x90, 0, 0x7, 0, 2);

	/* LCDC_AXI_CLK */
	hws[WOKOO_SCLK_LCDC_AXI] = wokoo_hw_register_divider2(NULL, "lcdc_axi_clk", "pll_out", 0, base + 0x9c, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);
	hws[WOKOO_SCLK_LCDC0]    = wokoo_hw_register_divider2(NULL, "lcdc0_mclk",   "pll_mclk", CLK_SET_RATE_ADD_1_DIV, base + 0xa0, 0, 0xf, 16, 4, 0x1f, 17, 0x00, 0x02);
	//hws[WOKOO_CLK_LCDC_AXI]  = wokoo_clk_hw_gate("lcdc_axi_clk_en", "lcdc_axi_clk", base + 0x9c, 8, 18);
	//hws[WOKOO_MCLK_LCDC0]    = wokoo_clk_hw_gate("lcdc0_mclk_en", "lcdc0_mclk", base + 0xa0, 12, 18);

	/* ISP_AXI_CLK */
	hws[WOKOO_CLK_ISP_AXI]    = wokoo_hw_register_divider2(NULL, "isp_axi_clk",     "pll_out", 0, base + 0xac, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);
	hws[WOKOO_SCLK_ISP_2]     = wokoo_hw_register_divider2(NULL, "isp_psclk_sclk2", "pll_out", 0, base + 0xb0, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);
	hws[WOKOO_CLK_CPHY_CFG]   = wokoo_clk_hw_gate("cphy_cfg_clk", "isp_axi_clk", base + 0xac, 11, 21);
	hws[WOKOO_SCLK_LVDS_IDSP] = wokoo_clk_hw_gate("lvds_idsp_sclk", "isp_axi_clk", base + 0xac, 10, 20);
	hws[WOKOO_HCLK_ISP_EN]    = wokoo_clk_hw_gate("isp_hclk", "isp_axi_clk", base + 0xac, 9, 19);
	hws[WOKOO_SCLK_ISP_P_EN]  = wokoo_clk_hw_gate("isp_p_sclk", "isp_axi_clk", base + 0xac, 8, 18);
	// printk("wokoo ISPPWR reg = 0x%x\n", (base + 0xa0));
	
	/* TIMER_CLK */
	hws[WOKOO_CLK_TIMER0_SEL]   = wokoo_clk_hw_mux("timer0_sel", base + 0xc8, 28, 3, timer_pre_sels, ARRAY_SIZE(timer_pre_sels));
	hws[WOKOO_SCLK_TIMER0_FUNC] = wokoo_hw_register_divider(NULL, "timer0_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xc8, 24, 0xf,  0, 20);
	hws[WOKOO_SCLK_TIMER0_PBUS] = wokoo_hw_register_divider(NULL, "timer0_pbus_clk", "ctl_pclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xc8, 16, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER0_PLL]  = wokoo_hw_register_divider(NULL, "timer0_pll_clk",  "pll_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xc8,  8, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER0_BUS]  = wokoo_hw_register_divider(NULL, "timer0_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xc8,  0, 0xff, 0, 20);
	hws[WOKOO_CLK_TIMER1_SEL]   = wokoo_clk_hw_mux("timer1_sel", base + 0xcc, 28, 3, timer_pre_sels, ARRAY_SIZE(timer_pre_sels));
	hws[WOKOO_SCLK_TIMER1_FUNC] = wokoo_hw_register_divider(NULL, "timer1_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xcc, 24, 0xf,  0, 20);
	hws[WOKOO_SCLK_TIMER1_PBUS] = wokoo_hw_register_divider(NULL, "timer1_pbus_clk", "ctl_pclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xcc, 16, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER1_PLL]  = wokoo_hw_register_divider(NULL, "timer1_pll_clk",  "pll_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xcc,  8, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER1_BUS]  = wokoo_hw_register_divider(NULL, "timer1_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xcc,  0, 0xff, 0, 20);
	hws[WOKOO_CLK_TIMER2_SEL]   = wokoo_clk_hw_mux("timer2_sel", base + 0xd0, 28, 3, timer_pre_sels, ARRAY_SIZE(timer_pre_sels));
	hws[WOKOO_SCLK_TIMER2_FUNC] = wokoo_hw_register_divider(NULL, "timer2_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xd0, 24, 0xf,  0, 20);
	hws[WOKOO_SCLK_TIMER2_PBUS] = wokoo_hw_register_divider(NULL, "timer2_pbus_clk", "ctl_pclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xd0, 16, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER2_PLL]  = wokoo_hw_register_divider(NULL, "timer2_pll_clk",  "pll_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xd0,  8, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER2_BUS]  = wokoo_hw_register_divider(NULL, "timer2_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xd0,  0, 0xff, 0, 20);
	hws[WOKOO_CLK_TIMER3_SEL]   = wokoo_clk_hw_mux("timer3_sel", base + 0xd4, 28, 3, timer_pre_sels, ARRAY_SIZE(timer_pre_sels));
	hws[WOKOO_SCLK_TIMER3_FUNC] = wokoo_hw_register_divider(NULL, "timer3_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xd4, 24, 0xf,  0, 20);
	hws[WOKOO_SCLK_TIMER3_PBUS] = wokoo_hw_register_divider(NULL, "timer3_pbus_clk", "ctl_pclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xd4, 16, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER3_PLL]  = wokoo_hw_register_divider(NULL, "timer3_pll_clk",  "pll_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xd4,  8, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER3_BUS]  = wokoo_hw_register_divider(NULL, "timer3_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0xd4,  0, 0xff, 0, 20);
	hws[WOKOO_CLK_TIMER4_SEL]   = wokoo_clk_hw_mux("timer4_sel", base + 0x158, 28, 3, timer_pre_sels, ARRAY_SIZE(timer_pre_sels));
	hws[WOKOO_SCLK_TIMER4_FUNC] = wokoo_hw_register_divider(NULL, "timer4_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x158, 24, 0xf,  0, 20);
	hws[WOKOO_SCLK_TIMER4_PBUS] = wokoo_hw_register_divider(NULL, "timer4_pbus_clk", "ctl_pclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x158, 16, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER4_PLL]  = wokoo_hw_register_divider(NULL, "timer4_pll_clk",  "pll_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x158,  8, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER4_BUS]  = wokoo_hw_register_divider(NULL, "timer4_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x158,  0, 0xff, 0, 20);
	hws[WOKOO_CLK_TIMER5_SEL]   = wokoo_clk_hw_mux("timer5_sel", base + 0x15c, 28, 3, timer_pre_sels, ARRAY_SIZE(timer_pre_sels));
	hws[WOKOO_SCLK_TIMER5_FUNC] = wokoo_hw_register_divider(NULL, "timer5_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x15c, 24, 0xf,  0, 20);
	hws[WOKOO_SCLK_TIMER5_PBUS] = wokoo_hw_register_divider(NULL, "timer5_pbus_clk", "ctl_pclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x15c, 16, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER5_PLL]  = wokoo_hw_register_divider(NULL, "timer5_pll_clk",  "pll_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x15c,  8, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER5_BUS]  = wokoo_hw_register_divider(NULL, "timer5_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x15c,  0, 0xff, 0, 20);
	hws[WOKOO_CLK_TIMER6_SEL]   = wokoo_clk_hw_mux("timer6_sel", base + 0x160, 28, 3, timer_pre_sels, ARRAY_SIZE(timer_pre_sels));
	hws[WOKOO_SCLK_TIMER6_FUNC] = wokoo_hw_register_divider(NULL, "timer6_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x160, 24, 0xf,  0, 20);
	hws[WOKOO_SCLK_TIMER6_PBUS] = wokoo_hw_register_divider(NULL, "timer6_pbus_clk", "ctl_pclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x160, 16, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER6_PLL]  = wokoo_hw_register_divider(NULL, "timer6_pll_clk",  "pll_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x160,  8, 0xff, 0, 20);
	hws[WOKOO_SCLK_TIMER6_BUS]  = wokoo_hw_register_divider(NULL, "timer6_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x160,  0, 0xff, 0, 20);

	/* PWM_CLK */
	hws[WOKOO_SCLK_PWM]     = wokoo_hw_register_divider(NULL, "pwm_clk", "osc16m", CLK_SET_RATE_ADD_1_DIV | CLK_SET_RATE_NO_WRITEABLE, base + 0xd8, 0, 0xf, 0, 8);

	/* I2S_CLK */
	hws[WOKOO_SCLK_I2S0_GR] = wokoo_hw_register_divider(NULL, "i2s0_gr_clk", "pll_mclk", 0, base + 0xdc, 0, 0xf, 16, 0);
	hws[WOKOO_SCLK_I2S1_GR] = wokoo_hw_register_divider(NULL, "i2s1_gr_clk", "pll_mclk", 0, base + 0xdc, 4, 0xf, 17, 0);
	/* I2Sx_DIV[0~15] = 1024, I2Sx_MUL[16~31] = 6240 */
	hws[WOKOO_SCLK_I2S0]    = wokoo_hw_register_mult_divider(NULL, "i2s0_clk", "i2s0_gr_clk", CLK_SET_RATE_NO_WRITEABLE, base + 0xe0, 16, 0xffff, 0, 0xffff, 1024, 6240);
	hws[WOKOO_SCLK_I2S1]    = wokoo_hw_register_mult_divider(NULL, "i2s1_clk", "i2s1_gr_clk", CLK_SET_RATE_NO_WRITEABLE, base + 0xe4, 16, 0xffff, 0, 0xffff, 1024, 6240);

	/* SSIx_CLK div1 ssi_gr, div2 ssix_clk */
	hws[WOKOO_SCLK_SSI0_GR] = wokoo_hw_register_divider(NULL, "ssi0_gr_clk", "pll_mclk", 0, base + 0xe8, 0, 0xf, 16, 1);
	hws[WOKOO_SCLK_SSI1_GR] = wokoo_hw_register_divider(NULL, "ssi1_gr_clk", "pll_mclk", 0, base + 0xe8, 4, 0xf, 17, 1);
	hws[WOKOO_SCLK_SSI0]    = wokoo_hw_register_divider(NULL, "ssi0_clk", "ssi0_gr_clk", CLK_SET_RATE_ADD_1_DIV, base + 0xec, 0, 0xf, 16, 16);
	hws[WOKOO_SCLK_SSI1]    = wokoo_hw_register_divider(NULL, "ssi1_clk", "ssi1_gr_clk", CLK_SET_RATE_ADD_1_DIV, base + 0xec, 4, 0xf, 17, 16);

	/* UART_CLK div1 uart_gr, div2 mul -> uart_clk */
	hws[WOKOO_SCLK_UART0_GR] = wokoo_hw_register_divider(NULL, "uart0_gr_clk", "pll_mclk", 0, base + 0xf0, 0, 0xf, 16, 2);
	hws[WOKOO_SCLK_UART1_GR] = wokoo_hw_register_divider(NULL, "uart1_gr_clk", "pll_mclk", 0, base + 0xf0, 0, 0xf, 17, 2);
	hws[WOKOO_SCLK_UART2_GR] = wokoo_hw_register_divider(NULL, "uart2_gr_clk", "pll_mclk", 0, base + 0xf0, 0, 0xf, 18, 2);
	hws[WOKOO_SCLK_UART0]    = wokoo_hw_register_mult_divider(NULL, "uart0_mclk", "uart0_gr_clk", CLK_SET_RATE_NO_WRITEABLE | CLK_SET_RATE_MUL_2, base + 0xf4, 16, 0xffff, 0, 0xffff, 72, 3750);
	hws[WOKOO_SCLK_UART1]    = wokoo_hw_register_mult_divider(NULL, "uart1_mclk", "uart1_gr_clk", CLK_SET_RATE_NO_WRITEABLE | CLK_SET_RATE_MUL_2, base + 0xf8, 16, 0xffff, 0, 0xffff, 72, 3750);
	hws[WOKOO_SCLK_UART2]    = wokoo_hw_register_mult_divider(NULL, "uart2_mclk", "uart2_gr_clk", CLK_SET_RATE_NO_WRITEABLE | CLK_SET_RATE_MUL_2, base + 0xfc, 16, 0xffff, 0, 0xffff, 72, 3750);

	/* I2S_CLK */
	hws[WOKOO_SCLK_I2C0] = wokoo_hw_register_divider(NULL, "i2c0_clk", "pll_mclk", CLK_SET_RATE_ADD_1_DIV | CLK_SET_RATE_NO_WRITEABLE, base + 0x100, 0, 0x1f, 0, 12);
	hws[WOKOO_SCLK_I2C1] = wokoo_hw_register_divider(NULL, "i2c1_clk", "pll_mclk", CLK_SET_RATE_ADD_1_DIV | CLK_SET_RATE_NO_WRITEABLE, base + 0x100, 8, 0x1f, 0, 12);

	/* ATB_CLK */
	hws[WOKOO_SCLK_ATB] = wokoo_hw_register_divider2(NULL, "atb_clk", "pll_out", 0, base + 0x128, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);
	/* A7 DBG_PCLK */
	hws[WOKOO_PCLK_DBG] = wokoo_hw_register_divider(NULL, "dbg_pclk", "atb_clk", CLK_SET_RATE_ADD_1_DIV, base + 0x12c, 4, 0x7, 17, 2);
	/* Do not register for debug */
	//hws[WOKOO_CLK_ATB]  = wokoo_clk_hw_gate("atb_clk_en", "atb_clk", base + 0x128, 8, 18);
	//hws[WOKOO_CLK_DBG]  = wokoo_clk_hw_gate("dbg_pclk_en", "dbg_pclk", base + 0x12c, 0, 16);

	/* CS_CLK */
	hws[WOKOO_SCLK_CS_CFG] = wokoo_hw_register_divider2(NULL, "cs_cfg_clk", "pll_out", 0, base + 0x130, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);
	hws[WOKOO_SCLK_CS_CTM] = wokoo_hw_register_divider2(NULL, "cs_ctm_clk", "pll_out", 0, base + 0x134, 0, 0xf, 16, 4, 0x3, 17, 0, 0x03);

	/* CLK_OUT */
	hws[WOKOO_CLK_OUT1_SEL]  = wokoo_clk_hw_mux("clkout1_sel", base + 0x148, 0, 3, clkout_pre_sels, ARRAY_SIZE(clkout_pre_sels));
	hws[WOKOO_CLK_OUT1_32K]  = wokoo_hw_register_divider(NULL, "clkout1_32K_clk",  "osc32k",    CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x14c, 28, 0xf,  0, 20);
	hws[WOKOO_CLK_OUT1_FUNC] = wokoo_hw_register_divider(NULL, "clkout1_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x14c, 24, 0xf,  0, 20);
	/* clkout_pll = clkout_pll_div0 / div1 */
	hws[WOKOO_CLK_OUT1_PLL_DIV0]  = wokoo_hw_register_divider(NULL, "clkout1_pll_div0_clk",  "pll_out",  CLK_SET_RATE_ADD_1_DIV | CLK_SET_RATE_NO_WRITEABLE, base + 0x14c,  8, 0xff, 0, 10);
	hws[WOKOO_CLK_OUT1_PLL]  = wokoo_hw_register_divider(NULL, "clkout1_pll_clk",  "clkout1_pll_div0_clk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x14c,  16, 0xf, 0, 20);
	hws[WOKOO_CLK_OUT1_BUS]  = wokoo_hw_register_divider(NULL, "clkout1_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x14c,  0, 0xff, 0, 20);
	hws[WOKOO_CLK_OUT2_SEL]  = wokoo_clk_hw_mux("clkout2_sel", base + 0x148, 4, 3, clkout_pre_sels, ARRAY_SIZE(clkout_pre_sels));
	hws[WOKOO_CLK_OUT2_32K]  = wokoo_hw_register_divider(NULL, "clkout2_32K_clk",  "osc32k",    CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x150, 28, 0xf,  0, 20);
	hws[WOKOO_CLK_OUT2_FUNC] = wokoo_hw_register_divider(NULL, "clkout2_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x150, 24, 0xf,  0, 20);
	/* clkout_pll = clkout_pll_div0 / div1 */
	hws[WOKOO_CLK_OUT2_PLL_DIV0]  = wokoo_hw_register_divider(NULL, "clkout2_pll_div0_clk",  "pll_out",  CLK_SET_RATE_ADD_1_DIV | CLK_SET_RATE_NO_WRITEABLE, base + 0x150,  8, 0xff, 0, 10);
	hws[WOKOO_CLK_OUT2_PLL]  = wokoo_hw_register_divider(NULL, "clkout2_pll_clk",  "clkout1_pll_div0_clk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x150,  16, 0xf, 0, 20);
	hws[WOKOO_CLK_OUT2_BUS]  = wokoo_hw_register_divider(NULL, "clkout2_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x150,  0, 0xff, 0, 20);
	hws[WOKOO_CLK_OUT3_SEL]  = wokoo_clk_hw_mux("clkout3_sel", base + 0x148, 8, 3, clkout_pre_sels, ARRAY_SIZE(clkout_pre_sels));
	hws[WOKOO_CLK_OUT3_32K]  = wokoo_hw_register_divider(NULL, "clkout3_32K_clk",  "osc32k",    CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x154, 28, 0xf,  0, 20);
	hws[WOKOO_CLK_OUT3_FUNC] = wokoo_hw_register_divider(NULL, "clkout3_func_clk", "func_mclk", CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x154, 24, 0xf,  0, 20);
	/* clkout_pll = clkout_pll_div0 / div1 */
	hws[WOKOO_CLK_OUT3_PLL_DIV0]  = wokoo_hw_register_divider(NULL, "clkout3_pll_div0_clk",  "pll_out",  CLK_SET_RATE_ADD_1_DIV | CLK_SET_RATE_NO_WRITEABLE, base + 0x154,  8, 0xff, 0, 10);
	hws[WOKOO_CLK_OUT3_PLL]  = wokoo_hw_register_divider(NULL, "clkout3_pll_clk",  "clkout1_pll_div0_clk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x154,  16, 0xf, 0, 20);
	hws[WOKOO_CLK_OUT3_BUS]  = wokoo_hw_register_divider(NULL, "clkout3_bus_clk",  "bus_mclk",  CLK_SET_RATE_DIV_MUL_2 | CLK_SET_RATE_NO_WRITEABLE, base + 0x154,  0, 0xff, 0, 20);

	wokoo_check_clk_hws(hws, WOKOO_CLK_END);
	of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_hw_data);
	/* Register restart for system */
	wokoo_register_restart_notifier(base, WOKOO_GLB_SFRST_CTL, NULL);
	/* Register reset for devices */
	wokoo_register_softrst(np, WOKOO_SOFTRST_NUM, base + WOKOO_SOFTRST_CON, 0);
}

CLK_OF_DECLARE(wokoo, "wokoo,wokoo-clks", wokoo_clocks_init);
