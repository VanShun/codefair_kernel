/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 */
#ifndef _LINUX_WOKOO_FB_H_
#define _LINUX_WOKOO_FB_H_

/*contrll register shift address*/
#define LCDC_SWITCH				0x00
#define LCDC_GCTRL				0x04
#define LCDC_INT_STATUS			0x08
#define LCDC_INT_MSK			0x0c
#define LCDC_INT_CLR			0x010
#define RGB_H_TMG				0x014
#define RGB_V_TMG				0x018
#define RGB_W_TMG				0x01c
#define RGB_DCLK				0x020
#define M_CS_CTRL				0x024
#define DeltaRGB_CFG			0x28
#define BACKGROUD				0x2c
#define WIN0_CFG_A				0x30
#define WIN0_CFG_B				0x34
#define WIN0_CFG_C				0x38
#define WIN01_HSIZE				0x90
#define ALPHA_VALUE				0xa0
#define PanelDataFormat			0xa4
#define Win0StartAddr0			0xb8


/*LCDC  para*/
#define LCDC_DATA_TRAN			(1<<0)
#define LCDC_EN_WOKOO			(1<<0)
#define LCDC_UNEN_WOKOO			0xFFFE
#define LCDC_ENCOF_WIN0			(1<<2)
#define LCDC_UNCOF_WIN0			~(1<<2)
#define LCDC_BACKLIGHT_EN		0x01
#define LCDC_DCLK_INPUT_CLK		(1<<16)

#define LCDC_SET_POLOAR			(~(0x3<<7))
#define LCDC_SET_POLOAR_SM		(~(0x1<<7))

#define LCDC_RGB_MODE			(1<<1)
#define LCDC_INT_SEL_FRAME		(1<<15)
#define LCDC_EN_INT				0x0


#define write_reg(val, reg) do { writel((val), (reg)); } while(0)

/* Without this delay, the graphics appears somehow scaled and
 * there is a lot of jitter in scanlines. This delay is probably
 * needed only after setting some specific register(s) somewhere,
 * not all over the place... */
#define write_reg_dly(val, reg) do { writel((val), reg); udelay(1000); } while(0)


struct wokoo_fb_videomode {
	struct fb_videomode mode;
	u32 pcr;
	bool aus_mode;
	unsigned char	bpp;
};

#endif
