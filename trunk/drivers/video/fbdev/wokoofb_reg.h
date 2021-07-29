/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 */
#ifndef _LINUX_WOKOO_FB_H_
#define _LINUX_WOKOO_FB_H_

/*contrll register shift address*/
#define LCDC_SWITCH_ADDR		 0x00
#define LCDC_GCTRL_ADDR			 0x04
#define LCDC_INT_STATUS_ADDR	 0x08
#define LCDC_INT_MSK_ADDR		 0x0c
#define LCDC_INT_CLR_ADDR		 0x10
#define RGB_H_TMG_ADDR			 0x14
#define RGB_V_TMG_ADDR			 0x18
#define RGB_W_TMG_ADDR			 0x1c
#define RGB_DCLK_ADDR			 0x20
#define M_CS_CTRL_ADDR			 0x24
#define DeltaRGB_CFG			 0x28
#define BACKGROUD_ADDR			 0x2c
#define WIN0_CFG_A_ADDR			 0x30
#define WIN0_CFG_B_ADDR		     0x34
#define WIN0_CFG_C				 0x38
#define WIN01_HSIZE_ADDR	     0x90
#define ALPHA_VALUE				 0xa0
#define PAL_DAT_FORMAT_ADDR      0XA4
#define WIN0_START_ADDR0         0xb8


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

#define  CFG_WIN0_EN            BIT(24)
#define  CFG_WIN0_DIS          ~BIT(24)
#define   CFG_CC0_EN             BIT(25)
#define   CFG_CC0_DIS            ~BIT(25)
#define  CFG_RGB_HBK_SFET     (0)     /*Horizontal blank for HSYNC shift*/
#define  CFG_RGB_HFP_SFET     (16)    /*Front porch for HSYNC shift*/
#define  CFG_RGB_VBK_SFET     (0)     /*Vertical blank for VSYNC shift*/
#define  CFG_RGB_VFP_SFET     (16)    /*Front porch for VSYNC shift*/
#define  CFG_RGB_HPW_SFET     (0)     /*Pulse width of HSYNC shift*/
#define  CFG_RGB_VPW_SFET     (8)     /*Pulse width unit of VSYNC shift*/


#define WOKOOFB_INTR_UFLWI   BIT(7)

#define write_reg(val, reg) do { writel((val), (reg)); } while(0)

/* Without this delay, the graphics appears somehow scaled and
 * there is a lot of jitter in scanlines. This delay is probably
 * needed only after setting some specific register(s) somewhere,
 * not all over the place... */
#define write_reg_dly(val, reg) do { writel((val), reg); udelay(1000); } while(0)
typedef enum {
   DPI_WOKOO_RGB565 = 0X4,
   DPI_WOKOO_RGB1555,
   DPI_WOKOO_RGB4444,
   DPI_WOKOO_RGB8888
}wokoo_fb_rgb_e;

typedef enum {
   DPI_WOKOO_DEP_RGB332 = 0X0,
   DPI_WOKOO_DEP_RGB444,
   DPI_WOKOO_DEP_RGB565,
   DPI_WOKOO_DEP_RGB666,
   DPI_WOKOO_DEP_RGB888
}wokoo_cfg_rgb_dep_e;
struct wokoo_fb_videomode {
	struct fb_videomode mode;
	u32 pcr;
	bool aus_mode;
	unsigned char	bpp;
};

#endif /*_LINUX_WOKOO_FB_H_*/
