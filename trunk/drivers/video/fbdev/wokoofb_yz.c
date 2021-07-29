/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <dt-bindings/gpio/gpio.h>
#include "wokoofb_reg.h"
#include <linux/regulator/consumer.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/platform_data/video-imxfb.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif



//#define WOKOO_DEBUG    1
int gpio_reset,gpio_bl,gpio_sda,gpio_dclk,gpio_cs;




static const struct fb_var_screeninfo wokoofb_default = {
	.xres = 480,
	.yres = 480,
	.xres_virtual = 480,
	.yres_virtual = 800,
	.xoffset = 160 ,
	.yoffset = 0,
	.bits_per_pixel = 24,
	.red = {11, 5, 0},
	.green = {5, 6, 0},
	.blue = {0, 5, 0},
	.activate = FB_ACTIVATE_TEST,
	.height = 800,
	.width = 480,
	.pixclock = 31250,
	.left_margin = 2,			//back   (thb)
	.right_margin = 2,		//fron   porch   (thfb)
	.upper_margin = 10,			//back   (tvb)
	.lower_margin = 10,			//fron   (tvfb)
	.hsync_len = 12,
	.vsync_len = 12,
	.vmode = FB_VMODE_NONINTERLACED,
	.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
};

static const struct fb_fix_screeninfo wokoofb_fix = {
	.id = "wokoo",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE,
};

#define LCD_SPI_CS(a)  \
						if (a)	\
						gpio_set_value(gpio_cs, 1); 	\
						else		\
						gpio_set_value(gpio_cs, 0);
						
#define SPI_DCLK(a)	\
						if (a)	\
						gpio_set_value(gpio_dclk, 1); 	\
						else		\
						gpio_set_value(gpio_dclk, 0);
						
#define SPI_SDA(a)	\
						if (a)	\
						gpio_set_value(gpio_sda, 1);	\
						else		\
						gpio_set_value(gpio_sda, 0);







struct wokoofb_info {
	struct platform_device	*pdev;
	void __iomem			*regs;
	struct clk				*clk_ipg;
	struct clk				*clk_ahb;
	struct clk				*clk_per;
	bool					enabled;
	dma_addr_t				map_dma;
	u_int					map_size;
	u_int					palette_size;
	bool					cmap_inverse;
	bool					cmap_static;

	struct wokoo_fb_videomode	*mode;

};


static const struct of_device_id wokoofb_of_dev_id[] = {
	{
		.compatible = "wokoo,wokoo-lcd",

	},  {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, wokoofb_of_dev_id);


#define DRIVER_NAME	"WOKOO"



volatile void LCD_delay(volatile int time)
{
	volatile unsigned int i;	
	while(time--)
	for(i=500;i>0;i--);
}



void lcd_reset(void)
{
	

	gpio_set_value(gpio_reset, 0);   //pull down

	LCD_delay(300);

	gpio_set_value(gpio_reset, 1);  //pull up


	LCD_delay(300);

}

void LCD_WriteByteSPI(unsigned char byte)
{
    unsigned char n;

    for(n=0; n<8; n++)
    {
        if(byte&0x80) SPI_SDA(1)
        else SPI_SDA(0)
        byte<<= 1;
        SPI_DCLK(0);
        SPI_DCLK(1);
    }
}
void SPI_WriteComm(unsigned     int CMD)  //3线9bit 串行接口
{
	LCD_SPI_CS(0);
	SPI_SDA(0);
	SPI_DCLK(0);
	SPI_DCLK(1);
	LCD_WriteByteSPI(CMD);
	LCD_SPI_CS(1);
}
void SPI_WriteData(unsigned     int tem_data)
{
	LCD_SPI_CS(0);
	SPI_SDA(1);
	SPI_DCLK(0);
	SPI_DCLK(1);
	LCD_WriteByteSPI(tem_data);
	LCD_SPI_CS(1);
}



void Lcd_Initialize_l(void){

    printk("liuyz init lcdc=========\n");

	  LCD_SPI_CS(1);
	    LCD_delay(20);
	    LCD_SPI_CS(0);
	lcd_reset();


	SPI_WriteComm(0x11);
	LCD_delay(40);
	SPI_WriteComm(0xB0);
	SPI_WriteData(0x04);

	SPI_WriteComm(0xB3);
	SPI_WriteData(0x10);//RGB=0x10;MCU=0x00;  0x02
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xB6);
	SPI_WriteData(0x52);
	SPI_WriteData(0x83);

	SPI_WriteComm(0xB7);
	SPI_WriteData(0x80);
	SPI_WriteData(0x72);
	SPI_WriteData(0x11);
	SPI_WriteData(0x25);

	SPI_WriteComm(0xB8);
	SPI_WriteData(0x00);
	SPI_WriteData(0x0F);
	SPI_WriteData(0x0F);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xC8);
	SPI_WriteData(0xC8);
	SPI_WriteData(0x02);
	SPI_WriteData(0x18);
	SPI_WriteData(0x10);
	SPI_WriteData(0x10);
	SPI_WriteData(0x37);
	SPI_WriteData(0x5A);
	SPI_WriteData(0x87);
	SPI_WriteData(0xBE);
	SPI_WriteData(0xFF);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xB9);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xBD);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xC0);
	SPI_WriteData(0x02);
	SPI_WriteData(0x76);

	SPI_WriteComm(0xC1);
	SPI_WriteData(0x63);
	SPI_WriteData(0x31);
	SPI_WriteData(0x00);
	SPI_WriteData(0x27);
	SPI_WriteData(0x27);
	SPI_WriteData(0x32);
	SPI_WriteData(0x12);
	SPI_WriteData(0x28);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x10);
	SPI_WriteData(0xA5);
	SPI_WriteData(0x0F);
	SPI_WriteData(0x58);
	SPI_WriteData(0x21);
	SPI_WriteData(0x01);

	SPI_WriteComm(0xC2);
	SPI_WriteData(0x28);
	SPI_WriteData(0x06);
	SPI_WriteData(0x06);
	SPI_WriteData(0x01);
	SPI_WriteData(0x03);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xC3);
	SPI_WriteData(0x40);
	SPI_WriteData(0x00);
	SPI_WriteData(0x03);
	SPI_WriteComm(0xC4);
	SPI_WriteData(0x00);
	SPI_WriteData(0x01);
	SPI_WriteComm(0xC6);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xC7);
	SPI_WriteData(0x11);
	SPI_WriteData(0x8D);
	SPI_WriteData(0xA0);
	SPI_WriteData(0xF5);
	SPI_WriteData(0x27);
	SPI_WriteComm(0xC8);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteComm(0xC9);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteComm(0xCA);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteComm(0xD0);
	SPI_WriteData(0xA9);
	SPI_WriteData(0x03);
	SPI_WriteData(0xCC);
	SPI_WriteData(0xA5);
	SPI_WriteData(0x00);
	SPI_WriteData(0x53);
	SPI_WriteData(0x20);
	SPI_WriteData(0x10);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x01);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x03);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xD1);
	SPI_WriteData(0x18);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x23);
	SPI_WriteData(0x03);
	SPI_WriteData(0x75);
	SPI_WriteData(0x02);
	SPI_WriteData(0x50);
	SPI_WriteComm(0xD3);
	SPI_WriteData(0x33);
	SPI_WriteComm(0xD5);
	SPI_WriteData(0x2a);
	SPI_WriteData(0x2a);
	SPI_WriteComm(0xD6);
	SPI_WriteData(0x28);//a8
	SPI_WriteComm(0xD7);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0xAA);
	SPI_WriteData(0xC0);
	SPI_WriteData(0x2A);
	SPI_WriteData(0x2C);
	SPI_WriteData(0x22);
	SPI_WriteData(0x12);
	SPI_WriteData(0x71);
	SPI_WriteData(0x0A);
	SPI_WriteData(0x12);
	SPI_WriteData(0x00);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x03);
	SPI_WriteComm(0xD8);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);
	SPI_WriteData(0x22);
	SPI_WriteData(0x44);
	SPI_WriteData(0x21);
	SPI_WriteData(0x46);
	SPI_WriteData(0x42);
	SPI_WriteData(0x40);
	SPI_WriteComm(0xD9);
	SPI_WriteData(0xCF);
	SPI_WriteData(0x2D);
	SPI_WriteData(0x51);
	SPI_WriteComm(0xDA);
	SPI_WriteData(0x01);
	SPI_WriteComm(0xDE);
	SPI_WriteData(0x01);
	SPI_WriteData(0x51);//58
	SPI_WriteComm(0xE1);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xE6);
	SPI_WriteData(0x55);//58
	SPI_WriteComm(0xF3);
	SPI_WriteData(0x06);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x24);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xF8);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xFA);
	SPI_WriteData(0x01);
	SPI_WriteComm(0xFB);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xFC);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xFD);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x70);
	SPI_WriteData(0x00);
	SPI_WriteData(0x72);
	SPI_WriteData(0x31);
	SPI_WriteData(0x37);
	SPI_WriteData(0x70);
	SPI_WriteData(0x32);
	SPI_WriteData(0x31);
	SPI_WriteData(0x07);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xFE);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x20);
	SPI_WriteComm(0xB0);
	SPI_WriteData(0x04); //04
	LCD_delay(40);
	SPI_WriteComm(0x35);
	SPI_WriteData(0x00);
	SPI_WriteComm(0x44);
	SPI_WriteData(0x00);
	SPI_WriteComm(0x36);
	SPI_WriteData(0x00);
	SPI_WriteComm(0x3A);
	SPI_WriteData(0x77);
	SPI_WriteComm(0x2A);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x01);
	SPI_WriteData(0xDF);
	SPI_WriteComm(0x2B);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x03);
	SPI_WriteData(0x1F);
	SPI_WriteComm(0x29);
	LCD_delay(10);
	SPI_WriteComm(0x2C);
	LCD_delay(10);

	SPI_WriteComm(0x36);
	SPI_WriteData(0x68);
	SPI_WriteComm(0x2C);



}




static int wokoofb_activate_var(struct fb_var_screeninfo *var,
		struct fb_info *info);

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int wokoofb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	struct wokoofb_info *fbi = info->par;
	unsigned int val;
	int ret = 1;

	/*
	 * If inverse mode was selected, invert all the colours
	 * rather than the register number.  The register number
	 * is what you poke into the framebuffer to produce the
	 * colour you requested.
	 */
	if (fbi->cmap_inverse) {
		red   = 0xffff - red;
		green = 0xffff - green;
		blue  = 0xffff - blue;
	}

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no mater what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 12 or 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red, &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue, &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	}

	return ret;
}


/*
 * wokoofb_set_par():
 *	Set the user defined part of the display for the specified console
 */
static int wokoofb_set_par(struct fb_info *info)
{
	struct wokoofb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;

	if (var->bits_per_pixel == 16 || var->bits_per_pixel == 32)
		info->fix.visual = FB_VISUAL_TRUECOLOR;
	else if (!fbi->cmap_static)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		info->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;
	fbi->palette_size = var->bits_per_pixel == 8 ? 256 : 16;

	wokoofb_activate_var(var, info);
	return 0;
}

/*
*  wokoofb_enable_controller()
*	Enable the contrller
*/
static int wokoofb_enable_controller(struct wokoofb_info *fbi)
{
	int ret;

	if (fbi->enabled)
		return 0;

	pr_debug("Enabling LCD controller\n");

	/*enable controller*/
	write_reg(readl(fbi->regs + LCDC_GCTRL)|LCDC_EN_WOKOO, fbi->regs + LCDC_GCTRL);

	ret = clk_prepare_enable(fbi->clk_ahb);
	if (ret)
		goto err_enable_ahb;
	clk_set_rate(fbi->clk_ahb, 128000000);

	ret = clk_prepare_enable(fbi->clk_per);
	if (ret)
		goto err_enable_per;
	clk_set_rate(fbi->clk_per, 16000000);

#ifdef WOKOO_DEBUG
	printk("ahb clk set value = %ld", clk_get_rate(fbi->clk_ahb));
	printk("per clk set value= %ld", clk_get_rate(fbi->clk_per));
#endif

	fbi->enabled = true;
	return 0;

err_enable_per:
	clk_disable_unprepare(fbi->clk_ahb);
err_enable_ahb:
	clk_disable_unprepare(fbi->clk_ipg);


	return ret;
}


/*
* 	wokoofb_disable_controller()
*	Disable the contrller
*/
static void wokoofb_disable_controller(struct wokoofb_info *fbi)
{
	if (!fbi->enabled)
		return;

	pr_debug("Disabling LCD controller\n");

	clk_disable_unprepare(fbi->clk_per);
	clk_disable_unprepare(fbi->clk_ahb);
	clk_disable_unprepare(fbi->clk_ipg);
	fbi->enabled = false;

	/*disable  controller*/
	write_reg(readl(fbi->regs + LCDC_GCTRL)&LCDC_UNEN_WOKOO, fbi->regs + LCDC_GCTRL);
}

static int wokoofb_blank(int blank, struct fb_info *info)
{
	struct wokoofb_info *fbi = info->par;

#ifdef	WOKOO_DEBUG
	printk("wokoofb_blank: blank=%d\n", blank);
#endif

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		wokoofb_disable_controller(fbi);
		break;

	case FB_BLANK_UNBLANK:
		return wokoofb_enable_controller(fbi);
	}
	return 0;
}

static struct fb_ops wokoofb_ops = {
	.owner			= THIS_MODULE,
	.fb_set_par		= wokoofb_set_par,				//set para
	.fb_setcolreg	= wokoofb_setcolreg,		//rgb color
	.fb_blank		= wokoofb_blank,				//enable lcdc
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,

};

/*
 * wokoofb_activate_var():
 *	Configures LCD Controller based on entries in var parameter.  Settings are
 *	only written to the controller if changes were made.
 */
static int wokoofb_activate_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct wokoofb_info *fbi = info->par;

	pr_debug("var: xres=%d hslen=%d lm=%d rm=%d\n",
		var->xres, var->hsync_len,
		var->left_margin, var->right_margin);
	pr_debug("var: yres=%d vslen=%d um=%d bm=%d\n",
		var->yres, var->vsync_len,
		var->upper_margin, var->lower_margin);


#ifdef	WOKOO_DEBUG
	printk(KERN_ERR "%s: invalid xres %d\n",
			info->fix.id, var->xres);
	printk(KERN_ERR "%s: invalid hsync_len %d\n",
			info->fix.id, var->hsync_len);
	printk(KERN_ERR "%s: invalid left_margin %d\n",
			info->fix.id, var->left_margin);
	printk(KERN_ERR "%s: invalid right_margin %d\n",
			info->fix.id, var->right_margin);
	printk(KERN_ERR "%s: invalid yres %d\n",
			info->fix.id, var->yres);
	printk(KERN_ERR "%s: invalid vsync_len %d\n",
			info->fix.id, var->vsync_len);
	printk(KERN_ERR "%s: invalid upper_margin %d\n",
			info->fix.id, var->upper_margin);
	printk(KERN_ERR "%s: invalid lower_margin %d\n",
			info->fix.id, var->lower_margin);
#endif

	LCD_delay(50);
	/* enable win0 config*/
	write_reg(readl(fbi->regs + LCDC_SWITCH)|LCDC_ENCOF_WIN0, fbi->regs + LCDC_SWITCH);

	/*win0  config*/
	write_reg(fbi->map_dma, fbi->regs + Win0StartAddr0);
	write_reg((((var->yres & 0xFFF) - 1) << 12) | ((var->xres & 0xFFF) - 1) | (1<<24), fbi->regs + WIN0_CFG_A);
	write_reg(((var->xoffset & 0xFFF) << 12) | (var->yoffset & 0xFFF) | 0x7<<24, fbi->regs + WIN0_CFG_B);
	write_reg(var->xres-1, fbi->regs + WIN01_HSIZE);

//	write_reg((((var->yres & 0xFFF) - 1) << 12) | ((var->xres & 0xFFF) - 1) | (1<<24)|(1<<25), fbi->regs + WIN0_CFG_A);
//	write_reg(0xff0000,fbi->regs + WIN0_CFG_C);

	/*control config*/
	write_reg((var->xres_virtual-1) | (var->yres_virtual - 1) << 12, fbi->regs + BACKGROUD);
	write_reg(0x43, fbi->regs + PanelDataFormat);
	write_reg(LCDC_EN_INT, fbi->regs + LCDC_INT_MSK);
	write_reg(LCDC_DCLK_INPUT_CLK, fbi->regs + RGB_DCLK);
    write_reg(LCDC_BACKLIGHT_EN, fbi->regs + M_CS_CTRL);

	/*timing para*/
	write_reg((2 | 20 << 16), fbi->regs + RGB_H_TMG);
	write_reg((2  | 25 << 16), fbi->regs + RGB_V_TMG);
	write_reg((12 | 12 << 8), fbi->regs + RGB_W_TMG);

	/*RGB MODE/POLOAR*/
	write_reg(readl(fbi->regs + LCDC_GCTRL) & LCDC_SET_POLOAR_SM, fbi->regs + LCDC_GCTRL);
	write_reg(readl(fbi->regs + LCDC_GCTRL) | LCDC_EN_WOKOO | LCDC_RGB_MODE | LCDC_INT_SEL_FRAME, fbi->regs + LCDC_GCTRL);
	write_reg(readl(fbi->regs + LCDC_SWITCH) & LCDC_UNCOF_WIN0, fbi->regs + LCDC_SWITCH);

	/*start trans*/
	write_reg(readl(fbi->regs + LCDC_SWITCH) | LCDC_DATA_TRAN, fbi->regs + LCDC_SWITCH);

	return 0;
}

static int wokoofb_init_fbinfo(struct platform_device *pdev)
{

	struct fb_info *info = dev_get_drvdata(&pdev->dev);
	struct wokoofb_info *fbi = info->par;

	pr_debug("%s\n", __func__);

	info->pseudo_palette = kmalloc_array(16, sizeof(u32), GFP_KERNEL);
	if (!info->pseudo_palette)
		return -ENOMEM;

	memset(fbi, 0, sizeof(struct wokoofb_info));


	strlcpy(info->fix.id, DRIVER_NAME, sizeof(info->fix.id));

	info->fix = wokoofb_fix;
	info->var = wokoofb_default;
	info->fbops = &wokoofb_ops;
	info->fix.smem_len = info->var.xres * info->var.yres*(info->var.bits_per_pixel/8)*2;
	info->flags	= FBINFO_FLAG_DEFAULT |
				 FBINFO_READS_FAST;

	return 0;
}


static int wokoofb_probe(struct platform_device *pdev)
{

	struct wokoofb_info *fbi;
	struct fb_info *info;
	struct resource *res;
	const struct of_device_id *of_id;
	int ret;

	int gpio_switch;

	enum of_gpio_flags  flags;
	struct device_node *gpio_node = pdev->dev.of_node;
	void *fb_mem_virt;

#ifdef WOKOO_DEBUG
	printk("========enter wokoofb probe========\n");
#endif


	of_id = of_match_device(wokoofb_of_dev_id, &pdev->dev);
	if (of_id)
		pdev->id_entry = of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;


	info = framebuffer_alloc(sizeof(struct wokoofb_info), &pdev->dev);
	if (!info)
		return -ENOMEM;


	fbi = info->par;

	platform_set_drvdata(pdev, info);


	ret = wokoofb_init_fbinfo(pdev);
	if (ret < 0)
		goto failed_init;


	res = request_mem_region(res->start, resource_size(res),
				DRIVER_NAME);
	if (!res) {
		ret = -EBUSY;
		goto failed_req;
	}

	fbi->clk_ipg = devm_clk_get(&pdev->dev, "lcdc");
	if (IS_ERR(fbi->clk_ipg)) {
		ret = PTR_ERR(fbi->clk_ipg);
		goto failed_getclock;
	}

	ret = clk_prepare_enable(fbi->clk_ipg);
	if (ret)
		goto failed_getclock;


	fbi->clk_ahb = devm_clk_get(&pdev->dev, "axi");
	if (IS_ERR(fbi->clk_ahb)) {
		ret = PTR_ERR(fbi->clk_ahb);
		goto failed_getclock;
	}

	fbi->clk_per = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(fbi->clk_per)) {
		ret = PTR_ERR(fbi->clk_per);
		goto failed_getclock;
	}

	gpio_bl = of_get_named_gpio_flags(gpio_node, "light-gpio_bl", 0, &flags);
	if (gpio_is_valid(gpio_bl)) {
		gpio_direction_output(gpio_bl, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
		gpio_set_value(gpio_bl, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
		gpio_free(gpio_bl);
	} else
		pr_err("gpio_bl request fail\n");

		
	 gpio_switch = of_get_named_gpio_flags(gpio_node, "light-gpio_switch", 0, &flags);
		if (gpio_is_valid(gpio_switch)) {
			gpio_direction_output(gpio_switch, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
			gpio_set_value(gpio_switch, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
			gpio_free(gpio_switch);
		} else
			pr_err("gpio_switch request fail\n");

	gpio_reset = of_get_named_gpio_flags(gpio_node, "light-gpio_reset", 0, &flags);
		if (gpio_is_valid(gpio_reset)) {
			gpio_direction_output(gpio_reset, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
//			gpio_set_value(gpio_reset, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
		} else
			pr_err("gpio_reset request fail\n");



	gpio_sda = of_get_named_gpio_flags(gpio_node, "light-gpio_sda", 0, &flags);
		if (gpio_is_valid(gpio_sda)) {
			gpio_direction_output(gpio_sda, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
//			gpio_set_value(gpio_sda, (flags == GPIO_ACTIVE_HIGH) ? 1:0);

		} else
			pr_err("gpio_sda request fail\n");

	gpio_dclk = of_get_named_gpio_flags(gpio_node, "light-gpio_dclk", 0, &flags);
		if (gpio_is_valid(gpio_dclk)) {
			
			gpio_direction_output(gpio_dclk, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
//			gpio_set_value(gpio_dclk, (flags == GPIO_ACTIVE_HIGH) ? 1:0);

		} else
			pr_err("gpio_dclk request fail\n");

	  gpio_cs = of_get_named_gpio_flags(gpio_node, "light-gpio_cs", 0, &flags);
		if (gpio_is_valid(gpio_cs)) {
			gpio_direction_output(gpio_cs, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
//			gpio_set_value(gpio_cs, (flags == GPIO_ACTIVE_HIGH) ? 1:0);

		} else
			pr_err("gpio_cs request fail\n");

	//init spi screen  
	Lcd_Initialize_l();

	fbi->regs = ioremap(res->start, resource_size(res));
	if (fbi->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map frame buffer registers\n");
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	fbi->map_size = PAGE_ALIGN(info->fix.smem_len);
	fb_mem_virt = kzalloc(fbi->map_size, GFP_KERNEL);
	if (!fb_mem_virt) {
		pr_err("%s: Failed to allocate framebuffer\n", __func__);
		goto failed_map;
	}

	info->fix.smem_start = virt_to_phys(fb_mem_virt);
	fbi->map_dma = info->fix.smem_start;
	info->screen_base = fb_mem_virt;
	info->screen_size = info->fix.smem_len;

#ifdef WOKOO_DEBUG
	printk("wokoo fix.smem_len = %d", info->fix.smem_len);
	printk("wokoo map_size = %d", fbi->map_size);
	printk("wokoo phys addr =%x", fbi->map_dma);
#endif


	/*
	 * For modes > 8bpp, the color map is bypassed.
	 * Therefore, 256 entries are enough.
	 */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret < 0)
		goto failed_init;

	wokoofb_set_par(info);

	ret = register_framebuffer(info);

	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register framebuffer\n");
		goto failed_register;
	}
	wokoofb_enable_controller(fbi);

#ifdef WOKOO_DEBUG
	printk("======== liuyz exit wokoofb probe ========\n");
#endif
	return 0;


failed_register:
	fb_dealloc_cmap(&info->cmap);
failed_map:
	iounmap(fbi->regs);
failed_ioremap:
failed_getclock:
	release_mem_region(res->start, resource_size(res));
failed_req:
	kfree(info->pseudo_palette);
failed_init:
	framebuffer_release(info);
	return ret;

}

static int wokoofb_remove(struct platform_device *pdev)
{
	struct imx_fb_platform_data *pdata;
	struct fb_info *info = platform_get_drvdata(pdev);
	struct wokoofb_info *fbi = info->par;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	wokoofb_disable_controller(fbi);

	unregister_framebuffer(info);
	fb_dealloc_cmap(&info->cmap);
	pdata = dev_get_platdata(&pdev->dev);
	if (pdata && pdata->exit)
		pdata->exit(fbi->pdev);
	iounmap(fbi->regs);
	release_mem_region(res->start, resource_size(res));
	kfree(info->pseudo_palette);
	framebuffer_release(info);

	return 0;
}



static struct platform_driver wokoofb_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = wokoofb_of_dev_id,

	},
	.probe		= wokoofb_probe,
	.remove		= wokoofb_remove,
};
module_platform_driver(wokoofb_driver);

MODULE_DESCRIPTION("Wokoo framebuffer driver");
MODULE_AUTHOR("liuyz,Codefair");
MODULE_LICENSE("GPL");
