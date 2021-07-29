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


#define WOKOO_DEBUG    1


static const struct fb_var_screeninfo wokoofb_default = {
	.xres = 640,
	.yres = 480,
	.xres_virtual = 800,
	.yres_virtual = 480,
	.bits_per_pixel = 24,
	.red = {11, 5, 0},
	.green = {5, 6, 0},
	.blue = {0, 5, 0},
	.activate = FB_ACTIVATE_TEST,
	.height = 480,
	.width = 800,
	.pixclock = 31250,
	.left_margin = 46,			//back   (thb)
	.right_margin = 210,		//fron   porch   (thfb)
	.upper_margin = 23,			//back   (tvb)
	.lower_margin = 22,			//fron   (tvfb)
	.hsync_len = 20,
	.vsync_len = 3,
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

	/* enable win0 config*/
	write_reg(readl(fbi->regs + LCDC_SWITCH)|LCDC_ENCOF_WIN0, fbi->regs + LCDC_SWITCH);

	/*win0  config*/
	write_reg(fbi->map_dma, fbi->regs + Win0StartAddr0);
	write_reg((((var->yres & 0xFFF) - 1) << 12) | ((var->xres & 0xFFF) - 1) | (1<<24), fbi->regs + WIN0_CFG_A);
	write_reg(((var->xoffset & 0xFFF) << 12) | (var->yoffset & 0xFFF) | 0x7<<24, fbi->regs + WIN0_CFG_B);
	write_reg(var->xres-1, fbi->regs + WIN01_HSIZE);

	/*control config*/
	write_reg((var->xres_virtual-1) | (var->yres_virtual - 1) << 12, fbi->regs + BACKGROUD);
	write_reg(0x43, fbi->regs + PanelDataFormat);
	write_reg(LCDC_EN_INT, fbi->regs + LCDC_INT_MSK);
	write_reg(LCDC_DCLK_INPUT_CLK, fbi->regs + RGB_DCLK);
    write_reg(LCDC_BACKLIGHT_EN, fbi->regs + M_CS_CTRL);

	/*timing para*/
	write_reg((46 | 210 << 16), fbi->regs + RGB_H_TMG);
	write_reg((23  | 22 << 16), fbi->regs + RGB_V_TMG);
	write_reg((3 | 3 << 8), fbi->regs + RGB_W_TMG);

	/*RGB MODE/POLOAR*/
	write_reg(readl(fbi->regs + LCDC_GCTRL) & LCDC_SET_POLOAR, fbi->regs + LCDC_GCTRL);
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
	int gpio;
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

	gpio = of_get_named_gpio_flags(gpio_node, "light-gpio", 0, &flags);
	if (gpio_is_valid(gpio)) {
		ret = gpio_request(gpio, "light-gpio");
		gpio_direction_output(gpio, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
		gpio_set_value(gpio, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
		gpio_free(gpio);
	} else
		pr_err("light-gpio request fail\n");


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
