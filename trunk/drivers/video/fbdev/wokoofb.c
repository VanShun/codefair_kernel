// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Wokoofb.c Wokoo Frame Buffer device driver
 *
 */
 
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <video/of_display_timing.h>
#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include<linux/spinlock.h>
#include <linux/ioport.h>
#include <linux/reset.h>
#include <linux/property.h>

#include "wokoofb_reg.h"

#define DRIVER_NAME	"Wokoo_fb"

struct wokoofb_info {
	void __iomem			*regs;
	struct reset_control    *rst;
	struct clk				*clk_ipg;
	struct clk				*clk_ahb;
	struct clk				*clk_per;
	bool					enabled;
	bool					clk_enabled;
	dma_addr_t				map_dma;
	u_int					map_size;
	u_int					palette_size;
	bool					cmap_inverse;
	bool					cmap_static;
	int                     sync_polar;
	int                     dotclk_polar;
	int                     en_polar;
	unsigned long           value_rate;
	bool                    issuspend;
	int                     irq;
	struct work_struct	    task;
	char                    cfg_win0_fmt;
	char                    cfg_color_dep;
	struct backlight_device	*backlight;
	struct fb_info          *info;
	struct mutex            lock;
};

static const struct fb_fix_screeninfo wokoofb_fix = {
	.id = "wokoofb_fix_id",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.type_aux = 0,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE,
};

/**
*wokoo_of_get_backlight-get devicetree backlight device 
*@child_np: wokoofb devicetree node
*@fb:   wokoo_fb info
**/
static int wokoo_of_get_backlight(struct fb_info *info)
{
	 struct device *dev  = info->device;
	 struct wokoofb_info  *fbi = info->par;

	fbi->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(fbi->backlight))
	{
	//	printk("[%s]:find not backlight12\n",__FUNCTION__);
		return PTR_ERR(fbi->backlight);
	}
	return 0;
}

/**
*wokoofb_turn_on_bl-turn on  backlight
*@fb:   wokoo_fb info
**/
static void wokoofb_turn_on_bl(struct wokoofb_info  *fbi)
{
	     /*turn on backlight*/
	if (fbi && fbi ->backlight)
	{
		fbi ->backlight->props.power=FB_BLANK_UNBLANK;
		backlight_update_status(fbi ->backlight);
	}
}

/**
*wokoofb_turn_on_bl-turn off  backlight
*@fb:   wokoo_fb info
**/
static void wokoofb_turn_off_bl(struct wokoofb_info  *fbi)
{
    printk("----wokoofb_turn_off_bl---\n");
    /*turn off backlight*/
	if (fbi && fbi ->backlight)
	{
		fbi ->backlight->props.power=FB_BLANK_POWERDOWN;
		backlight_update_status(fbi ->backlight);
	}
}

/**
*wokoo_enable_ctrl-enable lcd ctrl 
*@info:fb info
**/
static void wokoo_enable_ctrl(struct fb_info *info)
{
	struct wokoofb_info  *fbi = info->par;
	pr_debug("Enabling LCD controller\n");
	if (fbi)
	{
		if (false == fbi ->clk_enabled)
		{
		    /*enable clk*/
	        clk_prepare_enable(fbi->clk_ahb);
	        clk_prepare_enable(fbi->clk_ipg);
            clk_prepare_enable(fbi->clk_per);
	        clk_set_rate(fbi->clk_ahb, 128000000);
	        clk_set_rate(fbi->clk_per,  fbi->value_rate);
			fbi ->clk_enabled = true;
		}
		//udelay(10);
		pr_debug("ahb clk set value = %ld", clk_get_rate(fbi->clk_ahb));
	    pr_debug("per clk set value= %ld", clk_get_rate(fbi->clk_per));
		/*enable controller*/
		write_reg(readl(fbi->regs + LCDC_GCTRL_ADDR)|LCDC_EN_WOKOO, fbi->regs + LCDC_GCTRL_ADDR);
	}
}

/**
*wokoo_disable_ctrl-disable lcd ctrl 
*@info:fb info
**/
static void wokoo_disable_ctrl(struct fb_info *info)
{
	struct wokoofb_info  *fbi = info->par;
	if (fbi)
	{
	  write_reg(readl(fbi->regs + LCDC_GCTRL_ADDR) & ~LCDC_EN_WOKOO, fbi->regs + LCDC_GCTRL_ADDR);
	
		if (true == fbi ->clk_enabled)
		{
		   clk_disable_unprepare(fbi->clk_ahb);
		   clk_disable_unprepare(fbi->clk_per);
		   clk_disable_unprepare(fbi->clk_ipg);
		   fbi ->clk_enabled = false;	   
		}
	}
}

/**
*wokoo_rst_ctrl-reset lcdc ctrl
*@info:fb info
**/
static void wokoo_rst_ctrl(struct wokoofb_info  *fbi)
{
	reset_control_assert(fbi->rst);
	udelay(1);
	reset_control_deassert(fbi->rst);
	udelay(1);
}

/**
 * wokoofb_activate_var():
 * @info: fb info
 *	Configures LCD Controller based on entries in var parameter.  Settings are
 *	only written to the controller if changes were made.
 **/
static int wokoofb_activate_var(struct fb_info *info)
{
	struct wokoofb_info *fbi = info->par;
    struct fb_var_screeninfo *var = &info->var;
	u8 bits_per_pixel =0;
	pr_debug("var: xres=%d hslen=%d lm=%d rm=%d\n",
		var->xres, var->hsync_len,
		var->left_margin, var->right_margin);
	pr_debug("var: yres=%d vslen=%d um=%d bm=%d\n",
		var->yres, var->vsync_len,
		var->upper_margin, var->lower_margin);
	pr_debug("var: xoffset=%d yoffset=%d bits_per_pixel=%d cfg_color_dep=%d\n",
		var->xoffset, var->yoffset,
		var->bits_per_pixel, fbi->cfg_color_dep);
	pr_debug("var:pclock:%d,sync_polar:%d,en_polar:%d,dotclk_polar%d\n",var ->pixclock,fbi->sync_polar,fbi->en_polar,fbi->dotclk_polar);
	pr_debug("var->sync:%d\n",var->sync);
	
	pr_debug("var:LCDC_SWITCH_ADDR:0x%x\n",readl(fbi->regs + LCDC_SWITCH_ADDR));
	pr_debug("var:LCDC_GCTRL_ADDR:0x%x\n",readl(fbi->regs + LCDC_GCTRL_ADDR));
	pr_debug("var:WIN01_HSIZE_ADDR:0x%x\n",readl(fbi->regs + WIN01_HSIZE_ADDR));
	
	pr_debug("var:WIN0_CFG_A_ADDR:0x%x\n",readl(fbi->regs + WIN0_CFG_A_ADDR));
	pr_debug("var:RGB_V_TMG_ADDR:0x%x\n",readl(fbi->regs + RGB_V_TMG_ADDR));
	pr_debug("var:RGB_W_TMG_ADDR:0x%x\n",readl(fbi->regs + RGB_W_TMG_ADDR));
	pr_debug("var:RGB_H_TMG_ADDR:0x%x\n",readl(fbi->regs + RGB_H_TMG_ADDR));
	pr_debug("var:RGB_DCLK_ADDR:0x%x\n",readl(fbi->regs + RGB_DCLK_ADDR));
	
	/* enable win0 config*/
	write_reg(readl(fbi->regs + LCDC_SWITCH_ADDR)|LCDC_ENCOF_WIN0, fbi->regs + LCDC_SWITCH_ADDR);

	/*win0  config*/
	write_reg(fbi->map_dma, fbi->regs + WIN0_START_ADDR0);
	write_reg((((var->yres & 0xFFF) - 1) << 12) | ((var->xres & 0xFFF) - 1) | CFG_WIN0_EN, fbi->regs + WIN0_CFG_A_ADDR);
	write_reg(((var->xoffset & 0xFFF) << 12) | (var->yoffset & 0xFFF) | ((fbi->cfg_win0_fmt & 0xff)<< 24), fbi->regs + WIN0_CFG_B_ADDR);
	//	write_reg((((var->yres & 0xFFF) - 1) << 12) | ((var->xres & 0xFFF) - 1) | (1<<24)|(1<<25), fbi->regs + WIN0_CFG_A_ADDR);
	//write_reg(0xff0000,fbi->regs + WIN0_CFG_C);
  
	write_reg(var->xres-1, fbi->regs + WIN01_HSIZE_ADDR);
	
	/*control config*/
	write_reg((var->xres_virtual-1) | (var->yres_virtual - 1) << 12, fbi->regs + BACKGROUD_ADDR);
	if ( var ->bits_per_pixel > 24)
		bits_per_pixel = 24;
	else
		bits_per_pixel = var ->bits_per_pixel;
	write_reg(readl(fbi->regs + PAL_DAT_FORMAT_ADDR)  \
	         | (((bits_per_pixel/8) & 0x3) << 0) \
			 | ((fbi->cfg_color_dep & 0x7) << 4), fbi->regs + PAL_DAT_FORMAT_ADDR);
			 
	write_reg(LCDC_EN_INT, fbi->regs + LCDC_INT_MSK_ADDR);
	write_reg(LCDC_DCLK_INPUT_CLK, fbi->regs + RGB_DCLK_ADDR);
    write_reg(LCDC_BACKLIGHT_EN, fbi->regs + M_CS_CTRL_ADDR);

	/*timing para*/
	 write_reg((readl(fbi->regs + RGB_H_TMG_ADDR)& 0x0) \
	           |((var->left_margin & 0xffff) << CFG_RGB_HBK_SFET)    \
	           | ((var->right_margin & 0xffff) << CFG_RGB_HFP_SFET),  \
			   fbi->regs + RGB_H_TMG_ADDR);
	 	   
    write_reg((readl(fbi->regs + RGB_V_TMG_ADDR) & 0x0)  \
	           |((var->upper_margin & 0xffff) << CFG_RGB_VBK_SFET)   \
	           | ((var->lower_margin & 0xffff) << CFG_RGB_VFP_SFET),  \
			   fbi->regs + RGB_V_TMG_ADDR);
	
    write_reg((readl(fbi->regs + RGB_W_TMG_ADDR) & ~0xffff)  \
		      |(((var->hsync_len) & 0xff) << CFG_RGB_HPW_SFET)   \
		      | (((var->vsync_len) & 0xff) << CFG_RGB_VPW_SFET), \
		      fbi->regs + RGB_W_TMG_ADDR);
			   
	
	/*RGB MODE/POLOAR*/
	//write_reg(readl(fbi->regs + LCDC_GCTRL_ADDR) & LCDC_SET_POLOAR_SM, fbi->regs + LCDC_GCTRL_ADDR);
	write_reg((readl(fbi->regs + LCDC_GCTRL_ADDR) & ~(3<< 7))| ((var->sync & 0x3) << 7), fbi->regs + LCDC_GCTRL_ADDR);
	//write_reg((readl(fbi->regs + LCDC_GCTRL_ADDR) & ~(3<< 7))| (fbi->sync_polar << 7), fbi->regs + LCDC_GCTRL_ADDR);
	write_reg(readl(fbi->regs + LCDC_GCTRL_ADDR) | LCDC_EN_WOKOO | LCDC_RGB_MODE  \
	                | LCDC_INT_SEL_FRAME|(fbi->dotclk_polar << 6)   \
					|(fbi->en_polar << 5), fbi->regs + LCDC_GCTRL_ADDR);
					
	write_reg(readl(fbi->regs + LCDC_SWITCH_ADDR) & LCDC_UNCOF_WIN0, fbi->regs + LCDC_SWITCH_ADDR);
	
	/*start trans*/
	write_reg(readl(fbi->regs + LCDC_SWITCH_ADDR) | LCDC_DATA_TRAN, fbi->regs + LCDC_SWITCH_ADDR);
	return 0;
}

/**
*wokoofb_set_par-set lcd params
*@info:fb info
**/
static int wokoofb_set_par(struct fb_info *info)
{
	struct wokoofb_info *fbi = info->par;
	
	if (24 == info->var.bits_per_pixel 
	|| 32 == info->var.bits_per_pixel) 
	{
        info->var.red.offset = 0;
		info->var.red.length = 8;
		info->var.red.msb_right = 0;
		info->var.green.offset = 8;
		info->var.green.length = 8;
		info->var.green.msb_right = 0;
		info->var.blue.offset = 16;
		info->var.blue.length = 8;
		info->var.blue.msb_right = 0;

		//info->fix.line_length = info->var.xres_virtual << 2;
		fbi->cfg_win0_fmt = DPI_WOKOO_RGB8888;
		fbi->cfg_color_dep = DPI_WOKOO_DEP_RGB888;
	} 
	else if (info->var.bits_per_pixel == 16) 
	{
		info->var.red.offset = 11;
		info->var.red.length = 5;
		info->var.red.msb_right = 0;
		info->var.green.offset = 5;
		info->var.green.length = 6;
		info->var.green.msb_right = 0;
		info->var.blue.offset = 0;
		info->var.blue.length = 5;
		info->var.blue.msb_right = 0;
		//info->fix.line_length = info->var.xres_virtual << 1;
		fbi->cfg_win0_fmt = DPI_WOKOO_RGB565;
		fbi->cfg_color_dep = DPI_WOKOO_DEP_RGB565;
	}
	else
	{
		fbi->cfg_win0_fmt = DPI_WOKOO_RGB8888;
		fbi->cfg_color_dep = DPI_WOKOO_DEP_RGB888;
	}

	if (info->var.bits_per_pixel == 16 
	    || info->var.bits_per_pixel == 32 
	    || info->var.bits_per_pixel == 24)
	{
		info->fix.visual = FB_VISUAL_TRUECOLOR;
	}
	else if (!fbi->cmap_static)
	{
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	}
	else 
	{
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		info->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}
	
	info->fix.line_length = info->var.xres_virtual * info->var.bits_per_pixel / 8;
	fbi->palette_size = info->var.bits_per_pixel == 8 ? 256 : 16;
	mutex_lock(&fbi->lock);
	wokoofb_activate_var(info);
	mutex_unlock(&fbi->lock);
	return 0;
}

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

/**
*wokoofb_set_par-set lcd colour
*@info:fb info
*@regno:bit number
*@red ,green,blue:
*@trans:
**/
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

static int wokoofb_blank(int blank, struct fb_info *info)
{
	//struct wokoofb_info *fbi = info->par;
	pr_debug("wokoofb_blank: blank=%d\n", blank);
	
	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
	    wokoo_disable_ctrl(info);
		break;
	case FB_BLANK_UNBLANK:
	    wokoo_enable_ctrl(info);
		break; 
	}
	return 0;
}

static struct fb_ops wokoofb_ops = {
	.owner			= THIS_MODULE,
	.fb_set_par		= wokoofb_set_par,				
	.fb_setcolreg	= wokoofb_setcolreg,		
	.fb_blank		= wokoofb_blank,				
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,

};

static const struct of_device_id wokoofb_of_dev_ids[] = {
	{
		.compatible = "wokoo,wokoo-fb",

	},  {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, wokoofb_of_dev_ids);

/**
*wokoofb_dt_init-read wokoo devicetree 
*@info:fb info
**/
static int  wokoofb_dt_init(struct fb_info *info)
{
    struct device *dev  = info->device;
	struct device_node *np =dev->of_node;
	struct fb_var_screeninfo *var = &info->var;
	struct  wokoofb_info *fbi = info->par;
	struct fb_videomode fb_mode;
	int ret = 0;
	
	ret = of_property_read_u32(np, "bits-per-pixel", &var->bits_per_pixel);
	if (ret < 0) 
	{
		dev_err(dev, "failed to get property bits-per-pixel:%d\n",ret);
		return  -ENODEV;
	}
		
	ret = of_property_read_u32(np, "dotclk_polar", &fbi->dotclk_polar);
	if (ret < 0) 
	{
		dev_err(dev, "failed to get property dotclk_polar:%d\n",ret);
		return  -ENODEV;
	}
	

	ret = of_property_read_u32(np, "en_polar", &fbi->en_polar);
	if (ret < 0) 
	{
		dev_err(dev, "failed to get property en_polar:%d\n",ret);
		return  -ENODEV;
	}

	ret = of_get_fb_videomode(np, &fb_mode, OF_USE_NATIVE_MODE);
	if (ret) 
	{
		dev_err(dev, "failed to get videomode from DT:%d\n",ret);
		return  -ENODEV;
	}
	
	fb_videomode_to_var(var, &fb_mode);
	var->nonstd		= 0;
	var->activate	= FB_ACTIVATE_NOW;
	var->height		= -1;
	var->width		= -1;
	ret = wokoo_of_get_backlight(info);
	return ret;
}

/**
*wokoofb_init_fbinfo-init lcd ctrl
*@info:fb info
**/
static int wokoofb_init_fbinfo(struct fb_info *info)
{
	struct wokoofb_info *fbi = info->par;
	void *fb_mem_virt = NULL;
	unsigned long smem_len;
	pr_debug("%s\n", __func__);

	info->pseudo_palette = kmalloc_array(16, sizeof(u32), GFP_KERNEL);
	if (NULL == info->pseudo_palette)
	{
		//dev_err(dev malloc pseudo_palette failed\n");
			return -ENOMEM;
	}	
	strlcpy(info->fix.id, DRIVER_NAME, sizeof(info->fix.id));
	info->fix = wokoofb_fix;
	info->fbops = &wokoofb_ops;
	
	smem_len = info->var.xres ;
	smem_len *= info->var.yres;
	smem_len *= (info->var.bits_per_pixel/8);
	if (info->fix.smem_len  < smem_len)
		info->fix.smem_len = smem_len;
	info->flags	= FBINFO_FLAG_DEFAULT; //|
			 //FBINFO_READS_FAST;
	   
	fbi->map_size = PAGE_ALIGN(info->fix.smem_len);
	fb_mem_virt = kzalloc(fbi->map_size, GFP_KERNEL);
	if (!fb_mem_virt) {
		if (info->pseudo_palette)
		{
			kfree(info->pseudo_palette);
			info->pseudo_palette = NULL;
		}
		pr_err("%s: Failed to allocate pseudo_palette\n", __func__);
		return  -ENOMEM;
	}

	info->fix.smem_start = virt_to_phys(fb_mem_virt);
	fbi->map_dma = info->fix.smem_start;
	info->screen_base = fb_mem_virt;
	info->screen_size = info->fix.smem_len;
	   
	/*set dotclk*/
	fbi->value_rate = DIV_ROUND_UP(PICOS2KHZ(info->var.pixclock),8000);
	fbi->value_rate = fbi->value_rate*8000 *1000;
	return 0;
}

/**
*wokoofb_reinit_info-reinit lcd ctrl
*@info:fb info
**/
static void wokoofb_reinit_info(struct fb_info *info)
{
	if (NULL != info)
	{
		wokoofb_turn_off_bl(info->par);
		wokoo_disable_ctrl(info);
	
		if (NULL != info->pseudo_palette)
	    {
		    kfree(info->pseudo_palette);
		    info->pseudo_palette = NULL;
	    }
			
	    if (info->screen_base)
	    {
		    kfree(info->screen_base);
		    info->screen_base = NULL;
	    }
	}	
}

/**
*wokoofb_interrupt- lcd interrupt handle
*@irq:irq num
*@dev_id:data
**/
static irqreturn_t wokoofb_interrupt(int irq, void *dev_id)
{
	struct fb_info *info = dev_id;
	struct wokoofb_info *fbi = info->par;
	u32 status;
	
	status = readl(fbi ->regs+ LCDC_INT_STATUS_ADDR);
	if (status & WOKOOFB_INTR_UFLWI) 
	{
		dev_warn(info->device, "FIFO underflow %#x\n", status);
		schedule_work(&fbi->task);
	}
	
	write_reg(status,fbi ->regs+LCDC_INT_CLR_ADDR);
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM_SLEEP
/**
*wokoofb_suspend- lcdc  suspend
*@dev
**/
static int wokoofb_suspend(struct device *dev)
{
    struct fb_info	*data = dev_get_drvdata(dev);
	struct wokoofb_info *fbi  = data->par ;

    printk("----debug wokoofb_suspend ---\n");
	if (true == fbi ->issuspend)
    {
        return 0;
    }
//	fb_set_suspend(data, 1);
    wokoo_disable_ctrl(data);
 	wokoofb_turn_off_bl(fbi);
	fbi ->issuspend = true;
	return 0;
}

/**
*wokoofb_resume- lcdc  resume
*@dev
**/
static int wokoofb_resume(struct device *dev)
{
	struct fb_info  *data = dev_get_drvdata(dev);
	struct wokoofb_info *fbi  = data->par ;
	
	if (false == fbi ->issuspend)
    {
        return 0;	
    }
	
	wokoofb_turn_on_bl(fbi);
	wokoo_rst_ctrl(fbi);
	wokoofb_set_par(data);
	wokoo_enable_ctrl(data);
	//fb_set_suspend(data, 0);
	fbi ->issuspend = false;	
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops wokoofb_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wokoofb_suspend, wokoofb_resume)
};

/*
 * LCD controller task (to reset the LCD)
 */
static void wokoofb_lcdfb_task(struct work_struct *work)
{
	struct wokoofb_info *fbi =
		container_of(work, struct wokoofb_info, task);
	 if (fbi && fbi->rst)
     {
         printk("---debug wokoofb_lcdfb_task rst\n");
		 wokoo_disable_ctrl(fbi->info);
		 wokoo_rst_ctrl(fbi);
		 wokoofb_set_par(fbi->info);
		 wokoo_enable_ctrl(fbi->info);
	 }	
}

/**
*wokoofb_probe- lcd probe
*@pdev:
**/
static int wokoofb_probe(struct platform_device *pdev)
{
	struct wokoofb_info *fbi;
	struct fb_info *info;
	struct resource *res;
	int ret;
    u32 lcdc_ctrl;
    
    printk("----debug wokoofb_probe\n");
	info = framebuffer_alloc(sizeof(struct wokoofb_info), &pdev->dev);
	if (NULL == info)
	{
		dev_err(&pdev->dev,"malloc framebuffer failed\n");
		return -ENOMEM;
	}	
	//memset(info, 0, sizeof(struct fb_info));
	if (pdev->dev.of_node)
	{
		ret = wokoofb_dt_init(info);	
		if (ret)
		{
			dev_err(&pdev->dev, "cannot get dt configuration\n");
		    goto free_info;
		}
	} else {
		dev_err(&pdev->dev, "cannot get dt  configuration\n");
		ret = -ENODEV;
		goto free_info;	
	}
	
    fbi = info->par;
	fbi ->info = info;
	mutex_init(&fbi->lock);
	/*open bl*/
	wokoofb_turn_on_bl(fbi);
	/*IO Remap*/
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fbi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fbi->regs))
	{
	    dev_err(&pdev->dev, "wkoofb ioremap failed\n");
		ret = PTR_ERR(fbi->regs);
		goto free_info;
	}	 
	
	/* rest crtl*/
    fbi->rst = devm_reset_control_get(&pdev->dev, "reset");
	if (!IS_ERR(fbi->rst)) 
	{
		wokoo_rst_ctrl(fbi);
	}
	
	/* interrupt */
	fbi->irq = platform_get_irq(pdev, 0);
    printk("---debug wokoofb irq :%d", fbi->irq);
	if (fbi->irq < 0) 
	{
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = fbi->irq;
		goto free_info;
	}
	
	ret = request_irq(fbi->irq, wokoofb_interrupt, 0, pdev->name, info);
	if (ret) 
	{
		dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
		goto free_info;
	}

    lcdc_ctrl = readl(fbi ->regs + LCDC_GCTRL_ADDR);
    printk("----debug lcdc_ctrl: %x\n", lcdc_ctrl);
   // write_reg(lcdc_ctrl | (1 << 15), fbi ->regs + LCDC_GCTRL_ADDR);
	/* clk init*/
	fbi->clk_ipg = devm_clk_get(&pdev->dev, "lcdc");
	if (IS_ERR(fbi->clk_ipg)) 
	{
		ret = PTR_ERR(fbi->clk_ipg);
		goto failed_clk;
	}
	ret = clk_prepare_enable(fbi->clk_ipg);
	if (ret)
	{
        goto failed_clk;
	}
	fbi->clk_ahb = devm_clk_get(&pdev->dev, "axi");
	if (IS_ERR(fbi->clk_ahb)) 
	{
		ret = PTR_ERR(fbi->clk_ahb);
		goto failed_clk;
	}
	fbi->clk_per = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(fbi->clk_per)) 
	{
		ret = PTR_ERR(fbi->clk_per);
		goto failed_clk;
	}
	ret = wokoofb_init_fbinfo(info);
	if (ret)
	{
		dev_err(&pdev->dev, "wkoofb init failed\n");
		goto failed_clk;
	}
     /*
	 * For modes > 8bpp, the color map is bypassed.
	 * Therefore, 256 entries are enough.
	 */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret < 0)
	{ 
		pr_err("%s: Failed to allocate cmap\n", __func__);
		return  -ENOMEM;
	} 
	platform_set_drvdata(pdev, info);
	wokoofb_set_par(info);
	ret = register_framebuffer(info);
	if (ret < 0) 
	{
		dev_err(&pdev->dev,
			"Failed to register framebuffer device: %d\n", ret);
		goto unregister;
	}
	wokoo_enable_ctrl(info);
	INIT_WORK(&fbi->task, wokoofb_lcdfb_task);
	pr_info("wokoofb controller init success\n");
	fbi->enabled = true;
	return 0;
unregister:
    unregister_framebuffer(info);	
failed_clk:

	 if (fbi->clk_ipg)
	 {
		 clk_disable_unprepare(fbi->clk_ipg);
	 }
free_info:
     wokoofb_reinit_info(info);
	 framebuffer_release(info);
	 fbi->enabled = false;
	return ret;
}

/**
*wokoofb_remove- lcd remove
*@pdev:
**/
static int wokoofb_remove(struct platform_device *pdev)
{	
	 struct fb_info *info = platform_get_drvdata(pdev);
	 struct wokoofb_info *fbi = info->par;
     if (fbi->enabled != true)
     {
		 return 0;
	 }
	 if (fbi->irq > 0)
	 {
		 free_irq(fbi->irq,info);
	 }
     wokoofb_reinit_info(info);
	 if (info->cmap.len)
	 {
		fb_dealloc_cmap(&info->cmap);
		info->cmap.len = 0;
	 }
     unregister_framebuffer(info);
	 framebuffer_release(info);
	return 0;
}

static struct platform_driver wokoofb_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = wokoofb_of_dev_ids,
		.pm = &wokoofb_pm_ops,

	},
	.probe		= wokoofb_probe,
	.remove		= wokoofb_remove,
};


module_platform_driver(wokoofb_driver);
MODULE_DESCRIPTION("Wokoo framebuffer driver");
MODULE_AUTHOR("linsh,Codefair");
MODULE_LICENSE("GPL");
