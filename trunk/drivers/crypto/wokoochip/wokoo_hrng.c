/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 */
#include <linux/module.h>  
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>  
#include <linux/fs.h>  
#include <linux/interrupt.h>  
#include <linux/irq.h>  
#include <linux/sched.h>  
#include <linux/pm.h>  
#include <linux/sysctl.h>  
#include <linux/proc_fs.h>  
#include <linux/delay.h>  
#include <linux/input.h>  
#include <asm/uaccess.h>   
#include <asm/io.h>  

#include <linux/cdev.h>
#include <linux/device.h>

#include <linux/of.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>

#include "wokoo_key.h"

static struct class *hrng_cls;  
static int major = 0; 
static void __iomem *_iobase;
static spinlock_t lock;

uint8_t get_hrng(void)
{	
	iowrite32(0x02, _iobase + HRNG_CMPRES_REG);
	
	iowrite32(ioread32(_iobase + HRNG_CTRL_REG) | HRNG_CTRL_RNG_EN0 | HRNG_CTRL_RNG_EN1 | HRNG_CTRL_RNG_EN2 | 
	          HRNG_CTRL_RNG_EN3 | HRNG_CTRL_SCLK_SEL,_iobase + HRNG_CTRL_REG);

	while ((ioread32(_iobase + HRNG_STATUS_REG) & HRNG_STATUS_FIFO_NOT_EMPTY) == 0);

	return (uint8_t)(ioread32(_iobase + HRNG_LFSR_REG));
}

static int uac_open(struct inode *inode, struct file * file)  
{  
	//printk("uac_open hrng...\n");
    return 0;  
}  

static ssize_t uac_read(struct file * file, char __user *buffer, size_t size , loff_t *p)
{
	unsigned char dataout = 0;
	
	//printk("uac_read hrng...\n");
	
	iowrite32(0x02, _iobase + HRNG_CMPRES_REG);
	
	iowrite32(ioread32(_iobase + HRNG_CTRL_REG) | HRNG_CTRL_RNG_EN0 | HRNG_CTRL_RNG_EN1 | HRNG_CTRL_RNG_EN2 | 
	          HRNG_CTRL_RNG_EN3 | HRNG_CTRL_SCLK_SEL,_iobase + HRNG_CTRL_REG);

	while ((ioread32(_iobase + HRNG_STATUS_REG) & HRNG_STATUS_FIFO_NOT_EMPTY) == 0);

	dataout = ioread32(_iobase + HRNG_LFSR_REG);
	
	//printk("dataout = %x, size = %x\n", dataout, size);
	
	if (copy_to_user(buffer, (unsigned char *)&dataout, size))
		return -1;
	
	//printk("uac_read hrng...end\n");
	
	return 0;
}

static const struct file_operations hrng_fops = {  
    .owner = THIS_MODULE,  
    .open  = uac_open,  
	.read  = uac_read,
};  

static int wokoo_hrng_probe(struct platform_device *pdev)  
{  
	struct resource *res;
	struct wokoo_crypto_info *crypto_info;
	int err = 0;

	crypto_info = devm_kzalloc(&pdev->dev, sizeof(*crypto_info), GFP_KERNEL);
	if (!crypto_info) {
		err = -ENOMEM;
		return err;
	}

	spin_lock_init(&lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	_iobase = devm_ioremap_resource(&pdev->dev, res);
	//printk("res = %x, _iobase = %x\n", res->start, _iobase);
	if (IS_ERR(_iobase)) {
		err = PTR_ERR(_iobase);
		return err;
	}

#if 1
	crypto_info->hclk = devm_clk_get(&pdev->dev, "hclk");
    if (IS_ERR(crypto_info->hclk)) {
        err = PTR_ERR(crypto_info->hclk);
        dev_err(&pdev->dev, "failed to get crypto clk: %d\n", err);
        return err;
    }
	
	err = clk_prepare_enable(crypto_info->hclk);
	if (err) {
		dev_err(&pdev->dev, "failed to prepare crypto clock\n");
		return err;
	}
	
	crypto_info->hrng_clk = devm_clk_get(&pdev->dev, "hrng_clk");
    if (IS_ERR(crypto_info->hrng_clk)) {
        err = PTR_ERR(crypto_info->hrng_clk);
        dev_err(&pdev->dev, "failed to get crypto clk: %d\n", err);
        return err;
    }
	
	err = clk_prepare_enable(crypto_info->hrng_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to prepare crypto clock\n");
		return err;
	}
	
	crypto_info->alg_clk = devm_clk_get(&pdev->dev, "alg_clk");
    if (IS_ERR(crypto_info->alg_clk)) {
        err = PTR_ERR(crypto_info->alg_clk);
        dev_err(&pdev->dev, "failed to get crypto clk: %d\n", err);
        return err;
    }
	
	err = clk_prepare_enable(crypto_info->alg_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to prepare crypto clock\n");
		return err;
	}
	
	crypto_info->sclk = devm_clk_get(&pdev->dev, "sclk");
    if (IS_ERR(crypto_info->sclk)) {
        err = PTR_ERR(crypto_info->sclk);
        dev_err(&pdev->dev, "failed to get crypto clk: %d\n", err);
        return err;
    }
	clk_set_rate(crypto_info->sclk, 256000000);
#endif

	crypto_info->dev = &pdev->dev;
	platform_set_drvdata(pdev, crypto_info);

    major = register_chrdev(0, "wokoo_hrng", &hrng_fops);
    hrng_cls = class_create(THIS_MODULE,"wokoo_hrng");  
    device_create(hrng_cls, NULL, MKDEV(major, 0), NULL, "wokoo_hrng"); 

	//printk("wokoo_uac_hrng register--------------------------------------\n");
	
    return 0;  
}  

static int wokoo_hrng_remove(struct platform_device *pdev)  
{  
    device_destroy(hrng_cls, MKDEV(major, 0));  
    class_destroy(hrng_cls);  
    unregister_chrdev(major, "wokoo_hrng");  
	
    return 0;  
}
  
static const struct of_device_id hrng_id_table[] = {
	{ .compatible = "wokoo,wokoo-hrng" },
	{}
};
MODULE_DEVICE_TABLE(of, hrng_id_table);

static struct platform_driver hrng_driver = {
	.probe		= wokoo_hrng_probe,
	.remove		= wokoo_hrng_remove,
	.driver		= {
		.name	= "wokoo-hrng",
		.of_match_table	= hrng_id_table,
	},
};
module_platform_driver(hrng_driver); 
  
MODULE_ALIAS("platform:wokoo-hrng");
MODULE_DESCRIPTION("WOKOO hrng driver");
MODULE_LICENSE("GPL v2"); 
