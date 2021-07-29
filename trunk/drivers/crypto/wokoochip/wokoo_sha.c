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

static struct class *sha_cls;  
static int major = 0; 

static void __iomem *_iobase;
static spinlock_t lock;

#define CIPHER_SHA_TYPE_CPU    0x00
#define CIPHER_SHA_TYPE_DMA    0x01

#define CIPHER_SHA_MOD_SHA1    0x00
#define CIPHER_SHA_MOD_SHA256  0x01

struct wokoo_sha_t {
	uint32_t bit_mod;
	uint32_t data_mod;
	uint32_t line_num;
	uint32_t input_data[16];
	uint32_t output_data[16];
};
struct wokoo_sha_t wokoo_sha;

void HAL_CIPHER_SHA_DMA(void)
{
	int i = 0;
	
	iowrite32(1, _iobase + SHA_INTE_REG);
	
	iowrite32(CIPHER_SHA_TYPE_DMA, _iobase + SHA_IN_TYPE_REG);
	
	iowrite32(wokoo_sha.bit_mod, _iobase + SHA_MOD_REG);
	
	iowrite32((uint32_t)(wokoo_sha.input_data), _iobase + SHA_SADDR_REG);
	
	iowrite32(wokoo_sha.line_num, _iobase + SHA_LINE_NUM_REG);
	
	iowrite32(1, _iobase + SHA_CTL_REG);

	while (ioread32(_iobase + SHA_INTS_REG) == 0);
	iowrite32(1, _iobase + SHA_INTS_REG);
	
	if (ioread32(_iobase + SHA_MOD_REG) == CIPHER_SHA_MOD_SHA1) {
		for (i = 0; i < 5; i++)
			wokoo_sha.output_data[i] = ioread32(_iobase + SHA_H0_REG + i * 4);
	} else {
		for (i = 0; i < 8; i++)
			wokoo_sha.output_data[i] = ioread32(_iobase + SHA_H0_REG + i * 4);
	}
}

void HAL_CIPHER_SHA_CPU(void)
{
	int i = 0;
	//int num = wokoo_sha.line_num - 1;
	int num = wokoo_sha.line_num;

	iowrite32(1, _iobase + SHA_INTE_REG);
	
	iowrite32(CIPHER_SHA_TYPE_CPU, _iobase + SHA_IN_TYPE_REG);

	iowrite32(wokoo_sha.bit_mod, _iobase + SHA_MOD_REG);
	
	for (i = 0; i < 16; i++)
		iowrite32(wokoo_sha.input_data[i], _iobase + SHA_W0_REG + i * 4);
	
	iowrite32(1, _iobase + SHA_CTL_REG);

	while (ioread32(_iobase + SHA_INTS_REG) == 0);

	iowrite32(1, _iobase + SHA_INTS_REG);
	
	while (num--) {
		for (i = 0; i < 16; i++)
			iowrite32(wokoo_sha.input_data[i], _iobase + SHA_W0_REG + i * 4);
		
		iowrite32(2, _iobase + SHA_CTL_REG);

		while (ioread32(_iobase + SHA_INTS_REG) == 0);
		iowrite32(1, _iobase + SHA_INTS_REG);
	}
	
	if (ioread32(_iobase + SHA_MOD_REG) == CIPHER_SHA_MOD_SHA1) {
		for (i = 0; i < 5; i++)
			wokoo_sha.output_data[i] = ioread32(_iobase + SHA_H0_REG + i * 4);
	} else {
		for (i = 0; i < 8; i++)
			wokoo_sha.output_data[i] = ioread32(_iobase + SHA_H0_REG + i * 4);
	}
}

static int sha_open(struct inode *inode, struct file * file)  
{  
	//printk("uac_open sha...\n");
    return 0;  
}  

static ssize_t sha_read(struct file * file, char __user *buffer, size_t size , loff_t *p)
{
	//printk("uac_read sha...\n");
	
	if (wokoo_sha.data_mod == CIPHER_SHA_TYPE_DMA) 
		HAL_CIPHER_SHA_DMA();
	else
		HAL_CIPHER_SHA_CPU();
	
	if (copy_to_user(buffer, &wokoo_sha.output_data, sizeof(wokoo_sha.output_data)))
		return -1;
	
	//printk("uac_read sha...end\n");
	
	return 0;
}

static ssize_t sha_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)  
{  
	//printk("uac_write sha...\n");
	
	if (copy_from_user(&wokoo_sha, buf, sizeof(struct wokoo_sha_t)))
		return -1;

	//printk("uac_write sha...end\n");
	
	return 0;  
}

static const struct file_operations sha_fops = {  
    .owner = THIS_MODULE,  
    .open  = sha_open,  
	.read  = sha_read,
    .write = sha_write, 
}; 

static int wokoo_sha_probe(struct platform_device *pdev) 
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
	
	crypto_info->dev = &pdev->dev;
	platform_set_drvdata(pdev, crypto_info);
	
	major = register_chrdev(0, "wokoo_sha", &sha_fops);
    sha_cls = class_create(THIS_MODULE, "wokoo_sha");  
    device_create(sha_cls, NULL, MKDEV(major, 0), NULL, "wokoo_sha");
	
	return 0;
}

static int wokoo_sha_remove(struct platform_device *pdev)  
{  	
    return 0;  
}

static const struct of_device_id sha_id_table[] = {
	{ .compatible = "wokoo,wokoo-sha" },
	{}
};
MODULE_DEVICE_TABLE(of, sha_id_table);

static struct platform_driver sha_driver = {
	.probe		= wokoo_sha_probe,
	.remove		= wokoo_sha_remove,
	.driver		= {
		.name	= "wokoo-sha",
		.of_match_table	= sha_id_table,
	},
};
module_platform_driver(sha_driver);
  
MODULE_ALIAS("platform:wokoo-sha");
MODULE_DESCRIPTION("WOKOO sha driver");
MODULE_LICENSE("GPL v2");




