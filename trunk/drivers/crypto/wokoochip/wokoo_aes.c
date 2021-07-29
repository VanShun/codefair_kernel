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

#define CIPHER_AES_KEY_LEN_128BIT  0x00
#define CIPHER_AES_KEY_LEN_192BIT  0x01
#define CIPHER_AES_KEY_LEN_256BIT  0x02

#define CIPHER_AES_MODE_AES_CM    0x00
#define CIPHER_AES_MODE_AES_F8    0x01
#define CIPHER_AES_MODE_AES_STD   0x02

#define AES_ENCRYPT_MODE 	0x01

#define CIPHER_AES_TYPE_CPU    0x00
#define CIPHER_AES_TYPE_DMA    0x01

static struct class *aes_cls;  
static int major = 0; 

static void __iomem *_iobase;
static spinlock_t lock;

struct wokoo_aes_t{
	uint32_t mode;
	uint32_t encrypt;
	uint32_t type;
	uint32_t key_len;
	uint32_t blk_num;
	uint32_t key[8];
	uint32_t key_f8_mask[8];
	uint32_t input_data[4];
	uint32_t output_data[4];
};
struct wokoo_aes_t wokoo_aes;

static void HAL_CIPHER_AES_Config(uint32_t Mode, uint32_t Encrypt, uint32_t InType)
{
	printk("Mode = %x\n", Mode);
	printk("Encrypt = %x\n", Encrypt);
	printk("InType = %x\n", InType);
	
	iowrite32(1, _iobase + AES_INTE_REG);
	printk("AES_INTE_REG = %x\n", ioread32(_iobase + AES_INTE_REG));
	
	iowrite32(1, _iobase + AES_INTS_REG);
	printk("AES_INTS_REG = %x\n", ioread32(_iobase + AES_INTS_REG));

	iowrite32((ioread32(_iobase + AES_CTL_REG) & (~0x03)) + Mode, _iobase + AES_CTL_REG);
	
	if (Encrypt == AES_ENCRYPT_MODE)
		iowrite32(ioread32(_iobase + AES_CTL_REG) | 0x04, _iobase + AES_CTL_REG);
	else
		iowrite32(ioread32(_iobase + AES_CTL_REG) & (~0x04), _iobase + AES_CTL_REG);

	if (InType == CIPHER_AES_TYPE_DMA)
		iowrite32(ioread32(_iobase + AES_CTL_REG) | 0x08, _iobase + AES_CTL_REG);
	else
		iowrite32(ioread32(_iobase + AES_CTL_REG) & (~0x08), _iobase + AES_CTL_REG);
	
	printk("AES_CTL_REG = %x\n", ioread32(_iobase + AES_CTL_REG));
}

static void set_key_data(uint8_t len, uint8_t enable)
{
	uint32_t i = 0;
	
	printk("len = %x\n", len);
	printk("enable = %x\n", enable);
	for (i = 0; i < len; i++) {
		iowrite32(wokoo_aes.key[i], _iobase + AES_KEY0_REG + i * 4);
		if (enable == 0x5A) 
			iowrite32(wokoo_aes.key_f8_mask[i], _iobase + AES_KEY_MASK0_REG + i * 4);
	}
	
	printk("AES_KEY0_REG = %x\n", ioread32(_iobase + AES_KEY0_REG));
	printk("AES_KEY1_REG = %x\n", ioread32(_iobase + AES_KEY1_REG));
	printk("AES_KEY2_REG = %x\n", ioread32(_iobase + AES_KEY2_REG));
	printk("AES_KEY3_REG = %x\n", ioread32(_iobase + AES_KEY3_REG));
}

static void HAL_CIPHER_InputKey(void)
{
	uint8_t len = 8, enable = 0;
	
	iowrite32(wokoo_aes.key_len, _iobase + AES_KEY_LEN_REG);

	if (wokoo_aes.key_len == CIPHER_AES_KEY_LEN_128BIT) {
		len = 4;
	} else if (wokoo_aes.key_len == CIPHER_AES_KEY_LEN_192BIT){
		len = 6;
	}

	if ((ioread32(_iobase + AES_CTL_REG) & AES_CTL_MODE) == CIPHER_AES_MODE_AES_F8) 
		enable = 0x5A;
		
	set_key_data(len, enable);
	
	iowrite32(1, _iobase + AES_KEY_UPDATE_REG);
}

void HAL_CIPHER_Std_AES(void)
{
	int i;
	
	HAL_CIPHER_AES_Config(CIPHER_AES_MODE_AES_STD, wokoo_aes.encrypt, wokoo_aes.type);

	HAL_CIPHER_InputKey();

	if (wokoo_aes.type == CIPHER_AES_TYPE_CPU) {
		for (i = 0; i < wokoo_aes.blk_num; i++) {
			iowrite32(wokoo_aes.input_data[0], _iobase + AES_IN0_REG);
			iowrite32(wokoo_aes.input_data[1], _iobase + AES_IN1_REG);
			iowrite32(wokoo_aes.input_data[2], _iobase + AES_IN2_REG);
			iowrite32(wokoo_aes.input_data[3], _iobase + AES_IN3_REG);

			iowrite32(1, _iobase + AES_START_REG);
			while (ioread32(_iobase + AES_INTS_REG) == 0);
			iowrite32(1, _iobase + AES_INTS_REG);

			wokoo_aes.output_data[0] = ioread32(_iobase + AES_OUT0_REG);
			wokoo_aes.output_data[1] = ioread32(_iobase + AES_OUT1_REG);
			wokoo_aes.output_data[2] = ioread32(_iobase + AES_OUT2_REG);
			wokoo_aes.output_data[3] = ioread32(_iobase + AES_OUT3_REG);
		}
	} else {
		iowrite32((uint32_t)(wokoo_aes.input_data), _iobase + AES_SADDR_REG);
		iowrite32((uint32_t)(wokoo_aes.output_data), _iobase + AES_TADDR_REG);
		iowrite32(wokoo_aes.blk_num, _iobase + AES_BLK_NUM_REG);
		
		iowrite32(1, _iobase + AES_START_REG);
		while (ioread32(_iobase + AES_INTS_REG) == 0);
		iowrite32(1, _iobase + AES_INTS_REG);
	}
}

void HAL_CIPHER_CM_AES(void)
{
	HAL_CIPHER_AES_Config(CIPHER_AES_MODE_AES_CM, wokoo_aes.encrypt, CIPHER_AES_TYPE_DMA);
	
	HAL_CIPHER_InputKey();

	printk("(uint32_t)(wokoo_aes.output_data) = %x\n", (uint32_t)(wokoo_aes.output_data));
	iowrite32((uint32_t)(wokoo_aes.output_data), _iobase + AES_TADDR_REG);
	iowrite32(wokoo_aes.blk_num, _iobase + AES_BLK_NUM_REG);
	
	printk("AES_TADDR_REG = %x\n", ioread32(_iobase + AES_TADDR_REG));
	printk("AES_BLK_NUM_REG = %x\n", ioread32(_iobase + AES_BLK_NUM_REG));
	
	iowrite32(wokoo_aes.input_data[0], _iobase + AES_IN0_REG);
	iowrite32(wokoo_aes.input_data[1], _iobase + AES_IN1_REG);
	iowrite32(wokoo_aes.input_data[2], _iobase + AES_IN2_REG);
	iowrite32(wokoo_aes.input_data[3], _iobase + AES_IN3_REG);
	
	printk("AES_IN0_REG = %x\n", ioread32(_iobase + AES_IN0_REG));
	printk("AES_IN1_REG = %x\n", ioread32(_iobase + AES_IN1_REG));
	printk("AES_IN2_REG = %x\n", ioread32(_iobase + AES_IN2_REG));
	printk("AES_IN3_REG = %x\n", ioread32(_iobase + AES_IN3_REG));

	iowrite32(1, _iobase + AES_START_REG);	
	while (ioread32(_iobase + AES_INTS_REG) == 0);
	iowrite32(1, _iobase + AES_INTS_REG);
}

void HAL_CIPHER_F8_AES(void)
{
	HAL_CIPHER_AES_Config(CIPHER_AES_MODE_AES_F8, wokoo_aes.encrypt, CIPHER_AES_TYPE_DMA);
	
	HAL_CIPHER_InputKey();

	iowrite32((uint32_t)(wokoo_aes.output_data), _iobase + AES_TADDR_REG);
	iowrite32(wokoo_aes.blk_num, _iobase + AES_BLK_NUM_REG);
	
	iowrite32(wokoo_aes.input_data[0], _iobase + AES_IN0_REG);
	iowrite32(wokoo_aes.input_data[1], _iobase + AES_IN1_REG);
	iowrite32(wokoo_aes.input_data[2], _iobase + AES_IN2_REG);
	iowrite32(wokoo_aes.input_data[3], _iobase + AES_IN3_REG);

	iowrite32(1, _iobase + AES_START_REG);
	while (ioread32(_iobase + AES_INTS_REG) == 0);
	iowrite32(1, _iobase + AES_INTS_REG);
}

static int aes_open(struct inode *inode, struct file * file)  
{  
	//printk("uac_open aes...\n");
	
    return 0;  
}  

static ssize_t aes_read(struct file * file, char __user *buffer, size_t size , loff_t *p)
{
	//printk("uac_read aes...\n");
	
	if (wokoo_aes.mode == CIPHER_AES_MODE_AES_STD) {
		HAL_CIPHER_Std_AES();
	} else if (wokoo_aes.mode == CIPHER_AES_MODE_AES_CM) {
		printk("HAL_CIPHER_CM_AES--------\n");
		HAL_CIPHER_CM_AES();
	} else {
		HAL_CIPHER_F8_AES();
	}
	
	printk("wokoo_aes.output_data[0] = %x\n", wokoo_aes.output_data[0]);
	printk("wokoo_aes.output_data[1] = %x\n", wokoo_aes.output_data[1]);
	printk("wokoo_aes.output_data[2] = %x\n", wokoo_aes.output_data[2]);
	printk("wokoo_aes.output_data[3] = %x\n", wokoo_aes.output_data[3]);
	
	if (copy_to_user(buffer, &wokoo_aes.output_data, sizeof(wokoo_aes.output_data)))
		return -1;
	
	//printk("uac_read aes...end\n");
	
	return 0;
}

static ssize_t aes_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)  
{  
	//printk("uac_write aes...\n");
	
	if (copy_from_user(&wokoo_aes, buf, sizeof(struct wokoo_aes_t)))
		return -1;
	
	printk("wokoo_aes.key[0] = %x\n", wokoo_aes.key[0]);
	printk("wokoo_aes.key[1] = %x\n", wokoo_aes.key[1]);
	printk("wokoo_aes.key[2] = %x\n", wokoo_aes.key[2]);
	printk("wokoo_aes.key[3] = %x\n", wokoo_aes.key[3]);
	printk("wokoo_aes.input_data[0] = %x\n", wokoo_aes.input_data[0]);
	printk("wokoo_aes.input_data[1] = %x\n", wokoo_aes.input_data[1]);
	printk("wokoo_aes.input_data[2] = %x\n", wokoo_aes.input_data[2]);
	printk("wokoo_aes.input_data[3] = %x\n", wokoo_aes.input_data[3]);
	
	//printk("uac_write aes...end\n");
	
	return 0;  
}

static const struct file_operations aes_fops = {  
    .owner = THIS_MODULE,  
    .open  = aes_open,  
	.read  = aes_read,
    .write = aes_write, 
}; 

static int wokoo_aes_probe(struct platform_device *pdev) 
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
	
	major = register_chrdev(0, "wokoo_aes", &aes_fops);
    aes_cls = class_create(THIS_MODULE, "wokoo_aes");  
    device_create(aes_cls, NULL, MKDEV(major, 0), NULL, "wokoo_aes");

	return 0;
}

static int wokoo_aes_remove(struct platform_device *pdev)  
{  	
    return 0;  
}

static const struct of_device_id aes_id_table[] = {
	{ .compatible = "wokoo,wokoo-aes" },
	{}
};
MODULE_DEVICE_TABLE(of, aes_id_table);

static struct platform_driver aes_driver = {
	.probe		= wokoo_aes_probe,
	.remove		= wokoo_aes_remove,
	.driver		= {
		.name	= "wokoo-aes",
		.of_match_table	= aes_id_table,
	},
};
module_platform_driver(aes_driver);
  
MODULE_ALIAS("platform:wokoo-aes");
MODULE_DESCRIPTION("WOKOO aes driver");
MODULE_LICENSE("GPL v2"); 


