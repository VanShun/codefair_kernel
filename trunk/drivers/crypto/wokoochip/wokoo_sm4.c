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

static struct class *sm4_cls;  
static int major = 0; 
static void __iomem *_iobase;
static spinlock_t lock;
static int mode = 0;

struct sm4_data_t {
	unsigned int dataout[512];
	unsigned int keyin[4];
	unsigned int datain[512];
	unsigned int ivin[4];
	unsigned int groups;
};
struct sm4_data_t sm4_data;

//sm4_kernel_api
/*****************************************************************************/
void write_sm4_data(unsigned char *buffer)
{
	if (!buffer)
		return;
	
	memcpy(&sm4_data, buffer, sizeof(struct sm4_data_t));
	
	iowrite32(0x00000000, _iobase + SM4_CONTROL_REG);
	
	iowrite32(sm4_data.keyin[0], _iobase + SM4_KEYIN_REG);
	iowrite32(sm4_data.keyin[1], _iobase + SM4_KEYIN_REG);
	iowrite32(sm4_data.keyin[2], _iobase + SM4_KEYIN_REG);
	iowrite32(sm4_data.keyin[3], _iobase + SM4_KEYIN_REG);

	iowrite32((ioread32(_iobase + SM4_CONTROL_REG) | SM4_CONTROL_KEY_START), _iobase + SM4_CONTROL_REG);
	while ((ioread32(_iobase + SM4_STATE_REG) & SM4_STATE_KEY_DONE) == 0);
	iowrite32(SM4_STATE_KEY_DONE, _iobase + SM4_STATE_REG);
	
	iowrite32(sm4_data.datain[0], _iobase + SM4_DATAIN_REG);
	iowrite32(sm4_data.datain[1], _iobase + SM4_DATAIN_REG);
	iowrite32(sm4_data.datain[2], _iobase + SM4_DATAIN_REG);
	iowrite32(sm4_data.datain[3], _iobase + SM4_DATAIN_REG);
}

void set_sm4_mode(unsigned int cmd)
{
	if (cmd == SM4_DIR_DECRYPT) {
		iowrite32(ioread32(_iobase + SM4_CONTROL_REG) | SM4_CONTROL_CRYPT, _iobase + SM4_CONTROL_REG);
	} else {
		iowrite32(ioread32(_iobase + SM4_CONTROL_REG) & (~SM4_CONTROL_CRYPT), _iobase + SM4_CONTROL_REG);
	}		
	
	iowrite32(ioread32(_iobase + SM4_CONTROL_REG) | SM4_CONTROL_CRYPT_START, _iobase + SM4_CONTROL_REG);
	
	while((ioread32(_iobase + SM4_STATE_REG) & SM4_STATE_CRYPT_DONE) == 0);
	iowrite32(SM4_STATE_CRYPT_DONE, _iobase + SM4_STATE_REG);
}

void read_sm4_data(unsigned char *buffer)
{
	if (!buffer)
		return;
	
	sm4_data.dataout[0] = ioread32(_iobase + SM4_DATAOUT_REG);
	sm4_data.dataout[1] = ioread32(_iobase + SM4_DATAOUT_REG);
	sm4_data.dataout[2] = ioread32(_iobase + SM4_DATAOUT_REG);
	sm4_data.dataout[3] = ioread32(_iobase + SM4_DATAOUT_REG);
	
	memcpy(buffer, &sm4_data.dataout, sizeof(sm4_data.dataout));

}
/*****************************************************************************/

static int uac_open(struct inode *inode, struct file * file)  
{  
	//printk("uac_open sm4...\n");
    return 0;  
}  

static ssize_t uac_read(struct file * file, char __user *buffer, size_t size , loff_t *p)
{
	//printk("uac_read sm4...\n");
	
//	sm4_data.dataout[0] = ioread32(_iobase + SM4_DATAOUT_REG);
//	sm4_data.dataout[1] = ioread32(_iobase + SM4_DATAOUT_REG);
//	sm4_data.dataout[2] = ioread32(_iobase + SM4_DATAOUT_REG);
//	sm4_data.dataout[3] = ioread32(_iobase + SM4_DATAOUT_REG);
	
	if (copy_to_user(buffer, &sm4_data.dataout, sizeof(sm4_data.dataout)))
		return -1;
	
	//printk("uac_read sm4...end\n");
	
	return 0;
}

static ssize_t uac_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)  
{  
	//printk("uac_write sm4...\n");
	
	if (copy_from_user(&sm4_data, buf, sizeof(struct sm4_data_t)))
		return -1;
	
	iowrite32(0x00000000, _iobase + SM4_CONTROL_REG);
	iowrite32(ioread32(_iobase + SM4_CONTROL_REG) | SM4_CONTROL_SWAP, _iobase + SM4_CONTROL_REG);   //swap
	
	if(mode == 1)
	{
	
		iowrite32(ioread32(_iobase + SM4_CONTROL_REG) | SM4_CONTROL_CBC, _iobase + SM4_CONTROL_REG);

		
			
		iowrite32(sm4_data.ivin[0], _iobase + SM4_IVIN_REG);
		iowrite32(sm4_data.ivin[1], _iobase + SM4_IVIN_REG);
		iowrite32(sm4_data.ivin[2], _iobase + SM4_IVIN_REG);
		iowrite32(sm4_data.ivin[3], _iobase + SM4_IVIN_REG);

		
		
	}else if(mode == 0)
	    iowrite32(ioread32(_iobase + SM4_CONTROL_REG) & (~SM4_CONTROL_CBC), _iobase + SM4_CONTROL_REG);
	iowrite32(sm4_data.keyin[0], _iobase + SM4_KEYIN_REG);
	iowrite32(sm4_data.keyin[1], _iobase + SM4_KEYIN_REG);
	iowrite32(sm4_data.keyin[2], _iobase + SM4_KEYIN_REG);
	iowrite32(sm4_data.keyin[3], _iobase + SM4_KEYIN_REG);

	iowrite32((ioread32(_iobase + SM4_CONTROL_REG) | SM4_CONTROL_KEY_START), _iobase + SM4_CONTROL_REG);
	while ((ioread32(_iobase + SM4_STATE_REG) & SM4_STATE_KEY_DONE) == 0);
	iowrite32(SM4_STATE_KEY_DONE, _iobase + SM4_STATE_REG);
	

	//printk("uac_write sm4...end\n");
	
	return 0;  
}

static long uac_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	//printk("uac_ioctl sm4 ...\n");
	int i,j;
	
	switch (cmd)
	{
		case SM4_DIR_DECRYPT:
			iowrite32(ioread32(_iobase + SM4_CONTROL_REG) | SM4_CONTROL_CRYPT, _iobase + SM4_CONTROL_REG);
			break;
			
		case SM4_DIR_ENCRYPT:
			iowrite32(ioread32(_iobase + SM4_CONTROL_REG) & (~SM4_CONTROL_CRYPT), _iobase + SM4_CONTROL_REG);
			break;
		
		case GMALG_SM4_CBC:					
			mode = 1;			
			return 0;		
			break;
			
		case GMALG_SM4_ECB:
			mode = 0;
			return 0;
			break;
	}
	i =0;
	for(j=0;j<sm4_data.groups;j++)
	{
		iowrite32(sm4_data.datain[i], _iobase + SM4_DATAIN_REG);
		iowrite32(sm4_data.datain[i+1], _iobase + SM4_DATAIN_REG);
		iowrite32(sm4_data.datain[i+2], _iobase + SM4_DATAIN_REG);
		iowrite32(sm4_data.datain[i+3], _iobase + SM4_DATAIN_REG);
		
		iowrite32(ioread32(_iobase + SM4_CONTROL_REG) | SM4_CONTROL_CRYPT_START, _iobase + SM4_CONTROL_REG);
		while((ioread32(_iobase + SM4_STATE_REG) & SM4_STATE_CRYPT_DONE) == 0);
		iowrite32(SM4_STATE_CRYPT_DONE, _iobase + SM4_STATE_REG);

		
		sm4_data.dataout[i] = ioread32(_iobase + SM4_DATAOUT_REG);
		sm4_data.dataout[i+1] = ioread32(_iobase + SM4_DATAOUT_REG);
		sm4_data.dataout[i+2] = ioread32(_iobase + SM4_DATAOUT_REG);
		sm4_data.dataout[i+3] = ioread32(_iobase + SM4_DATAOUT_REG);
		i = i + 4;

	}
//	printk("uac_ioctl sm4...end\n");
	
	iowrite32(SM4_STATE_CBC_DONE, _iobase + SM4_STATE_REG);           //clear cbc flag
//	printk("SM4_DATAOUT_REG  register = %x%x%x%x\n",ioread32(_iobase + SM4_DATAOUT_REG),ioread32(_iobase + SM4_DATAOUT_REG),ioread32(_iobase + SM4_DATAOUT_REG),ioread32(_iobase + SM4_DATAOUT_REG));
	return 0;
}

static const struct file_operations sm4_fops = {  
    .owner = THIS_MODULE,  
    .open  = uac_open,  
	.read  = uac_read,
    .write = uac_write, 
	.unlocked_ioctl = uac_ioctl,
};  

static int wokoo_sm4_probe(struct platform_device *pdev)  
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
	
	crypto_info->sm4_clk = devm_clk_get(&pdev->dev, "sm4_clk");
    if (IS_ERR(crypto_info->sm4_clk)) {
        err = PTR_ERR(crypto_info->sm4_clk);
        dev_err(&pdev->dev, "failed to get crypto clk: %d\n", err);
        return err;
    }
	
	err = clk_prepare_enable(crypto_info->sm4_clk);
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

#if 0
	printk("--------------------------------------------\n");
	printk("SM4_CONTROL_REG = %x\n", ioread32(_iobase + SM4_CONTROL_REG));
	iowrite32(0x00000020, _iobase + SM4_CONTROL_REG);
	printk("SM4_CONTROL_REG = %x\n", ioread32(_iobase + SM4_CONTROL_REG));
	printk("--------------------------------------------\n");
#endif

    major = register_chrdev(0, "wokoo_sm4", &sm4_fops);
    sm4_cls = class_create(THIS_MODULE,"wokoo_sm4");  
    device_create(sm4_cls, NULL, MKDEV(major, 0), NULL, "wokoo_sm4"); 

	//printk("wokoo_uac_sm4 register--------------------------------------\n");
	
    return 0;  
}  

static int wokoo_sm4_remove(struct platform_device *pdev)  
{  
    device_destroy(sm4_cls, MKDEV(major, 0));  
    class_destroy(sm4_cls);  
    unregister_chrdev(major, "wokoo_sm4");  
	
    return 0;  
}
  
static const struct of_device_id sm4_id_table[] = {
	{ .compatible = "wokoo,wokoo-sm4" },
	{}
};
MODULE_DEVICE_TABLE(of, sm4_id_table);

static struct platform_driver sm4_driver = {
	.probe		= wokoo_sm4_probe,
	.remove		= wokoo_sm4_remove,
	.driver		= {
		.name	= "wokoo-sm4",
		.of_match_table	= sm4_id_table,
	},
};
module_platform_driver(sm4_driver); 
  
MODULE_ALIAS("platform:wokoo-sm4");
MODULE_DESCRIPTION("WOKOO sm4 driver");
MODULE_LICENSE("GPL v2"); 
