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

static struct class *sec_cls;  
static int major = 0; 
static void __iomem *_iobase;
static spinlock_t lock;



typedef struct 
{
    uint32_t    ChipID0;    /*!< Efuse Chip 0 */
    uint32_t    ChipID1;    /*!< Efuse Chip 1 */
    uint32_t    ChipID2;    /*!< Efuse Chip 2 */
    uint32_t    Key_0;      /*!< Efuse Key 0 */
    uint32_t    Key_1;      /*!< Efuse Key 1 */
    uint32_t    Key_2;      /*!< Efuse Key 2 */
    uint32_t    Key_3;      /*!< Efuse Key 3 */
    uint32_t    Sec_FF;     /*!< Efuse Security Function Flag  */
}Sec_EfuseData_TypeDef;




static void HAL_SEC_EFUSE_LOAD_ENABLE(void)                
{
	
	iowrite32(ioread32(_iobase + EFUSE_CTRL) | SEC_EFUSE_CTRL_LOAD_EN,_iobase + EFUSE_CTRL);
	//(SEC->EFUSE_CTRL |= SEC_EFUSE_CTRL_LOAD_EN)
}

static void  HAL_SEC_WAIT_UNTIL_EFUSE_LOAD_DONE(void)
{
	
	do {  
		while(ioread32(_iobase + EFUSE_CTRL) & SEC_EFUSE_CTRL_LOAD_EN) ;                                                       
		}
	while(0);

}

 static void HAL_SEC_EFUSE_UPDATE(void)
 {                  
 	//(SEC->EFUSE_CTRL |= SEC_EFUSE_CTRL_UPDATE)
 	iowrite32(ioread32(_iobase + EFUSE_CTRL) | SEC_EFUSE_CTRL_UPDATE,_iobase + EFUSE_CTRL);
 }


/**
  * @brief  update security data to register from efuse data
  */
static void HAL_SEC_EFUSE_UpdateData(void)
{
    HAL_SEC_EFUSE_LOAD_ENABLE();
    HAL_SEC_WAIT_UNTIL_EFUSE_LOAD_DONE();
    HAL_SEC_EFUSE_UPDATE();
}


/**
  * @brief  read efuse data 
  * @param  data: efuse data ,Sec_EfuseData_TypeDef type
  */
void HAL_SEC_EFUSE_ReadData(Sec_EfuseData_TypeDef *data)
{
    if(data)
    {
      HAL_SEC_EFUSE_UpdateData();
      data->ChipID0 = ioread32(_iobase + CHIP_ID_0);
      data->ChipID1 = ioread32(_iobase + CHIP_ID_1);
      data->ChipID2 = ioread32(_iobase + CHIP_ID_2);
    }
}


static int uac_open(struct inode *inode, struct file * file)  
{  
//	printk("uac_open hrng...\n");
    return 0;  
}  




static ssize_t uac_read(struct file * file, char __user *buffer, size_t size , loff_t *p)
{
	

	Sec_EfuseData_TypeDef EFUSE_DATA;


	HAL_SEC_EFUSE_ReadData(&EFUSE_DATA);

//	printk("chipid = %x %x %x \n",EFUSE_DATA.ChipID0,EFUSE_DATA.ChipID1,EFUSE_DATA.ChipID2);
	
	if (copy_to_user(buffer, &EFUSE_DATA.ChipID0, 3*sizeof(EFUSE_DATA.ChipID0)))
		return -1;
	
	
	return 0;
}

static const struct file_operations sec_fops = {  
    .owner = THIS_MODULE,  
    .open  = uac_open,  
	.read  = uac_read,
};  

static int wokoo_sec_probe(struct platform_device *pdev)  
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
//	printk("sec res = %x, _iobase = %x\n", res->start, _iobase);
	if (IS_ERR(_iobase)) {
		err = PTR_ERR(_iobase);
		return err;
	}

	crypto_info->hclk = devm_clk_get(&pdev->dev, "hclk");
    if (IS_ERR(crypto_info->hclk)) {
        err = PTR_ERR(crypto_info->hclk);
        dev_err(&pdev->dev, "failed to get sec clk: %d\n", err);
        return err;
    }
	
	err = clk_prepare_enable(crypto_info->hclk);
	if (err) {
		dev_err(&pdev->dev, "failed to prepare sec clock\n");
		return err;
	}


	crypto_info->dev = &pdev->dev;
	platform_set_drvdata(pdev, crypto_info);

    major = register_chrdev(0, "wokoo_sec", &sec_fops);
    sec_cls = class_create(THIS_MODULE,"wokoo_sec");  
    device_create(sec_cls, NULL, MKDEV(major, 0), NULL, "wokoo_sec"); 

//	printk("wokoo_uac_sec register--------------------------------------\n");
	
    return 0;  
}  

static int wokoo_sec_remove(struct platform_device *pdev)  
{  
    device_destroy(sec_cls, MKDEV(major, 0));  
    class_destroy(sec_cls);  
    unregister_chrdev(major, "wokoo_sec");  
	
    return 0;  
}
  
static const struct of_device_id sec_id_table[] = {
	{ .compatible = "wokoo,wokoo-sec" },
	{}
};
MODULE_DEVICE_TABLE(of, sec_id_table);

static struct platform_driver sec_driver = {
	.probe		= wokoo_sec_probe,
	.remove		= wokoo_sec_remove,
	.driver		= {
		.name	= "wokoo-sec",
		.of_match_table	= sec_id_table,
	},
};
module_platform_driver(sec_driver); 
  
MODULE_ALIAS("platform:wokoo-sec");
MODULE_DESCRIPTION("WOKOO sec driver");
MODULE_LICENSE("GPL v2"); 
