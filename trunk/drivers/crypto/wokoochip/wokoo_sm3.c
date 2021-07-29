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

#define SM3_T1  0x79CC4519								// Rounds  0-15
#define SM3_T2  0x7A879D8A								// Rounds  16-63
#define SM3_FF1(x,y,z)      (x ^ y ^ z)					// Rounds  0-15  x,y,z is word length;
#define SM3_FF2(x,y,z)      ((x&y) | (x&z) | (y&z))		// Rounds  16-63  x,y,z is word length;
#define SM3_GG2(x,y,z)      ( (x&y) | (~x & z))			// Rounds  16-63 x,y,z is word length;
#define ROTATE_LEFT(n,x)    ((( x ) << n ) | (( x ) >> (32 - n)))
#define SM3_P0(x)           (x^ ROTATE_LEFT(9,x) ^  ROTATE_LEFT(17,x))
#define SM3_P1(x)           (x^ ROTATE_LEFT(15,x) ^  ROTATE_LEFT(23,x))
#define SM3_EXPAND(data,i)  (SM3_P1(data[i-16] ^data[i-9] ^ ROTATE_LEFT(15,data[i-3])) ^ ROTATE_LEFT(7,data[i-13]) ^ data[i-6])

static struct class *sm3_cls;  
static int major = 0; 
static void __iomem *_iobase;
static spinlock_t lock;

void SM3_Transform(uint32_t *pBlock, uint32_t *pDigest);
unsigned int hash_len = 0;

struct sm3_data_t {
	unsigned int datalen;
	unsigned int datain[512];
	unsigned int dataout[8];
};
struct sm3_data_t sm3_data;

//sm3_kernel_api
/*****************************************************************************/
void write_sm3_data(unsigned char *buffer, unsigned int count)
{
	if (!buffer)
		return;
	
	memcpy(sm3_data.datain, buffer, sizeof(sm3_data.datain));
	
	SM3_Hash(sm3_data.datain, count, sm3_data.dataout);
}
//EXPORT_SYMBOL(write_sm3_data);

void read_sm3_data(unsigned char *buffer)
{
	if (!buffer)
		return;
	
	memcpy(buffer, &sm3_data.dataout, sizeof(sm3_data.dataout));
}
//EXPORT_SYMBOL(read_sm3_data);
/*****************************************************************************/

void SM3_Memcpy(uint32_t *pDst, const uint32_t *pSrc, uint32_t cnt)
{
	if ((!pDst) || (!pSrc))
		return;
	
	while (cnt--) 
		*pDst++ = *pSrc++;
}
	
void SM3_Init(uint32_t *V, uint32_t *count)
{
	count[0] = 0;
	count[1] = 0;
	V[0] = 0x7380166f;
	V[1] = 0x4914b2b9;
	V[2] = 0x172442d7;
	V[3] = 0xda8a0600;
	V[4] = 0xa96f30bc;
	V[5] = 0x163138aa;
	V[6] = 0xe38dee4d;
	V[7] = 0xb0fb0e4e;
}
	
void SM3_Load(uint32_t *datain, uint32_t inlen, uint32_t *dataout, uint32_t *pDigest, uint32_t *count)
{
	uint32_t i;
	uint32_t word_remain;
	uint32_t byte_remain;
	uint32_t bit_remain;
	uint32_t padding_word;
	uint32_t actual_word_len;

	/* Update number of bits */
	if ((count[0] += (inlen & 0xfffffe00)) < (inlen & 0xfffffe00))
		count[1]++;

	word_remain = ((inlen >> 5) & 0x0f);
	byte_remain = ((inlen >> 3) & 0x03);
	bit_remain  = (inlen & 0x07);
	
	//fill remain data
	for (i = 0; i < word_remain; i++)
		(*dataout++) = (*datain++);

	if (byte_remain || bit_remain)
		(*dataout) = (*datain++);

	padding_word = 0x80000000;
	padding_word = (padding_word >> (bit_remain + byte_remain*8));
	(*dataout++) |= padding_word;
	
	actual_word_len = word_remain + 1;
	if (actual_word_len > 14) {
		for (i = 0; i < (16 - actual_word_len); i++)
			(*dataout++) = 0x00000000;
		
		dataout -= 16;
		SM3_Transform(dataout, pDigest);
		for (i = 0; i < 14; i++)
			(*dataout++) = 0x00000000;
	} else {
		for (i = 0; i < (14 - actual_word_len); i++)
			(*dataout++) = 0x00000000;
	}
	//padding length
	if ((count[0] += (inlen & 0x1ff)) < (inlen & 0x1ff))
		count[1]++;

	(*dataout++) = count[1];
	(*dataout++) = count[0];
}

void SM3_Ex(uint32_t *msg, uint32_t *msg1)
{
	int i;

	for(i = 16; i < 68; i++)
		msg[i] = SM3_EXPAND(msg, i);
	
	for (i = 0; i < 64; i++)
		msg1[i] = msg[i] ^ msg[i+4];
}

void SM3_CF(uint32_t *V, uint32_t *msg, uint32_t *msg1)
{
	int i,j;
	uint32_t SS1, SS2, TT1, TT2, REG[8];

	SM3_Memcpy(REG, V, 8);

	for (j = 0; j < 16; j++) {
		SS1 = ROTATE_LEFT(7, ((ROTATE_LEFT(12, REG[0]) + REG[4]) + ROTATE_LEFT(j, SM3_T1)));
		SS2 = SS1 ^ ROTATE_LEFT(12, REG[0]);
		TT1 = ((((SM3_FF1(REG[0], REG[1], REG[2])) + REG[3]) + SS2) + msg1[j]);
		TT2 = ((((SM3_FF1(REG[4], REG[5], REG[6])) + REG[7]) + SS1) + msg[j]);
		REG[3] = REG[2];				  	// C --> D
		REG[2] = ROTATE_LEFT(9, REG[1]);   	// B<<<9 --> C
		REG[1] = REG[0];				  	// A --> B
		REG[0] = TT1;
		REG[7] = REG[6];				  	// G --> H
		REG[6] = ROTATE_LEFT(19, REG[5]);  	// F<<<9 --> G
		REG[5] = REG[4];				  	// E --> F
		REG[4] = SM3_P0(TT2);
	}
	
	for (j = 16; j < 64; j++) {
		SS1 = ROTATE_LEFT(7, ((ROTATE_LEFT(12, REG[0]) + REG[4]) + ROTATE_LEFT(j % 32, SM3_T2)));
		SS2 = SS1 ^ ROTATE_LEFT(12, REG[0]);
		TT1 = ((((SM3_FF2(REG[0], REG[1], REG[2])) + REG[3]) + SS2) + msg1[j]);
		TT2 = ((((SM3_GG2(REG[4], REG[5], REG[6])) + REG[7]) + SS1) + msg[j]);
		REG[3] = REG[2];	// C --> D
		REG[2] = ROTATE_LEFT(9, REG[1]);   // B<<<9 --> C
		REG[1] = REG[0];
		REG[0] = TT1;
		REG[7] = REG[6];
		REG[6] = ROTATE_LEFT(19, REG[5]);
		REG[5] = REG[4];
		REG[4] = SM3_P0(TT2);
	}

	for (i = 0; i < 8; i++)
		V[i] = REG[i] ^ V[i];
}

void SM3_CF_HW(uint32_t *V,uint32_t *msg,uint32_t *msg1)
{
	int i;
	uint32_t REG[8];

	iowrite32(0x01, _iobase + SM3_CTRL_REG);	//select sm3
	
	for (i = 0; i < 8; i++)
		iowrite32(V[i], _iobase + SM3_MIDDATA_REG);

	for (i = 0; i < 64; i++) {
		iowrite32(*(msg+i), _iobase + SM3_DATAIN_REG);
		iowrite32(*(msg1+i), _iobase + SM3_DATAIN_REG);
	}
	
	for (i = 0; i < 8; i++)
		REG[i] = ioread32(_iobase + SM3_DATAOUT_REG);

	for (i = 0; i < 8; i++)
		V[i] = REG[i] ^ V[i];

	iowrite32(0x00, _iobase + SM3_CTRL_REG); 	//deselect sm3
}

void SM3_Transform(uint32_t *pBlock, uint32_t *pDigest)
{
	uint32_t W0[68];
	uint32_t W1[64];

	SM3_Memcpy(W0, pBlock, 16);

	SM3_Ex(W0, W1);

	SM3_CF_HW(pDigest, W0, W1);
}

void SM3_Hash(uint32_t *pDataIn, uint32_t DataLen, uint32_t *pDigest)
{
	uint32_t count[2];
	uint32_t data_buf[16];
	uint32_t block_len;
	uint32_t i;
	
	unsigned long iflags;
	
	spin_lock_irqsave(&lock, iflags);
	
	block_len = DataLen >> 9;
	SM3_Init(pDigest, count);
	
	for (i = 0; i < block_len; i++) {
		SM3_Memcpy(data_buf, (pDataIn + (i << 4)), 16);
		SM3_Transform(data_buf, pDigest);
	}
	
	for (i = 0; i < 16; i++)
		data_buf[i] = 0;
	
	SM3_Load((pDataIn + (block_len << 4)), DataLen, data_buf, pDigest, count);
	
	SM3_Transform(data_buf, pDigest);
	
	spin_unlock_irqrestore(&lock, iflags);
}

static int uac_open(struct inode *inode, struct file * file)  
{  
	//printk("uac_open sm3...\n");
	
    return 0;  
} 

static ssize_t uac_read(struct file *file, char __user *buffer, size_t size, loff_t *p)
{
	//printk("uac_read sm3...\n");
	
	if (copy_to_user(buffer, &sm3_data.dataout, sizeof(sm3_data.dataout)))
		return -1;
	
	//printk("uac_read sm3...end\n");
	
	return 0;
}

static ssize_t uac_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)  
{  
	//printk("uac_write sm3...\n");
	int length =  ((count/32)+1);

	//printk("uac_write sm3... count = %d  (32bit )\n",length);

	unsigned int  *sm3_buff;
	
	sm3_buff =  kzalloc(length, GFP_KERNEL);
	
	if (copy_from_user(sm3_buff, buf, 4*length))
		return -1;
	
	SM3_Hash(sm3_buff, count, sm3_data.dataout);

	kfree(sm3_buff);
	
	//printk("uac_write sm3...end\n");
	
	return 0;  
}   	


static const struct file_operations sm3_fops = {  
    .owner = THIS_MODULE,  
    .open  = uac_open,  
	.read  = uac_read,
    .write = uac_write, 
};  

static int wokoo_sm3_probe(struct platform_device *pdev)  
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
	
	crypto_info->sm3_clk = devm_clk_get(&pdev->dev, "sm3_clk");
    if (IS_ERR(crypto_info->sm3_clk)) {
        err = PTR_ERR(crypto_info->sm3_clk);
        dev_err(&pdev->dev, "failed to get crypto clk: %d\n", err);
        return err;
    }
	
	err = clk_prepare_enable(crypto_info->sm3_clk);
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

    major = register_chrdev(0, "wokoo_sm3", &sm3_fops);
    sm3_cls = class_create(THIS_MODULE,"wokoo_sm3");  
    device_create(sm3_cls, NULL, MKDEV(major, 0), NULL, "wokoo_sm3"); 

	//printk("wokoo_uac_sm3 register--------------------------------------\n");
	
    return 0;  
}  

static int wokoo_sm3_remove(struct platform_device *pdev)  
{  
    device_destroy(sm3_cls, MKDEV(major, 0));  
    class_destroy(sm3_cls);  
    unregister_chrdev(major, "wokoo_sm3");  
	
    return 0;  
}
  
static const struct of_device_id sm3_id_table[] = {
	{ .compatible = "wokoo,wokoo-sm3" },
	{}
};
MODULE_DEVICE_TABLE(of, sm3_id_table);

static struct platform_driver sm3_driver = {
	.probe		= wokoo_sm3_probe,
	.remove		= wokoo_sm3_remove,
	.driver		= {
		.name	= "wokoo-sm3",
		.of_match_table	= sm3_id_table,
	},
};
module_platform_driver(sm3_driver); 
  
MODULE_ALIAS("platform:wokoo-sm3");
MODULE_DESCRIPTION("WOKOO sm3 driver");
MODULE_LICENSE("GPL v2"); 
