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

static struct class *sm2_cls;  
static int major = 0; 
static void __iomem *_iobase;
static spinlock_t lock;
static uint32_t ctrl_flag = 0;

struct sm2_data_t {
	uint32_t plain[32];
	uint32_t idinfo[4];
	uint32_t message[8];
	uint32_t prikey[8];				
	uint32_t public_keyX[8];		
	uint32_t public_keyY[8];
	uint32_t en_data[32];		
	uint32_t de_data[32];	
	uint32_t signR[8];
	uint32_t signS[8];
	int     verifyOK;
};
struct sm2_data_t sm2_data;

uint8_t res_len = 0;
uint32_t random[8];
uint32_t sm2_a[8] = {0};
uint32_t sm2_b[8] = {0};
uint32_t sm2_p[8] = {0};
uint32_t sm2_n[8] = {0};
uint32_t sm2_g_x[8] = {0};
uint32_t sm2_g_y[8] = {0};

uint32_t *ECC_BUF_X0;
uint32_t *ECC_BUF_Y0;
uint32_t *ECC_BUF_X1;
uint32_t *ECC_BUF_Y1;
uint32_t *ECC_BUF_p;
uint32_t *ECC_BUF_a;
uint32_t *ECC_BUF_b;
uint32_t *ECC_BUF_k;
#define point_add  0x24
#define point_sub  0x28

//sm2_kernel_api
/*****************************************************************************/
int GM_GenSM2keypair(uint32_t prikey[], uint32_t *Q_X, uint32_t *Q_Y);
int GM_SM2Encrypt(uint32_t *encrydata, uint32_t *plain, unsigned int plainlen, uint32_t *pub_X, uint32_t *pub_Y);
int GM_SM2Decrypt(uint32_t *DecDate, uint32_t DecDatelen, uint32_t *input, uint32_t *pri_key);
void itostr(uint32_t *indata, int dalen, uint32_t *outdata, int mode);
void strtoi(uint32_t *indata, int dalen, uint32_t *outdata);

void get_sm2_key(void)
{
	unsigned long iflags;
	
	memset(&sm2_data.prikey[0], 0x00, sizeof(sm2_data.prikey));
	memset(&sm2_data.public_keyX[0], 0x00, sizeof(sm2_data.public_keyX));
	memset(&sm2_data.public_keyY[0], 0x00, sizeof(sm2_data.public_keyY));	

	spin_lock_irqsave(&lock, iflags);
	
	GM_GenSM2keypair(&sm2_data.prikey[0], &sm2_data.public_keyX[0], &sm2_data.public_keyY[0]);
	
	spin_unlock_irqrestore(&lock, iflags);	
}

void set_sm2_mode(unsigned int cmd)
{
	ctrl_flag = 0x5A5A5A5A;
	
	if (cmd == SM2_DIR_DECRYPT) {
		ctrl_flag = 0x5A5A3C3C;
	} else if (cmd == SM2_DIR_ENCRYPT) { 
		ctrl_flag = 0xA5A5C3C3;
	}
}

//(struct file * file, char __user *buffer, size_t size , loff_t *p)
void read_sm2_data(unsigned char *buffer, unsigned int size)
{
	uint8_t i = 0;
	uint32_t en_end[130] = {0};
	uint32_t endata[300]= {0}, dedata[32] = {0};
	uint32_t count;
	
	if (!buffer)
		return;

	if (size > 256)
		return;
	
	count = (size / 32) + 24;
	if (size % 32 != 0)
		count = (size / 32 + 1) + 24;
		
	//printk("uac_read sm2...\n");
		
	if (ctrl_flag == 0xA5A5C3C3) {
		if (GM_SM2Encrypt(endata, &sm2_data.plain[0], size, &sm2_data.public_keyX[0], &sm2_data.public_keyY[0]))
			return;
	
		strtoi(endata, count, en_end);
		
		for (i = 0; i < count; i++) {
			sm2_data.en_data[i] = en_end[i];
		}
		
		if (memcpy(buffer, &sm2_data.en_data, sizeof(sm2_data.en_data)))
			 return;
	} else if (ctrl_flag == 0x5A5A3C3C) {
		itostr(sm2_data.plain, count, endata, 16);
		
		if (GM_SM2Decrypt(dedata, size, endata, &sm2_data.prikey[0]))
			return;
		
		for (i = 0; i < (count - 24); i++) {
			sm2_data.de_data[i] = dedata[i];
		}
		
		if (memcpy(buffer, &sm2_data.de_data, sizeof(sm2_data.de_data)))
			 return;
	} else if (ctrl_flag == 0x5A5A5A5A) {
		if (memcpy(buffer, &sm2_data.prikey, sizeof(sm2_data.prikey) + sizeof(sm2_data.public_keyX) + sizeof(sm2_data.public_keyY)))
			 return;
	} else {
		return;
	}	
}

void write_sm2_data(unsigned char *buffer)
{	
	memcpy(&sm2_data.plain[0], buffer, sizeof(sm2_data.plain));
}
/*****************************************************************************/

void itostr(uint32_t *indata, int dalen, uint32_t *outdata, int mode)
{
	int i, j, k, n;
	uint32_t temp, temp_in;

	n = dalen -1;
	for (i = dalen-1; i >= 0; i-- ) {
		k = n -i;
		temp_in = indata[i];
		for (j = 7 + (8 * k); j >= 8 * k; j--) {
			temp = temp_in % mode;
			if (temp <= 9) {
				outdata[j] = temp + 0x30;
			} else {
				outdata[j] = temp + 0x37;
			}
			temp_in /= mode;
		}
	}
}

void strtoi(uint32_t *indata, int dalen, uint32_t *outdata)
{
	uint32_t temp, res;
	int i, j, k, n;
	
	for (i = dalen - 1; i >= 0; i--) {
		uint32_t me = 1;
		res = 0;
		n = dalen -1;
		k =  n - i;
		
		for (j = 7 + 8 * k; j >= 8 * k; j--) {
			if (indata[j] <= '9' && indata[j] >= '0') {
				temp = indata[j] - 0x30;
			} else {
				temp = indata[j] - 0x37;
			}
			res = res + temp * me;
			me *= 16;
		}
		outdata[i] = res;
	}
}

void mmcopy(uint32_t *buf, uint32_t *desbuf, int len)
{
	int i;
	
	for (i = 0; i < len; i++)
		desbuf[i] = buf[i];
}

int SM2_Compare(uint32_t *src, uint32_t *des, uint32_t len)
{
	int i;
	
	for (i = 0; i < len; i++)
		if (src[i] != des[i])
			return 0;
		
	return 1;
}

void ECC_write_N(uint32_t *n_data, uint8_t n_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_NSRAM_RST_REG);
	
	for (i = 0; i < n_len; i++)
		iowrite32(n_data[i], _iobase + PKI_NSRAM_REG);
	
	if ((n_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_NSRAM_REG);
}

void ECC_write0_N(uint8_t n_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_NSRAM_RST_REG);
	
	for (i = 0; i < n_len; i++)
		iowrite32(0x00000000, _iobase + PKI_NSRAM_REG);

	if ((n_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_NSRAM_REG);
}

void ECC_write_A(uint32_t *a_data, uint8_t a_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_ASRAM_RST_REG);
	
	for (i = 0; i < a_len; i++)
		iowrite32(a_data[i], _iobase + PKI_ASRAM_REG);
	
	if ((a_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_ASRAM_REG);
}

void ECC_write0_A(uint8_t a_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_ASRAM_RST_REG);
	
	for (i = 0; i < a_len; i++)
		iowrite32(0x00000000, _iobase + PKI_ASRAM_REG);

	if ((a_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_ASRAM_REG);
}

void ECC_write_B(uint32_t *b_data, uint8_t b_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_BSRAM_RST_REG);
	
	for (i = 0; i < b_len; i++)
		iowrite32(b_data[i], _iobase + PKI_BSRAM_REG);
		
	if ((b_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_BSRAM_REG);
}

void ECC_write0_B(uint8_t b_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_BSRAM_RST_REG);
	
	for (i = 0; i < b_len; i++)
		iowrite32(0x00000000, _iobase + PKI_BSRAM_REG);
	
	if ((b_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_BSRAM_REG);
}

void ECC_write_R(uint32_t *r_data, uint8_t r_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_RSRAM_RST_REG);
	
	for (i = 0; i < r_len; i++)
		iowrite32(r_data[i], _iobase + PKI_RSRAM_REG);
	
	if ((r_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_RSRAM_REG);
}

void ECC_write0_R(uint8_t r_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_RSRAM_RST_REG);
	
	for (i = 0; i < r_len; i++)
		iowrite32(0x00000000, _iobase + PKI_RSRAM_REG);
	
	if ((r_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_RSRAM_REG);
}

void ECC_write_T(uint32_t *t_data, uint8_t t_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_TSRAM_RST_REG);
	
	for (i = 0; i < t_len; i++)
		iowrite32(t_data[i], _iobase + PKI_TSRAM_REG);

	if ((t_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_TSRAM_REG);
}

void ECC_write0_T(uint8_t t_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_TSRAM_RST_REG);
	
	for (i = 0; i < t_len; i++)
		iowrite32(0x00000000, _iobase + PKI_TSRAM_REG);
	
	if ((t_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_TSRAM_REG);
}

void ECC_write_X0(uint32_t *X0_data, uint8_t X0_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_X0SRAM_RST_REG);
	
	for (i = 0; i < X0_len; i++)
		iowrite32(X0_data[i], _iobase + PKI_X0SRAM_REG);

	if ((X0_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_X0SRAM_REG);
}

void ECC_write_Y0(uint32_t *Y0_data, uint8_t Y0_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_Y0SRAM_RST_REG);
	
	for (i = 0; i < Y0_len; i++)
		iowrite32(Y0_data[i], _iobase + PKI_Y0SRAM_REG);
	
	if ((Y0_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_Y0SRAM_REG);
}

void ECC_write_X1(uint32_t *X1_data, uint8_t X1_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_X1SRAM_RST_REG);
	
	for (i = 0; i < X1_len; i++)
		iowrite32(X1_data[i], _iobase + PKI_X1SRAM_REG);
	
	if ((X1_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_X1SRAM_REG);
}

void ECC_write_Y1(uint32_t *Y1_data, uint8_t Y1_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_Y1SRAM_RST_REG);
	
	for (i = 0; i < Y1_len; i++)
		iowrite32(Y1_data[i], _iobase + PKI_Y1SRAM_REG);

	if ((Y1_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_Y1SRAM_REG);
}

void ECC_write_a(uint32_t *a_data, uint8_t a_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_aSRAM_RST_REG);
	
	for (i = 0; i < a_len; i++)
		iowrite32(a_data[i], _iobase + PKI_aSRAM_REG);
	
	if ((a_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_aSRAM_REG);
}

void ECC_write_b(uint32_t *b_data, uint8_t b_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_bSRAM_RST_REG);
	
	for (i = 0; i < b_len; i++)
		iowrite32(b_data[i], _iobase + PKI_bSRAM_REG);

	if ((b_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_bSRAM_REG);
}

void ECC_write_k(uint32_t *k_data, uint8_t k_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_kSRAM_RST_REG);
	
	for (i = 0; i < k_len; i++)
		iowrite32(k_data[i], _iobase + PKI_kSRAM_REG);

	if ((k_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_kSRAM_REG);
}

void ECC_write_Z0(uint32_t *Z0_data, uint8_t Z0_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_Z0RST_REG);
	
	for (i = 0; i < Z0_len; i++)
		iowrite32(Z0_data[i], _iobase + PKI_Z0SRAM_REG);
		
	if ((Z0_len & 0x01)==0x01)
		iowrite32(0x00000000, _iobase + PKI_Z0SRAM_REG);
}

void ECC_write_Y2(uint32_t *Y2_data, uint8_t Y2_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_Y2RST_REG);
	
	for (i = 0; i < Y2_len; i++)
		iowrite32(Y2_data[i], _iobase + PKI_Y2SRAM_REG);

	if ((Y2_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_Y2SRAM_REG);
}

void ECC_write_Z1(uint32_t *Z1_data, uint8_t Z1_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_Z1RST_REG);
	
	for (i = 0; i < Z1_len; i++)
		iowrite32(Z1_data[i], _iobase + PKI_Z1SRAM_REG);

	if ((Z1_len & 0x01) == 0x01)
		iowrite32(0x00000000, _iobase + PKI_Z1SRAM_REG);
}

void ECC_read_N(uint32_t *n_data, uint8_t *n_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_RSA_REG) + 1;
	
	iowrite32(0x00000000, _iobase + PKI_NSRAM_RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		 n_data[i] = ioread32(_iobase + PKI_NSRAM_REG);

	*n_len = length;
}

void ECC_read_A(uint32_t *a_data, uint8_t *a_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_RSA_REG) + 1;

	iowrite32(0x00000000, _iobase + PKI_ASRAM_RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		 a_data[i] = ioread32(_iobase + PKI_ASRAM_REG);

	*a_len = length << 1;
}

void ECC_read_R(uint32_t *r_data, uint8_t *r_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_MOD_REG) + 1;
	
	iowrite32(0x00000000, _iobase + PKI_RSRAM_RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		 r_data[i] = ioread32(_iobase + PKI_RSRAM_REG);

	*r_len = length << 1;
}

void ECC_read_X0(uint32_t *X0_data, uint8_t *X0_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_ECCX_REG) + 1;
	
	iowrite32(0x00000000, _iobase + PKI_X0SRAM_RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		X0_data[i] = ioread32(_iobase + PKI_X0SRAM_REG);

	*X0_len = length << 1;
}

void ECC_read_Y0(uint32_t *Y0_data, uint8_t *Y0_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_ECCY_REG) + 1;
	
	iowrite32(0x00000000, _iobase + PKI_Y0SRAM_RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		 Y0_data[i] = ioread32(_iobase + PKI_Y0SRAM_REG);

	*Y0_len = length << 1;
}

void ECC_read_X1(uint32_t *X1_data, uint8_t *X1_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_ECCX1_REG) + 1;
	
	iowrite32(0x00000000, _iobase + PKI_X1SRAM_RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		 X1_data[i] = ioread32(_iobase + PKI_X1SRAM_REG);
	
	*X1_len = length << 1;
}

void ECC_read_a(uint32_t *a_data, uint8_t a_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_aSRAM_RST_REG);
	
	for (i = 0; i < a_len; i++)
		a_data[i] = ioread32(_iobase + PKI_aSRAM_REG);
}

void ECC_read_b(uint32_t *b_data, uint8_t b_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_bSRAM_RST_REG);
	
	for (i = 0; i < b_len; i++)
		b_data[i] = ioread32(_iobase + PKI_bSRAM_REG);
}

void ECC_read_k(uint32_t *k_data, uint8_t k_len)
{
	uint8_t i = 0;
	
	iowrite32(0x00000000, _iobase + PKI_kSRAM_RST_REG);
	
	for (i = 0; i < k_len; i++)
		k_data[i] = ioread32(_iobase + PKI_kSRAM_REG);
}

void ECC_read_Z0(uint32_t *Z0_data, uint8_t *Z0_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_ECCZ_REG) + 1;
	
	iowrite32(0x00000000, _iobase + PKI_Z0RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		 Z0_data[i] = ioread32(_iobase + PKI_Z0SRAM_REG);
	
	*Z0_len = length << 1;
}

void ECC_read_Y2(uint32_t *Y2_data, uint8_t *Y2_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_ECCY1_REG) + 1;
	
	iowrite32(0x00000000, _iobase + PKI_Y2RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		Y2_data[i] = ioread32(_iobase + PKI_Y2SRAM_REG);

	*Y2_len = length<<1;
}

void ECC_read_Z1(uint32_t *Z1_data, uint8_t *Z1_len)
{
	uint8_t i = 0;
	uint8_t length = ioread32(_iobase + PKI_ECCZ1_REG) + 1;
	
	iowrite32(0x00000000, _iobase + PKI_Z1RST_REG);
	
	for (i = 0; i < (length << 1); i++)
		Z1_data[i] = ioread32(_iobase + PKI_Z1SRAM_REG);
	 
	*Z1_len = length << 1;
}

void ECC_mod_mul(uint32_t *in_data, uint8_t in_length, uint32_t *e_data, uint8_t e_length, uint32_t *n_data, uint8_t n_length, uint32_t *out_data, uint8_t *out_length)
{
	ECC_write_N(n_data, n_length);

	iowrite32(0x20, _iobase + PKI_CMD_REG);

	while ((ioread32(_iobase + PKI_STATUS_REG) & 0x01) != 0x01);

	ECC_write_A(in_data,in_length);

	ECC_write_B(e_data,e_length);

	iowrite32(0x01, _iobase + PKI_CMD_REG);

	while ((ioread32(_iobase + PKI_STATUS_REG) & 0x01) != 0x01);

	ECC_read_A(out_data, out_length);
}


void ECC_PA_PS_initial(uint32_t *p_data,uint32_t *a_data,uint32_t *x0_data,uint32_t *y0_data,uint32_t *x1_data,uint32_t *y1_data)
{
 	  ECC_BUF_p = p_data;
	  ECC_BUF_a = a_data;
	  ECC_BUF_X0 = x0_data;
	  ECC_BUF_Y0 = y0_data;
	  ECC_BUF_X1 = x1_data;
	  ECC_BUF_Y1 = y1_data;
}

int ECC_PA_PS(uint32_t curve_len,uint32_t cmd,uint32_t *pax_data,uint8_t *pax_len,uint32_t *pay_data,uint8_t *pay_len)
{
	//PKI->ECC_REG_CMD = 0x3f;
	iowrite32(0x3f, _iobase + PKI_CMD_REG);
	ECC_write_N(ECC_BUF_p,curve_len);
	ECC_write_a(ECC_BUF_a,curve_len);
	ECC_write_X0(ECC_BUF_X0,curve_len);
	ECC_write_Y0(ECC_BUF_Y0,curve_len);
	ECC_write_X1(ECC_BUF_X1,curve_len);
	ECC_write_Y1(ECC_BUF_Y1,curve_len);
	iowrite32(cmd, _iobase + PKI_CMD_REG);
	while((ioread32(_iobase + PKI_STATUS_REG) & 0x01)!=0x01);
	if(ioread32(_iobase + PKI_STATUS_REG) & 0x04)
	{
		return 0;
	}
	ECC_read_X0(pax_data,pax_len);
	ECC_read_Y0(pay_data,pay_len);
	return 1;
}


void ECC_mod_inv(
	uint32_t *in_data,
	uint8_t in_length,
	uint32_t *n_data,
	uint8_t n_length,
	uint32_t *out_data,
	uint8_t *out_length
	)
{
#if 1
	//write N
	ECC_write_N(n_data,n_length);
	//launch pre calculate
	iowrite32(PKI_CMD_FUNC_PRE_CALCULATION, _iobase + PKI_CMD_REG);

	while((ioread32(_iobase + PKI_STATUS_REG) & PKI_ECC_REG_STATUS_DONE) != PKI_ECC_REG_STATUS_DONE);
	//write A
	ECC_write_A(in_data,in_length);

	//launch module inv operation
	iowrite32(PKI_CMD_FUNC_MODULAR_INVERSION, _iobase + PKI_CMD_REG);

	while((ioread32(_iobase + PKI_STATUS_REG) & PKI_ECC_REG_STATUS_DONE) != PKI_ECC_REG_STATUS_DONE);
	//read result
	ECC_read_A(out_data,out_length);
#endif

}

void ECC_mod_add(
	uint32_t *a_data,
	uint8_t a_length,
	uint32_t *b_data,
	uint8_t b_length,
	uint32_t *n_data,
	uint8_t n_length,
	uint32_t *out_data,
	uint8_t *out_length
	)
{

	//write N
	ECC_write_N(n_data,n_length);
	//launch pre calculate
	iowrite32(PKI_CMD_FUNC_PRE_CALCULATION, _iobase + PKI_CMD_REG);

	while((ioread32(_iobase + PKI_STATUS_REG) & PKI_ECC_REG_STATUS_DONE) != PKI_ECC_REG_STATUS_DONE);
	//write A
	ECC_write_A(a_data,a_length);
	//write B
	ECC_write_B(b_data,b_length);
	
	//launch module add operation
	iowrite32(PKI_CMD_FUNC_MODULAR_ADDITIOn, _iobase + PKI_CMD_REG);

	while((ioread32(_iobase + PKI_STATUS_REG) & PKI_ECC_REG_STATUS_DONE) != PKI_ECC_REG_STATUS_DONE);
	//read result
	ECC_read_R(out_data,out_length);
}
	

void ECC_mod_sub(
	uint32_t *a_data,
	uint8_t a_length,
	uint32_t *b_data,
	uint8_t b_length,
	uint32_t *n_data,
	uint8_t n_length,
	uint32_t *out_data,
	uint8_t *out_length
	)
{
	iowrite32(0x3f, _iobase + PKI_CMD_REG);
	//write N
	ECC_write_N(n_data,n_length);
	//write A
	ECC_write_A(a_data,a_length);
	//write B
	ECC_write_B(b_data,b_length);
	
	iowrite32(PKI_CMD_FUNC_MODULAR_SUBTRACTION, _iobase + PKI_CMD_REG);
	
	while((ioread32(_iobase + PKI_STATUS_REG) & 0x01)!=0x01);
	//read result
	ECC_read_R(out_data,out_length);
}




void ECC_PM_initial(uint32_t *p_data, uint32_t *a_data, uint32_t *k_data, uint32_t *x0_data, uint32_t *y0_data)
{
	ECC_BUF_p  = p_data;
	ECC_BUF_a  = a_data;
	ECC_BUF_k  = k_data;
	ECC_BUF_X0 = x0_data;
	ECC_BUF_Y0 = y0_data;
}

uint8_t ECC_PM(uint32_t curve_len, uint32_t *pmx_data, uint8_t *pmx_len, uint32_t *pmy_data, uint8_t *pmy_len)
{
	iowrite32(0x3f, _iobase + PKI_CMD_REG);

	ECC_write_X0(ECC_BUF_X0, curve_len);
	ECC_write_Y0(ECC_BUF_Y0, curve_len);

	ECC_write_a(ECC_BUF_a, curve_len);
	ECC_write_N(ECC_BUF_p, curve_len);
	ECC_write_k(ECC_BUF_k, curve_len);

	iowrite32(0x21, _iobase + PKI_CMD_REG);

	while ((ioread32(_iobase + PKI_STATUS_REG) & 0x01) != 0x01);
	
	if (ioread32(_iobase + PKI_STATUS_REG) & 0x04)
		return 0;

	ECC_read_X0(pmx_data, pmx_len);
	ECC_read_Y0(pmy_data, pmy_len);
	
	return 0;
}

void ECC_PJ_initial(uint32_t *p_data, uint32_t *a_data, uint32_t *b_data, uint32_t *x0_data, uint32_t *y0_data)
{
 	  ECC_BUF_p  = p_data;
	  ECC_BUF_a  = a_data;
	  ECC_BUF_b  = b_data;
	  ECC_BUF_X0 = x0_data;
	  ECC_BUF_Y0 = y0_data;
}

uint8_t ECC_PJ(uint32_t curve_len)
{
	iowrite32(0x3f, _iobase + PKI_CMD_REG);
	
	ECC_write_X0(ECC_BUF_X0, curve_len);
	ECC_write_Y0(ECC_BUF_Y0, curve_len);
	
	ECC_write_N(ECC_BUF_p, curve_len);
	ECC_write_a(ECC_BUF_a, curve_len);
	ECC_write_b(ECC_BUF_b, curve_len);

	iowrite32(0x2A, _iobase + PKI_CMD_REG);

	while ((ioread32(_iobase + PKI_STATUS_REG) & 0x01) != 0x01);
	
	if (ioread32(_iobase + PKI_STATUS_REG) & 0x04)
		return 1;

	return 0;
}

void Hash_cal(uint32_t *pDataIn, uint32_t DataLen, uint32_t *pDigest)
{
}


int SM2_Compare_big(uint32_t *src,uint32_t *des,uint32_t len)
{
	int ret = 0;
	int i;
	for(i= len-1;i>=0;i--){
		if(src[i] > des[i])
		{
			ret = 1;
			break;
		}
		else if (src[i] < des[i])
		{
			ret = 0;
			break;
		}
		else
		{
			//src[i] == des[i]
		}

	}


	return ret;
}


uint32_t KDFwithsm3(uint32_t *outbuff, uint32_t *in_Z, uint32_t zLen, uint32_t klen)
{
    uint32_t ct = 1;
    uint32_t number = klen / 32;

    if ((klen % 32) != 0)
		number = (klen / 32) + 1;
	
    for (ct = 1; ct <= number; ct++) {
    	SM3_Hash(in_Z, zLen, outbuff + (ct - 1) * 32);
	}
    
	return 0;
}

void GM_Sm2para_init(void)
{
#if 0
	sm2_p[7] = 0x8542D69E; 
	sm2_p[6] = 0x4C044F18;
	sm2_p[5] = 0xE8B92435;
	sm2_p[4] = 0xBF6FF7DE;
	sm2_p[3] = 0x45728391;
	sm2_p[2] = 0x5C45517D;
	sm2_p[1] = 0x722EDB8B;
	sm2_p[0] = 0x08F1DFC3;

	sm2_n[7] = 0x8542D69E;
	sm2_n[6] = 0x4C044F18;
	sm2_n[5] = 0xE8B92435;
	sm2_n[4] = 0xBF6FF7DD;
	sm2_n[3] = 0x29772063;
	sm2_n[2] = 0x0485628D;
	sm2_n[1] = 0x5AE74EE7;   
	sm2_n[0] = 0xC32E79B7;   

	sm2_a[7] = 0x787968B4;    
	sm2_a[6] = 0xFA32C3FD;
	sm2_a[5] = 0x2417842E;
	sm2_a[4] = 0x73BBFEFF;
	sm2_a[3] = 0x2F3C848B;
	sm2_a[2] = 0x6831D7E0;
	sm2_a[1] = 0xEC65228B;
	sm2_a[0] = 0x3937E498;

	sm2_b[7] = 0x63E4C6D3;
	sm2_b[6] = 0xB23B0C84;
	sm2_b[5] = 0x9CF84241;
	sm2_b[4] = 0x484BFE48;
	sm2_b[3] = 0xF61D59A5;
	sm2_b[2] = 0xB16BA06E;
	sm2_b[1] = 0x6E12D1DA;
	sm2_b[0] = 0x27C5249A;

	sm2_g_x[7] = 0x421DEBD6;
	sm2_g_x[6] = 0x1B62EAB6;
	sm2_g_x[5] = 0x746434EB;
	sm2_g_x[4] = 0xC3CC315E;
	sm2_g_x[3] = 0x32220B3B;
	sm2_g_x[2] = 0xADD50BDC;
	sm2_g_x[1] = 0x4C4E6C14;
	sm2_g_x[0] = 0x7FEDD43D;

	sm2_g_y[7] = 0x0680512B;
	sm2_g_y[6] = 0xCBB42C07;
	sm2_g_y[5] = 0xD47349D2;
	sm2_g_y[4] = 0x153B70C4;
	sm2_g_y[3] = 0xE5D7FDFC;
	sm2_g_y[2] = 0xBFA36EA1;
	sm2_g_y[1] = 0xA85841B9;
	sm2_g_y[0] = 0xE46E09A2;
#endif
	sm2_p[7] = 0xFFFFFFFE; 
	sm2_p[6] = 0xFFFFFFFF;
	sm2_p[5] = 0xFFFFFFFF;
	sm2_p[4] = 0xFFFFFFFF;
	sm2_p[3] = 0xFFFFFFFF;
	sm2_p[2] = 0x00000000;
	sm2_p[1] = 0xFFFFFFFF;
	sm2_p[0] = 0xFFFFFFFF;

	sm2_n[7] = 0xFFFFFFFE;
	sm2_n[6] = 0xFFFFFFFF;
	sm2_n[5] = 0xFFFFFFFF;
	sm2_n[4] = 0xFFFFFFFF;
	sm2_n[3] = 0x7203DF6B;
	sm2_n[2] = 0x21C6052B;
	sm2_n[1] = 0x53BBF409;   
	sm2_n[0] = 0x39D54123;   

	sm2_a[7] = 0xFFFFFFFE;    
	sm2_a[6] = 0xFFFFFFFF;
	sm2_a[5] = 0xFFFFFFFF;
	sm2_a[4] = 0xFFFFFFFF;
	sm2_a[3] = 0xFFFFFFFF;
	sm2_a[2] = 0x00000000;
	sm2_a[1] = 0xFFFFFFFF;
	sm2_a[0] = 0xFFFFFFFC;

	sm2_b[7] = 0x28E9FA9E;
	sm2_b[6] = 0x9D9F5E34;
	sm2_b[5] = 0x4D5A9E4B;
	sm2_b[4] = 0xCF6509A7;
	sm2_b[3] = 0xF39789F5;
	sm2_b[2] = 0x15AB8F92;
	sm2_b[1] = 0xDDBCBD41;
	sm2_b[0] = 0x4D940E93;

	sm2_g_x[7] = 0x32C4AE2C;
	sm2_g_x[6] = 0x1F198119;
	sm2_g_x[5] = 0x5F990446;
	sm2_g_x[4] = 0x6A39C994;
	sm2_g_x[3] = 0x8FE30BBF;
	sm2_g_x[2] = 0xF2660BE1;
	sm2_g_x[1] = 0x715A4589;
	sm2_g_x[0] = 0x334C74C7;

	sm2_g_y[7] = 0xBC3736A2;
	sm2_g_y[6] = 0xF4F6779C;
	sm2_g_y[5] = 0x59BDCEE3;
	sm2_g_y[4] = 0x6B692153;
	sm2_g_y[3] = 0xD0A9877C;
	sm2_g_y[2] = 0xC62A4740;
	sm2_g_y[1] = 0x02DF32E5;
	sm2_g_y[0] = 0x2139F0A0;
}

int Ecc_sm2_genKeypair(uint32_t pri_key[], uint32_t *qx, uint32_t *qy, uint32_t *gx, uint32_t *gy, uint32_t *a, uint32_t *b, uint32_t n[], uint32_t *p)
{
	uint32_t len= 8;
	uint8_t x_lens, y_lens;
	uint8_t i;

   for (i = 0; i < 8; i++) 
		random[i]  = get_hrng() | (get_hrng() << 8) | (get_hrng() << 16) | (get_hrng() << 24);
	
	ECC_mod_mul(random, len, random, len, n, len, pri_key, &res_len);    
	
	ECC_PM_initial(p, a, pri_key, gx, gy);
	
	ECC_PM(len, qx, &x_lens, qy, &y_lens);
	
	return 0;
}

int GM_GenSM2keypair(uint32_t prikey[], uint32_t *Q_X, uint32_t *Q_Y)
{
	GM_Sm2para_init();
	
	Ecc_sm2_genKeypair(prikey, Q_X, Q_Y, sm2_g_x, sm2_g_y, sm2_a, sm2_b, sm2_n, sm2_p);
	
	return 0;
}

int GM_SM2Encrypt(uint32_t *encrydata, uint32_t *plain, unsigned int plainlen, uint32_t *pub_X, uint32_t *pub_Y)
{
	uint8_t len = 8;
	uint32_t random_k[8] = {0}, t[250] = {0};
	uint32_t x1[8] = {0}, y1[8] = {0}, x2[8] = {0}, y2[8] = {0};
	uint32_t c1_len = 0, c2_len = 0, c3_len = 0;
	uint32_t datax2y2[128] = {0}, datax2my2[200] = {0};
	uint32_t c1[128] = {0}, c2[8] = {0}, c3[100] = {0};
	uint32_t c2_temp[100] = {0}, c3_temp[100] = {0};
	int i, count = plainlen / 32;
	unsigned long iflags;

	spin_lock_irqsave(&lock, iflags);
	
	for (i = 0; i < 8; i++) 
		random[i]  = get_hrng() | (get_hrng() << 8) | (get_hrng() << 16) | (get_hrng() << 24);
	
	ECC_mod_mul(random, len, random, len, sm2_n, len, random_k, &res_len);

	ECC_PM_initial(sm2_p, sm2_a, random_k, sm2_g_x, sm2_g_y);
	ECC_PM(len, x1, &len, y1, &len);

	itostr(x1, 8, c1, 16);
	itostr(y1, 8, c1 + 8 * 8, 16);
	
	c1_len = 16;

	ECC_PM_initial(sm2_p, sm2_a, random_k, pub_X, pub_Y);
	ECC_PM(len, x2, &len, y2, &len);

	itostr(x2, 8, datax2y2, 16);
	itostr(y2, 8, datax2y2 + 8 * 8, 16);
	
	KDFwithsm3(t, datax2y2, 128, plainlen);
	
	if ((plainlen % 32) != 0)
		count = (plainlen / 32) + 1;
	
	for (i = 0; i < count; i++)
		c2[i] = plain[i] ^ t[i];

	c2_len = count;
	
	itostr(c2, count, c2_temp, 16);
	
	itostr(x2, 8, datax2my2, 16);
	
	itostr(plain, count, datax2my2 + 64, 16);
	
	itostr(y2, 8, datax2my2 + 8 * (count + 8), 16);
	
	SM3_Hash(datax2my2, 8 * (16 + count), c3);
	
	c3_len = 8;

	itostr(c3, 8, c3_temp, 16);

	mmcopy(c1, encrydata, 8 * c1_len);	
	mmcopy(c2_temp, encrydata + 8 * c1_len, 8 * c2_len);
	mmcopy(c3_temp, encrydata + 8 * c1_len + 8 * c2_len, 8 * c3_len);
	
	spin_unlock_irqrestore(&lock, iflags);
	
	return 0;
}

int GM_SM2Decrypt(uint32_t *DecDate, uint32_t DecDatelen, uint32_t *input, uint32_t *pri_key)
{
	int i;
	uint8_t len =8 ;
	uint32_t c2_len =0;
	uint32_t c2[64] = {0}, c3[64] = {0};
	uint32_t datax2y2[128] = {0}, datax2my2[200] = {0};
	uint32_t t[250] = {0}, M[10] = {0};
	uint32_t px[8] = {0}, py[8] = {0}, x2[8] = {0}, y2[8] = {0};
	uint32_t dgst_c3[64] = {0};
	int count = DecDatelen / 32;

	strtoi(input, len, px);
	strtoi(input + 64, len, py);
	
	ECC_PJ_initial(sm2_p, sm2_a, sm2_b, px, py);
	if (ECC_PJ(8) != 0)
		return -1;

	ECC_PM_initial(sm2_p, sm2_a, pri_key, px, py);
	ECC_PM(len, x2, &len, y2, &len);
	
	if (DecDatelen % 32 != 0)
		count = DecDatelen / 32 + 1;

	itostr(x2, 8, datax2y2, 16);
	itostr(y2, 8, datax2y2 + 8 * 8, 16);
	
	KDFwithsm3(t, datax2y2, 128, DecDatelen);

	c2_len = count;
	strtoi(input + 128, c2_len, c2);

	for (i = 0; i < c2_len; i++) {
		M[i] = c2[i] ^ t[i];
		*DecDate++ = M[i];
	}

	itostr(x2, 8, datax2my2, 16);
	
	itostr(M, count, datax2my2 + 64, 16);
	
	itostr(y2, 8, datax2my2 + 64 + count * 8, 16);
	
	SM3_Hash(datax2my2, 128 + count * 8, c3);

	strtoi(input + 128 + c2_len * 8, 8, dgst_c3);
			
	if (SM2_Compare(c3, dgst_c3, 64))
		return 0;
	
	return -1;
}


int  GM_SM2Sign(uint32_t *IDcode, uint32_t *message,uint32_t *prikey,uint32_t *outr,uint32_t  *outs)
{
	int i;

	uint32_t len= 8;
	uint8_t x_lens,y_lens;
	uint32_t x1[8],y1[8];

	uint32_t za[8] = {0};
	uint32_t e[8] = {0};
	uint32_t M1[16] = {0};

	uint32_t random_k[8] = {0};
	uint32_t random[8] = {0};

	uint32_t tempprikey[8] = {0};


	uint32_t zalen = 256; 
	uint8_t rlen=0;
	uint8_t tempdalen = 0;
	uint8_t tempslen = 0;
	uint8_t slen = 0;

	uint32_t s[10] = {0};
	uint32_t tempda[8] = {0};
	uint32_t temps[8] = {0};
	uint32_t tempmessage[4] = {0};

#if 0
	mmcopy(message,tempmessage,4);


	SM3_Hash(IDcode,zalen,za);     //za = 256 bit


	//apply Za£¬M£¬PA(public key)
	//step 1 :  M1 = za || M
	mmcopy(za,M1,8);


	for(i= 0;i<4;i++)
	{

        M1[8+i] = tempmessage[i];
	}


	//step 2 : e =H256(M1)
	SM3_Hash(M1,512,e);

#endif

	mmcopy(message,e,8);


	//setp 3 :get random k
	for(i= 0;i<8;i++)
	{
		random_k[i]  = get_hrng() | (get_hrng() << 8) | (get_hrng() << 16) | (get_hrng() << 24);
	}

	//step 4  (x1,y1) = k * G (xg,yg)
	ECC_mod_mul(random_k,len,random_k,len,sm2_n,len,random,&res_len);    //(random * random ) mod   n
	ECC_PM_initial(sm2_p,sm2_a,random,sm2_g_x,sm2_g_y);                 // G(gx1,gy1)
	ECC_PM(len,x1,&x_lens,y1,&y_lens);                                  // (x1,y1) = random * G


	//step 5  outr= (e + x1 )mod n
	ECC_mod_add(e,len,x1,len,sm2_n,len,outr,&rlen);        //********  out R******


	//step 6   s = (1 + da)^-1  (k-r * da) mod n
	mmcopy(prikey,tempprikey,8);

	// [ tempda * (1 + da)^ -1] mod n = 1
	tempprikey[0] = tempprikey[0] + 1 ;
	ECC_mod_inv(tempprikey,len,sm2_n,len,tempda,&tempdalen);    //[(1 + da)^-1  mod  sm2_n] = (1+da) ^-1


	tempprikey[0] = tempprikey[0] - 1 ;
	ECC_mod_mul(outr,len,tempprikey,len,sm2_n,len,temps,&tempslen);     //temps = r * da  mod n


	ECC_mod_sub(random,len,temps,len,sm2_n,len,s,&slen);   //   s =   k - r*da

	ECC_mod_mul(tempda,len,s,len,sm2_n,len,outs,&slen);    //  outs = (tempda * s) mod n


	if(SM2_Compare_big(outr,sm2_n,8))
	{
			//printf("SIGN R Illegal \n");
			return -1;
	}

	if(SM2_Compare_big(outs,sm2_n,8))
	{
			//printf("SIGN S Illegal\n");
			return -1;
	}

	return 0;

}




int  GM_SM2Verify(uint32_t *IDcode, uint32_t *message,uint32_t *outr,uint32_t  *outs,uint32_t *px,uint32_t *py)
{

	int i;
	uint32_t za[8] = {0};
	uint32_t e[8] = {0};
	uint32_t M1[16] = {0};

	uint32_t zalen = 256;
	uint8_t x_lens,y_lens;
	uint8_t rlen=0;

	uint8_t len = 8;

	uint32_t t[8];
	uint8_t tlen = 0;

	uint32_t x1[8] = {0};
	uint32_t y1[8] = {0};
	uint32_t x2[8] = {0};
	uint32_t y2[8] = {0};


	uint32_t R[8] = {0};
	uint32_t tempmessage[4] = {0};

	mmcopy(message,tempmessage,4);

	//step 1:  outr < sm2_n ?
	if(SM2_Compare_big(outr,sm2_n,8))
	{
		printk("Illegal parameter R \n");
		return -1;
	}

	//step 2:  out s < sm2_n ?
	if(SM2_Compare_big(outs,sm2_n,8))
	{
		printk("Illegal parameter S\n");
		return -1;
	}
#if 0
	//step 3:  m1 = za || M
	SM3_Hash(IDcode,zalen,za);     //za = 256 bit
	mmcopy(za,M1,8);
	for(i= 0;i<4;i++)
	{

        M1[8+i] = tempmessage[i];      //copy message to M1
	};

	//step 4:  e = H256(m1)
	SM3_Hash(M1,512,e);
#endif


	mmcopy(message,e,8);

	//step5 t = (r + s) mod n
	ECC_mod_add(outr,len,outs,len, sm2_n,len,t,&tlen);

	//step 6 (x1,y1) = s * G(xg,yg) + t * PA(pax,pay)
	ECC_PM_initial(sm2_p,sm2_a,outs,sm2_g_x,sm2_g_y);               //  G(gx1,gy1)
	ECC_PM(len,x1,&x_lens,y1,&y_lens);                              // (x1,y1) = outs * G

	ECC_PM_initial(sm2_p,sm2_a,t,px,py);                		   //     pubkey(pubkeyX,pubkeyY)
	ECC_PM(len,x2,&x_lens,y2,&y_lens);                             //    (x2,y2) = t * pubkey


	// add (x1,y1) & (x2,y2)
	ECC_PA_PS_initial(sm2_p,sm2_a,x1,y1,x2,y2);
	ECC_PA_PS(len,point_add,x1,&x_lens,y1,&y_lens);              //  (x1,y1) = (x1,y1) + (x2,y2)


	//step 7   R = (e + x1) mod n
	ECC_mod_add(e,len,x1,len,sm2_n,len,R,&rlen);
	//compare R == r ?
	if(SM2_Compare(R,outr,8))  
	{
		//printk("verify success \n");
		return 0;
	}
	else
	{
		//printk("verify fail \n");
		return -2;
	}


}


static int uac_open(struct inode *inode, struct file * file)  
{ 
	unsigned long iflags;
	
	//printk("uac_open sm2...\n");
	
	memset(&sm2_data.prikey[0], 0x00, sizeof(sm2_data.prikey));
	memset(&sm2_data.public_keyX[0], 0x00, sizeof(sm2_data.public_keyX));
	memset(&sm2_data.public_keyY[0], 0x00, sizeof(sm2_data.public_keyY));	

	spin_lock_irqsave(&lock, iflags);
	
	GM_GenSM2keypair(&sm2_data.prikey[0], &sm2_data.public_keyX[0], &sm2_data.public_keyY[0]);
	
	spin_unlock_irqrestore(&lock, iflags);	
	
    return 0;  
}  

static ssize_t uac_read(struct file * file, char __user *buffer, size_t size , loff_t *p)
{
	uint8_t i = 0;
	uint32_t en_end[130] = {0};
	uint32_t endata[300]= {0}, dedata[32] = {0};
	uint32_t count;
	uint32_t outr[8] = {0};
	uint32_t outs[8] = {0};
	
	if (size > 256)
		return -1;
	
	count = (size / 32) + 24;
	if (size % 32 != 0)
		count = (size / 32 + 1) + 24;
		
	//printk("uac_read sm2...\n");
		
	if (ctrl_flag == 0xA5A5C3C3) {
		if (GM_SM2Encrypt(endata, &sm2_data.plain[0], size, &sm2_data.public_keyX[0], &sm2_data.public_keyY[0]))
			return -1;
	
		strtoi(endata, count, en_end);
		
		for (i = 0; i < count; i++) {
			sm2_data.en_data[i] = en_end[i];
		}
		
		if (copy_to_user(buffer, &sm2_data.en_data, sizeof(sm2_data.en_data)))
			 return -1;
	} else if (ctrl_flag == 0x5A5A3C3C) {
		itostr(sm2_data.plain, count, endata, 16);
		
		if (GM_SM2Decrypt(dedata, size, endata, &sm2_data.prikey[0]))
			return -1;
		
		for (i = 0; i < (count - 24); i++) {
			sm2_data.de_data[i] = dedata[i];
		}
		
		if (copy_to_user(buffer, &sm2_data.de_data, sizeof(sm2_data.de_data)))
			 return -1;
	} else if (ctrl_flag == 0x5A5A5A5A) {
		if (copy_to_user(buffer, &sm2_data.prikey, sizeof(sm2_data.prikey) + sizeof(sm2_data.public_keyX) + sizeof(sm2_data.public_keyY)))
			 return -1;
	} else if (ctrl_flag == 0x5A5A2B2B){         //sign
		if(GM_SM2Sign(sm2_data.idinfo,sm2_data.message,sm2_data.prikey,outr,outs))
			return -1;
		for(i=0;i<8;i++)
		{
			sm2_data.signR[i] = outr[i];
			sm2_data.signS[i] = outs[i];
		}
		if (copy_to_user(buffer, &sm2_data.signR, sizeof(sm2_data.signR) + sizeof(sm2_data.signS)))
			 return -1;
	} else if (ctrl_flag == 0x5A5AB2B2){        //verify
		sm2_data.verifyOK = 0;
		if(GM_SM2Verify(sm2_data.idinfo,sm2_data.message,sm2_data.signR,sm2_data.signS,sm2_data.public_keyX,sm2_data.public_keyY))
			return -1;
		sm2_data.verifyOK = 1;
		if (copy_to_user(buffer, &sm2_data.verifyOK, sizeof(sm2_data.verifyOK)))
			 return -1;
	} else {
		return -1;
	}
	
	//printk("uac_read sm2...end\n");

	return 0;
}

static ssize_t uac_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)  
{  
	//printk("uac_write sm2...\n");
	if(ctrl_flag ==  0x5A5A3C3C  || ctrl_flag == 0xA5A5C3C3 )
	{
		if (copy_from_user(&sm2_data.plain[0], buf, sizeof(sm2_data.plain)))
			return -1;
	}
	else if(ctrl_flag == 0x5A5A2B2B ||  ctrl_flag == 0x5A5AB2B2 )
	{
		if (copy_from_user(&sm2_data, buf, sizeof(sm2_data)))
			return -1;
	}
		

	//printk("uac_write sm2...end\n");
	
	return 0;  
}

static long uac_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	//printk("uac_ioctl sm2 ...\n");

	if (cmd == SM2_DIR_DECRYPT) {
		ctrl_flag = 0x5A5A3C3C;
	} else if (cmd == SM2_DIR_ENCRYPT) { 
		ctrl_flag = 0xA5A5C3C3;
	}else if (cmd == SM2_DIR_SIGN){
		ctrl_flag = 0x5A5A2B2B;
	}else if (cmd == SM2_DIR_VERIFY){
		ctrl_flag = 0x5A5AB2B2;
	}else {
		ctrl_flag = 0x5A5A5A5A;
	}
	
//		return -1;

	 
//	printk("uac_ioctl sm2... = %x end\n",ctrl_flag);

	return 0;
}

static const struct file_operations sm2_fops = {  
    .owner = THIS_MODULE,  
    .open  = uac_open,  
	.read  = uac_read,
    .write = uac_write, 
	.unlocked_ioctl = uac_ioctl,
};  

static int wokoo_sm2_probe(struct platform_device *pdev)  
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
	
	crypto_info->pki_clk = devm_clk_get(&pdev->dev, "pki_clk");
    if (IS_ERR(crypto_info->pki_clk)) {
        err = PTR_ERR(crypto_info->pki_clk);
        dev_err(&pdev->dev, "failed to get crypto clk: %d\n", err);
        return err;
    }
	
	err = clk_prepare_enable(crypto_info->pki_clk);
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

    major = register_chrdev(0, "wokoo_sm2", &sm2_fops);
    sm2_cls = class_create(THIS_MODULE,"wokoo_sm2");  
    device_create(sm2_cls, NULL, MKDEV(major, 0), NULL, "wokoo_sm2"); 

	//printk("wokoo_uac_sm2 register--------------------------------------\n");
	
    return 0;  
}  

static int wokoo_sm2_remove(struct platform_device *pdev)  
{  
    device_destroy(sm2_cls, MKDEV(major, 0));  
    class_destroy(sm2_cls);  
    unregister_chrdev(major, "wokoo_sm2");  
	
    return 0;  
}
  
static const struct of_device_id sm2_id_table[] = {
	{ .compatible = "wokoo,wokoo-sm2" },
	{}
};
MODULE_DEVICE_TABLE(of, sm2_id_table);

static struct platform_driver sm2_driver = {
	.probe		= wokoo_sm2_probe,
	.remove		= wokoo_sm2_remove,
	.driver		= {
		.name	= "wokoo-sm2",
		.of_match_table	= sm2_id_table,
	},
};
module_platform_driver(sm2_driver); 
  
MODULE_ALIAS("platform:wokoo-sm2");
MODULE_DESCRIPTION("WOKOO sm2 driver");
MODULE_LICENSE("GPL v2"); 
