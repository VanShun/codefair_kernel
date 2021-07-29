// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 */


#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/highmem.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/stmp_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/mtd/spi-nor.h>
#include <linux/spinlock.h>
#include "spi-wokoo-ssi.h"


#define DRIVER_NAME		"wokoo-ssi"
#define SSP_PIOW_NUM 3


struct wokoo_ssi {
	struct device			*dev;
	struct completion		transfer_complete;
	void __iomem			*base;
	struct clk				*pclk;
	struct clk				*clk;
	unsigned int			clk_rate;
	struct completion		c;
	unsigned int			sck;	/* Rate requested (vs actual) */
	struct reset_control	*rst;
	spinlock_t				lock;
};


static void wokoo_ssi_read_st(struct wokoo_ssi *ssi, u8 *rxbuf, u32 cmd);
static int wokoo_ssi_exec_mem_op(struct spi_mem *mem, const struct spi_mem_op *op);

static const struct spi_controller_mem_ops wokoo_ssi_mem_ops = {
	.exec_op = wokoo_ssi_exec_mem_op,
};

static const struct of_device_id wokoo_ssi_match[] = {
	{ .compatible = "wokoo,wokoo-ssi"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wokoo_ssi_match);

/*
 * wokoo_ssi_wait - wait completion
 *
 * @regs: abase
 */
static void wokoo_ssi_wait(void *abase)
{
	/* Is SSI0 idle */
	while ((readl_relaxed(abase + WOKOO_SSI0_STS) & WOKOO_SSI0_STS_BUSY) == WOKOO_SSI0_STS_BUSY);
}

#if 0
static void wokoo_ssi_wait_prograss(struct wokoo_ssi *ssi)
{
	uint8_t reg_val;
	wokoo_ssi_read_st(ssi, &reg_val, SPINOR_OP_RDSR);
	while ((reg_val & 0x01) == 0x01) {
		wokoo_ssi_read_st(ssi, &reg_val, SPINOR_OP_RDSR);
	}
}
#endif

/*
 * wokoo_ssi_reg_init - ssi0 init
 *
 * @regs: abase
 * @baud: baud
 * @cs:   chip select
 */
static void wokoo_ssi_reg_init(void *abase, uint32_t baud, uint32_t cs)
{
	uint32_t reg_val;

	/* Disable SSI0 */
	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	/* Disable all interrupt */
	writel_relaxed(0, abase + WOKOO_SSI0_IE);
	reg_val = readl_relaxed(abase + WOKOO_SSI0_CTRL0);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	/* SSI0 clk = SSI0_mclk / SCKDV = 3.2M, SCKDV must be even */
	writel_relaxed(baud, abase + WOKOO_SSI0_BAUD);
	/* TX FIFO */
	writel_relaxed(0, abase + WOKOO_SSI0_TXFTL);
	/* RX FIFO */
	writel_relaxed(0, abase + WOKOO_SSI0_RXFTL);
	/* Select slave device cs */
	writel_relaxed(0x1 << cs, abase + WOKOO_SSI0_SE);
	reg_val = readl_relaxed(abase + WOKOO_SSI0_CTRL0);
	reg_val &= ~WOKOO_SSI0_CTRL0_TMOD_MSK;
	reg_val |= WOKOO_SSI0_CTRL0_TMOD_ROM_RD;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	/* Enable SSI0 */
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_read_flash_id - ssi0 read flash id
 *
 * @ssi: regs
 * @rxbuf: recv buffer
 * @len: length
 *
 */
static void wokoo_ssi_read_flash_id(struct wokoo_ssi *ssi, u8 *rxbuf, u32 len)
{
	uint32_t reg_val;
	uint32_t i = 0;
	void    *abase = ssi->base;
	unsigned long flags = 0;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_ROM_RD + (0 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val &= ~WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	/* Number of read data frame, 2(N) + 1 = 3 */
	writel_relaxed(0x02, abase + WOKOO_SSI0_CTRL1);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	/* Wait for transfer */
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);

	/* Reveice FIFO size, when >= set size then trigger interrupt */
	writel_relaxed(0x2, abase + WOKOO_SSI0_RXFTL);
	/* Write read id command */
	writel_relaxed(SPINOR_OP_RDID, abase + WOKOO_SSI0_DATA);

	/* Reveice 3 Byte id value */
	while (1) {
		reg_val = readl_relaxed(abase + WOKOO_SSI0_RIS);
		if (reg_val & WOKOO_SSI0_RIS_RXFIR)	{
			rxbuf[i++] = readl_relaxed(abase + WOKOO_SSI0_DATA);
			rxbuf[i++] = readl_relaxed(abase + WOKOO_SSI0_DATA);
			rxbuf[i++] = readl_relaxed(abase + WOKOO_SSI0_DATA);
			break;
		}
	}
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

static void wokoo_ssi_read_sfdp(void *abase, u8 *rxbuf, u32 len)
{
}

/*
 * wokoo_ssi_read_st - read status
 *
 * @ssi:   ssi
 * @rxbuf: buffer
 * @cmd:   cmd
 */
static void wokoo_ssi_read_st(struct wokoo_ssi *ssi, u8 *rxbuf, u32 cmd)
{
	uint32_t reg_val;
	uint32_t read_cnt;
	unsigned long flags = 0;
	void    *abase = ssi->base;

	read_cnt = 1;
	/* Disable SSI0 */
	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_ROM_RD + (7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(read_cnt, abase + WOKOO_SSI0_CTRL1);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);
	/* Just only reveice one data */
	writel_relaxed(read_cnt - 1, abase + WOKOO_SSI0_RXFTL);

	spin_lock_irqsave(&ssi->lock, flags);
	writel_relaxed(cmd, abase + WOKOO_SSI0_DATA);

	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & (WOKOO_SSI0_RIS_TXEIR | WOKOO_SSI0_RIS_RXFIR)) 
			   != (WOKOO_SSI0_RIS_TXEIR | WOKOO_SSI0_RIS_RXFIR));
	*rxbuf = readl_relaxed(abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_write_en - write enable
 *
 * @ssi: ssi
 */
static void wokoo_ssi_write_en(struct wokoo_ssi *ssi)
{
	uint32_t reg_val;
	unsigned long flags = 0;
	void    *abase = ssi->base;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TXRX + (0 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val &= ~WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_WREN, abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);
	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_write_dis - write disable
 *
 * @ssi: ssi
 */
static void wokoo_ssi_write_dis(struct wokoo_ssi *ssi)
{
	uint32_t reg_val;
	unsigned long flags = 0;
	void    *abase = ssi->base;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TXRX + (0 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val &= ~WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_WRDI, abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);
	wokoo_ssi_wait(abase);

}

/*
 * wokoo_ssi_write_status - write status
 *
 * @ssi:   ssi
 * @txbuf: buffer
 */
static void wokoo_ssi_write_status(struct wokoo_ssi *ssi, u8 *txbuf)
{
	uint32_t reg_val;
	uint32_t write_cnt;
	void    *abase = ssi->base;
	unsigned long flags = 0;

	write_cnt = 1;
	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);
	writel_relaxed(write_cnt, abase + WOKOO_SSI0_TXFTL);
	//writel_relaxed(write_cnt, abase + WOKOO_SSI0_RXFTL);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_WRSR, abase + WOKOO_SSI0_DATA);
	writel_relaxed(*txbuf, abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_read - ssi read
 *
 * @ssi:   ssi
 * @addr:  offset address
 * @rxbuf: buffer
 * @len:   read length
 */
static void wokoo_ssi_read(struct wokoo_ssi *ssi, u32 addr, u8 *rxbuf, int len)
{
	unsigned char addr_div[4];
	uint32_t read_cnt;
	uint32_t reg_val;
	uint32_t read_addr;
	uint32_t i;
	unsigned long flags = 0;
	void *abase = ssi->base;

	read_cnt = len;
	read_addr = addr;

	while (read_cnt) {
		int one_len;

		/* Reveice FIFO threshold, max value is 64K */
		if (read_cnt > 1024)
			one_len = 1024;
		else
			one_len = read_cnt;

		addr_div[2] = (read_addr & 0x000000FF);
		addr_div[1] = (read_addr & 0x0000FF00) >> 8 ;
		addr_div[0] = (read_addr & 0x00FF0000) >> 16;
		writel_relaxed(0, abase + WOKOO_SSI0_EN);
		reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_ROM_RD + (7 << WOKOO_SSI0_CTRL0_CFS_POS);
		reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
		reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
		reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
		reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
		writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);

		writel_relaxed(one_len - 1, abase + WOKOO_SSI0_CTRL1);
		writel_relaxed(1, abase + WOKOO_SSI0_EN);
		writel_relaxed(0, abase + WOKOO_SSI0_RXFTL);

		spin_lock_irqsave(&ssi->lock, flags);
		/* Write command */
		writel_relaxed(SPINOR_OP_READ, abase + WOKOO_SSI0_DATA);
		/* Write address */
		for (i = 0; i < 3; i++)
			writel_relaxed(addr_div[i], abase + WOKOO_SSI0_DATA);

		while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & (WOKOO_SSI0_RIS_TXEIR | WOKOO_SSI0_RIS_RXFIR)) 
			   != (WOKOO_SSI0_RIS_TXEIR | WOKOO_SSI0_RIS_RXFIR));

		for (i = 0; i < one_len; i++) {
			while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_RXFIR) != WOKOO_SSI0_RIS_RXFIR);
			*rxbuf++ = readl_relaxed(abase + WOKOO_SSI0_DATA);
		}
		spin_unlock_irqrestore(&ssi->lock, flags);

		wokoo_ssi_wait(abase);
		read_cnt -= one_len;
		read_addr += one_len;
	}

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_addr4b_read - ssi 4b address read
 *
 * @ssi:   ssi
 * @addr:  offset address
 * @rxbuf: buffer
 * @len:   read length
 */
static void wokoo_ssi_addr4b_read(struct wokoo_ssi *ssi, u32 addr, u8 *rxbuf, int len)
{
	unsigned char addr_div[4];
	uint32_t read_cnt;
	uint32_t reg_val;
	uint32_t read_addr;
	uint32_t i;
	unsigned long flags = 0;
	void *abase = ssi->base;

	read_cnt = len;
	read_addr = addr;

	while (read_cnt) {
		int one_len;

		/* Reveice FIFO threshold, max value is 64K */
		if (read_cnt > 1024)
			one_len = 1024;
		else
			one_len = read_cnt;

		addr_div[3] = (read_addr & 0x000000FF);
		addr_div[2] = (read_addr & 0x0000FF00) >> 8 ;
		addr_div[1] = (read_addr & 0x00FF0000) >> 16;
		addr_div[0] = (read_addr & 0xFF000000) >> 24;
		writel_relaxed(0, abase + WOKOO_SSI0_EN);
		reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_ROM_RD + (7 << WOKOO_SSI0_CTRL0_CFS_POS);
		reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
		reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
		reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
		reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
		writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);

		writel_relaxed(one_len - 1, abase + WOKOO_SSI0_CTRL1);
		writel_relaxed(1, abase + WOKOO_SSI0_EN);
		writel_relaxed(0, abase + WOKOO_SSI0_RXFTL);

		spin_lock_irqsave(&ssi->lock, flags);
		/* Write command */
		writel_relaxed(SPINOR_OP_READ_4B, abase + WOKOO_SSI0_DATA);
		/* Write address */
		for (i = 0; i < 4; i++)
			writel_relaxed(addr_div[i], abase + WOKOO_SSI0_DATA);

		while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & (WOKOO_SSI0_RIS_TXEIR | WOKOO_SSI0_RIS_RXFIR)) 
			   != (WOKOO_SSI0_RIS_TXEIR | WOKOO_SSI0_RIS_RXFIR));

		for (i = 0; i < one_len; i++) {
			while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_RXFIR) != WOKOO_SSI0_RIS_RXFIR);
			*rxbuf++ = readl_relaxed(abase + WOKOO_SSI0_DATA);
		}
		spin_unlock_irqrestore(&ssi->lock, flags);

		wokoo_ssi_wait(abase);
		read_cnt -= one_len;
		read_addr += one_len;
	}

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_sector_erase - ssi sector erase
 *
 * @ssi:   ssi
 * @addr:  offset address
 */
static void wokoo_ssi_sector_erase(struct wokoo_ssi *ssi, u32 addr)
{
	unsigned char addr_div[4];
	unsigned long flags = 0;
	uint32_t      reg_val;
	void *abase = ssi->base;

	addr_div[2] = (addr & 0x000000FF);
	addr_div[1] = (addr & 0x0000FF00) >> 8 ;
	addr_div[0] = (addr & 0x00FF0000) >> 16;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_TXFTL);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_BE_4K, abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[0], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[1], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[2], abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_sector_erase - ssi 4b address sector erase
 *
 * @ssi:   ssi
 * @addr:  offset address
 */
static void wokoo_ssi_sector_addr4b_erase(struct wokoo_ssi *ssi, u32 addr)
{
	unsigned char addr_div[4];
	unsigned long flags = 0;
	uint32_t      reg_val;
	void *abase = ssi->base;

	addr_div[3] = (addr & 0x000000FF);
	addr_div[2] = (addr & 0x0000FF00) >> 8 ;
	addr_div[1] = (addr & 0x00FF0000) >> 16;
	addr_div[0] = (addr & 0xFF000000) >> 24;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_TXFTL);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_BE_4K_4B, abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[0], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[1], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[2], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[3], abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_block_erase - ssi block erase
 *
 * @ssi:   ssi
 * @addr:  offset address
 */
static void wokoo_ssi_block_erase(struct wokoo_ssi *ssi, u32 addr)
{
	unsigned char addr_div[4];
	unsigned long flags = 0;
	uint32_t      reg_val;
	void *abase = ssi->base;

	addr_div[2] = (addr & 0x000000FF);
	addr_div[1] = (addr & 0x0000FF00) >> 8 ;
	addr_div[0] = (addr & 0x00FF0000) >> 16;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_SE, abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[0], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[1], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[2], abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_block_addr4b_erase - ssi 4b address block erase
 *
 * @ssi:   ssi
 * @addr:  offset address
 */
static void wokoo_ssi_block_addr4b_erase(struct wokoo_ssi *ssi, u32 addr)
{
	unsigned char addr_div[4];
	unsigned long flags = 0;
	uint32_t      reg_val;
	void *abase = ssi->base;

	addr_div[3] = (addr & 0x000000FF);
	addr_div[2] = (addr & 0x0000FF00) >> 8 ;
	addr_div[1] = (addr & 0x00FF0000) >> 16;
	addr_div[0] = (addr & 0xFF000000) >> 24;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_SE_4B, abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[0], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[1], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[2], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[3], abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_block_32k_erase - ssi 32K block erase
 *
 * @ssi:   ssi
 * @addr:  offset address
 */
static void wokoo_ssi_block_32k_erase(struct wokoo_ssi *ssi, u32 addr)
{
	unsigned char addr_div[4];
	unsigned long flags = 0;
	uint32_t      reg_val;
	void *abase = ssi->base;

	addr_div[2] = (addr & 0x000000FF);
	addr_div[1] = (addr & 0x0000FF00) >> 8 ;
	addr_div[0] = (addr & 0x00FF0000) >> 16;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_BE_32K, abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[0], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[1], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[2], abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_block_addr4b_32k_erase - ssi 4b address 32K block erase
 *
 * @ssi:   ssi
 * @addr:  offset address
 */
static void wokoo_ssi_block_addr4b_32k_erase(struct wokoo_ssi *ssi, u32 addr)
{
	unsigned char addr_div[4];
	unsigned long flags = 0;
	uint32_t      reg_val;
	void *abase = ssi->base;

	addr_div[3] = (addr & 0x000000FF);
	addr_div[2] = (addr & 0x0000FF00) >> 8 ;
	addr_div[1] = (addr & 0x00FF0000) >> 16;
	addr_div[0] = (addr & 0xFF000000) >> 24;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_BE_32K_4B, abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[0], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[1], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[2], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[3], abase + WOKOO_SSI0_DATA);
	spin_unlock_irqrestore(&ssi->lock, flags);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_erase_all - ssi erase all
 *
 * @abase: abase
 */
static void wokoo_ssi_erase_all(void *abase)
{
	uint32_t      reg_val;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TXRX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val &= ~WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);

	writel_relaxed(SPINOR_OP_CHIP_ERASE, abase + WOKOO_SSI0_DATA);

	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_write_page - ssi write page
 *
 * @ssi:   ssi
 * @addr:  offset address
 * @txbuf: buffer
 * @len:   read length
 */
static void wokoo_ssi_write_page(struct wokoo_ssi *ssi, u32 addr, u8 *txbuf, int len)
{
	int cnt;
	unsigned char addr_div[4];
	uint32_t reg_val;
	unsigned long flags = 0;
	void *abase = ssi->base;

	addr_div[2] = (addr & 0x000000FF);
	addr_div[1] = (addr & 0x0000FF00) >> 8 ;
	addr_div[0] = (addr & 0x00FF0000) >> 16;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(0x7, abase + WOKOO_SSI0_TXFTL);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);
	writel_relaxed(1, abase + WOKOO_SSI0_RXFTL);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_PP, abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[0], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[1], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[2], abase + WOKOO_SSI0_DATA);

	/* transmit max 8 bytes one loop*/
	if (len > 8)
		cnt = 8;
	else
		cnt = len;

	while (len) {
		reg_val = readl_relaxed(abase + WOKOO_SSI0_RIS);
		/* wait for TX FIFO empty intr */
		if (reg_val & WOKOO_SSI0_RIS_TXEIR) {
			/* transmit once */
			int cnt2 = cnt;
			while (cnt2--)
				writel_relaxed(*txbuf++, abase + WOKOO_SSI0_DATA);
			/* left bytes for next */
			len -= cnt;
			if (len > 8)
				cnt = 8;
			else
				cnt = len;
		} else
			continue;
	}

	spin_unlock_irqrestore(&ssi->lock, flags);
	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_write_addr4b_page - ssi 4b address write page
 *
 * @ssi:   ssi
 * @addr:  offset address
 * @txbuf: buffer
 * @len:   read length
 */
static void wokoo_ssi_write_addr4b_page(struct wokoo_ssi *ssi, u32 addr, u8 *txbuf, int len)
{
	int cnt;
	unsigned char addr_div[4];
	uint32_t reg_val;
	unsigned long flags = 0;
	void *abase = ssi->base;

	addr_div[3] = (addr & 0x000000FF);
	addr_div[2] = (addr & 0x0000FF00) >> 8 ;
	addr_div[1] = (addr & 0x00FF0000) >> 16;
	addr_div[0] = (addr & 0xFF000000) >> 24;

	writel_relaxed(0, abase + WOKOO_SSI0_EN);
	reg_val = (0 << WOKOO_SSI0_CTRL0_DFS_POS) + (SSI0_CTRL0_FRF_MOTOROLA_SPI << WOKOO_SSI0_CTRL0_FRF_POS)
			+ WOKOO_SSI0_CTRL0_TMOD_TX + (0x7 << WOKOO_SSI0_CTRL0_CFS_POS);
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPH;
	reg_val &= ~WOKOO_SSI0_CTRL0_SCPOL;
	reg_val |= WOKOO_SSI0_CTRL0_SLV_OE;
	reg_val &= ~WOKOO_SSI0_CTRL0_SRL;
	writel_relaxed(reg_val, abase + WOKOO_SSI0_CTRL0);
	writel_relaxed(0x7, abase + WOKOO_SSI0_TXFTL);
	writel_relaxed(1, abase + WOKOO_SSI0_EN);
	writel_relaxed(1, abase + WOKOO_SSI0_RXFTL);

	spin_lock_irqsave(&ssi->lock, flags);
	while ((readl_relaxed(abase + WOKOO_SSI0_RIS) & WOKOO_SSI0_RIS_TXEIR) == 0);
	writel_relaxed(SPINOR_OP_PP_4B, abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[0], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[1], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[2], abase + WOKOO_SSI0_DATA);
	writel_relaxed(addr_div[3], abase + WOKOO_SSI0_DATA);

	/* transmit max 8 bytes one loop*/
	if (len > 8)
		cnt = 8;
	else
		cnt = len;

	while (len) {
		reg_val = readl_relaxed(abase + WOKOO_SSI0_RIS);
		/* wait for TX FIFO empty intr */
		if (reg_val & WOKOO_SSI0_RIS_TXEIR) {
			/* transmit once */
			int cnt2 = cnt;
			while (cnt2--)
				writel_relaxed(*txbuf++, abase + WOKOO_SSI0_DATA);
			/* left bytes for next */
			len -= cnt;
			if (len > 8)
				cnt = 8;
			else
				cnt = len;
		} else
			continue;
	}

	spin_unlock_irqrestore(&ssi->lock, flags);
	wokoo_ssi_wait(abase);
}

/*
 * wokoo_ssi_setup - ssi setup
 *
 * @spi: spi
 */
static int wokoo_ssi_setup(struct spi_device *spi)
{
	struct wokoo_ssi *ssi = spi_master_get_devdata(spi->master);
	unsigned long     rate = ssi->clk_rate;

	if (spi->master->busy) {
		dev_dbg(ssi->dev, "master busy doing other transfers\n");
		return -EBUSY;
	}

	/* SSI clock now can only run under 8M */
	if (spi->max_speed_hz > 8000000) {
		spi->max_speed_hz = 8000000;
	}

	wokoo_ssi_reg_init(ssi->base, rate / spi->max_speed_hz, spi->chip_select);
	return 0;
}

static int wokoo_ssi_exec_mem_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct wokoo_ssi *ssi = spi_controller_get_devdata(mem->spi->master);
	int err = 0;

	dev_dbg(ssi->dev, "cmd:%#x mode:%d.%d.%d.%d\n",
		op->cmd.opcode, op->cmd.buswidth, op->addr.buswidth,
		op->dummy.buswidth, op->data.buswidth);

	switch (op->cmd.opcode)	{
	case SPINOR_OP_RDID:
		wokoo_ssi_read_flash_id(ssi, (u8 *)(op->data.buf.in), op->data.nbytes);
		break;
	case SPINOR_OP_RDSFDP:
		wokoo_ssi_read_sfdp(ssi->base, (u8 *)(op->data.buf.in), op->data.nbytes);
		break;
	case SPINOR_OP_RDCR:
		wokoo_ssi_read_st(ssi, (u8 *)(op->data.buf.in), SPINOR_OP_RDCR);
		break;
	case SPINOR_OP_RDSR:
		wokoo_ssi_read_st(ssi, (u8 *)(op->data.buf.in), SPINOR_OP_RDSR);
		break;
	case SPINOR_OP_WREN:
		wokoo_ssi_write_en(ssi);
		break;
	case SPINOR_OP_WRSR:
		wokoo_ssi_write_status(ssi, (u8 *)(op->data.buf.out));
		break;
	case SPINOR_OP_READ:
		wokoo_ssi_read(ssi, op->addr.val, (u8 *)(op->data.buf.in), op->data.nbytes);
		break;
	case SPINOR_OP_READ_4B:
		wokoo_ssi_addr4b_read(ssi, op->addr.val, (u8 *)(op->data.buf.in), op->data.nbytes);
		break;
	case SPINOR_OP_PP:
		wokoo_ssi_write_page(ssi, op->addr.val, (u8 *)(op->data.buf.out), op->data.nbytes);
		break;
	case SPINOR_OP_PP_4B:
		wokoo_ssi_write_addr4b_page(ssi, op->addr.val, (u8 *)(op->data.buf.out), op->data.nbytes);
		break;
	case SPINOR_OP_BE_32K:
		wokoo_ssi_block_32k_erase(ssi, op->addr.val);
		break;
	case SPINOR_OP_BE_32K_4B:
		wokoo_ssi_block_addr4b_32k_erase(ssi, op->addr.val);
		break;
	case SPINOR_OP_BE_4K:
		wokoo_ssi_sector_erase(ssi, op->addr.val);
		break;
	case SPINOR_OP_BE_4K_4B:
		wokoo_ssi_sector_addr4b_erase(ssi, op->addr.val);
		break;
	case SPINOR_OP_SE:
		wokoo_ssi_block_erase(ssi, op->addr.val);
		break;
	case SPINOR_OP_SE_4B:
		wokoo_ssi_block_addr4b_erase(ssi, op->addr.val);
		break;
	case SPINOR_OP_CHIP_ERASE:
		wokoo_ssi_erase_all(ssi->base);
		break;
	case SPINOR_OP_WRDI:
		wokoo_ssi_write_dis(ssi);
		break;
	default:
		dev_err(ssi->dev, "hhc ssi--cmd:%#x ignored------=\n", op->cmd.opcode);
		break;
	}

	return err;
}


static irqreturn_t wokoo_ssi_irq(int irq, void *dev_id)
{
//	struct wokoo_ssp *ssp = dev_id;
//	dev_err(ssp->dev, "%s[%i] CTRL1=%08x STATUS=%08x\n", __func__, __LINE__, readl(ssp->base + HW_SSP_CTRL1(ssp)), readl(ssp->base + HW_SSP_STATUS(ssp)));
	return IRQ_HANDLED;
}


static int wokoo_ssi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct device_node *np = pdev->dev.of_node;
	struct wokoo_ssi *ssi;
	struct device *dev = &pdev->dev;
	struct clk *clk;
	int ret = 0, irq_err;
	uint32_t wrate;

	ctlr = spi_alloc_master(&pdev->dev, sizeof(*ssi));

	if (!ctlr)
		return -ENOMEM;
	ssi = spi_controller_get_devdata(ctlr);
	ssi->dev = dev;

	platform_set_drvdata(pdev, ssi);
	ssi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ssi->base)) {
		ret = PTR_ERR(ssi->base);
		goto remove_master;
	}

	ssi->rst = devm_reset_control_get(&pdev->dev, "reset");
	if (!IS_ERR(ssi->rst)) {
		reset_control_assert(ssi->rst);
		udelay(2);
		reset_control_deassert(ssi->rst);
	}

	clk = devm_clk_get(ssi->dev, "ssi1_clk");
	if (IS_ERR(clk)) {
		printk("hhc ssi--get ssi1_clk error\r\n");
		return PTR_ERR(clk);
	}
	clk_set_rate(clk, 32000000);
	wrate = clk_get_rate(clk);
	clk = devm_clk_get(ssi->dev, "ssi0_clk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	clk_set_rate(clk, 32000000);
	wrate = clk_get_rate(clk);
	ssi->clk_rate = wrate;

	clk = devm_clk_get(ssi->dev, "ssi0_gr_clk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	wrate = clk_get_rate(clk);
	clk = devm_clk_get(ssi->dev, "ssi0_pclk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(ssi->dev, "failed to enable per clk: %d\n", ret);
		return ret;
	}

	init_completion(&ssi->transfer_complete);

	ret = clk_prepare_enable(ssi->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable clk clock.\n");
		goto remove_master;
	}

	irq_err = platform_get_irq(pdev, 0);
	if (irq_err < 0) {
		ret = -ENXIO;
		goto clk_dis_pclk;
	}

	ret = devm_request_irq(&pdev->dev, irq_err, wokoo_ssi_irq, 0, pdev->name, ssi);
	if (ret != 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "request_irq failed\n");
		goto clk_dis_pclk;
	}

	spin_lock_init(&ssi->lock);

	ctlr->num_chipselect = 1;
	ctlr->mode_bits =  SPI_MODE_0;
	ctlr->mem_ops = &wokoo_ssi_mem_ops;
	ctlr->setup   = wokoo_ssi_setup;
	ctlr->dev.of_node = np;

	ret = devm_spi_register_controller(&pdev->dev, ctlr);

	if (ret) {
		dev_err(&pdev->dev, "spi_register_master failed\n");
		goto clk_dis_pclk;
	}

	return ret;

clk_dis_pclk:
	clk_disable_unprepare(ssi->clk);
remove_master:
	spi_controller_put(ctlr);
	return ret;
}

static int wokoo_ssi_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver wokoo_ssi_driver = {
	.probe = wokoo_ssi_probe,
	.remove = wokoo_ssi_remove,
	.driver = {
		.name = "wokoo-ssi",
		.of_match_table = wokoo_ssi_match,
	}
};

module_platform_driver(wokoo_ssi_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("WOKOO SSI controller driver");
MODULE_ALIAS("platform:wokoo-ssi");




