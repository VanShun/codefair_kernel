/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "virt-dma.h"




/* DMA status reg */
#define WOKOO_DMA_STATUS        0x000
#define WOKOO_DMA_PRIOR         0x004
#define WOKOO_DMA_INT_EN        0x00c
#define WOKOO_DMA_INT_STATUS    0x010
#define WOKOO_DMA_INT_MASK      0x01c
#define WOKOO_DMA_LP_EN         0x050

/* DMA Stream x Configuration Register */
#define WOKOO_DMA_CTL(x)		(0x100 + 0x40 * (x)) /* x = 0..3 */
#define WOKOO_DMA_CTL_EN		BIT(0) /* Stream Enable */
#define WOKOO_DMA_CTL_MODE      BIT(1) /* 0 - reg mode, 1 - list mode */
#define WOKOO_DMA_CTL_RESET     BIT(2) /* FIFO soft reset */

/* DMA channel x Configuration Register */
#define WOKOO_DMA_CH_SIZE(x)        (0x104 + 0x40 * (x))
#define WOKOO_DMA_CH_PARA(x)        (0x108 + 0x40 * (x))
#define WOKOO_DMA_SRC_ADDR(x)       (0x10c + 0x40 * (x))
#define WOKOO_DMA_DST_ADDR(x)       (0x110 + 0x40 * (x))
#define WOKOO_DMA_CH_INT_EN(x)      (0x114 + 0x40 * (x))
#define WOKOO_DMA_CH_INT_STA(x)     (0x118 + 0x40 * (x))
#define WOKOO_DMA_CH_INT_RAW(x)     (0x11c + 0x40 * (x))
#define WOKOO_DMA_CUR_SRC_ADDR(x)   (0x120 + 0x40 * (x))
#define WOKOO_DMA_CUR_DST_ADDR(x)   (0x124 + 0x40 * (x))
#define WOKOO_DMA_CH_SPACE(x)       (0x128 + 0x40 * (x))

#define WOKOO_DMA_CH_INT_FIFO_FULL  BIT(11) /* FIFO full */
#define WOKOO_DMA_CH_INT_FIFO_HF    BIT(9)  /* FIFO half full */
#define WOKOO_DMA_CH_INT_FIFO_NEM   BIT(7)  /* FIFO not empty */
#define WOKOO_DMA_CH_INT_FIFO_ERR   BIT(6)  /* FIFO error */
#define WOKOO_DMA_CH_INT_SRC_DEC    BIT(5)  /* Source channel decode error */
#define WOKOO_DMA_CH_INT_SRC_SLA    BIT(4)  /* Source channel transfer error */
#define WOKOO_DMA_CH_INT_DST_DEC    BIT(3)  /* Destination channel transfer error */
#define WOKOO_DMA_CH_INT_DST_SLA    BIT(2)  /* Destination channel transfer error */
#define WOKOO_DMA_CH_INT_BLK_END    BIT(1)  /* Block transfer end */
#define WOKOO_DMA_CH_INT_TRANS_END  BIT(0)  /* Transfer end */

#define WOKOO_DMA_CH_INT_MASK		(WOKOO_DMA_CH_INT_TRANS_END \
					| WOKOO_DMA_CH_INT_FIFO_ERR \
					| WOKOO_DMA_CH_INT_FIFO_FULL)

/*#define WOKOO_DMA_CH_INT_MASK		(WOKOO_DMA_CH_INT_TRANS_END \
					| WOKOO_DMA_CH_INT_BLK_END \
					| WOKOO_DMA_CH_INT_DST_SLA \
					| WOKOO_DMA_CH_INT_DST_DEC \
					| WOKOO_DMA_CH_INT_SRC_SLA \
					| WOKOO_DMA_CH_INT_SRC_DEC \
					| WOKOO_DMA_CH_INT_FIFO_ERR \
					| WOKOO_DMA_CH_INT_FIFO_NEM \
					| WOKOO_DMA_CH_INT_FIFO_FULL)*/


/* DMA direction */
#define WOKOO_DMA_DEV_TO_MEM		0x00
#define	WOKOO_DMA_MEM_TO_DEV		0x01
#define	WOKOO_DMA_MEM_TO_MEM		0x02

#define WOKOO_DMA_MAX_DATA_ITEMS	0xffff
/*
 * Valid transfer starts from @0 to @0xFFFE leading to unaligned scatter
 * gather at boundary. Thus it's safer to round down this value on FIFO
 * size (16 Bytes)
 */
#define WOKOO_DMA_ALIGNED_MAX_DATA_ITEMS	\
	ALIGN_DOWN(WOKOO_DMA_MAX_DATA_ITEMS, 0xffff)
#define WOKOO_DMA_MAX_CHANNELS		0x04
#define WOKOO_DMA_MAX_REQUEST_ID	0x04
#define WOKOO_DMA_MAX_DATA_PARAM	0x03
#define WOKOO_DMA_FIFO_SIZE		64	/* FIFO is 16 bytes */


/**
 * struct wokoo_dma_cfg - WOKOO DMA custom configuration
 * @channel_id: channel ID
 * @request_line: DMA request
 * @stream_config: 32bit mask specifying the DMA channel configuration
 * @features: 32bit mask specifying the DMA Feature list
 */
struct wokoo_dma_cfg {
	u32 channel_id;
	u32 request_line;
	u32 stream_config;
	u32 features;
};

/**
 * struct wokoo_dma_chan_reg - WOKOO DMA channel regs 
 * @dma_ctl: CHx_CTL 
 * @dma_size: CHx_SIZE 
 * @dma_para: CHx_PARA 
 * @dma_src: CHx_SRC_ADDR 
 * @dma_dst: CHx_DST_ADDR 
 * @dma_space: CHx_SPACE 
 * @dma_intr: CHx_INTR_EN 
 * @dma_cur_src: CHx_CUR_SRC_ADDR 
 * @dma_cur_dst: CHx_CUR_DST_ADDR 
 */
struct wokoo_dma_chan_reg {
	u32 dma_ctl;
	u32 dma_size;
	u32 dma_para;
	u32 dma_src;
	u32 dma_dst;
	u32 dma_space;
	u32 dma_intr;
	u32 dma_cur_src;
	u32 dma_cur_dst;
};

struct wokoo_dma_sg_req {
	u32 len;
	struct wokoo_dma_chan_reg chan_reg;
};

/**
 * struct wokoo_dma_desc - dma desc 
 * @vdesc: virtual dma desc 
 * @cyclic: cyclic 
 * @num_sgs: number regs 
 * @sg_req: regs
 */
struct wokoo_dma_desc {
	struct virt_dma_desc vdesc;
	bool cyclic;
	u32 num_sgs;
	struct wokoo_dma_sg_req sg_req[];
};

/**
 * struct wokoo_dma_chan - WOKOO DMA channel
 * @vchan: virtual channel
 * @config_init: init flag
 * @busy: DMA busy flag
 * @id: channel ID 
 * @irq: irqno 
 * @desc: DMA desc
 * @next_sg: next sg 
 * @dma_sconfig: dma slave config 
 * @chan_reg: channel regs 
 */
struct wokoo_dma_chan {
	struct virt_dma_chan vchan;
	bool config_init;
	bool busy;
	u32 id;
	u32 irq;
	struct wokoo_dma_desc *desc;
	u32 next_sg;
	struct dma_slave_config	dma_sconfig;
	struct wokoo_dma_chan_reg chan_reg;
	u32 threshold;
	u32 mem_burst;
	u32 mem_width;
};

/**
 * struct wokoo_dma_device - WOKOO DMA device
 * @ddev: dma device
 * @base: dma regs base
 * @dma_clk: clock for dmag_clk
 * @bus_clk: clock for dma_bus_clk
 * @axi_clk: clock for dma_axi_clk 
 * @rst: reset control
 * @mem2mem: flag for mem to mem 
 * @wokoo_dma_chan: dma channels
 */
struct wokoo_dma_device {
	struct dma_device ddev;
	void __iomem *base;
	struct clk *dma_clk;
	struct clk *bus_clk;
	struct clk *axi_clk;
	struct reset_control *rst;
	bool mem2mem;
	struct wokoo_dma_chan chan[WOKOO_DMA_MAX_CHANNELS];
};





static struct wokoo_dma_device *wokoo_dma_get_dev(struct wokoo_dma_chan *chan)
{
	return container_of(chan->vchan.chan.device, struct wokoo_dma_device,
				ddev);
}

static struct wokoo_dma_chan *to_wokoo_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct wokoo_dma_chan, vchan.chan);
}

static struct wokoo_dma_desc *to_wokoo_dma_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct wokoo_dma_desc, vdesc);
}

/*
 * chan2dev - get device
 */
static struct device *chan2dev(struct wokoo_dma_chan *chan)
{
	return &chan->vchan.chan.dev->device;
}

/*
 * wokoo_dma_read - read reg
 */
static u32 wokoo_dma_read(struct wokoo_dma_device *dmadev, u32 reg)
{
	return readl_relaxed(dmadev->base + reg);
}

/*
 * wokoo_dma_write - write reg
 */
static void wokoo_dma_write(struct wokoo_dma_device *dmadev, u32 reg, u32 val)
{
	writel_relaxed(val, dmadev->base + reg);
}

/*
 * wokoo_dma_slave_config - slave config
 */
static int wokoo_dma_slave_config(struct dma_chan *c,
					struct dma_slave_config *config)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);

	memcpy(&chan->dma_sconfig, config, sizeof(*config));

	chan->config_init = true;

	return 0;
}

/*
 * wokoo_dma_irq_status - read irq status
 */
static u32 wokoo_dma_irq_status(struct wokoo_dma_chan *chan)
{
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);
	u32 int_sta;

	int_sta = wokoo_dma_read(dmadev, WOKOO_DMA_CH_INT_STA(chan->id));

	return int_sta & WOKOO_DMA_CH_INT_MASK;
}

/*
 * wokoo_dma_irq_clear - clear irq status
 */
static void wokoo_dma_irq_clear(struct wokoo_dma_chan *chan, u32 flags)
{
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);

	flags &= WOKOO_DMA_CH_INT_MASK;

	wokoo_dma_write(dmadev, WOKOO_DMA_CH_INT_STA(chan->id), flags);
}

/*
 * wokoo_dma_disable_chan - disable dma channel
 */
static int wokoo_dma_disable_chan(struct wokoo_dma_chan *chan)
{
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);
	unsigned long timeout = jiffies + msecs_to_jiffies(5000);
	u32 dma_scr, id;

	id = chan->id;
	dma_scr = wokoo_dma_read(dmadev, WOKOO_DMA_CTL(id));

	if (dma_scr & WOKOO_DMA_CTL_EN) {
		dma_scr &= ~WOKOO_DMA_CTL_EN;
		wokoo_dma_write(dmadev, WOKOO_DMA_CTL(id), dma_scr);

		do {
			dma_scr = wokoo_dma_read(dmadev, WOKOO_DMA_CTL(id));
			dma_scr &= WOKOO_DMA_CTL_EN;
			if (!dma_scr)
				break;

			if (time_after_eq(jiffies, timeout)) {
				dev_err(chan2dev(chan), "%s: timeout!\n",
					__func__);
				return -EBUSY;
			}
			cond_resched();
		} while (1);
	}

	return 0;
}

/*
 * wokoo_dma_stop - stop dma channel
 */
static void wokoo_dma_stop(struct wokoo_dma_chan *chan)
{
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);
	u32 dma_int, status;
	int ret;

	/* Disable chan interrupts */
	dma_int = wokoo_dma_read(dmadev, WOKOO_DMA_CH_INT_EN(chan->id));
	dma_int &= ~WOKOO_DMA_CH_INT_MASK;
	wokoo_dma_write(dmadev, WOKOO_DMA_CH_INT_EN(chan->id), dma_int);


	/* Disable DMA */
	ret = wokoo_dma_disable_chan(chan);
	if (ret < 0)
		return;

	/* Clear interrupt status if it is there */
	status = wokoo_dma_irq_status(chan);
	if (status) {
		dev_dbg(chan2dev(chan), "%s(): clearing interrupt: 0x%08x\n",
			__func__, status);
		wokoo_dma_irq_clear(chan, status);
	}

	chan->busy = false;
}

/*
 * wokoo_dma_stop - stop all dma channel
 */
static int wokoo_dma_terminate_all(struct dma_chan *c)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vchan.lock, flags);

	if (chan->busy) {
		wokoo_dma_stop(chan);
		chan->desc = NULL;
	}

	vchan_get_all_descriptors(&chan->vchan, &head);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
	vchan_dma_desc_free_list(&chan->vchan, &head);

	return 0;
}

/*
 * wokoo_dma_synchronize - synchronize dma channel
 */
static void wokoo_dma_synchronize(struct dma_chan *c)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);

	vchan_synchronize(&chan->vchan);
}

/*
 * wokoo_dma_dump_reg - dump dma channel regs
 */
static void wokoo_dma_dump_reg(struct wokoo_dma_chan *chan)
{
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);
	u32 ctl = wokoo_dma_read(dmadev, WOKOO_DMA_CTL(chan->id));
	u32 src = wokoo_dma_read(dmadev, WOKOO_DMA_SRC_ADDR(chan->id));
	u32 dst = wokoo_dma_read(dmadev, WOKOO_DMA_DST_ADDR(chan->id));
	u32 size = wokoo_dma_read(dmadev, WOKOO_DMA_CH_SIZE(chan->id));
	u32 para = wokoo_dma_read(dmadev, WOKOO_DMA_CH_PARA(chan->id));
	u32 space = wokoo_dma_read(dmadev, WOKOO_DMA_CH_SPACE(chan->id));
	u32 intr = wokoo_dma_read(dmadev, WOKOO_DMA_CH_INT_EN(chan->id));

	dev_dbg(chan2dev(chan), "CTL:   0x%08x\n", ctl);
	dev_dbg(chan2dev(chan), "SRC:  0x%08x\n", src);
	dev_dbg(chan2dev(chan), "DST:  0x%08x\n", dst);
	dev_dbg(chan2dev(chan), "SIZE: 0x%08x\n", size);
	dev_dbg(chan2dev(chan), "PARA: 0x%08x\n", para);
	dev_dbg(chan2dev(chan), "SPACE:  0x%08x\n", space);
	dev_dbg(chan2dev(chan), "INTR:  0x%08x\n", intr);
}

static void wokoo_dma_configure_next_sg(struct wokoo_dma_chan *chan);

/*
 * wokoo_dma_start_transfer - write dma channel regs and enable dma
 */
static void wokoo_dma_start_transfer(struct wokoo_dma_chan *chan)
{
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);
	struct virt_dma_desc *vdesc;
	struct wokoo_dma_sg_req *sg_req;
	struct wokoo_dma_chan_reg *reg;
	u32 status, para;
	int ret;

	//dev_dbg(chan2dev(chan), "wokoo_dma_start_transfer start ch id = %d\n", chan->id);
	ret = wokoo_dma_disable_chan(chan);
	if (ret < 0)
		return;

	if (!chan->desc) {
		vdesc = vchan_next_desc(&chan->vchan);
		if (!vdesc)
			return;

		chan->desc = to_wokoo_dma_desc(vdesc);
		chan->next_sg = 0;
	}

	if (chan->next_sg == chan->desc->num_sgs)
		chan->next_sg = 0;

	sg_req = &chan->desc->sg_req[chan->next_sg];
	reg = &sg_req->chan_reg;

	/* Write DMA regs */
	wokoo_dma_write(dmadev, WOKOO_DMA_SRC_ADDR(chan->id), reg->dma_src);
	wokoo_dma_write(dmadev, WOKOO_DMA_DST_ADDR(chan->id), reg->dma_dst);
	wokoo_dma_write(dmadev, WOKOO_DMA_CH_SIZE(chan->id), reg->dma_size);
	wokoo_dma_write(dmadev, WOKOO_DMA_CH_INT_EN(chan->id), reg->dma_intr);

	/* Source and dest transfer type is INCR */
	para = wokoo_dma_read(dmadev, WOKOO_DMA_CH_PARA(chan->id));
	para = (para & 0xFFF0FFFF) | 0x00050000;
	wokoo_dma_write(dmadev, WOKOO_DMA_CH_PARA(chan->id), para);
	wokoo_dma_write(dmadev, WOKOO_DMA_CH_SPACE(chan->id), reg->dma_space);

	chan->next_sg++;

	/* Clear interrupt status if it is there */
	status = wokoo_dma_irq_status(chan);
	if (status)
		wokoo_dma_irq_clear(chan, status);

	if (chan->desc->cyclic)
		wokoo_dma_configure_next_sg(chan);

	wokoo_dma_dump_reg(chan);

	/* Start DMA */
	reg->dma_ctl |= WOKOO_DMA_CTL_EN;
	wokoo_dma_write(dmadev, WOKOO_DMA_CTL(chan->id), reg->dma_ctl);

	/* Now is busy */
	chan->busy = true;

	dev_dbg(chan2dev(chan), "vchan %pK: started\n", &chan->vchan);
}

static void wokoo_dma_configure_next_sg(struct wokoo_dma_chan *chan)
{
}

/*
 * wokoo_dma_handle_chan_done - handle for dma compelte
 */
static void wokoo_dma_handle_chan_done(struct wokoo_dma_chan *chan)
{
	if (chan->desc) {
		if (chan->desc->cyclic) {
			vchan_cyclic_callback(&chan->desc->vdesc);
			chan->next_sg++;
			wokoo_dma_configure_next_sg(chan);
		} else {
			chan->busy = false;
			if (chan->next_sg == chan->desc->num_sgs) {
				list_del(&chan->desc->vdesc.node);
				vchan_cookie_complete(&chan->desc->vdesc);
				chan->desc = NULL;
			}
			wokoo_dma_start_transfer(chan);
		}
	}
}

/*
 * wokoo_dma_chan_irq_handler - irq handle for dma channel
 */
static void wokoo_dma_chan_irq_handler(struct wokoo_dma_device *dmadev,
										struct wokoo_dma_chan *chan)
{
	u32 status, ctl;

	spin_lock(&chan->vchan.lock);
	status = wokoo_dma_irq_status(chan);
	ctl    = wokoo_dma_read(dmadev, WOKOO_DMA_CTL(chan->id));

	//dev_dbg(chan2dev(chan), "wokoo_dma_chan_irq_handler status = %x, ctl = %x\n", status, ctl);

	if (status & WOKOO_DMA_CH_INT_TRANS_END) {
		wokoo_dma_irq_clear(chan, WOKOO_DMA_CH_INT_TRANS_END);
		wokoo_dma_handle_chan_done(chan);
		status &= ~WOKOO_DMA_CH_INT_TRANS_END;
	}
	if (status & WOKOO_DMA_CH_INT_FIFO_FULL) {
		wokoo_dma_irq_clear(chan, WOKOO_DMA_CH_INT_FIFO_FULL);
		status &= ~WOKOO_DMA_CH_INT_FIFO_FULL;
		if (!(ctl & WOKOO_DMA_CTL_EN))
			dev_err(chan2dev(chan), "FIFO Error\n");
		else
			dev_dbg(chan2dev(chan), "FIFO over/underrun\n");
	}
	if (status) {
		wokoo_dma_irq_clear(chan, status);
		dev_err(chan2dev(chan), "DMA error: status=0x%08x\n", status);
		if (!(ctl & WOKOO_DMA_CTL_EN))
			dev_err(chan2dev(chan), "chan disabled by HW\n");
	}
	spin_unlock(&chan->vchan.lock);
}

/*
 * wokoo_dma_irq - irq handle for all channels
 */
static irqreturn_t wokoo_dma_irq(int irq, void *devid)
{
	struct wokoo_dma_device *dmadev = devid;
	u32 i, intsta;


	intsta = wokoo_dma_read(dmadev, WOKOO_DMA_INT_STATUS);
	//dev_dbg(chan2dev(chan), "wokoo_dma_irq intsta = %x\n", intsta);

	/* Read all channel irq */
	for (i = 0; i < WOKOO_DMA_MAX_CHANNELS; i++) {
		if (intsta & 0x1) {
			wokoo_dma_chan_irq_handler(dmadev, &dmadev->chan[i]);
		}

		intsta >>= 1;
	}

	return IRQ_HANDLED;
}

/*
 * wokoo_dma_issue_pending - issue pending
 */
static void wokoo_dma_issue_pending(struct dma_chan *c)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);
	unsigned long flags;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	if (vchan_issue_pending(&chan->vchan) && !chan->desc && !chan->busy) {
		dev_dbg(chan2dev(chan), "vchan %pK: issued\n", &chan->vchan);
		wokoo_dma_start_transfer(chan);

	}
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

/*
 * wokoo_dma_calc_size - calculation CHx_SIZE
 */
static u32 wokoo_dma_calc_size(u32 buf_len) 
{
	u32 i = 0, x, y;
	u32 size_reg;

	if (buf_len < 0xffff) {
		size_reg = 1 << 16 | buf_len;
	} else {
		for (i = 1; i < 16; i++) {
			x = (1 << i);
			y = buf_len >> i;
			if (y < 0xffff) {
				break;
			}
		}

		size_reg = x << 16 | y;
	}

	//printk("wokoo_dma_calc_size = %x\n", size_reg);
	return size_reg;
}

/*
 * wokoo_dma_set_xfer_param - xfer param config
 */
static int wokoo_dma_set_xfer_param(struct wokoo_dma_chan *chan,
					enum dma_transfer_direction direction, u32 buf_len, u32 memaddr)
{
	/* Set data size */
	chan->chan_reg.dma_size = wokoo_dma_calc_size(buf_len);

	switch (direction) {
	case DMA_MEM_TO_DEV:
		chan->chan_reg.dma_src = memaddr;
		chan->chan_reg.dma_dst = chan->dma_sconfig.dst_addr;
		break;

	case DMA_DEV_TO_MEM:
		chan->chan_reg.dma_src = chan->dma_sconfig.src_addr;
		chan->chan_reg.dma_dst = memaddr;
		break;

	default:
		dev_err(chan2dev(chan), "Dma direction is not supported\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * wokoo_dma_clear_reg - clear channel regs
 */
static void wokoo_dma_clear_reg(struct wokoo_dma_chan_reg *regs)
{
	memset(regs, 0, sizeof(struct wokoo_dma_chan_reg));
}

/*
 * wokoo_dma_prep_slave_sg - prepare slave SG param
 */
static struct dma_async_tx_descriptor *wokoo_dma_prep_slave_sg(
	struct dma_chan *c, struct scatterlist *sgl,
	u32 sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);
	struct wokoo_dma_desc *desc;
	struct scatterlist *sg;
	int i, ret;

	if (!chan->config_init) {
		dev_err(chan2dev(chan), "dma channel is not configured\n");
		return NULL;
	}

	if (sg_len < 1) {
		dev_err(chan2dev(chan), "Invalid segment length %d\n", sg_len);
		return NULL;
	}

	desc = kzalloc(struct_size(desc, sg_req, sg_len), GFP_NOWAIT);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		ret = wokoo_dma_set_xfer_param(chan, direction, sg_dma_len(sg), sg_dma_address(sg));
		if (ret < 0)
			goto err;

		desc->sg_req[i].len = sg_dma_len(sg);

		wokoo_dma_clear_reg(&desc->sg_req[i].chan_reg);
		desc->sg_req[i].chan_reg.dma_ctl = chan->chan_reg.dma_ctl;
		desc->sg_req[i].chan_reg.dma_src = chan->chan_reg.dma_src;
		desc->sg_req[i].chan_reg.dma_size = chan->chan_reg.dma_size;
		desc->sg_req[i].chan_reg.dma_dst = chan->chan_reg.dma_dst;
		desc->sg_req[i].chan_reg.dma_intr = chan->chan_reg.dma_intr;
	}

	desc->num_sgs = sg_len;
	desc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);

err:
	kfree(desc);
	return NULL;
}

/*
 * wokoo_dma_prep_dma_cyclic - prepare dma cyslic
 */
static struct dma_async_tx_descriptor *wokoo_dma_prep_dma_cyclic(
	struct dma_chan *c, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);
	struct wokoo_dma_desc *desc;
	u32 num_periods;
	int i, ret;

	if (!buf_len || !period_len) {
		dev_err(chan2dev(chan), "Invalid buffer/period len\n");
		return NULL;
	}

	if (!chan->config_init) {
		dev_err(chan2dev(chan), "dma channel is not configured\n");
		return NULL;
	}

	if (buf_len % period_len) {
		dev_err(chan2dev(chan), "buf_len not multiple of period_len\n");
		return NULL;
	}

	/*
	 * We allow to take more number of requests till DMA is
	 * not started. The driver will loop over all requests.
	 * Once DMA is started then new requests can be queued only after
	 * terminating the DMA.
	 */
	if (chan->busy) {
		dev_err(chan2dev(chan), "Request not allowed when dma busy\n");
		return NULL;
	}

	ret = wokoo_dma_set_xfer_param(chan, direction, period_len, buf_addr);
	if (ret < 0)
		return NULL;

	num_periods = buf_len / period_len;

	desc = kzalloc(struct_size(desc, sg_req, num_periods), GFP_NOWAIT);
	if (!desc)
		return NULL;

	for (i = 0; i < num_periods; i++) {
		desc->sg_req[i].len = period_len;

		wokoo_dma_clear_reg(&desc->sg_req[i].chan_reg);
		desc->sg_req[i].chan_reg.dma_ctl = chan->chan_reg.dma_ctl;
		desc->sg_req[i].chan_reg.dma_size = chan->chan_reg.dma_size;
		desc->sg_req[i].chan_reg.dma_src = chan->chan_reg.dma_src;
		desc->sg_req[i].chan_reg.dma_dst = chan->chan_reg.dma_dst;
		desc->sg_req[i].chan_reg.dma_intr = chan->chan_reg.dma_intr;
		buf_addr += period_len;
	}

	desc->num_sgs = num_periods;
	desc->cyclic = true;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

/*
 * wokoo_dma_prep_dma_memcpy - dma memory to memory
 *
 */
static struct dma_async_tx_descriptor *wokoo_dma_prep_dma_memcpy(
	struct dma_chan *c, dma_addr_t dest,
	dma_addr_t src, size_t len, unsigned long flags)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);
	struct wokoo_dma_desc *desc;
	size_t xfer_count, offset;
	u32 num_sgs;
	int i;

	num_sgs = DIV_ROUND_UP(len, WOKOO_DMA_ALIGNED_MAX_DATA_ITEMS);
	desc = kzalloc(struct_size(desc, sg_req, num_sgs), GFP_NOWAIT);
	if (!desc)
		return NULL;

	for (offset = 0, i = 0; offset < len; offset += xfer_count, i++) {
		xfer_count = min_t(size_t, len - offset,
					WOKOO_DMA_ALIGNED_MAX_DATA_ITEMS);

		wokoo_dma_clear_reg(&desc->sg_req[i].chan_reg);
		/* Compute best size */
		chan->chan_reg.dma_size = wokoo_dma_calc_size(xfer_count);
		desc->sg_req[i].chan_reg.dma_ctl = chan->chan_reg.dma_ctl;
		desc->sg_req[i].chan_reg.dma_size = chan->chan_reg.dma_size;
		desc->sg_req[i].chan_reg.dma_src = src + offset;
		desc->sg_req[i].chan_reg.dma_dst = dest + offset;
		desc->sg_req[i].chan_reg.dma_intr = chan->chan_reg.dma_intr;
		desc->sg_req[i].len = xfer_count;
	}

	desc->num_sgs = num_sgs;
	desc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

/*
 * wokoo_dma_get_remaining_bytes - get remaining bytes
 *
 */
static u32 wokoo_dma_get_remaining_bytes(struct wokoo_dma_chan *chan)
{
	u32 size;
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);

	size = wokoo_dma_read(dmadev, WOKOO_DMA_CH_SIZE(chan->id));

	return size;
}

/*
 * wokoo_dma_desc_residue - residue
 *
 */
static size_t wokoo_dma_desc_residue(struct wokoo_dma_chan *chan,
					struct wokoo_dma_desc *desc,
					u32 next_sg)
{
	u32 residue;

	residue = wokoo_dma_get_remaining_bytes(chan);
	return residue;
}

/*
 * wokoo_dma_tx_status - tx status
 *
 */
static enum dma_status wokoo_dma_tx_status(struct dma_chan *c,
						dma_cookie_t cookie,
						struct dma_tx_state *state)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);
	struct virt_dma_desc *vdesc;
	enum dma_status status;
	unsigned long flags;
	u32 residue = 0;

	status = dma_cookie_status(c, cookie, state);
	if (status == DMA_COMPLETE || !state)
		return status;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	vdesc = vchan_find_desc(&chan->vchan, cookie);
	if (chan->desc && cookie == chan->desc->vdesc.tx.cookie)
		residue = wokoo_dma_desc_residue(chan, chan->desc,
						 chan->next_sg);
	else if (vdesc)
		residue = wokoo_dma_desc_residue(chan,
						 to_wokoo_dma_desc(vdesc), 0);
	dma_set_residue(state, residue);

	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	return status;
}

/*
 * wokoo_dma_alloc_chan_resources - channel alloc
 *
 */
static int wokoo_dma_alloc_chan_resources(struct dma_chan *c)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);
	int ret = -1;

	chan->config_init = false;

	ret = pm_runtime_get_sync(dmadev->ddev.dev);
	if (ret < 0)
		return ret;

	ret = wokoo_dma_disable_chan(chan);
	if (ret < 0)
		pm_runtime_put(dmadev->ddev.dev);

	return ret;
}

/*
 * wokoo_dma_free_chan_resources - channel free
 *
 */
static void wokoo_dma_free_chan_resources(struct dma_chan *c)
{
	struct wokoo_dma_chan *chan = to_wokoo_dma_chan(c);
	struct wokoo_dma_device *dmadev = wokoo_dma_get_dev(chan);
	unsigned long flags;

	dev_dbg(chan2dev(chan), "Freeing channel %d\n", chan->id);

	if (chan->busy) {
		spin_lock_irqsave(&chan->vchan.lock, flags);
		wokoo_dma_stop(chan);
		chan->desc = NULL;
		spin_unlock_irqrestore(&chan->vchan.lock, flags);
	}

	pm_runtime_put(dmadev->ddev.dev);

	vchan_free_chan_resources(to_virt_chan(c));
}

static void wokoo_dma_desc_free(struct virt_dma_desc *vdesc)
{
	kfree(container_of(vdesc, struct wokoo_dma_desc, vdesc));
}

/*
 * wokoo_dma_set_config - clear regs, enable channel irq
 *
 */
static void wokoo_dma_set_config(struct wokoo_dma_chan *chan,
				 struct wokoo_dma_cfg *cfg)
{
	wokoo_dma_clear_reg(&chan->chan_reg);
	/* Enable channel irq */
	chan->chan_reg.dma_intr = WOKOO_DMA_CH_INT_MASK;
}

/*
 * wokoo_dma_of_xlate - parsing device tree
 *
 */
static struct dma_chan *wokoo_dma_of_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	struct wokoo_dma_device *dmadev = ofdma->of_dma_data;
	struct device *dev = dmadev->ddev.dev;
	struct wokoo_dma_cfg cfg;
	struct wokoo_dma_chan *chan;
	struct dma_chan *c;

	if (dma_spec->args_count < 4) {
		dev_err(dev, "Bad number of cells\n");
		return NULL;
	}

	cfg.channel_id = dma_spec->args[0];
	cfg.request_line = dma_spec->args[1];
	cfg.stream_config = dma_spec->args[2];
	cfg.features = dma_spec->args[3];

	//printk("wokoo_dma_of_xlate channel_id = %d, request_line = %d\n", cfg.channel_id, cfg.request_line);
	if (cfg.channel_id >= WOKOO_DMA_MAX_CHANNELS ||
		cfg.request_line >= WOKOO_DMA_MAX_REQUEST_ID) {
		dev_err(dev, "Bad channel and/or request id\n");
		return NULL;
	}

	chan = &dmadev->chan[cfg.channel_id];

	c = dma_get_slave_channel(&chan->vchan.chan);
	if (!c) {
		dev_err(dev, "No more channels available\n");
		return NULL;
	}

	wokoo_dma_set_config(chan, &cfg);

	return c;
}

/*
 * wokoo_dma_hw_init - enable dma irq
 *
 */
static void wokoo_dma_hw_init(struct wokoo_dma_device *dmadev)
{
	/* Enable DMA channels irq and unmask irq */
	wokoo_dma_write(dmadev, WOKOO_DMA_INT_EN, 0xf);
	wokoo_dma_write(dmadev, WOKOO_DMA_INT_MASK, 0x0);
}

static const struct of_device_id wokoo_dma_of_match[] = {
	{ .compatible = "wokoo,wokoo-dma", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, wokoo_dma_of_match);

static int wokoo_dma_probe(struct platform_device *pdev)
{
	struct wokoo_dma_chan *chan;
	struct wokoo_dma_device *dmadev;
	struct dma_device *dd;
	const struct of_device_id *match;
	struct resource *res;
	int i, ret;

	match = of_match_device(wokoo_dma_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	dmadev = devm_kzalloc(&pdev->dev, sizeof(*dmadev), GFP_KERNEL);
	if (!dmadev)
		return -ENOMEM;

	dd = &dmadev->ddev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmadev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmadev->base))
		return PTR_ERR(dmadev->base);

	dmadev->dma_clk = devm_clk_get(&pdev->dev, "dmag");
	if (IS_ERR(dmadev->dma_clk)) {
		dev_err(&pdev->dev, "Error: Missing controller dma clock\n");
		return PTR_ERR(dmadev->dma_clk);
	}

	ret = clk_prepare_enable(dmadev->dma_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "clk_prep_enable dma clock error: %d\n", ret);
		return ret;
	}

	dmadev->bus_clk = devm_clk_get(&pdev->dev, "dma_bus");
	if (IS_ERR(dmadev->bus_clk)) {
		dev_err(&pdev->dev, "Error: Missing controller dma bus clock\n");
		clk_disable_unprepare(dmadev->dma_clk);
		return PTR_ERR(dmadev->bus_clk);
	}

	ret = clk_prepare_enable(dmadev->bus_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "clk_prep_enable dma bus clock error: %d\n", ret);
		return ret;
	}

	dmadev->axi_clk = devm_clk_get(&pdev->dev, "axi_bus");
	if (IS_ERR(dmadev->axi_clk)) {
		dev_err(&pdev->dev, "Error: Missing controller dma axi clock\n");
		clk_disable_unprepare(dmadev->dma_clk);
		clk_disable_unprepare(dmadev->bus_clk);
		return PTR_ERR(dmadev->axi_clk);
	}

	ret = clk_prepare_enable(dmadev->axi_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "clk_prep_enable dma axi clock error: %d\n", ret);
		return ret;
	}

	dmadev->mem2mem = of_property_read_bool(pdev->dev.of_node,
						"wokoo,mem2mem");

	dmadev->rst = devm_reset_control_get(&pdev->dev, "reset");
	if (!IS_ERR(dmadev->rst)) {
		reset_control_assert(dmadev->rst);
		udelay(2);
		reset_control_deassert(dmadev->rst);
	}

	dma_cap_set(DMA_SLAVE, dd->cap_mask);
	dma_cap_set(DMA_PRIVATE, dd->cap_mask);
	dma_cap_set(DMA_CYCLIC, dd->cap_mask);
	dd->device_alloc_chan_resources = wokoo_dma_alloc_chan_resources;
	dd->device_free_chan_resources = wokoo_dma_free_chan_resources;
	dd->device_tx_status = wokoo_dma_tx_status;
	dd->device_issue_pending = wokoo_dma_issue_pending;
	dd->device_prep_slave_sg = wokoo_dma_prep_slave_sg;
	dd->device_prep_dma_cyclic = wokoo_dma_prep_dma_cyclic;
	dd->device_config = wokoo_dma_slave_config;
	dd->device_terminate_all = wokoo_dma_terminate_all;
	dd->device_synchronize = wokoo_dma_synchronize;
	dd->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	dd->residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dd->dev = &pdev->dev;
	INIT_LIST_HEAD(&dd->channels);

	if (dmadev->mem2mem) {
		dma_cap_set(DMA_MEMCPY, dd->cap_mask);
		dd->device_prep_dma_memcpy = wokoo_dma_prep_dma_memcpy;
		dd->directions |= BIT(DMA_MEM_TO_MEM);
	}

	for (i = 0; i < WOKOO_DMA_MAX_CHANNELS; i++) {
		chan = &dmadev->chan[i];
		chan->id = i;
		chan->vchan.desc_free = wokoo_dma_desc_free;
		vchan_init(&chan->vchan, dd);
	}

	ret = dma_async_device_register(dd);
	if (ret)
		goto clk_free;

	ret = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, ret,
					wokoo_dma_irq, 0,
					dev_name(&pdev->dev), dmadev);

	if (ret) {
		dev_err(&pdev->dev,
			"request_irq failed with err %d channel %d\n",
			ret, i);
		goto err_unregister;
	}

	ret = of_dma_controller_register(pdev->dev.of_node,
					 wokoo_dma_of_xlate, dmadev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"WOKOO DMA DMA OF registration failed %d\n", ret);
		goto err_unregister;
	}

	platform_set_drvdata(pdev, dmadev);
	wokoo_dma_hw_init(dmadev);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_put(&pdev->dev);

	dev_info(&pdev->dev, "WOKOO DMA driver registered\n");

	return 0;

err_unregister:
	dma_async_device_unregister(dd);
clk_free:
	clk_disable_unprepare(dmadev->dma_clk);
	clk_disable_unprepare(dmadev->bus_clk);
	clk_disable_unprepare(dmadev->axi_clk);

	return ret;
}

#ifdef CONFIG_PM
static int wokoo_dma_runtime_suspend(struct device *dev)
{
	struct wokoo_dma_device *dmadev = dev_get_drvdata(dev);

	clk_disable_unprepare(dmadev->dma_clk);
	clk_disable_unprepare(dmadev->bus_clk);
	clk_disable_unprepare(dmadev->axi_clk);

	return 0;
}

static int wokoo_dma_runtime_resume(struct device *dev)
{
	struct wokoo_dma_device *dmadev = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(dmadev->dma_clk);
	if (ret) {
		dev_err(dev, "failed to prepare_enable dmag clock\n");
		return ret;
	}

	ret = clk_prepare_enable(dmadev->bus_clk);
	if (ret) {
		dev_err(dev, "failed to prepare_enable dma bus clock\n");
		return ret;
	}

	ret = clk_prepare_enable(dmadev->axi_clk);
	if (ret) {
		dev_err(dev, "failed to prepare_enable dma axi clock\n");
		return ret;
	}

	return 0;
}
#endif

static const struct dev_pm_ops wokoo_dma_pm_ops = {
	SET_RUNTIME_PM_OPS(wokoo_dma_runtime_suspend,
				wokoo_dma_runtime_resume, NULL)
};

static struct platform_driver wokoo_dma_driver = {
	.driver = {
		.name = "wokoo-dma",
		.of_match_table = wokoo_dma_of_match,
        #if 0
		.pm = &wokoo_dma_pm_ops,
        #endif
	},
};

static int __init wokoo_dma_init(void)
{
	return platform_driver_probe(&wokoo_dma_driver, wokoo_dma_probe);
}
subsys_initcall(wokoo_dma_init);
