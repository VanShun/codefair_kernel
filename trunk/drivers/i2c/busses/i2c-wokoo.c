/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/stmp_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/clk.h>

#define DRIVER_NAME "wokoo-i2c"

#define wokoo_I2C_CTRL0		(0x00)
#define wokoo_I2C_CTRL0_SET	(0x04)
#define wokoo_I2C_CTRL0_CLR	(0x08)

#define wokoo_I2C_CTRL0_SFTRST			0x80000000
#define wokoo_I2C_CTRL0_CLKGATE			0x40000000
#define wokoo_I2C_CTRL0_RUN			0x20000000
#define wokoo_I2C_CTRL0_SEND_NAK_ON_LAST		0x02000000
#define wokoo_I2C_CTRL0_PIO_MODE			0x01000000
#define wokoo_I2C_CTRL0_RETAIN_CLOCK		0x00200000
#define wokoo_I2C_CTRL0_POST_SEND_STOP		0x00100000
#define wokoo_I2C_CTRL0_PRE_SEND_START		0x00080000
#define wokoo_I2C_CTRL0_MASTER_MODE		0x00020000
#define wokoo_I2C_CTRL0_DIRECTION			0x00010000
#define wokoo_I2C_CTRL0_XFER_COUNT(v)		((v) & 0x0000FFFF)

#define wokoo_I2C_TIMING0		(0x10)
#define wokoo_I2C_TIMING1		(0x20)
#define wokoo_I2C_TIMING2		(0x30)

#define wokoo_I2C_CTRL1		(0x40)
#define wokoo_I2C_CTRL1_SET	(0x44)
#define wokoo_I2C_CTRL1_CLR	(0x48)

#define wokoo_I2C_CTRL1_CLR_GOT_A_NAK		0x10000000
#define wokoo_I2C_CTRL1_BUS_FREE_IRQ		0x80
#define wokoo_I2C_CTRL1_DATA_ENGINE_CMPLT_IRQ	0x40
#define wokoo_I2C_CTRL1_NO_SLAVE_ACK_IRQ		0x20
#define wokoo_I2C_CTRL1_OVERSIZE_XFER_TERM_IRQ	0x10
#define wokoo_I2C_CTRL1_EARLY_TERM_IRQ		0x08
#define wokoo_I2C_CTRL1_MASTER_LOSS_IRQ		0x04
#define wokoo_I2C_CTRL1_SLAVE_STOP_IRQ		0x02
#define wokoo_I2C_CTRL1_SLAVE_IRQ			0x01

#define wokoo_RD_QUEUE_IRQ			0x40000000
#define wokoo_WR_QUEUE_IRQ			0x20000000




#define wokoo_I2C_STAT		(0x50)
#define wokoo_I2C_STAT_GOT_A_NAK			0x10000000
#define wokoo_I2C_STAT_BUS_BUSY			0x00000800
#define wokoo_I2C_STAT_CLK_GEN_BUSY		0x00000400


#define wokoo_I2C_QUEUECTRL		(0x60)
#define wokoo_I2C_QUEUECTRL_SET	(0x64)
#define wokoo_I2C_QUEUECTRL_CLR	(0x68)


#define wokoo_I2C_QUEUESTAT		(0x70)
#define wokoo_I2C_QUEUESTAT_SET	(0x74)
#define wokoo_I2C_QUEUESTAT_CLR	(0x78)


#define wokoo_I2C_QUEUECMD		(0x80)
#define wokoo_I2C_QUEUECMD_SET	(0x84)
#define wokoo_I2C_QUEUECMD_CLR	(0x88)

#define QUEUE_RUN    BIT(5)
#define WR_CLEAR  BIT(3)
#define RD_CLEAR  BIT(4)
#define WR_QUEUE_EMPTY BIT(5)
#define WR_QUEUE_FULL BIT (6)


#define PIO_QUEUE_MODE  BIT(2)

#define RD_QUEUE_IRQ_EN  BIT(1)
#define  WR_QUEUE_IRQ_EN  BIT(0)

#define I2C_CTRL0_SFTRST  BIT(31)

#define I2C_CTRL0_CLKGATE  BIT(30)


#define RD_QUEUE_EMPTY   BIT(13)
#define wokoo_I2C_DATA(i2c)	((i2c->dev_type == wokoo_I2C_V1) ? 0x60 : 0xa0)


#define wokoo_I2C_QUEUE_DATA(i2c)	(0x90)

#define wokoo_I2C_DEBUG0_CLR(i2c)	((i2c->dev_type == wokoo_I2C_V1) ? 0x78 : 0xb8)

#define wokoo_I2C_DEBUG0_DMAREQ	0x80000000

#define wokoo_I2C_IRQ_MASK	(wokoo_I2C_CTRL1_DATA_ENGINE_CMPLT_IRQ | \
				 wokoo_I2C_CTRL1_NO_SLAVE_ACK_IRQ | \
				 wokoo_I2C_CTRL1_EARLY_TERM_IRQ | \
				 wokoo_I2C_CTRL1_MASTER_LOSS_IRQ | \
				 wokoo_I2C_CTRL1_SLAVE_STOP_IRQ | \
				 wokoo_I2C_CTRL1_SLAVE_IRQ)


#define wokoo_CMD_I2C_SELECT	(wokoo_I2C_CTRL0_RETAIN_CLOCK |	\
				 wokoo_I2C_CTRL0_PRE_SEND_START |	\
				 wokoo_I2C_CTRL0_MASTER_MODE |	\
				 wokoo_I2C_CTRL0_DIRECTION |	\
				 wokoo_I2C_CTRL0_XFER_COUNT(1))

#define wokoo_CMD_I2C_WRITE	(wokoo_I2C_CTRL0_PRE_SEND_START |	\
				 wokoo_I2C_CTRL0_MASTER_MODE |	\
				 wokoo_I2C_CTRL0_DIRECTION)

#define wokoo_CMD_I2C_READ	(wokoo_I2C_CTRL0_SEND_NAK_ON_LAST | \
				 wokoo_I2C_CTRL0_MASTER_MODE)



#define wokoo_CMD_QUE_I2C_SELECT	(wokoo_I2C_CTRL0_RETAIN_CLOCK |	\
				 wokoo_I2C_CTRL0_PRE_SEND_START |	\
				 wokoo_I2C_CTRL0_MASTER_MODE |	\
				 wokoo_I2C_CTRL0_DIRECTION |	\
				 wokoo_I2C_CTRL0_XFER_COUNT(1))

#define wokoo_CMD_QUE_I2C_WRITE	(wokoo_I2C_CTRL0_PRE_SEND_START |	\
				 wokoo_I2C_CTRL0_MASTER_MODE |	\
				 wokoo_I2C_CTRL0_DIRECTION)

#define wokoo_CMD_QUE_I2C_READ	(wokoo_I2C_CTRL0_SEND_NAK_ON_LAST | \
   wokoo_I2C_CTRL0_POST_SEND_STOP \
|   wokoo_I2C_CTRL0_MASTER_MODE 	\
|  wokoo_I2C_CTRL0_PRE_SEND_START  \
)



#define I2C_DMA_BUFFER_SIZE    4096

enum wokoo_i2c_devtype {
	wokoo_I2C_UNKNOWN = 0,
	wokoo_I2C_V1,
	wokoo_I2C_V2,
};

enum wokoo_i2c_mode{
    WOKOO_I2C_PIO= 0,
	WOKOO_I2C_PIO_QUE,
	WOKOO_I2C_DMA
};

/**
 * struct wokoo_i2c_dev - per device, private wokoo-I2C data
 *
 * @dev: driver model device node
 * @dev_type: distinguish i.MX23/i.MX28 features
 * @regs: IO registers pointer
 * @cmd_complete: completion object for transaction wait
 * @cmd_err: error code for last transaction
 * @adapter: i2c subsystem adapter node
 */
struct wokoo_i2c_dev {
	struct device *dev;
	enum wokoo_i2c_devtype dev_type;
	void __iomem *regs;
	struct completion cmd_complete;
	int cmd_err;
	enum wokoo_i2c_mode mode;
	struct i2c_adapter adapter;
	struct clk				*clk_sclk;
	struct clk				*clk_mclk;
	struct clk				*clk_pclk;
	uint32_t timing0;
	uint32_t timing1;
	uint32_t timing2;

	/* DMA support components */
	struct dma_chan			*rx_dma;
	struct dma_chan			*tx_dma;
	uint32_t			pio_data[2];
	uint32_t			addr_data;
	struct scatterlist		sg_io[2];
	bool				dma_read;
};
static void wokoo_i2c_pio_trigger_cmd(struct wokoo_i2c_dev *i2c, u32 cmd);
static int wokoo_i2c_pio_wait_xfer_end(struct wokoo_i2c_dev *i2c);
static void wokoo_i2c_pio_trigger_write_cmd(struct wokoo_i2c_dev *i2c, u32 cmd,
					  u32 data);
					  
static  void  wokoo_i2c_queue_read_data(struct wokoo_i2c_dev *i2c,struct i2c_msg *msg)
{
	int sef_len;
	int len;
	int i = 0;
	int length = 0;
	unsigned int data;
    sef_len = msg->len % 4;
	len = msg->len /4;
	while (len > 0)
	{  
		data = readl(i2c->regs + wokoo_I2C_QUEUE_DATA(i2c));
    	for (i = 0; i < 4 ; i ++)
        {
			msg->buf[length ++] = data & 0xff;
			data >>= 8;
     	}
	
		len -= 1;
	}	
	
	if (sef_len)
	{	
		data = readl(i2c->regs + wokoo_I2C_QUEUE_DATA(i2c));
		for (i = 0; i < sef_len ; i ++)
        {
		   msg->buf[length ++] = data & 0xff;
			data >>= 8;
     	}
	}
}
static int wokoo_i2c_reset(struct wokoo_i2c_dev *i2c)
{
	int ret = stmp_reset_block(i2c->regs);
	if (ret)
		return ret;

	/*
	 * Configure timing for the I2C block. The I2C TIMING2 register has to
	 * be programmed with this particular magic number. The rest is derived
	 * from the XTAL speed and requested I2C speed.
	 */
	writel(i2c->timing0, i2c->regs + wokoo_I2C_TIMING0);
	writel(i2c->timing1, i2c->regs + wokoo_I2C_TIMING1);
	writel(i2c->timing2, i2c->regs + wokoo_I2C_TIMING2);

	writel(wokoo_I2C_IRQ_MASK << 8 , i2c->regs + wokoo_I2C_CTRL1_SET);

	return 0;
}

static int wokoo_i2c_pio_wait_xfer_end(struct wokoo_i2c_dev *i2c)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);

	while (readl(i2c->regs + wokoo_I2C_CTRL0) & wokoo_I2C_CTRL0_RUN) {
		
		if (readl(i2c->regs + wokoo_I2C_CTRL1) &
				wokoo_I2C_CTRL1_NO_SLAVE_ACK_IRQ)
			return -ENXIO;
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cond_resched();
	}

	return 0;
}



static int wokoo_i2c_pio_check_error_state(struct wokoo_i2c_dev *i2c)
{
	u32 state;

	state = readl(i2c->regs + wokoo_I2C_CTRL1_CLR) & wokoo_I2C_IRQ_MASK;

	if (state & wokoo_I2C_CTRL1_NO_SLAVE_ACK_IRQ)
		i2c->cmd_err = -ENXIO;
	else if (state & (wokoo_I2C_CTRL1_EARLY_TERM_IRQ |
			  wokoo_I2C_CTRL1_MASTER_LOSS_IRQ |
			  wokoo_I2C_CTRL1_SLAVE_STOP_IRQ |
			  wokoo_I2C_CTRL1_SLAVE_IRQ))
		i2c->cmd_err = -EIO;

	return i2c->cmd_err;
}

static void wokoo_i2c_pio_trigger_cmd(struct wokoo_i2c_dev *i2c, u32 cmd)
{
	u32 reg;

	writel(cmd, i2c->regs + wokoo_I2C_CTRL0);
	/* readback makes sure the write is latched into hardware */
	reg = readl(i2c->regs + wokoo_I2C_CTRL0);
	reg |= wokoo_I2C_CTRL0_RUN;
	writel(reg, i2c->regs + wokoo_I2C_CTRL0);
}


/*
 * Start WRITE transaction on the I2C bus. By studying wokoo datasheet,
 * CTRL0::PIO_MODE bit description clarifies the order in which the registers
 * must be written during PIO mode operation. First, the CTRL0 register has
 * to be programmed with all the necessary bits but the RUN bit. Then the
 * payload has to be written into the DATA register. Finally, the transmission
 * is executed by setting the RUN bit in CTRL0.
 */
static void wokoo_i2c_pio_trigger_write_cmd(struct wokoo_i2c_dev *i2c,
                                                         u32 cmd,
					                                     u32 data)
{
	writel(cmd, i2c->regs + wokoo_I2C_CTRL0);
	if (i2c->dev_type == wokoo_I2C_V1)
		writel(wokoo_I2C_CTRL0_PIO_MODE, i2c->regs + wokoo_I2C_CTRL0_SET);
	writel(data, i2c->regs + wokoo_I2C_DATA(i2c));
	writel(wokoo_I2C_CTRL0_RUN, i2c->regs + wokoo_I2C_CTRL0_SET);
	
}

static void wokoo_i2c_queue_pio_trigger_write_cmd(struct wokoo_i2c_dev *i2c, u32 cmd,
					  u32 data,bool en_run, bool en_data,bool wr)
{
    u32 reg;
    writel(PIO_QUEUE_MODE  |  readl (i2c->regs + wokoo_I2C_QUEUECTRL_SET), i2c->regs + wokoo_I2C_QUEUECTRL_SET);
	writel(cmd, i2c->regs + wokoo_I2C_QUEUECMD);
	if (en_data)
	    writel(data, i2c->regs + wokoo_I2C_DATA(i2c));
    if (en_run)
    {
		reg = readl(i2c->regs + wokoo_I2C_QUEUECTRL);
		reg |= QUEUE_RUN ;
	    writel(reg, i2c->regs + wokoo_I2C_QUEUECTRL_SET);
	}

}


static int wokoo_i2_queue_pio_wait_xfer_end(struct wokoo_i2c_dev *i2c)
{
	unsigned long time_left;
    int ret = 0;
    if (readl(i2c->regs + wokoo_I2C_QUEUECTRL) & QUEUE_RUN)
    {
        time_left = wait_for_completion_timeout(&i2c->cmd_complete,
								msecs_to_jiffies(100));
		if (!time_left)
		{
		    ret = -ETIMEDOUT;
		}
	}
	else
	{
		ret  = -ENXIO;
	}
	writel(QUEUE_RUN,i2c->regs + wokoo_I2C_QUEUECTRL_CLR);
    return ret;
}


static bool wokoo_i2c_read_is_empty(struct wokoo_i2c_dev *i2c)
{
    unsigned long timeout = jiffies + msecs_to_jiffies(10);
    bool ret = false;
	while (readl(i2c->regs + wokoo_I2C_QUEUESTAT) & RD_QUEUE_EMPTY) 
	{
		
	
		if (time_after(jiffies, timeout))
		{
		   writel(RD_CLEAR,i2c->regs + wokoo_I2C_QUEUECTRL);
		   udelay(1);
		   writel(RD_CLEAR,i2c->regs + wokoo_I2C_QUEUECTRL_CLR);
		    ret = true;
            break;

		}
			
		cond_resched();
	}
	return ret;
}


static bool wokoo_i2c_read_is_full(struct wokoo_i2c_dev *i2c)
{
    unsigned long timeout = jiffies + msecs_to_jiffies(10);
    bool ret = false;
	while (readl(i2c->regs + wokoo_I2C_QUEUESTAT) & WR_QUEUE_FULL) 
	{
		if (time_after(jiffies, timeout))
		{
		  
		   if (readl(i2c->regs + wokoo_I2C_QUEUESTAT) & WR_QUEUE_FULL)
		   {
			   ret = true;
		   }
           break;
		}
			
		cond_resched();
	}
	return ret;
}

static int wokoo_i2c_pio_setup_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msg, uint32_t flags)
{
	struct wokoo_i2c_dev *i2c = i2c_get_adapdata(adap);
	uint32_t addr_data = i2c_8bit_addr_from_msg(msg);
	uint32_t data = 0;
	int i, ret, xlen = 0, xmit = 0;
	uint32_t start;

	/* Mute IRQs coming from this block. */
	writel(wokoo_I2C_IRQ_MASK << 8, i2c->regs + wokoo_I2C_CTRL1_CLR);

	/*
	 * MX23 idea:
	 * - Enable CTRL0::PIO_MODE (1 << 24)
	 * - Enable CTRL1::ACK_MODE (1 << 27)
	 *
	 * WARNING! The MX23 is broken in some way, even if it claims
	 * to support PIO, when we try to transfer any amount of data
	 * that is not aligned to 4 bytes, the DMA engine will have
	 * bits in DEBUG1::DMA_BYTES_ENABLES still set even after the
	 * transfer. This in turn will mess up the next transfer as
	 * the block it emit one byte write onto the bus terminated
	 * with a NAK+STOP. A possible workaround is to reset the IP
	 * block after every PIO transmission, which might just work.
	 *
	 * NOTE: The CTRL0::PIO_MODE description is important, since
	 * it outlines how the PIO mode is really supposed to work.
	 */
	if (msg->flags & I2C_M_RD) {
		/*
		 * PIO READ transfer:
		 *
		 * This transfer MUST be limited to 4 bytes maximum. It is not
		 * possible to transfer more than four bytes via PIO, since we
		 * can not in any way make sure we can read the data from the
		 * DATA register fast enough. Besides, the RX FIFO is only four
		 * bytes deep, thus we can only really read up to four bytes at
		 * time. Finally, there is no bit indicating us that new data
		 * arrived at the FIFO and can thus be fetched from the DATA
		 * register.
		 */
		BUG_ON(msg->len > 4);
		/* SELECT command. */
		
	    wokoo_i2c_pio_trigger_write_cmd(i2c, wokoo_CMD_I2C_SELECT,
					      addr_data);

		ret = wokoo_i2c_pio_wait_xfer_end(i2c);
		if (ret) {
			dev_dbg(i2c->dev,
				"PIO: Failed to send SELECT command!\n");
			goto cleanup;
		}

		/* READ command. */
 	wokoo_i2c_pio_trigger_cmd(i2c,
					wokoo_CMD_I2C_READ | flags |
					wokoo_I2C_CTRL0_XFER_COUNT(msg->len));

		ret = wokoo_i2c_pio_wait_xfer_end(i2c);
		if (ret) {
			dev_dbg(i2c->dev,
				"PIO: Failed to send READ command!\n");
			goto cleanup;
		}



		data = readl(i2c->regs + wokoo_I2C_DATA(i2c));
		for (i = 0; i < msg->len; i++) {
			msg->buf[i] = data & 0xff;
			data >>= 8;
		}
		
	} else {
		
		/*
		 * PIO WRITE transfer:
		 *
		 * The code below implements clock stretching to circumvent
		 * the possibility of kernel not being able to supply data
		 * fast enough. It is possible to transfer arbitrary amount
		 * of data using PIO write.
		 */

		/*
		 * The LSB of data buffer is the first byte blasted across
		 * the bus. Higher order bytes follow. Thus the following
		 * filling schematic.
		 */

		data = addr_data << 24;

		/* Start the transfer with START condition. */
		start = wokoo_I2C_CTRL0_PRE_SEND_START;

		/* If the transfer is long, use clock stretching. */
		if (msg->len > 3)
			start |= wokoo_I2C_CTRL0_RETAIN_CLOCK;

		for (i = 0; i < msg->len; i++) {
			data >>= 8;
			data |= (msg->buf[i] << 24);

			xmit = 0;

			/* This is the last transfer of the message. */
			if (i + 1 == msg->len) {
				/* Add optional STOP flag. */
				start |= flags;
				/* Remove RETAIN_CLOCK bit. */
				start &= ~wokoo_I2C_CTRL0_RETAIN_CLOCK;
				xmit = 1;
			}

			/* Four bytes are ready in the "data" variable. */
			if ((i & 3) == 2)
				xmit = 1;

			/* Nothing interesting happened, continue stuffing. */
			if (!xmit)
				continue;

			/*
			 * Compute the size of the transfer and shift the
			 * data accordingly.
			 *
			 * i = (4k + 0) .... xlen = 2
			 * i = (4k + 1) .... xlen = 3
			 * i = (4k + 2) .... xlen = 4
			 * i = (4k + 3) .... xlen = 1
			 */

			if ((i % 4) == 3)
				xlen = 1;
			else
				xlen = (i % 4) + 2;

			data >>= (4 - xlen) * 8;

			dev_dbg(i2c->dev,
			"PIO: len=%i pos=%i total=%i [W%s%s%s],0x%x\n",
				xlen, i, msg->len,
				start & wokoo_I2C_CTRL0_PRE_SEND_START ? "S" : "",
				start & wokoo_I2C_CTRL0_POST_SEND_STOP ? "E" : "",
				start & wokoo_I2C_CTRL0_RETAIN_CLOCK ? "C" : "",data);

			writel(wokoo_I2C_DEBUG0_DMAREQ,
			       i2c->regs + wokoo_I2C_DEBUG0_CLR(i2c));

			wokoo_i2c_pio_trigger_write_cmd(i2c,
				start | wokoo_I2C_CTRL0_MASTER_MODE |
				wokoo_I2C_CTRL0_DIRECTION |
				wokoo_I2C_CTRL0_XFER_COUNT(xlen), data);

			/* The START condition is sent only once. */
			start &= ~wokoo_I2C_CTRL0_PRE_SEND_START;

			/* Wait for the end of the transfer. */
			ret = wokoo_i2c_pio_wait_xfer_end(i2c);
			if (ret) {
				dev_dbg(i2c->dev,
					"PIO: Failed to finish WRITE cmd!\n");
				break;
			}

			/* Check NAK here. */
			ret = readl(i2c->regs + wokoo_I2C_STAT) &
				    wokoo_I2C_STAT_GOT_A_NAK;
			if (ret) {
				ret = -ENXIO;
				goto cleanup;
			}
		}
	}

	/* make sure we capture any occurred error into cmd_err */
	ret = wokoo_i2c_pio_check_error_state(i2c);

cleanup:
	/* Clear any dangling IRQs and re-enable interrupts. */
	writel(wokoo_I2C_IRQ_MASK, i2c->regs + wokoo_I2C_CTRL1_CLR);
	writel(wokoo_I2C_IRQ_MASK << 8, i2c->regs + wokoo_I2C_CTRL1_SET);

	/* Clear the PIO_MODE on i.MX23 */
	if (i2c->dev_type == wokoo_I2C_V1)
		writel(wokoo_I2C_CTRL0_PIO_MODE, i2c->regs + wokoo_I2C_CTRL0_CLR);

	return ret;
}


static int wokoo_i2c_queue_pio_setup_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msg, uint32_t flags)
{
	struct wokoo_i2c_dev *i2c = i2c_get_adapdata(adap);
	uint32_t addr_data = i2c_8bit_addr_from_msg(msg);
	uint32_t data = 0;
	int i, xlen = 0, xmit = 0;
	uint32_t start;
	int ret  = 0;
	if (msg->flags & I2C_M_RD) {
		
		/* SELECT command. */
			
	   wokoo_i2c_queue_pio_trigger_write_cmd(i2c,
	                                         wokoo_CMD_I2C_SELECT,
	                                         addr_data,false,true,false);
		/* READ command. */
 	    wokoo_i2c_queue_pio_trigger_write_cmd(i2c,
					                    wokoo_CMD_QUE_I2C_READ 
					                    | flags 
					                    | wokoo_I2C_CTRL0_XFER_COUNT(msg->len),
					                    0,true,false,false);
		
					
        ret = wokoo_i2_queue_pio_wait_xfer_end(i2c);
		if (!ret && false == wokoo_i2c_read_is_empty(i2c))
		{
		    wokoo_i2c_queue_read_data(i2c,msg);
		}
		
	} else {
		
		data = addr_data << 24;

		/* Start the transfer with START condition. */
		start = wokoo_I2C_CTRL0_PRE_SEND_START;

		/* If the transfer is long, use clock stretching. */
		if (msg->len > 3)
            start |= wokoo_I2C_CTRL0_RETAIN_CLOCK;

		for (i = 0; i < msg->len; i++) {
			data >>= 8;
			data |= (msg->buf[i] << 24);

			xmit = 0;

			/* This is the last transfer of the message. */
			if (i + 1 == msg->len) {
				/* Add optional STOP flag. */
				start |= flags;
				/* Remove RETAIN_CLOCK bit. */
				start &= ~wokoo_I2C_CTRL0_RETAIN_CLOCK;
				xmit = 1;
			}

			/* Four bytes are ready in the "data" variable. */
			if ((i & 3) == 2)
				xmit = 1;

			/* Nothing interesting happened, continue stuffing. */
			if (!xmit)
				continue;

			/*
			 * Compute the size of the transfer and shift the
			 * data accordingly.
			 *
			 * i = (4k + 0) .... xlen = 2
			 * i = (4k + 1) .... xlen = 3
			 * i = (4k + 2) .... xlen = 4
			 * i = (4k + 3) .... xlen = 1
			 */

			if ((i % 4) == 3)
				xlen = 1;
			else
				xlen = (i % 4) + 2;

			data >>= (4 - xlen) * 8;

			dev_dbg(i2c->dev,
			"PIO: len=%i pos=%i total=%i [W%s%s%s],0x%x\n",
				xlen, i, msg->len,
				start & wokoo_I2C_CTRL0_PRE_SEND_START ? "S" : "",
				start & wokoo_I2C_CTRL0_POST_SEND_STOP ? "E" : "",
				start & wokoo_I2C_CTRL0_RETAIN_CLOCK ? "C" : "",data);
			writel(wokoo_I2C_DEBUG0_DMAREQ,
						   i2c->regs + wokoo_I2C_DEBUG0_CLR(i2c));

			writel(I2C_CTRL0_SFTRST |I2C_CTRL0_CLKGATE,i2c->regs + wokoo_I2C_CTRL0_CLR);
			writel(WR_CLEAR | RD_CLEAR,i2c->regs + wokoo_I2C_QUEUECTRL);
		  
		   writel(WR_CLEAR | RD_CLEAR,i2c->regs + wokoo_I2C_QUEUECTRL_CLR);
           
			
			if (true ==  wokoo_i2c_read_is_full(i2c))
			{
				ret  = -EIO;
				goto cleanup;
			}
   
			wokoo_i2c_queue_pio_trigger_write_cmd(i2c,
				start | wokoo_I2C_CTRL0_MASTER_MODE |
				wokoo_I2C_CTRL0_DIRECTION |
				wokoo_I2C_CTRL0_XFER_COUNT(xlen), data,true,true,true);	
			 
			  ret = wokoo_i2_queue_pio_wait_xfer_end(i2c);
			  if (ret)
			  	break;
		}
	}
cleanup:
		writel(PIO_QUEUE_MODE, i2c->regs + wokoo_I2C_QUEUESTAT_CLR);
	return ret;
}


/*
 * Low level master read/write transaction.
 */
static int wokoo_i2c_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg,
				int stop)
{
	struct wokoo_i2c_dev *i2c = i2c_get_adapdata(adap);
	int ret = 0;
	int flags;
	//int use_pio = 0;
	//unsigned long time_left;
	//char restart_cnt = 1;

	flags = stop ? wokoo_I2C_CTRL0_POST_SEND_STOP : 0;
	i2c->cmd_err = 0;
	if (i2c->mode == WOKOO_I2C_PIO) {
	
		ret = wokoo_i2c_pio_setup_xfer(adap, msg, flags);
	
		/* No need to reset the block if NAK was received. */
		if (ret && (ret != -ENXIO))
			wokoo_i2c_reset(i2c);
	} else {
		    reinit_completion(&i2c->cmd_complete);
			
		    ret = wokoo_i2c_queue_pio_setup_xfer(adap, msg, flags);
	
		    if (!ret)
			    ret = i2c->cmd_err;
			if (ret && (ret != -ENXIO))
			wokoo_i2c_reset(i2c);
            	
	}

	if (ret == -ENXIO) {
		
		/*
		 * If the transfer fails with a NAK from the slave the
		 * controller halts until it gets told to return to idle state.
		 */
		 
		
		writel(wokoo_I2C_CTRL1_CLR_GOT_A_NAK,
		       i2c->regs + wokoo_I2C_CTRL1_SET);
	}
	return ret;

timeout:
	dev_info(i2c->dev, "Timeout!\n");
	ret = wokoo_i2c_reset(i2c);
	if (ret)
		return ret;

	return -ETIMEDOUT;
}

static int wokoo_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			int num)
{
	int i;
	int err;

	for (i = 0; i < num; i++) {
		err = wokoo_i2c_xfer_msg(adap, &msgs[i], i == (num - 1));
		if (err)
			return err;
	}

	return num;
}

static u32 wokoo_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static irqreturn_t wokoo_i2c_isr(int this_irq, void *dev_id)
{
	struct wokoo_i2c_dev *i2c = dev_id;
	u32 stat = readl(i2c->regs + wokoo_I2C_CTRL1) & wokoo_I2C_IRQ_MASK;

	if (!stat)
		return IRQ_NONE;
     
	if (stat & wokoo_I2C_CTRL1_NO_SLAVE_ACK_IRQ)
	{
	   
		i2c->cmd_err = -ENXIO;
	}
	else if (stat &(wokoo_I2C_CTRL1_EARLY_TERM_IRQ |
			  wokoo_I2C_CTRL1_MASTER_LOSS_IRQ |
			  wokoo_I2C_CTRL1_SLAVE_STOP_IRQ |
			  wokoo_I2C_CTRL1_SLAVE_IRQ))
	{
		i2c->cmd_err = -EIO;
	}

	
	if (i2c->mode != WOKOO_I2C_PIO)
	    complete(&i2c->cmd_complete);
	writel(stat, i2c->regs + wokoo_I2C_CTRL1_CLR);

	return IRQ_HANDLED;
}

static const struct i2c_algorithm wokoo_i2c_algo = {
	.master_xfer = wokoo_i2c_xfer,
	.functionality = wokoo_i2c_func,
};

static const struct i2c_adapter_quirks wokoo_i2c_quirks = {
	.flags = I2C_AQ_NO_ZERO_LEN,
};

static void wokoo_i2c_derive_timing(struct wokoo_i2c_dev *i2c, uint32_t speed)
{
	/* The I2C block clock runs at 24MHz */
	// const uint32_t clk = 24000000;
#if 0
	printk("old i2c\n");
	const uint32_t clk = 24000000;
	uint32_t divider;
	uint16_t high_count, low_count, rcv_count, xmit_count;
	uint32_t bus_free, leadin;
	struct device *dev = i2c->dev;

	divider = DIV_ROUND_UP(clk, speed);

	if (divider < 25) {
		/*
		 * limit the divider, so that min(low_count, high_count)
		 * is >= 1
		 */
		divider = 25;
		dev_warn(dev,
			"Speed too high (%u.%03u kHz), using %u.%03u kHz\n",
			speed / 1000, speed % 1000,
			clk / divider / 1000, clk / divider % 1000);
	} else if (divider > 1897) {
		/*
		 * limit the divider, so that max(low_count, high_count)
		 * cannot exceed 1023
		 */
		divider = 1897;
		dev_warn(dev,
			"Speed too low (%u.%03u kHz), using %u.%03u kHz\n",
			speed / 1000, speed % 1000,
			clk / divider / 1000, clk / divider % 1000);
	}

	/*
	 * The I2C spec specifies the following timing data:
	 *                          standard mode  fast mode Bitfield name
	 * tLOW (SCL LOW period)     4700 ns        1300 ns
	 * tHIGH (SCL HIGH period)   4000 ns         600 ns
	 * tSU;DAT (data setup time)  250 ns         100 ns
	 * tHD;STA (START hold time) 4000 ns         600 ns
	 * tBUF (bus free time)      4700 ns        1300 ns
	 *
	 * The hardware (of the i.MX28 at least) seems to add 2 additional
	 * clock cycles to the low_count and 7 cycles to the high_count.
	 * This is compensated for by subtracting the respective constants
	 * from the values written to the timing registers.
	 */
	if (speed > 100000) {
		/* fast mode */
		low_count = DIV_ROUND_CLOSEST(divider * 13, (13 + 6));
		high_count = DIV_ROUND_CLOSEST(divider * 6, (13 + 6));
		leadin = DIV_ROUND_UP(600 * (clk / 1000000), 1000);
		bus_free = DIV_ROUND_UP(1300 * (clk / 1000000), 1000);
	} else {
		/* normal mode */
		low_count = DIV_ROUND_CLOSEST(divider * 47, (47 + 40));
		high_count = DIV_ROUND_CLOSEST(divider * 40, (47 + 40));
		leadin = DIV_ROUND_UP(4700 * (clk / 1000000), 1000);
		bus_free = DIV_ROUND_UP(4700 * (clk / 1000000), 1000);
	}
	rcv_count = high_count * 3 / 8;
	xmit_count = low_count * 3 / 8;

	dev_dbg(dev,
		"speed=%u(actual %u) divider=%u low=%u high=%u xmit=%u rcv=%u leadin=%u bus_free=%u\n",
		speed, clk / divider, divider, low_count, high_count,
		xmit_count, rcv_count, leadin, bus_free);

	low_count -= 2;
	high_count -= 7;
	i2c->timing0 = (high_count << 16) | rcv_count;
	i2c->timing1 = (low_count << 16) | xmit_count;
	i2c->timing2 = (bus_free << 16 | leadin);
#endif
#if 1
	if (speed ==100000)
	{
	i2c->timing0 = 0x00ff0080;
	i2c->timing1 = 0x00ff0080;
	i2c->timing2 = 0x0015000d;

	}
	else{

	i2c->timing0 = 0x0020000F;
	i2c->timing1 = 0x00200012;
	i2c->timing2 = 0x0015000d;

	}
	// printk("wokoo i2c clk\n");
#endif
}

static int wokoo_i2c_get_ofdata(struct wokoo_i2c_dev *i2c)
{
	uint32_t speed;
	struct device *dev = i2c->dev;
	struct device_node *node = dev->of_node;
	int ret;

	ret = of_property_read_u32(node, "clock-frequency", &speed);
	if (ret) {
		dev_warn(dev, "No I2C speed selected, using 100kHz\n");
		speed = 100000;
	}
    
	ret = of_property_read_u32(node, "mode", &i2c->mode);
	if (ret) {
		dev_warn(dev, " I2C mode select failed\n");
	}
	else{
		i2c->mode = 0;
	}
	wokoo_i2c_derive_timing(i2c, speed);

	return 0;
}

static const struct platform_device_id wokoo_i2c_devtype[] = {
	 {
		.name = "wokoo-i2c",
		.driver_data = wokoo_I2C_V2,
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, wokoo_i2c_devtype);

static const struct of_device_id wokoo_i2c_dt_ids[] = {
	{ .compatible = "wokoo,wokoo-i2c", .data = &wokoo_i2c_devtype[1], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wokoo_i2c_dt_ids);

static int wokoo_i2c_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =of_match_device(wokoo_i2c_dt_ids, &pdev->dev);
	struct device *dev = &pdev->dev;
	struct wokoo_i2c_dev *i2c;
	struct i2c_adapter *adap;
	struct resource *res;
	int err, irq,ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	if (of_id) {
		const struct platform_device_id *device_id = of_id->data;
		i2c->dev_type = device_id->driver_data;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2c->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2c->regs))
		return PTR_ERR(i2c->regs);



	i2c->clk_pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(i2c->clk_pclk)) {
		dev_err(&pdev->dev, "can't get I2C pclk clock\n");
		return PTR_ERR(i2c->clk_pclk);
	}

	ret = clk_prepare_enable(i2c->clk_pclk);
	if (ret) {
		dev_err(&pdev->dev, "can't enable I2C pclk clock\n");
		return ret;
	}
	i2c->clk_mclk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(i2c->clk_mclk)) {
		dev_err(&pdev->dev, "can't get I2C mclk clock\n");
		return PTR_ERR(i2c->clk_mclk);
	}
    ret = clk_prepare_enable(i2c->clk_mclk);
	if (ret) {
		dev_err(&pdev->dev, "can't enable I2C mclk clock\n");
		return ret;
	}
	i2c->clk_sclk = devm_clk_get(&pdev->dev, "sclk");
	if (IS_ERR(i2c->clk_sclk)) {
		dev_err(&pdev->dev, "can't get I2C sclk clock\n");
		return PTR_ERR(i2c->clk_sclk);
	}
	clk_set_rate(i2c->clk_sclk, 24000000);
	ret = clk_prepare_enable(i2c->clk_sclk);
	if (ret) {
		dev_err(&pdev->dev, "can't enable I2C sclk clock\n");
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	err = devm_request_irq(dev, irq, wokoo_i2c_isr, 0, dev_name(dev), i2c);
	if (err)
		return err;

	i2c->dev = dev;

	init_completion(&i2c->cmd_complete);
	if (dev->of_node) {
		err = wokoo_i2c_get_ofdata(i2c);
		if (err)
			return err;
	}
	platform_set_drvdata(pdev, i2c);

	/* Do reset to enforce correct startup after pinmuxing */
	err = wokoo_i2c_reset(i2c);
	if (err)
		return err;

	adap = &i2c->adapter;
	strlcpy(adap->name, "wokoo I2C adapter", sizeof(adap->name));
	adap->owner = THIS_MODULE;
	adap->algo = &wokoo_i2c_algo;
	adap->quirks = &wokoo_i2c_quirks;
	adap->dev.parent = dev;
	adap->nr = pdev->id;
	adap->dev.of_node = pdev->dev.of_node;
	i2c_set_adapdata(adap, i2c);
	err = i2c_add_numbered_adapter(adap);
	if (err) {
		writel(wokoo_I2C_CTRL0_SFTRST ,
				i2c->regs + wokoo_I2C_CTRL0_SET);
		return err;
	}
	

	return 0;
}

static int wokoo_i2c_remove(struct platform_device *pdev)
{
	struct wokoo_i2c_dev *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adapter);

	writel(wokoo_I2C_CTRL0_SFTRST, i2c->regs + wokoo_I2C_CTRL0_SET);

	return 0;
}

static struct platform_driver wokoo_i2c_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = wokoo_i2c_dt_ids,
		   },
	.probe = wokoo_i2c_probe,
	.remove = wokoo_i2c_remove,
};

static int __init wokoo_i2c_init(void)
{
	return platform_driver_register(&wokoo_i2c_driver);
}
subsys_initcall(wokoo_i2c_init);

static void __exit wokoo_i2c_exit(void)
{
	platform_driver_unregister(&wokoo_i2c_driver);
}
module_exit(wokoo_i2c_exit);

MODULE_AUTHOR("linronghua Codefair");
MODULE_DESCRIPTION("wokoo I2C Bus Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
