// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/reset-controller.h>




struct wokoo_i2s_regs
{
	uint32_t SCLK_CFG;     		/*!< Offset: 0x000  Clock configuration register */
	uint32_t FSYNC_CFG;     	/*!< Offset: 0x004  Frame configuration register */
	uint32_t FIFO_STA;     		/*!< Offset: 0x008  FIFO status register */
	uint32_t reserved0;     	/*!< Offset: 0x00C  reserved register */
	uint32_t MODE;     			/*!< Offset: 0x010  Mode register */
	uint32_t REC_FIFO;     		/*!< Offset: 0x014  Receive FIFO address register */
	uint32_t TRAN_FIFO;     	/*!< Offset: 0x018  Transfer FIFO address register */
	uint32_t I2S_IS;            /*!< Offset: 0x01C  Interrupt status register */
	uint32_t I2S_IE;            /*!< Offset: 0x020  Interrupt enable register */
};




#define I2S_SCLK_CFG_SCLK_DIV_POS		(0)
#define I2S_SCLK_CFG_SCLK_DIV_MSK		(0xFFFUL << I2S_SCLK_CFG_SCLK_DIV_POS)

#define I2S_FSYNC_CFG_FSYNC_DIV_POS		(0)
#define I2S_FSYNC_CFG_FSYNC_DIV_MSK		(0xFFFUL << I2S_FSYNC_CFG_FSYNC_DIV_POS)
#define I2S_FSYNC_CFG_FSYNC_BIT_POS		(12)
#define I2S_FSYNC_CFG_FSYNC_BIT_MSK		(0x3UL << I2S_FSYNC_CFG_FSYNC_BIT_POS)
#define I2S_FSYNC_CFG_CHANNEL_NUM_POS	(16)
#define I2S_FSYNC_CFG_CHANNEL_NUM_MSK	(0x7UL << I2S_FSYNC_CFG_CHANNEL_NUM_POS)

#define I2S_FIFO_STA_OBUF_OV			(0x1UL << 0) /* Transfer over run */       
#define I2S_FIFO_STA_IBUF_RE			(0x1UL << 1) /* Receive read empty error */      
#define I2S_FIFO_STA_OBUF_EM			(0x1UL << 2) /* Transfer empty */          
#define I2S_FIFO_STA_OBUF_AE			(0x1UL << 3) /* Transfer close to empty */ 
#define I2S_FIFO_STA_OBUF_HE			(0x1UL << 4) /* Transfer half empty */     
#define I2S_FIFO_STA_OBUF_NF			(0x1UL << 5) /* Transfer not full */       
#define I2S_FIFO_STA_IBUF_FL			(0x1UL << 6) /* Receive full */            
#define I2S_FIFO_STA_IBUF_NE			(0x1UL << 7) /* Receive not empty */       
#define I2S_FIFO_STA_IBUF_HF			(0x1UL << 8) /* Receive half full */       
#define I2S_FIFO_STA_IBUF_AF			(0x1UL << 9) /* Receive close to full */   

#define I2S_MODE_SLAVE_EN				(0x1UL << 0)
#define I2S_MODE_TRAN_EN				(0x1UL << 1)
#define I2S_MODE_REC_EN					(0x1UL << 2)
#define I2S_MODE_I2S_EN					(0x1UL << 3)
#define I2S_MODE_FLUSH_REC_BUF			(0x1UL << 4)
#define I2S_MODE_FLUSH_TRAN_BUF			(0x1UL << 5)
#define I2S_MODE_DLY_MODE				(0x1UL << 6)
#define I2S_MODE_I2S_MODE				(0x1UL << 8)

#define WOKOO_I2S_FIFO_TH				(30)
#define WOKOO_I2S_PLAY_INT_MASK			(I2S_FIFO_STA_OBUF_AE | I2S_FIFO_STA_OBUF_OV)
#define WOKOO_I2S_CAP_INT_MASK			(I2S_FIFO_STA_IBUF_AF | I2S_FIFO_STA_IBUF_RE)


struct wokoo_i2s {
	struct device *dev;
	struct reset_control *rc;
	struct clk *clk[8];
	int clocks;
	struct snd_soc_dai_driver dai;
	void __iomem *base;
	struct wokoo_i2s_regs *regs;
	phys_addr_t base_phys;
	struct snd_dmaengine_dai_dma_data dma_data[2];
	int clk_rate;
	spinlock_t lock;
	int rate;
	int format;
	u8 bits;
	u8 channels;
	u8 id;
	u8 use;
	u32 master:1;
	u32 status:1;
	struct snd_pcm_substream __rcu *tx_substream;
	struct snd_pcm_substream __rcu *rx_substream;
	unsigned int tx_ptr;
	unsigned int rx_ptr;
};

enum {
	CLK_I2S_PCLK,
	CLK_I2S_SCLK,
	CLK_I2S_GR_CLK
};

enum wokoo_bits {
	WOKOO_BITS_16,
	WOKOO_BITS_18,
	WOKOO_BITS_20,
	WOKOO_BITS_24,
};

#define BUFFER_BYTES_MAX	(2 * 2 * 8 * PERIOD_BYTES_MIN)
/* Must > 4096 for 96K fs */
#define PERIOD_BYTES_MIN	8192
#define PERIODS_MIN			2

static const struct snd_pcm_hardware wokoo_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.buffer_bytes_max = BUFFER_BYTES_MAX,
	.period_bytes_min = PERIOD_BYTES_MIN,
	.period_bytes_max = BUFFER_BYTES_MAX / PERIODS_MIN,
	.periods_min = PERIODS_MIN,
	.periods_max = BUFFER_BYTES_MAX / PERIOD_BYTES_MIN,
	.fifo_size = 128,
};

static void wokoo_i2s_enable_play_irq(struct wokoo_i2s *i2s)
{
	writel(WOKOO_I2S_PLAY_INT_MASK, &i2s->regs->I2S_IE);
}

static void wokoo_i2s_enable_capture_irq(struct wokoo_i2s *i2s)
{
	writel(WOKOO_I2S_CAP_INT_MASK, &i2s->regs->I2S_IE);
}

static void wokoo_i2s_disable_irq(struct wokoo_i2s *i2s)
{
	writel(0, &i2s->regs->I2S_IE);
}

static int wokoo_i2s_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static void wokoo_i2s_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *cpu_dai)
{
}

static void wokoo_i2s_txctrl(struct snd_soc_dai *cpu_dai, int on)
{
	struct wokoo_i2s *i2s = dev_get_drvdata(cpu_dai->dev);
	struct wokoo_i2s_regs *regs = i2s->regs;
	u32 val;

	spin_lock(&i2s->lock);
	val  = readl(&regs->MODE);

	/* Reset fifo */
	val |= I2S_MODE_FLUSH_TRAN_BUF;
	writel(val, &i2s->regs->MODE);
	val &= ~I2S_MODE_FLUSH_TRAN_BUF;
	writel(val, &i2s->regs->MODE);

	/* Enable module */
	if (on) {
		val |= I2S_MODE_TRAN_EN | I2S_MODE_I2S_EN;
	} else {
		val &= ~(I2S_MODE_TRAN_EN | I2S_MODE_I2S_EN);
	}

	writel(val, &i2s->regs->MODE);
	spin_unlock(&i2s->lock);
}

static void wokoo_i2s_rxctrl(struct snd_soc_dai *cpu_dai, int on)
{
	struct wokoo_i2s *i2s = dev_get_drvdata(cpu_dai->dev);
	u32 val;

	spin_lock(&i2s->lock);
	val  = readl(&i2s->regs->MODE);

	/* Reset fifo */
	val |= I2S_MODE_FLUSH_REC_BUF;
	writel(val, &i2s->regs->MODE);
	val &= ~I2S_MODE_FLUSH_REC_BUF;
	writel(val, &i2s->regs->MODE);

	/* Enable module */
	if (on) {
		val |= I2S_MODE_REC_EN | I2S_MODE_I2S_EN;
	} else {
		val &= ~(I2S_MODE_REC_EN | I2S_MODE_I2S_EN);
	}

	writel(val, &i2s->regs->MODE);
	spin_unlock(&i2s->lock);
}

static int wokoo_i2s_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct wokoo_i2s *i2s = dev_get_drvdata(cpu_dai->dev);

	/*
	 * We don't actually set the hardware until the hw_params
	 * call, but we need to validate the user input here.
	 */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	i2s->format = fmt;
	i2s->master = (i2s->format & SND_SOC_DAIFMT_MASTER_MASK) ==
		      SND_SOC_DAIFMT_CBS_CFS;

	return 0;
}


static int wokoo_i2s_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *cpu_dai)
{
	struct wokoo_i2s *i2s = dev_get_drvdata(cpu_dai->dev);
	u32 bits = 0, rate = 0;
	u32 i2s_sclk = 0;
	u32 sclk_div = 0;
	u32 fsyn_div = 0;
	u32 channel  = 0;
	u32 tdm  = 0;
	u32 val;
	u32 ret;
	struct snd_dmaengine_dai_dma_data *dma_data;

	switch (params_format(params)) {
	/* I2S only support 16Bit, (tdm = 0) == i2s */
	case SNDRV_PCM_FORMAT_S16_LE:
		bits = WOKOO_BITS_16;
		tdm  = 0;
		break;

	case SNDRV_PCM_FORMAT_S20_LE:
		tdm  = 1;
		bits = WOKOO_BITS_20;
		break;

	case SNDRV_PCM_FORMAT_S24_LE:
		tdm  = 1;
		bits = WOKOO_BITS_24;
		break;

	default:
		dev_err(cpu_dai->dev, "Bad format\n");
		return -EINVAL;
	}


	rate    = params_rate(params);
	channel = params_channels(params);

	if (!channel) {
		dev_err(cpu_dai->dev, "Bad channels\n");
		return -EINVAL;
	}

	if (channel > 6 && rate > 96000) {
		dev_err(cpu_dai->dev, "Unsupport channel > 6 or rate > 96K\n");
		return -EINVAL;
	} else if (channel >= 8 && rate > 48000) {
		dev_err(cpu_dai->dev, "Unsupport channel == 8 or rate > 48K\n");
		return -EINVAL;
	}

	switch (rate) {
		case 8000:
		case 16000:
		case 32000:
		case 48000:
		case 64000:
		case 96000:
		case 192000:
			break;

		case 11025:
		case 22050:
		case 44100:
		case 88200:
		case 176400:
			break;

		default:
			dev_err(cpu_dai->dev, "Bad rate: %d\n", params_rate(params));
			return -EINVAL;
	}

	/* Set GR clock to parent */
	clk_set_rate(i2s->clk[CLK_I2S_GR_CLK], clk_get_rate(clk_get_parent(i2s->clk[CLK_I2S_GR_CLK])));

	i2s->channels = channel;

	/* i2s_mclk >= Fsclk * 6, so set to 8 */
	sclk_div = 8;
	/* Fsclk = Fsync * CHANNEL_NUM * 32 */
	fsyn_div = 32;
	i2s_sclk = fsyn_div * rate * sclk_div * channel;
	clk_set_rate(i2s->clk[CLK_I2S_SCLK], i2s_sclk);

	dma_data = snd_soc_dai_get_dma_data(cpu_dai, substream);

	/* Set i2s_sclk division factor */
	writel(sclk_div, &i2s->regs->SCLK_CFG);

	/* Set channel */
	val  = readl(&i2s->regs->FSYNC_CFG);
	val &= ~I2S_FSYNC_CFG_CHANNEL_NUM_MSK;
	val |= (channel - 1) << I2S_FSYNC_CFG_CHANNEL_NUM_POS;

	/* Set bits */
	val &= ~I2S_FSYNC_CFG_FSYNC_BIT_MSK;
	val |= bits << I2S_FSYNC_CFG_FSYNC_BIT_POS;

	/* Set fsync */
	val &= ~I2S_FSYNC_CFG_FSYNC_DIV_MSK;
	val |= fsyn_div << I2S_FSYNC_CFG_FSYNC_DIV_POS;
	writel(val, &i2s->regs->FSYNC_CFG);

	val  = readl(&i2s->regs->MODE);
	val &= ~I2S_MODE_SLAVE_EN;
	val &= ~I2S_MODE_I2S_MODE;

	if (tdm == 1) {
		val |= I2S_MODE_I2S_MODE;
	}

	switch (i2s->format & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		i2s->master = false;
		val |= I2S_MODE_SLAVE_EN;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		i2s->master = true;
		break;
	default:
		WARN_ONCE(1, "Invalid i2s->fmt MASTER_MASK. This shouldn't happen\n");
		return -EINVAL;
	}

	val &= ~I2S_MODE_REC_EN;
	val &= ~I2S_MODE_TRAN_EN;

	writel(val, &i2s->regs->MODE);
	if (!i2s->master)
		return 0;

	ret = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(params));
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int wokoo_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			  struct snd_soc_dai *cpu_dai)
{
	struct wokoo_i2s *i2s = dev_get_drvdata(cpu_dai->dev);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			rcu_assign_pointer(i2s->rx_substream, substream);
			WRITE_ONCE(i2s->rx_ptr, 0);
			wokoo_i2s_rxctrl(cpu_dai, 1);
			wokoo_i2s_enable_capture_irq(i2s);
		} else {
			rcu_assign_pointer(i2s->tx_substream, substream);
			WRITE_ONCE(i2s->tx_ptr, 0);
			wokoo_i2s_txctrl(cpu_dai, 1);
			wokoo_i2s_enable_play_irq(i2s);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			rcu_assign_pointer(i2s->rx_substream, NULL);
			wokoo_i2s_rxctrl(cpu_dai, 0);
		} else {
			rcu_assign_pointer(i2s->tx_substream, NULL);
			wokoo_i2s_txctrl(cpu_dai, 0);
		}

		wokoo_i2s_disable_irq(i2s);
		break;
	default:
		dev_err(cpu_dai->dev, "unknown cmd\n");
		return -EINVAL;
	}
	return 0;
}


static int wokoo_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct wokoo_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,
				  &i2s->dma_data[SNDRV_PCM_STREAM_PLAYBACK],
				  &i2s->dma_data[SNDRV_PCM_STREAM_CAPTURE]);
	return 0;
}

static int wokoo_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct wokoo_i2s *dev = snd_soc_dai_get_drvdata(rtd->cpu_dai);

	snd_soc_set_runtime_hwparams(substream, &wokoo_pcm_hardware);
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	runtime->private_data = dev;

	return 0;
}

static int wokoo_pcm_close(struct snd_pcm_substream *substream)
{
	synchronize_rcu();
	return 0;
}

static int wokoo_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static snd_pcm_uframes_t wokoo_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct wokoo_i2s *dev = runtime->private_data;
	snd_pcm_uframes_t pos;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pos = READ_ONCE(dev->tx_ptr);
	else
		pos = READ_ONCE(dev->rx_ptr);

	return pos < runtime->buffer_size ? pos : 0;
}


static int wokoo_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	size_t size = wokoo_pcm_hardware.buffer_bytes_max;

	snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL), size, size);
	return 0;
}

static void wokoo_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

/*
 * wokoo_i2s_transfer - irq handle for transfer
 */
static void wokoo_i2s_transfer(struct wokoo_i2s *i2s, bool push)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	bool active, period_elapsed;
	uint32_t *p, i;

	rcu_read_lock();
	if (push)
		substream = rcu_dereference(i2s->tx_substream);
	else
		substream = rcu_dereference(i2s->rx_substream);

	runtime = substream->runtime;
	active  = substream && snd_pcm_running(substream);
	if (active) {
		unsigned int ptr, tx_ptr, rx_ptr;
		unsigned int new_ptr;
		unsigned int period_pos;
		unsigned int len;

		/* Just for one channel playback */
		if (i2s->channels == 1 && push) {
			len = runtime->buffer_size >> 1;
		} else {
			len = runtime->buffer_size;
		}

		p = (uint32_t *)runtime->dma_area;
		if (push) {
			ptr        = READ_ONCE(i2s->tx_ptr);
			tx_ptr     = ptr;
			period_pos = i2s->tx_ptr % runtime->period_size;

			for (i = 0; i < WOKOO_I2S_FIFO_TH; i++) {
				writel(p[tx_ptr], &i2s->regs->TRAN_FIFO);
				period_pos += 2;

				if (++tx_ptr >= len)
					tx_ptr = 0;
			}

			new_ptr = tx_ptr;
			/* Renew ptr for playback */
			cmpxchg(&i2s->tx_ptr, ptr, new_ptr);
		} else {
			ptr        = READ_ONCE(i2s->rx_ptr);
			rx_ptr     = ptr;
			period_pos = i2s->rx_ptr % runtime->period_size;

			for (i = 0; i < WOKOO_I2S_FIFO_TH; i++) {
				p[rx_ptr] = readl(&i2s->regs->REC_FIFO);
				period_pos += 2;
				if (++rx_ptr >= len)
					rx_ptr = 0;
			}

			new_ptr = rx_ptr;
			cmpxchg(&i2s->rx_ptr, ptr, new_ptr);
		}

		/* Complete playback or capture */
		period_elapsed = period_pos >= runtime->period_size;
		if (period_elapsed) {
			snd_pcm_period_elapsed(substream);
		}
	}
	rcu_read_unlock();
}

/*
 * wokoo_i2s_push_fifo - irq handle for playback
 */
static void wokoo_i2s_push_fifo(struct wokoo_i2s *i2s)
{
	wokoo_i2s_transfer(i2s, 1);
}

/*
 * wokoo_i2s_pop_fifo - irq handle for capture
 */
static void wokoo_i2s_pop_fifo(struct wokoo_i2s *i2s)
{
	wokoo_i2s_transfer(i2s, 0);
}

/*
 * wokoo_i2s_irq - irq handle for i2s
 */
static irqreturn_t wokoo_i2s_irq(int irq, void *devid)
{
	u32 sta;
	struct wokoo_i2s *i2s = devid;

	sta = readl(&i2s->regs->I2S_IS) & 
		(WOKOO_I2S_PLAY_INT_MASK | WOKOO_I2S_CAP_INT_MASK);

	if (sta == I2S_FIFO_STA_OBUF_AE) {
		wokoo_i2s_push_fifo(i2s);
	}

	if (sta == I2S_FIFO_STA_IBUF_AF) {
		wokoo_i2s_pop_fifo(i2s);
	}

	/* Clear irq */
	if (sta == I2S_FIFO_STA_OBUF_OV) {
		writel(sta | I2S_FIFO_STA_OBUF_OV, &i2s->regs->I2S_IS);
	}

	if (sta == I2S_FIFO_STA_IBUF_RE) {
		writel(sta | I2S_FIFO_STA_IBUF_RE, &i2s->regs->I2S_IS);
	}

	if (sta) {
		return IRQ_HANDLED;
	} else {
		dev_err(i2s->dev, "wokoo_i2s_irq sta = %x\n", readl(&i2s->regs->I2S_IS));
		return IRQ_NONE;
	}
}


static const struct snd_pcm_ops wokoo_pcm_ops = {
	.open = wokoo_pcm_open,
	.close = wokoo_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_free = wokoo_pcm_hw_free,
	.pointer = wokoo_pcm_pointer,
};


static const struct snd_soc_dai_ops wokoo_i2s_dai_ops = {
	.trigger	= wokoo_i2s_trigger,
	.hw_params	= wokoo_i2s_hw_params,
	.set_fmt	= wokoo_i2s_set_fmt,
	.startup	= wokoo_i2s_startup,
	.shutdown	= wokoo_i2s_shutdown,
};

static struct snd_soc_dai_driver wokoo_i2s_dai_init = {
	.name	= "wokoo-i2s-dai",
	.id		= 0,
	.probe	= wokoo_i2s_dai_probe,
	.playback = {
		.channels_min = 1,
		.channels_max = 8,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S20_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
		.rates = SNDRV_PCM_RATE_8000_192000,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 8,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S20_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
		.rates = SNDRV_PCM_RATE_8000_192000,
	},
	.ops = &wokoo_i2s_dai_ops,
};

static const struct snd_soc_component_driver wokoo_i2s_comp = {
	.name = "wokoo-i2s",
};

static const struct snd_soc_component_driver wokoo_pcm_component = {
	.pcm_new = wokoo_pcm_new,
	.pcm_free = wokoo_pcm_free,
	.ops = &wokoo_pcm_ops,
};

#ifdef CONFIG_PM
static int wokoo_i2s_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct wokoo_i2s *i2s = platform_get_drvdata(pdev);

	clk_disable_unprepare(i2s->clk[CLK_I2S_PCLK]);
	return 0;
}

static int wokoo_i2s_resume(struct platform_device *pdev)
{
	int  ret = -1;
	struct wokoo_i2s *i2s = platform_get_drvdata(pdev);

	ret = clk_prepare_enable(i2s->clk[CLK_I2S_PCLK]);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to enable i2s clock: %d\n", ret);
		return ret;
	}

	return ret;
}
#endif

static int wokoo_i2s_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wokoo_i2s *i2s;
	struct resource *res;
	int ret;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);

	if (!i2s)
		return -ENOMEM;


	i2s->dev = dev;
	spin_lock_init(&i2s->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2s->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(i2s->base))
		return PTR_ERR(i2s->base);

	i2s->dai  = wokoo_i2s_dai_init;
	i2s->regs = (struct wokoo_i2s_regs *)i2s->base;
	i2s->base_phys = res->start;

	dev_set_drvdata(&pdev->dev, i2s);

	i2s->clk[CLK_I2S_PCLK] = devm_clk_get(&pdev->dev, "i2s_pclk");
	if (IS_ERR_OR_NULL(i2s->clk[CLK_I2S_PCLK]))
		return PTR_ERR(i2s->clk[CLK_I2S_PCLK]);
	i2s->clocks++;

	ret = clk_prepare_enable(i2s->clk[CLK_I2S_PCLK]);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	i2s->clk[CLK_I2S_SCLK] = devm_clk_get(&pdev->dev, "i2s_sclk");
	if (IS_ERR_OR_NULL(i2s->clk[CLK_I2S_SCLK]))
		return PTR_ERR(i2s->clk[CLK_I2S_SCLK]);
	i2s->clocks++;
	i2s->clk[CLK_I2S_GR_CLK] = devm_clk_get(&pdev->dev, "i2s_gr_clk");
	if (IS_ERR_OR_NULL(i2s->clk[CLK_I2S_GR_CLK]))
		return PTR_ERR(i2s->clk[CLK_I2S_GR_CLK]);
	i2s->clocks++;

	ret = devm_snd_soc_register_component(&pdev->dev, &wokoo_i2s_comp,
					 &wokoo_i2s_dai_init, 1);

	ret = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, ret,
					wokoo_i2s_irq, 0,
					dev_name(&pdev->dev), i2s);

	if (ret) {
		dev_err(&pdev->dev,
			"request_irq failed with err %d \n", ret);
		return ret;
	}

	devm_snd_soc_register_component(&pdev->dev, &wokoo_pcm_component,
					       NULL, 0);
	return ret;
}

static const struct of_device_id wokoo_i2s_dt_ids[] = {
	{ .compatible = "wokoo,wokoo-i2s" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, wokoo_i2s_dt_ids);

static struct platform_driver wokoo_i2s_driver = {
	.probe   = wokoo_i2s_probe,
#ifdef CONFIG_PM
	.suspend = wokoo_i2s_suspend,
	.resume  = wokoo_i2s_resume,
#endif
	.driver = {
		.name = "wokoo-i2s",
		.of_match_table = wokoo_i2s_dt_ids,
	},
};

module_platform_driver(wokoo_i2s_driver);

MODULE_DESCRIPTION("Codefair wokoo I2S driver");
MODULE_LICENSE("GPL");

