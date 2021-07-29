/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#ifndef _WOKOO_CSI_H_
#define _WOKOO_CSI_H_


#define CSI_EN_REG			0x00

#define CSI_CFG_REG			0x04
#define CSI_CFG_INPUT_FMT(fmt)			((fmt) << 20)
#define CSI_CFG_OUTPUT_FMT(fmt)			((fmt) << 16)
#define CSI_CFG_YUV_DATA_SEQ(seq)		((seq) << 8)
#define CSI_CFG_VSYNC_POL(pol)			((pol) << 2)
#define CSI_CFG_HSYNC_POL(pol)			((pol) << 1)
#define CSI_CFG_PCLK_POL(pol)			((pol) << 0)

#define CSI_CPT_CTRL_REG		0x08
#define CSI_CPT_CTRL_VIDEO_START		BIT(1)
#define CSI_CPT_CTRL_IMAGE_START		BIT(0)

#define CSI_BUF_ADDR_REG(fifo, buf)	(0x10 + (0x8 * (fifo)) + (0x4 * (buf)))

#define CSI_BUF_CTRL_REG		0x28
#define CSI_BUF_CTRL_DBN			BIT(2)
#define CSI_BUF_CTRL_DBS			BIT(1)
#define CSI_BUF_CTRL_DBE			BIT(0)

#define CSI_INT_EN_REG			0x30
#define CSI_INT_FRM_DONE			BIT(1)
#define CSI_INT_CPT_DONE			BIT(0)

#define CSI_INT_STA_REG			0x34

#define CSI_WIN_CTRL_W_REG		0x40
#define CSI_WIN_CTRL_W_ACTIVE(w)		((w) << 16)

#define CSI_WIN_CTRL_H_REG		0x44
#define CSI_WIN_CTRL_H_ACTIVE(h)		((h) << 16)

#define CSI_BUF_LEN_REG			0x48

#define CSI_MAX_BUFFER		2
#define CSI_MAX_HEIGHT		8192U
#define CSI_MAX_WIDTH		8192U

enum csi_input {
	CSI_INPUT_RAW	= 0,
	CSI_INPUT_BT656	= 2,
	CSI_INPUT_YUV	= 3,
};

enum csi_output_raw {
	CSI_OUTPUT_RAW_PASSTHROUGH = 0,
};

enum csi_output_yuv {
	CSI_OUTPUT_YUV_422_PLANAR	= 0,
	CSI_OUTPUT_YUV_420_PLANAR	= 1,
	CSI_OUTPUT_YUV_422_UV		= 4,
	CSI_OUTPUT_YUV_420_UV		= 5,
	CSI_OUTPUT_YUV_422_MACRO	= 8,
	CSI_OUTPUT_YUV_420_MACRO	= 9,
};

enum csi_yuv_data_seq {
	CSI_YUV_DATA_SEQ_YUYV	= 0,
	CSI_YUV_DATA_SEQ_YVYU	= 1,
	CSI_YUV_DATA_SEQ_UYVY	= 2,
	CSI_YUV_DATA_SEQ_VYUY	= 3,
};

enum csi_subdev_pads {
	CSI_SUBDEV_SINK,
	CSI_SUBDEV_SOURCE,

	CSI_SUBDEV_PADS,
};

extern const struct v4l2_subdev_ops wokoo_csi_subdev_ops;

struct wokoo_csi_format {
	u32			mbus;
	u32			fourcc;
	enum csi_input		input;
	u32			output;
	unsigned int		num_planes;
	u8			bpp[3];
	unsigned int		hsub;
	unsigned int		vsub;
};

const struct wokoo_csi_format *wokoo_csi_find_format(const u32 *fourcc,const u32 *mbus);

struct wokoo_csi {
	/* Device resources */
	dev_t devid;			/*  	 */
	struct cdev IspDevs;		/* cdev 	*/
	struct class *class;	/*  		*/
	void *private_data;
	const char *type;
	const char *name;
	const char *memsize;
	// struct device *device;	/* 	 */
	// struct device_node	*nd; /*  */
	int major;
	struct device			*dev;

	void __iomem			*regs;
	void __iomem			*pwr_regs;	
	void __iomem			*ctl_regs;

	struct clk			*bus_clk;
	struct clk			*isp_clk;
	struct clk			*ram_clk;
	
	struct clk			*isp_axi_clk;
	struct clk			*isp_psclk_sclk2;
	struct clk			*cphy_cfg_clk;
	struct clk			*lvds_idsp_sclk;
	struct clk			*isp_hclk;
	struct clk			*isp_p_sclk;
	struct clk			*pclk;
	struct clk			*clk_out_bus;
	struct clk			*clk_out_bus_1;

	struct reset_control		*rsta;
	struct reset_control		*rstp;
	dma_addr_t			tx_bb_dma_addr;
	struct {
		size_t			size;
		void			*vaddr;
		dma_addr_t		paddr;
	} scratch;
	struct fasync_struct *async_isp;
	struct mutex		lock;
	spinlock_t			qlock;
	unsigned int		sequence;
	bool                issup;
	unsigned int        ispdataflag;
	unsigned int        ispint;
	unsigned int        stype;
};

int wokoo_csi_dma_register(struct wokoo_csi *csi, int irq);
void wokoo_csi_dma_unregister(struct wokoo_csi *csi);
#endif /* _WOKOO_CSI_H_ */
