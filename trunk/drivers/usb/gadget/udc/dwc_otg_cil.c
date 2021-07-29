
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include "dwc_otg_cil.h"

#include "wokoo_usb.h"

#define log_out printk

dwc_otg_core_if_t g_core_if;
dwc_otg_dev_if_t g_dev_if;
dwc_otg_host_if_t g_host_if;
dwc_otg_core_params_t g_core_params;

uint8_t dma_enable = 0;

static int dwc_otg_setup_params(dwc_otg_core_if_t * core_if);

/**
 * This function initializes the commmon interrupts, used in both
 * device and host modes.
 *
 * @param core_if Programming view of the DWC_otg controller
 *
 */
static void dwc_otg_enable_common_interrupts(dwc_otg_core_if_t * core_if) {
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0;

	/* Clear any pending OTG Interrupts */
	dwc_write_reg32(&global_regs->gotgint, 0xFFFFFFFF);

	/* Clear any pending interrupts */
	dwc_write_reg32(&global_regs->gintsts, 0xFFFFFFFF);

	/*
	 * Enable the interrupts in the GINTMSK.
	 */
	intr_mask.b.modemismatch = 1;
	intr_mask.b.otgintr = 1;

	if (!core_if->dma_enable)
	{
		intr_mask.b.rxstsqlvl = 1;
	}

	intr_mask.b.conidstschng = 1;
	intr_mask.b.wkupintr = 1;
	intr_mask.b.disconnect = 1;
	intr_mask.b.usbsuspend = 1;
	intr_mask.b.sessreqintr = 1;

	dwc_write_reg32(&global_regs->gintmsk, intr_mask.d32);
}

dwc_otg_core_if_t *dwc_otg_cil_init(const uint32_t * reg_base_addr) {
	dwc_otg_core_if_t *core_if = 0;
	dwc_otg_dev_if_t *dev_if = 0;
	dwc_otg_host_if_t *host_if = 0;
	uint8_t *reg_base = (uint8_t *) reg_base_addr;
	int i = 0;

	core_if = &g_core_if;
	core_if->phy_init_done = 0;
	core_if->core_global_regs = (dwc_otg_core_global_regs_t *) reg_base;

	/*
	 * Allocate the Device Mode structures.
	 */
	dev_if = &g_dev_if;
	dev_if->dev_global_regs = (dwc_otg_device_global_regs_t *)(reg_base + DWC_DEV_GLOBAL_REG_OFFSET);

	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		dev_if->in_ep_regs[i] = (dwc_otg_dev_in_ep_regs_t *)(reg_base + DWC_DEV_IN_EP_REG_OFFSET + (i * DWC_EP_REG_OFFSET));
		dev_if->out_ep_regs[i] = (dwc_otg_dev_out_ep_regs_t *)(reg_base + DWC_DEV_OUT_EP_REG_OFFSET + (i * DWC_EP_REG_OFFSET));
	}

	dev_if->speed = 0;
	core_if->dev_if = dev_if;

	/*
	 * Allocate the Host Mode structures.
	 */
	host_if = &g_host_if;

	host_if->host_global_regs = (dwc_otg_host_global_regs_t *)(reg_base + DWC_OTG_HOST_GLOBAL_REG_OFFSET);

	host_if->hprt0 = (uint32_t *)(reg_base + DWC_OTG_HOST_PORT_REGS_OFFSET);

	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		host_if->hc_regs[i] = (dwc_otg_hc_regs_t *)(reg_base + DWC_OTG_HOST_CHAN_REGS_OFFSET + (i * DWC_OTG_CHAN_REGS_OFFSET));
	}

	host_if->num_host_channels = MAX_EPS_CHANNELS;
	core_if->host_if = host_if;

	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		core_if->data_fifo[i] = (uint32_t *)(reg_base + DWC_OTG_DATA_FIFO_OFFSET + (i * DWC_OTG_DATA_FIFO_SIZE));
	}
	core_if->pcgcctl = (uint32_t *)(reg_base + DWC_OTG_PCGCCTL_OFFSET);

	/* Initiate lx_state to L3 disconnected state */
	core_if->lx_state = DWC_OTG_L3;

	/*
	 * Store the contents of the hardware configuration registers here for
	 * easy access later.
	 */
	core_if->hwcfg1.d32 = dwc_read_reg32(&core_if->core_global_regs->ghwcfg1);
	core_if->hwcfg2.d32 = dwc_read_reg32(&core_if->core_global_regs->ghwcfg2);
	core_if->hwcfg3.d32 = dwc_read_reg32(&core_if->core_global_regs->ghwcfg3);
	core_if->hwcfg4.d32 = dwc_read_reg32(&core_if->core_global_regs->ghwcfg4);
	core_if->hcfg.d32 = dwc_read_reg32(&core_if->host_if->host_global_regs->hcfg);
	core_if->dcfg.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dcfg);

	/*
	 * Set the SRP sucess bit for FS-I2c
	 */
	core_if->srp_success = 0;
	core_if->srp_timer_started = 0;

	core_if->snpsid = dwc_read_reg32(&core_if->core_global_regs->gsnpsid);

	dwc_otg_setup_params(core_if);

	core_if->hibernation_suspend = 0;

	/** ADP initialization */

	return core_if;

}

static void dwc_otg_set_uninitialized(int32_t * p, int size) {
	int i;
	for (i = 0; i < size; i++) {
		p[i] = -1;
	}
}

/*
static int dwc_otg_param_initialized(int32_t val) {
	return val != -1;
}
*/

/**
 * Register PCD callbacks. The callbacks are used to start and stop
 * the PCD for interrupt processing.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param cb the PCD callback structure.
 * @param p pointer to be passed to callback function (pcd*).
 */
void dwc_otg_cil_register_pcd_callbacks(dwc_otg_core_if_t * core_if,
		dwc_otg_cil_callbacks_t * cb, void *p) {
	core_if->pcd_cb = cb;
	cb->p = p;
}

static int dwc_otg_setup_params(dwc_otg_core_if_t * core_if) {

	int i;
	core_if->core_params = &g_core_params;

	dwc_otg_set_uninitialized((int32_t *) core_if->core_params,	sizeof(*core_if->core_params) / sizeof(int32_t));

	core_if->core_params->otg_cap = 2;
	core_if->core_params->dma_enable = 0;
	core_if->core_params->dma_desc_enable = 0;
	core_if->core_params->opt = dwc_param_opt_default;
	core_if->core_params->dma_burst_size = dwc_param_dma_burst_size_default;
	core_if->core_params->host_support_fs_ls_low_power = dwc_param_host_support_fs_ls_low_power_default;
	core_if->core_params->enable_dynamic_fifo = 0;
	core_if->core_params->data_fifo_size = dwc_param_data_fifo_size_default;
	core_if->core_params->dev_rx_fifo_size = dwc_param_dev_rx_fifo_size_default;
	core_if->core_params->dev_nperio_tx_fifo_size = dwc_param_dev_nperio_tx_fifo_size_default;
	core_if->core_params->host_rx_fifo_size = dwc_param_host_rx_fifo_size_default;
	core_if->core_params->host_nperio_tx_fifo_size = dwc_param_host_nperio_tx_fifo_size_default;
	core_if->core_params->host_perio_tx_fifo_size = dwc_param_host_perio_tx_fifo_size_default;
	core_if->core_params->max_transfer_size = dwc_param_max_transfer_size_default;
	core_if->core_params->max_packet_count = dwc_param_max_packet_count_default;
	core_if->core_params->host_channels = 4;
	core_if->core_params->dev_endpoints = 4;
	core_if->core_params->phy_type = dwc_param_phy_type_default;
	core_if->core_params->speed = dwc_param_speed_default;
	core_if->core_params->host_ls_low_power_phy_clk = dwc_param_host_ls_low_power_phy_clk_default;
	core_if->core_params->phy_ulpi_ddr = dwc_param_phy_ulpi_ddr_default;
	core_if->core_params->phy_ulpi_ext_vbus = dwc_param_phy_ulpi_ext_vbus_default;
	core_if->core_params->phy_utmi_width = dwc_param_phy_utmi_width_default;
	core_if->core_params->ulpi_fs_ls = dwc_param_ulpi_fs_ls_default;
	core_if->core_params->ts_dline = dwc_param_ts_dline_default;
	core_if->core_params->i2c_enable = dwc_param_i2c_enable_default;
	core_if->core_params->en_multiple_tx_fifo = 0;

	for (i = 0; i < 4; i++) {
		core_if->core_params->dev_perio_tx_fifo_size[i] = dwc_param_dev_perio_tx_fifo_size_default;
	}
	for (i = 0; i < 4; i++) {
		core_if->core_params->dev_tx_fifo_size[i] = dwc_param_dev_tx_fifo_size_default;
	}
	core_if->core_params->thr_ctl = dwc_param_thr_ctl_default;

	core_if->core_params->mpi_enable = 0;
	core_if->core_params->pti_enable = dwc_param_pti_enable_default;
	core_if->core_params->lpm_enable = 0;
	core_if->core_params->adp_supp_enable = dwc_param_adp_enable_default;
	core_if->core_params->ic_usb_cap = dwc_param_ic_usb_cap_default;
	core_if->core_params->ahb_thr_ratio = dwc_param_ahb_thr_ratio_default;
	core_if->core_params->power_down = dwc_param_power_down_default;
	core_if->core_params->otg_ver = dwc_param_otg_ver_default;
	core_if->core_params->tx_thr_length = dwc_param_tx_thr_length_default;
	core_if->core_params->rx_thr_length = dwc_param_rx_thr_length_default;

	return 0;
}

/**
 * This function enables the controller's Global Interrupt in the AHB Config
 * register.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_enable_global_interrupts(dwc_otg_core_if_t * core_if) {
	gahbcfg_data_t ahbcfg;
	ahbcfg.d32 = 0;
	ahbcfg.b.glblintrmsk = 1; /* Enable interrupts */
	dwc_modify_reg32(&core_if->core_global_regs->gahbcfg, 0, ahbcfg.d32);
}

/**
 * This function disables the controller's Global Interrupt in the AHB Config
 * register.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_disable_global_interrupts(dwc_otg_core_if_t * core_if) {
	gahbcfg_data_t ahbcfg;
	ahbcfg.d32 = 0;
	ahbcfg.b.glblintrmsk = 1; /* Enable interrupts */
	dwc_modify_reg32(&core_if->core_global_regs->gahbcfg, ahbcfg.d32, 0);
}

/**
 * Initializes the DevSpd field of the DCFG register depending on the PHY type
 * and the enumeration speed of the device.
 */
static void init_devspd(dwc_otg_core_if_t * core_if) {
	uint32_t val;
	dcfg_data_t dcfg;

	if (((core_if->hwcfg2.b.hs_phy_type == 2)
			&& (core_if->hwcfg2.b.fs_phy_type == 1)
			&& (core_if->core_params->ulpi_fs_ls))
			|| (core_if->core_params->phy_type == DWC_PHY_TYPE_PARAM_FS)) {
		/* Full speed PHY */
		printk("full speed phy\n");
		val = 0x3;
	} else if (core_if->core_params->speed == DWC_SPEED_PARAM_FULL) {
		/* High speed PHY running at full speed */
		printk("high speed phy full spd\n");
		val = 0x1;
	} else {
		/* High speed PHY running at high speed */
		printk("high speed phy high spd\n");
		val = 0x0;
	}
	val = 0x1;
	dcfg.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dcfg);
	dcfg.b.devspd = val;
	dwc_write_reg32(&core_if->dev_if->dev_global_regs->dcfg, dcfg.d32);
}

#if 0
/**
 * This function calculates the number of IN EPS
 * using GHWCFG1 and GHWCFG2 registers values
 *
 * @param core_if Programming view of the DWC_otg controller
 */
static uint32_t calc_num_in_eps(dwc_otg_core_if_t * core_if) {
	uint32_t num_in_eps = 0;
	uint32_t num_eps = core_if->hwcfg2.b.num_dev_ep;
	uint32_t hwcfg1 = core_if->hwcfg1.d32 >> 3;
	uint32_t num_tx_fifos = core_if->hwcfg4.b.num_in_eps;
	int i;

	for (i = 0; i < num_eps; ++i) {
		if (!(hwcfg1 & 0x1))
			num_in_eps++;
		hwcfg1 >>= 2;
	}

	if (core_if->hwcfg4.b.ded_fifo_en) {
		num_in_eps = (num_in_eps > num_tx_fifos) ? num_tx_fifos : num_in_eps;
	}

	return num_in_eps;
}

/**
 * This function calculates the number of OUT EPS
 * using GHWCFG1 and GHWCFG2 registers values
 *
 * @param core_if Programming view of the DWC_otg controller
 */
static uint32_t calc_num_out_eps(dwc_otg_core_if_t * core_if) {
	uint32_t num_out_eps = 0;
	uint32_t num_eps = core_if->hwcfg2.b.num_dev_ep;
	uint32_t hwcfg1 = core_if->hwcfg1.d32 >> 2;
	int i;

	for (i = 0; i < num_eps; ++i) {
		if (!(hwcfg1 & 0x1))
			num_out_eps++;
		hwcfg1 >>= 2;
	}
	return num_out_eps;
}
#endif

/**
 * This function initializes the DWC_otg controller registers and
 * prepares the core for device mode or host mode operation.
 *
 * @param core_if Programming view of the DWC_otg controller
 *
 */
void dwc_otg_core_init(dwc_otg_core_if_t * core_if) {
	int i = 0;
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	gahbcfg_data_t ahbcfg;
	gusbcfg_data_t usbcfg;
	uint8_t brst_sz = 0;

	ahbcfg.d32 = 0;
	usbcfg.d32 = 0;

	/* Common Initialization */
	usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);

	/* Program the ULPI External VBUS bit if needed */
	usbcfg.b.ulpi_ext_vbus_drv = (core_if->core_params->phy_ulpi_ext_vbus == DWC_PHY_ULPI_EXTERNAL_VBUS) ? 1 : 0;

	/* Set external TS Dline pulsing */
	usbcfg.b.term_sel_dl_pulse =  0;
	dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

	/* Reset the Controller */
	dwc_otg_core_reset(core_if);

	core_if->adp_enable = core_if->core_params->adp_supp_enable;
	core_if->power_down = core_if->core_params->power_down;

	/* Initialize parameters from Hardware configuration registers. */
	dev_if->num_in_eps = 3;//calc_num_in_eps(core_if);
	dev_if->num_out_eps = 3;//calc_num_out_eps(core_if);

	for (i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++) {
		dev_if->perio_tx_fifo_size[i] = dwc_read_reg32(&global_regs->dtxfsiz[i]) >> 16;
	}

	for (i = 0; i < core_if->hwcfg4.b.num_in_eps; i++) {
		dev_if->tx_fifo_size[i] = dwc_read_reg32(&global_regs->dtxfsiz[i]) >> 16;
	}

	core_if->total_fifo_size = core_if->hwcfg3.b.dfifo_depth;
	core_if->rx_fifo_size = dwc_read_reg32(&global_regs->grxfsiz);
	core_if->nperio_tx_fifo_size =
	dwc_read_reg32(&global_regs->gnptxfsiz) >> 16;

	/* This programming sequence needs to happen in FS mode before any other
	 * programming occurs */

	/* High speed PHY. */
	if (!core_if->phy_init_done) {
		core_if->phy_init_done = 1;
		/* HS PHY parameters.  These parameters are preserved
		 * during soft reset so only program the first time.  Do
		 * a soft reset immediately after setting phyif.  */
		if (core_if->core_params->phy_type == 2) {
			/* ULPI interface */
			usbcfg.b.ulpi_utmi_sel = 1;
			usbcfg.b.phyif = 0;
			usbcfg.b.ddrsel = core_if->core_params->phy_ulpi_ddr;
		} else if (core_if->core_params->phy_type == 1) {
			/* UTMI+ interface */
			usbcfg.b.ulpi_utmi_sel = 0;
			if (core_if->core_params->phy_utmi_width == 16) {
				usbcfg.b.phyif = 1;
			} else {
				usbcfg.b.phyif = 0;
			}
		} else {
		}
		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);
		/* Reset after setting the PHY parameters */
		dwc_otg_core_reset(core_if);
	}

	if ((core_if->hwcfg2.b.hs_phy_type == 2)
			&& (core_if->hwcfg2.b.fs_phy_type == 1)
			&& (core_if->core_params->ulpi_fs_ls)) {
		usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
		usbcfg.b.ulpi_fsls = 1;
		usbcfg.b.ulpi_clk_sus_m = 1;
		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);
	} else {
		usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
		usbcfg.b.ulpi_fsls = 0;
		usbcfg.b.ulpi_clk_sus_m = 0;
		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);
	}

	/* Program the GAHBCFG Register. */
	switch (core_if->hwcfg2.b.architecture) {
	case DWC_SLAVE_ONLY_ARCH:
        printk("DWC_SLAVE_ONLY_ARCH\n");
		ahbcfg.b.nptxfemplvl_txfemplvl = DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;
		ahbcfg.b.ptxfemplvl = DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;
		core_if->dma_enable = 0;
		core_if->dma_desc_enable = 0;
		break;

	case DWC_EXT_DMA_ARCH:
        printk("DWC_EXT_DMA_ARCH\n");
		brst_sz = core_if->core_params->dma_burst_size;
		ahbcfg.b.hburstlen = 0;
		while (brst_sz > 1) {
			ahbcfg.b.hburstlen++;
			brst_sz >>= 1;
		}

		core_if->dma_enable = (core_if->core_params->dma_enable != 0);
		core_if->dma_desc_enable = (core_if->core_params->dma_desc_enable != 0);
		break;

	case DWC_INT_DMA_ARCH:
        printk("DWC_INT_DMA_ARCH\n");
		ahbcfg.b.hburstlen = DWC_GAHBCFG_INT_DMA_BURST_INCR;
		core_if->dma_enable = dma_enable;
		core_if->dma_desc_enable = 0;
		break;
	}

	if (core_if->dma_enable) {
		if (core_if->dma_desc_enable) {
		}
	} else
		core_if->dma_desc_enable = 0;

	ahbcfg.b.dmaenable = core_if->dma_enable;
	dwc_write_reg32(&global_regs->gahbcfg, ahbcfg.d32);

	core_if->en_multiple_tx_fifo = core_if->hwcfg4.b.ded_fifo_en;

	core_if->pti_enh_enable = core_if->core_params->pti_enable != 0;
	core_if->multiproc_int_enable = core_if->core_params->mpi_enable;

	/*
	 * Program the GUSBCFG register.
	 */
	usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);

	switch (core_if->hwcfg2.b.op_mode) {
	case DWC_MODE_HNP_SRP_CAPABLE:
		usbcfg.b.hnpcap = (core_if->core_params->otg_cap == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE);
		usbcfg.b.srpcap = (core_if->core_params->otg_cap != DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;

	case DWC_MODE_SRP_ONLY_CAPABLE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap != DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;

	case DWC_MODE_NO_HNP_SRP_CAPABLE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;

	case DWC_MODE_SRP_CAPABLE_DEVICE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap != DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;

	case DWC_MODE_NO_SRP_CAPABLE_DEVICE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;

	case DWC_MODE_SRP_CAPABLE_HOST:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap != DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;

	case DWC_MODE_NO_SRP_CAPABLE_HOST:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;
	}

	dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

	if (core_if->core_params->ic_usb_cap) {
		gusbcfg_data_t gusbcfg;
		gusbcfg.d32 = 0;
		gusbcfg.b.ic_usb_cap = 1;
		dwc_modify_reg32(&core_if->core_global_regs->gusbcfg, 0, gusbcfg.d32);
	}
	{
		gotgctl_data_t gotgctl;
		gotgctl.d32 = 0;
		gotgctl.b.otgver = core_if->core_params->otg_ver;
		dwc_modify_reg32(&core_if->core_global_regs->gotgctl, 0, gotgctl.d32);
		/* Set OTG version supported */
		core_if->otg_ver = core_if->core_params->otg_ver;
	}

	/* Enable common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

	/* Do device or host intialization based on mode during PCD
	 * and HCD initialization  */
	if (dwc_otg_mode(core_if) == DWC_HOST_MODE) {
		printk("core host mode\n");
		core_if->op_state = A_HOST;
	} else {
		printk("core device mode\n");
		core_if->op_state = B_PERIPHERAL;
	}
}

/**
 * Flush a Tx FIFO.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param num Tx FIFO to flush.
 */
void dwc_otg_flush_tx_fifo(dwc_otg_core_if_t * core_if, const int num) {
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset;
	int count = 0;
	greset.d32 = 0;

	greset.b.txfflsh = 1;
	greset.b.txfnum = num;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);

	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 10000) {
			break;
		}
		dwc_udelay(1);
	} while (greset.b.txfflsh == 1);

	/* Wait for 3 PHY Clocks */
	dwc_udelay(1);
}

/**
 * Flush Rx FIFO.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_flush_rx_fifo(dwc_otg_core_if_t * core_if) {
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset;
	int count = 0;
	greset.d32 = 0;

	greset.b.rxfflsh = 1;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);

	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 10000) {
			break;
		}
		dwc_udelay(1);
	} while (greset.b.rxfflsh == 1);

	/* Wait for 3 PHY Clocks */
	dwc_udelay(1);
}

/**
 * Do core a soft reset of the core.  Be careful with this because it
 * resets all the internal state machines of the core.
 */
void dwc_otg_core_reset(dwc_otg_core_if_t * core_if) {
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset;
	int count = 0;
	greset.d32 = 0;

	/* Wait for AHB master IDLE state. */
	/* Core Soft Reset */
	count = 0;
	greset.b.csftrst = 1;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);
	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 10000) {
			break;
		}
		dwc_udelay(1);
	}
	while (greset.b.csftrst == 1);
	/* Core Soft Reset */

	/* Wait for 3 PHY Clocks */
	dwc_mdelay(100);
}

/**
 * This function enables the Device mode interrupts.
 *
 * @param core_if Programming view of DWC_otg controller
 */
void dwc_otg_enable_device_interrupts(dwc_otg_core_if_t * core_if) {
	gintmsk_data_t intr_mask;
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;

	intr_mask.d32 = 0;

	/* Disable all interrupts. */
	dwc_write_reg32(&global_regs->gintmsk, 0);

	/* Clear any pending interrupts */
	dwc_write_reg32(&global_regs->gintsts, 0xFFFFFFFF);

	/* Enable the common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

	/* Enable interrupts */
	intr_mask.b.usbreset = 1;
	intr_mask.b.enumdone = 1;

	if (!core_if->multiproc_int_enable) {
		intr_mask.b.inepintr = 1;
		intr_mask.b.outepintr = 1;
	}

	intr_mask.b.erlysuspend = 1;

	if (core_if->en_multiple_tx_fifo == 0) {
		intr_mask.b.epmismatch = 1;
	}

	/* Enable the ignore frame number for ISOC xfers - MAS */

	dwc_modify_reg32(&global_regs->gintmsk, intr_mask.d32, intr_mask.d32);
}

/**
 * This function initializes the DWC_otg controller registers for
 * device mode.
 *
 * @param core_if Programming view of DWC_otg controller
 *
 */
void dwc_otg_core_dev_init(dwc_otg_core_if_t * core_if) {
	int i;
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	dwc_otg_core_params_t *params = core_if->core_params;
	dcfg_data_t dcfg;
	grstctl_t resetctl;
	uint32_t rx_fifo_size;
	fifosize_data_t nptxfifosize;
	fifosize_data_t txfifosize;
	dthrctl_data_t dthrctl;
	fifosize_data_t ptxfifosize;
	uint16_t rxfsiz, nptxfsiz;
	gdfifocfg_data_t gdfifocfg;
//	hwcfg3_data_t hwcfg3;
	dcfg.d32 = 0;
	resetctl.d32 = 0;
	gdfifocfg.d32 = 0;
//	hwcfg3.d32 = 0;
	/* Restart the Phy Clock */
	dwc_write_reg32(core_if->pcgcctl, 0);

	/* Device configuration register */
	init_devspd(core_if);
	dcfg.d32 = dwc_read_reg32(&dev_if->dev_global_regs->dcfg);
	dcfg.b.descdma = (core_if->dma_desc_enable) ? 1 : 0;
	dcfg.b.perfrint = DWC_DCFG_FRAME_INTERVAL_80;

	dwc_write_reg32(&dev_if->dev_global_regs->dcfg, dcfg.d32);

	/* Configure data FIFO sizes */
	if (core_if->hwcfg2.b.dynamic_fifo && params->enable_dynamic_fifo) {

		/* Rx FIFO */

		rx_fifo_size = params->dev_rx_fifo_size;
		dwc_write_reg32(&global_regs->grxfsiz, rx_fifo_size);

		/** Set Periodic Tx FIFO Mask all bits 0 */
		core_if->p_tx_msk = 0;

		/** Set Tx FIFO Mask all bits 0 */
		core_if->tx_msk = 0;

		if (core_if->en_multiple_tx_fifo == 0) {

			/* Non-periodic Tx FIFO */
			nptxfifosize.b.depth = params->dev_nperio_tx_fifo_size;
			nptxfifosize.b.startaddr = params->dev_rx_fifo_size;

			dwc_write_reg32(&global_regs->gnptxfsiz, nptxfifosize.d32);

			/**@todo NGS: Fix Periodic FIFO Sizing! */
			/*
			 * Periodic Tx FIFOs These FIFOs are numbered from 1 to 15.
			 * Indexes of the FIFO size module parameters in the
			 * dev_perio_tx_fifo_size array and the FIFO size registers in
			 * the dptxfsiz array run from 0 to 14.
			 */
			/** @todo Finish debug of this */
			ptxfifosize.b.startaddr = nptxfifosize.b.startaddr
					+ nptxfifosize.b.depth;
			for (i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++) {
				ptxfifosize.b.depth = params->dev_perio_tx_fifo_size[i];
				dwc_write_reg32(&global_regs->dtxfsiz[i], ptxfifosize.d32);
				ptxfifosize.b.startaddr += ptxfifosize.b.depth;
			}
		} else {

			/*
			 * Tx FIFOs These FIFOs are numbered from 1 to 15.
			 * Indexes of the FIFO size module parameters in the
			 * dev_tx_fifo_size array and the FIFO size registers in
			 * the dtxfsiz array run from 0 to 14.
			 */

			/* Non-periodic Tx FIFO */

			nptxfifosize.b.depth = params->dev_nperio_tx_fifo_size;
			nptxfifosize.b.startaddr = params->dev_rx_fifo_size;

			dwc_write_reg32(&global_regs->gnptxfsiz, nptxfifosize.d32);

			txfifosize.b.startaddr = nptxfifosize.b.startaddr
					+ nptxfifosize.b.depth;

			for (i = 0; i < core_if->hwcfg4.b.num_in_eps; i++) {

				txfifosize.b.depth = params->dev_tx_fifo_size[i];

				dwc_write_reg32(&global_regs->dtxfsiz[i], txfifosize.d32);
				txfifosize.b.startaddr += txfifosize.b.depth;
			}
		}
	}

	/* Calculating DFIFOCFG for Device mode to include RxFIFO and NPTXFIFO*/
	gdfifocfg.d32 = dwc_read_reg32(&global_regs->gdfifocfg);
//	hwcfg3.d32 = dwc_read_reg32(&global_regs->ghwcfg3);
	gdfifocfg.b.gdfifocfg = (dwc_read_reg32(&global_regs->ghwcfg3) >> 16);
	dwc_write_reg32(&global_regs->gdfifocfg, gdfifocfg.d32);

	rxfsiz = (dwc_read_reg32(&global_regs->grxfsiz) & 0x0000ffff);
	nptxfsiz = (dwc_read_reg32(&global_regs->gnptxfsiz) >> 16);
	gdfifocfg.b.epinfobase = rxfsiz + nptxfsiz;
	dwc_write_reg32(&global_regs->gdfifocfg, gdfifocfg.d32);

	/* Flush the FIFOs */
	dwc_otg_flush_tx_fifo(core_if, 0x10); /* all Tx FIFOs */
	dwc_otg_flush_rx_fifo(core_if);

	/* Flush the Learning Queue. */
	resetctl.b.intknqflsh = 1;
	dwc_write_reg32(&core_if->core_global_regs->grstctl, resetctl.d32);

	/* Clear all pending Device Interrupts */
	/** @todo - if the condition needed to be checked
	 *  or in any case all pending interrutps should be cleared?
	 */
	if (core_if->multiproc_int_enable) {
	//	log_out("dev init pos a1\n");
		for (i = 0; i < core_if->dev_if->num_in_eps; ++i) {
			dwc_write_reg32(&dev_if->dev_global_regs->diepeachintmsk[i], 0);
		}

		for (i = 0; i < core_if->dev_if->num_out_eps; ++i) {
			dwc_write_reg32(&dev_if->dev_global_regs->doepeachintmsk[i], 0);
		}

		dwc_write_reg32(&dev_if->dev_global_regs->deachint, 0xFFFFFFFF);
		dwc_write_reg32(&dev_if->dev_global_regs->deachintmsk, 0);
	} else {
	//	log_out("dev init pos a2");
		dwc_write_reg32(&dev_if->dev_global_regs->diepmsk, 0);
		dwc_write_reg32(&dev_if->dev_global_regs->doepmsk, 0);
		dwc_write_reg32(&dev_if->dev_global_regs->daint, 0xFFFFFFFF);
		dwc_write_reg32(&dev_if->dev_global_regs->daintmsk, 0);
	}
//	log_out("dev init pos 2");
	for (i = 0; i <= dev_if->num_in_eps; i++) {

		depctl_data_t depctl;
	//	log_out("dev init pos 3");
		depctl.d32 = dwc_read_reg32(&dev_if->in_ep_regs[i]->diepctl);

		if (depctl.b.epena) {
			depctl.d32 = 0;
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
		} else {
			depctl.d32 = 0;
		}

		dwc_write_reg32(&dev_if->in_ep_regs[i]->diepctl, depctl.d32);
//		reg_val_out("dIepctl", depctl.d32);
		dwc_write_reg32(&dev_if->in_ep_regs[i]->dieptsiz, 0);
		dwc_write_reg32(&dev_if->in_ep_regs[i]->diepdma, 0);
		dwc_write_reg32(&dev_if->in_ep_regs[i]->diepint, 0xFF);
	}
//	reg_val_out("dev_if->num_out_eps", dev_if->num_out_eps);
//	for (i = 0; i <= dev_if->num_out_eps; i++) {
	for (i = 0; i <= 3; i++) {
		depctl_data_t depctl;

		depctl.d32 = dwc_read_reg32(&dev_if->out_ep_regs[i]->doepctl);
		if (depctl.b.epena) {
			depctl.d32 = 0;
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
		} else {
			depctl.d32 = 0;
		}

		dwc_write_reg32(&dev_if->out_ep_regs[i]->doepctl, depctl.d32);
//		reg_val_out("DOEPCTL", depctl.d32);
		dwc_write_reg32(&dev_if->out_ep_regs[i]->doeptsiz, 0);
		dwc_write_reg32(&dev_if->out_ep_regs[i]->doepdma, 0);
		dwc_write_reg32(&dev_if->out_ep_regs[i]->doepint, 0xFF);
	}

	if (core_if->en_multiple_tx_fifo && core_if->dma_enable) {
		dev_if->non_iso_tx_thr_en = params->thr_ctl & 0x1;
		dev_if->iso_tx_thr_en = (params->thr_ctl >> 1) & 0x1;
		dev_if->rx_thr_en = (params->thr_ctl >> 2) & 0x1;

		dev_if->rx_thr_length = params->rx_thr_length;
		dev_if->tx_thr_length = params->tx_thr_length;

		dev_if->setup_desc_index = 0;

		dthrctl.d32 = 0;
		dthrctl.b.non_iso_thr_en = dev_if->non_iso_tx_thr_en;
		dthrctl.b.iso_thr_en = dev_if->iso_tx_thr_en;
		dthrctl.b.tx_thr_len = dev_if->tx_thr_length;
		dthrctl.b.rx_thr_en = dev_if->rx_thr_en;
		dthrctl.b.rx_thr_len = dev_if->rx_thr_length;
		dthrctl.b.ahb_thr_ratio = params->ahb_thr_ratio;

		dwc_write_reg32(&dev_if->dev_global_regs->dtknqr3_dthrctl, dthrctl.d32);
	}

	dwc_otg_enable_device_interrupts(core_if);

	{
		diepmsk_data_t msk;
		msk.d32 = 0;
		msk.b.txfifoundrn = 1;
		if (core_if->multiproc_int_enable) {
			dwc_modify_reg32(&dev_if->dev_global_regs->diepeachintmsk[0],
					msk.d32, msk.d32);
	//		reg_val_out("DIEPEACHINTMSK", msk.d32);
		} else {
			dwc_modify_reg32(&dev_if->dev_global_regs->diepmsk, msk.d32,
					msk.d32);
		//	reg_val_out("DIEPMSK", *USB_DIEPMSK);

		}
	}

	if (core_if->multiproc_int_enable) {
		/* Set NAK on Babble */
		dctl_data_t dctl;
		dctl.d32 = 0;
		dctl.b.nakonbble = 1;
		dwc_modify_reg32(&dev_if->dev_global_regs->dctl, 0, dctl.d32);
	}
}

/**
 * This function reads a setup packet from the Rx FIFO into the destination
 * buffer. This function is called from the Rx Status Queue Level (RxStsQLvl)
 * Interrupt routine when a SETUP packet has been received in Slave mode.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param dest Destination buffer for packet data.
 */
void dwc_otg_read_setup_packet(dwc_otg_core_if_t * core_if, uint32_t * dest) {
	/* Get the 8 bytes of a setup transaction data */

	/* Pop 2 DWORDS off the receive data FIFO into memory */
	dest[0] = dwc_read_reg32(core_if->data_fifo[0]);
	dest[1] = dwc_read_reg32(core_if->data_fifo[0]);
}

/**
 * This function reads a packet from the Rx FIFO into the destination
 * buffer. To read SETUP data use dwc_otg_read_setup_packet.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param dest    Destination buffer for the packet.
 * @param bytes  Number of bytes to copy to the destination.
 */
void dwc_otg_read_packet(dwc_otg_core_if_t * core_if, uint32_t * dest,
		uint16_t bytes) {
	int i;
	int word_count = (bytes + 3) / 4;

	uint32_t *data_buff = (uint32_t *) dest;

	/**
	 * @todo Account for the case where _dest is not dword aligned. This
	 * requires reading data from the FIFO into a uint32_t temp buffer,
	 * then moving it into the data buffer.
	 */
	for (i = 0; i < word_count; i++, data_buff++) {
		*data_buff = dwc_read_reg32(core_if->data_fifo[0]);
	}

	return;
}


/**
 * This function writes a packet into the Tx FIFO associated with the
 * EP. For non-periodic EPs the non-periodic Tx FIFO is written.  For
 * periodic EPs the periodic Tx FIFO associated with the EP is written
 * with all packets for the next micro-frame.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to write packet for.
 * @param dma Indicates if DMA is being used.
 */
void dwc_otg_ep_write_packet(dwc_otg_core_if_t * core_if, dwc_ep_t * ep,
		int dma) {
	/**
	 * The buffer is padded to DWORD on a per packet basis in
	 * slave/dma mode if the MPS is not DWORD aligned. The last
	 * packet, if short, is also padded to a multiple of DWORD.
	 *
	 * ep->xfer_buff always starts DWORD aligned in memory and is a
	 * multiple of DWORD in length
	 *
	 * ep->xfer_len can be any number of bytes
	 *
	 * ep->xfer_count is a multiple of ep->maxpacket until the last
	 *	packet
	 *
	 * FIFO access is DWORD */

	uint32_t i;
	uint32_t byte_count;
	uint32_t dword_count;
	uint32_t *fifo;
	uint32_t *data_buff = (uint32_t *) ep->xfer_buff;

	if (ep->xfer_count >= ep->xfer_len) {

		return;
	}

	/* Find the byte length of the packet either short packet or MPS */
	if ((ep->xfer_len - ep->xfer_count) < ep->maxpacket) {
		byte_count = ep->xfer_len - ep->xfer_count;
	} else {
		byte_count = ep->maxpacket;
	}

	/* Find the DWORD length, padded by extra bytes as neccessary if MPS
	 * is not a multiple of DWORD */
	dword_count = (byte_count + 3) / 4;

#ifdef VERBOSE
	dump_msg(ep->xfer_buff, byte_count);
#endif

	/**@todo NGS Where are the Periodic Tx FIFO addresses
	 * intialized?	What should this be? */

	fifo = core_if->data_fifo[ep->num];
	if (!dma) {
		for (i = 0; i < dword_count; i++, data_buff++) {
			dwc_write_reg32(fifo, *data_buff);
		}
	}

	ep->xfer_count += byte_count;
	ep->xfer_buff += byte_count;
	ep->dma_addr += byte_count;
}

/**
 * This function enables EP0 OUT to receive SETUP packets and configures EP0
 * IN for transmitting packets. It is normally called when the
 * "Enumeration Done" interrupt occurs.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_activate(dwc_otg_core_if_t * core_if, dwc_ep_t * ep) {
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	dsts_data_t dsts;
	depctl_data_t diepctl;
	depctl_data_t doepctl;
	dctl_data_t dctl;
	dctl.d32 = 0;
	/* Read the Device Status and Endpoint 0 Control registers */
	dsts.d32 = dwc_read_reg32(&dev_if->dev_global_regs->dsts);
	diepctl.d32 = dwc_read_reg32(&dev_if->in_ep_regs[0]->diepctl);
	doepctl.d32 = dwc_read_reg32(&dev_if->out_ep_regs[0]->doepctl);
	/* Set the MPS of the IN EP based on the enumeration speed */
	switch (dsts.b.enumspd) {
	case DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_48MHZ:
		diepctl.b.mps = DWC_DEP0CTL_MPS_64;
		break;
	case DWC_DSTS_ENUMSPD_LS_PHY_6MHZ:
		diepctl.b.mps = DWC_DEP0CTL_MPS_8;
		log_out("MPS_8\n");
		break;
	}
	dwc_write_reg32(&dev_if->in_ep_regs[0]->diepctl, diepctl.d32);

	/* Enable OUT EP for receive */
	doepctl.b.epena = 1;
//	doepctl.b.epena = 0;
	dwc_write_reg32(&dev_if->out_ep_regs[0]->doepctl, doepctl.d32);
//	reg_val_out("doepctl.d32", doepctl.d32);
#ifdef VERBOSE
	DWC_DEBUGPL(DBG_PCDV, "doepctl0=%0x\n", dwc_read_reg32(&dev_if->out_ep_regs[0]->doepctl));
	DWC_DEBUGPL(DBG_PCDV, "diepctl0=%0x\n", dwc_read_reg32(&dev_if->in_ep_regs[0]->diepctl));
#endif
	dctl.b.cgnpinnak = 1;

	dwc_modify_reg32(&dev_if->dev_global_regs->dctl, dctl.d32, dctl.d32);
}

/**
 * This function does the setup for a data transfer for EP0 and starts
 * the transfer.  For an IN transfer, the packets will be loaded into
 * the appropriate Tx FIFO in the ISR. For OUT transfers, the packets are
 * unloaded from the Rx FIFO in the ISR.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_start_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep) {
	depctl_data_t depctl;
	deptsiz0_data_t deptsiz;
	gintmsk_data_t intr_mask;
	//dwc_otg_dev_dma_desc_t *dma_desc;
//	uint32_t i;
	intr_mask.d32 = 0;

	ep->total_len = ep->xfer_len;

	/* IN endpoint */
//	 ep->is_in=1;

//	if (ep->is_in == 1) {
	if(1) {

		dwc_otg_dev_in_ep_regs_t *in_regs = core_if->dev_if->in_ep_regs[0];
		gnptxsts_data_t gtxstatus;
	//	log_out("ep0 start pos 2\n");

		gtxstatus.d32 =	dwc_read_reg32(&core_if->core_global_regs->gnptxsts);
	//	log_out("ep0 start pos 3\n");
		if (core_if->en_multiple_tx_fifo == 0 && gtxstatus.b.nptxqspcavail == 0) {
			return;
		}

		depctl.d32 = dwc_read_reg32(&in_regs->diepctl);
		deptsiz.d32 = dwc_read_reg32(&in_regs->dieptsiz);
	//	log_out("ep0 start pos 4\n");
		/* Zero Length Packet? */
		if (ep->xfer_len == 0) {
	//		log_out("ep0 start pos 5a\n");
			deptsiz.b.xfersize = 0;
			deptsiz.b.pktcnt = 1;
		} else {

		//	if (ep->xfer_len > ep->maxpacket) {
		//		log_out("ppos 3a\n");
		//		ep->xfer_len = ep->maxpacket;
		//		deptsiz.b.xfersize = ep->maxpacket;
		//	} else {
		//		log_out("ppos 3b\n");
		//		deptsiz.b.xfersize = ep->xfer_len;
		//	}

			deptsiz.b.xfersize = ep->xfer_len - ep->xfer_count;
			deptsiz.b.pktcnt = (ep->xfer_len - ep->xfer_count - 1 + ep->maxpacket) / ep->maxpacket;

			if (core_if->dma_enable) {
				if(deptsiz.b.xfersize > 64) {
					deptsiz.b.reserved20_28 = 0x01;
				}
			}
	//		for(i= 0; i< 0x300; i++);
			//deptsiz.b.xfersize = ep->xfer_len;
			//deptsiz.b.pktcnt = 1;
		}

		/* Write the DMA register */
		if (core_if->dma_enable) {
			log_out("write dma reg\n");
			if (core_if->dma_desc_enable == 0) {
				dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
				log_out("write dma reg\n");
				dwc_write_reg32(&(in_regs->diepdma), (uint32_t) ep->dma_addr);
			} else {
//				dma_desc = core_if->dev_if->in_desc_addr;

				/** DMA Descriptor Setup */
//				dma_desc->status.b.bs = BS_HOST_BUSY;
//				dma_desc->status.b.l = 1;
//				dma_desc->status.b.ioc = 1;
//				dma_desc->status.b.sp = (ep->xfer_len == ep->maxpacket) ? 0 : 1;
//				dma_desc->status.b.bytes = ep->xfer_len;
//				dma_desc->buf = ep->dma_addr;
//				dma_desc->status.b.bs = BS_HOST_READY;

				/** DIEPDMA0 Register write */
//				dwc_write_reg32(&in_regs->diepdma, core_if->dev_if->dma_in_desc_addr);
			}
		} else {
		//	log_out("ep0 pos 66\n");
			dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
		}

		/* EP enable, IN data in FIFO */

		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

		/**
		 * Enable the Non-Periodic Tx FIFO empty interrupt, the
		 * data will be written into the fifo by the ISR.
		 */
		if (!core_if->dma_enable) {
		//	log_out("eeee\n");
			if (core_if->en_multiple_tx_fifo == 0) {
			//	log_out("eeeee  11\n");;
				intr_mask.b.nptxfempty = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gintmsk, intr_mask.d32, intr_mask.d32);
			} else {
			//	log_out("eeeeee 22");
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if (ep->xfer_len > 0) {
					uint32_t fifoemptymsk = 0;
					fifoemptymsk |= 1 << ep->num;
					dwc_modify_reg32( &core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk, 0, fifoemptymsk);
				}
			}
		}
	}
}

/**
 * This function does the setup for a data transfer for an EP and
 * starts the transfer. For an IN transfer, the packets will be
 * loaded into the appropriate Tx FIFO in the ISR. For OUT transfers,
 * the packets are unloaded from the Rx FIFO in the ISR.  the ISR.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to start the transfer on.
 */

void dwc_otg_ep_start_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep) {
	depctl_data_t depctl;
	deptsiz_data_t deptsiz;
	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0;

	/* IN endpoint */

	if (1) {
		dwc_otg_dev_in_ep_regs_t *in_regs = core_if->dev_if->in_ep_regs[ep->num];

		gnptxsts_data_t gtxstatus;

		gtxstatus.d32 = dwc_read_reg32(&core_if->core_global_regs->gnptxsts);
	//	log_out("in step1\n");
		if (core_if->en_multiple_tx_fifo == 0
				&& gtxstatus.b.nptxqspcavail == 0) {
			return;
		}

//		log_out("in step2\n");
		depctl.d32 = dwc_read_reg32(&(in_regs->diepctl));
		deptsiz.d32 = dwc_read_reg32(&(in_regs->dieptsiz));

		ep->xfer_len += (ep->maxxfer < (ep->total_len - ep->xfer_len)) ? ep->maxxfer : (ep->total_len - ep->xfer_len);

//		log_out("in step3\n");
		/* Zero Length Packet? */
		if ((ep->xfer_len - ep->xfer_count) == 0) {
			deptsiz.b.xfersize = 0;
			deptsiz.b.pktcnt = 1;
		} else {
			/* Program the transfer size and packet count
			 *      as follows: xfersize = N * maxpacket +
			 *      short_packet pktcnt = N + (short_packet
			 *      exist ? 1 : 0)
			 */
			deptsiz.b.xfersize = ep->xfer_len - ep->xfer_count;
			deptsiz.b.pktcnt = (ep->xfer_len - ep->xfer_count - 1 + ep->maxpacket) / ep->maxpacket;
		}

		/* Write the DMA register */
		if (core_if->dma_enable) {
			log_out("in step3 - dma\n");
			if (core_if->dma_desc_enable == 0) {

				log_out("in step3 - dma\n");
			//	dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
			//	dwc_write_reg32(&(in_regs->diepdma), (uint32_t) ep->dma_addr);
				in_regs = core_if->dev_if->in_ep_regs[1];
				dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);

				printk(KERN_WARNING "in_regs->diepdma= %x",(uint32_t)(ep->dma_addr));
				dwc_write_reg32(&(in_regs->diepdma), (uint32_t) ep->dma_addr);
				in_regs = core_if->dev_if->in_ep_regs[2];
				dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
				dwc_write_reg32(&(in_regs->diepdma), (uint32_t) ep->dma_addr);


			} else {
			//	init_dma_desc_chain(core_if, ep);
				/** DIEPDMAn Register write */
				dwc_write_reg32(&in_regs->diepdma, ep->dma_addr);
			}
		} else {
//			log_out("in step4\n");
			dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
			if (ep->type != DWC_OTG_EP_TYPE_ISOC) {
				/**
				 * Enable the Non-Periodic Tx FIFO empty interrupt,
				 * or the Tx FIFO epmty interrupt in dedicated Tx FIFO mode,
				 * the data will be written into the fifo by the ISR.
				 */
				if (core_if->en_multiple_tx_fifo == 0) {
					intr_mask.b.nptxfempty = 1;
					dwc_modify_reg32(&core_if->core_global_regs->gintmsk,
							intr_mask.d32, intr_mask.d32);
				} else {
	//				log_out("in step5\n");
					/* Enable the Tx FIFO Empty Interrupt for this EP */
					if (ep->xfer_len > 0) {
						uint32_t fifoemptymsk = 0;
						fifoemptymsk = 1 << ep->num;
						dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
								0, fifoemptymsk);
					}
				}
			}
		}

		if (core_if->dma_enable) {
			in_regs = core_if->dev_if->in_ep_regs[2];
			/* EP enable, IN data in FIFO */
			depctl.b.epdis =0;
			depctl.b.cnak = 1;
			depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);
		depctl.d32 =
		dwc_read_reg32(&core_if->dev_if->in_ep_regs[2]->diepctl);
		dwc_write_reg32(&core_if->dev_if->in_ep_regs[2]->diepctl, depctl.d32);

		in_regs = core_if->dev_if->in_ep_regs[1];
		/* EP enable, IN data in FIFO */
		depctl.b.epdis =0;
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

		depctl.d32 = dwc_read_reg32(&core_if->dev_if->in_ep_regs[1]->diepctl);
		depctl.b.nextep = ep->num;
		dwc_write_reg32(&core_if->dev_if->in_ep_regs[1]->diepctl, depctl.d32);
		}else {
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

	//	log_out("in step6\n");
		depctl.d32 =
		dwc_read_reg32(&core_if->dev_if->in_ep_regs[0]->diepctl);
		depctl.b.nextep = ep->num;
		dwc_write_reg32(&core_if->dev_if->in_ep_regs[0]->diepctl, depctl.d32);
		}
	}
}

/**
 * This function continues control IN transfers started by
 * dwc_otg_ep0_start_transfer, when the transfer does not fit in a
 * single packet.  NOTE: The DIEPCTL0/DOEPCTL0 registers only have one
 * bit for the packet count.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_continue_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep) {
	depctl_data_t depctl;
	deptsiz0_data_t deptsiz;
	gintmsk_data_t intr_mask;
	//dwc_otg_dev_dma_desc_t *dma_desc;
	intr_mask.d32 = 0;
	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs = core_if->dev_if->in_ep_regs[0];

		/** @todo Should there be check for room in the Tx
		 * Status Queue.  If not remove the code above this comment. */

		depctl.d32 = dwc_read_reg32(&in_regs->diepctl);
		deptsiz.d32 = dwc_read_reg32(&in_regs->dieptsiz);

		/* Program the transfer size and packet count
		 *      as follows: xfersize = N * maxpacket +
		 *      short_packet pktcnt = N + (short_packet
		 *      exist ? 1 : 0)
		 */

		if (core_if->dma_desc_enable == 0) {
			deptsiz.b.xfersize =
					(ep->total_len - ep->xfer_count) > ep->maxpacket ?
							ep->maxpacket : (ep->total_len - ep->xfer_count);
			deptsiz.b.pktcnt = 1;
			if (core_if->dma_enable == 0) {
				ep->xfer_len += deptsiz.b.xfersize;
			} else {
				ep->xfer_len = deptsiz.b.xfersize;
			}
			dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
		} else {
//			ep->xfer_len =
//					(ep->total_len - ep->xfer_count) > ep->maxpacket ?
//							ep->maxpacket : (ep->total_len - ep->xfer_count);

//			dma_desc = core_if->dev_if->in_desc_addr;

			/** DMA Descriptor Setup */
//			dma_desc->status.b.bs = BS_HOST_BUSY;
//			dma_desc->status.b.l = 1;
//			dma_desc->status.b.ioc = 1;
//			dma_desc->status.b.sp = (ep->xfer_len == ep->maxpacket) ? 0 : 1;
//			dma_desc->status.b.bytes = ep->xfer_len;
//			dma_desc->buf = ep->dma_addr;
//			dma_desc->status.b.bs = BS_HOST_READY;

			/** DIEPDMA0 Register write */
//			dwc_write_reg32(&in_regs->diepdma,
//					core_if->dev_if->dma_in_desc_addr);
//		}

		/* Write the DMA register */
//		if (core_if->hwcfg2.b.architecture == DWC_INT_DMA_ARCH) {
//			if (core_if->dma_desc_enable == 0)
//				dwc_write_reg32(&(in_regs->diepdma), (uint32_t) ep->dma_addr);
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

		/**
		 * Enable the Non-Periodic Tx FIFO empty interrupt, the
		 * data will be written into the fifo by the ISR.
		 */
		if (!core_if->dma_enable) {
//			if (core_if->en_multiple_tx_fifo == 0) {
				/* First clear it from GINTSTS */
//				intr_mask.b.nptxfempty = 1;
//				dwc_modify_reg32(&core_if->core_global_regs->gintmsk,
//						intr_mask.d32, intr_mask.d32);

//			} else {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
//				if (ep->xfer_len > 0) {
//					uint32_t fifoemptymsk = 0;
//					fifoemptymsk |= 1 << ep->num;
//					dwc_modify_reg32(
//							&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
//							0, fifoemptymsk);
//				}
//			}
		}
	} else {
		dwc_otg_dev_out_ep_regs_t *out_regs = core_if->dev_if->out_ep_regs[0];

		depctl.d32 = dwc_read_reg32(&out_regs->doepctl);
		deptsiz.d32 = dwc_read_reg32(&out_regs->doeptsiz);

		/* Program the transfer size and packet count
		 *      as follows: xfersize = N * maxpacket +
		 *      short_packet pktcnt = N + (short_packet
		 *      exist ? 1 : 0)
		 */
		deptsiz.b.xfersize = ep->maxpacket;
		deptsiz.b.pktcnt = 1;

		if (core_if->dma_desc_enable == 0) {
			dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);
		} else {
//			dma_desc = core_if->dev_if->out_desc_addr;

			/** DMA Descriptor Setup */
//			dma_desc->status.b.bs = BS_HOST_BUSY;
//			dma_desc->status.b.l = 1;
//			dma_desc->status.b.ioc = 1;
//			dma_desc->status.b.bytes = ep->maxpacket;
//			dma_desc->buf = ep->dma_addr;
//			dma_desc->status.b.bs = BS_HOST_READY;

			/** DOEPDMA0 Register write */
//			dwc_write_reg32(&out_regs->doepdma,
//					core_if->dev_if->dma_out_desc_addr);
		}

		/* Write the DMA register */
		if (core_if->hwcfg2.b.architecture == DWC_INT_DMA_ARCH) {
//			if (core_if->dma_desc_enable == 0)
//				dwc_write_reg32(&(out_regs->doepdma), (uint32_t) ep->dma_addr);
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&out_regs->doepctl, depctl.d32);

	}
}

/**
 * Set the EP STALL.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to set the stall on.
 */
void dwc_otg_ep_set_stall(dwc_otg_core_if_t * core_if, dwc_ep_t * ep) {
	depctl_data_t depctl;
	volatile uint32_t *depctl_addr;
	if (ep->is_in == 1) {
		depctl_addr = &(core_if->dev_if->in_ep_regs[ep->num]->diepctl);
		depctl.d32 = dwc_read_reg32(depctl_addr);
		/* set the disable and stall bits */
		if (depctl.b.epena) {
			depctl.b.epdis = 1;
		}
		depctl.b.stall = 1;
		dwc_write_reg32(depctl_addr, depctl.d32);
	} else {
		depctl_addr = &(core_if->dev_if->out_ep_regs[ep->num]->doepctl);
		depctl.d32 = dwc_read_reg32(depctl_addr);
		/* set the stall bit */
		depctl.b.stall = 1;
		dwc_write_reg32(depctl_addr, depctl.d32);
	}
	return;
}

/**
 * This function setup a zero length transfer in Buffer DMA and
 * Slave modes for usb requests with zero field set
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to start the transfer on.
 *
 */
void dwc_otg_ep_start_zl_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep) {

	depctl_data_t depctl;
	deptsiz_data_t deptsiz;
	gintmsk_data_t intr_mask;

	intr_mask.d32 = 0;

	log_out("zlp transfer\n");
	/* IN endpoint */
	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs = core_if->dev_if->in_ep_regs[ep->num];

		depctl.d32 = dwc_read_reg32(&(in_regs->diepctl));
		deptsiz.d32 = dwc_read_reg32(&(in_regs->dieptsiz));

		deptsiz.b.xfersize = 0;
		deptsiz.b.pktcnt = 1;

		/* Write the DMA register */
		if (core_if->dma_enable) {
//			if (core_if->dma_desc_enable == 0) {
//				dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
//				dwc_write_reg32(&(in_regs->diepdma), (uint32_t) ep->dma_addr);
//			}
		} else {
			dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
			/**
			 * Enable the Non-Periodic Tx FIFO empty interrupt,
			 * or the Tx FIFO epmty interrupt in dedicated Tx FIFO mode,
			 * the data will be written into the fifo by the ISR.
			 */
			if (core_if->en_multiple_tx_fifo == 0) {
				intr_mask.b.nptxfempty = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gintmsk,
						intr_mask.d32, intr_mask.d32);
			} else {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if (ep->xfer_len > 0) {
					uint32_t fifoemptymsk = 0;
					fifoemptymsk = 1 << ep->num;
					dwc_modify_reg32(
							&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
							0, fifoemptymsk);
				}
			}
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

		depctl.d32 =
		dwc_read_reg32(&core_if->dev_if->in_ep_regs[0]->diepctl);
		depctl.b.nextep = ep->num;
		dwc_write_reg32(&core_if->dev_if->in_ep_regs[0]->diepctl, depctl.d32);

	} else {
		/* OUT endpoint */
		dwc_otg_dev_out_ep_regs_t *out_regs =
				core_if->dev_if->out_ep_regs[ep->num];

		depctl.d32 = dwc_read_reg32(&(out_regs->doepctl));
		deptsiz.d32 = dwc_read_reg32(&(out_regs->doeptsiz));

		/* Zero Length Packet */
		deptsiz.b.xfersize = ep->maxpacket;
		deptsiz.b.pktcnt = 1;

		if (core_if->dma_enable) {
//			if (!core_if->dma_desc_enable) {
//				dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);

//				dwc_write_reg32(&(out_regs->doepdma), (uint32_t) ep->dma_addr);
//			}
		} else {
			dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);
		}

		/* EP enable */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;

		dwc_write_reg32(&out_regs->doepctl, depctl.d32);

	}
}
/**
 * Clear the EP STALL.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to clear stall from.
 */
void dwc_otg_ep_clear_stall(dwc_otg_core_if_t * core_if, dwc_ep_t * ep) {
	depctl_data_t depctl;
	volatile uint32_t *depctl_addr;
	if (ep->is_in == 1) {
		depctl_addr = &(core_if->dev_if->in_ep_regs[ep->num]->diepctl);
	} else {
		depctl_addr = &(core_if->dev_if->out_ep_regs[ep->num]->doepctl);
	}

	depctl.d32 = dwc_read_reg32(depctl_addr);

	/* clear the stall bits */
	depctl.b.stall = 0;

	/*
	 * USB Spec 9.4.5: For endpoints using data toggle, regardless
	 * of whether an endpoint has the Halt feature set, a
	 * ClearFeature(ENDPOINT_HALT) request always results in the
	 * data toggle being reinitialized to DATA0.
	 */
	if (ep->type == DWC_OTG_EP_TYPE_INTR || ep->type == DWC_OTG_EP_TYPE_BULK) {
		depctl.b.setd0pid = 1; /* DATA0 */
	}

	dwc_write_reg32(depctl_addr, depctl.d32);

	return;
}
/**
 * This function activates an EP.  The Device EP control register for
 * the EP is configured as defined in the ep structure. Note: This
 * function is not used for EP0.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to activate.
 */
void dwc_otg_ep_activate(dwc_otg_core_if_t * core_if, dwc_ep_t * ep) {
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	depctl_data_t depctl;
	volatile uint32_t *addr;
	daint_data_t daintmsk;
	daintmsk.d32 = 0;

	/* Read DEPCTLn register */
	if (ep->is_in == 1) {
		//log_out("=====in4 \n");
		addr = &dev_if->in_ep_regs[ep->num]->diepctl;
		daintmsk.ep.in = 1 << ep->num;
	} else {
		addr = &dev_if->out_ep_regs[ep->num]->doepctl;
		daintmsk.ep.out = 1 << ep->num;
	}

	/* If the EP is already active don't change the EP Control
	 * register. */
	depctl.d32 = dwc_read_reg32(addr);
//	reg_val_out("depctl", depctl.d32);
//	if (!depctl.b.usbactep)
	{
//		log_out("=====in5 \n");
		depctl.b.mps = ep->maxpacket;
		depctl.b.eptype = ep->type;
		depctl.b.txfnum = ep->tx_fifo_num;
		if (ep->is_in == 1){
			depctl.b.epdis =1;
		}
		else
		{
			depctl.b.epdis =0;
		}
		depctl.b.snak =0;
		depctl.b.cnak = 1;
		depctl.b.epena =1;


		if (ep->type == DWC_OTG_EP_TYPE_ISOC) {
			depctl.b.setd0pid = 1;
		} else {
			depctl.b.setd0pid = 1;
		}
		depctl.b.usbactep = 1;

		dwc_write_reg32(addr, depctl.d32);
	}

	/* Enable the Interrupt for this EP */
	if (core_if->multiproc_int_enable) {
		if (ep->is_in == 1) {

			diepmsk_data_t diepmsk;
//			log_out("=====in4 \n");
			diepmsk.d32 = 0;
			diepmsk.b.xfercompl = 1;
			diepmsk.b.timeout = 1;
			diepmsk.b.epdisabled = 1;
			diepmsk.b.ahberr = 1;
			diepmsk.b.intknepmis = 1;
			diepmsk.b.txfifoundrn = 1;
			/*
			 if (core_if->dma_desc_enable) {
			 diepmsk.b.bna = 1;
			 }
			 */
			/*
			 if (core_if->dma_enable) {
			 doepmsk.b.nak = 1;
			 }
			 */

			dwc_write_reg32(&dev_if->dev_global_regs->diepeachintmsk[ep->num], diepmsk.d32);

		} else {
			doepmsk_data_t doepmsk;
			doepmsk.d32 = 0;
			doepmsk.b.xfercompl = 1;
			doepmsk.b.ahberr = 1;
			doepmsk.b.epdisabled = 1;

			/*

			 if (core_if->dma_desc_enable) {
			 doepmsk.b.bna = 1;
			 }
			 */
			/*
			 doepmsk.b.babble = 1;
			 doepmsk.b.nyet = 1;
			 doepmsk.b.nak = 1;
			 */
			dwc_write_reg32(&dev_if->dev_global_regs->doepeachintmsk[ep->num], doepmsk.d32);
		}
		dwc_modify_reg32(&dev_if->dev_global_regs->deachintmsk, 0, daintmsk.d32);
	} else {
		printk(KERN_WARNING "daintmsk= %x",daintmsk.d32);
		dwc_modify_reg32(&dev_if->dev_global_regs->daintmsk, 0, daintmsk.d32);
	}

	ep->stall_clear_flag = 0;

	return;
}

