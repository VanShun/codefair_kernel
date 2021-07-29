#ifndef DWC_OTG_CIL_H_
#define DWC_OTG_CIL_H_

#include "dwc_otg_core_if.h"
#include "dwc_otg_regs.h"
#include <linux/types.h>



/** Macros defined for DWC OTG HW Release verison */

#define OTG_CORE_REV_2_60a	0x4F54260A
#define OTG_CORE_REV_2_71a	0x4F54271A
#define OTG_CORE_REV_2_72a	0x4F54272A
#define OTG_CORE_REV_2_80a	0x4F54280A
#define OTG_CORE_REV_2_81a	0x4F54281A
#define OTG_CORE_REV_2_90a	0x4F54290A
#define OTG_CORE_REV_2_91a	0x4F54291A
/**
 * Information for each ISOC packet.
 */
typedef struct iso_pkt_info {
	uint32_t offset;
	uint32_t length;
	int32_t status;
} iso_pkt_info_t;

/**
 * The <code>dwc_ep</code> structure represents the state of a single
 * endpoint when acting in device mode. It contains the data items
 * needed for an endpoint to be activated and transfer packets.
 */
typedef struct dwc_ep {
	/** EP number used for register address lookup */
	uint8_t num;
	/** EP direction 0 = OUT */
	uint8_t is_in;
	/** EP active. */
	uint8_t active;

	/**
	 *Periodic Tx FIFO # for IN EPs For INTR EP set to 0 to use non-periodic Tx FIFO
	 * If dedicated Tx FIFOs are enabled for all IN Eps - Tx FIFO # FOR IN EPs*/
	uint8_t tx_fifo_num;
	/** EP type: 0 - Control, 1 - ISOC,	 2 - BULK,	3 - INTR */
	uint8_t type;
#define DWC_OTG_EP_TYPE_CONTROL	   0
#define DWC_OTG_EP_TYPE_ISOC	   1
#define DWC_OTG_EP_TYPE_BULK	   2
#define DWC_OTG_EP_TYPE_INTR	   3

	/** DATA start PID for INTR and BULK EP */
	uint8_t data_pid_start:1;
	/** Frame (even/odd) for ISOC EP */
	uint8_t even_odd_frame:1;
	/** Max Packet bytes */
	uint16_t maxpacket;

	/** Max Transfer size */
	uint32_t maxxfer;

	/** @name Transfer state */
	/** @{ */

	/**
	 * Pointer to the beginning of the transfer buffer -- do not modify
	 * during transfer.
	 */

	dwc_dma_t dma_addr;

//	dwc_dma_t dma_desc_addr;
	dwc_otg_dev_dma_desc_t *desc_addr;

	uint8_t *start_xfer_buff;
	/** pointer to the transfer buffer */
	uint8_t *xfer_buff;
	/** Number of bytes to transfer */
	uint32_t xfer_len;
	/** Number of bytes transferred. */
	uint32_t xfer_count;
	/** Sent ZLP */
	uint32_t sent_zlp;
	/** Total len for control transfer */
	uint32_t total_len;

	/** stall clear flag */
	uint8_t stall_clear_flag;

	/** Allocated DMA Desc count */
	uint32_t desc_cnt;


	//add by zz
	uint32_t xfer_len_bak ;
	uint32_t xfer_count_bak;
	uint32_t total_len_bak;
	uint8_t *xfer_buff_bak;

/** @} */
} dwc_ep_t;

/*
 * Device States
 */
typedef enum dwc_otg_lx_state {
	/** On state */
	DWC_OTG_L0,
	/** LPM sleep state*/
	DWC_OTG_L1,
	/** USB suspend state*/
	DWC_OTG_L2,
	/** Off state*/
	DWC_OTG_L3
} dwc_otg_lx_state_e;


struct dwc_otg_global_regs_backup {
	uint32_t gotgctl_local;
	uint32_t gintmsk_local;
	uint32_t gahbcfg_local;
	uint32_t gusbcfg_local;
	uint32_t grxfsiz_local;
	uint32_t gnptxfsiz_local;
#ifdef CONFIG_USB_DWC_OTG_LPM
	uint32_t glpmcfg_local;
#endif
	uint32_t gi2cctl_local;
	uint32_t hptxfsiz_local;
	uint32_t pcgcctl_local;
	uint32_t gdfifocfg_local;
	uint32_t dtxfsiz_local[MAX_EPS_CHANNELS];
	uint32_t gpwrdn_local;
};


struct dwc_otg_dev_regs_backup {
	uint32_t dcfg;
	uint32_t dctl;
	uint32_t daintmsk;
	uint32_t diepmsk;
	uint32_t doepmsk;
	uint32_t diepctl[MAX_EPS_CHANNELS];
	uint32_t dieptsiz[MAX_EPS_CHANNELS];
	uint32_t diepdma[MAX_EPS_CHANNELS];
	uint32_t doepfn[MAX_EPS_CHANNELS];
};

/**
 * The following parameters may be specified when starting the module. These
 * parameters define how the DWC_otg controller should be configured.
 */
typedef struct dwc_otg_core_params {
	int32_t opt;

	/**
	 * Specifies the OTG capabilities. The driver will automatically
	 * detect the value for this parameter if none is specified.
	 * 0 - HNP and SRP capable (default)
	 * 1 - SRP Only capable
	 * 2 - No HNP/SRP capable
	 */
	int32_t otg_cap;

	/**
	 * Specifies whether to use slave or DMA mode for accessing the data
	 * FIFOs. The driver will automatically detect the value for this
	 * parameter if none is specified.
	 * 0 - Slave
	 * 1 - DMA (default, if available)
	 */
	int32_t dma_enable;

	/**
	 * When DMA mode is enabled specifies whether to use address DMA or DMA Descritor mode for accessing the data
	 * FIFOs in device mode. The driver will automatically detect the value for this
	 * parameter if none is specified.
	 * 0 - address DMA
	 * 1 - DMA Descriptor(default, if available)
	 */
	int32_t dma_desc_enable;
	/** The DMA Burst size (applicable only for External DMA
	 * Mode). 1, 4, 8 16, 32, 64, 128, 256 (default 32)
	 */
	int32_t dma_burst_size;	/* Translate this to GAHBCFG values */

	/**
	 * Specifies the maximum speed of operation in host and device mode.
	 * The actual speed depends on the speed of the attached device and
	 * the value of phy_type. The actual speed depends on the speed of the
	 * attached device.
	 * 0 - High Speed (default)
	 * 1 - Full Speed
	 */
	int32_t speed;
	/** Specifies whether low power mode is supported when attached
	 *	to a Full Speed or Low Speed device in host mode.
	 * 0 - Don't support low power mode (default)
	 * 1 - Support low power mode
	 */
	int32_t host_support_fs_ls_low_power;

	/** Specifies the PHY clock rate in low power mode when connected to a
	 * Low Speed device in host mode. This parameter is applicable only if
	 * HOST_SUPPORT_FS_LS_LOW_POWER is enabled. If PHY_TYPE is set to FS
	 * then defaults to 6 MHZ otherwise 48 MHZ.
	 *
	 * 0 - 48 MHz
	 * 1 - 6 MHz
	 */
	int32_t host_ls_low_power_phy_clk;

	/**
	 * 0 - Use cC FIFO size parameters
	 * 1 - Allow dynamic FIFO sizing (default)
	 */
	int32_t enable_dynamic_fifo;

	/** Total number of 4-byte words in the data FIFO memory. This
	 * memory includes the Rx FIFO, non-periodic Tx FIFO, and periodic
	 * Tx FIFOs.
	 * 32 to 32768 (default 8192)
	 * Note: The total FIFO memory depth in the FPGA configuration is 8192.
	 */
	int32_t data_fifo_size;

	/** Number of 4-byte words in the Rx FIFO in device mode when dynamic
	 * FIFO sizing is enabled.
	 * 16 to 32768 (default 1064)
	 */
	int32_t dev_rx_fifo_size;

	/** Number of 4-byte words in the non-periodic Tx FIFO in device mode
	 * when dynamic FIFO sizing is enabled.
	 * 16 to 32768 (default 1024)
	 */
	int32_t dev_nperio_tx_fifo_size;

	/** Number of 4-byte words in each of the periodic Tx FIFOs in device
	 * mode when dynamic FIFO sizing is enabled.
	 * 4 to 768 (default 256)
	 */
	uint32_t dev_perio_tx_fifo_size[MAX_PERIO_FIFOS];

	/** Number of 4-byte words in the Rx FIFO in host mode when dynamic
	 * FIFO sizing is enabled.
	 * 16 to 32768 (default 1024)
	 */
	int32_t host_rx_fifo_size;

	/** Number of 4-byte words in the non-periodic Tx FIFO in host mode
	 * when Dynamic FIFO sizing is enabled in the core.
	 * 16 to 32768 (default 1024)
	 */
	int32_t host_nperio_tx_fifo_size;

	/** Number of 4-byte words in the host periodic Tx FIFO when dynamic
	 * FIFO sizing is enabled.
	 * 16 to 32768 (default 1024)
	 */
	int32_t host_perio_tx_fifo_size;

	/** The maximum transfer size supported in bytes.
	 * 2047 to 65,535  (default 65,535)
	 */
	int32_t max_transfer_size;

	/** The maximum number of packets in a transfer.
	 * 15 to 511  (default 511)
	 */
	int32_t max_packet_count;

	/** The number of host channel registers to use.
	 * 1 to 16 (default 12)
	 * Note: The FPGA configuration supports a maximum of 12 host channels.
	 */
	int32_t host_channels;

	/** The number of endpoints in addition to EP0 available for device
	 * mode operations.
	 * 1 to 15 (default 6 IN and OUT)
	 * Note: The FPGA configuration supports a maximum of 6 IN and OUT
	 * endpoints in addition to EP0.
	 */
	int32_t dev_endpoints;

		/**
		 * Specifies the type of PHY interface to use. By default, the driver
		 * will automatically detect the phy_type.
		 *
		 * 0 - Full Speed PHY
		 * 1 - UTMI+ (default)
		 * 2 - ULPI
		 */
	int32_t phy_type;

	/**
	 * Specifies the UTMI+ Data Width. This parameter is
	 * applicable for a PHY_TYPE of UTMI+ or ULPI. (For a ULPI
	 * PHY_TYPE, this parameter indicates the data width between
	 * the MAC and the ULPI Wrapper.) Also, this parameter is
	 * applicable only if the OTG_HSPHY_WIDTH cC parameter was set
	 * to "8 and 16 bits", meaning that the core has been
	 * configured to work at either data path width.
	 *
	 * 8 or 16 bits (default 16)
	 */
	int32_t phy_utmi_width;

	/**
	 * Specifies whether the ULPI operates at double or single
	 * data rate. This parameter is only applicable if PHY_TYPE is
	 * ULPI.
	 *
	 * 0 - single data rate ULPI interface with 8 bit wide data
	 * bus (default)
	 * 1 - double data rate ULPI interface with 4 bit wide data
	 * bus
	 */
	int32_t phy_ulpi_ddr;

	/**
	 * Specifies whether to use the internal or external supply to
	 * drive the vbus with a ULPI phy.
	 */
	int32_t phy_ulpi_ext_vbus;

	/**
	 * Specifies whether to use the I2Cinterface for full speed PHY. This
	 * parameter is only applicable if PHY_TYPE is FS.
	 * 0 - No (default)
	 * 1 - Yes
	 */
	int32_t i2c_enable;

	int32_t ulpi_fs_ls;

	int32_t ts_dline;

	/**
	 * Specifies whether dedicated transmit FIFOs are
	 * enabled for non periodic IN endpoints in device mode
	 * 0 - No
	 * 1 - Yes
	 */
	int32_t en_multiple_tx_fifo;

	/** Number of 4-byte words in each of the Tx FIFOs in device
	 * mode when dynamic FIFO sizing is enabled.
	 * 4 to 768 (default 256)
	 */
	uint32_t dev_tx_fifo_size[MAX_TX_FIFOS];

	/** Thresholding enable flag-
	 * bit 0 - enable non-ISO Tx thresholding
	 * bit 1 - enable ISO Tx thresholding
	 * bit 2 - enable Rx thresholding
	 */
	uint32_t thr_ctl;

	/** Thresholding length for Tx
	 *	FIFOs in 32 bit DWORDs
	 */
	uint32_t tx_thr_length;

	/** Thresholding length for Rx
	 *	FIFOs in 32 bit DWORDs
	 */
	uint32_t rx_thr_length;

	/**
	 * Specifies whether LPM (Link Power Management) support is enabled
	 */
	int32_t lpm_enable;

	/** Per Transfer Interrupt
	 *	mode enable flag
	 * 1 - Enabled
	 * 0 - Disabled
	 */
	int32_t pti_enable;

	/** Multi Processor Interrupt
	 *	mode enable flag
	 * 1 - Enabled
	 * 0 - Disabled
	 */
	int32_t mpi_enable;

	/** IS_USB Capability
	 * 1 - Enabled
	 * 0 - Disabled
	 */
	int32_t ic_usb_cap;

	/** AHB Threshold Ratio
	 * 2'b00 AHB Threshold = 	MAC Threshold
	 * 2'b01 AHB Threshold = 1/2 	MAC Threshold
	 * 2'b10 AHB Threshold = 1/4	MAC Threshold
	 * 2'b11 AHB Threshold = 1/8	MAC Threshold
	 */
	int32_t ahb_thr_ratio;

	/** ADP Support
	 * 1 - Enabled
	 * 0 - Disabled
	 */
	int32_t adp_supp_enable;

		/** Core Power down mode
	 * 0 - No Power Down is enabled
	 * 1 - Reserved
	 * 2 - Complete Power Down (Hibernation)
	 */
	int32_t power_down;

	/** OTG revision supported
	 * 0 - OTG 1.3 revision
	 * 1 - OTG 2.0 revision
	 */
	int32_t otg_ver;

} dwc_otg_core_params_t;

/**
 * DWC_otg CIL callback structure. This structure allows the HCD and
 * PCD to register functions used for starting and stopping the PCD
 * and HCD for role change on for a DRD.
 */
typedef struct dwc_otg_cil_callbacks {
	/** Start function for role change */
	int (*start) (void *_p);
	/** Stop Function for role change */
	int (*stop) (void *_p);
	/** Disconnect Function for role change */
	int (*disconnect) (void *_p);
	/** Resume/Remote wakeup Function */
	int (*resume_wakeup) (void *_p);
	/** Suspend function */
	int (*suspend) (void *_p);
	/** Session Start (SRP) */
	int (*session_start) (void *_p);
	/** Pointer passed to start() and stop() */
	void *p;
} dwc_otg_cil_callbacks_t;

/**
 * The <code>dwc_otg_core_if</code> structure contains information needed to manage
 * the DWC_otg controller acting in either host or device mode. It
 * represents the programming view of the controller as a whole.
 */
struct dwc_otg_core_if {

	/** Parameters that define how the core should be configured.*/
	dwc_otg_core_params_t *core_params;

	/** Core Global registers starting at offset 000h. */
	dwc_otg_core_global_regs_t *core_global_regs;

	/** Device-specific information */
	dwc_otg_dev_if_t *dev_if;
	/** Host-specific information */
	dwc_otg_host_if_t *host_if;

	/** Value from SNPSID register */
	uint32_t snpsid;

	/*
	 * Set to 1 if the core PHY interface bits in USBCFG have been
	 * initialized.
	 */
	uint8_t phy_init_done;

	/*
	 * SRP Success flag, set by srp success interrupt in FS I2C mode
	 */
	uint8_t srp_success;
	uint8_t srp_timer_started;
	/** Timer for SRP. If it expires before SRP is successful
	 * clear the SRP. */
	dwc_timer_t *srp_timer;

	/* Common configuration information */
	/** Power and Clock Gating Control Register */
	volatile uint32_t *pcgcctl;
#define DWC_OTG_PCGCCTL_OFFSET 0xE00

	/** Push/pop addresses for endpoints or host channels.*/
	uint32_t *data_fifo[MAX_EPS_CHANNELS];
#define DWC_OTG_DATA_FIFO_OFFSET 0x1000
#define DWC_OTG_DATA_FIFO_SIZE 0x1000

	/** Total RAM for FIFOs (Bytes) */
	uint16_t total_fifo_size;
	/** Size of Rx FIFO (Bytes) */
	uint16_t rx_fifo_size;
	/** Size of Non-periodic Tx FIFO (Bytes) */
	uint16_t nperio_tx_fifo_size;

	/** 1 if DMA is enabled, 0 otherwise. */
	uint8_t dma_enable;

	/** 1 if DMA descriptor is enabled, 0 otherwise. */
	uint8_t dma_desc_enable;

	/** 1 if PTI Enhancement mode is enabled, 0 otherwise. */
	uint8_t pti_enh_enable;

	/** 1 if MPI Enhancement mode is enabled, 0 otherwise. */
	uint8_t multiproc_int_enable;

	/** 1 if dedicated Tx FIFOs are enabled, 0 otherwise. */
	uint8_t en_multiple_tx_fifo;

	/** Set to 1 if multiple packets of a high-bandwidth transfer is in
	 * process of being queued */
	uint8_t queuing_high_bandwidth;

	/** Hardware Configuration -- stored here for convenience.*/
	hwcfg1_data_t hwcfg1;
	hwcfg2_data_t hwcfg2;
	hwcfg3_data_t hwcfg3;
	hwcfg4_data_t hwcfg4;

	/** Host and Device Configuration -- stored here for convenience.*/
	hcfg_data_t hcfg;
	dcfg_data_t dcfg;

	/** The operational State, during transations
	 * (a_host>>a_peripherial and b_device=>b_host) this may not
	 * match the core but allows the software to determine
	 * transitions.
	 */
	uint8_t op_state;

	/**
	 * Set to 1 if the HCD needs to be restarted on a session request
	 * interrupt. This is required if no connector ID status change has
	 * occurred since the HCD was last disconnected.
	 */
	uint8_t restart_hcd_on_session_req;

	/** HCD callbacks */
	/** A-Device is a_host */
#define A_HOST		(1)
	/** A-Device is a_suspend */
#define A_SUSPEND	(2)
	/** A-Device is a_peripherial */
#define A_PERIPHERAL	(3)
	/** B-Device is operating as a Peripheral. */
#define B_PERIPHERAL	(4)
	/** B-Device is operating as a Host. */
#define B_HOST		(5)

	/** HCD callbacks */
	struct dwc_otg_cil_callbacks *hcd_cb;
	/** PCD callbacks */
	struct dwc_otg_cil_callbacks *pcd_cb;

	/** Device mode Periodic Tx FIFO Mask */
	uint32_t p_tx_msk;
	/** Device mode Periodic Tx FIFO Mask */
	uint32_t tx_msk;

	/** Workqueue object used for handling several interrupts */
//	dwc_workq_t *wq_otg;

	/** Timer object used for handling "Wakeup Detected" Interrupt */
//	dwc_timer_t *wkp_timer;

	/** Lx state of device */
	dwc_otg_lx_state_e lx_state;

	/** Saved Core Global registers */
	struct dwc_otg_global_regs_backup *gr_backup;
	/** Saved Host registers */
	struct dwc_otg_host_regs_backup *hr_backup;
	/** Saved Device registers */
	struct dwc_otg_dev_regs_backup *dr_backup;

	/** Power Down Enable */
	uint32_t power_down;

	/** ADP support Enable */
	uint32_t adp_enable;

	/** ADP structure object */
//	dwc_otg_adp_t adp;

	/** hibernation/suspend flag */
	int hibernation_suspend;

	/** OTG revision supported */
	uint32_t otg_ver;

	/** OTG status flag used for HNP polling */
	uint8_t otg_sts;

};

/**
 * This function returns the Core Interrupt register.
 */
static __inline uint32_t dwc_otg_read_core_intr(dwc_otg_core_if_t * core_if)
{
	return (dwc_read_reg32(&core_if->core_global_regs->gintsts) &
		dwc_read_reg32(&core_if->core_global_regs->gintmsk));
}


/**
 * This function returns the mode of the operation, host or device.
 *
 * @return 0 - Device Mode, 1 - Host Mode
 */
static __inline uint32_t dwc_otg_mode(dwc_otg_core_if_t * _core_if)
{
	return (dwc_read_reg32(&_core_if->core_global_regs->gintsts) & 0x1);
}


/** Start the PCD.  Helper function for using the PCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static __inline void cil_pcd_start(dwc_otg_core_if_t * core_if)
{
	if (core_if->pcd_cb && core_if->pcd_cb->start) {
		core_if->pcd_cb->start(core_if->pcd_cb->p);
	}
}
/** Stop the PCD.  Helper function for using the PCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static __inline void cil_pcd_stop(dwc_otg_core_if_t * core_if)
{
	if (core_if->pcd_cb && core_if->pcd_cb->stop) {
		core_if->pcd_cb->stop(core_if->pcd_cb->p);
	}
}
/** Disconnect the HCD.  Helper function for using the HCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static __inline void cil_hcd_disconnect(dwc_otg_core_if_t * core_if)
{
	if (core_if->hcd_cb && core_if->hcd_cb->disconnect) {
		core_if->hcd_cb->disconnect(core_if->hcd_cb->p);
	}
}
/** Suspend the PCD.  Helper function for using the PCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static __inline void cil_pcd_suspend(dwc_otg_core_if_t * core_if)
{
	if (core_if->pcd_cb && core_if->pcd_cb->suspend) {
		core_if->pcd_cb->suspend(core_if->pcd_cb->p);
	}
}
/** Resume the PCD.  Helper function for using the PCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static __inline void cil_pcd_resume(dwc_otg_core_if_t * core_if)
{
	if (core_if->pcd_cb && core_if->pcd_cb->resume_wakeup) {
		core_if->pcd_cb->resume_wakeup(core_if->pcd_cb->p);
	}
}
/**
 * This function reads the Device All Endpoints Interrupt register and
 * returns the IN endpoint interrupt bits.
 */
static __inline uint32_t dwc_otg_read_dev_all_in_ep_intr(dwc_otg_core_if_t *
						       core_if)
{

	uint32_t v;

	if (core_if->multiproc_int_enable) {
		v = dwc_read_reg32(&core_if->dev_if->dev_global_regs->
				   deachint) & dwc_read_reg32(&core_if->dev_if->
							      dev_global_regs->
							      deachintmsk);
	} else {
		v = dwc_read_reg32(&core_if->dev_if->dev_global_regs->daint) &
		    dwc_read_reg32(&core_if->dev_if->dev_global_regs->daintmsk);
	}
	return (v & 0xffff);
}
/**
 * This function returns the Device IN EP Interrupt register
 */
static __inline uint32_t dwc_otg_read_dev_in_ep_intr(dwc_otg_core_if_t * core_if,
						   dwc_ep_t * ep)
{
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	uint32_t v, msk, emp;

	if (core_if->multiproc_int_enable) {
		msk =
		    dwc_read_reg32(&dev_if->dev_global_regs->
				   diepeachintmsk[ep->num]);
		emp =
		    dwc_read_reg32(&dev_if->dev_global_regs->
				   dtknqr4_fifoemptymsk);
		msk |= ((emp >> ep->num) & 0x1) << 7;
		v = dwc_read_reg32(&dev_if->in_ep_regs[ep->num]->diepint) & msk;
	} else {
		msk = dwc_read_reg32(&dev_if->dev_global_regs->diepmsk);
		emp =
		    dwc_read_reg32(&dev_if->dev_global_regs->
				   dtknqr4_fifoemptymsk);
		msk |= ((emp >> ep->num) & 0x1) << 7;
		v = dwc_read_reg32(&dev_if->in_ep_regs[ep->num]->diepint) & msk;
	}

	return v;
}
/**
 * This function reads the Device All Endpoints Interrupt register and
 * returns the OUT endpoint interrupt bits.
 */
static __inline uint32_t dwc_otg_read_dev_all_out_ep_intr(dwc_otg_core_if_t *
							core_if)
{
	uint32_t v;

	if (core_if->multiproc_int_enable) {
		v = dwc_read_reg32(&core_if->dev_if->dev_global_regs->
				   deachint) & dwc_read_reg32(&core_if->dev_if->
							      dev_global_regs->
							      deachintmsk);
	} else {
		v = dwc_read_reg32(&core_if->dev_if->dev_global_regs->daint) &
		    dwc_read_reg32(&core_if->dev_if->dev_global_regs->daintmsk);
	}

	return ((v & 0xffff0000) >> 16);
}
/**
 * This function returns the Device OUT EP Interrupt register
 */
static __inline uint32_t dwc_otg_read_dev_out_ep_intr(dwc_otg_core_if_t *
						    _core_if, dwc_ep_t * _ep)
{
	dwc_otg_dev_if_t *dev_if = _core_if->dev_if;
	uint32_t v;
	doepmsk_data_t msk;
	msk.d32 = 0;

	if (_core_if->multiproc_int_enable) {
		msk.d32 =
		    dwc_read_reg32(&dev_if->dev_global_regs->
				   doepeachintmsk[_ep->num]);
		if (_core_if->pti_enh_enable) {
			msk.b.pktdrpsts = 1;
		}
		v = dwc_read_reg32(&dev_if->out_ep_regs[_ep->num]->
				   doepint) & msk.d32;
	} else {
		msk.d32 = dwc_read_reg32(&dev_if->dev_global_regs->doepmsk);
		if (_core_if->pti_enh_enable) {
			msk.b.pktdrpsts = 1;
		}
		v = dwc_read_reg32(&dev_if->out_ep_regs[_ep->num]->
				   doepint) & msk.d32;
	}
	return v;
}



extern void dwc_otg_core_reset(dwc_otg_core_if_t * _core_if);
void dwc_otg_cil_register_pcd_callbacks(dwc_otg_core_if_t * core_if, dwc_otg_cil_callbacks_t * cb, void *p);

void dwc_otg_core_dev_init(dwc_otg_core_if_t * core_if);
void dwc_otg_flush_tx_fifo(dwc_otg_core_if_t * core_if, const int num);
void dwc_otg_flush_rx_fifo(dwc_otg_core_if_t * core_if);
void dwc_otg_ep_write_packet(dwc_otg_core_if_t * core_if, dwc_ep_t * ep, int dma);
void dwc_otg_ep_write_packet(dwc_otg_core_if_t * core_if, dwc_ep_t * ep, int dma);
void dwc_otg_ep0_activate(dwc_otg_core_if_t * core_if, dwc_ep_t * ep);
void dwc_otg_read_setup_packet(dwc_otg_core_if_t * core_if, uint32_t * dest);
void dwc_otg_read_packet(dwc_otg_core_if_t * core_if, uint32_t * dest, uint16_t bytes);
void dwc_otg_ep0_start_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep);
void dwc_otg_ep_start_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep);
void dwc_otg_ep0_continue_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep);
void dwc_otg_ep_set_stall(dwc_otg_core_if_t * core_if, dwc_ep_t * ep);
void dwc_otg_ep_start_zl_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep);
void dwc_otg_ep_clear_stall(dwc_otg_core_if_t * core_if, dwc_ep_t * ep);
void dwc_otg_ep_activate(dwc_otg_core_if_t * core_if, dwc_ep_t * ep);
#endif


















