
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/prefetch.h>
#include <linux/clk.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/usb/ch9.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/unaligned.h>

#include "dwc_otg_pcd.h"
#include "wk_def.h"
#include "dwc_otg_pcd_if.h"
#include "desc.h"
#include "dwc_otg_pcd_rtos.h"
#include "wk_def.h"
#include "wokoo_usb.h"
#include "wokoo_udc.h"

#define log_out printk

extern int wkoo_add_queue(void);

extern struct gadget_wrapper g_gadget_wrapper;
extern uint8_t configuration_descriptor[70];
extern  uint8_t device_descriptor[0x20];
extern  uint8_t LANGUAGE_ID[4];
extern  uint8_t device_serial_number[12];
extern  uint8_t ManufacturerString[80];
extern  uint8_t ProducterString[28];
extern  uint8_t c1_feedback[70];
extern  uint8_t c0_feedback[70];
extern uint32_t setup_pkt[10];
extern uint32_t setup_pkt_bak[0x100];
extern uint32_t Bulk_out[80];
extern uint32_t Bulk_out_bak[80];
uint32_t DMA_address[72];

static const char ep0name [] = "ep0";


uint8_t trans_cycles=0;

uint32_t setup_packet[10];
uint32_t out_packet[128];
uint8_t NUM1_sendf =0;
uint8_t  Enumpacket64 = 0;

uint8_t deal_0_bulkout = 0;
//extern uint8_t delayflag;
uint8_t senddata_flag = 0;
uint8_t recvflag = 0;
uint16_t sendnum = 0;

uint8_t get_descript[200]; //__attribute__ ((at(0x402D0)));
uint8_t get_descript_bak[200];// __attribute__ ((at(0x403A0)));

uint8_t Bulk_in[0x200];// __attribute__ ((at(0x40500)));
uint8_t Bulk_in_bak[0x200];// __attribute__ ((at(0x40800)));

uint8_t key[8] = {0x00,0x00,0x0B,0x00,0x00,0x00,0x00,0x00};
uint8_t key1[8] = {0x02,0x00,0x0B,0x00,0x00,0x00,0x00,0x00};

int k = 0;

static void dwc_otg_pcd_update_otg(dwc_otg_pcd_t * pcd, const unsigned reset) {

	if (reset) {
		pcd->b_hnp_enable = 0;
		pcd->a_hnp_support = 0;
		pcd->a_alt_hnp_support = 0;
	}

	if (pcd->fops->hnp_changed) {
		pcd->fops->hnp_changed(pcd);
	}
}

/**
 * This function is called when the Device is disconnected. It stops
 * any active requests and informs the Gadget driver of the
 * disconnect.
 */

void dwc_otg_pcd_stop(dwc_otg_pcd_t * pcd) {
	int i, num_in_eps, num_out_eps;
	dwc_otg_pcd_ep_t *ep;

	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0;

//	DWC_SPINLOCK(pcd->lock);

	num_in_eps = GET_CORE_IF(pcd)->dev_if->num_in_eps;
	num_out_eps = GET_CORE_IF(pcd)->dev_if->num_out_eps;

	/* don't disconnect drivers more than once */
	if (pcd->ep0state == EP0_DISCONNECT) {

		return;
	}
	pcd->ep0state = EP0_DISCONNECT;

	/* Reset the OTG state. */
	dwc_otg_pcd_update_otg(pcd, 1);

	/* Disable the NP Tx Fifo Empty Interrupt. */
	intr_mask.b.nptxfempty = 1;
	dwc_modify_reg32(&GET_CORE_IF(pcd)->core_global_regs->gintmsk,
			intr_mask.d32, 0);

	/* Flush the FIFOs */
	/**@todo NGS Flush Periodic FIFOs */
	dwc_otg_flush_tx_fifo(GET_CORE_IF(pcd), 0x10);
	dwc_otg_flush_rx_fifo(GET_CORE_IF(pcd));

	/* prevent new request submissions, kill any outstanding requests  */
	ep = &pcd->ep0;
	dwc_otg_request_nuke(ep);
	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < num_in_eps; i++) {
		dwc_otg_pcd_ep_t *ep = &pcd->in_ep[i];
		dwc_otg_request_nuke(ep);
	}
	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < num_out_eps; i++) {
		dwc_otg_pcd_ep_t *ep = &pcd->out_ep[i];
		dwc_otg_request_nuke(ep);
	}

	/* report disconnect; the driver is already quiesced */
	if (pcd->fops->disconnect) {
//		DWC_SPINUNLOCK(pcd->lock);
		pcd->fops->disconnect(pcd);
//		DWC_SPINLOCK(pcd->lock);
	}
//	DWC_SPINUNLOCK(pcd->lock);

}
/**
 * This functions gets a pointer to an EP from the wIndex address
 * value of the control request.
 */
dwc_otg_pcd_ep_t *get_ep_by_addr(dwc_otg_pcd_t * pcd, u16 wIndex) {
	dwc_otg_pcd_ep_t *ep;
	uint32_t ep_num = UE_GET_ADDR(wIndex);

	if (ep_num == 0) {
		ep = &pcd->ep0;
	} else if (UE_GET_DIR(wIndex) == UE_DIR_IN) { /* in ep */
		ep = &pcd->in_ep[ep_num - 1];
	} else {
		ep = &pcd->out_ep[ep_num - 1];
	}

	return ep;
}
/**
 * This function examines the Device IN Token Learning Queue to
 * determine the EP number of the last IN token received.  This
 * implementation is for the Mass Storage device where there are only
 * 2 IN EPs (Control-IN and BULK-IN).
 *
 * The EP numbers for the first six IN Tokens are in DTKNQR1 and there
 * are 8 EP Numbers in each of the other possible DTKNQ Registers.
 *
 * @param core_if Programming view of DWC_otg controller.
 *
 */
static __inline int get_ep_of_last_in_token(dwc_otg_core_if_t * core_if) {
	dwc_otg_device_global_regs_t *dev_global_regs =
			core_if->dev_if->dev_global_regs;
	const uint32_t TOKEN_Q_DEPTH = core_if->hwcfg2.b.dev_token_q_depth;
	/* Number of Token Queue Registers */
	const int DTKNQ_REG_CNT = (TOKEN_Q_DEPTH + 7) / 8;
	dtknq1_data_t dtknqr1;
	uint32_t in_tkn_epnums[4];
	int ndx = 0;
	int i = 0;
	volatile uint32_t *addr = &dev_global_regs->dtknqr1;
	int epnum = 0;

	//DWC_DEBUGPL(DBG_PCD,"dev_token_q_depth=%d\n",TOKEN_Q_DEPTH);

	/* Read the DTKNQ Registers */
	for (i = 0; i < DTKNQ_REG_CNT; i++) {
		in_tkn_epnums[i] = dwc_read_reg32(addr);

		if (addr == &dev_global_regs->dvbusdis) {
			addr = &dev_global_regs->dtknqr3_dthrctl;
		} else {
			++addr;
		}

	}

	/* Copy the DTKNQR1 data to the bit field. */
	dtknqr1.d32 = in_tkn_epnums[0];
	/* Get the EP numbers */
	in_tkn_epnums[0] = dtknqr1.b.epnums0_5;
	ndx = dtknqr1.b.intknwptr - 1;

	//DWC_DEBUGPL(DBG_PCDV,"ndx=%d\n",ndx);
	if (ndx == -1) {
		/** @todo Find a simpler way to calculate the max
		 * queue position.*/
		int cnt = TOKEN_Q_DEPTH;
		if (TOKEN_Q_DEPTH <= 6) {
			cnt = TOKEN_Q_DEPTH - 1;
		} else if (TOKEN_Q_DEPTH <= 14) {
			cnt = TOKEN_Q_DEPTH - 7;
		} else if (TOKEN_Q_DEPTH <= 22) {
			cnt = TOKEN_Q_DEPTH - 15;
		} else {
			cnt = TOKEN_Q_DEPTH - 23;
		}
		epnum = (in_tkn_epnums[DTKNQ_REG_CNT - 1] >> (cnt * 4)) & 0xF;
	} else {
		if (ndx <= 5) {
			epnum = (in_tkn_epnums[0] >> (ndx * 4)) & 0xF;
		} else if (ndx <= 13) {
			ndx -= 6;
			epnum = (in_tkn_epnums[1] >> (ndx * 4)) & 0xF;
		} else if (ndx <= 21) {
			ndx -= 14;
			epnum = (in_tkn_epnums[2] >> (ndx * 4)) & 0xF;
		} else if (ndx <= 29) {
			ndx -= 22;
			epnum = (in_tkn_epnums[3] >> (ndx * 4)) & 0xF;
		}
	}
	//DWC_DEBUGPL(DBG_PCD,"epnum=%d\n",epnum);
	return epnum;
}

/**
 * This function returns pointer to in ep struct with number ep_num
 */
static __inline dwc_otg_pcd_ep_t *get_in_ep(dwc_otg_pcd_t * pcd,
		uint32_t ep_num) {
	int i;
	int num_in_eps = GET_CORE_IF(pcd)->dev_if->num_in_eps;
	if (ep_num == 0) {
		return &pcd->ep0;
	} else {
		for (i = 1; i < num_in_eps; ++i) {
			if (pcd->in_ep[i].dwc_ep.num == ep_num)
				return &pcd->in_ep[i];
		}
		return 0;
	}
}

static __inline void do_setup_in_status_phase(dwc_otg_pcd_t * pcd) {
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	if (pcd->ep0state == EP0_STALL) {
		return;
	}

//	log_out("set address 1\n");
	pcd->ep0state = EP0_IN_STATUS_PHASE;

	/* Prepare for more SETUP Packets */

	ep0->dwc_ep.xfer_len = 0;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.is_in = 1;
//	ep0->dwc_ep.dma_addr = pcd->setup_pkt_dma_handle;
	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &ep0->dwc_ep);

	/* Prepare for more SETUP Packets */
	//ep0_out_start(GET_CORE_IF(pcd), pcd);
}

/**
 * This function configures EPO to receive SETUP packets.
 *
 * @todo NGS: Update the comments from the HW FS.
 *
 *	-# Program the following fields in the endpoint specific registers
 *	for Control OUT EP 0, in order to receive a setup packet
 *	- DOEPTSIZ0.Packet Count = 3 (To receive up to 3 back to back
 *	  setup packets)
 *	- DOEPTSIZE0.Transfer Size = 24 Bytes (To receive up to 3 back
 *	  to back setup packets)
 *		- In DMA mode, DOEPDMA0 Register with a memory address to
 *		  store any setup packets received
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param pcd	  Programming view of the PCD.
 */

static __inline void ep0_out_start(dwc_otg_core_if_t * core_if, dwc_otg_pcd_t * pcd) {
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	deptsiz0_data_t doeptsize0;
	dwc_otg_dev_dma_desc_t *dma_desc;
	depctl_data_t doepctl;
	doeptsize0.d32 = 0;
	doepctl.d32 = 0;

	doeptsize0.b.supcnt = 3;
	doeptsize0.b.pktcnt = 1;
	doeptsize0.b.xfersize = 8*3 ;
	if (core_if->dma_enable) {
		if (!core_if->dma_desc_enable) {
			/** put here as for Hermes mode deptisz register should not be written */
			dwc_write_reg32(&dev_if->out_ep_regs[0]->doeptsiz, doeptsize0.d32);
			dwc_write_reg32(&dev_if->out_ep_regs[0]->doepdma, (uint32_t)setup_pkt_bak);
		} else {
			dev_if->setup_desc_index = (dev_if->setup_desc_index + 1) & 1;
			dma_desc = dev_if->setup_desc_addr[dev_if->setup_desc_index];

			/** DMA Descriptor Setup */
			dma_desc->status.b.bs = BS_HOST_BUSY;
			dma_desc->status.b.l = 1;
			dma_desc->status.b.ioc = 1;
			dma_desc->status.b.bytes = pcd->ep0.dwc_ep.maxpacket;
			dma_desc->buf = pcd->setup_pkt_dma_handle;
			dma_desc->status.b.bs = BS_HOST_READY;

			dwc_write_reg32(&dev_if->out_ep_regs[0]->doeptsiz, doeptsize0.d32);

			/** DOEPDMA0 Register write */
			dwc_write_reg32(&dev_if->out_ep_regs[0]->doepdma, dev_if->dma_setup_desc_addr[dev_if->setup_desc_index]);
		}
	} else {
		/** put here as for Hermes mode deptisz register should not be written */
		dwc_write_reg32(&dev_if->out_ep_regs[0]->doeptsiz, doeptsize0.d32);
	}

	/** DOEPCTL0 Register write */
	doepctl.b.epena = 1;
	doepctl.b.cnak = 1;
	dwc_write_reg32(&dev_if->out_ep_regs[0]->doepctl, doepctl.d32);
}

/**
 * This function starts the Zero-Length Packet for the OUT status phase
 * of a 2 stage control transfer.
 */
static __inline void do_setup_out_status_phase(dwc_otg_pcd_t * pcd) {
	//dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	if (pcd->ep0state == EP0_STALL) {

		return;
	}
	pcd->ep0state = EP0_OUT_STATUS_PHASE;

//	ep0->dwc_ep.xfer_len = 0;
//	ep0->dwc_ep.xfer_count = 0;
//	ep0->dwc_ep.is_in = 0;
//	ep0->dwc_ep.dma_addr = pcd->setup_pkt_dma_handle;
//	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &ep0->dwc_ep);

	/* Prepare for more SETUP Packets */
	if (GET_CORE_IF(pcd)->dma_enable == 0) {
		ep0_out_start(GET_CORE_IF(pcd), pcd);
	}
}

/**
 * This function returns pointer to out ep struct with number ep_num
 */
static __inline dwc_otg_pcd_ep_t *get_out_ep(dwc_otg_pcd_t * pcd,
		uint32_t ep_num) {
	int i;
//	int num_out_eps = GET_CORE_IF(pcd)->dev_if->num_out_eps;

	if (ep_num == 0) {
		return &pcd->ep0;
	} else {
		for (i = 0; i < 3; ++i) {
//			log_out("loop get out ep\n");
			if (pcd->out_ep[i].dwc_ep.num == ep_num)
				return &pcd->out_ep[i];
		}
		return 0;
	}
}
/**
 * This funcion stalls EP0.
 */
static __inline void ep0_do_stall(dwc_otg_pcd_t * pcd, const int err_val) {
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
//	usb_device_request_t *ctrl = &pcd->setup_pkt->req;

	ep0->dwc_ep.is_in = 1;
	dwc_otg_ep_set_stall(GET_CORE_IF(pcd), &ep0->dwc_ep);
	pcd->ep0.stopped = 1;
	pcd->ep0state = EP0_IDLE;
	ep0_out_start(GET_CORE_IF(pcd), pcd);
}

/**
 * This function process the GET_STATUS Setup Commands.
 */
static __inline void do_get_status(dwc_otg_pcd_t * pcd) {

	usb_device_request_t ctrl = pcd->setup_pkt->req;
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	uint16_t *status = pcd->status_buf;

	switch (UT_GET_RECIPIENT(ctrl.bmRequestType)) {
	case UT_DEVICE:
		*status = 0x1; /* Self powered */
		*status |= pcd->remote_wakeup_enable << 1;
		break;

	case UT_INTERFACE:
		*status = 0;
		break;

	case UT_ENDPOINT:
		ep = get_ep_by_addr(pcd, UGETW(ctrl.wIndex));
		if (ep == 0 || UGETW(ctrl.wLength) > 2) {
			ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			return;
		}
		/** @todo check for EP stall */
		*status = ep->stopped;
		break;
	}
	pcd->ep0_pending = 1;
	ep0->dwc_ep.start_xfer_buff = (uint8_t *) status;
	ep0->dwc_ep.xfer_buff = (uint8_t *) status;
	ep0->dwc_ep.dma_addr = pcd->status_buf_dma_handle;
	ep0->dwc_ep.xfer_len = 2;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = ep0->dwc_ep.xfer_len;
	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &ep0->dwc_ep);

}

/**
 * This function process the SET_FEATURE Setup Commands.
 */
static __inline void do_set_feature(dwc_otg_pcd_t * pcd) {

	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	usb_device_request_t ctrl = pcd->setup_pkt->req;
	dwc_otg_pcd_ep_t *ep = 0;
	int32_t otg_cap_param = core_if->core_params->otg_cap;
	gotgctl_data_t gotgctl;
	gotgctl.d32 = 0;

	switch (UT_GET_RECIPIENT(ctrl.bmRequestType)) {
	case UT_DEVICE:
		switch (UGETW(ctrl.wValue)) {
		case UF_DEVICE_REMOTE_WAKEUP:
			pcd->remote_wakeup_enable = 1;
			break;

		case UF_TEST_MODE:
			/* Setup the Test Mode tasklet to do the Test
			 * Packet generation after the SETUP Status
			 * phase has completed. */

			/** @todo This has not been tested since the
			 * tasklet struct was put into the PCD
			 * struct! */
//			pcd->test_mode = UGETW(ctrl.wIndex) >> 8;
//			DWC_TASK_SCHEDULE(pcd->test_mode_tasklet);
			break;

		case UF_DEVICE_B_HNP_ENABLE:

			/* dev may initiate HNP */
			if (otg_cap_param == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) {
				pcd->b_hnp_enable = 1;
				dwc_otg_pcd_update_otg(pcd, 0);

				/**@todo Is the gotgctl.devhnpen cleared
				 * by a USB Reset? */
				gotgctl.b.devhnpen = 1;
				gotgctl.b.hnpreq = 1;
				dwc_write_reg32(&global_regs->gotgctl, gotgctl.d32);
			} else {
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			}
			break;

		case UF_DEVICE_A_HNP_SUPPORT:
			/* RH port supports HNP */

			if (otg_cap_param == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) {
				pcd->a_hnp_support = 1;
				dwc_otg_pcd_update_otg(pcd, 0);
			} else {
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			}
			break;

		case UF_DEVICE_A_ALT_HNP_SUPPORT:
			/* other RH port does */

			if (otg_cap_param == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) {
				pcd->a_alt_hnp_support = 1;
				dwc_otg_pcd_update_otg(pcd, 0);
			} else {
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			}
			break;

		default:
			ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			break;

		}
//		log_out("set up send 0");
		do_setup_in_status_phase(pcd);
		break;

	case UT_INTERFACE:
//		do_gadget_setup(pcd, &ctrl);
		break;

	case UT_ENDPOINT:
		if (UGETW(ctrl.wValue) == UF_ENDPOINT_HALT) {
			ep = get_ep_by_addr(pcd, UGETW(ctrl.wIndex));
			if (ep == 0) {
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
				return;
			}
			ep->stopped = 1;
			dwc_otg_ep_set_stall(core_if, &ep->dwc_ep);
		}
//		log_out("set up send 1");
		do_setup_in_status_phase(pcd);
		break;
	}
}

static __inline void pcd_clear_halt(dwc_otg_pcd_t * pcd, dwc_otg_pcd_ep_t * ep)
{
	if (ep->dwc_ep.stall_clear_flag == 0)
		dwc_otg_ep_clear_stall(GET_CORE_IF(pcd), &ep->dwc_ep);

	/* Reactive the EP */
	dwc_otg_ep_activate(GET_CORE_IF(pcd), &ep->dwc_ep);
	if (ep->stopped) {
		ep->stopped = 0;
		ep->queue_sof = 1;
	}

	do_setup_in_status_phase(pcd);
}

static __inline void do_clear_feature(dwc_otg_pcd_t * pcd)
{
	usb_device_request_t ctrl = pcd->setup_pkt->req;
	dwc_otg_pcd_ep_t *ep = 0;

	switch (UT_GET_RECIPIENT(ctrl.bmRequestType)) {
	case UT_DEVICE:
		switch (UGETW(ctrl.wValue)) {
		case UF_DEVICE_REMOTE_WAKEUP:
			pcd->remote_wakeup_enable = 0;
			break;

		case UF_TEST_MODE:
			break;

		default:
			ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			break;
		}
		do_setup_in_status_phase(pcd);
		break;

	case UT_ENDPOINT:
		ep = get_ep_by_addr(pcd, UGETW(ctrl.wIndex));
		if (ep == 0) {
			ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			return;
		}

		pcd_clear_halt(pcd, ep);

		break;
	}

}

usb_device_request_t ctrl;

static __inline void do_set_address(dwc_otg_pcd_t * pcd)
{
	dwc_otg_dev_if_t *dev_if = GET_CORE_IF(pcd)->dev_if;
	if (ctrl.bmRequestType == UT_DEVICE) {
		dcfg_data_t dcfg;
		dcfg.d32 = 0;

		dcfg.b.devaddr = UGETW(ctrl.wValue);
		dwc_modify_reg32(&dev_if->dev_global_regs->dcfg, 0, dcfg.d32);
		do_setup_in_status_phase(pcd);
	}
}

void write_wokoo_fifo(struct wokoo_ep *ep, struct wokoo_request *req,dwc_otg_pcd_t * pcd) {


	dwc_otg_pcd_ep_t *ep0 ;
    unsigned total;
	u8		*buf;

	if (ep->ep.name == ep0name)
    	ep0  = &pcd->ep0;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);
	total = req->req.length - req->req.actual;

	uint8_t i = 0, data_buf[5];
    memcpy((uint8_t *)data_buf, (uint8_t *)buf, 5);
    for (i = 0; i < 5; i++) {
		printk("--------------------------");
		printk("data = %x\n", data_buf[i]);
   		printk("--------------------------");
    }

	ep0->dwc_ep.start_xfer_buff = buf;
	ep0->dwc_ep.xfer_buff =  buf;

	ep0->dwc_ep.xfer_len = total;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = total;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =total;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;

	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));

    pcd->ep0_pending = 0;

	pcd->ep0state = EP0_IN_DATA_PHASE;
}

uint8_t get_config[72] = {
    0x09,0x02,0x46,0x00,0x01,0x01,0x00,0xe0,0xfa,0x09,0x04,0x00,0x00,0x02,0xdc,0xa0,0xb0,0x00,
    0x07,0x05,0x01,0x02,0x00,0x02,0x00,0x07,0x05,0x81,0x02,0x00,0x02,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0a,0x0b,0x0c,0x0d,0x01,0x02,0x03,0x04
};

static __inline void do_get_descriptor(dwc_otg_pcd_t * pcd) {
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	uint8_t i;

	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<20; i++)
		{

			get_descript[i] = device_descriptor[i];
		}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)device_descriptor;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)device_descriptor;
	}

	NUM1_sendf =0;

	ep0->dwc_ep.xfer_len = 18;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = 18;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =18;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;

	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}

void do_endpoint1_trans(dwc_otg_pcd_t * pcd, uint32_t sendlength )
{
	dwc_otg_pcd_ep_t *ep = &pcd->in_ep[1];

	ep->dwc_ep.start_xfer_buff = (uint8_t *)out_packet;
	ep->dwc_ep.xfer_buff =  (uint8_t *)out_packet;
	ep->dwc_ep.dma_addr = (uint32_t)(&out_packet);

	ep->dwc_ep.xfer_len = sendlength;
	ep->dwc_ep.maxpacket = 512;
	ep->dwc_ep.xfer_count = 0;
	ep->dwc_ep.total_len = sendlength;
	ep->dwc_ep.sent_zlp = 0;
	ep->dwc_ep.is_in = 1;
	ep->dwc_ep.num = 1;
	ep->dwc_ep.maxxfer = 512;

	ep->dwc_ep.xfer_len_bak =sendlength;
	ep->dwc_ep.xfer_count_bak =0;
	ep->dwc_ep.total_len_bak =0;

	dwc_otg_ep_start_transfer(GET_CORE_IF(pcd), &(ep->dwc_ep));

//	log_out("end 3\n");

//	pcd->ep0_pending = 1;




	if (GET_CORE_IF(pcd)->dma_enable == 0)
	{

		dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
		dwc_otg_dev_if_t *dev_if = core_if->dev_if;

				volatile uint32_t *addr1, *addr2;
				depctl_data_t depctl;
				addr1 = &dev_if->in_ep_regs[1]->diepctl;
				addr2 = &dev_if->out_ep_regs[1]->doepctl;

				depctl.d32 = dwc_read_reg32(addr1);

				depctl.b.epdis =1;
				depctl.b.snak =0;
				depctl.b.cnak = 1;
				depctl.b.epena =1;
				dwc_write_reg32(addr1, depctl.d32);


				depctl.d32 = dwc_read_reg32(addr2);
				depctl.b.epdis =0;
				depctl.b.snak =0;
				depctl.b.cnak = 1;
				depctl.b.epena =1;
				dwc_write_reg32(addr2, depctl.d32);





	}
	else
	{
		dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
		dwc_otg_dev_if_t *dev_if = core_if->dev_if;

		volatile uint32_t *addr1;
		depctl_data_t depctl;

		addr1 = &dev_if->in_ep_regs[1]->diepctl;

		depctl.d32 = dwc_read_reg32(addr1);

		depctl.b.epdis =1;
		depctl.b.snak =1;
		depctl.b.cnak = 1;
		depctl.b.epena =0;
		dwc_write_reg32(addr1, depctl.d32);

		addr1 = &dev_if->in_ep_regs[2]->diepctl;
		depctl.d32 = dwc_read_reg32(addr1);

		depctl.b.epdis =1;
		depctl.b.snak =1;
		depctl.b.cnak = 1;
		depctl.b.epena =0;
		dwc_write_reg32(addr1, depctl.d32);
	}
}

static __inline void do_get_config_conti(dwc_otg_pcd_t * pcd, uint8_t contilength)
{
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;

	uint8_t  i;

	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<70; i++)
		{
			get_descript[i] = configuration_descriptor[i];
		}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)configuration_descriptor;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)configuration_descriptor;
	}

	if(contilength == 0xFF)
		contilength = 0x20;

	ep0->dwc_ep.xfer_len = contilength;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = contilength;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =contilength;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;
	ep0->dwc_ep.xfer_buff_bak = (uint8_t *)get_config;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));


}


static __inline void do_get_port(dwc_otg_pcd_t * pcd) {


	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//uint16_t *status = pcd->status_buf;

	uint8_t  i;


	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<4; i++)
		{
			get_descript[i] = key[i];
		}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)key;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)key;
	}



	ep0->dwc_ep.xfer_len = 0x02;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = 0x02;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =0x02;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;
	ep0->dwc_ep.xfer_buff_bak = (uint8_t *)key;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}


static __inline void do_get_c0_ff(dwc_otg_pcd_t * pcd)
{
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	uint8_t  i;

	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<4; i++)
		{
			get_descript[i] = key1[i];
		}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)key1;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)key1;
	}



	ep0->dwc_ep.xfer_len = 0x01;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = 0x01;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =0x01;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;
	ep0->dwc_ep.xfer_buff_bak = (uint8_t *)key1;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}

static __inline void do_get_c1_08(dwc_otg_pcd_t * pcd) {

	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//uint16_t *status = pcd->status_buf;

	uint8_t  i;

	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<4; i++)
		{
			get_descript[i] = key[i];
		}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)key;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)key;
	}

	ep0->dwc_ep.xfer_len = 0x01;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = 0x01;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =0x01;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;
	ep0->dwc_ep.xfer_buff_bak = (uint8_t *)key;

	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}

static __inline void do_get_c1(dwc_otg_pcd_t * pcd) {

	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//uint16_t *status = pcd->status_buf;

	uint8_t  i;


	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<64; i++)
		{
			get_descript[i] = c1_feedback[i];
		}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)c1_feedback;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)c1_feedback;
	}



	ep0->dwc_ep.xfer_len = 0x40;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = 0x40;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =0x40;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;
	ep0->dwc_ep.xfer_buff_bak = (uint8_t *)c1_feedback;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}

static __inline void do_get_c0(dwc_otg_pcd_t * pcd) {


	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//uint16_t *status = pcd->status_buf;

	uint8_t  i;

	for(i=0; i<24; i++)
	{
		c1_feedback[i] = 0x00;
	}

	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<24; i++)
		{
			get_descript[i] = c1_feedback[i];
		}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)c1_feedback;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)c1_feedback;
	}

	ep0->dwc_ep.xfer_len = 19;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = 19;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =19;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;
	ep0->dwc_ep.xfer_buff_bak = (uint8_t *)c1_feedback;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}


static __inline void get_languageid(dwc_otg_pcd_t * pcd) {


	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//uint16_t *status = pcd->status_buf;
	uint8_t i;

	if (GET_CORE_IF(pcd)->dma_enable){
	for(i=0; i<4; i++)
	{
		get_descript[i] = LANGUAGE_ID[i];
	}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)LANGUAGE_ID;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)LANGUAGE_ID;
	}

	ep0->dwc_ep.xfer_len = 4;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = 4;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =4;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}


static __inline void get_serialnumber(dwc_otg_pcd_t * pcd) {


	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//uint16_t *status = pcd->status_buf;
	uint8_t i;

	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<12; i++)
		{
			get_descript[i] = device_serial_number[i];
		}

			ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)device_serial_number;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)device_serial_number;
	}

	ep0->dwc_ep.xfer_len = 10;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = 10;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =10;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}


static __inline void get_manufacture(dwc_otg_pcd_t * pcd, uint8_t manulength) {

	uint8_t i = 0;
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//uint16_t *status = pcd->status_buf;


	if (GET_CORE_IF(pcd)->dma_enable){
        for (i = 0; i < 74; i++)
		{
			get_descript[i] = ManufacturerString[i];
		}

		ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	} else {
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)ManufacturerString;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)ManufacturerString;
	}

	if(manulength == 0xFF)
		manulength = 30;

	ep0->dwc_ep.xfer_len = manulength;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = manulength;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =manulength;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}



static __inline void get_producter(dwc_otg_pcd_t * pcd, uint8_t prolength) {


	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//uint16_t *status = pcd->status_buf;
	uint8_t i;

	if (GET_CORE_IF(pcd)->dma_enable){
		for(i=0; i<28; i++)
		{
				get_descript[i] = ProducterString[i];
		}

			ep0->dwc_ep.dma_addr = (uint32_t)(&get_descript);
	}
	else
	{
		ep0->dwc_ep.start_xfer_buff = (uint8_t *)ProducterString;
		ep0->dwc_ep.xfer_buff =  (uint8_t *)ProducterString;
	}

	if(prolength == 0xFF)
		prolength = 10;


	ep0->dwc_ep.xfer_len = prolength;
	ep0->dwc_ep.maxpacket = 64;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.total_len = prolength;
	ep0->dwc_ep.sent_zlp = 0;
	ep0->dwc_ep.is_in = 1;
	ep0->dwc_ep.num = 0;

	pcd->ep0_pending = 1;

	ep0->dwc_ep.xfer_len_bak =prolength;
	ep0->dwc_ep.xfer_count_bak =0;
	ep0->dwc_ep.total_len_bak =0;


	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &(ep0->dwc_ep));
}


/**
 * This functions delegates the setup command to the gadget driver.
 */
static __inline void do_gadget_setup(dwc_otg_pcd_t * pcd, usb_device_request_t * ctrl) {

	//int ret = 0;

	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	if (pcd->ep0state == EP0_STALL) {
		return;
	}

//	log_out("set address 1\n");
	pcd->ep0state = EP0_IN_STATUS_PHASE;

	/* Prepare for more SETUP Packets */

	ep0->dwc_ep.xfer_len = 0;
	ep0->dwc_ep.xfer_count = 0;
	ep0->dwc_ep.is_in = 1;
//	ep0->dwc_ep.dma_addr = pcd->setup_pkt_dma_handle;
	dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &ep0->dwc_ep);
}


/**
 *	This function processes SETUP commands. In Linux, the USB Command
 *	processing is done in two places - the first being the PCD and the
 *	second in the Gadget Driver (for example, the File-Backed Storage
 *	Gadget Driver).
 *
 * <table>
 * <tr><td>Command	</td><td>Driver </td><td>Description</td></tr>
 *
 * <tr><td>GET_STATUS </td><td>PCD </td><td>Command is processed as
 * defined in chapter 9 of the USB 2.0 Specification chapter 9
 * </td></tr>
 *
 * <tr><td>CLEAR_FEATURE </td><td>PCD </td><td>The Device and Endpoint
 * requests are the ENDPOINT_HALT feature is procesed, all others the
 * interface requests are ignored.</td></tr>
 *
 * <tr><td>SET_FEATURE </td><td>PCD </td><td>The Device and Endpoint
 * requests are processed by the PCD.  Interface requests are passed
 * to the Gadget Driver.</td></tr>
 *
 * <tr><td>SET_ADDRESS </td><td>PCD </td><td>Program the DCFG reg,
 * with device address received </td></tr>
 *
 * <tr><td>GET_DESCRIPTOR </td><td>Gadget Driver </td><td>Return the
 * requested descriptor</td></tr>
 *
 * <tr><td>SET_DESCRIPTOR </td><td>Gadget Driver </td><td>Optional -
 * not implemented by any of the existing Gadget Drivers.</td></tr>
 *
 * <tr><td>SET_CONFIGURATION </td><td>Gadget Driver </td><td>Disable
 * all EPs and enable EPs for new configuration.</td></tr>
 *
 * <tr><td>GET_CONFIGURATION </td><td>Gadget Driver </td><td>Return
 * the current configuration</td></tr>
 *
 * <tr><td>SET_INTERFACE </td><td>Gadget Driver </td><td>Disable all
 * EPs and enable EPs for new configuration.</td></tr>
 *
 * <tr><td>GET_INTERFACE </td><td>Gadget Driver </td><td>Return the
 * current interface.</td></tr>
 *
 * <tr><td>SYNC_FRAME </td><td>PCD </td><td>Display debug
 * message.</td></tr>
 * </table>
 *
 * When the SETUP Phase Done interrupt occurs, the PCD SETUP commands are
 * processed by pcd_setup. Calling the Function Driver's setup function from
 * pcd_setup processes the gadget SETUP commands.
 */


static __inline void pcd_setup(dwc_otg_pcd_t * pcd,  struct wokoo_udc *udc) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
//	usb_device_request_t ctrl = (usb_device_request_t)setup_packet;
//	usb_device_request_t ctrl;
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	int status = 0;

    struct usb_ctrlrequest  usbreq;
	//char *descriptor;

	deptsiz0_data_t doeptsize0;

	doeptsize0.d32 = 0;
	doeptsize0.d32 = dwc_read_reg32(&dev_if->out_ep_regs[0]->doeptsiz);

	ctrl.bmRequestType = (uByte)setup_packet[0];
	ctrl.bRequest = (uByte)(setup_packet[0]>>8);

	ctrl.wValue[0] = (uByte)(setup_packet[0]>>16);
	ctrl.wValue[1] = (uByte)(setup_packet[0]>>24);
	ctrl.wIndex[0] = (uByte)(setup_packet[1]);
	ctrl.wIndex[1] = (uByte)(setup_packet[1]>>8);
	ctrl.wLength[0] = (uByte)(setup_packet[1]>>16);
	ctrl.wLength[1] = (uByte)(setup_packet[1]>>24);


    usbreq.bRequestType = (uByte)setup_packet[0];
	usbreq.bRequest     = (uByte)(setup_packet[0] >> 8);
	usbreq.wValue  		= (uint16_t)(setup_packet[0] >> 16);
	usbreq.wIndex  		= (uint16_t)setup_packet[1] ;
	usbreq.wLength 		= (uint16_t)(setup_packet[1] >> 16) ;

	if (udc->driver) {
	//spin_lock(&udc->lock);
	status = udc->driver->setup(&udc->gadget, &usbreq);
    //spin_unlock(&udc->lock);
    }



	return;

//	ctrl.wValue = (uByte)(setup_packet[0]>>16)|((uByte)(setup_packet[0]>>24)<<8);
//	ctrl.wIndex = (uWord)setup_packet[1];
//	ctrl.wLength = (uWord)(setup_packet[1]>>16);
	/** @todo handle > 1 setup packet , assert error for now */

	/* Clean up the request queue */


	if (ctrl.bmRequestType & UE_DIR_IN) {
		ep0->dwc_ep.is_in = 1;
		pcd->ep0state = EP0_IN_DATA_PHASE;
	} else {
		ep0->dwc_ep.is_in = 0;
		pcd->ep0state = EP0_OUT_DATA_PHASE;
	}

	if (UGETW(ctrl.wLength) == 0) {
		ep0->dwc_ep.is_in = 1;
		pcd->ep0state = EP0_IN_STATUS_PHASE;
	}

	if(ctrl.bmRequestType == 0x80 && ctrl.bRequest == 0x00)
	{
		do_get_port(pcd);
		pcd->ep0_pending = 0;
		pcd->ep0state = EP0_IN_DATA_PHASE;
		return;
	}

	if(ctrl.bmRequestType == 0xc0 && ctrl.bRequest == 0xFF)
	{

		do_get_c0_ff(pcd);
		pcd->ep0_pending = 0;
		pcd->ep0state = EP0_IN_DATA_PHASE;

		return;
	}

	if ((ctrl.bmRequestType == 0xc1 && ctrl.bRequest == 0x08))
	{
		do_get_c1_08(pcd);
		pcd->ep0_pending = 0;
		pcd->ep0state = EP0_IN_DATA_PHASE;

		return;
	}

//	if(ctrl.bmRequestType== 0x41  && ctrl.bRequest == 0x00)
//			{
//		//		log_out("21 09\n");
//
//				ep0_out_start(core_if, pcd);
//	//			do_setup_out_status_phase(pcd);
//	//			do_gadget_setup(pcd, &ctrl);
//	//			dwc_otg_read_setup_packet(core_if, setup_packet);
//	//			dwc_otg_pcd_handle_out_ep_intr(pcd);
//	//			dwc_otg_read_setup_packet(core_if, setup_packet);
//	//			pcd->ep0state = EP0_OUT_STATUS_PHASE;
//	//			return;
//			}

	if(ctrl.bmRequestType == 0xc1 && ctrl.bRequest == 0x0F)
	{
			do_get_c1(pcd);
			pcd->ep0_pending = 0;
			pcd->ep0state = EP0_IN_DATA_PHASE;

			return;
	}

	if(ctrl.bmRequestType == 0xc0 && ctrl.bRequest == 0x10)
	{
			do_get_c0(pcd);
			pcd->ep0_pending = 0;
			pcd->ep0state = EP0_IN_DATA_PHASE;

			return;
	}

	if (UT_GET_TYPE(ctrl.bmRequestType) != UT_STANDARD) {
		/* handle non-standard (class/vendor) requests in the gadget driver */
		do_gadget_setup(pcd, &ctrl);
		return;
	}

	switch (ctrl.bRequest) {
	case UR_GET_STATUS:
		do_get_status(pcd);
		break;

	case UR_CLEAR_FEATURE:
		do_clear_feature(pcd);
		break;

	case UR_SET_FEATURE:
		do_set_feature(pcd);
		break;

	case UR_SET_ADDRESS:
		pcd->ep0_pending = 0;
		do_set_address(pcd);
		break;
//#------------------------------------
	case UR_GET_DESCRIPTOR:




		if(ctrl.wValue[1] == 0x01)
		{
			do_get_descriptor(pcd);
			//gadget_add_eps(&g_gadget_wrapper);
		}

		if(ctrl.wValue[1] == 0x02)
		{
			do_get_config_conti(pcd, ctrl.wLength[0]);
		}

		if(ctrl.wValue[1] == 0x22)
		{
			get_languageid(pcd);
			log_out("send finish\n");
		}

		if(ctrl.bmRequestType== 0x21  && ctrl.bRequest == 0x09)
		{
			log_out("21 09\n");

			ep0_out_start(core_if, pcd);
//			do_setup_out_status_phase(pcd);
//			do_gadget_setup(pcd, &ctrl);
//			dwc_otg_read_setup_packet(core_if, setup_packet);
//			dwc_otg_pcd_handle_out_ep_intr(pcd);
//			dwc_otg_read_setup_packet(core_if, setup_packet);
//			pcd->ep0state = EP0_OUT_STATUS_PHASE;
//			return;
		}

		if(ctrl.wValue[1] == 0x03)
		{
		//do_get_descriptor(pcd);
			if(ctrl.wValue[0] == 0x00)
				get_languageid(pcd);
			if(ctrl.wValue[0] == 0x01)
				get_serialnumber(pcd);
			if(ctrl.wValue[0] == 0x02)
				get_manufacture(pcd, ctrl.wLength[0]);
			if(ctrl.wValue[0] == 0x03)
				get_producter(pcd, ctrl.wLength[0]);
		}

		pcd->ep0_pending = 0;

		pcd->ep0state = EP0_IN_DATA_PHASE;
		break;

	case UR_SET_INTERFACE:
	case UR_SET_CONFIG:
		do_gadget_setup(pcd, &ctrl);
		break;

	case UR_SYNCH_FRAME:
		do_gadget_setup(pcd, &ctrl);
		break;

	default:
		/* Call the Gadget Driver's setup functions */
		do_gadget_setup(pcd, &ctrl);
		break;
	}
}

static __inline void pcd_setup_DMA(dwc_otg_pcd_t * pcd) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
//	usb_device_request_t ctrl = (usb_device_request_t)setup_packet;
//	usb_device_request_t ctrl;
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//char *descriptor;
	//uint32_t delay =0;
	deptsiz0_data_t doeptsize0;
//	log_out(" 1\n");

	doeptsize0.d32 = 0;
	doeptsize0.d32 = dwc_read_reg32(&dev_if->out_ep_regs[0]->doeptsiz);

	ctrl.bmRequestType = (uByte)setup_pkt_bak[0];
	ctrl.bRequest = (uByte)(setup_pkt_bak[0]>>8);

	ctrl.wValue[0] = (uByte)(setup_pkt_bak[0]>>16);
	ctrl.wValue[1] = (uByte)(setup_pkt_bak[0]>>24);
	ctrl.wIndex[0] = (uByte)(setup_pkt_bak[1]);
	ctrl.wIndex[1] = (uByte)(setup_pkt_bak[1]>>8);
	ctrl.wLength[0] = (uByte)(setup_pkt_bak[1]>>16);
	ctrl.wLength[1] = (uByte)(setup_pkt_bak[1]>>24);

//	ctrl.wValue = (uByte)(setup_packet[0]>>16)|((uByte)(setup_packet[0]>>24)<<8);
//	ctrl.wIndex = (uWord)setup_packet[1];
//	ctrl.wLength = (uWord)(setup_packet[1]>>16);
	/** @todo handle > 1 setup packet , assert error for now */

	if (core_if->dma_enable && core_if->dma_desc_enable == 0
			&& (doeptsize0.b.supcnt < 2)) {

	}
//	log_out("2\n");
	/* Clean up the request queue */
	dwc_otg_request_nuke(ep0);
	ep0->stopped = 0;

	if (ctrl.bmRequestType & UE_DIR_IN) {
		ep0->dwc_ep.is_in = 1;
		pcd->ep0state = EP0_IN_DATA_PHASE;
	} else {
		ep0->dwc_ep.is_in = 0;
		pcd->ep0state = EP0_OUT_DATA_PHASE;
	}

//	log_out("set up pos 3\n");
	if (UGETW(ctrl.wLength) == 0) {
		ep0->dwc_ep.is_in = 1;
		pcd->ep0state = EP0_IN_STATUS_PHASE;
	}

//	log_out("set up pos 4\n");
	if (UT_GET_TYPE(ctrl.bmRequestType) != UT_STANDARD) {


		/* handle non-standard (class/vendor) requests in the gadget driver */
		do_gadget_setup(pcd, &ctrl);
		return;
	}

//	reg_val_out("pcd->ep0state", pcd->ep0state);
//	log_out("set up pos 5\n");
	/** @todo NGS: Handle bad setup packet? */
//	reg_val_out( "ctrl.bmRequestType=", ctrl.bmRequestType );
//	reg_val_out( "ctrl.bRequest=", ctrl.bRequest );
//	reg_val_out( "ctrl.wValue=", UGETW(ctrl.wValue) );
//	reg_val_out( "ctrl.wIndex=", UGETW(ctrl.wIndex) );
//	reg_val_out( "ctrl.wLength=", UGETW(ctrl.wLength) );

///////////////////////////////////////////
//// --- Standard Request handling --- ////
	switch (ctrl.bRequest) {
	case UR_GET_STATUS:
		log_out("req= get status\n");
//		do_get_status(pcd);

		break;

	case UR_CLEAR_FEATURE:
//		log_out("req= clear feature\n");
		do_clear_feature(pcd);
		break;

	case UR_SET_FEATURE:
//		log_out("req= set feature\n");
		do_set_feature(pcd);
		break;

	case UR_SET_ADDRESS:
		pcd->ep0_pending = 0;
//		log_out("req= set address\n");
		do_set_address(pcd);
		break;
//#------------------------------------
	case UR_GET_DESCRIPTOR:
	//	log_out("do get descriptor\n");

		if(ctrl.wValue[1] == 0x01)
		{
			do_get_descriptor(pcd);
		//	gadget_add_eps(&g_gadget_wrapper);
		}
		if(ctrl.wValue[1] == 0x02)
		{
			do_get_config_conti(pcd, ctrl.wLength[0]);
		}


		if(ctrl.wValue[1] == 0x03)
		{
			//do_get_descriptor(pcd);
			if(ctrl.wValue[0] == 0x00)
				get_languageid(pcd);
			if(ctrl.wValue[0] == 0x01)
				get_serialnumber(pcd);
			if(ctrl.wValue[0] == 0x02)
				get_manufacture(pcd, ctrl.wLength[0]);
			if(ctrl.wValue[0] == 0x03)
				get_producter(pcd, ctrl.wLength[0]);
		}

		pcd->ep0_pending = 0;

		pcd->ep0state = EP0_IN_DATA_PHASE;


		break;
//###-------------------------------end
	case UR_SET_INTERFACE:
	case UR_SET_CONFIG:
//		log_out("req= gadget setup 1\n");
		do_gadget_setup(pcd, &ctrl);
		break;

	case UR_SYNCH_FRAME:
//		log_out("req= gadget setup 2\n");
		do_gadget_setup(pcd, &ctrl);
		break;

	default:
		/* Call the Gadget Driver's setup functions */
//		log_out("req= gadget setup 3\n");
		do_gadget_setup(pcd, &ctrl);
		break;
	}

//	for(delay= 0; delay<500;delay++);
	ep0_out_start(core_if, pcd);
}


/**
 * This interrupt occurs when the non-periodic Tx FIFO is half-empty.
 * The active request is checked for the next packet to be loaded into
 * the non-periodic Tx FIFO.
 */
int32_t dwc_otg_pcd_handle_np_tx_fifo_empty_intr(dwc_otg_pcd_t * pcd) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	dwc_otg_dev_in_ep_regs_t *ep_regs;
	gnptxsts_data_t txstatus;
	gintsts_data_t gintsts;

	int epnum = 0;
	dwc_otg_pcd_ep_t *ep = 0;
	uint32_t len = 0;
	int dwords;
	txstatus.d32 = 0;
//	log_out("tx fifo empty pos 1\n");
	/* Get the epnum from the IN Token Learning Queue. */
	epnum = get_ep_of_last_in_token(core_if);
	ep = get_in_ep(pcd, epnum);

	ep_regs = core_if->dev_if->in_ep_regs[epnum];
//	log_out("tx fifo empty pos 2\n");
	len = ep->dwc_ep.xfer_len - ep->dwc_ep.xfer_count;
	if (len > ep->dwc_ep.maxpacket) {
		len = ep->dwc_ep.maxpacket;
	}
	dwords = (len + 3) / 4;

	/* While there is space in the queue and space in the FIFO and
	 * More data to tranfer, Write packets to the Tx FIFO */
	txstatus.d32 = dwc_read_reg32(&global_regs->gnptxsts);
//	log_out("tx fifo empty pos 3\n");
	while (txstatus.b.nptxqspcavail > 0 && txstatus.b.nptxfspcavail > dwords
			&& ep->dwc_ep.xfer_count < ep->dwc_ep.xfer_len) {
		/* Write the FIFO */
//		log_out("tx fifo empty pos 4 loop\n");
		dwc_otg_ep_write_packet(core_if, &ep->dwc_ep, 0);
		len = ep->dwc_ep.xfer_len - ep->dwc_ep.xfer_count;

		if (len > ep->dwc_ep.maxpacket) {
			len = ep->dwc_ep.maxpacket;
		}

		dwords = (len + 3) / 4;
		txstatus.d32 = dwc_read_reg32(&global_regs->gnptxsts);

	}
//	log_out("tx fifo empty pos 5\n");
	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.nptxfempty = 1;
	dwc_write_reg32(&global_regs->gintsts, gintsts.d32);

	(void) ep_regs;
	return 1;
}
/**
 * This function is called when dedicated Tx FIFO Empty interrupt occurs.
 * The active request is checked for the next packet to be loaded into
 * apropriate Tx FIFO.
 */

uint8_t  enum1_cycle = 0;
static int32_t write_empty_tx_fifo(dwc_otg_pcd_t * pcd, uint32_t epnum) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	dwc_otg_dev_in_ep_regs_t *ep_regs;
	dtxfsts_data_t txstatus;
	dwc_otg_pcd_ep_t *ep = 0;
	int32_t len = 0;
//	uint32_t temp =0;
	depctl_data_t depctl;
	volatile uint32_t *addr;
	int dwords,j;
	txstatus.d32 = 0;

	if(epnum ==1)
	{
		addr = &dev_if->in_ep_regs[1]->diepctl;

		depctl.d32 = dwc_read_reg32(addr);

		for(j=0; j<0x2000;j++);
		depctl.b.epdis =0;
		dwc_write_reg32(addr, depctl.d32);
		NUM1_sendf = 1;
	}


	ep = get_in_ep(pcd, epnum);
		ep_regs = core_if->dev_if->in_ep_regs[epnum];

//		reg_val_out( "ep", (uint32_t)ep );

//		log_out("tx 2");
		ep->dwc_ep.xfer_len = ep->dwc_ep.xfer_len_bak;
		ep->dwc_ep.xfer_count = ep->dwc_ep.xfer_count_bak;



//		reg_val_out( "ep->dwc_ep.xfer_len_bak", ep->dwc_ep.xfer_len_bak );
//		reg_val_out( "ep->dwc_ep.xfer_count_bak", ep->dwc_ep.xfer_count_bak );
//		reg_val_out( "ep->dwc_ep.xfer_len", ep->dwc_ep.xfer_len );
//		reg_val_out( "ep->dwc_ep.xfer_count", ep->dwc_ep.xfer_count );
//		reg_val_out( "ep->dwc_ep.maxpacket", ep->dwc_ep.maxpacket );

//		log_out("tx 3");
		len = ep->dwc_ep.xfer_len - ep->dwc_ep.xfer_count;
		if (len > ep->dwc_ep.maxpacket) {
			len = ep->dwc_ep.maxpacket;
			if(epnum == 0)
			Enumpacket64 = 1;
		}

		dwords = (len + 3) / 4;

		/* While there is space in the queue and space in the FIFO and
		 * More data to tranfer, Write packets to the Tx FIFO */


		txstatus.d32 = dwc_read_reg32(&dev_if->in_ep_regs[epnum]->dtxfsts);
		if(epnum != 0)
		{
			while (txstatus.b.txfspcavail > dwords
					&& ep->dwc_ep.xfer_count < ep->dwc_ep.xfer_len
					&& ep->dwc_ep.xfer_len != 0) {
				/* Write the FIFO */
				dwc_otg_ep_write_packet(core_if, &ep->dwc_ep, 0);
				len = ep->dwc_ep.xfer_len - ep->dwc_ep.xfer_count;
				if (len > ep->dwc_ep.maxpacket) {
					len = ep->dwc_ep.maxpacket;
				}
		//		log_out("tx 5");
				dwords = (len + 3) / 4;
				txstatus.d32 =
				dwc_read_reg32(&dev_if->in_ep_regs[epnum]->dtxfsts);

			}
		}
		else
		{

			dwc_otg_ep_write_packet(core_if, &ep->dwc_ep, 0);
			len = ep->dwc_ep.xfer_len - ep->dwc_ep.xfer_count;
			if (len > ep->dwc_ep.maxpacket) {
					len = ep->dwc_ep.maxpacket;
			}
		//	log_out("tx 5");
			dwords = (len + 3) / 4;
			txstatus.d32 =
			dwc_read_reg32(&dev_if->in_ep_regs[epnum]->dtxfsts);
		}




/*		if(epnum ==1 && ep->dwc_ep.xfer_len_bak == 0)
		{
//			device_grxsts_data_t status;
//			dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
			volatile uint32_t *addr1, *addr2;

					log_out("=====in4 \n");
			addr1 = &dev_if->in_ep_regs[1]->diepctl;
			addr2 = &dev_if->out_ep_regs[1]->doepctl;

			depctl.d32 = dwc_read_reg32(addr1);

			depctl.b.epdis =1;
			depctl.b.snak =0;
			depctl.b.cnak = 1;
			depctl.b.epena =1;
			dwc_write_reg32(addr1, depctl.d32);
//			reg_val_out("depctl", depctl.d32);

			depctl.d32 = dwc_read_reg32(addr2);
			depctl.b.epdis =0;
			depctl.b.snak =0;
			depctl.b.cnak = 1;
			depctl.b.epena =1;
			dwc_write_reg32(addr2, depctl.d32);
//			reg_val_out("depctl", depctl.d32);

		}*/

		if(Enumpacket64 == 1)
		{
			ep->dwc_ep.xfer_count =64;
			ep->dwc_ep.xfer_len_bak = ep->dwc_ep.xfer_len_bak - 64;
			ep->dwc_ep.xfer_count_bak = 00;
			ep->dwc_ep.xfer_buff =  ep->dwc_ep.xfer_buff_bak+64;
			Enumpacket64 = 0;

		}
		else{
			ep->dwc_ep.xfer_len_bak = 0;
			ep->dwc_ep.xfer_count_bak = 0;

		}


		(void) ep_regs;
		return 1;
}


/**
 * This function checks the EP request queue, if the queue is not
 * empty the next request is started.
 */
void start_next_request(dwc_otg_pcd_ep_t * ep) {

	dwc_otg_pcd_request_t *req = 0;
	uint32_t max_transfer =	GET_CORE_IF(ep->pcd)->core_params->max_transfer_size;


	if (!DWC_CIRCLEQ_EMPTY(&ep->queue)) {

		req = DWC_CIRCLEQ_FIRST(&ep->queue);

			/* Setup and start the Transfer */
			ep->dwc_ep.dma_addr = req->dma;
			ep->dwc_ep.start_xfer_buff = req->buf;
			ep->dwc_ep.xfer_buff = (uint8_t *) (req->buf);
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = req->length;
			ep->dwc_ep.xfer_len = 0;
			ep->dwc_ep.xfer_count = 0;

			ep->dwc_ep.maxxfer = max_transfer;
			if (GET_CORE_IF(ep->pcd)->dma_desc_enable) {
				uint32_t out_max_xfer = DDMA_MAX_TRANSFER_SIZE
						- (DDMA_MAX_TRANSFER_SIZE % 4);
				if (ep->dwc_ep.is_in) {
					if (ep->dwc_ep.maxxfer >
					DDMA_MAX_TRANSFER_SIZE) {
						ep->dwc_ep.maxxfer =
						DDMA_MAX_TRANSFER_SIZE;
					}
				} else {
					if (ep->dwc_ep.maxxfer > out_max_xfer) {
						ep->dwc_ep.maxxfer = out_max_xfer;
					}
				}
			}
			if (ep->dwc_ep.maxxfer < ep->dwc_ep.total_len) {
				ep->dwc_ep.maxxfer -=
						(ep->dwc_ep.maxxfer % ep->dwc_ep.maxpacket);
			}
			if (req->sent_zlp) {
				if ((ep->dwc_ep.total_len % ep->dwc_ep.maxpacket == 0)
						&& (ep->dwc_ep.total_len != 0)) {
					ep->dwc_ep.sent_zlp = 1;
				}

			}
		dwc_otg_ep_start_transfer(GET_CORE_IF(ep->pcd), &ep->dwc_ep);

	}

}

/**
 * This function handles the Rx Status Queue Level Interrupt, which
 * indicates that there is a least one packet in the Rx FIFO.  The
 * packets are moved from the FIFO to memory, where they will be
 * processed when the Endpoint Interrupt Register indicates Transfer
 * Complete or SETUP Phase Done.
 *
 * Repeat the following until the Rx Status Queue is empty:
 *	 -# Read the Receive Status Pop Register (GRXSTSP) to get Packet
 *		info
 *	 -# If Receive FIFO is empty then skip to step Clear the interrupt
 *		and exit
 *	 -# If SETUP Packet call dwc_otg_read_setup_packet to copy the
 *		SETUP data to the buffer
 *	 -# If OUT Data Packet call dwc_otg_read_packet to copy the data
 *		to the destination buffer
 */


int32_t dwc_otg_pcd_handle_rx_status_q_level_intr(dwc_otg_pcd_t * pcd, struct wokoo_udc *udc) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	gintmsk_data_t gintmask;
	device_grxsts_data_t status;
	dwc_otg_pcd_ep_t *ep;
	gintsts_data_t gintsts;
    unsigned int bufferspace;
    struct wokoo_ep		*ep0 = &udc->ep[0];
    struct wokoo_request	*req;
    u8		*buf;

//	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
//				dwc_otg_dev_if_t *dev_if = core_if->dev_if;
//				usb_device_request_t ctrl = pcd->setup_pkt->req;
	//			dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;

	gintmask.d32 = 0;

	/* Disable the Rx Status Queue Level interrupt */
	gintmask.b.rxstsqlvl = 1;
	dwc_modify_reg32(&global_regs->gintmsk, gintmask.d32, 0);

	/* Get the Status from the top of the FIFO */
	status.d32 = dwc_read_reg32(&global_regs->grxstsp);

	/* Get pointer to EP structure */
	ep = get_out_ep(pcd, status.b.epnum);

	if(status.b.epnum == 0) {
		switch (status.b.pktsts) {
		case DWC_DSTS_GOUT_NAK:
        	printk("----------1111111111----------");
			break;

		case DWC_STS_DATA_UPDT:
        	printk("----------2222222222----------");
			if (status.b.bcnt && ep->dwc_ep.xfer_buff) {
				/** @todo NGS Check for buffer overflow? */
				dwc_otg_read_packet(core_if, (uint32_t *)ep->dwc_ep.xfer_buff, status.b.bcnt);
				ep->dwc_ep.xfer_count += status.b.bcnt;
				ep->dwc_ep.xfer_buff += status.b.bcnt;
			}
			break;

		case DWC_STS_XFER_COMP:
			printk("----------3333333333----------");
			break;

		case DWC_DSTS_SETUP_COMP:
            printk("----------4444444444----------");
		/*	if (list_empty(&ep0->queue))
				req = NULL;
			else {
				req = list_entry(ep0->queue.next, struct wokoo_request, queue);
			}

			if (req) {
                buf = req->req.buf + req->req.actual;
				bufferspace = req->req.length - req->req.actual;
				done(ep0, req, 0);
            }*/

			do_setup_in_status_phase(pcd);
		#if 0
			if (deal_0_bulkout == 1) {
				senddata_flag = 1;
				deal_0_bulkout = 0;
				do_setup_in_status_phase(pcd);
			}

			if (deal_0_bulkout == 2 || deal_0_bulkout == 3 || deal_0_bulkout == 4) {
				deal_0_bulkout = 0;
				do_setup_in_status_phase(pcd);
			}
		#endif

			break;

		case DWC_DSTS_SETUP_UPDT:
			printk("----------5555555555----------");

		//	nuke(ep0, 0);
        //    udc->req_pending = 0;
            //handle_setup(udc, ep0);

		udc->ep[0].is_in = 1;
		dwc_otg_read_setup_packet(core_if, setup_packet);
        pcd_setup(pcd, udc);

		#if 0
			dwc_otg_read_setup_packet(core_if, setup_packet);

			if(setup_packet[0] == 0x02000921)
			{
				deal_0_bulkout = 2;
				ep0_out_start(core_if, pcd);
				break;
			}

			if(setup_packet[0] == 0x00001341 || setup_packet[0] == 0x00001941 || setup_packet[0] == 0x00001E41)
			{
				if(setup_packet[0] == 0x00001341)
					deal_0_bulkout = 2;
				if(setup_packet[0] == 0x00001941)
					deal_0_bulkout = 3;
				if(setup_packet[0] == 0x00001E41)
					deal_0_bulkout = 4;

            	ep0_out_start(core_if, pcd);
				break;
			}

			ep->dwc_ep.xfer_count += status.b.bcnt;

			pcd_setup(pcd);
        #endif

			break;

		default:
			printk("----------6666666666----------");
			break;
		}
	} else {
		switch (status.b.pktsts) {
		case DWC_DSTS_GOUT_NAK:
			break;

		case DWC_STS_DATA_UPDT:
			/** @todo NGS Check for buffer overflow? */
			dwc_otg_read_packet(core_if, out_packet, status.b.bcnt);

			//		do_endpoint1_trans( pcd, status.b.bcnt);
			sendnum = status.b.bcnt;
			recvflag = 1;
			break;

		case DWC_STS_XFER_COMP:
			break;

		case DWC_DSTS_SETUP_COMP:
			break;

		case DWC_DSTS_SETUP_UPDT:
			dwc_otg_read_setup_packet(core_if, setup_packet);

			ep->dwc_ep.xfer_count += status.b.bcnt;

			pcd_setup(pcd,udc);
			break;

		default:
			break;
		}
	}

	/* Enable the Rx Status Queue Level interrupt */
	dwc_modify_reg32(&global_regs->gintmsk, 0, gintmask.d32);

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.rxstsqlvl = 1;
	dwc_write_reg32(&global_regs->gintsts, gintsts.d32);

	return 1;
}

/**
 * This interrupt indicates that ...
 */
int32_t dwc_otg_pcd_handle_early_suspend_intr(dwc_otg_pcd_t * pcd) {

	gintsts_data_t gintsts;
	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.erlysuspend = 1;
	dwc_write_reg32(&GET_CORE_IF(pcd)->core_global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This interrupt occurs when a USB Reset is detected. When the USB
 * Reset Interrupt occurs the device state is set to DEFAULT and the
 * EP0 state is set to IDLE.
 *	-#	Set the NAK bit for all OUT endpoints (DOEPCTLn.SNAK = 1)
 *	-#	Unmask the following interrupt bits
 *		- DAINTMSK.INEP0 = 1 (Control 0 IN endpoint)
 *	- DAINTMSK.OUTEP0 = 1 (Control 0 OUT endpoint)
 *	- DOEPMSK.SETUP = 1
 *	- DOEPMSK.XferCompl = 1
 *	- DIEPMSK.XferCompl = 1
 *	- DIEPMSK.TimeOut = 1
 *	-# Program the following fields in the endpoint specific registers
 *	for Control OUT EP 0, in order to receive a setup packet
 *	- DOEPTSIZ0.Packet Count = 3 (To receive up to 3 back to back
 *	  setup packets)
 *	- DOEPTSIZE0.Transfer Size = 24 Bytes (To receive up to 3 back
 *	  to back setup packets)
 *		- In DMA mode, DOEPDMA0 Register with a memory address to
 *		  store any setup packets received
 * At this point, all the required initialization, except for enabling
 * the control 0 OUT endpoint is done, for receiving SETUP packets.
 */
int32_t dwc_otg_pcd_handle_usb_reset_intr(dwc_otg_pcd_t * pcd) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	depctl_data_t doepctl;
	daint_data_t daintmsk;
	doepmsk_data_t doepmsk;
	diepmsk_data_t diepmsk;
	dcfg_data_t dcfg;
	grstctl_t resetctl;
	dctl_data_t dctl;
	int i = 0;
	gintsts_data_t gintsts;
	pcgcctl_data_t power;

	doepctl.d32 = 0;
	daintmsk.d32 = 0;
	doepmsk.d32 = 0;
	diepmsk.d32 = 0;
	dcfg.d32 = 0;
	resetctl.d32 = 0;
	dctl.d32 = 0;
	power.d32 = 0;

	power.d32 = dwc_read_reg32(core_if->pcgcctl);
	if (power.b.stoppclk) {
		power.d32 = 0;
		power.b.stoppclk = 1;
		dwc_modify_reg32(core_if->pcgcctl, power.d32, 0);

		power.b.pwrclmp = 1;
		dwc_modify_reg32(core_if->pcgcctl, power.d32, 0);

		power.b.rstpdwnmodule = 1;
		dwc_modify_reg32(core_if->pcgcctl, power.d32, 0);
//		log_out("power\n");
	}

	core_if->lx_state = DWC_OTG_L0;

	/* reset the HNP settings */
//	dwc_otg_pcd_update_otg(pcd, 1);

	/* Clear the Remote Wakeup Signalling */
	dctl.b.rmtwkupsig = 1;
	dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dctl, dctl.d32, 0);

	/* Set NAK for all OUT EPs */
	doepctl.b.snak = 1;

//	for (i = 0; i <= dev_if->num_out_eps; i++) {
	for (i = 0; i <= 2; i++) {
		dwc_write_reg32(&dev_if->out_ep_regs[i]->doepctl, doepctl.d32);
	//	reg_val_out("doepctl", doepctl.d32);
	}

	/* Flush the NP Tx FIFO */
	dwc_otg_flush_tx_fifo(core_if, 0x10);
	/* Flush the Learning Queue */
	resetctl.b.intknqflsh = 1;
	dwc_write_reg32(&core_if->core_global_regs->grstctl, resetctl.d32);

	if (core_if->multiproc_int_enable) {
		daintmsk.b.inep0 = 1;
		daintmsk.b.outep0 = 1;
		dwc_write_reg32(&dev_if->dev_global_regs->deachintmsk, daintmsk.d32);

		doepmsk.b.setup = 1;
		doepmsk.b.xfercompl = 1;
		doepmsk.b.ahberr = 1;
		doepmsk.b.epdisabled = 1;

		if (core_if->dma_desc_enable) {
			doepmsk.b.stsphsercvd = 1;
			//doepmsk.b.bna = 1;
		}
		/*
		 doepmsk.b.babble = 1;
		 doepmsk.b.nyet = 1;

		 if (core_if->dma_enable) {
		 doepmsk.b.nak = 1;
		 }
		 */
		dwc_write_reg32(&dev_if->dev_global_regs->doepeachintmsk[0], doepmsk.d32);

		diepmsk.b.xfercompl = 1;
		diepmsk.b.timeout = 1;
		diepmsk.b.epdisabled = 1;
		diepmsk.b.ahberr = 1;
		diepmsk.b.intknepmis = 1;

		/*		if (core_if->dma_desc_enable) {
		 diepmsk.b.bna = 1;
		 }
		 */
		/*
		 if (core_if->dma_enable) {
		 diepmsk.b.nak = 1;
		 }
		 */
		dwc_write_reg32(&dev_if->dev_global_regs->diepeachintmsk[0],
				diepmsk.d32);
	} else {

		daintmsk.b.inep0 = 1;
		daintmsk.b.outep0 = 1;
		dwc_write_reg32(&dev_if->dev_global_regs->daintmsk, daintmsk.d32);

		doepmsk.b.setup = 1;
		doepmsk.b.xfercompl = 1;
		doepmsk.b.ahberr = 1;
		doepmsk.b.epdisabled = 1;

		if (core_if->dma_desc_enable) {
			doepmsk.b.stsphsercvd = 1;
			//doepmsk.b.bna = 1;
		}
		dwc_write_reg32(&dev_if->dev_global_regs->doepmsk, doepmsk.d32);
	//	reg_val_out("doepmsk.d32", doepmsk.d32);

		diepmsk.b.xfercompl = 1;
		diepmsk.b.timeout = 1;
		diepmsk.b.epdisabled = 1;
		diepmsk.b.ahberr = 1;
		diepmsk.b.intknepmis = 1;
		/*
		 if (core_if->dma_desc_enable) {
		 diepmsk.b.bna = 1;
		 }
		 */

		dwc_write_reg32(&dev_if->dev_global_regs->diepmsk, diepmsk.d32);
	//	reg_val_out("diepmsk.d32", diepmsk.d32);

	}

	/* Reset Device Address */
	dcfg.d32 = dwc_read_reg32(&dev_if->dev_global_regs->dcfg);
	dcfg.b.devaddr = 0;
	dwc_write_reg32(&dev_if->dev_global_regs->dcfg, dcfg.d32);
//	reg_val_out("dcfg.d32", dcfg.d32);


	/* setup EP0 to receive SETUP packets */
	ep0_out_start(core_if, pcd);

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.usbreset = 1;
	gintsts.b.inepint = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);
//	reg_val_out("gintsts.d32", gintsts.d32);

	return 1;
}

/**
 * Get the device speed from the device status register and convert it
 * to USB speed constant.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static int get_device_speed(dwc_otg_core_if_t * core_if) {
	dsts_data_t dsts;
	int speed = 0;
	dsts.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dsts);

	switch (dsts.b.enumspd) {
	case DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ:
//		log_out("dev speed is high\n");
		speed = USB_SPEED_HIGH;
		break;
	case DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_48MHZ:
	//	log_out("dev speed is FULL\n");
		speed = USB_SPEED_FULL;
		break;

	case DWC_DSTS_ENUMSPD_LS_PHY_6MHZ:
//		log_out("dev speed is LOW\n");
		speed = USB_SPEED_LOW;
		break;
	}

	return speed;
}

/**
 * Read the device status register and set the device speed in the
 * data structure.
 * Set up EP0 to receive SETUP packets by calling dwc_ep0_activate.
 */
int32_t dwc_otg_pcd_handle_enum_done_intr(dwc_otg_pcd_t * pcd) {
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	gintsts_data_t gintsts;
	gusbcfg_data_t gusbcfg;
	dwc_otg_core_global_regs_t *global_regs =
	GET_CORE_IF(pcd)->core_global_regs;
	uint8_t utmi16b, utmi8b;
	int speed;

	if (GET_CORE_IF(pcd)->snpsid >= OTG_CORE_REV_2_60a) {
		utmi16b = 6;
		utmi8b = 9;
	} else {
		utmi16b = 4;
		utmi8b = 8;
	}
	dwc_otg_ep0_activate(GET_CORE_IF(pcd), &ep0->dwc_ep);


	if (pcd->ep0state == EP0_DISCONNECT) {
		pcd->ep0state = EP0_IDLE;
	} else if (pcd->ep0state == EP0_STALL) {
		pcd->ep0state = EP0_IDLE;
	}

	pcd->ep0state = EP0_IDLE;

	ep0->stopped = 0;
	speed = get_device_speed(GET_CORE_IF(pcd));

//	pcd->fops->connect(pcd, speed);

	/* Set USB turnaround time based on device speed and PHY interface. */
	gusbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
	if (speed == USB_SPEED_HIGH) {
		log_out("high speed\n");
		if (GET_CORE_IF(pcd)->hwcfg2.b.hs_phy_type ==
		DWC_HWCFG2_HS_PHY_TYPE_ULPI) {
			/* ULPI interface */
			gusbcfg.b.usbtrdtim = 9;
		}
		if (GET_CORE_IF(pcd)->hwcfg2.b.hs_phy_type ==
		DWC_HWCFG2_HS_PHY_TYPE_UTMI) {
			/* UTMI+ interface */
			if (GET_CORE_IF(pcd)->hwcfg4.b.utmi_phy_data_width == 0) {
				gusbcfg.b.usbtrdtim = utmi8b;
			} else if (GET_CORE_IF(pcd)->hwcfg4.b.utmi_phy_data_width == 1) {
				gusbcfg.b.usbtrdtim = utmi16b;
			} else if (GET_CORE_IF(pcd)->core_params->phy_utmi_width == 8) {
				gusbcfg.b.usbtrdtim = utmi8b;
			} else {
				gusbcfg.b.usbtrdtim = utmi16b;
			}
		}
		if (GET_CORE_IF(pcd)->hwcfg2.b.hs_phy_type ==
		DWC_HWCFG2_HS_PHY_TYPE_UTMI_ULPI) {
			/* UTMI+  OR  ULPI interface */
			if (gusbcfg.b.ulpi_utmi_sel == 1) {
				/* ULPI interface */
				gusbcfg.b.usbtrdtim = 9;
			} else {
				/* UTMI+ interface */
				if (GET_CORE_IF(pcd)->core_params->phy_utmi_width == 16) {
					gusbcfg.b.usbtrdtim = utmi16b;
				} else {
					gusbcfg.b.usbtrdtim = utmi8b;
				}
			}
		}
	} else {
	//	log_out("full or low speed\n");
		/* Full or low speed */
		gusbcfg.b.usbtrdtim = 9;
	}

	//gusbcfg.b.usbtrdtim = 10;
	dwc_write_reg32(&global_regs->gusbcfg, gusbcfg.d32);
//	reg_val_out("gusbcfg.d32", gusbcfg.d32);
	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.enumdone = 1;
	dwc_write_reg32(&GET_CORE_IF(pcd)->core_global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This interrupt indicates the end of the portion of the micro-frame
 * for periodic transactions.  If there is a periodic transaction for
 * the next frame, load the packets into the EP periodic Tx FIFO.
 */
int32_t dwc_otg_pcd_handle_end_periodic_frame_intr(dwc_otg_pcd_t * pcd) {
	gintmsk_data_t intr_mask;
	gintsts_data_t gintsts;
	intr_mask.d32 = 0;


	intr_mask.b.eopframe = 1;
	dwc_modify_reg32(&GET_CORE_IF(pcd)->core_global_regs->gintmsk,
			intr_mask.d32, 0);

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.eopframe = 1;
	dwc_write_reg32(&GET_CORE_IF(pcd)->core_global_regs->gintsts, gintsts.d32);

	return 1;
}

/**
 * Restart transfer
 */
static void restart_transfer(dwc_otg_pcd_t * pcd, const uint32_t epnum) {
	dwc_otg_core_if_t *core_if;
	dwc_otg_dev_if_t *dev_if;
	deptsiz_data_t dieptsiz;
	dwc_otg_pcd_ep_t *ep;
	dieptsiz.d32 = 0;
	ep = get_in_ep(pcd, epnum);
	core_if = GET_CORE_IF(pcd);
	dev_if = core_if->dev_if;

	dieptsiz.d32 = dwc_read_reg32(&dev_if->in_ep_regs[epnum]->dieptsiz);

	/*
	 * If xfersize is 0 and pktcnt in not 0, resend the last packet.
	 */
	if (dieptsiz.b.pktcnt && dieptsiz.b.xfersize == 0
			&& ep->dwc_ep.start_xfer_buff != 0) {
		if (ep->dwc_ep.total_len <= ep->dwc_ep.maxpacket) {
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.xfer_buff = ep->dwc_ep.start_xfer_buff;
			ep->dwc_ep.xfer_len = ep->dwc_ep.xfer_count;
		} else {
			ep->dwc_ep.xfer_count -= ep->dwc_ep.maxpacket;
			/* convert packet size to dwords. */
			ep->dwc_ep.xfer_buff -= ep->dwc_ep.maxpacket;
			ep->dwc_ep.xfer_len = ep->dwc_ep.xfer_count;
		}
		ep->stopped = 0;

		if (epnum == 0) {
			dwc_otg_ep0_start_transfer(core_if, &ep->dwc_ep);
		} else {
			dwc_otg_ep_start_transfer(core_if, &ep->dwc_ep);
		}
	}
}

/**
 * handle the IN EP disable interrupt.
 */
static __inline void handle_in_ep_disable_intr(dwc_otg_pcd_t * pcd,	const uint32_t epnum) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	deptsiz_data_t dieptsiz;
	dctl_data_t dctl;
	dwc_otg_pcd_ep_t *ep;
	dwc_ep_t *dwc_ep;
	dieptsiz.d32 = 0;
	dctl.d32 = 0;
	ep = get_in_ep(pcd, epnum);
	dwc_ep = &ep->dwc_ep;

	if (dwc_ep->type == DWC_OTG_EP_TYPE_ISOC) {
		dwc_otg_flush_tx_fifo(core_if, dwc_ep->tx_fifo_num);
		return;
	}

	dieptsiz.d32 = dwc_read_reg32(&dev_if->in_ep_regs[epnum]->dieptsiz);

	if (ep->stopped) {
		/* Flush the Tx FIFO */
		dwc_otg_flush_tx_fifo(core_if, dwc_ep->tx_fifo_num);
		/* Clear the Global IN NP NAK */
		dctl.d32 = 0;
		dctl.b.cgnpinnak = 1;
		dwc_modify_reg32(&dev_if->dev_global_regs->dctl, dctl.d32, 0);
		/* Restart the transaction */
		if (dieptsiz.b.pktcnt != 0 || dieptsiz.b.xfersize != 0) {
			restart_transfer(pcd, epnum);
		}
	} else {
		/* Restart the transaction */
		if (dieptsiz.b.pktcnt != 0 || dieptsiz.b.xfersize != 0) {
			restart_transfer(pcd, epnum);
		}

	}
}

/**
 * This function completes the ep0 control transfer.
 */
static int32_t ep0_complete_request(dwc_otg_pcd_ep_t * ep) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(ep->pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	dwc_otg_dev_in_ep_regs_t *in_ep_regs = dev_if->in_ep_regs[ep->dwc_ep.num];

	deptsiz0_data_t deptsiz;
	dev_dma_desc_sts_t desc_sts;
	dwc_otg_pcd_request_t *req = NULL;
	int is_last = 0;
	dwc_otg_pcd_t *pcd = ep->pcd;

	desc_sts.b.bytes = 0;

	if (pcd->ep0_pending ) {
		if (ep->dwc_ep.is_in) {
			do_setup_out_status_phase(pcd);
		} else {
			log_out("set up send 5");
			do_setup_in_status_phase(pcd);
		}
		pcd->ep0_pending = 0;
		return 1;
	}

	if (pcd->ep0state == EP0_OUT_STATUS_PHASE || pcd->ep0state == EP0_IN_STATUS_PHASE) {
		is_last = 1;
	} else if (ep->dwc_ep.is_in) {
		deptsiz.d32 = dwc_read_reg32(&in_ep_regs->dieptsiz);
		if (((core_if->dma_desc_enable == 0) && (deptsiz.b.xfersize == 0)) || ((core_if->dma_desc_enable != 0) && (desc_sts.b.bytes == 0))) {
			//req->actual = ep->dwc_ep.xfer_count;
			/* Is a Zero Len Packet needed? */
			do_setup_out_status_phase(pcd);
		}
	} else {
		//req->actual = ep->dwc_ep.xfer_count;
		/* Is a Zero Len Packet needed? */
		//if (req->sent_zlp) {

			//req->sent_zlp = 0;
		//}
		if (core_if->dma_desc_enable == 0)
		{
			do_setup_in_status_phase(pcd);
		}
	}

	/* Complete the request */
	if (is_last) {
		dwc_otg_request_done(ep, req, 0);
		ep->dwc_ep.start_xfer_buff = 0;
		ep->dwc_ep.xfer_buff = 0;
		ep->dwc_ep.xfer_len = 0;
		return 1;
	}

	return 0;
}

/**
 * This function completes the request for the EP. If there are
 * additional requests for the EP in the queue they will be started.
 */
static void complete_ep(dwc_otg_pcd_ep_t * ep) {

	dwc_otg_core_if_t *core_if = GET_CORE_IF(ep->pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	dwc_otg_dev_in_ep_regs_t *in_ep_regs = dev_if->in_ep_regs[ep->dwc_ep.num];
	deptsiz_data_t deptsiz;
	dev_dma_desc_sts_t desc_sts;
//	dwc_otg_pcd_request_t *req = 0;
	dwc_otg_dev_dma_desc_t *dma_desc;
	uint32_t byte_count = 0;
	deptsiz_data_t doeptsize0;
	int is_last = 0;
	int i;

	/* Get any pending requests */
/*	if (!DWC_CIRCLEQ_EMPTY(&ep->queue)) {			//delete by zz
		req = DWC_CIRCLEQ_FIRST(&ep->queue);
		if (!req) {
			return;
		}
	} else {
		return;
	}*/

//	log_out("complete 1\n");
	if (ep->dwc_ep.is_in) {
		deptsiz.d32 = dwc_read_reg32(&in_ep_regs->dieptsiz);

//		log_out("complete 2\n");
		if (core_if->dma_enable) {
			if (core_if->dma_desc_enable == 0) {
				depctl_data_t depctl;
				volatile uint32_t *addr2;

									//log_out("=====in4 \n");
//				addr1 = &dev_if->in_ep_regs[1]->diepctl;
				addr2 = &dev_if->out_ep_regs[1]->doepctl;
//
//				depctl.d32 = dwc_read_reg32(addr1);
//
//				depctl.b.epdis =1;
//				depctl.b.snak =0;
//				depctl.b.cnak = 1;
//				depctl.b.epena =1;
//				dwc_write_reg32(addr1, depctl.d32);
//				reg_val_out("depctl", depctl.d32);
//
				depctl.d32 = dwc_read_reg32(addr2);
				depctl.b.epdis =0;
				depctl.b.snak =0;
				depctl.b.cnak = 1;
				depctl.b.epena =1;
				dwc_write_reg32(addr2, depctl.d32);
//				reg_val_out("depctl", depctl.d32);

				doeptsize0.d32 = 0;


				doeptsize0.b.mc = 3;
				doeptsize0.b.pktcnt = 1;
				doeptsize0.b.xfersize = 8*3 ;

//				log_out("complete ep\n");
				dwc_write_reg32(&dev_if->out_ep_regs[1]->doeptsiz, doeptsize0.d32);
				dwc_write_reg32(&dev_if->out_ep_regs[1]->doepdma,
													(uint32_t)Bulk_out_bak);
//				dwc_write_reg32(&dev_if->out_ep_regs[2]->doeptsiz, doeptsize0.d32);
//				dwc_write_reg32(&dev_if->out_ep_regs[2]->doepdma,
//													(uint32_t)Bulk_out_bak);
//				dwc_write_reg32(&dev_if->out_ep_regs[3]->doeptsiz, doeptsize0.d32);
//				dwc_write_reg32(&dev_if->out_ep_regs[3]->doepdma,
//													(uint32_t)Bulk_out_bak);





				if (deptsiz.b.xfersize == 0 && deptsiz.b.pktcnt == 0) {
					byte_count = ep->dwc_ep.xfer_len - ep->dwc_ep.xfer_count;

					ep->dwc_ep.xfer_buff += byte_count;
					ep->dwc_ep.dma_addr += byte_count;
					ep->dwc_ep.xfer_count += byte_count;

					if (ep->dwc_ep.xfer_len < ep->dwc_ep.total_len) {
						dwc_otg_ep_start_transfer(core_if, &ep->dwc_ep);
					} else if (ep->dwc_ep.sent_zlp) {
						/*
						 * This fragment of code should initiate 0
						 * length trasfer in case if it is queued
						 * a trasfer with size divisible to EPs max
						 * packet size and with usb_request zero field
						 * is set, which means that after data is transfered,
						 * it is also should be transfered
						 * a 0 length packet at the end. For Slave and
						 * Buffer DMA modes in this case SW has
						 * to initiate 2 transfers one with transfer size,
						 * and the second with 0 size. For Desriptor
						 * DMA mode SW is able to initiate a transfer,
						 * which will handle all the packets including
						 * the last  0 legth.
						 */
						ep->dwc_ep.sent_zlp = 0;
						dwc_otg_ep_start_zl_transfer(core_if, &ep->dwc_ep);
					} else {
						is_last = 1;
					}
				} else {

				}
			} else {
				dma_desc = ep->dwc_ep.desc_addr;
				byte_count = 0;
				ep->dwc_ep.sent_zlp = 0;

					for (i = 0; i < ep->dwc_ep.desc_cnt; ++i) {
						desc_sts = dma_desc->status;
						byte_count += desc_sts.b.bytes;
						dma_desc++;
					}

				if (byte_count == 0) {
					ep->dwc_ep.xfer_count = ep->dwc_ep.total_len;
					is_last = 1;
				} else {

				}
			}
		} else {
			if (deptsiz.b.xfersize == 0 && deptsiz.b.pktcnt == 0) {

	//			log_out("complete 3\n");
				/*      Check if the whole transfer was completed,
				 *      if no, setup transfer for next portion of data
				 */
				if (ep->dwc_ep.xfer_len < ep->dwc_ep.total_len) {
	//				log_out("complete 4\n");
					dwc_otg_ep_start_transfer(core_if, &ep->dwc_ep);
				} else if (ep->dwc_ep.sent_zlp) {
					/*
					 * This fragment of code should initiate 0
					 * length trasfer in case if it is queued
					 * a trasfer with size divisible to EPs max
					 * packet size and with usb_request zero field
					 * is set, which means that after data is transfered,
					 * it is also should be transfered
					 * a 0 length packet at the end. For Slave and
					 * Buffer DMA modes in this case SW has
					 * to initiate 2 transfers one with transfer size,
					 * and the second with 0 size. For Desriptor
					 * DMA mode SW is able to initiate a transfer,
					 * which will handle all the packets including
					 * the last  0 legth.
					 */
	//				log_out("complete 5\n");
					ep->dwc_ep.sent_zlp = 0;
					dwc_otg_ep_start_zl_transfer(core_if, &ep->dwc_ep);
				} else {
					is_last = 1;
				}
			} else {

			}
		}
	} else {
		dwc_otg_dev_out_ep_regs_t *out_ep_regs =
				dev_if->out_ep_regs[ep->dwc_ep.num];
		desc_sts.d32 = 0;
	//	log_out("complete 6\n");
		if (core_if->dma_enable) {
			if (core_if->dma_desc_enable) {
				dma_desc = ep->dwc_ep.desc_addr;
				byte_count = 0;
				ep->dwc_ep.sent_zlp = 0;


					for (i = 0; i < ep->dwc_ep.desc_cnt; ++i) {
						desc_sts = dma_desc->status;
						byte_count += desc_sts.b.bytes;
						dma_desc++;
					}


				ep->dwc_ep.xfer_count = ep->dwc_ep.total_len - byte_count
						+ ((4 - (ep->dwc_ep.total_len & 0x3)) & 0x3);
				is_last = 1;
			} else {
				deptsiz.d32 = 0;
				deptsiz.d32 =
				dwc_read_reg32(&out_ep_regs->doeptsiz);

				byte_count = (ep->dwc_ep.xfer_len - ep->dwc_ep.xfer_count
						- deptsiz.b.xfersize);
				ep->dwc_ep.xfer_buff += byte_count;
				ep->dwc_ep.dma_addr += byte_count;
				ep->dwc_ep.xfer_count += byte_count;

				/*      Check if the whole transfer was completed,
				 *      if no, setup transfer for next portion of data
				 */
				if (ep->dwc_ep.xfer_len < ep->dwc_ep.total_len) {
					dwc_otg_ep_start_transfer(core_if, &ep->dwc_ep);
				} else if (ep->dwc_ep.sent_zlp) {
					/*
					 * This fragment of code should initiate 0
					 * length trasfer in case if it is queued
					 * a trasfer with size divisible to EPs max
					 * packet size and with usb_request zero field
					 * is set, which means that after data is transfered,
					 * it is also should be transfered
					 * a 0 length packet at the end. For Slave and
					 * Buffer DMA modes in this case SW has
					 * to initiate 2 transfers one with transfer size,
					 * and the second with 0 size. For Desriptor
					 * DMA mode SW is able to initiate a transfer,
					 * which will handle all the packets including
					 * the last  0 legth.
					 */
					ep->dwc_ep.sent_zlp = 0;
					dwc_otg_ep_start_zl_transfer(core_if, &ep->dwc_ep);
				} else {
					is_last = 1;
				}
			}
		} else {
			/*      Check if the whole transfer was completed,
			 *      if no, setup transfer for next portion of data
			 */
//			log_out("transfer completed\n");
			if (ep->dwc_ep.xfer_len < ep->dwc_ep.total_len) {
				log_out("transfer 1\n");
				dwc_otg_ep_start_transfer(core_if, &ep->dwc_ep);
			} else if (ep->dwc_ep.sent_zlp) {
//				log_out("transfer 2\n");
				/*
				 * This fragment of code should initiate 0
				 * length trasfer in case if it is queued
				 * a trasfer with size divisible to EPs max
				 * packet size and with usb_request zero field
				 * is set, which means that after data is transfered,
				 * it is also should be transfered
				 * a 0 length packet at the end. For Slave and
				 * Buffer DMA modes in this case SW has
				 * to initiate 2 transfers one with transfer size,
				 * and the second with 0 size. For Desriptor
				 * DMA mode SW is able to initiate a transfer,
				 * which will handle all the packets including
				 * the last  0 legth.
				 */
				ep->dwc_ep.sent_zlp = 0;
				dwc_otg_ep_start_zl_transfer(core_if, &ep->dwc_ep);
			} else {
//				log_out("transfer 3\n");
				is_last = 1;
			}
		}

	}

//	log_out("complete 7\n");
	/* Complete the request */
	if (is_last) {
	//	req->actual = ep->dwc_ep.xfer_count;

//		log_out("is last 1\n");
	//	dwc_otg_request_done(ep, req, 0);
//		log_out("is last 2\n");
		ep->dwc_ep.start_xfer_buff = 0;
		ep->dwc_ep.xfer_buff = 0;
		ep->dwc_ep.xfer_len = 0;

		/* If there is a request in the queue start it. */
	//	start_next_request(ep);
//		log_out("is last 3\n");
//		gadget_add_eps(&g_gadget_wrapper);

	}

}

/**
 * Handler for the IN EP timeout handshake interrupt.
 */
static __inline void handle_in_ep_timeout_intr(dwc_otg_pcd_t * pcd,	const uint32_t epnum) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	dctl_data_t dctl;
	dwc_otg_pcd_ep_t *ep;

	gintmsk_data_t intr_mask;
	dctl.d32 = 0;
	intr_mask.d32 = 0;
	ep = get_in_ep(pcd, epnum);

	/* Disable the NP Tx Fifo Empty Interrrupt */
	if (!core_if->dma_enable) {
		intr_mask.b.nptxfempty = 1;
		dwc_modify_reg32(&core_if->core_global_regs->gintmsk, intr_mask.d32, 0);
	}
	/** @todo NGS Check EP type.
	 * Implement for Periodic EPs */
	/*
	 * Non-periodic EP
	 */
	/* Enable the Global IN NAK Effective Interrupt */
	intr_mask.b.ginnakeff = 1;
	dwc_modify_reg32(&core_if->core_global_regs->gintmsk, 0, intr_mask.d32);

	/* Set Global IN NAK */
	dctl.b.sgnpinnak = 1;
	dwc_modify_reg32(&dev_if->dev_global_regs->dctl, dctl.d32, dctl.d32);

	ep->stopped = 1;

}

/**
 * This function handles EP0 Control transfers.
 *
 * The state of the control tranfers are tracked in
 * <code>ep0state</code>.
 */
static void handle_ep0(dwc_otg_pcd_t * pcd) {

	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_pcd_ep_t *ep0 = &pcd->ep0;
	//dev_dma_desc_sts_t desc_sts;
	deptsiz0_data_t deptsiz;
//	device_grxsts_data_t status;
	uint32_t byte_count= 0;
//	log_out("handle ep0 alsdjfklsdjfsklajfd\n");

//	pcd->ep0state = EP0_OUT_DATA_PHASE;
	switch (pcd->ep0state) {
	case EP0_DISCONNECT:
//		log_out("handle ep0 1\n");
		break;

	case EP0_IDLE:
		pcd->request_config = 0;

		if (core_if->dma_enable != 0)
		{
			pcd_setup_DMA(pcd);
		}
		break;

	case EP0_IN_DATA_PHASE:
		if (core_if->dma_enable != 0) {
			if (core_if->dma_desc_enable == 0) {

				deptsiz.d32 =
				dwc_read_reg32(&core_if->dev_if->in_ep_regs[0]->dieptsiz);
				byte_count = ep0->dwc_ep.xfer_len - deptsiz.b.xfersize;


			} else {
			}
			ep0->dwc_ep.xfer_count += byte_count;
			ep0->dwc_ep.xfer_buff += byte_count;
			ep0->dwc_ep.dma_addr += byte_count;

		}

		if (ep0->dwc_ep.xfer_count < ep0->dwc_ep.total_len) {

			dwc_otg_ep0_continue_transfer(GET_CORE_IF(pcd), &ep0->dwc_ep);

		} else if (ep0->dwc_ep.sent_zlp) {
			dwc_otg_ep0_continue_transfer(GET_CORE_IF(pcd), &ep0->dwc_ep);
			ep0->dwc_ep.sent_zlp = 0;

		} else {
			ep0_complete_request(ep0);
		}

		ep0_out_start(core_if, pcd);
		break;

	case EP0_OUT_DATA_PHASE:
		if (core_if->dma_enable != 0) {
			if (core_if->dma_desc_enable == 0) {
				deptsiz.d32 =
				dwc_read_reg32(&core_if->dev_if->out_ep_regs[0]->doeptsiz);
				byte_count = ep0->dwc_ep.maxpacket - deptsiz.b.xfersize;
			}

			ep0->dwc_ep.xfer_count += byte_count;
			ep0->dwc_ep.xfer_buff += byte_count;
			ep0->dwc_ep.dma_addr += byte_count;
		}
		if (ep0->dwc_ep.xfer_count < ep0->dwc_ep.total_len) {
			dwc_otg_ep0_continue_transfer(GET_CORE_IF(pcd), &ep0->dwc_ep);

		} else if (ep0->dwc_ep.sent_zlp) {
			dwc_otg_ep0_continue_transfer(GET_CORE_IF(pcd), &ep0->dwc_ep);
			ep0->dwc_ep.sent_zlp = 0;
		} else {
			ep0_complete_request(ep0);
		}
		break;

	case EP0_IN_STATUS_PHASE:
	case EP0_OUT_STATUS_PHASE:
		ep0_complete_request(ep0);
		pcd->ep0state = EP0_IDLE;
		ep0->stopped = 1;
		ep0->dwc_ep.is_in = 0; /* OUT for next SETUP */

		/* Prepare for more SETUP Packets */
		if (core_if->dma_enable) {
			ep0_out_start(core_if, pcd);
		}
		break;

	case EP0_STALL:
		break;
	}
}

/**
 * Handler for the IN EP NAK interrupt.
 */
static __inline int32_t handle_in_ep_nak_intr(dwc_otg_pcd_t * pcd,
		const uint32_t epnum) {
	/** @todo implement ISR */
	dwc_otg_core_if_t *core_if;
	diepmsk_data_t intr_mask;
	intr_mask.d32 = 0;

	core_if = GET_CORE_IF(pcd);
	intr_mask.b.nak = 1;

	if (core_if->multiproc_int_enable) {
		dwc_modify_reg32(
				&core_if->dev_if->dev_global_regs->diepeachintmsk[epnum],
				intr_mask.d32, 0);
	} else {
		dwc_modify_reg32(&core_if->dev_if->dev_global_regs->diepmsk,
				intr_mask.d32, 0);
	}

	return 1;
}

/**
 * This interrupt indicates that an IN EP has a pending Interrupt.
 * The sequence for handling the IN EP interrupt is shown below:
 * -#	Read the Device All Endpoint Interrupt register
 * -#	Repeat the following for each IN EP interrupt bit set (from
 *		LSB to MSB).
 * -#	Read the Device Endpoint Interrupt (DIEPINTn) register
 * -#	If "Transfer Complete" call the request complete function
 * -#	If "Endpoint Disabled" complete the EP disable procedure.
 * -#	If "AHB Error Interrupt" log error
 * -#	If "Time-out Handshake" log error
 * -#	If "IN Token Received when TxFIFO Empty" write packet to Tx
 *		FIFO.
 * -#	If "IN Token EP Mismatch" (disable, this is handled by EP
 *		Mismatch Interrupt)
 */
static int32_t dwc_otg_pcd_handle_in_ep_intr(dwc_otg_pcd_t * pcd) {

	#define CLEAR_IN_EP_INTR(__core_if,__epnum,__intr) \
do { \
		diepint_data_t diepint; \
		diepint.d32=0; \
		diepint.b.__intr = 1; \
		dwc_write_reg32(&__core_if->dev_if->in_ep_regs[__epnum]->diepint, \
		diepint.d32); \
} while (0)

	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	diepint_data_t diepint;
	dctl_data_t dctl;
	depctl_data_t depctl;
	uint32_t ep_intr;
	uint32_t epnum = 0;
	dwc_otg_pcd_ep_t *ep;
	dwc_ep_t *dwc_ep;
	gintmsk_data_t intr_mask;

	diepint.d32 = 0;
	dctl.d32 = 0;
	depctl.d32 = 0;
	intr_mask.d32 = 0;
	/* Read in the device interrupt bits */
	ep_intr = dwc_otg_read_dev_all_in_ep_intr(core_if);

	/* Service the Device IN interrupts for each endpoint */
	while (ep_intr) {
	//	log_out("epin\n");
		if (ep_intr & 0x1) {

			uint32_t empty_msk;
			/* Get EP pointer */
			ep = get_in_ep(pcd, epnum);
			dwc_ep = &ep->dwc_ep;

			depctl.d32 = dwc_read_reg32(&dev_if->in_ep_regs[epnum]->diepctl);
			empty_msk = dwc_read_reg32(&dev_if->dev_global_regs->dtknqr4_fifoemptymsk);

			diepint.d32 = dwc_otg_read_dev_in_ep_intr(core_if, dwc_ep);

			/* Transfer complete */

			if (diepint.b.xfercompl || Enumpacket64 ==1||NUM1_sendf == 1) {
				/* Disable the NP Tx FIFO Empty
				 * Interrrupt */
			//	NUM1_sendf =0;

			/*	if(core_if->dma_enable)
				{
					volatile uint32_t *addr1, *addr2;

					log_out("=====in4 \n");
					addr1 = &dev_if->in_ep_regs[2]->diepctl;

					depctl.d32 = dwc_read_reg32(addr1);


					depctl.b.snak =0;
					depctl.b.cnak = 0;
					depctl.b.epena =0;
					dwc_write_reg32(addr1, depctl.d32);
					//			reg_val_out("depctl", depctl.d32);

				}*/


				if (core_if->en_multiple_tx_fifo == 0) {
					intr_mask.b.nptxfempty = 1;
					dwc_modify_reg32(&core_if->core_global_regs->gintmsk,
							intr_mask.d32, 0);
				} else {
					/* Disable the Tx FIFO Empty Interrupt for this EP */
					uint32_t fifoemptymsk = 0x1 << dwc_ep->num;
					dwc_modify_reg32(
							&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
							fifoemptymsk, 0);
				}
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(core_if, epnum, xfercompl);

				/* Complete the transfer */
				if (epnum == 0) {
					handle_ep0(pcd);
				}
				else {
					complete_ep(ep);
				}
				(void) empty_msk;
			}

			/* Endpoint disable      */
			if (diepint.b.epdisabled) {
				handle_in_ep_disable_intr(pcd, epnum);

				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(core_if, epnum, epdisabled);
			}

			/* AHB Error */
			if (diepint.b.ahberr) {
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(core_if, epnum, ahberr);
			}
			/* TimeOUT Handshake (non-ISOC IN EPs) */
			if (diepint.b.timeout) {
				handle_in_ep_timeout_intr(pcd, epnum);

				CLEAR_IN_EP_INTR(core_if, epnum, timeout);
			}

			/** IN Token received with TxF Empty */
			if (diepint.b.intktxfemp) {
				if (!ep->stopped && epnum != 0) {

					diepmsk_data_t diepmsk;
					diepmsk.d32 = 0;
					diepmsk.b.intktxfemp = 1;

					if (core_if->multiproc_int_enable) {
						dwc_modify_reg32(
								&dev_if->dev_global_regs->diepeachintmsk[epnum],
								diepmsk.d32, 0);
					} else {
						dwc_modify_reg32(&dev_if->dev_global_regs->diepmsk,
								diepmsk.d32, 0);
					}
				} else if (core_if->dma_desc_enable && epnum == 0
						&& pcd->ep0state == EP0_OUT_STATUS_PHASE) {
					// EP0 IN set STALL
					depctl.d32 =
					dwc_read_reg32(&dev_if->in_ep_regs[epnum]->diepctl);

					/* set the disable and stall bits */
					if (depctl.b.epena) {
						depctl.b.epdis = 1;
					}
					depctl.b.stall = 1;
					dwc_write_reg32(&dev_if->in_ep_regs[epnum]->diepctl,
							depctl.d32);
				}
				CLEAR_IN_EP_INTR(core_if, epnum, intktxfemp);
			}

			/** IN Token Received with EP mismatch */
			if (diepint.b.intknepmis) {
				CLEAR_IN_EP_INTR(core_if, epnum, intknepmis);
			}
			/** IN Endpoint NAK Effective */
			if (diepint.b.inepnakeff) {
				/* Periodic EP */
				if (ep->disabling) {
//					log_out("disable end 2");
					depctl.d32 = 0;
					depctl.b.snak = 1;
					depctl.b.epdis = 1;
					dwc_modify_reg32(&dev_if->in_ep_regs[epnum]->diepctl,
							depctl.d32, depctl.d32);
				}
				CLEAR_IN_EP_INTR(core_if, epnum, inepnakeff);

			}


			/** IN EP Tx FIFO Empty Intr */
			if (diepint.b.emptyintr) {
//				log_out("empty pos 1\n");
//				if(Enumpacket64 == 0)
//				{
					write_empty_tx_fifo(pcd, epnum);
//				}
//				else
//				{
//					Enumpacket64 = 0;
//				}
//				log_out("empty pos 2\n");
				CLEAR_IN_EP_INTR(core_if, epnum, emptyintr);

			}

			/** IN EP BNA Intr */
			if (diepint.b.bna) {
				CLEAR_IN_EP_INTR(core_if, epnum, bna);
				if (core_if->dma_desc_enable) {
					{
						dctl.d32 = dwc_read_reg32(
								&dev_if->dev_global_regs->dctl);

						/* If Global Continue on BNA is disabled - disable EP */
						if (!dctl.b.gcontbna) {
							depctl.d32 = 0;
							depctl.b.snak = 1;
							depctl.b.epdis = 1;
							dwc_modify_reg32(
									&dev_if->in_ep_regs[epnum]->diepctl,
									depctl.d32, depctl.d32);
						} else {
							start_next_request(ep);
						}
					}
				}
			}

			/* NAK Interrutp */
			if (diepint.b.nak) {
				handle_in_ep_nak_intr(pcd, epnum);

				CLEAR_IN_EP_INTR(core_if, epnum, nak);
			}

		}//if

		epnum++;
		ep_intr >>= 1;

	}//while

	return 1;
#undef CLEAR_IN_EP_INTR
}

/**
 * Handler for the OUT EP Babble interrupt.
 */
static __inline int32_t handle_out_ep_babble_intr(dwc_otg_pcd_t * pcd,
		const uint32_t epnum) {
	/** @todo implement ISR */
	dwc_otg_core_if_t *core_if;
	doepmsk_data_t intr_mask;
	intr_mask.d32 = 0;

	core_if = GET_CORE_IF(pcd);
	intr_mask.b.babble = 1;

	if (core_if->multiproc_int_enable) {
		dwc_modify_reg32(
				&core_if->dev_if->dev_global_regs->doepeachintmsk[epnum],
				intr_mask.d32, 0);
	} else {
		dwc_modify_reg32(&core_if->dev_if->dev_global_regs->doepmsk,
				intr_mask.d32, 0);
	}

	return 1;
}

/**
 * This interrupt indicates that an OUT EP has a pending Interrupt.
 * The sequence for handling the OUT EP interrupt is shown below:
 * -#	Read the Device All Endpoint Interrupt register
 * -#	Repeat the following for each OUT EP interrupt bit set (from
 *		LSB to MSB).
 * -#	Read the Device Endpoint Interrupt (DOEPINTn) register
 * -#	If "Transfer Complete" call the request complete function
 * -#	If "Endpoint Disabled" complete the EP disable procedure.
 * -#	If "AHB Error Interrupt" log error
 * -#	If "Setup Phase Done" process Setup Packet (See Standard USB
 *		Command Processing)
 */
static int32_t dwc_otg_pcd_handle_out_ep_intr(dwc_otg_pcd_t * pcd) {

#define CLEAR_OUT_EP_INTR(__core_if,__epnum,__intr) \
do { \
		doepint_data_t doepint; \
		doepint.d32=0; \
		doepint.b.__intr = 1; \
		dwc_write_reg32(&__core_if->dev_if->out_ep_regs[__epnum]->doepint, \
		doepint.d32); \
} while (0)

	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	uint32_t ep_intr;
	doepint_data_t doepint;
	dctl_data_t dctl;
	depctl_data_t doepctl;
	uint32_t epnum = 0;
	dwc_otg_pcd_ep_t *ep;
	dwc_ep_t *dwc_ep;
	doepint.d32 = 0;
	dctl.d32 = 0;
	doepctl.d32 = 0;

	/* Read in the device interrupt bits */
	ep_intr = dwc_otg_read_dev_all_out_ep_intr(core_if);

	while (ep_intr) {

		if (ep_intr & 0x1) {

			/* Get EP pointer */
			ep = get_out_ep(pcd, epnum);
			dwc_ep = &ep->dwc_ep;

			doepint.d32 = dwc_otg_read_dev_out_ep_intr(core_if, dwc_ep);

			if ((core_if->dma_enable != 0) &&(dwc_ep->num == 0x01)) {
				recvflag = 1;
			}

			/* Transfer complete */
			if (doepint.b.xfercompl) {

				if (epnum == 0) {
					/* Clear the bit in DOEPINTn for this interrupt */
					CLEAR_OUT_EP_INTR(core_if, epnum, xfercompl);
					if (core_if->dma_desc_enable == 0
							|| pcd->ep0state != EP0_IDLE)
//						log_out("out pos 1\n");
						handle_ep0(pcd);
				} else {
					/* Clear the bit in DOEPINTn for this interrupt */
					CLEAR_OUT_EP_INTR(core_if, epnum, xfercompl);
					complete_ep(ep);
				}

			}

			/* Endpoint disable      */
			if (doepint.b.epdisabled) {

				/* Clear the bit in DOEPINTn for this interrupt */
				CLEAR_OUT_EP_INTR(core_if, epnum, epdisabled);
			}
			/* AHB Error */
			if (doepint.b.ahberr) {

				CLEAR_OUT_EP_INTR(core_if, epnum, ahberr);
			}
			/* Setup Phase Done (contorl EPs) */
			if (doepint.b.setup) {
				CLEAR_OUT_EP_INTR(core_if, epnum, setup);

//				log_out("out pos 2\n");
				handle_ep0(pcd);
			}

			/** OUT EP BNA Intr */
			if (doepint.b.bna) {
				CLEAR_OUT_EP_INTR(core_if, epnum, bna);
				if (core_if->dma_desc_enable) {
					{
						dctl.d32 =
						dwc_read_reg32(&dev_if->dev_global_regs->dctl);

						/* If Global Continue on BNA is disabled - disable EP */
						if (!dctl.b.gcontbna) {
//							log_out("disable end 4");
							doepctl.d32 = 0;
							doepctl.b.snak = 1;
							doepctl.b.epdis = 1;
							dwc_modify_reg32(
									&dev_if->out_ep_regs[epnum]->doepctl,
									doepctl.d32, doepctl.d32);
						} else {
							start_next_request(ep);
						}
					}
				}
			}

			if (doepint.b.stsphsercvd) {
				CLEAR_OUT_EP_INTR(core_if, epnum, stsphsercvd);
				if (core_if->dma_desc_enable) {
//					log_out("set up send 7");
					do_setup_in_status_phase(pcd);
				}
			}

			/* Babble Interrutp */
			if (doepint.b.babble) {

				handle_out_ep_babble_intr(pcd, epnum);

				CLEAR_OUT_EP_INTR(core_if, epnum, babble);
			}
			/* NAK Interrutp */
//			if (doepint.b.nak) {

//				handle_out_ep_nak_intr(pcd, epnum);

//				CLEAR_OUT_EP_INTR(core_if, epnum, nak);
//			}
			/* NYET Interrutp */
//			if (doepint.b.nyet) {

//				handle_out_ep_nyet_intr(pcd, epnum);

//				CLEAR_OUT_EP_INTR(core_if, epnum, nyet);
//			}

		}

		epnum++;
		ep_intr >>= 1;
	}

	return 1;

#undef CLEAR_OUT_EP_INTR
}


uint8_t delayflag1 = 0;
/**
 * PCD interrupt handler.
 *
 * The PCD handles the device interrupts.  Many conditions can cause a
 * device interrupt. When an interrupt occurs, the device interrupt
 * service routine determines the cause of the interrupt and
 * dispatches handling to the appropriate function. These interrupt
 * handling functions are described below.
 *
 * All interrupt registers are processed from LSB to MSB.
 *
 */
int32_t dwc_otg_pcd_handle_intr(dwc_otg_pcd_t * pcd, struct wokoo_udc *_udc) {
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);
	gintsts_data_t gintr_status;
	int32_t retval = 0;
	uint32_t gintmsk;

	//dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;

	gintmsk = dwc_read_reg32(&core_if->core_global_regs->gintmsk);

	/* Exit from ISR if core is hibernated*/
	if (core_if->hibernation_suspend == 1) {
		return retval;
	}

	if (dwc_otg_mode(core_if) != DWC_HOST_MODE ) {
		gintr_status.d32 = dwc_otg_read_core_intr(core_if);

		if (gintr_status.b.rxstsqlvl) {
            printk("dwc_otg_pcd_handle_rx_status_q_level_intr\n");
			retval |= dwc_otg_pcd_handle_rx_status_q_level_intr(pcd, _udc);
		}

		if (gintr_status.b.nptxfempty) {
            printk("dwc_otg_pcd_handle_np_tx_fifo_empty_intr\n");
			retval |= dwc_otg_pcd_handle_np_tx_fifo_empty_intr(pcd);
		}

		if (gintr_status.b.erlysuspend) {
			retval |= dwc_otg_pcd_handle_early_suspend_intr(pcd);
			printk("dwc_otg_pcd_handle_early_suspend_intr\n");
		}

		if (gintr_status.b.usbreset) {
			retval |= dwc_otg_pcd_handle_usb_reset_intr(pcd);
			printk("dwc_otg_pcd_handle_usb_reset_intr\n");
		}

		if (gintr_status.b.enumdone) {
			retval |= dwc_otg_pcd_handle_enum_done_intr(pcd);
			printk("dwc_otg_pcd_handle_enum_done_intr\n");
		}

		if (gintr_status.b.eopframe) {
			retval |= dwc_otg_pcd_handle_end_periodic_frame_intr(pcd);
            printk("dwc_otg_pcd_handle_end_periodic_frame_intr\n");
		}

		if (gintr_status.b.inepint) {
			printk("in ep intr come\n");
			if (!core_if->multiproc_int_enable) {
				retval |= dwc_otg_pcd_handle_in_ep_intr(pcd);
			}
		}

		if (gintr_status.b.outepintr) {
            printk("out ep intr come\n");
			if (!core_if->multiproc_int_enable) {
				retval |= dwc_otg_pcd_handle_out_ep_intr(pcd);
			}
		}

		/* In MPI mode De vice Endpoints intterrupts are asserted
		 * without setting outepintr and inepint bits set, so these
		 * Interrupt handlers are called without checking these bit-fields
		 */
		if (core_if->multiproc_int_enable) {
			retval |= dwc_otg_pcd_handle_in_ep_intr(pcd);
			retval |= dwc_otg_pcd_handle_out_ep_intr(pcd);
            printk("in and out ep intr come\n");
		}

		gintr_status.d32 = dwc_otg_read_core_intr(core_if);
	//	DWC_SPINUNLOCK(pcd->lock);
	}

	return retval;
}

