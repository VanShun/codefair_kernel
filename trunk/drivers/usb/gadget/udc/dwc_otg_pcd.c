
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>


#include "dwc_otg_cil.h"
#include "dwc_otg_pcd.h"
#include "wk_def.h"
#include "dwc_otg_pcd_if.h"
#include "wokoo_usb.h"
#include "dwc_os.h"

#define log_out printk

dwc_otg_pcd_t g_pcd;
uint32_t setup_pkt_bak[20];// __attribute__ ((at(0x40000)));
uint32_t Bulk_out[80]; // __attribute__ ((at(0x40050)));
uint32_t Bulk_out_bak[80]; // __attribute__ ((at(0x40190)));
uint16_t status_buf;

uint32_t setup_pkt[10];


/**
 * This function terminates all the requsts in the EP request queue.
 */
static void dwc_otg_pcd_init_ep(dwc_otg_pcd_t * pcd, dwc_otg_pcd_ep_t * pcd_ep, uint32_t is_in, uint32_t ep_num);
void dwc_otg_request_nuke(dwc_otg_pcd_ep_t * ep)
{
//	dwc_otg_pcd_request_t *req;

	ep->stopped = 1;

	/* called with irqs blocked?? */
//	while (!DWC_CIRCLEQ_EMPTY(&ep->queue)) {
//		req = DWC_CIRCLEQ_FIRST(&ep->queue);
//		dwc_otg_request_done(ep, req, -DWC_E_SHUTDOWN);
//	}
}

void dwc_otg_pcd_start(dwc_otg_pcd_t * pcd, const struct dwc_otg_pcd_function_ops *fops)
{
	pcd->fops = fops;
}
/**
 * PCD Callback function for initializing the PCD when switching to
 * device mode.
 *
 * @param p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_start_cb(void *p)
{

	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *) p;

	/*
	 * Initialized the Core for Device mode.
	 */
	if (dwc_otg_mode(GET_CORE_IF(pcd)) != DWC_HOST_MODE ) {
		dwc_otg_core_dev_init(GET_CORE_IF(pcd));
	}

	return 1;
}

/**
 * PCD Callback function for notifying the PCD when resuming from
 * suspend.
 *
 * @param p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_resume_cb(void *p)
{

	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *) p;

	if (pcd->fops->resume) {
		pcd->fops->resume(pcd);
	}

	/* Stop the SRP timeout timer. */
	if ((GET_CORE_IF(pcd)->core_params->phy_type != DWC_PHY_TYPE_PARAM_FS)
	    || (!GET_CORE_IF(pcd)->core_params->i2c_enable)) {
		if (GET_CORE_IF(pcd)->srp_timer_started) {
			GET_CORE_IF(pcd)->srp_timer_started = 0;
//			DWC_TIMER_CANCEL(GET_CORE_IF(pcd)->srp_timer);
		}
	}

	return 1;
}
/**
 * PCD Callback function for notifying the PCD device is suspended.
 *
 * @param p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_suspend_cb(void *p)
{

	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *) p;

	if (pcd->fops->suspend) {
		pcd->fops->suspend(pcd);
	}

	return 1;
}

/**
 * PCD Callback function for stopping the PCD when switching to Host
 * mode.
 *
 * @param p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_stop_cb(void *p)
{

	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *) p;
	extern void dwc_otg_pcd_stop(dwc_otg_pcd_t * _pcd);

	dwc_otg_pcd_stop(pcd);
	return 1;

}
/**
 * This function completes a request.  It call's the request call back.
 */
void dwc_otg_request_done(dwc_otg_pcd_ep_t * ep, dwc_otg_pcd_request_t * req,
			  int32_t status)
{
	unsigned stopped = ep->stopped;


	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
//	DWC_SPINUNLOCK(ep->pcd->lock);

	ep->stopped = stopped;
	//dwc_free(req);
}

//#####------------
/**
 * PCD Callback structure for handling mode switching.
 */
static dwc_otg_cil_callbacks_t pcd_callbacks;

static void wk_pcd_callbacks_init(void)
{
	pcd_callbacks.start = dwc_otg_pcd_start_cb;
	pcd_callbacks.stop = dwc_otg_pcd_stop_cb;
	pcd_callbacks.suspend = dwc_otg_pcd_suspend_cb;
	pcd_callbacks.resume_wakeup = dwc_otg_pcd_resume_cb;
	pcd_callbacks.p = 0;			/* Set at registration */
}

static void dwc_otg_pcd_init_ep(dwc_otg_pcd_t * pcd, dwc_otg_pcd_ep_t * pcd_ep, uint32_t is_in, uint32_t ep_num)
{
	/* Init EP structure */
	pcd_ep->desc = 0;
	pcd_ep->pcd = pcd;
	pcd_ep->stopped = 1;
	pcd_ep->queue_sof = 0;

	/* Init DWC ep structure */
	pcd_ep->dwc_ep.is_in = is_in;
	pcd_ep->dwc_ep.num = ep_num;
	pcd_ep->dwc_ep.active = 0;
	pcd_ep->dwc_ep.tx_fifo_num = 0;
	/* Control until ep is actvated */
	pcd_ep->dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
	pcd_ep->dwc_ep.maxpacket = MAX_PACKET_SIZE;
	pcd_ep->dwc_ep.dma_addr = 0;
	pcd_ep->dwc_ep.start_xfer_buff = 0;
	pcd_ep->dwc_ep.xfer_buff = 0;
	pcd_ep->dwc_ep.xfer_len = 0;
	pcd_ep->dwc_ep.xfer_count = 0;
	pcd_ep->dwc_ep.sent_zlp = 0;
	pcd_ep->dwc_ep.total_len = 0;
	pcd_ep->dwc_ep.desc_addr = 0;
//	pcd_ep->dwc_ep.dma_desc_addr = 0;
//	DWC_CIRCLEQ_INIT(&pcd_ep->queue);
}

/**
 * Initialize ep's
 */
static void dwc_otg_pcd_reinit(dwc_otg_pcd_t * pcd)
{
	int i;
	uint32_t hwcfg1;
	dwc_otg_pcd_ep_t *ep;
	int in_ep_cntr, out_ep_cntr;
	uint32_t num_in_eps = (GET_CORE_IF(pcd))->dev_if->num_in_eps;
	uint32_t num_out_eps = (GET_CORE_IF(pcd))->dev_if->num_out_eps;

	/**
	 * Initialize the EP0 structure.
	 */
	ep = &pcd->ep0;
	dwc_otg_pcd_init_ep(pcd, ep, 0, 0);

	in_ep_cntr = 0;
	hwcfg1 = (GET_CORE_IF(pcd))->hwcfg1.d32 >> 3;
	for (i = 1; in_ep_cntr < num_in_eps; i++) {
		if ((hwcfg1 & 0x1) == 0) {
			dwc_otg_pcd_ep_t *ep = &pcd->in_ep[in_ep_cntr];
			in_ep_cntr++;
			/**
			 * @todo NGS: Add direction to EP, based on contents
			 * of HWCFG1.  Need a copy of HWCFG1 in pcd structure?
			 * sprintf(";r
			 */
			dwc_otg_pcd_init_ep(pcd, ep, 1 /* IN */ , i);

//			DWC_CIRCLEQ_INIT(&ep->queue);
		}
		hwcfg1 >>= 2;
	}

	out_ep_cntr = 0;
	hwcfg1 = (GET_CORE_IF(pcd))->hwcfg1.d32 >> 2;
	for (i = 1; out_ep_cntr < num_out_eps; i++) {
		if ((hwcfg1 & 0x1) == 0) {
			dwc_otg_pcd_ep_t *ep = &pcd->out_ep[out_ep_cntr];
			out_ep_cntr++;
			/**
			 * @todo NGS: Add direction to EP, based on contents
			 * of HWCFG1.  Need a copy of HWCFG1 in pcd structure?
			 * sprintf(";r
			 */
			dwc_otg_pcd_init_ep(pcd, ep, 0 /* OUT */ , i);
//			DWC_CIRCLEQ_INIT(&ep->queue);
		}
		hwcfg1 >>= 2;
	}

	pcd->ep0state = EP0_DISCONNECT;
	pcd->ep0.dwc_ep.maxpacket = MAX_EP0_SIZE;
	pcd->ep0.dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
}

/**
 * This function initialized the PCD portion of the driver.
 *
 */
dwc_otg_pcd_t *dwc_otg_pcd_init(dwc_otg_core_if_t * core_if)
{
    dwc_otg_pcd_t *pcd ;
	dctl_data_t dctl;

	/*
	 * Allocate PCD structure
	 */
	pcd = &g_pcd;
    wk_pcd_callbacks_init();

	pcd->core_if = core_if;
	/*
	 * Initialized the Core for Device mode.
	 */

	if ( dwc_otg_mode(core_if) != DWC_HOST_MODE ) {
		dwc_otg_core_dev_init(core_if);
	}else
		printk("not dev mode\n");

	/*
	 * Register the PCD Callbacks.
	 */
	dwc_otg_cil_register_pcd_callbacks(core_if, &pcd_callbacks, pcd);
	/*
	 * Initialize the DMA buffer for SETUP packets
	 */

	if(GET_CORE_IF(pcd)->dma_enable)
	{
		pcd->setup_pkt =(void *)setup_pkt_bak;
		pcd->status_buf = &status_buf;
	}
	else
	{
		pcd->setup_pkt = (void *)setup_pkt;
		pcd->status_buf = &status_buf;
	}
	//&pcd->setup_pkt_dma_handle;

	dwc_otg_pcd_reinit(pcd);

	/* Initialize tasklets */
	//pcd->start_xfer_tasklet = DWC_TASK_ALLOC(start_xfer_tasklet_func, pcd);
	//pcd->test_mode_tasklet = DWC_TASK_ALLOC(do_test_mode, pcd);

	/* Initialize SRP timer */
//	core_if->srp_timer = DWC_TIMER_ALLOC("SRP TIMER", srp_timeout, core_if);

	dctl.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dctl);
	dctl.d32 = dctl.d32 & 0xFFFFFFFD;
	dwc_write_reg32(&core_if->dev_if->dev_global_regs->dctl, dctl.d32);

	return pcd;
}

static uint32_t assign_perio_tx_fifo(dwc_otg_core_if_t * core_if)
{
	uint32_t PerTxMsk = 1;
	int i;
	for (i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; ++i) {
		if ((PerTxMsk & core_if->p_tx_msk) == 0) {
			core_if->p_tx_msk |= PerTxMsk;
			return i + 1;
		}
		PerTxMsk <<= 1;
	}
	return 0;
}

static uint32_t assign_tx_fifo(dwc_otg_core_if_t * core_if)
{
	uint32_t TxMsk = 1;
	int i;
	core_if->tx_msk = 0x00;

	for (i = 0; i < core_if->hwcfg4.b.num_in_eps; ++i) {
		if ((TxMsk & core_if->tx_msk) == 0) {
			core_if->tx_msk |= TxMsk;
			return i + 1;
		}
		TxMsk <<= 1;
	}
	return 0;
}

int dwc_otg_pcd_ep_enable(dwc_otg_pcd_t * pcd, usb_endpoint_descriptor_t * ep_desc, void *usb_ep)
{
	int num, dir;
	dwc_otg_pcd_ep_t *ep = 0;
	usb_endpoint_descriptor_t *desc;
//	uint32_t flags;
//	uint32_t readaddr;
	fifosize_data_t dptxfsiz;
	gdfifocfg_data_t gdfifocfg;
	gdfifocfg_data_t gdfifocfgbase;
	dwc_otg_dev_dma_desc_t  Outaddr;
	int retval = 0;
	int i, epcount;
	dptxfsiz.d32 = 0;
	gdfifocfg.d32 = 0;
	gdfifocfgbase.d32 = 0;

	desc = (usb_endpoint_descriptor_t *)ep_desc;

	if (ep_desc==NULL) {
		pcd->ep0.priv = usb_ep;
		ep = &pcd->ep0;
		retval = -DWC_E_INVALID;
		goto out;
	}

	num = desc->bEndpointAddress & 0x0F;
	dir = desc->bEndpointAddress & 0x80;

	if (!desc->wMaxPacketSize) {

		retval = -DWC_E_INVALID;
		goto out;
	}

	if (dir == UE_DIR_IN) {
		epcount = pcd->core_if->dev_if->num_in_eps;
		for (i=0; i < epcount; i++) {
			if (num == pcd->in_ep[i].dwc_ep.num) {
				ep = &pcd->in_ep[i];
				break;
			}
		}
	} else {
		epcount = pcd->core_if->dev_if->num_out_eps;
		for (i=0; i < epcount; i++) {
			if (num == pcd->out_ep[i].dwc_ep.num) {
				ep = &pcd->out_ep[i];
				break;
			}
		}
	}

	if (!ep) {
		retval = -DWC_E_INVALID;
		goto out;
	}

	ep->desc = desc;
	ep->priv = usb_ep;

	/*
	 * Activate the EP
	 */
	ep->stopped = 0;

	ep->dwc_ep.is_in = (dir == UE_DIR_IN);
	ep->dwc_ep.maxpacket = desc->wMaxPacketSize;
	ep->dwc_ep.type = desc->bmAttributes & UE_XFERTYPE;

	if (ep->dwc_ep.is_in) {
		if (!GET_CORE_IF(pcd)->en_multiple_tx_fifo) {

//			log_out("=====in1 \n");
			if (ep->dwc_ep.type == UE_ISOCHRONOUS) {
				/*
				 * if ISOC EP then assign a Periodic Tx FIFO.
				 */
				ep->dwc_ep.tx_fifo_num =
				    assign_perio_tx_fifo(GET_CORE_IF(pcd));
			}
		} else {
			/*
			 * if Dedicated FIFOs mode is on then assign a Tx FIFO.
			 */
			//log_out("=====in2 \n");
			ep->dwc_ep.tx_fifo_num =
			    assign_tx_fifo(GET_CORE_IF(pcd));
		}

//		log_out("=====in3 \n");
		/* Calculating EP info controller base address */
		if (ep->dwc_ep.tx_fifo_num) {
			//log_out("=====in3 \n");
			gdfifocfg.d32 = dwc_read_reg32(&GET_CORE_IF(pcd)->core_global_regs->gdfifocfg);
			gdfifocfgbase.d32 = gdfifocfg.d32 >> 16;
			dptxfsiz.d32 = (dwc_read_reg32(&GET_CORE_IF(pcd)->core_global_regs->
						dtxfsiz[ep->dwc_ep.tx_fifo_num]) >> 16);
			gdfifocfg.b.epinfobase = gdfifocfgbase.d32 + dptxfsiz.d32;
			dwc_write_reg32(&GET_CORE_IF(pcd)->core_global_regs->gdfifocfg, gdfifocfg.d32);
		}
	}

	/* Set initial data PID. */
	if (ep->dwc_ep.type == UE_BULK) {
		ep->dwc_ep.data_pid_start = 0;
	}

	if(GET_CORE_IF(pcd)->dma_enable)
			{
			//	if (ep->dwc_ep.is_in ==0 )
				{
					deptsiz_data_t doeptsize0;
//					uint32_t readaddr;
					dwc_otg_dev_if_t *dev_if =  pcd->core_if->dev_if;
					doeptsize0.d32 = 0;


					doeptsize0.b.mc = 3;
					doeptsize0.b.pktcnt = 1;
					doeptsize0.b.xfersize = 8*3 ;

					//dwc_write_reg32(&dev_if->out_ep_regs[0]->doeptsiz, doeptsize0.d32);
					dwc_write_reg32(&dev_if->out_ep_regs[num]->doeptsiz, doeptsize0.d32);
					dwc_write_reg32(&dev_if->out_ep_regs[num]->doepdma,
									(uint32_t)Bulk_out);
				}
			}


	/* Alloc DMA Descriptors */
	if(GET_CORE_IF(pcd)->dma_desc_enable)
	{
		log_out("should not be here");
		if(ep->dwc_ep.type != UE_ISOCHRONOUS)
		{
			Outaddr.buf = (uint32_t)Bulk_out;
			ep->dwc_ep.desc_addr = &Outaddr;
			if(!ep->dwc_ep.desc_addr)
			{
				log_out("alloc addr failed!");
				goto out;
			}


		}

	}

//	log_out("=====ep finish \n");

	  dwc_otg_ep_activate(GET_CORE_IF(pcd), &ep->dwc_ep);
      out:
	return retval;
}

