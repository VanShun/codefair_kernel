#ifndef DWC_OTG_PCD_H_
#define DWC_OTG_PCD_H_

#include "usb.h"
#include "dwc_otg_cil.h"
#include "dwc_list.h"

int dwc_otg_pcd_ep_enable(dwc_otg_pcd_t * pcd, usb_endpoint_descriptor_t * ep_desc, void *usb_ep);

/** Max Transfer size for any EP */
#define DDMA_MAX_TRANSFER_SIZE 65535

/** Max DMA Descriptor count for any EP */
#define MAX_DMA_DESC_CNT 256

/**
 * States of EP0.
 */
typedef enum ep0_state {
	EP0_DISCONNECT,		/* no host */
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_IN_STATUS_PHASE,
	EP0_OUT_STATUS_PHASE,
	EP0_STALL,
} ep0state_e;

/**
 * Get the pointer to the core_if from the pcd pointer.
 */
#define GET_CORE_IF( _pcd ) (_pcd->core_if)

/** DWC_otg request structure.
 * This structure is a list of requests.
 */
typedef struct dwc_otg_pcd_request {
	void *priv;
	void *buf;
	dwc_dma_t dma;
	uint32_t length;
	uint32_t actual;
	uint8_t sent_zlp;

	 DWC_CIRCLEQ_ENTRY(dwc_otg_pcd_request) queue_entry;
#ifdef DWC_UTE_PER_IO
	struct dwc_iso_xreq_port ext_req;
	//void *priv_ereq_nport; /*  */
#endif
} dwc_otg_pcd_request_t;

DWC_CIRCLEQ_HEAD(req_list, dwc_otg_pcd_request);

typedef struct dwc_otg_pcd_ep {
	/** USB EP Descriptor */
	const usb_endpoint_descriptor_t *desc;

	/** queue of dwc_otg_pcd_requests. */
	struct req_list queue;
	uint8_t stopped;
	uint8_t disabling;
	uint8_t dma;
	uint8_t queue_sof;

	/** DWC_otg ep data. */
	dwc_ep_t dwc_ep;

	/** Pointer to PCD */
	struct dwc_otg_pcd *pcd;

	void *priv;
} dwc_otg_pcd_ep_t;

/** DWC_otg PCD Structure.
 * This structure encapsulates the data for the dwc_otg PCD.
 */
struct dwc_otg_pcd {
	const struct dwc_otg_pcd_function_ops *fops;
	/** Core Interface */
	dwc_otg_core_if_t *core_if;
	/** State of EP0 */
	ep0state_e ep0state;
	/** EP0 Request is pending */
	uint8_t ep0_pending;
	/** Indicates when SET CONFIGURATION Request is in process */
	uint8_t request_config;
	/** The state of the Remote Wakeup Enable. */
	uint8_t remote_wakeup_enable;
	/** The state of the B-Device HNP Enable. */
	uint8_t b_hnp_enable;
	/** The state of A-Device HNP Support. */
	uint8_t a_hnp_support;
	/** The state of the A-Device Alt HNP support. */
	uint8_t a_alt_hnp_support;
	/** Count of pending Requests */
	uint8_t request_pending;

	/** SETUP packet for EP0
	 * This structure is allocated as a DMA buffer on PCD initialization
	 * with enough space for up to 3 setup packets.
	 */
	union {
		usb_device_request_t req;
		uint32_t d32[2];
	} *setup_pkt;

	dwc_dma_t setup_pkt_dma_handle;

	/** 2-byte dma buffer used to return status from GET_STATUS */
	uint16_t *status_buf;
	dwc_dma_t status_buf_dma_handle;

	/** EP0 */
	dwc_otg_pcd_ep_t ep0;

	/** Array of IN EPs. */
	dwc_otg_pcd_ep_t in_ep[MAX_EPS_CHANNELS - 1];
	/** Array of OUT EPs. */
	dwc_otg_pcd_ep_t out_ep[MAX_EPS_CHANNELS - 1];
	/** number of valid EPs in the above array. */
	uint8_t      num_eps;
//	dwc_spinlock_t *lock;

	/** Tasklet to defer starting of TEST mode transmissions until
	 *	Status Phase has been completed.
	 */
//	dwc_tasklet_t *test_mode_tasklet;

	/** Tasklet to delay starting of xfer in DMA mode */
//	dwc_tasklet_t *start_xfer_tasklet;

	/** The test mode to enter when the tasklet is executed. */
	uint8_t test_mode;
};

dwc_otg_pcd_t *dwc_otg_pcd_init(dwc_otg_core_if_t * core_if);
void dwc_otg_request_nuke(dwc_otg_pcd_ep_t * ep);
void dwc_otg_request_done(dwc_otg_pcd_ep_t * ep, dwc_otg_pcd_request_t * req, int32_t status);

#endif

