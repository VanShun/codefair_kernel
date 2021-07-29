
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/usb/gadget.h>

#include "dwc_otg_pcd_if.h"
#include "dwc_otg_pcd.h"
#include "dwc_otg_driver.h"

#define log_out printk

static struct gadget_wrapper {
	dwc_otg_pcd_t *pcd;

	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;

	struct usb_ep ep0;
	struct usb_ep in_ep[16];
	struct usb_ep out_ep[16];
} *gadget_wrapper;
struct gadget_wrapper g_gadget_wrapper;

usb_endpoint_descriptor_t  eptemp;

static struct gadget_wrapper *alloc_wrapper(dwc_otg_device_t *_dev)
{
	memset(&g_gadget_wrapper, 0, sizeof(g_gadget_wrapper));
	g_gadget_wrapper.pcd = _dev->pcd;
	return (&g_gadget_wrapper);
}

static const struct dwc_otg_pcd_function_ops fops = {
};

void gadget_add_eps(struct gadget_wrapper *d)
{
    #if 0
	static const char *names[] = {
		"ep0",
		"ep1in",
		"ep2in",
		"ep3in",
		"ep4in",
		"ep5in",
		"ep6in",
		"ep7in",
		"ep8in",
		"ep9in",
		"ep10in",
		"ep11in",
		"ep12in",
		"ep13in",
		"ep14in",
		"ep15in",
		"ep1out",
		"ep2out",
		"ep3out",
		"ep4out",
		"ep5out",
		"ep6out",
		"ep7out",
		"ep8out",
		"ep9out",
		"ep10out",
		"ep11out",
		"ep12out",
		"ep13out",
		"ep14out",
		"ep15out"
	};
	#endif

//	dwc_otg_pcd_ep_enable(d->pcd, NULL, &d->ep0);

//	ep1in.bDescriptorType = 0x01;
	eptemp.bLength = 0x07;
	eptemp.bDescriptorType = 0x24;
	eptemp.bEndpointAddress = 0x01;
	eptemp.bmAttributes = UE_BULK;
	eptemp.wMaxPacketSize = 512;
	eptemp.bInterval = 0x01;

	dwc_otg_pcd_ep_enable(d->pcd, &eptemp, &d->out_ep[1]);

	eptemp.bLength = 0x07;
	eptemp.bDescriptorType = 0x24;
	eptemp.bEndpointAddress = 0x81;
	eptemp.bmAttributes = UE_BULK;
	eptemp.wMaxPacketSize = 512;
	eptemp.bInterval = 0x01;

	dwc_otg_pcd_ep_enable(d->pcd, &eptemp, &d->in_ep[1]);
}

/**
 * This function initialized the PCD portion of the driver.
 *
 */
int pcd_init(dwc_otg_device_t *_dev)
{
	int retval = 0;
    dwc_otg_device_t *otg_dev ;
	otg_dev = _dev;

	otg_dev->pcd = dwc_otg_pcd_init(otg_dev->core_if);

	/*
	 * Initialize EP structures
	 */
	gadget_wrapper = alloc_wrapper(_dev);

	/*
	 * Initialize EP structures
	 */
	 gadget_add_eps(gadget_wrapper);

	printk("---------------------pcd_start-1---------------------\n");
    //dwc_otg_pcd_start(gadget_wrapper->pcd, &fops);

	return retval;
}

