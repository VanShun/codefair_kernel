
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
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/unaligned.h>

#include "wokoo_udc.h"
#include "dwc_otg_core_if.h"
#include "dwc_otg_cil.h"
#include "dwc_otg_driver.h"
#include "dwc_otg_pcd.h"
#include "dwc_otg_regs.h"
#include "dwc_otg_pcd_if.h"

extern int dwc_otg_driver_probe(void __iomem * baseaddr);
//extern int32_t dwc_otg_pcd_handle_intr(dwc_otg_pcd_t * pcd);


extern int wokoo_ep_deal(struct wokoo_ep *_ep, struct wokoo_request *_req);


static const char driver_name [] = "wokoo_udc";

static const struct {
	const char *name;
	const struct usb_ep_caps caps;
} ep_info[] = {
#define EP_INFO(_name, _caps) \
	{ \
		.name = _name, \
		.caps = _caps, \
	}

	EP_INFO("ep0",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_CONTROL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep1",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep2",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep3-int",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_INT, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep4",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep5",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),

#undef EP_INFO
};

#define ep0name		ep_info[0].name






static int wokoo_start(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
{
	struct wokoo_udc	*udc;

	udc = container_of(gadget, struct wokoo_udc, gadget);
	udc->driver = driver;
	udc->gadget.dev.of_node = udc->pdev->dev.of_node;
	udc->enabled = 1;
	udc->gadget.is_selfpowered = 1;

	return 0;
}

static const struct usb_gadget_ops wokoo_udc_ops = {
	.udc_start		= wokoo_start,
};

void done(struct wokoo_ep *ep, struct wokoo_request *req, int status)
{
	unsigned	stopped = ep->stopped;
	//struct wokoo_udc	*udc = ep->udc;

	list_del_init(&req->queue);

	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;
	if (status && status != -ESHUTDOWN)
		printk("%s done %p, status %d\n", ep->ep.name, req, status);

	ep->stopped = 1;

	//spin_unlock(&udc->lock);

	usb_gadget_giveback_request(&ep->ep, &req->req);

	//spin_lock(&udc->lock);

	ep->stopped = stopped;


	/* ep0 is always ready; other endpoints need a non-empty queue */
	if (list_empty(&ep->queue) && ep->int_mask != (1 << 0)) {
		//at91_udp_write(udc, AT91_UDP_IDR, ep->int_mask);
	}
}

void nuke(struct wokoo_ep *ep, int status)
{
	struct wokoo_request *req;

	/* terminate any request in the queue */
	ep->stopped = 1;
	if (list_empty(&ep->queue))
		return;

	//VDBG("%s %s\n", __func__, ep->ep.name);
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct wokoo_request, queue);
		done(ep, req, status);
	}
}

static int wokoo_ep_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
    printk("xxxxxxxxxxxxxxxxxxxx1111");
	struct wokoo_ep	*ep = container_of(_ep, struct wokoo_ep, ep);
	struct wokoo_udc *udc;
	u16		maxpacket;
	u32		tmp;
	unsigned long	flags;

	if (!_ep || !ep
			|| !desc || _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| (maxpacket = usb_endpoint_maxp(desc)) == 0
			|| maxpacket > ep->maxpacket) {
		printk("bad ep or descriptor\n");
		return -EINVAL;
	}

	udc = ep->udc;
	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		printk("bogus device state\n");
		return -ESHUTDOWN;
	}

	tmp = usb_endpoint_type(desc);
	switch (tmp) {
	case USB_ENDPOINT_XFER_CONTROL:
		printk("only one control endpoint\n");
		return -EINVAL;
	case USB_ENDPOINT_XFER_INT:
		if (maxpacket > 64)
			goto bogus_max;
		break;
	case USB_ENDPOINT_XFER_BULK:
		switch (maxpacket) {
		case 8:
		case 16:
		case 32:
		case 64:
			goto ok;
		}
bogus_max:
		printk("bogus maxpacket %d\n", maxpacket);
		return -EINVAL;
	case USB_ENDPOINT_XFER_ISOC:
		if (!ep->is_pingpong) {
			printk("iso requires double buffering\n");
			return -EINVAL;
		}
		break;
	}

ok:
	spin_lock_irqsave(&udc->lock, flags);

	/* initialize endpoint to match this descriptor */
	ep->is_in = usb_endpoint_dir_in(desc);
	ep->is_iso = (tmp == USB_ENDPOINT_XFER_ISOC);
	ep->stopped = 0;
	if (ep->is_in)
		tmp |= 0x04;
	//tmp <<= 8;
	//tmp |= AT91_UDP_EPEDS;
	//__raw_writel(tmp, ep->creg);

	ep->ep.maxpacket = maxpacket;

	/*
	 * reset/init endpoint fifo.  NOTE:  leaves fifo_bank alone,
	 * since endpoint resets don't reset hw pingpong state.
	 */
	//at91_udp_write(udc, AT91_UDP_RST_EP, ep->int_mask);
	//at91_udp_write(udc, AT91_UDP_RST_EP, 0);

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int wokoo_ep_disable (struct usb_ep * _ep)
{
	struct wokoo_ep	*ep = container_of(_ep, struct wokoo_ep, ep);
	struct wokoo_udc	*udc = ep->udc;
	unsigned long	flags;

	if (ep == &ep->udc->ep[0])
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	nuke(ep, -ESHUTDOWN);

	/* restore the endpoint's pristine config */
	ep->ep.desc = NULL;
	ep->ep.maxpacket = ep->maxpacket;

	/* reset fifos and endpoint */
	if (ep->udc->clocked) {
		//at91_udp_write(udc, AT91_UDP_RST_EP, ep->int_mask);
		//at91_udp_write(udc, AT91_UDP_RST_EP, 0);
		//__raw_writel(0, ep->creg);
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static struct usb_request *wokoo_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct wokoo_request *req;

	req = kzalloc(sizeof (struct wokoo_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void wokoo_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
    struct wokoo_request *req;

    req = container_of(_req, struct wokoo_request, req);

    BUG_ON(!list_empty(&req->queue));

    kfree(req);
}

static int wokoo_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct wokoo_request	*req;
	struct wokoo_ep			*ep;
	struct wokoo_udc		*udc;
	int						status;
	unsigned long			flags;

	req = container_of(_req, struct wokoo_request, req);
	ep = container_of(_ep, struct wokoo_ep, ep);

	if (!_req || !_req->complete || !_req->buf || !list_empty(&req->queue)) {
		printk("invalid request\n");
		return -EINVAL;
	}

	if (!_ep || (!ep->ep.desc && ep->ep.name != ep0name)) {
		printk("invalid ep\n");
		return -EINVAL;
	}

	udc = ep->udc;

	if (!udc || !udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		printk("invalid device\n");
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	spin_lock_irqsave(&udc->lock, flags);

	/* try to kickstart any empty and idle queue */
	if (list_empty(&ep->queue) && !ep->stopped) {
		int	is_ep0;

		/*
		 * If this control request has a non-empty DATA stage, this
		 * will start that stage.  It works just like a non-control
		 * request (until the status stage starts, maybe early).
		 *
		 * If the data stage is empty, then this starts a successful
		 * IN/STATUS stage.  (Unsuccessful ones use set_halt.)
		 */
		is_ep0 = (ep->ep.name == ep0name);


		if (ep->is_in){
            wokoo_ep_deal(ep,   req);
		//	status = write_fifo(ep, req); ///ttttttttttttttttttttttttttttt
		}
		else {
		//	status = read_fifo(ep, req);
			printk("fdgdgfdgfdgfgf");
			/* IN/STATUS stage is otherwise triggered by irq */
		//	if (status && is_ep0)
		//		goto ep0_in_status;
		}
	} else
		status = 0;

	if (req && !status) {
		list_add_tail (&req->queue, &ep->queue);
		//at91_udp_write(udc, AT91_UDP_IER, ep->int_mask);
	}
done:
	spin_unlock_irqrestore(&udc->lock, flags);
	return (status < 0) ? status : 0;
}

static const struct usb_ep_ops wokoo_ep_ops = {
    .enable		= wokoo_ep_enable,
	.disable	= wokoo_ep_disable,
	.alloc_request	= wokoo_ep_alloc_request,
	.free_request	= wokoo_ep_free_request,
	.queue		= wokoo_ep_queue,
	//.dequeue	= wokoo_ep_dequeue,
	//.set_halt	= wokoo_ep_set_halt,
};

static void udc_reinit(struct wokoo_udc *udc)
{
    u32 i;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);
	udc->gadget.quirk_stall_not_supp = 1;

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct wokoo_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		ep->ep.desc = NULL;
		ep->stopped = 0;
		ep->fifo_bank = 0;
		usb_ep_set_maxpacket_limit(&ep->ep, ep->maxpacket);
		//ep->creg = (void __iomem *) udc->regmap + AT91_UDP_CSR(i);
		/* initialize one queue per endpoint */
		INIT_LIST_HEAD(&ep->queue);
	}
}

static int wokoo_udc_probe(struct platform_device *pdev)
{
	struct device *dev =  &pdev->dev;
    struct wokoo_ep	*ep;
	struct wokoo_udc *udc;

	struct resource *res0, *res1;
	uint32_t tmp, i;
	int ret = 0, irq = 0;

	printk("*********************************************************\n");
	printk("wokoo_udc_probe!\n");

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	udc->regmap = devm_ioremap_resource(&pdev->dev, res0);

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	udc->ctl_map = devm_ioremap_resource(&pdev->dev, res1);

	writel_relaxed(0x00000000, udc->ctl_map + 0x150);

    tmp = readl_relaxed(udc->ctl_map + 0x154);
	tmp |= 0x00000002;
	tmp &= 0xFFFFFFFE;
	writel_relaxed(tmp, udc->ctl_map + 0x154);
	for (i = 0; i < 0x1000; i++);

	writel_relaxed(0x00000001, udc->ctl_map + 0x150);

	udc->hclk = devm_clk_get(&pdev->dev, "hclk");
    if (IS_ERR(udc->hclk)) {
        ret = PTR_ERR(udc->hclk);
        dev_err(&pdev->dev, "failed to get udc hclk: %d\n", ret);
        return ret;
    }

	ret = clk_prepare_enable(udc->hclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare udc clock\n");
		return ret;
	}

	udc->mclk = devm_clk_get(&pdev->dev, "mclk");
    if (IS_ERR(udc->mclk)) {
        ret = PTR_ERR(udc->mclk);
        dev_err(&pdev->dev, "failed to get udc mclk: %d\n", ret);
        return ret;
    }

	ret = clk_prepare_enable(udc->mclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare udc clock\n");
		return ret;
	}

	for (i = 0; i < 0x1000; i++);

	tmp = readl_relaxed(udc->ctl_map + 0x154);
	tmp |= 0x00000001;
	writel_relaxed(tmp, udc->ctl_map + 0x154);
	for (i = 0; i < 0x1000; i++);

    tmp = readl_relaxed(udc->ctl_map + 0x150);
	tmp |= 0x00000010;
	writel_relaxed(tmp, udc->ctl_map + 0x150);

	dwc_otg_driver_probe(udc->regmap);

	/* init software state */
	udc->gadget.dev.parent = dev;
    //at91udc_of_init(udc, pdev->dev.of_node);
	udc->pdev = pdev;
	udc->enabled = 0;
	spin_lock_init(&udc->lock);

	udc->gadget.ops = &wokoo_udc_ops;
	udc->gadget.ep0 = &udc->ep[0].ep;
	udc->gadget.name = driver_name;
	udc->gadget.dev.init_name = "gadget";
    udc->gadget.speed = USB_SPEED_FULL;

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		ep = &udc->ep[i];
		ep->ep.name = ep_info[i].name;
		ep->ep.caps = ep_info[i].caps;
		ep->ep.ops =  &wokoo_ep_ops;
		ep->udc = udc;
		ep->int_mask = BIT(i);
		if (i != 0 && i != 3)
			ep->is_pingpong = 1;
	}

	udc_reinit(udc);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

    ret = devm_request_irq(&pdev->dev, irq, dwc_usb_otg_irq, 0, dev_name(&pdev->dev), udc);
	if (ret < 0) {
		dev_err(&pdev->dev, "request of irq failed\n");
		return -EBUSY;
	}


	usb_add_gadget_udc(dev, &udc->gadget);

	platform_set_drvdata(pdev, udc);

	printk("probe finish!\n");
	printk("*********************************************************\n");

	return 0;
}

static int wokoo_udc_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id wokoo_udc_dt_ids[] = {
	{ .compatible = "wokoo,wokoo-udc", },
	{ },
};
MODULE_DEVICE_TABLE(platform, wokoo_udc_dt_ids);

static struct platform_driver udc_driver_wokoo = {
	.probe		= wokoo_udc_probe,
	.remove		= wokoo_udc_remove,
	.driver		= {
		.name	= "wokoo-udc",
		.of_match_table = wokoo_udc_dt_ids,
	},
};

module_platform_driver(udc_driver_wokoo);

MODULE_ALIAS("platform:wokoo-udc");
MODULE_DESCRIPTION("WOKOO UDC driver");
MODULE_LICENSE("GPL v2");

