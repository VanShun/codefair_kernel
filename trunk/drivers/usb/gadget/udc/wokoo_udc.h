
#ifndef WOKOO_UDC_H
#define WOKOO_UDC_H

#define	NUM_ENDPOINTS	6

struct wokoo_udc;

struct wokoo_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	struct wokoo_udc		*udc;
	void __iomem			*creg;

	unsigned			maxpacket:16;
	u8					int_mask;
	unsigned			is_pingpong:1;

	unsigned			stopped:1;
	unsigned			is_in:1;
	unsigned			is_iso:1;
	unsigned			fifo_bank:1;
};

struct wokoo_udc {
	struct clk *hclk;
	struct clk *mclk;
	void __iomem *regmap;
	void __iomem *ctl_map;
	struct usb_gadget   gadget;
    struct wokoo_ep		ep[NUM_ENDPOINTS];
    struct usb_gadget_driver	*driver;

    unsigned			vbus:1;
	unsigned			enabled:1;
	unsigned			clocked:1;
	unsigned			suspended:1;
	unsigned			req_pending:1;
	unsigned			wait_for_addr_ack:1;
	unsigned			wait_for_config_ack:1;
	unsigned			active_suspend:1;

    struct platform_device		*pdev;
	spinlock_t  lock;
};

struct wokoo_request {
	struct usb_request		req;
	struct list_head		queue;
};

void done(struct wokoo_ep *ep, struct wokoo_request *req, int status);
void nuke(struct wokoo_ep *ep, int status);

#endif

