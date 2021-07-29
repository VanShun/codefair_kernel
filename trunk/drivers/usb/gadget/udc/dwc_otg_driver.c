

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

#include "wokoo_usb.h"
#include "dwc_otg_core_if.h"
#include "dwc_otg_pcd_rtos.h"
#include "wokoo_udc.h"

dwc_otg_device_t g_otg_device;
#define log_out printk

extern int32_t dwc_otg_pcd_handle_intr(dwc_otg_pcd_t * pcd, struct wokoo_udc *_udc);
extern void write_wokoo_fifo(struct wokoo_ep *ep, struct wokoo_request *req,dwc_otg_pcd_t * pcd);


dwc_otg_device_t *otg_dev = &g_otg_device;

irqreturn_t dwc_usb_otg_irq(int irq, void *_udc) {

	int32_t retval = IRQ_NONE;
    struct wokoo_udc		*udc = (struct wokoo_udc *)_udc;

	dwc_otg_pcd_handle_intr(otg_dev->pcd, udc);

	dwc_otg_handle_common_intr(otg_dev->core_if);

    return IRQ_RETVAL(retval);
}

int wokoo_ep_deal(struct wokoo_ep *_ep, struct wokoo_request *_req)
{
	dwc_otg_device_t *dwc_otg_device_send;
	dwc_otg_device_send = &g_otg_device;

	write_wokoo_fifo(_ep, _req,dwc_otg_device_send->pcd);


	return 0;
}

/**
 * This function is called when a lm_device is unregistered with the
 * dwc_otg_driver. This happens, for example, when the rmmod command is
 * executed. The device may or may not be electrically present. If it is
 * present, the driver stops device processing. Any resources used on behalf
 * of this device are freed.
 *
 * @param _dev
 */
//static void dwc_otg_driver_remove(dwc_otg_device_t *_dev)
//{
//}

/*void send_keyvalue(void)
{
	uint32_t i;
	dwc_otg_device_t *dwc_otg_device_send;
	dwc_otg_device_send = &g_otg_device;
	do_endpoint1_trans(dwc_otg_device_send->pcd);
	for(i=0; i<0x100;i++);
	key[0] = 0x00;
	key[2] = 0x00;
	do_endpoint1_trans(dwc_otg_device_send->pcd);

}
void simulate_keypad(const char * sendbyte) {

	uint32_t i;
	dwc_otg_device_t *dwc_otg_device_send;
	dwc_otg_device_send = &g_otg_device;


	while(1)
	{
		if(*sendbyte == 0)
		{
			key[2] = 0x28;
			send_keyvalue();
			break;
		}
		if(*sendbyte > 0x30 && *sendbyte <= 0x39)
		{
			key[2] = *sendbyte -19;
		}
		if(*sendbyte == 0x30)
		{
			key[2] = 0x27;
		}
		if(*sendbyte >= 0x61 && *sendbyte <= 0x7a)
		{
			key[2] = *sendbyte -93;
		}
		if(*sendbyte >= 0x41 && *sendbyte <= 0x5a)
		{
			key[0] = 0x02;
			key[2] = *sendbyte -61;
		}
		if(*sendbyte == 0x3a || *sendbyte == 0x3b)
		{
			if(*sendbyte == 0x3a)
			{
				key[0] = 0x02;
			}
			key[2] = 0x33;
		}
		if(*sendbyte == 0x2F|| *sendbyte == 0x2E)
		{
			key[2] = *sendbyte+ 0x09;
		}
		if(*sendbyte == 0x21 || *sendbyte == 0x23 || *sendbyte == 0x24 || *sendbyte == 0x25)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x03;
		}
		if(*sendbyte == 0x22)
		{
			key[0] = 0x02;
			key[2] = 0x34;
		}
		if(*sendbyte == 0x26 || *sendbyte == 0x28 || *sendbyte == 0x29)
		{
			key[0] = 0x02;
			key[2] = *sendbyte -2;
		}
		if(*sendbyte == 0x27)
		{
			key[2] = 0x34;
		}
		if(*sendbyte == 0x2A)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 5;
		}
		if(*sendbyte == 0x2B)
		{
			key[0] = 0x02;
			key[2] = *sendbyte + 3;
		}

		if(*sendbyte == 0x2C)
		{
			key[2] = *sendbyte + 0x0A;
		}
		if(*sendbyte == 0x2D)
		{
			key[2] = *sendbyte;
		}
		if(*sendbyte == 0x3C)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 6;
		}
		if(*sendbyte == 0x3D)
		{
			key[2] = *sendbyte - 0x0F;
		}
		if(*sendbyte == 0x3E || *sendbyte == 0x3F)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x07;
		}
		if(*sendbyte == 0x40)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x21;
		}
		if(*sendbyte == 0x5B)
		{
			key[2] = *sendbyte - 0x2C;
		}
		if(*sendbyte == 0x5C || *sendbyte == 0x60)
		{
			key[2] = *sendbyte - 0x2B;
		}
		if(*sendbyte == 0x5D)
		{
			key[2] = *sendbyte - 0x2D;
		}
		if(*sendbyte == 0x5E)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x3B;
		}
		if(*sendbyte == 0x5F)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x32;
		}
		if(*sendbyte == 0x7B)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x4C;
		}
		if(*sendbyte == 0x7C)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x4B;
		}
		if(*sendbyte == 0x7D)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x4D;
		}

		if(*sendbyte == 0x7E)
		{
			key[0] = 0x02;
			key[2] = *sendbyte - 0x49;
		}
		send_keyvalue();
		for(i=0; i<0x100;i++);
		sendbyte++;
	}

}*/



/*extern uint16_t sendnum;
void bulk_send()
{

	dwc_otg_device_t *dwc_otg_device_send;
	dwc_otg_device_send = &g_otg_device;

	do_endpoint1_trans(dwc_otg_device_send->pcd, sendnum);

}*/


int dwc_otg_driver_probe(void __iomem * baseaddr)
{
	int retval = 0;
	dwc_otg_device_t *dwc_otg_device = &g_otg_device;

	memset(dwc_otg_device, 0, sizeof(*dwc_otg_device));
	dwc_otg_device->reg_offset = 0xFFFFFFFF;

	dwc_otg_device->base = (void *)baseaddr;

	dwc_otg_device->core_if = dwc_otg_cil_init(dwc_otg_device->base);

	dwc_otg_disable_global_interrupts(dwc_otg_device->core_if);

	dwc_otg_device->common_irq_installed = 1;

	dwc_otg_core_init(dwc_otg_device->core_if);

	retval = pcd_init(dwc_otg_device);

	dwc_otg_enable_global_interrupts(dwc_otg_device->core_if);

	return 0;

//fail:
//	dwc_otg_driver_remove(dwc_otg_device);

//	return retval;
}

