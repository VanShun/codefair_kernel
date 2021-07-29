#ifndef DWC_OTG_DRIVER_H_
#define DWC_OTG_DRIVER_H_


#include "dwc_otg_core_if.h"
#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/platform_device.h>

/**
 * This structure is a wrapper that encapsulates the driver components used to
 * manage a single DWC_otg controller.
 */
typedef struct dwc_otg_device {
	/** Base address returned from ioremap() */
	void *base;

	/** Pointer to the core interface structure. */
	dwc_otg_core_if_t *core_if;

	/** Register offset for Diagnostic API. */
	uint32_t reg_offset;

	/** Pointer to the PCD structure. */
	struct dwc_otg_pcd *pcd;

	/** Pointer to the HCD structure. */
	struct dwc_otg_hcd *hcd;

	/** Flag to indicate whether the common IRQ handler is installed. */
	uint8_t common_irq_installed;

} dwc_otg_device_t;


irqreturn_t dwc_usb_otg_irq(int irq, void *_udc);
int dwc_otg_driver_probe(void __iomem * baseaddr);

#endif




