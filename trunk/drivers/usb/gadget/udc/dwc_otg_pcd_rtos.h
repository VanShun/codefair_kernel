
#ifndef DWC_OTG_PCD_RTOS_H_
#define DWC_OTG_PCD_RTOS_H_

#include "dwc_otg_pcd_if.h"
#include "dwc_otg_pcd.h"
#include "dwc_otg_driver.h"
#include <linux/types.h>

int pcd_init(dwc_otg_device_t *_dev);
//void gadget_add_eps(struct gadget_wrapper *d);

#endif

