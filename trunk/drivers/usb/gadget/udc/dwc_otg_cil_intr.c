
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>

#include "dwc_otg_cil.h"
#include "dwc_otg_regs.h"

#include "dwc_os.h"

#define log_out printk


struct dwc_otg_dev_regs_backup g_dev_regs_backup;
struct dwc_otg_global_regs_backup g_global_regs_backup;
/** This function will log a debug message
 *
 * @param core_if Programming view of DWC_otg controller.
 */
int32_t dwc_otg_handle_mode_mismatch_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gintsts;

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.modemismatch = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This function handles the OTG Interrupts. It reads the OTG
 * Interrupt Register (GOTGINT) to determine what interrupt has
 * occurred.
 *
 * @param core_if Programming view of DWC_otg controller.
 */

int32_t dwc_otg_handle_otg_intr(dwc_otg_core_if_t * core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	gotgint_data_t gotgint;
	gotgctl_data_t gotgctl;


	gotgint.d32 = dwc_read_reg32(&global_regs->gotgint);
	gotgctl.d32 = dwc_read_reg32(&global_regs->gotgctl);


	if (gotgint.b.sesenddet) {

		gotgctl.d32 = dwc_read_reg32(&global_regs->gotgctl);

		if (core_if->op_state == B_HOST) {
			cil_pcd_start(core_if);
			core_if->op_state = B_PERIPHERAL;
		} else {
			/* If not B_HOST and Device HNP still set. HNP
			 * Did not succeed!*/
			if (gotgctl.b.devhnpen) {

				__DWC_ERROR("Device Not Connected/Responding!\n");
			}

			/* If Session End Detected the B-Cable has
			 * been disconnected. */
			/* Reset PCD and Gadget driver to a
			 * clean state. */
			core_if->lx_state = DWC_OTG_L0;
			cil_pcd_stop(core_if);


		}

		gotgctl.d32 = 0;
		gotgctl.b.devhnpen = 1;
		dwc_modify_reg32(&global_regs->gotgctl, gotgctl.d32, 0);
	}

	if (gotgint.b.sesreqsucstschng) {

		gotgctl.d32 = dwc_read_reg32(&global_regs->gotgctl);
		if (gotgctl.b.sesreqscs) {

			if ((core_if->core_params->phy_type ==
			     DWC_PHY_TYPE_PARAM_FS) && (core_if->core_params->i2c_enable)) {
				core_if->srp_success = 1;
			} else {
				cil_pcd_resume(core_if);
				/* Clear Session Request */
				gotgctl.d32 = 0;
				gotgctl.b.sesreq = 1;
				dwc_modify_reg32(&global_regs->gotgctl,
						 gotgctl.d32, 0);
			}
		}
	}

	if (gotgint.b.hstnegsucstschng) {
		/* Print statements during the HNP interrupt handling
		 * can cause it to fail.*/
		gotgctl.d32 = dwc_read_reg32(&global_regs->gotgctl);
		if (gotgctl.b.hstnegscs) {

		} else {
			gotgctl.d32 = 0;
			gotgctl.b.hnpreq = 1;
			gotgctl.b.devhnpen = 1;
			dwc_modify_reg32(&global_regs->gotgctl, gotgctl.d32, 0);

			__DWC_ERROR("Device Not Connected/Responding\n");
		}
	}

	if (gotgint.b.hstnegdet) {
		/* The disconnect interrupt is set at the same time as
		 * Host Negotiation Detected.  During the mode
		 * switch all interrupts are cleared so the disconnect
		 * interrupt handler will not get executed.
		 */

		if (dwc_otg_mode(core_if) != DWC_HOST_MODE) {

			cil_hcd_disconnect(core_if);
			cil_pcd_start(core_if);
			core_if->op_state = A_PERIPHERAL;
		} else {
		}
	}
	if (gotgint.b.adevtoutchng) {

	}
	if (gotgint.b.debdone) {

	}

	/* Clear GOTGINT */
	dwc_write_reg32(&core_if->core_global_regs->gotgint, gotgint.d32);

	return 1;
}

/**
 * This function handles the Connector ID Status Change Interrupt.  It
 * reads the OTG Interrupt Register (GOTCTL) to determine whether this
 * is a Device to Host Mode transition or a Host Mode to Device
 * Transition.
 *
 * This only occurs when the cable is connected/removed from the PHY
 * connector.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
int32_t dwc_otg_handle_conn_id_status_change_intr(dwc_otg_core_if_t * core_if)
{

	/*
	 * Need to disable SOF interrupt immediately. If switching from device
	 * to host, the PCD interrupt handler won't handle the interrupt if
	 * host mode is already set. The HCD interrupt handler won't get
	 * called if the HCD state is HALT. This means that the interrupt does
	 * not get handled and Linux complains loudly.
	 */
	gintmsk_data_t gintmsk;
	gintsts_data_t gintsts;
	gintmsk.d32 = 0;
	gintsts.d32 = 0;
	gintmsk.b.sofintr = 1;
	dwc_modify_reg32(&core_if->core_global_regs->gintmsk, gintmsk.d32, 0);

	/* Set flag and clear interrupt */
	gintsts.b.conidstschng = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}
/**
 * This interrupt indicates that a device has been disconnected from
 * the root port.
 */
int32_t dwc_otg_handle_disconnect_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gintsts;


/** @todo Consolidate this if statement. */
#ifndef DWC_HOST_ONLY
	if (core_if->op_state == B_HOST) {

	} else if (dwc_otg_mode(core_if) != DWC_HOST_MODE) {
		gotgctl_data_t gotgctl;
		gotgctl.d32 = 0;
		gotgctl.d32 = dwc_read_reg32(&core_if->core_global_regs->gotgctl);
		if (gotgctl.b.hstsethnpen == 1) {
			/* Do nothing, if HNP in process the OTG
			 * interrupt "Host Negotiation Detected"
			 * interrupt will do the mode switch.
			 */
		} else if (gotgctl.b.devhnpen == 0) {
			/* If in device mode Disconnect and stop the HCD, then
			 * start the PCD. */
			cil_hcd_disconnect(core_if);
			cil_pcd_start(core_if);
			core_if->op_state = B_PERIPHERAL;
		} else {

		}
	} else {
		if (core_if->op_state == A_HOST) {
			/* A-Cable still connected but device disconnected. */
			cil_hcd_disconnect(core_if);

		}
	}
#endif
	/* Change to L3(OFF) state */
	core_if->lx_state = DWC_OTG_L3;

	gintsts.d32 = 0;
	gintsts.b.disconnect = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}
/**
 * This interrupt indicates that a device is initiating the Session
 * Request Protocol to request the host to turn on bus power so a new
 * session can begin. The handler responds by turning on bus power. If
 * the DWC_otg controller is in low power mode, the handler brings the
 * controller out of low power mode before turning on bus power.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
int32_t dwc_otg_handle_session_req_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gintsts;

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.sessreqintr = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}
/**
 * This interrupt indicates that the DWC_otg controller has detected a
 * resume or remote wakeup sequence. If the DWC_otg controller is in
 * low power mode, the handler must brings the controller out of low
 * power mode. The controller automatically begins resume
 * signaling. The handler schedules a time to stop resume signaling.
 */
int32_t dwc_otg_handle_wakeup_detected_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gintsts;

//	if (dwc_otg_mode(core_if) != DWC_HOST_MODE ) {
//		dctl_data_t dctl;
//		dctl.d32 = 0;
//		if (core_if->lx_state == DWC_OTG_L2) {
//			/* Clear the Remote Wakeup Signaling */
//			dctl.b.rmtwkupsig = 1;
//			dwc_modify_reg32(&core_if->dev_if->dev_global_regs->
//					 dctl, dctl.d32, 0);
//
//			if (core_if->pcd_cb && core_if->pcd_cb->resume_wakeup) {
//				core_if->pcd_cb->resume_wakeup(core_if->pcd_cb->
//							       p);
//			}
//		} else {
//			glpmcfg_data_t lpmcfg;
//			lpmcfg.d32 =
//			    dwc_read_reg32(&core_if->core_global_regs->glpmcfg);
//			lpmcfg.b.hird_thres &= (~(1 << 4));
//			dwc_write_reg32(&core_if->core_global_regs->glpmcfg,
//					lpmcfg.d32);
//		}
//		/** Change to L0 state*/
//		core_if->lx_state = DWC_OTG_L0;
//	} else {
//	}

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.wkupintr = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}

int dwc_otg_save_dev_regs(dwc_otg_core_if_t *core_if) {
	struct dwc_otg_dev_regs_backup *dr;
	int i;

	dr = core_if->dr_backup;
	if (!dr) {
		dr = &g_dev_regs_backup;

		core_if->dr_backup = dr;
	}

	dr->dcfg = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dcfg);
	dr->dctl = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dctl);
	dr->daintmsk = dwc_read_reg32(&core_if->dev_if->dev_global_regs->daintmsk);
	dr->diepmsk = dwc_read_reg32(&core_if->dev_if->dev_global_regs->diepmsk);
	dr->doepmsk = dwc_read_reg32(&core_if->dev_if->dev_global_regs->doepmsk);

	for (i = 0; i < core_if->dev_if->num_in_eps; ++i) {
		dr->diepctl[i] = dwc_read_reg32(
				&core_if->dev_if->in_ep_regs[i]->diepctl);
		dr->dieptsiz[i] = dwc_read_reg32(
				&core_if->dev_if->in_ep_regs[i]->dieptsiz);
		dr->diepdma[i] = dwc_read_reg32(
				&core_if->dev_if->in_ep_regs[i]->diepdma);
	}

	for (i = 0; i < core_if->dev_if->num_out_eps; ++i) {
		dr->doepfn[i] = dwc_read_reg32(
				&core_if->dev_if->out_ep_regs[i]->doepfn);
	}

	for (i = 0; i < core_if->dev_if->num_in_eps; ++i) {

	}

	return 0;
}

/** Saves some register values into system memory. */
int dwc_otg_save_global_regs(dwc_otg_core_if_t *core_if) {
	struct dwc_otg_global_regs_backup *gr;
	int i;

	gr = core_if->gr_backup;
	if (!gr) {
		gr = &g_global_regs_backup;
		if (!gr) {
			return -DWC_E_NO_MEMORY;
		}
		core_if->gr_backup = gr;
	}

	gr->gotgctl_local = dwc_read_reg32(&core_if->core_global_regs->gotgctl);
	gr->gintmsk_local = dwc_read_reg32(&core_if->core_global_regs->gintmsk);
	gr->gahbcfg_local = dwc_read_reg32(&core_if->core_global_regs->gahbcfg);
	gr->gusbcfg_local = dwc_read_reg32(&core_if->core_global_regs->gusbcfg);
	gr->grxfsiz_local = dwc_read_reg32(&core_if->core_global_regs->grxfsiz);
	gr->gnptxfsiz_local = dwc_read_reg32(&core_if->core_global_regs->gnptxfsiz);
	gr->hptxfsiz_local = dwc_read_reg32(&core_if->core_global_regs->hptxfsiz);
	gr->gi2cctl_local = dwc_read_reg32(&core_if->core_global_regs->gi2cctl);
	gr->pcgcctl_local = dwc_read_reg32(core_if->pcgcctl);
	gr->gdfifocfg_local = dwc_read_reg32(&core_if->core_global_regs->gdfifocfg);
	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		gr->dtxfsiz_local[i] = dwc_read_reg32(
				&(core_if->core_global_regs->dtxfsiz[i]));
	}

	return 0;
}

/**
 * This interrupt indicates that SUSPEND state has been detected on
 * the USB.
 *
 * For HNP the USB Suspend interrupt signals the change from
 * "a_peripheral" to "a_host".
 *
 * When power management is enabled the core will be put in low power
 * mode.
 */
int32_t dwc_otg_handle_usb_suspend_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gintsts;
	dcfg_data_t dcfg;

	if (dwc_otg_mode(core_if) != DWC_HOST_MODE ) {
		/* Check the Device status register to determine if the Suspend
		 * state is active. */

		/* PCD callback for suspend. */
//		cil_pcd_suspend(core_if);

		if (core_if->power_down == 2)
		{
			log_out("suspend pos 1\n");
			dcfg.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dcfg);

			if (core_if->lx_state != DWC_OTG_L3 && dcfg.b.devaddr) {

				pcgcctl_data_t pcgcctl;
				gpwrdn_data_t gpwrdn;
				gusbcfg_data_t gusbcfg;

				pcgcctl.d32 = 0;
				gpwrdn.d32 = 0;
				gusbcfg.d32 = 0;

				log_out("suspend pos 2\n");
				/* Change to L2(suspend) state */
				core_if->lx_state = DWC_OTG_L2;

				/* Clear interrupt in gintsts*/
				gintsts.d32 = 0;
				gintsts.b.usbsuspend = 1;
				dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);

				dwc_otg_save_global_regs(core_if);
				dwc_otg_save_dev_regs(core_if);

				gusbcfg.d32 = dwc_read_reg32(&core_if->core_global_regs->gusbcfg);
				if (gusbcfg.b.ulpi_utmi_sel == 1) {
				/* ULPI interface */
					/* Suspend the Phy Clock */
					pcgcctl.d32 = 0;
					pcgcctl.b.stoppclk = 1;
					dwc_modify_reg32(core_if->pcgcctl, 0, pcgcctl.d32);
					dwc_udelay(10);
					gpwrdn.b.pmuactv = 1;
					dwc_modify_reg32(&core_if->core_global_regs->gpwrdn, 0, gpwrdn.d32);
					log_out("suspend pos 3a\n");
				} else {
				/* UTMI+ Interface */
					gpwrdn.b.pmuactv = 1;
					dwc_modify_reg32(&core_if->core_global_regs->gpwrdn, 0, gpwrdn.d32);
					dwc_udelay(10);
					pcgcctl.b.stoppclk = 1;
					dwc_modify_reg32(core_if->pcgcctl, 0, pcgcctl.d32);
					dwc_udelay(10);
					log_out("suspend pos 3b\n");
				}

				/* Set flag to indicate that we are in hibernation*/
				core_if->hibernation_suspend = 1;
				/* Enable interrupts from wake up logic*/
				gpwrdn.d32 = 0;
				gpwrdn.b.pmuintsel = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gpwrdn, 0, gpwrdn.d32);
				dwc_udelay(10);

				/* Unmask device mode interrupts in GPWRDN*/
				gpwrdn.d32 = 0;
				gpwrdn.b.rst_det_msk = 1;
				gpwrdn.b.lnstchng_msk = 1;
				gpwrdn.b.sts_chngint_msk = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gpwrdn, 0, gpwrdn.d32);
				dwc_udelay(10);

				/* Enable Power Down Clamp */
				gpwrdn.d32 = 0;
				gpwrdn.b.pwrdnclmp = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gpwrdn, 0, gpwrdn.d32);
				dwc_udelay(10);

				/* Switch off VDD */
				gpwrdn.d32 = 0;
				gpwrdn.b.pwrdnswtch = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gpwrdn, 0, gpwrdn.d32);

				/* Save gpwrdn register for further usage if stschng interrupt*/
				core_if->gr_backup->gpwrdn_local =
							dwc_read_reg32(&core_if->core_global_regs->gpwrdn);

				log_out("suspend pos 4\n");
				return 1;

			}
		}
	}

	/* Change to L2(suspend) state */
	core_if->lx_state = DWC_OTG_L2;

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.usbsuspend = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);
	log_out("suspend pos 5\n");
	return 1;

}

/**
 * This function returns the Core Interrupt register.
 */
static __inline uint32_t dwc_otg_read_common_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gintsts;
	gintmsk_data_t gintmsk;
	gintmsk_data_t gintmsk_common;
	gintmsk_common.d32 = 0 ;
	gintmsk_common.b.wkupintr = 1;
	gintmsk_common.b.sessreqintr = 1;
	gintmsk_common.b.conidstschng = 1;
	gintmsk_common.b.otgintr = 1;
	gintmsk_common.b.modemismatch = 1;
	gintmsk_common.b.disconnect = 1;
	gintmsk_common.b.usbsuspend = 1;
	gintmsk_common.b.restoredone = 1;
	/** @todo: The port interrupt occurs while in device
         * mode. Added code to CIL to clear the interrupt for now!
         */
	gintmsk_common.b.portintr = 1;
	gintsts.d32 = dwc_read_reg32(&core_if->core_global_regs->gintsts);
	gintmsk.d32 = dwc_read_reg32(&core_if->core_global_regs->gintmsk);

	return ((gintsts.d32 & gintmsk.d32) & gintmsk_common.d32);

}

/* MACRO for clearing interupt bits in GPWRDN register */
#define CLEAR_GPWRDN_INTR(__core_if,__intr) \
do { \
		gpwrdn_data_t gpwrdn; \
		gpwrdn.d32=0; \
		gpwrdn.b.__intr = 1; \
		dwc_modify_reg32(&__core_if->core_global_regs->gpwrdn, 0, gpwrdn.d32);	\
} while (0)

/**
 * Common interrupt handler.
 *
 * The common interrupts are those that occur in both Host and Device mode.
 * This handler handles the following interrupts:
 * - Mode Mismatch Interrupt
 * - Disconnect Interrupt
 * - OTG Interrupt
 * - Connector ID Status Change Interrupt
 * - Session Request Interrupt.
 * - Resume / Remote Wakeup Detected Interrupt.
 * - LPM Transaction Received Interrupt
 * - ADP Transaction Received Interrupt
 *
 */
int32_t dwc_otg_handle_common_intr(dwc_otg_core_if_t * core_if)
{
	int retval = 0;
	gintsts_data_t gintsts;

	if (core_if->hibernation_suspend <= 0) {

		gintsts.d32 = dwc_otg_read_common_intr(core_if);

		if (gintsts.b.modemismatch) {
			retval |= dwc_otg_handle_mode_mismatch_intr(core_if);
		}

		if (gintsts.b.otgintr) {
			retval |= dwc_otg_handle_otg_intr(core_if);
		}

		if (gintsts.b.conidstschng) {
			retval |= dwc_otg_handle_conn_id_status_change_intr(core_if);
		}

		if (gintsts.b.disconnect) {
			retval |= dwc_otg_handle_disconnect_intr(core_if);
		}

		if (gintsts.b.sessreqintr) {
			retval |= dwc_otg_handle_session_req_intr(core_if);
		}

		if (gintsts.b.wkupintr) {
			retval |= dwc_otg_handle_wakeup_detected_intr(core_if);
		}

		if (gintsts.b.usbsuspend) {
			retval |= dwc_otg_handle_usb_suspend_intr(core_if);
		}

		if (gintsts.b.portintr && (dwc_otg_mode(core_if) != DWC_HOST_MODE )) {
			gintsts.d32 = 0;
			gintsts.b.portintr = 1;
			dwc_write_reg32(&core_if->core_global_regs->gintsts,gintsts.d32);
			retval |= 1;
		}
	}

	return retval;
}

