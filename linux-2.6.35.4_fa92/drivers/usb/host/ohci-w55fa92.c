/***********************************************************************
 *
 * 
 * Copyright (c) 2008 Nuvoton Technology
 * All rights reserved.
 *
 * 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Changelog:
 *  
 *
 ***********************************************************************/


#include <linux/platform_device.h>
#include <linux/signal.h>

#include <linux/clk.h>
#include <mach/w55fa92_reg.h>

#define DBG_PRINTF			printk
//#define DBG_PRINTF(...)

/**
 * usb_hcd_ppc_soc_probe - initialize On-Chip HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */

extern unsigned int w55fa92_apll_clock;
extern unsigned int w55fa92_upll_clock;

static int usb_hcd_w55fa92_probe(const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd;
	struct ohci_hcd *ohci ;

	DBG_PRINTF("%s\n",__FUNCTION__);	
	
	hcd = usb_create_hcd(driver, &pdev->dev, "w55fa92-ohci");
	if (!hcd)
		return -ENOMEM;
		
	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		printk(__FILE__ ": request_mem_region failed\n");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		printk(__FILE__ ": ioremap failed\n");
		retval = -ENOMEM;
		goto err2;
	}

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_DISABLED);

	//return 0;
	
	if (retval == 0)
		return retval;

	printk("Due to some thing wrong! Removing W55FA92 USB Controller\n");

	iounmap(hcd->regs);
 err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
 err1:
 
 	usb_put_hcd(hcd);
	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_ppc_soc_remove - shutdown processing for On-Chip HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_ppc_soc_probe().
 * It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_w55fa92_remove(struct usb_hcd *hcd,
		struct platform_device *dev)
{
	DBG_PRINTF("%s\n",__FUNCTION__);		
	usb_remove_hcd(hcd);

	//pr_debug("stopping W55FA92 USB Controller\n");

	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}


static int ohci_w55fa92_start (struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int ret;
	DBG_PRINTF("%s\n",__FUNCTION__);	
	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run (ohci)) < 0) {
		printk ("can't start %s", hcd->self.bus_name);
		ohci_stop (hcd);
		return ret;
	}

	return 0;
}


static const struct hc_driver ohci_w55fa92_hc_driver = {
	.description =		hcd_name,
	.product_desc = 	"Nuvoton W55FA92 OHCI Host Controller",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =        ohci_w55fa92_start,
	.stop =			ohci_stop,
	.shutdown = 		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};


static int ohci_hcd_w55fa92_drv_probe(struct platform_device *pdev)
{
	int ret;
	DBG_PRINTF("%s\n",__FUNCTION__);	
	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_w55fa92_probe(&ohci_w55fa92_hc_driver, pdev);
	return ret;
}

static int ohci_hcd_w55fa92_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_w55fa92_remove(hcd, pdev);
	return 0;
}

static struct platform_driver ohci_hcd_w55fa92_driver = {
	.probe		= ohci_hcd_w55fa92_drv_probe,
	.remove		= ohci_hcd_w55fa92_drv_remove,
#ifdef	CONFIG_PM

#endif
	.driver		= {
		.name	= "w55fa92-ohci",
		.owner	= THIS_MODULE,
	},
};

