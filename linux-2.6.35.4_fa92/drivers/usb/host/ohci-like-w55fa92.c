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

struct clk  *usbh11_clk;

static void usbh_DisablePorts(
	u8 bDisablePort1,
	u8 bDisablePort2
)
{
	writel(  (readl(REG_HC_RH_OP_MODE) & ~(BIT16 | BIT17)) |  \
						(((bDisablePort1 & 1)<<16) | ((bDisablePort2 & 1)<<17)), \
						REG_HC_RH_OP_MODE); 	
}
static void usbh_LikeModePort(unsigned int u32HlmPort)
{
	if(u32HlmPort==1)
	{
		writel (((readl(REG_GPAFUN1) & ~(MF_GPA11 | MF_GPA10)) | 0x4400), REG_GPAFUN1); 
	}
	else if(u32HlmPort==2)
	{		
		writel (((readl(REG_GPDFUN0) & ~(MF_GPD3 | MF_GPD4)) | 0xCC000), REG_GPDFUN0); 	
	}
	else if(u32HlmPort==3)
	{	
		writel (((readl(REG_GPAFUN0) & ~(MF_GPA4 | MF_GPA3)) | 0x33000), REG_GPAFUN0); 
	}
	else if(u32HlmPort==4)
	{
		writel (((readl(REG_GPDFUN1) & ~(MF_GPD14| MF_GPD15)) | 0x33000000), REG_GPDFUN1); 
	}
}


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

static int usb_hcd_like_w55fa92_probe(const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd;
	struct ohci_hcd *ohci ;
	int disablePort1 = 0, disablePort2=0;
	u32 u32clock ,u32Div = 0;

	DBG_PRINTF("%s\n",__FUNCTION__);	
	
	/*
	 * Config GPIO
	 */
	//writel(readl(REG_GPIOA_OMD) & ~0x01, REG_GPIOA_OMD);	// GPIOA0 input mode
	//writel(readl(REG_GPIOB_OMD) | 0x5000, REG_GPIOB_OMD);	// GPIOB12,B14 output mode
	//writel(readl(REG_GPIOB_DOUT) | 0x5000, REG_GPIOB_DOUT);	// GPIOB12,B14 output high

	/* 
	 * Config multi-function
	 */	
	// No over currnet pin assigned on EV board
	// writel((readl(REG_PINFUN) & 0x00ffffff)/* | 0x11000000 */, REG_PINFUN);
	/* enable USB Host clock (UHC_EN, USB48_EN) */
        usbh11_clk = clk_get(&pdev->dev, "usb11h");

	clk_enable(usbh11_clk);

	/* reset USB host */
	writel( readl(REG_AHBIPRST) | UHC_RST, 
			REG_AHBIPRST);

	DBG_PRINTF("USBH IP  Reset\n");

	writel( readl(REG_AHBIPRST) & ~UHC_RST, 
			REG_AHBIPRST);

	if((w55fa92_upll_clock != 0) && ((w55fa92_upll_clock  % 48000) == 0))
	{
		u32clock = w55fa92_upll_clock / 48000;
		u32Div = 0x18 | ((u32clock - 1) << 8);
		printk("USBH1.1 Clock source is UPLL\n"); 
	}
	else if((w55fa92_apll_clock != 0) && ((w55fa92_apll_clock % 48000) == 0))
	{
		u32clock = w55fa92_apll_clock / 48000;
		u32Div = 0x10 | ((u32clock - 1) << 8);
		printk("USBH1.1 Clock source is APLL\n");
	} 
	
	writel( (readl(REG_CLKDIV2) & ~0xF1F) | u32Div, REG_CLKDIV2);	

//#if defined(CONFIG_W55FA92_USB_HOST_PORT1_DISABLE) || 
//	defined(CONFIG_W55FA92_USB_HOST_PORT2_DISABLE) 
	
	usbh_DisablePorts(disablePort1, disablePort2);	//Default two port enable	
//#endif



#ifdef CONFIG_W55FA92_USB_HOST_LIKE_PORT1
	printk("W55FA92_USB_HOST1.1_PORT1 (GPA10 & GPA11)\n");
	usbh_LikeModePort(1);				/* GPA10 and GPA11 from port 1*/
#endif
#ifdef CONFIG_W55FA92_USB_HOST_LIKE_PORT2
	#ifdef CONFIG_W55FA92_USB_HOST_PORT2_GROUP0
		printk("W55FA92_USB_HOST1.1_PORT2 (GPD3 & GPD4)\n");
		usbh_LikeModePort(2);				/* GPD3 and GPD4 from port 2*/
	#endif
	#ifdef CONFIG_W55FA92_USB_HOST_PORT2_GROUP1
		printk("W55FA92_USB_HOST1.1_PORT2 (GPA3 & GPA4)\n");
		usbh_LikeModePort(3);				/* GPD3 and GPD4 from port 2*/
	#endif
	#ifdef CONFIG_W55FA92_USB_HOST_PORT2_GROUP2
		printk("W55FA92_USB_HOST1.1_PORT2 (GPD14 & GPD15)\n");
		usbh_LikeModePort(4);				/* GPD3 and GPD4 from port 2*/
	#endif
#endif

#if defined(CONFIG_W55FA92_USB_HOST_LIKE_PORT1)
	disablePort1 = 0;
#else
	disablePort1 = 1;
#endif
	
#if defined(CONFIG_W55FA92_USB_HOST_LIKE_PORT2)
	disablePort2 = 0;
#else
	disablePort2 = 1;
#endif
	usbh_DisablePorts(disablePort1, disablePort2);	//1: Disable, 0:Enable ==>Disabke port 2
	
	// Set 0x08 for over current low active, 0 for high active
	// writel(0x08, W55FA92_VA_USBH_BASE + 0x204);

	hcd = usb_create_hcd(driver, &pdev->dev, "w55fa92-ohci-like");
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
static void usb_hcd_like_w55fa92_remove(struct usb_hcd *hcd,
		struct platform_device *dev)
{
	DBG_PRINTF("%s\n",__FUNCTION__);		
	usb_remove_hcd(hcd);

	//pr_debug("stopping W55FA92 USB Controller\n");

	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}


static int ohci_like_w55fa92_start (struct usb_hcd *hcd)
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


static const struct hc_driver ohci_like_w55fa92_hc_driver = {
	.description =		hcd_name,
	.product_desc = 	"Nuvoton W55FA92 OHCI Host Like Controller",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =        ohci_like_w55fa92_start,
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


static int ohci_hcd_like_w55fa92_drv_probe(struct platform_device *pdev)
{
	int ret;
	DBG_PRINTF("%s\n",__FUNCTION__);	
	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_like_w55fa92_probe(&ohci_like_w55fa92_hc_driver, pdev);
	return ret;
}

static int ohci_hcd_like_w55fa92_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_like_w55fa92_remove(hcd, pdev);
	return 0;
}

static struct platform_driver ohci_hcd_like_w55fa92_driver = {
	.probe		= ohci_hcd_like_w55fa92_drv_probe,
	.remove		= ohci_hcd_like_w55fa92_drv_remove,
#ifdef	CONFIG_PM

#endif
	.driver		= {
		.name	= "w55fa92-ohci-like",
		.owner	= THIS_MODULE,
	},
};

