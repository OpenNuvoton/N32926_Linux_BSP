/*
 * linux/driver/usb/host/ehci-w55fa92.c
 *
 * Copyright (c) 2010 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */


#include <linux/platform_device.h>
#include <linux/signal.h>

#include <linux/clk.h>
#include <mach/irqs.h>
#include <mach/w55fa92_reg.h>

extern void w55fa92_enable_group_irq(int src);

extern unsigned int w55fa92_apll_clock;
extern unsigned int w55fa92_upll_clock;
static int usb_w55fa92_probe(const struct hc_driver *driver,
                            struct platform_device *pdev)
{
        struct usb_hcd *hcd;
        struct ehci_hcd *ehci;
	u32 u32clock ,u32Div = 0;
        int retval;
	int volatile loop;
	

	__raw_writel((readl(REG_GPEFUN1) & ~MF_GPE9) | 0x10, REG_GPEFUN1);
#ifdef CONFIG_W55FA92_USB_HOST_OVERCURRENT_ENABLE
	__raw_writel((readl(REG_GPDFUN0) & ~MF_GPD6) | 0x1000000, REG_GPDFUN0);
#else
	__raw_writel((readl(REG_GPDFUN0) & ~MF_GPD6), REG_GPDFUN0);
#endif
	if((w55fa92_upll_clock != 0) && ((w55fa92_upll_clock  % 48000) == 0))
	{
		u32clock = w55fa92_upll_clock / 48000;
		u32Div = 0x18 | ((u32clock - 1) << 8);
		printk("USBH2.0 Clock source is UPLL, divider is %d\n", u32clock); 
	}
	else if((w55fa92_apll_clock != 0) && ((w55fa92_apll_clock % 48000) == 0))
	{
		u32clock = w55fa92_apll_clock / 48000;
		u32Div = 0x10 | ((u32clock - 1) << 8);
		printk("USBH2.0 Clock source is APLL, divider is %d\n", u32clock); 
	}
	
	__raw_writel(u32Div, REG_CLKDIV6);	

	clk_enable(clk_get(&pdev->dev, NULL));
	clk_enable(clk_get(&pdev->dev, "ohci"));
	
	for (loop = 0; loop < 0x1000; loop++);

        __raw_writel(0x160, REG_USBPCR0);

#ifdef CONFIG_W55FA92_USB_HOST_OVERCURRENT_ENABLE
	#ifdef CONFIG_W55FA92_USB_HOST_OVERCURRENT_HIGH_ACTIVE
		writel((readl(REG_OpModEn) & ~0x08), REG_OpModEn);

		printk("Over-current is High Active - REG_OpModEn = 0x%08X\n",readl(REG_OpModEn));
	#endif

	#ifdef CONFIG_W55FA92_USB_HOST_OVERCURRENT_LOW_ACTIVE
		writel((readl(REG_OpModEn) | 0x08), REG_OpModEn);

		printk("Over-current is Low Active - REG_OpModEn = 0x%08X\n",readl(REG_OpModEn));
	#endif
#else
	writel((readl(REG_OpModEn) & ~0x08), REG_OpModEn);

	printk("Over-current is Disabled - REG_OpModEn = 0x%08X\n",readl(REG_OpModEn));
#endif
	writel((readl(REG_UPSCR0) & ~BIT13), REG_UPSCR0);
 
        if (pdev->resource[1].flags != IORESOURCE_IRQ) {
                pr_debug("resource[1] is not IORESOURCE_IRQ");
                retval = -ENOMEM;
        }

        hcd = usb_create_hcd(driver, &pdev->dev, "w55fa92-ehci");
        if (!hcd) {
                retval = -ENOMEM;
                goto err1;
        }

        hcd->rsrc_start = pdev->resource[0].start;
        hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

        if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
                pr_debug("ehci probe request_mem_region failed");
                retval = -EBUSY;
                goto err2;
        }

        hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
        if (hcd->regs == NULL) {
                pr_debug("ehci error mapping memory\n");
                retval = -EFAULT;
                goto err3;
        }

        ehci = hcd_to_ehci(hcd);
        ehci->caps = hcd->regs;
        ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));

        /* cache this readonly data; minimize chip reads */
        ehci->hcs_params = readl(&ehci->caps->hcs_params);
        ehci->sbrn = 0x20;

        retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);
	
        if (retval != 0)
                goto err4;
	
        /* enable EHCI */
        __raw_writel(1, &ehci->regs->configured_flag);

	if((readl(REG_UPSCR0) & 0x1803) == 0x1800)
	{
		__raw_writel(0x3000, REG_UPSCR0);	
		for (loop = 0; loop < 0x1000; loop++);
	       	__raw_writel(0x1000, REG_UPSCR0);	
	}

        return retval;

err4:
        iounmap(hcd->regs);
err3:
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err2:
        usb_put_hcd(hcd);
err1:

        return retval;
}

void usb_w55fa92_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
        usb_remove_hcd(hcd);
        iounmap(hcd->regs);
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
        usb_put_hcd(hcd);
}


static const struct hc_driver ehci_w55fa92_hc_driver = {
        .description = hcd_name,
        .product_desc = "Nuvoton w55fa92 EHCI Host Controller",
        .hcd_priv_size = sizeof(struct ehci_hcd),

        /*
         * generic hardware linkage
         */
        .irq = ehci_irq,
        .flags = HCD_USB2|HCD_MEMORY,

        /*
         * basic lifecycle operations
         */
        .reset = ehci_init,
        .start = ehci_run,

        .stop = ehci_stop,

        /*
         * managing i/o requests and associated device resources
         */
        .urb_enqueue = ehci_urb_enqueue,
        .urb_dequeue = ehci_urb_dequeue,
        .endpoint_disable = ehci_endpoint_disable,

        /*
         * scheduling support
         */
        .get_frame_number = ehci_get_frame,

        /*
         * root hub support
         */
        .hub_status_data = ehci_hub_status_data,
        .hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
        .bus_suspend = ehci_bus_suspend,
        .bus_resume = ehci_bus_resume,
#endif
};

static int ehci_w55fa92_probe(struct platform_device *pdev)
{
        if (usb_disabled())
                return -ENODEV;

        return usb_w55fa92_probe(&ehci_w55fa92_hc_driver, pdev);
}

static int ehci_w55fa92_remove(struct platform_device *pdev)
{
        struct usb_hcd *hcd = platform_get_drvdata(pdev);

        usb_w55fa92_remove(hcd, pdev);

        return 0;
}

static struct platform_driver ehci_hcd_w55fa92_driver = {

        .probe = ehci_w55fa92_probe,
        .remove = ehci_w55fa92_remove,
        .driver = {
                .name = "w55fa92-ehci",
                .owner= THIS_MODULE,
        },
};

static int __init ehci_w55fa92_init(void)
{
	return platform_driver_register(&ehci_hcd_w55fa92_driver);
}

static void __exit ehci_w55fa92_cleanup(void)
{
	platform_driver_unregister(&ehci_hcd_w55fa92_driver);

}

module_init(ehci_w55fa92_init);
module_exit(ehci_w55fa92_cleanup);
