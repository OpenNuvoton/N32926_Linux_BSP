/* arch/arm/mach-w55fa92/devs.h
 *
 *Based on linux/include/asm-arm/plat-s3c24xx/devs.h by Ben Dooks
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * wanzongshun,zswan@nuvoton.com
 *
 * Header file for s3c2410 standard platform devices
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
//#include <linux/config.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

extern struct platform_device *w55fa92_uart_devs[2];
extern struct platform_device w55fa92_usbdevice;
extern struct platform_device w55fa92_usbhdevice;
extern struct platform_device w55fa92_device_lcd;
extern struct platform_device w55fa92_adcdevice;
extern struct platform_device w55fa92_device_i2c;
//extern struct platform_device w55fa92_device_wdt;
extern struct platform_device w55fa92_device_rtc;

#define W55FA92_RECS(name)				\
struct resource w55fa92_##name##_resource[]= {		\
							\
	[0] = {						\
		.start = W55FA92_PA_##name,		\
		.end   = W55FA92_PA_##name + 0x0ff,	\
		.flags = IORESOURCE_MEM,		\
	},						\
	[1] = {						\
		.start = IRQ_##name,			\
		.end   = IRQ_##name,			\
		.flags = IORESOURCE_IRQ,		\
	}						\
							\
}

#define W55FA92_DEVICE(devname,regname,devid,platdevname)		\
struct platform_device w55fa92_##devname = {				\
	.name		  = platdevname,				\
	.id		  = devid,					\
	.num_resources	  = ARRAY_SIZE(w55fa92_##regname##_resource),	\
	.resource	  = w55fa92_##regname##_resource,		\
}
