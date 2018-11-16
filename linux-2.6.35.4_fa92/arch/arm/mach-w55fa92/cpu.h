/* 
 * arch/arm/mach-w55fa92/cpu.h
 *
 * Based on linux/include/asm-arm/plat-s3c24xx/cpu.h by Ben Dooks
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * Header file for W55FA92 CPU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

//#define IODESC_ENT(y) { (unsigned long)W55FA92_VA_##y, __phys_to_pfn(W55FA92_PA_##y), W55FA92_SZ_##y, MT_DEVICE }
#define IODESC_ENT(y)                                  \
{                                                      \
       .virtual = (unsigned long)W55FA92_VA_##y,       \
       .pfn     = __phys_to_pfn(W55FA92_PA_##y),       \
       .length  = W55FA92_SZ_##y,                      \
       .type    = MT_DEVICE,                           \
}

#ifndef MHZ
#define MHZ (1000*1000)
#endif
#define print_mhz(m) ((m) / MHZ), ((m / 1000) % 1000)

/* forward declaration */
struct w55fa92_uartcfg;
struct map_desc;
struct sys_timer;
extern struct sys_timer w55fa92_timer;

#define W55FA92_DEVICE(devname,regname,devid,platdevname)		\
struct platform_device w55fa92_##devname = {				\
	.name		  = platdevname,				\
	.id		  = devid,					\
	.num_resources	  = ARRAY_SIZE(w55fa92_##regname##_resource),	\
	.resource	  = w55fa92_##regname##_resource,		\
}

#define W55FA92_8250PORT(name)					\
{								\
	.membase	= name##_BA,				\
	.mapbase	= name##_PA,				\
	.irq		= IRQ_##name,				\
	.uartclk	= CLOCK_TICK_RATE,			\
	.regshift	= 2,					\
	.iotype		= UPIO_MEM,				\
	.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,	\
}

/* CPU identifier register*/

#define W55FA92_CPUID	0x00FAD007

/* core initialisation functions */

extern void w55fa92_init_irq(void);
extern void w55fa92_init_clocks(void);
extern void w55fa92_init_uarts(struct w55fa92_uartcfg *cfg, int no);
extern void w55fa92_map_io(void);
extern void cpu_map_io(struct map_desc *mach_desc, int mach_size);
extern void w55fa92_board_init(void);
extern void w55fa92_dev_init(void);

//extern void init_uarts_w55fa92(struct w55fa92_uartcfg *cfg, int no);
//extern void w55fa92_init_io(struct map_desc *mach_desc, int size);
//extern int init_w55fa92(void);
//extern void init_clocks_w55fa92(int xtal);

/* for public w55fa92 */

extern struct platform_device *w55fa92_uart_devs[2];
extern struct platform_device w55fa92_serial_device;
extern struct platform_device w55fa92_device_usb;
extern struct platform_device w55fa92_device_usbh;
extern struct platform_device w55fa92_device_lcd;
extern struct platform_device w55fa92_device_ts;
extern struct platform_device w55fa92_device_i2c;
//extern struct platform_device w55fa92_device_wdt;
extern struct platform_device w55fa92_device_rtc;
extern struct platform_device w55fa92_device_fmi;
extern struct platform_device w55fa92_device_kpi;
