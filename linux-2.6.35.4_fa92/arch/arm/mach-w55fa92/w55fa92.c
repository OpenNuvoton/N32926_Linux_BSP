/* 
 * linux/arch/arm/mach-w55fa92/w55fa92.c
 *
 * Based on linux/arch/arm/plat-s3c24xx/s3c244x.c by Ben Dooks
 *
 * Copyright (c) 2013 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
*/

#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <mach/w55fa92_reg.h>
#include <mach/hardware.h>
#include <mach/serial.h>
#include "cpu.h"
#include "dev.h"
#include "clock.h"

/* Initial IO mappings */
static struct map_desc w55fa92_iodesc[] __initdata = {
	IODESC_ENT(IRQ),
	IODESC_ENT(UART),

	IODESC_ENT(TIMER),
	IODESC_ENT(TIMER2),
	IODESC_ENT(USBD),
	IODESC_ENT(GCR),

	IODESC_ENT(SDIC),
	IODESC_ENT(SIC),
	IODESC_ENT(SDIO),

	IODESC_ENT(EDMA),

	IODESC_ENT(VPOST),
	IODESC_ENT(BLT),
	IODESC_ENT(GVE),

	IODESC_ENT(VIDEOIN),
	IODESC_ENT(I2SM),

	IODESC_ENT(SPU),

	IODESC_ENT(JPEG),
	IODESC_ENT(USBH),

	IODESC_ENT(PWM),
	IODESC_ENT(GPIO),
	IODESC_ENT(ADC),
	IODESC_ENT(TP),

	IODESC_ENT(SPI0),

	IODESC_ENT(RTC),
	IODESC_ENT(I2C),
	IODESC_ENT(KPI),
	IODESC_ENT(EMC),

	IODESC_ENT(VDE),
	IODESC_ENT(ENC),
	IODESC_ENT(ROT),
	IODESC_ENT(CRC),

	IODESC_ENT(MDCT),
	IODESC_ENT(UOHCI20),

	IODESC_ENT(UEHCI20),
	IODESC_ENT(AES),
};

/* Initial clock declarations. */
static DEFINE_AHBCLK(cpu, 0);
static DEFINE_AHBCLK(apbclk, 1);
static DEFINE_AHBCLK(hclk, 2);
static DEFINE_AHBCLK(sram, 3);
static DEFINE_AHBCLK(dram, 4);
static DEFINE_AHBCLK(blt, 5);
static DEFINE_AHBCLK(vpe, 6);
static DEFINE_AHBCLK(jpg, 7);
static DEFINE_AHBCLK(hclk1, 8);
static DEFINE_AHBCLK(ipsec, 9);
static DEFINE_AHBCLK(edma0, 10);
static DEFINE_AHBCLK(edma1, 11);
static DEFINE_AHBCLK(edma2, 12);
static DEFINE_AHBCLK(edma3, 13);
static DEFINE_AHBCLK(edma4, 14);
static DEFINE_AHBCLK(vde, 15);
static DEFINE_AHBCLK(hclk3, 16);
static DEFINE_AHBCLK(usb11h, 17);
static DEFINE_AHBCLK(u20phy, 18);
static DEFINE_AHBCLK(gve, 19);
static DEFINE_AHBCLK(sic, 21);
static DEFINE_AHBCLK(nand, 22);
static DEFINE_AHBCLK(sd, 23);
static DEFINE_AHBCLK(hclk4, 24);
static DEFINE_AHBCLK(spu, 25);
static DEFINE_AHBCLK(i2s, 26);
static DEFINE_AHBCLK(vpost, 27);
static DEFINE_AHBCLK(cap0, 28);
static DEFINE_AHBCLK(sen0, 29);
static DEFINE_AHBCLK(ado, 30);
static DEFINE_AHBCLK(sdio, 31);

static DEFINE_AHBCLK2(edma5, 0);
static DEFINE_AHBCLK2(edma6, 1);
static DEFINE_AHBCLK2(cap1, 2);
static DEFINE_AHBCLK2(sen1, 3);
static DEFINE_AHBCLK2(ohci, 4);
static DEFINE_AHBCLK2(h20phy, 5);
static DEFINE_AHBCLK2(crc, 6);
static DEFINE_AHBCLK2(emc, 7);
static DEFINE_AHBCLK2(aac, 8);
static DEFINE_AHBCLK2(vdec, 9);
static DEFINE_AHBCLK2(venc, 10);
static DEFINE_AHBCLK2(edma20, 11);
static DEFINE_AHBCLK2(edma21, 12);
static DEFINE_AHBCLK2(edma22, 13);
static DEFINE_AHBCLK2(edma23, 14);
static DEFINE_AHBCLK2(edma24, 15);
static DEFINE_AHBCLK2(edma25, 16);
static DEFINE_AHBCLK2(edma26, 17);
static DEFINE_AHBCLK2(rot, 18);
static DEFINE_AHBCLK2(crc2, 19);

static DEFINE_APBCLK(adc, 0);
static DEFINE_APBCLK(i2c, 1);
static DEFINE_APBCLK(rtc, 2);
static DEFINE_APBCLK(uart0, 3);
static DEFINE_APBCLK(uart1, 4);
static DEFINE_APBCLK(pwm, 5);
static DEFINE_APBCLK(spims0, 6);
static DEFINE_APBCLK(spims1, 7);
static DEFINE_APBCLK(timer0, 8);
static DEFINE_APBCLK(timer1, 9);
static DEFINE_APBCLK(touch, 10);
static DEFINE_APBCLK(cnvenc, 11);
static DEFINE_APBCLK(rsc, 12);
static DEFINE_APBCLK(wdt, 15);
static DEFINE_APBCLK(timer2, 16);
static DEFINE_APBCLK(timer3, 17);
static DEFINE_APBCLK(tic, 24);
static DEFINE_APBCLK(kpi, 25);

static struct clk_lookup w55fa92_clkregs[] = {
	DEF_CLKLOOK(&clk_cpu,		NULL,			"cpu"), 
	DEF_CLKLOOK(&clk_apbclk,	NULL,			"apbclk"), 
	DEF_CLKLOOK(&clk_hclk,		NULL,			"hclk"), 
	DEF_CLKLOOK(&clk_sram,		NULL,			"sram"), 
	DEF_CLKLOOK(&clk_dram,		NULL,			"dram"), 
	DEF_CLKLOOK(&clk_hclk1,		NULL,			"hclk1"), 
	DEF_CLKLOOK(&clk_hclk3,		NULL,			"hclk3"), 
	DEF_CLKLOOK(&clk_hclk4,		NULL,			"hclk4"), 

	DEF_CLKLOOK(&clk_blt,		NULL,			"blt"), 
	DEF_CLKLOOK(&clk_vpe,		NULL,			"vpe"), 
	DEF_CLKLOOK(&clk_jpg,		NULL,			"jpg"),
	DEF_CLKLOOK(&clk_vde,		NULL,			"vde"), 
	DEF_CLKLOOK(&clk_gve,		NULL,			"gve"), 
	DEF_CLKLOOK(&clk_vdec,		NULL,			"vdec"), 
	DEF_CLKLOOK(&clk_venc,		NULL,			"venc"), 

	DEF_CLKLOOK(&clk_ipsec,		NULL,			"ipsec"), 
	DEF_CLKLOOK(&clk_crc,		NULL,			"crc"), 
	DEF_CLKLOOK(&clk_crc2,		NULL,			"crc2"), 
	DEF_CLKLOOK(&clk_cnvenc,	NULL,			"cnvenc"), 
	DEF_CLKLOOK(&clk_rsc,		NULL,			"rsc"), 

	DEF_CLKLOOK(&clk_usb11h,	"w55fa92-ohci-like",		"usb11h"),
	DEF_CLKLOOK(&clk_u20phy,	"w55fa92-usbgadget",	NULL),
	DEF_CLKLOOK(&clk_ohci,		"w55fa92-ehci",		"ohci"),
	DEF_CLKLOOK(&clk_h20phy,	"w55fa92-ehci",		NULL),

	DEF_CLKLOOK(&clk_sic,		NULL,			"sic"),
	DEF_CLKLOOK(&clk_nand,		NULL,			"nand"),
	DEF_CLKLOOK(&clk_sd,		NULL,			"sd"),
	DEF_CLKLOOK(&clk_sdio,		NULL,			"sdio"),
	DEF_CLKLOOK(&clk_emc,		"w55fa92-emc",		NULL), 

	DEF_CLKLOOK(&clk_spu,		NULL,			"spu"),
	DEF_CLKLOOK(&clk_i2s,		NULL,			"i2s"),
	DEF_CLKLOOK(&clk_ado,		NULL,			"ado"),
	DEF_CLKLOOK(&clk_aac,		NULL,			"aac"),
	DEF_CLKLOOK(&clk_adc,		NULL,			"adc"),

	DEF_CLKLOOK(&clk_vpost,		"w55fa92-lcd",		NULL),
	DEF_CLKLOOK(&clk_rot,		NULL,			"rot"), 
	DEF_CLKLOOK(&clk_cap0,		NULL,			"cap0"),
	DEF_CLKLOOK(&clk_sen0,		NULL,			"sen0"),
	DEF_CLKLOOK(&clk_cap1,		NULL,			"cap1"),
	DEF_CLKLOOK(&clk_sen1,		NULL,			"sen1"),
	DEF_CLKLOOK(&clk_pwm,		NULL,			"pwm"),

	DEF_CLKLOOK(&clk_i2c,		"w55fa92-i2c",		NULL),
	DEF_CLKLOOK(&clk_rtc,		NULL,			"rtc"),
	DEF_CLKLOOK(&clk_uart0,		"w55fa92-uart0",	NULL),
	DEF_CLKLOOK(&clk_uart1,		"w55fa92-uart1",	NULL),
	DEF_CLKLOOK(&clk_spims0,	"w55fa92-spi.0",		"ms0"),
	DEF_CLKLOOK(&clk_spims1,	"w55fa92-spi.1",		"ms1"),
	DEF_CLKLOOK(&clk_timer0,	NULL,			"timer0"),
	DEF_CLKLOOK(&clk_timer1,	NULL,			"timer1"),
	DEF_CLKLOOK(&clk_timer2,	NULL,			"timer2"),
	DEF_CLKLOOK(&clk_timer3,	NULL,			"timer3"),
	DEF_CLKLOOK(&clk_touch,		NULL,			"touch"), 
	DEF_CLKLOOK(&clk_wdt,		"w55fa92-wdt",		NULL),
	DEF_CLKLOOK(&clk_tic,		NULL,			"tic"),
	DEF_CLKLOOK(&clk_kpi,		"w55fa92-kpi",		NULL),

	DEF_CLKLOOK(&clk_edma0,		NULL,			"edma0"),
	DEF_CLKLOOK(&clk_edma1,		NULL,			"edma1"),
	DEF_CLKLOOK(&clk_edma2,		NULL,			"edma2"),
	DEF_CLKLOOK(&clk_edma3,		NULL,			"edma3"),
	DEF_CLKLOOK(&clk_edma4,		NULL,			"edma4"),
	DEF_CLKLOOK(&clk_edma5,		NULL,			"edma5"),
	DEF_CLKLOOK(&clk_edma6,		NULL,			"edma6"),
	DEF_CLKLOOK(&clk_edma20,	NULL,			"edma20"),
	DEF_CLKLOOK(&clk_edma21,	NULL,			"edma21"),
	DEF_CLKLOOK(&clk_edma22,	NULL,			"edma22"),
	DEF_CLKLOOK(&clk_edma23,	NULL,			"edma23"),
	DEF_CLKLOOK(&clk_edma24,	NULL,			"edma24"),
	DEF_CLKLOOK(&clk_edma25,	NULL,			"edma25"),
	DEF_CLKLOOK(&clk_edma26,	NULL,			"edma26"),
};

struct resource w55fa92_HUART_resource[]= {
	[0] = {
		.start = W55FA92_PA_UART,
		.end   = W55FA92_PA_UART + 0x0ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_HUART,
		.end   = IRQ_HUART,
		.flags = IORESOURCE_IRQ,
	}
};

struct resource w55fa92_UART_resource[]= {
	[0] = {
		.start = W55FA92_PA_UART + 0x100,
		.end   = W55FA92_PA_UART + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UART,
		.end   = IRQ_UART,
		.flags = IORESOURCE_IRQ,
	}
};

/*Init the plat dev*/
W55FA92_DEVICE(uart0,HUART,0,"w55fa92-uart");
W55FA92_DEVICE(uart1,UART,1,"w55fa92-uart");

static struct platform_device *uart_devices[] __initdata = {
	&w55fa92_uart0,
	&w55fa92_uart1
};

static int w55fa92_uart_count = 0;

/* uart registration process */

void __init w55fa92_init_uarts(struct w55fa92_uartcfg *cfg, int no)
{
	struct platform_device *platdev;
	int uart;

	// enable UART clock
	//__raw_writel(__raw_readl(REG_CLKMAN) | 1 << 11, REG_CLKMAN);

	uart_devices[0]->dev.init_name = "w55fa92-uart0";
	uart_devices[1]->dev.init_name = "w55fa92-uart1";
	for (uart = 0; uart < no; uart++, cfg++) {
		platdev = uart_devices[cfg->hwport];

		w55fa92_uart_devs[uart] = platdev;
		platdev->dev.platform_data = cfg;
	}

	w55fa92_uart_count = uart;
}

/* w55fa92_map_io
 *
 * register the standard cpu IO areas, and any passed in from the
 * machine specific initialisation.
*/

void __init w55fa92_map_io()
{
	/* register our io-tables */

	cpu_map_io(w55fa92_iodesc, ARRAY_SIZE(w55fa92_iodesc));
}

static DEFINE_SPINLOCK(nvt_general_lock);
static unsigned long nvt_general_flags;

void nvt_lock(void)
{
	spin_lock_irqsave(&nvt_general_lock, nvt_general_flags);
}
EXPORT_SYMBOL(nvt_lock);

void nvt_unlock(void)
{
	spin_unlock_irqrestore(&nvt_general_lock, nvt_general_flags);
}
EXPORT_SYMBOL(nvt_unlock);

unsigned int w55fa92_external_clock;
unsigned int w55fa92_apll_clock;
unsigned int w55fa92_upll_clock;
unsigned int w55fa92_mpll_clock;
unsigned int w55fa92_system_clock;
unsigned int w55fa92_cpu_clock;
unsigned int w55fa92_ahb_clock;
unsigned int w55fa92_apb_clock;
EXPORT_SYMBOL(w55fa92_external_clock);
EXPORT_SYMBOL(w55fa92_apll_clock);
EXPORT_SYMBOL(w55fa92_upll_clock);
EXPORT_SYMBOL(w55fa92_mpll_clock);
EXPORT_SYMBOL(w55fa92_system_clock);
EXPORT_SYMBOL(w55fa92_cpu_clock);
EXPORT_SYMBOL(w55fa92_ahb_clock);
EXPORT_SYMBOL(w55fa92_apb_clock);

// pll = 0 for APLL, 1 for UPLL
unsigned int w55fa92_get_pllcon(char pll, unsigned int targetKHz)
{
	unsigned char NOMap[4] = {1, 2, 4, 8};
	int NR, NF, NO, out_dv, in_dv, fb_dv;
	unsigned int FinKHz, FoutKHz, pllcon;

	FinKHz = w55fa92_external_clock / 1000;
	// to get little jitter on PLL output, large value is better
	for (out_dv = 3; out_dv >= 0; out_dv--) {
		NO = NOMap[out_dv];
		for (in_dv = 2; in_dv < 16; in_dv++) {
			// NR >= 2, small value is better
			NR = in_dv;
			// 1MHz < FIN/NR < 50MHz
			if (((FinKHz / NR) <= 1000) || ((FinKHz / NR) >= 50000))
				continue;
			for (fb_dv = 127; fb_dv >= 2; fb_dv--) {
				// NF >= 4, large value is better
				NF = fb_dv * 2;
				//printk("NO=%d, NR=%d, NF=%d\n", NO, NR, NF);
				// 500MHz <= FOUT*NO <= 1500MHz
				if (((FinKHz * NF / NR) < 500000) || ((FinKHz * NF / NR) > 1500000))
					continue;
				FoutKHz = FinKHz * NF / NR / NO;
				//printk("FoutKHz = %d\n", FoutKHz);
				if (targetKHz == FoutKHz) {
					pllcon = (out_dv << 11) | (in_dv << 7) | fb_dv;
					return pllcon;
				}
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL(w55fa92_get_pllcon);

int w55fa92_set_apll_clock(unsigned int clock)
{
	int ret = 0;

#if 1
	if (w55fa92_external_clock == 12000000) {
		if (clock == 344064) {
			// for SPU/I2S 32/48/64/96KHz * 256, TD = 0.0186%
			__raw_writel(0x09D6, REG_APLLCON);
		}
		else if (clock == 316108) {
			// for SPU/I2S 44.1/88.2KHz * 256, TD = 0.0344%
			__raw_writel(0x09CF, REG_APLLCON);
		}
		else if (clock == 258048) {
			// for I2S 48/96KHz * 384, TD = 0.0186%
			__raw_writel(0x1156, REG_APLLCON);
		}
		else if (clock == 254016) {
			// for SPU/I2S 22.05KHz * 256, I2S 22.05/44.1KHz * 384, TD = 0.0063%
			__raw_writel(0x11FF, REG_APLLCON);
		}
		else if (clock == 237081) {
			// for I2S 88.2KHz * 384, TD = 0.0344%
			__raw_writel(0x114F, REG_APLLCON);
		}
		else if (clock == 172032) {
			// for I2S 32/64KHz * 384, TD = 0.0186%
			__raw_writel(0x11D6, REG_APLLCON);
		}
		else if (clock == 135000) {
			// for TV 27MHz, TD = 0%
			__raw_writel(0x195A, REG_APLLCON);
		}
		else {
			printk("%s does not support %dKHz APLL clock!\n", __FUNCTION__, clock);
			ret = -1;
		}
	}
	else {
		printk("%s does not support %d.%dMHz xtal clock!\n", __FUNCTION__, print_mhz(w55fa92_external_clock));
		ret = -1;
	}
#else
	unsigned int pllcon;

	if (clock != w55fa92_apll_clock) {
		pllcon = w55fa92_get_pllcon(0, clock);
		if (pllcon == 0) {
			printk("Cannot calculate APLL %d KHz!!\n", clock);
			ret = -1;
		} else
			__raw_writel(pllcon, REG_APLLCON);
	}

#endif

	if (ret == 0)
		w55fa92_apll_clock = clock;

	return ret;
}
EXPORT_SYMBOL(w55fa92_set_apll_clock);

static inline unsigned int
w55fa92_get_pll(char pll, unsigned int xtal)
{
	unsigned char NOMap[4] = {1, 2, 4, 8};
	unsigned int pllcon;
	unsigned int NR, NF, NO;
	uint64_t fvco = 0;

	if (pll == 0)
		pllcon = __raw_readl(REG_APLLCON);
	else if (pll == 1)
		pllcon = __raw_readl(REG_UPLLCON);
	else if (pll == 2)
		pllcon = __raw_readl(REG_MPLLCON);
	else
		return 0;

	NF = (pllcon & FB_DV) << 1;
	NR = (pllcon & IN_DV) >> 7;
	NO = NOMap[((pllcon & OUT_DV) >> 11)];

	fvco = (uint64_t)xtal * NF;
	do_div(fvco, (NR * NO));

	return (unsigned int)fvco;
}

/* w55fa92_init_clocks
 *
 * Initialise the clock subsystem and associated information from the
 * given master crystal value.
 *
 */
void __init w55fa92_init_clocks(void)
{
	int xtal;

	xtal = 12000000;
	w55fa92_external_clock = xtal;
	w55fa92_apll_clock = w55fa92_get_pll(0, xtal) / 1000;
	w55fa92_upll_clock = w55fa92_get_pll(1, xtal) / 1000;
	w55fa92_mpll_clock = w55fa92_get_pll(2, xtal) / 1000;

	if ((__raw_readl(REG_CLKDIV0) & SYSTEM_S) == 0x18)
		w55fa92_system_clock = w55fa92_upll_clock / ((__raw_readl(REG_CLKDIV0) & SYSTEM_N0) + 1); 
	else if ((__raw_readl(REG_CLKDIV0) & SYSTEM_S) == 0x10)
		w55fa92_system_clock = w55fa92_apll_clock / ((__raw_readl(REG_CLKDIV0) & SYSTEM_N0) + 1); 
	else if ((__raw_readl(REG_CLKDIV0) & SYSTEM_S) == 0x08)
		w55fa92_system_clock = w55fa92_mpll_clock / ((__raw_readl(REG_CLKDIV0) & SYSTEM_N0) + 1); 
	else if ((__raw_readl(REG_CLKDIV0) & SYSTEM_S) == 0x00)
		w55fa92_system_clock = w55fa92_external_clock; 
	else
		printk("w55fa92_system_clock is unsupported!\n");

	w55fa92_cpu_clock = w55fa92_system_clock/((__raw_readl(REG_CLKDIV4)&CPU_N) + 1);
	w55fa92_ahb_clock = ((__raw_readl(REG_CLKDIV4)&CPU_N) == 0 ) ? w55fa92_cpu_clock/2 : w55fa92_cpu_clock;
	w55fa92_apb_clock = w55fa92_ahb_clock/(((__raw_readl(REG_CLKDIV4)&APB_N)>>8) + 1);
#if 1
	printk("w55fa92_external_clock	= %d.%d MHz\n", print_mhz(w55fa92_external_clock));
	printk("w55fa92_apll_clock	= %d KHz\n", w55fa92_apll_clock);
	printk("w55fa92_upll_clock	= %d KHz\n", w55fa92_upll_clock);
	printk("w55fa92_mpll_clock	= %d KHz\n", w55fa92_mpll_clock);
	printk("w55fa92_system_clock	= %d KHz\n", w55fa92_system_clock);
	printk("w55fa92_cpu_clock	= %d KHz\n", w55fa92_cpu_clock);
	printk("w55fa92_ahb_clock	= %d KHz\n", w55fa92_ahb_clock);
	printk("w55fa92_apb_clock	= %d KHz\n", w55fa92_apb_clock);
#endif
	clkdev_add_table(w55fa92_clkregs, ARRAY_SIZE(w55fa92_clkregs));
}

void __init w55fa92_board_init(void)
{
	platform_add_devices(w55fa92_uart_devs, w55fa92_uart_count);
	w55fa92_dev_init();
}
