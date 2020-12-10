/*
 * arch/arm/mach-w55fa92/include/mach/irqs.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation.
 *
 * Based on arch/arm/mach-s3c2410/include/mach/irqs.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/*
 * we keep the first set of CPU IRQs out of the range of
 * the ISA space, so that the PC104 has them to itself
 * and we don't end up having to do horrible things to the
 * standard ISA drivers....
 *
 */

#define W55FA92_IRQ(x) (x)

/* Main cpu interrupts */

#define IRQ_WDT        W55FA92_IRQ(1)
#define IRQ_GPIO0      W55FA92_IRQ(2)
#define IRQ_GPIO1      W55FA92_IRQ(3)
#define IRQ_GPIO2      W55FA92_IRQ(4)
#define IRQ_GPIO3      W55FA92_IRQ(5)
#define IRQ_IPSEC      W55FA92_IRQ(6)
#define IRQ_SPU        W55FA92_IRQ(7)
#define IRQ_I2S        W55FA92_IRQ(8)
#define IRQ_VPOST      W55FA92_IRQ(9)
#define IRQ_CAP        W55FA92_IRQ(10)
#define IRQ_MDCT       W55FA92_IRQ(11)
#define IRQ_BLT        W55FA92_IRQ(12)
#define IRQ_VPE        W55FA92_IRQ(13)
#define IRQ_HUART      W55FA92_IRQ(14)
#define IRQ_TIMER0     W55FA92_IRQ(15)
#define IRQ_TIMER1     W55FA92_IRQ(16)
#define IRQ_USBD       W55FA92_IRQ(17)
#define IRQ_SIC        W55FA92_IRQ(18)
#define IRQ_SDIO       W55FA92_IRQ(19)
#define IRQ_USBH       W55FA92_IRQ(20)
#define IRQ_EHCI       W55FA92_IRQ(21)
#define IRQ_OHCI       W55FA92_IRQ(22)
#define IRQ_EDMA       W55FA92_IRQ(23)
#define IRQ_EDMA0      W55FA92_IRQ(23)
#define IRQ_EDMA1      W55FA92_IRQ(24)
#define IRQ_SPI0       W55FA92_IRQ(25)
#define IRQ_SPI1       W55FA92_IRQ(26)
#define IRQ_ADC        W55FA92_IRQ(27)
#define IRQ_TOUCH      W55FA92_IRQ(28)
#define IRQ_RTC        W55FA92_IRQ(29)
#define IRQ_UART       W55FA92_IRQ(30)
#define IRQ_PWM        W55FA92_IRQ(31)
#define IRQ_JPEG       W55FA92_IRQ(32)
#define IRQ_VDE        W55FA92_IRQ(33)
#define IRQ_VEN        W55FA92_IRQ(34)
#define IRQ_SDIC       W55FA92_IRQ(35)
#define IRQ_EMCTX      W55FA92_IRQ(36)
#define IRQ_EMCRX      W55FA92_IRQ(37)
#define IRQ_I2C        W55FA92_IRQ(38)
#define IRQ_KPI        W55FA92_IRQ(39)
#define IRQ_RSC        W55FA92_IRQ(40)
#define IRQ_VTB        W55FA92_IRQ(41)
#define IRQ_ROT        W55FA92_IRQ(42)
#define IRQ_PWR        W55FA92_IRQ(43)
#define IRQ_LVD        W55FA92_IRQ(44)
#define IRQ_CAP2       W55FA92_IRQ(45)
#define IRQ_TIMER2     W55FA92_IRQ(46)
#define IRQ_TIMER3     W55FA92_IRQ(47)

#ifndef CONFIG_GPIO_W55FA92
#define NR_IRQS        48
#else
#define IRQ_GPIO_START 	W55FA92_IRQ(W55FA92_IRQ(0x100))
#define IRQ_GPIO_END 	W55FA92_IRQ(W55FA92_IRQ(0x100+0xE0))
#define NR_IRQS        (IRQ_GPIO_END + 1)
#endif

#endif /* __ASM_ARCH_IRQ_H */
