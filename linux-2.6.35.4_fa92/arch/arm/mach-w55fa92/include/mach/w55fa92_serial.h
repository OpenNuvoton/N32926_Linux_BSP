/* linux/include/asm-arm/arch-w55fa92/w55fa92_serial.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */


#ifndef __W55FA92_SERIAL_H
#define __W55FA92_SERIAL_H

#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
static irqreturn_t w55fa92_serial_check_ms(int irq, void *id);
#endif
static irqreturn_t w55fa92_serial_tx_chars(int irq, void *id);

/* structures */
struct w55fa92_uart_info
{
	char			*name;
	unsigned int		type;
	unsigned int		fifosize;

	/* uart controls */
	int (*reset_port)(struct uart_port *, struct w55fa92_uartcfg *);
};

struct w55fa92_uart_port
{
	unsigned char			rx_claimed;
	unsigned char			tx_claimed;
	unsigned char			rts_cts;

	struct w55fa92_uart_info	*info;
	struct w55fa92_uart_clksrc	*clksrc;
	struct clk			*clk;
	struct clk			*baudclk;
	struct uart_port		port;
};


/* configuration defines */
#define dbg(x...) do {} while(0)
//#define dbg(x...) printk(x)

/* UART name and device definitions */
#define W55FA92_SERIAL_NAME	"ttyS"
#define W55FA92_SERIAL_DEVFS    "tts/"
#define W55FA92_SERIAL_MAJOR	4
#define W55FA92_SERIAL_MINOR	64

/* conversion functions */
#define w55fa92_dev_to_port(__dev) (struct uart_port *)dev_get_drvdata(__dev)
#define w55fa92_dev_to_cfg(__dev) (struct w55fa92_uartcfg *)((__dev)->platform_data)

/* we can support 2 uarts, but not always use them */

#define NR_PORTS (2)

/* port irq numbers */
#define TX_IRQ(port) ((port)->irq)
#define RX_IRQ(port) ((port)->irq)

/* register access controls */
#define portaddr(port, reg) ((port)->membase + (reg))
#define rd_regb(port, reg) (__raw_readb(portaddr(port, reg)))
#define rd_regl(port, reg) (__raw_readl(portaddr(port, reg)))
#define wr_regb(port, reg, val) \
  do { __raw_writeb(val, portaddr(port, reg)); } while(0)
#define wr_regl(port, reg, val) \
  do { __raw_writel(val, portaddr(port, reg)); } while(0)

/* macros to change one thing to another */
#define tx_enabled(port) ((port)->unused[0])
#define rx_enabled(port) ((port)->unused[1])
#define tx_disable(port) wr_regl(port, W55FA92_COM_IER, rd_regl(port, W55FA92_COM_IER) & ~UART_IER_THRI)
#define tx_enable(port)	wr_regl(port, W55FA92_COM_IER, rd_regl(port, W55FA92_COM_IER) | UART_IER_THRI | UART_IER_RTO | UART_IER_TOUT)
#define rx_disable(port) wr_regl(port, W55FA92_COM_IER, rd_regl(port, W55FA92_COM_IER) & ~UART_IER_RDI); wr_regl(port, W55FA92_COM_TOR, 0x00)
#define rx_enable(port)	wr_regl(port, W55FA92_COM_IER, rd_regl(port, W55FA92_COM_IER) | UART_IER_RDI | UART_IER_RTO | UART_IER_TOUT); wr_regl(port, W55FA92_COM_TOR, 0x20)

/* flag to ignore all characters comming in */
#define RXSTAT_DUMMY_READ (0x10000000)

#endif /* __W55FA92_SERIAL_H */
