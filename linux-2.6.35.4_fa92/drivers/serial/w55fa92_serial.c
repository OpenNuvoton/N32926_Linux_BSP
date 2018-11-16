/*
 * linux/drivers/serial/w55fa92_serial.c
 *
 * based on linux/drivers/serial/samsuing.c by Ben Dooks
 *
 * UART driver for Nuvoton W55FA92
 *
 * Copyright (c) 2013 Nuvoton technology corporation
 * All rights reserved.
 * www.nuvoton.com wanzongshun,zswan@nuvoton.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * LZXU modify this file for nuvoton serial driver.
*/
 

//#include <linux/config.h>
#if defined(CONFIG_SERIAL_W55FA92_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/serial.h>
#include <mach/w55fa92_serial.h>
#include <mach/w55fa92_reg.h>

extern unsigned int w55fa92_external_clock;
extern unsigned int w55fa92_upll_clock;

#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
#define DEF_HUART_DELAY		HZ/50
static struct timer_list	huart_timer;
static unsigned int			ns_delay;
#endif

static inline struct w55fa92_uart_port *to_ourport(struct uart_port *port)
{
	return container_of(port, struct w55fa92_uart_port, port);
}

/* translate a port to the device name */

static inline const char *w55fa92_serial_portname(struct uart_port *port)
{
	return to_platform_device(port->dev)->name;
}

static int w55fa92_serial_txempty_nofifo(struct uart_port *port)
{
	return (rd_regl(port, W55FA92_COM_FSR) & UART_FSR_TFE);
}

static void w55fa92_serial_rx_enable(struct uart_port *port)
{
	unsigned long flags;
	unsigned int fcr;


	spin_lock_irqsave(&port->lock, flags);

	fcr = rd_regl(port, W55FA92_COM_FCR);
	fcr |= UART_FCR_RFR | UARTx_FCR_FIFO_LEVEL14;
	wr_regl(port, W55FA92_COM_FCR, fcr);
	
	rx_enable(port);
	rx_enabled(port) = 1;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void w55fa92_serial_rx_disable(struct uart_port *port)
{
	unsigned long flags;


	spin_lock_irqsave(&port->lock, flags);

	rx_disable(port);

	rx_enabled(port) = 0;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void w55fa92_serial_stop_tx(struct uart_port *port)
{

	if (tx_enabled(port)) {
		tx_disable(port);
		tx_enabled(port) = 0;
		if (port->flags & UPF_CONS_FLOW)
			w55fa92_serial_rx_enable(port);
	}
}

static void w55fa92_serial_start_tx(struct uart_port *port)
{

	if (!tx_enabled(port)) {
		if (port->flags & UPF_CONS_FLOW)
			w55fa92_serial_rx_disable(port);

		tx_enable(port);
		tx_enabled(port) = 1;
	}
}


static void w55fa92_serial_stop_rx(struct uart_port *port)
{
	if (rx_enabled(port)) {

		rx_disable(port);
		rx_enabled(port) = 0;
	}
}

static void w55fa92_serial_enable_ms(struct uart_port *port)
{
}

static inline struct w55fa92_uart_info *w55fa92_port_to_info(struct uart_port *port)
{
	return to_ourport(port)->info;
}

static inline struct w55fa92_uartcfg *w55fa92_port_to_cfg(struct uart_port *port)
{
	if (port->dev == NULL)
		return NULL;

	return (struct w55fa92_uartcfg *)port->dev->platform_data;
}

static int max_count;
/* ? - where has parity gone?? */
static inline int  CAN_GETC(struct uart_port *port, unsigned int isr)
{
	if (isr & UART_ISR_RDA_INT)
		return max_count--;
	else if (isr & UART_ISR_Tout_INT)
		return !(rd_regl(port, W55FA92_COM_FSR) & UART_FSR_RFE);
	return -1;
}		

static irqreturn_t
w55fa92_serial_rx_chars(int irq, void *dev_id)
{
	struct w55fa92_uart_port *ourport = dev_id;
	struct uart_port *port = &ourport->port;
	struct tty_struct *tty = port->state->port.tty; //port->info->tty;
	unsigned int isr, ch, flag, fsrstat;
	struct w55fa92_uart_info *info = ourport->info;
	irqreturn_t ret;

	max_count = info->fifosize;

	isr = rd_regl(port, W55FA92_COM_ISR);

#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
	if (isr & UART_ISR_Modem_INT) {
		ret = w55fa92_serial_check_ms(irq, dev_id);
		//return ret;
	}
#endif
	if (isr & UART_ISR_THRE_INT) {
		ret = w55fa92_serial_tx_chars(irq, dev_id);
		//return ret;
	}
	if (isr & UART_ISR_RDA_INT)
		max_count = info->fifosize;
	else if (isr & UART_ISR_Tout_INT)
		max_count = 1;
	else
		return IRQ_HANDLED;
	while ((max_count = CAN_GETC(port, isr)) > 0) {
		fsrstat = rd_regl(port, W55FA92_COM_FSR);

		if (fsrstat & UART_FSR_RFE) //1 = Rx FIFO is empty
			break;

		ch = rd_regb(port, W55FA92_COM_RX);

		if (port->flags & UPF_CONS_FLOW) {
			int txe = w55fa92_serial_txempty_nofifo(port);

			if (rx_enabled(port)) {
				if (!txe) {
					rx_enabled(port) = 0;
					continue;
				}
			} else {
				if (txe) {
					wr_regl(port, W55FA92_COM_FCR, rd_regl(port, W55FA92_COM_FCR) | UART_FCR_RFR);
					rx_enabled(port) = 1;
					goto out;
				}
				continue;
			}
		}

		/* insert the character into the buffer */

		flag = TTY_NORMAL;
		port->icount.rx++;

		if (unlikely(fsrstat & W55FA92_FSRSTAT_ANY)) {
			dbg("rxerr: port ch=0x%02x, rxs=0x%08x\n",
		    	ch, fsrstat);

			/* check for break */
			if (fsrstat & UART_FSR_BI) {
				dbg("break!\n");
				port->icount.brk++;
				if (uart_handle_break(port))
			    	goto ignore_char;
			}

			if (fsrstat & UART_FSR_FE)
				port->icount.frame++;
			if (fsrstat & (UART_FSR_ROE))
				port->icount.overrun++;

			fsrstat &= port->read_status_mask;

			if (fsrstat & UART_FSR_BI)
				flag = TTY_BREAK;
			else if (fsrstat & UART_FSR_PE)
				flag = TTY_PARITY;
			else if (fsrstat & ( UART_FSR_FE | UART_FSR_ROE))
				flag = TTY_FRAME;
		}

//		if (uart_handle_sysrq_char(port, ch, regs))
		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		uart_insert_char(port, fsrstat, UART_FSR_ROE, ch, flag);

	ignore_char:
		continue;
	}
	
	tty_flip_buffer_push(tty);

 out:
	return IRQ_HANDLED;
}

#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
static irqreturn_t w55fa92_serial_check_ms(int irq, void *id)
{
	struct w55fa92_uart_port *ourport = id;
	struct uart_port *port = &ourport->port;
	unsigned int msr;

	msr = rd_regl(port, W55FA92_COM_MSR);
	if (msr & UART_MSR_DCTS) {
		wr_regl(port, W55FA92_COM_MSR, msr);
		uart_handle_cts_change(port, !(msr & UART_MSR_CTS_st));
		if (msr & UART_MSR_CTS_st) {
			mod_timer(&huart_timer, jiffies + DEF_HUART_DELAY);
		}
	}

	return IRQ_HANDLED;
}

static void read_cts(unsigned long data)
{
	struct w55fa92_uart_port *ourport = (struct w55fa92_uart_port *)(data);
	struct uart_port *port = &ourport->port;
	unsigned int msr;
	unsigned long volatile flags;

	local_irq_save(flags);
	msr = rd_regl(port, W55FA92_COM_MSR);
	if (msr & UART_MSR_CTS_st) {
		mod_timer(&huart_timer, jiffies + DEF_HUART_DELAY);
	} else {
		del_timer(&huart_timer);
		uart_handle_cts_change(port, !(msr & UART_MSR_CTS_st));
	}
	local_irq_restore(flags);
}
#endif

static irqreturn_t w55fa92_serial_tx_chars(int irq, void *id)
{
	struct w55fa92_uart_port *ourport = id;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit; //&port->info->xmit;
	struct w55fa92_uart_info *info = ourport->info;
	int count = info->fifosize;
	if (port->x_char) {
		wr_regb(port, W55FA92_COM_TX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		goto out;
	}

	/* if there isnt anything more to transmit, or the uart is now
	 * stopped, disable the uart and exit
	*/

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		w55fa92_serial_stop_tx(port);
		goto out;
	}

	/* try and drain the buffer... */

	while (!uart_circ_empty(xmit) && count-- > 0) {
#ifdef CONFIG_SERIAL_W55FA92_HUART_1BYTE_TX
		if (ourport->rts_cts == 1) {
			while ((rd_regl(port, W55FA92_COM_FSR) & UART_FSR_TEMT) != UART_FSR_TEMT)
				;
			// delay for 1.5 stop bit
			ndelay(ns_delay);
			if (rd_regl(port, W55FA92_COM_MSR) & UART_MSR_CTS_st)
				break;
		}
#endif
		wr_regb(port, W55FA92_COM_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		w55fa92_serial_stop_tx(port);

 out:
	return IRQ_HANDLED;
}

static unsigned int w55fa92_serial_tx_empty(struct uart_port *port)
{

	unsigned long fsrstat = rd_regl(port, W55FA92_COM_FSR);

	if ((fsrstat&UART_FSR_TFE) == 0)
		return 0;
	else
		return 1;
}

/* no modem control lines */
static unsigned int w55fa92_serial_get_mctrl(struct uart_port *port)
{
	unsigned int msr, ret;

	msr = rd_regl(port, W55FA92_COM_MSR);

	ret = 0;
	if ((msr & UART_MSR_CTS_st) == 0)
		ret |= TIOCM_CTS;
	return ret;
}

static void w55fa92_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* todo - possibly remove AFC and do manual CTS */
}

static void w55fa92_serial_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	unsigned int ucon;

	spin_lock_irqsave(&port->lock, flags);

	ucon = rd_regl(port, W55FA92_COM_LCR);

	if (break_state)
		ucon |= UART_LCR_SBC;
	else
		ucon &= ~UART_LCR_SBC;

	wr_regl(port, W55FA92_COM_LCR, ucon);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void w55fa92_serial_shutdown(struct uart_port *port)
{
	struct w55fa92_uart_port *ourport = to_ourport(port);

	if (ourport->tx_claimed || ourport->rx_claimed) {
		if (ourport->tx_claimed) {	
			tx_disable(port);
			tx_enabled(port) = 0;
			ourport->tx_claimed = 0;
		}

		if (ourport->rx_claimed) {
			rx_disable(port);
			ourport->rx_claimed = 0;
			rx_enabled(port) = 0;
		}

		free_irq(RX_IRQ(port), ourport);
#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
		if (ourport->port.irq == IRQ_HUART)
			del_timer(&huart_timer);
#endif
		clk_disable(ourport->clk);
	}
}

static int w55fa92_serial_startup(struct uart_port *port)
{
	struct w55fa92_uart_port *ourport = to_ourport(port);
	int ret;

	clk_enable(ourport->clk);
	// set high-speed UART and UART multi-pin function
	//__raw_writel(__raw_readl(IRQLHSEL)|1<<8|0xF, IRQLHSEL);
	if (ourport->port.irq == IRQ_UART) {
		__raw_writel((__raw_readl(REG_GPAFUN1)&~(MF_GPA10|MF_GPA11))|(0x33<<8) , REG_GPAFUN1);
	}
	else if (ourport->port.irq == IRQ_HUART) {
		// check HUART kernel option
#ifdef CONFIG_SERIAL_W55FA92_HUART
		__raw_writel((__raw_readl(REG_GPDFUN0)&~(MF_GPD1|MF_GPD2))|(0x11<<4), REG_GPDFUN0);
#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
		__raw_writel((__raw_readl(REG_GPDFUN0)&~(MF_GPD3|MF_GPD4))|(0x11<<12), REG_GPDFUN0);
		wr_regl(port, W55FA92_COM_IER, rd_regl(port, W55FA92_COM_IER)|UART_IER_MSI);
#endif
#else
		printk(KERN_ERR "ERROR: kernel configuration does not support HUART option!!\n");
		return -ENODEV;
#endif
	}

	ret = request_irq(RX_IRQ(port),
			  w55fa92_serial_rx_chars, 0,
			  w55fa92_serial_portname(port), ourport);

	if (ret != 0) {
		printk(KERN_ERR "cannot get irq %d\n", RX_IRQ(port));
		return ret;
	}

	rx_enable(port);
	rx_enabled(port) = 1;

	ourport->rx_claimed = 1;
	ourport->tx_claimed = 1;

	return ret;
}

/* power power management control */

static void w55fa92_serial_pm(struct uart_port *port, unsigned int level,
			      unsigned int old)
{

}

/* baud rate calculation
 *
*/


#define MAX_CLKS (8)

struct baud_calc {
	struct w55fa92_uart_clksrc	*clksrc;
	unsigned int			 calc;
	unsigned int			 quot;
	struct clk			*src;
};

static int w55fa92_serial_calcbaud(struct baud_calc *calc,
				   struct uart_port *port,
				   struct w55fa92_uart_clksrc *clksrc,
				   unsigned int baud)
{
#ifdef CONFIG_SERIAL_W55FA92_HUART
	struct w55fa92_uart_port *ourport = to_ourport(port);

	if (ourport->port.irq == IRQ_HUART) {
		if (baud > 115200*8) {
			// set the HUART clock source to UPLL
			if ((__raw_readl(REG_CLKDIV3) & UART0_S) != (0x11 << 3)) {
				__raw_writel((__raw_readl(REG_CLKDIV3) & ~(UART0_S)) | (0x3 << 3), REG_CLKDIV3);
				port->uartclk = w55fa92_upll_clock*1000;
			}
		} else {
			// set the HUART clock source to XIN
			if ((__raw_readl(REG_CLKDIV3) & UART0_S) != (0x00 << 3)) {
				__raw_writel((__raw_readl(REG_CLKDIV3) & ~(UART0_S)) | (0x0 << 3), REG_CLKDIV3);
				port->uartclk = w55fa92_external_clock;
			}
		}
	}
#endif

	calc->quot = (port->uartclk * 10) / baud;
	if ((calc->quot % 10) >= 5)
		calc->quot = (calc->quot / 10) + 1;
	else
		calc->quot /= 10;
	calc->quot -= 2;

	return 1;
}

static unsigned int w55fa92_serial_getclk(struct uart_port *port,
					  struct w55fa92_uart_clksrc **clksrc,
					  struct clk **clk,
					  unsigned int baud)
{
	struct w55fa92_uartcfg *cfg = w55fa92_port_to_cfg(port);
	struct w55fa92_uart_clksrc *clkp;
	struct baud_calc res;

	clkp = cfg->clocks;
	
	w55fa92_serial_calcbaud(&res, port, clkp, baud);
	
	/* ok, we now need to select the best clock we found */

	printk(KERN_DEBUG "selected clock %x quot %d\n",
	       port->uartclk, res.quot);

	/* store results to pass back */

	*clksrc = res.clksrc;
	*clk    = res.src;

	return res.quot;
}

static void w55fa92_serial_set_termios(struct uart_port *port,
				       struct ktermios *termios,
				       struct ktermios *old)
{
#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
	struct w55fa92_uart_port *ourport = to_ourport(port);
#endif
	struct w55fa92_uart_clksrc *clksrc = NULL; 
	struct clk *clk = NULL;
	unsigned long flags;
	unsigned int baud, quot;
	unsigned int ulcon, mcr;

	/*
	 * We don't support modem control lines.
	 */
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * Ask the core to calculate the divisor for us.
	 */

	baud = uart_get_baud_rate(port, termios, old, 0, 4000000);
	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
		quot = port->custom_divisor;
	else
		quot = w55fa92_serial_getclk(port, &clksrc, &clk, baud);
	dbg("baud=%d, quot=%d\n", baud, quot);

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		dbg("w-config: 5bits/char\n");
		ulcon = UART_LCR_WLEN5;
		break;
	case CS6:
		dbg("w-config: 6bits/char\n");
		ulcon = UART_LCR_WLEN6;
		break;
	case CS7:
		dbg("w-config: 7bits/char\n");
		ulcon = UART_LCR_WLEN7;
		break;
	case CS8:
	default:
		dbg("w-config: 8bits/char\n");
		ulcon = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		ulcon |= UART_LCR_NSB1_5;

	mcr = 0;
#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
	ourport->rts_cts = 0;
	if ((termios->c_cflag & CRTSCTS) && (ourport->port.irq == IRQ_HUART)) {
		ourport->rts_cts = 1;
		wr_regl(port, W55FA92_COM_IER, rd_regl(port, W55FA92_COM_IER)|UART_IER_Auto_RTS|UART_IER_Auto_CTS);
		/* MCR & MSR control */
		//wr_regl(port, W55FA92_COM_MCR, rd_regl(port, W55FA92_COM_MCR)|UART_MCR_Lev_RTS);
		mcr = UART_MCR_Lev_RTS;
		wr_regl(port, W55FA92_COM_MSR, rd_regl(port, W55FA92_COM_MSR)|UART_MSR_Lev_CTS);
		wr_regl(port, W55FA92_COM_FCR, rd_regl(port, W55FA92_COM_FCR)|UARTx_FCR_RTS_LEVEL14|UARTx_FCR_FIFO_LEVEL14);
		// delay for 1.5 stop bit, 1000000000 / 2 / baud_rate
		ns_delay = 500000000 / baud;
	}
#endif

	if (termios->c_cflag & PARENB) {
		ulcon |= UART_LCR_PARITY;
		if (termios->c_cflag & PARODD)
			ulcon |= UART_LCR_OPAR;
		else
			ulcon |= UART_LCR_EPAR;
	} else {
		ulcon |= UART_LCR_NPAR;
	}
	spin_lock_irqsave(&port->lock, flags);
	wr_regl(port, W55FA92_COM_BAUD, (0x3<<28) | (quot&0xFFFF));
	wr_regl(port, W55FA92_COM_LCR, ulcon);
	wr_regl(port, W55FA92_COM_MCR, mcr);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Which character status flags are we interested in?
	 */
	port->read_status_mask = UART_FSR_ROE | UART_FSR_TOE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_FSR_FE | UART_FSR_PE;

	/*
	 * Which character status flags should we ignore?
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_FSR_ROE | UART_FSR_TOE;
	if (termios->c_iflag & IGNBRK && termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_FSR_FE;

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= RXSTAT_DUMMY_READ;
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *w55fa92_serial_type(struct uart_port *port)
{
	switch (port->type) {
	case PORT_W55FA92:
		return "W55FA92";
	default:
		return NULL;
	}
}

#define MAP_SIZE (0x100)

static void w55fa92_serial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, MAP_SIZE);
}

static int w55fa92_serial_request_port(struct uart_port *port)
{
	const char *name = w55fa92_serial_portname(port);
	return request_mem_region(port->mapbase, MAP_SIZE, name) ? 0 : -EBUSY;
}

static void w55fa92_serial_config_port(struct uart_port *port, int flags)
{
	struct w55fa92_uart_info *info = w55fa92_port_to_info(port);

	if (flags & UART_CONFIG_TYPE &&
	    w55fa92_serial_request_port(port) == 0)
		port->type = info->type;
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int
w55fa92_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct w55fa92_uart_info *info = w55fa92_port_to_info(port);

	if (ser->type != PORT_UNKNOWN && ser->type != info->type)
		return -EINVAL;

	return 0;
}


#ifdef CONFIG_SERIAL_W55FA92_CONSOLE

static struct console W55FA92_serial_console;

#define W55FA92_SERIAL_CONSOLE &W55FA92_serial_console
#else
#define W55FA92_SERIAL_CONSOLE NULL
#endif

static struct uart_ops w55fa92_serial_ops = {
	.pm		= w55fa92_serial_pm,
	.tx_empty	= w55fa92_serial_tx_empty,
	.get_mctrl	= w55fa92_serial_get_mctrl,
	.set_mctrl	= w55fa92_serial_set_mctrl,
	.stop_tx	= w55fa92_serial_stop_tx,
	.start_tx	= w55fa92_serial_start_tx,
	.stop_rx	= w55fa92_serial_stop_rx,
	.enable_ms	= w55fa92_serial_enable_ms,
	.break_ctl	= w55fa92_serial_break_ctl,
	.startup	= w55fa92_serial_startup,
	.shutdown	= w55fa92_serial_shutdown,
	.set_termios	= w55fa92_serial_set_termios,
	.type		= w55fa92_serial_type,
	.release_port	= w55fa92_serial_release_port,
	.request_port	= w55fa92_serial_request_port,
	.config_port	= w55fa92_serial_config_port,
	.verify_port	= w55fa92_serial_verify_port,
};


static struct uart_driver w55fa92_uart_drv = {
	.owner		= THIS_MODULE,
	.dev_name	= W55FA92_SERIAL_NAME,
	.nr		= NR_PORTS,
	.cons		= W55FA92_SERIAL_CONSOLE,
	.driver_name	= W55FA92_SERIAL_NAME,
//	.devfs_name	= W55FA92_SERIAL_DEVFS,
	.major		= W55FA92_SERIAL_MAJOR,
	.minor		= W55FA92_SERIAL_MINOR,
};

static struct w55fa92_uart_port w55fa92_serial_ports[NR_PORTS] = {
	[0] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= IRQ_HUART,
			.uartclk	= CLOCK_TICK_RATE,
			.fifosize	= 14,
			.ops		= &w55fa92_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 0,
		}
	},
	[1] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= IRQ_UART,
			.uartclk	= CLOCK_TICK_RATE,
			.fifosize	= 14,
			.ops		= &w55fa92_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 1,
		}
	}
};

/* w55fa92\_serial_resetport
 *
 * wrapper to call the specific reset for this port (reset the fifos
 * and the settings)
*/

static inline int w55fa92_serial_resetport(struct uart_port * port,
					   struct w55fa92_uartcfg *cfg)
{
	struct w55fa92_uart_info *info = w55fa92_port_to_info(port);

	return (info->reset_port)(port, cfg);
}

/* w55fa92\_serial_init_port
 *
 * initialise a single serial port from the platform device given
 */

static int w55fa92_serial_init_port(struct w55fa92_uart_port *ourport,
				    struct w55fa92_uart_info *info,
				    struct platform_device *platdev)
{
	struct uart_port *port = &ourport->port;
	struct w55fa92_uartcfg *cfg;
	struct resource *res;

	dbg("w55fa92_serial_init_port: port=%p, platdev=%p\n", port, platdev);

	if (platdev == NULL)
		return -ENODEV;
		
	cfg = w55fa92_dev_to_cfg(&platdev->dev);

	if (port->mapbase != 0)
		return 0;

	/* setup info for port */
	port->dev	= &platdev->dev;
	ourport->info	= info;

	/* copy the info in from provided structure */
	ourport->port.fifosize = info->fifosize;

	dbg("w55fa92_serial_init_port: %p (hw %d)...\n", port, cfg->hwport);

	port->uartclk = w55fa92_external_clock;
	
	if (cfg->uart_flags & UPF_CONS_FLOW) {
		dbg("w55fa92_serial_init_port: enabling flow control\n");
		port->flags |= UPF_CONS_FLOW;
	}
	
	/* sort our the physical and virtual addresses for each UART */

	res = platform_get_resource(platdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "failed to find memory resource for uart\n");
		return -EINVAL;
	}

	dbg("resource %p (%x..%x)\n", res, res->start, res->end);

	port->mapbase	= res->start;
	port->membase	= W55FA92_VA_UART + (res->start - W55FA92_PA_UART);
	port->irq	= platform_get_irq(platdev, 0);
	if (port->irq < 0)
		port->irq = 0;

	ourport->clk	= clk_get(&platdev->dev, NULL);

	dbg("port: map=%08x, mem=%p, irq=%d, clock=%d\n",
	    port->mapbase, port->membase, port->irq, port->uartclk);

	/* reset the fifos (and setup the uart) */
	w55fa92_serial_resetport(port, cfg);
	return 0;
}

/* Device driver serial port probe */

static int probe_index = 0;

static int w55fa92_serial_probe(struct platform_device *dev,
				struct w55fa92_uart_info *info)
{
	struct w55fa92_uart_port *ourport;
	int ret;

	dbg("w55fa92_serial_probe(%p, %p) %d\n", dev, info, probe_index);

	ourport = &w55fa92_serial_ports[probe_index];
	probe_index++;

	dbg("%s: initialising port %p...\n", __FUNCTION__, ourport);

	ret = w55fa92_serial_init_port(ourport, info, dev);
	if (ret < 0)
		goto probe_err;

	dbg("%s: adding port\n", __FUNCTION__);
	uart_add_one_port(&w55fa92_uart_drv, &ourport->port);
	platform_set_drvdata(dev, &ourport->port);

#ifdef CONFIG_SERIAL_W55FA92_HUART_RTS_CTS
	if (ourport->port.irq == IRQ_HUART) {
		init_timer(&huart_timer);
		huart_timer.function = read_cts;
		huart_timer.data = (unsigned long)(ourport);
	}
#endif

	return 0;

 probe_err:
	return ret;
}

static int w55fa92_serial_remove(struct platform_device *dev)
{
	struct uart_port *port = w55fa92_dev_to_port(&dev->dev);

	if (port)
		uart_remove_one_port(&w55fa92_uart_drv, port);

	return 0;
}

/* UART power management code */

#ifdef CONFIG_PM

static int w55fa92_serial_suspend(struct platform_device *dev, pm_message_t state)
{
	struct uart_port *port = w55fa92_dev_to_port(&dev->dev);

	if (port)
		uart_suspend_port(&w55fa92_uart_drv, port);

	return 0;
}

static int w55fa92_serial_resume(struct platform_device *dev)
{
	struct uart_port *port = w55fa92_dev_to_port(&dev->dev);
	struct w55fa92_uart_port *ourport = to_ourport(port);

	if (port) {
		clk_enable(ourport->clk);
		w55fa92_serial_resetport(port, w55fa92_port_to_cfg(port));
		clk_disable(ourport->clk);

		uart_resume_port(&w55fa92_uart_drv, port);
	}

	return 0;
}

#else
#define w55fa92_serial_suspend NULL
#define w55fa92_serial_resume  NULL
#endif

static int w55fa92_serial_init(struct platform_driver *drv,
			       struct w55fa92_uart_info *info)
{
	dbg("w55fa92_serial_init(%p,%p)\n", drv, info);
	return platform_driver_register(drv);
}


/* now comes the code to initialise either the W55FA92 serial
 * port information
*/

static int serial_resetport_w55fa92(struct uart_port *port,
				    struct w55fa92_uartcfg *cfg)
{
	dbg("w55fa92_serial_resetport: port=%p (%08x), cfg=%p\n",
	    port, port->mapbase, cfg);
	
	/* reset both fifos */
	if (port->line == 0)
		wr_regl(port, W55FA92_COM_FCR, UART_FCR_RFR | UART_FCR_TFR | UARTx_FCR_FIFO_LEVEL14); /* reset Tx and Rx FIFO */

	return 0;
}

static struct w55fa92_uart_info w55fa92_uart_inf = {
	.name		= "Nuvoton W55FA92 UART",
	.type		= PORT_W55FA92,
	.fifosize	= 16,
	.reset_port	= serial_resetport_w55fa92,
};

/* device management */

static int serial_probe_w55fa92(struct platform_device *dev)
{
	return w55fa92_serial_probe(dev, &w55fa92_uart_inf);
}

static struct platform_driver w55fa92_serial_drv = {
	.probe		= serial_probe_w55fa92,
	.remove		= w55fa92_serial_remove,
	.suspend	= w55fa92_serial_suspend,
	.resume		= w55fa92_serial_resume,
	.driver		= {
		.name	= "w55fa92-uart",
		.owner	= THIS_MODULE,
	},
};

static inline int serial_init_w55fa92(void)
{

	printk("W55FA92 uart driver has been initialized successfully!\n");
	
	return w55fa92_serial_init(&w55fa92_serial_drv, &w55fa92_uart_inf);
}

static inline void w55fa92_serial_exit(void)
{
	platform_driver_unregister(&w55fa92_serial_drv);
}

#define w55fa92_uart_inf_at &w55fa92_uart_inf


/* module initialisation code */

static int __init w55fa92_serial_modinit(void)
{
	int ret;

	ret = uart_register_driver(&w55fa92_uart_drv);
	if (ret < 0) {
		printk(KERN_ERR "failed to register UART driver\n");
		return -1;
	}

	serial_init_w55fa92();

	return 0;
}

static void __exit w55fa92_serial_modexit(void)
{
	w55fa92_serial_exit();

	uart_unregister_driver(&w55fa92_uart_drv);
}


module_init(w55fa92_serial_modinit);
module_exit(w55fa92_serial_modexit);

/* Console code */

#ifdef CONFIG_SERIAL_W55FA92_CONSOLE

static struct uart_port *cons_uart;

static int
w55fa92_serial_console_txrdy(struct uart_port *port, unsigned int ufcon)
{
	unsigned long fsrstat;

	fsrstat = rd_regl(port, W55FA92_COM_FSR);
	return (fsrstat & UART_FSR_TFF) ? 0 : 1;
}

static void
w55fa92_serial_console_putchar(struct uart_port *port, int ch)
{
#if 1//clyu for emulate
	while (!w55fa92_serial_console_txrdy(port, 0))
		barrier();
#endif
	wr_regb(cons_uart, W55FA92_COM_TX, ch);
}

static void
w55fa92_serial_console_write(struct console *co, const char *s,
			     unsigned int count)
{
	uart_console_write(cons_uart, s, count, w55fa92_serial_console_putchar);
}

static void __init
w55fa92_serial_get_options(struct uart_port *port, int *baud,
			   int *parity, int *bits)
{

	unsigned int lcrcon;


	lcrcon  = UART_LCR_WLEN8 | UART_LCR_NPAR;
	*baud = 115200;

		/* consider the serial port configured if the tx/rx mode set */

		switch (lcrcon & UART_LCR_CSMASK) {
		case UART_LCR_WLEN5:
			*bits = 5;
			break;
		case UART_LCR_WLEN6:
			*bits = 6;
			break;
		case UART_LCR_WLEN7:
			*bits = 7;
			break;
		default:
		case UART_LCR_WLEN8:
			*bits = 8;
			break;
		}
		
		if (lcrcon & UART_LCR_PARITY) {
			switch (lcrcon & UART_LCR_PMMASK) {
			case UART_LCR_EPAR:
				*parity = 'e';
				break;
			case UART_LCR_OPAR:
				*parity = 'o';
				break;
			default:
				*parity = 'n';
			}
		}
		else
			*parity = 'n';


		dbg("calculated baud %d\n", *baud);
	

}

/* w55fa92\_serial_init_ports
 *
 * initialise the serial ports from the machine provided initialisation
 * data.
*/

static int w55fa92_serial_init_ports(struct w55fa92_uart_info *info)
{
	struct w55fa92_uart_port *ptr = w55fa92_serial_ports;
	struct platform_device **platdev_ptr;
	int i;

	dbg("w55fa92_serial_init_ports: initialising ports...\n");

	platdev_ptr = w55fa92_uart_devs;

	for (i = 0; i < NR_PORTS; i++, ptr++, platdev_ptr++) {
		w55fa92_serial_init_port(ptr, info, *platdev_ptr);
	}

	return 0;
}

static int __init
w55fa92_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/* is this a valid port */

	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;

	port = &w55fa92_serial_ports[co->index].port;

	/* is the port configured? */

	if (port->mapbase == 0x0) {
		co->index = 0;
		port = &w55fa92_serial_ports[co->index].port;
	}

	cons_uart = port;

	dbg("w55fa92_serial_console_setup: port=%p (%d)\n", port, co->index);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		w55fa92_serial_get_options(port, &baud, &parity, &bits);

	dbg("w55fa92_serial_console_setup: baud %d\n", baud);
	return uart_set_options(port, co, baud, parity, bits, flow);
}

/* w55fa92_serial_initconsole
 *
 * initialise the console from one of the uart drivers
*/

static struct console w55fa92_serial_console =
{
	.name		= W55FA92_SERIAL_NAME,
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= w55fa92_serial_console_write,
	.setup		= w55fa92_serial_console_setup
};

static int w55fa92_serial_initconsole(void)
{
	struct w55fa92_uart_info *info;
	struct platform_device *dev = w55fa92_uart_devs[probe_index];

	dbg("w55fa92_serial_initconsole, %s\n", dev->name);

	/* select driver based on the cpu */

	if (dev == NULL) {
		printk(KERN_ERR "w55fa92: no devices for console init\n");
		return 0;
	}

	if (strcmp(dev->name, "w55fa92-uart") == 0) {
		info = w55fa92_uart_inf_at;
	} else {
		printk(KERN_ERR "w55fa92: no driver for %s\n", dev->name);
		return 0;
	}

	if (info == NULL) {
		printk(KERN_ERR "w55fa92: no driver for console\n");
		return 0;
	}

	w55fa92_serial_console.data = &w55fa92_uart_drv;
	w55fa92_serial_init_ports(info);

	register_console(&w55fa92_serial_console);
	return 0;
}

console_initcall(w55fa92_serial_initconsole);

#endif /* CONFIG_SERIAL_W55FA92_CONSOLE */

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nuvoton W55FA92 Serial port driver");
