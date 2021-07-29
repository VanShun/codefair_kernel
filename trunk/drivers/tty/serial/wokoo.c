/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

/*#if defined(CONFIG_SERIAL_WOKOO_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif*/

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/io.h>
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
#include <linux/cpufreq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <asm/irq.h>

#include "serial_mctrl_gpio.h"




#if 0
#if	defined(CONFIG_SERIAL_WOKOO_DEBUG) &&	\
	!defined(MODULE)

extern void printascii(const char *);

__printf(1, 2)
static void dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vscnprintf(buff, sizeof(buff), fmt, va);
	va_end(va);

	printascii(buff);
}

#else
#define dbg(fmt, ...) do { if (0) no_printk(fmt, ##__VA_ARGS__); } while (0)
#endif
#endif

/* UART name and device definitions */
#define WOKOO_UART_RBR          0x00 /* Receive buffer register */
#define WOKOO_UART_THR          0x00 /* Transmit holding register */
#define WOKOO_UART_DLL          0x00 /* Divisor latch low register */
#define WOKOO_UART_IER          0x04 /* Interupt enable register */
#define WOKOO_UART_DLH          0x04 /* Divisor latch high register */
#define WOKOO_UART_IIR          0x08 /* Interupt status register */
#define WOKOO_UART_FCR          0x08 /* FIEO control register */
#define WOKOO_UART_TCR          0x0c /* Transfer control register */
#define WOKOO_UART_MCR          0x10 /* Flow control register */
#define WOKOO_UART_TSR          0x14 /* Transfer status register */
#define WOKOO_UART_MSR          0x18 /* Flow status register */
#define WOKOO_UART_USR          0x7C /* Uart status register */

#define WOKOO_FCR_FIFOMODE      (1 << 0) /* FIFO enable */
#define WOKOO_UART_TSR_TXFULL   (1 << 5) /* Send empty */
#define WOKOO_UART_TSR_BI       (1 << 4) /* Stop error */
#define WOKOO_UART_TSR_FE       (1 << 3) /* Frame error */
#define WOKOO_UART_TSR_PE       (1 << 2) /* Odd error */
#define WOKOO_UART_TSR_OE       (1 << 1) /* FIFO over error */
#define WOKOO_UART_TSR_RBR      (1 << 0) /* Receive ready */
#define WOKOO_UART_MSR_DCTS     (1 << 0) /* CTS pin is change */
#define WOKOO_UART_MSR_CTS      (1 << 4) /* CTS pin value */
#define WOKOO_UART_MSR_AFC      (1 << 5) /* Enable AFC mode */
#define WOKOO_UART_MCR_RTS      (1 << 1) /* RTS pin value */
#define WOKOO_UART_IER_EMSI     (1 << 3) /* Flow change interupt */
#define WOKOO_UART_IER_ETSI     (1 << 2) /* Error interupt */
#define WOKOO_UART_IER_ETHEI    (1 << 1) /* TX empty interupt */
#define WOKOO_UART_IER_ERDAI    (1 << 0) /* RX valid and timeout interupt */
#define WOKOO_UART_TCR_DLAB     (1 << 7) /* DLAB enable */
#define WOKOO_UART_TCR_BREAK    (1 << 6) /* Stop control bit */
#define WOKOO_UART_TCR_EPS      (1 << 4) /* Parity select bit */
#define WOKOO_UART_TCR_PEN      (1 << 3) /* Parity select bit */
#define WOKOO_UART_TCR_STOP     (1 << 2) /* Stop bit number */
#define WOKOO_UART_IIR_FLOW     0x0 /* Flow change */
#define WOKOO_UART_IIR_NONE     0x1 /* Not interupt */
#define WOKOO_UART_IIR_THR      0x2 /* Transmit empty */
#define WOKOO_UART_IIR_REV      0x4 /* Receive valid */
#define WOKOO_UART_IIR_ERR      0x6 /* Error */
#define WOKOO_UART_IIR_BUSY     0x7 /* Busy */
#define WOKOO_UART_IIR_TOUT     0xc /* Char timeout */
#define WOKOO_UART_RCVR_SHF     6 /* FIFO receive threshold */
#define WOKOO_UART_TXMT_SHF     4 /* FIFO send threshold */
#define WOKOO_UART_CLS_SHF      0
#define WOKOO_UART_HAVE_RTSCTS  (1 << 0)

/* We've been assigned a range on the "Low-density serial ports" major */
#define SERIAL_WOKOO_MAJOR	207
#define MINOR_START			16
#define DEV_NAME			"ttyWK"


/*
 * This determines how often we check the modem status signals
 * for any change.  They generally aren't connected to an IRQ
 * so we have to poll them.  We also check immediately before
 * filling the TX fifo incase CTS has been dropped.
 */
#define MCTRL_TIMEOUT	(250*HZ/1000)

#define DRIVER_NAME		"WOKOO-uart"

#define UART_NR			3

#define WOKOO_TX_BUF_L	200
#define WOKOO_RX_BUF_L	200




struct wokoouart_platform_data {
	unsigned int flags;
};

/**
 * struct wokoo_port - WOKOO uart custom configuration
 * @port: uart device
 * @timer: timer
 * @status: flow control status 
 * @old_status: flow control old status 
 * @have_rtscts: have rtscts 
 * @have_rtsgpio: have rts gpio 
 * @dte_mode: dte mode 
 * @clk_pclk: gate clock 
 * @clk_sclk: source clock 
 * @gpios: mctrl gpios 
 */
struct wokoo_port {
	struct uart_port	port;
	struct timer_list	timer;
	unsigned int		status;
	unsigned int		old_status;
	unsigned int		have_rtscts:1;
	unsigned int		have_rtsgpio:1;
	unsigned int		dte_mode:1;
	struct clk			*clk_pclk;
	struct clk			*clk_sclk;

	struct mctrl_gpios	*gpios;
	unsigned int		saved_reg[6];
	bool				context_saved;
};


static struct wokoo_port *wokoo_uart_ports[UART_NR];




/*
 * wokoo_set_bits - set reg bits
 */
static void wokoo_set_bits(struct uart_port *port, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(port->membase + reg);
	val |= bits;
	writel_relaxed(val, port->membase + reg);
}

/*
 * wokoo_clr_bits - clear reg bits
 */
static void  wokoo_clr_bits(struct uart_port *port, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(port->membase + reg);
	val &= ~bits;
	writel_relaxed(val, port->membase + reg);
}

/*
 * We have a modem side uart, so the meanings of RTS and CTS are inverted.
 */
static unsigned int wokoo_uart_get_hwmctrl(struct wokoo_port *sport)
{
	unsigned int tmp = TIOCM_DSR;
	unsigned int msr = readl_relaxed(sport->port.membase + WOKOO_UART_MSR);

	if (msr & WOKOO_UART_MSR_CTS)
		tmp |= TIOCM_CTS;

	return tmp;
}

/*
 * wokoo_uart_rts_active - set RTS active
 */
static void wokoo_uart_rts_active(struct wokoo_port *sport)
{
	wokoo_clr_bits(&sport->port, WOKOO_UART_MCR, WOKOO_UART_MCR_RTS);

	sport->port.mctrl |= TIOCM_RTS;
	mctrl_gpio_set(sport->gpios, sport->port.mctrl);
}

/*
 * wokoo_uart_rts_inactive - set RTS inactive
 */
static void wokoo_uart_rts_inactive(struct wokoo_port *sport)
{
	wokoo_set_bits(&sport->port, WOKOO_UART_MCR, WOKOO_UART_MCR_RTS);

	sport->port.mctrl &= ~TIOCM_RTS;
	mctrl_gpio_set(sport->gpios, sport->port.mctrl);
}

static void wokoo_uart_start_rx(struct uart_port *port)
{
	wokoo_set_bits(port, WOKOO_UART_IER, WOKOO_UART_IER_ERDAI);
}

/*
 * wokoo_uart_stop_tx - stop tx
 */
static void wokoo_uart_stop_tx(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;

	/* Disable THR interput */
	wokoo_clr_bits(port, WOKOO_UART_IER, WOKOO_UART_IER_ETHEI);

	/* in rs485 mode disable transmitter if shifter is empty */
	if (port->rs485.flags & SER_RS485_ENABLED &&
		!(readl_relaxed(sport->port.membase + WOKOO_UART_TSR) & WOKOO_UART_TSR_TXFULL)) {
		if (port->rs485.flags & SER_RS485_RTS_AFTER_SEND)
			wokoo_uart_rts_active(sport);
		else
			wokoo_uart_rts_inactive(sport);

		wokoo_uart_start_rx(port);
	}
}

/*
 * wokoo_uart_stop_rx - stop rx
 */
static void wokoo_uart_stop_rx(struct uart_port *port)
{
	wokoo_clr_bits(port, WOKOO_UART_IER, WOKOO_UART_IER_ERDAI);
}

/*
 * wokoo_uart_enable_ms - enable timer
 */
static void wokoo_uart_enable_ms(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;

	mod_timer(&sport->timer, jiffies);
	mctrl_gpio_enable_ms(sport->gpios);
}

/*
 * wokoo_uart_transmit_buffer - transmit buffer
 */
static inline void wokoo_uart_transmit_buffer(struct wokoo_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;

	if (sport->port.x_char) {
		/* Send next char */
		writel_relaxed(sport->port.x_char, sport->port.membase + WOKOO_UART_THR);
		sport->port.icount.tx++;
		sport->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
		wokoo_uart_stop_tx(&sport->port);
		return;
	}

	while (!uart_circ_empty(xmit) &&
			(readl_relaxed(sport->port.membase + WOKOO_UART_TSR) & WOKOO_UART_TSR_TXFULL)) {
		/* send xmit->buf[xmit->tail]
		 * out the port here */
		writel_relaxed(xmit->buf[xmit->tail], sport->port.membase + WOKOO_UART_THR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
		/* If have a receive interput, stop and return */
		if (readl_relaxed(sport->port.membase + WOKOO_UART_TSR) & WOKOO_UART_TSR_RBR) {
			if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
				uart_write_wakeup(&sport->port);
			wokoo_uart_stop_tx(&sport->port);
			return;
		}
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		wokoo_uart_stop_tx(&sport->port);
}

/*
 * wokoo_uart_start_tx - start tx
 */
static void wokoo_uart_start_tx(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;

	if (!sport->port.x_char && uart_circ_empty(&port->state->xmit))
		return;

	if (port->rs485.flags & SER_RS485_ENABLED) {
		/* RTS pin control */
		if (port->rs485.flags & SER_RS485_RTS_ON_SEND)
			wokoo_clr_bits(port, WOKOO_UART_MCR, WOKOO_UART_MCR_RTS);
		else
			wokoo_set_bits(port, WOKOO_UART_MCR, WOKOO_UART_MCR_RTS);

		if (!(port->rs485.flags & SER_RS485_RX_DURING_TX))
			wokoo_uart_stop_rx(port);
	}

	/* Enable thr interput */
	wokoo_set_bits(port, WOKOO_UART_IER, WOKOO_UART_IER_ETHEI);
}

#define TXMT_DEFAULT 0x0 /* reset default */
#define RCVR_DEFAULT 0x0 /* reset default */

/*
 * wokoo_uart_setup_ufcr - FIFO setting
 */
static void wokoo_uart_setup_ufcr(struct wokoo_port *sport,
				unsigned char txmt, unsigned char rcvr)
{
	unsigned int val;

	/* set receiver / transmitter trigger level */
	val = readl_relaxed(sport->port.membase + WOKOO_UART_FCR);
	val |= (txmt << WOKOO_UART_TXMT_SHF) | (rcvr << WOKOO_UART_RCVR_SHF) | WOKOO_FCR_FIFOMODE;
	writel_relaxed(val, sport->port.membase + WOKOO_UART_FCR);
}

/*
 * wokoo_uart_disable_irq - disable irq
 */
static void wokoo_uart_disable_irq(struct uart_port *port)
{
	/* Clear receive buffer */
	while (readl_relaxed(port->membase + WOKOO_UART_TSR) & WOKOO_UART_TSR_RBR)
	{
		readl_relaxed(port->membase + WOKOO_UART_RBR);
	}

	/* disable THR interrupt before requesting IRQs */
	/* disable RV interrupt before requesting IRQs  */
	/* disable error interrupt before requesting IRQs */
	/* disable stream interrupt before requesting IRQs */
	wokoo_clr_bits(port, WOKOO_UART_IER, WOKOO_UART_IER_ETHEI | WOKOO_UART_IER_ERDAI |
										 WOKOO_UART_IER_ETSI  | WOKOO_UART_IER_EMSI);
}

/*
 * wokoo_uart_enable_irq - enable irq
 */
static void wokoo_uart_enable_irq(struct uart_port *port)
{
	wokoo_set_bits(port, WOKOO_UART_IER, WOKOO_UART_IER_ETHEI | WOKOO_UART_IER_ERDAI |
										 WOKOO_UART_IER_ETSI/*  | WOKOO_UART_IER_EMSI*/);
}

/*
 * wokoo_uart_startup - startup
 */
static int wokoo_uart_startup(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	unsigned long flags;
	int           retval;

	retval = clk_prepare_enable(sport->clk_pclk);
	if (retval) {
		return retval;
	}

	wokoo_uart_setup_ufcr(sport, TXMT_DEFAULT, RCVR_DEFAULT);

	wokoo_uart_disable_irq(port);

	spin_lock_irqsave(&sport->port.lock, flags);

	/*
	 * Finally, clear and enable interrupts
	 */
	wokoo_uart_enable_irq(port);

	spin_unlock_irqrestore(&sport->port.lock, flags);

	return 0;
}

/*
 * wokoo_uart_shutdown - shutdown
 */
static void wokoo_uart_shutdown(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	unsigned long flags;

	mctrl_gpio_disable_ms(sport->gpios);

	/* Disable interupt */
	spin_lock_irqsave(&sport->port.lock, flags);
	wokoo_clr_bits(port, WOKOO_UART_IER, WOKOO_UART_IER_ETHEI | WOKOO_UART_IER_ERDAI |
										 WOKOO_UART_IER_ETSI  | WOKOO_UART_IER_EMSI);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	/*
	 * Stop our timer.
	 */
	del_timer_sync(&sport->timer);

	/* Disable gate clock */
	clk_disable_unprepare(sport->clk_pclk);
}

static void wokoo_uart_flush_buffer(struct uart_port *port)
{
}

/*
 * wokoo_uart_set_termios - set term
 */
static void wokoo_uart_set_termios(struct uart_port *port,
				struct ktermios *termios, struct ktermios *old)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	unsigned long flags;
	u32 utcr, old_utcr, cls;
	unsigned int baud, quot;
	unsigned long div;


	del_timer_sync(&sport->timer);

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);
	quot = uart_get_divisor(port, baud);

	spin_lock_irqsave(&sport->port.lock, flags);

	/*
	 * Read current UCR2 and save it for future use, then clear all the bits
	 * except those we will or may need to preserve.
	 */
	old_utcr = readl_relaxed(sport->port.membase + WOKOO_UART_TCR);
	utcr     = old_utcr;

	/* Setting transfer data length */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cls = 0x0;
		break;
	case CS6:
		cls = 0x1;
		break;
	case CS7:
		cls = 0x2;
		break;
	default:
		cls = 0x3;
		break;
	}

	utcr &= 0xfffffffc;
	utcr |= cls;

	if (!sport->have_rtscts)
		termios->c_cflag &= ~CRTSCTS;

	if (port->rs485.flags & SER_RS485_ENABLED) {
		/*
		 * RTS is mandatory for rs485 operation, so keep
		 * it under manual control and keep transmitter
		 * disabled.
		 */
		if (port->rs485.flags & SER_RS485_RTS_AFTER_SEND)
			wokoo_uart_rts_active(sport);
		else
			wokoo_uart_rts_inactive(sport);

	} else if (termios->c_cflag & CRTSCTS) {
		wokoo_set_bits(port, WOKOO_UART_MCR, WOKOO_UART_MSR_AFC);
	} else {
		wokoo_clr_bits(port, WOKOO_UART_MCR, WOKOO_UART_MSR_AFC);
	}

	if (termios->c_cflag & CSTOPB)
		utcr |= WOKOO_UART_TCR_STOP;
	if (termios->c_cflag & PARENB) {
		utcr |= WOKOO_UART_TCR_PEN;
		if (termios->c_cflag & PARODD)
			utcr |= WOKOO_UART_TCR_EPS;
	}

	sport->port.read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |= (WOKOO_UART_TSR_FE | WOKOO_UART_TSR_PE);
	if (termios->c_iflag & (BRKINT | PARMRK))
		sport->port.read_status_mask |= WOKOO_UART_TSR_BI;

	/*
	 * Characters to ignore
	 */
	sport->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		sport->port.ignore_status_mask |= WOKOO_UART_TSR_PE | WOKOO_UART_TSR_FE;
	if (termios->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |= WOKOO_UART_TSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |= WOKOO_UART_TSR_OE;
	}

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/* custom-baudrate handling */
	div = sport->port.uartclk / (baud * 16);
	if (baud == 38400 && quot != div)
		baud = sport->port.uartclk / (quot * 16);

	div = sport->port.uartclk / (baud * 16);
	if (!div)
		div = 1;

	writel_relaxed(utcr, sport->port.membase + WOKOO_UART_TCR);
	/* Setting baudrate */
	wokoo_set_bits(port, WOKOO_UART_TCR, WOKOO_UART_TCR_DLAB);
	writel_relaxed(div & 0xFF, sport->port.membase + WOKOO_UART_DLL);
	writel_relaxed((div & 0xFF00) >> 8, sport->port.membase + WOKOO_UART_DLH);
	wokoo_clr_bits(port, WOKOO_UART_TCR, WOKOO_UART_TCR_DLAB);

	if (UART_ENABLE_MS(&sport->port, termios->c_cflag))
		wokoo_uart_enable_ms(&sport->port);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

/*
 * wokoo_uart_type - set uart type
 */
static const char *wokoo_uart_type(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;

	return sport->port.type == PORT_WOKOO ? "WOKOO" : NULL;
}

/*
 * Configure/autoconfigure the port.
 */
static void wokoo_uart_config_port(struct uart_port *port, int flags)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;

	if (flags & UART_CONFIG_TYPE)
		sport->port.type = PORT_WOKOO;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_IMX and PORT_UNKNOWN
 */
static int
wokoo_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_WOKOO)
		ret = -EINVAL;
	if (sport->port.irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;
	if (sport->port.uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if (sport->port.mapbase != (unsigned long)ser->iomem_base)
		ret = -EINVAL;
	if (sport->port.iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

#if defined(CONFIG_CONSOLE_POLL)

static int wokoo_uart_poll_init(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	unsigned long flags;
	u32 ucr1, ucr2;

	clk_prepare_enable(sport->clk_sclk);
	wokoo_uart_setup_ufcr(sport, TXMT_DEFAULT, RCVR_DEFAULT);

	spin_lock_irqsave(&sport->port.lock, flags);
	wokoo_set_bits(port, WOKOO_UART_IER, WOKOO_UART_IER_ETHEI | WOKOO_UART_IER_ERDAI |
										 WOKOO_UART_IER_ETSI/*  | WOKOO_UART_IER_EMSI*/);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	return 0;
}

static int wokoo_uart_poll_get_char(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	if (!(readl_relaxed(sport->port.membase + WOKOO_UART_TSR) & WOKOO_UART_TSR_DR))
		return NO_POLL_CHAR;

	return readl_relaxed(sport->port.membase + WOKOO_UART_RBR);
}

static void wokoo_uart_poll_put_char(struct uart_port *port, unsigned char c)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	unsigned int status;

	/* drain */
	do {
		status = readl_relaxed(sport->port.membase + WOKOO_UART_TSR);
	} while (!(status & WOKOO_UART_TSR_TXFULL));

	/* write */
	writel_relaxed(c, sport->port.membase + WOKOO_UART_THR);
}
#endif

/*
 * wokoo_uart_rs485_config - rs485 config
 */
static int wokoo_uart_rs485_config(struct uart_port *port,
				 struct serial_rs485 *rs485conf)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;

	/* unimplemented */
	rs485conf->delay_rts_before_send = 0;
	rs485conf->delay_rts_after_send = 0;

	/* RTS is required to control the transmitter */
	if (!sport->have_rtscts && !sport->have_rtsgpio)
		rs485conf->flags &= ~SER_RS485_ENABLED;

	if (rs485conf->flags & SER_RS485_ENABLED) {
		/* Enable receiver if low-active RTS signal is requested */
		if (sport->have_rtscts &&  !sport->have_rtsgpio &&
			!(rs485conf->flags & SER_RS485_RTS_ON_SEND))
			rs485conf->flags |= SER_RS485_RX_DURING_TX;

		/* disable transmitter */
		if (rs485conf->flags & SER_RS485_RTS_AFTER_SEND)
			wokoo_uart_rts_active(sport);
		else
			wokoo_uart_rts_inactive(sport);
	}

	/* Make sure Rx is enabled in case Tx is active with Rx disabled */
	if (!(rs485conf->flags & SER_RS485_ENABLED) ||
		rs485conf->flags & SER_RS485_RX_DURING_TX)
		wokoo_uart_start_rx(port);

	port->rs485 = *rs485conf;

	return 0;
}

/*
 * wokoo_uart_rtsint - rts interupt
 */
static irqreturn_t wokoo_uart_rtsint(int irq, void *dev_id)
{
	struct  wokoo_port *sport = dev_id;
	u32 mcr;

	spin_lock(&sport->port.lock);

	mcr = readl_relaxed(sport->port.membase + WOKOO_UART_MCR) & WOKOO_UART_MCR_RTS;
	uart_handle_cts_change(&sport->port, !mcr);
	wake_up_interruptible(&sport->port.state->port.delta_msr_wait);

	spin_unlock(&sport->port.lock);
	return IRQ_HANDLED;
}

/*
 * wokoo_uart_rtsint - transmit interupt
 */
static irqreturn_t wokoo_uart_txint(int irq, void *dev_id)
{
	struct wokoo_port *sport = dev_id;

	spin_lock(&sport->port.lock);
	wokoo_uart_transmit_buffer(sport);
	spin_unlock(&sport->port.lock);
	return IRQ_HANDLED;
}

/*
 * wokoo_uart_rxint - Receive interupt
 */
static irqreturn_t wokoo_uart_rxint(int irq, void *dev_id)
{
	struct wokoo_port *sport = dev_id;
	unsigned int rx, tsr, flg, ignored = 0;
	struct tty_port *port = &sport->port.state->port;

	spin_lock(&sport->port.lock);
	wokoo_uart_stop_tx(&sport->port);

	while ( (tsr = readl_relaxed(sport->port.membase + WOKOO_UART_TSR)) & WOKOO_UART_TSR_RBR) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		rx  = readl_relaxed(sport->port.membase + WOKOO_UART_RBR);

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		if (tsr & WOKOO_UART_TSR_BI)
			sport->port.icount.brk++;
		if (tsr & WOKOO_UART_TSR_PE)
			sport->port.icount.parity++;
		if (tsr & WOKOO_UART_TSR_FE)
			sport->port.icount.frame++;
		if (tsr & WOKOO_UART_TSR_OE)
			sport->port.icount.overrun++;

		if (tsr & sport->port.ignore_status_mask) {
			if (++ignored > 100)
				goto out;
			continue;
		}

		tsr &= (sport->port.read_status_mask | 0xFF);

		if (tsr & WOKOO_UART_TSR_BI)
			flg = TTY_BREAK;
		else if (tsr & WOKOO_UART_TSR_PE)
			flg = TTY_PARITY;
		else if (tsr & WOKOO_UART_TSR_FE)
			flg = TTY_FRAME;
		if (tsr & WOKOO_UART_TSR_OE)
			flg = TTY_OVERRUN;

#ifdef SUPPORT_SYSRQ
		sport->port.sysrq = 0;
#endif

		if (tty_insert_flip_char(port, rx, flg) == 0)
			sport->port.icount.buf_overrun++;
	}

out:
	wokoo_uart_start_tx(&sport->port);
	spin_unlock(&sport->port.lock);
	tty_flip_buffer_push(port);
	return IRQ_HANDLED;
}

/*
 * Handle any change of modem status signal since we were last called.
 */
static void wokoo_uart_mctrl_check(struct wokoo_port *sport)
{
	unsigned int status, changed;

	status = wokoo_uart_get_hwmctrl(sport);
	changed = status ^ sport->old_status;

	if (changed == 0)
		return;

	sport->old_status = status;

	if (changed & TIOCM_RI && status & TIOCM_RI)
		sport->port.icount.rng++;
	if (changed & TIOCM_DSR)
		sport->port.icount.dsr++;
	if (changed & TIOCM_CAR)
		uart_handle_dcd_change(&sport->port, status & TIOCM_CAR);
	if (changed & TIOCM_CTS)
		uart_handle_cts_change(&sport->port, status & TIOCM_CTS);

	wake_up_interruptible(&sport->port.state->port.delta_msr_wait);
}

/*
 * wokoo_uart_int - uart interupt
 */
static irqreturn_t wokoo_uart_int(int irq, void *dev_id)
{
	struct wokoo_port *sport = dev_id;
	unsigned int uiir;
	irqreturn_t ret = IRQ_NONE;

	uiir = readl_relaxed(sport->port.membase + WOKOO_UART_IIR) & 0xF;

	if (uiir == WOKOO_UART_IIR_REV || uiir == WOKOO_UART_IIR_TOUT) {
		wokoo_uart_rxint(irq, dev_id);
		ret = IRQ_HANDLED;
	} else if (uiir == WOKOO_UART_IIR_THR) {
		wokoo_uart_txint(irq, dev_id);
		ret = IRQ_HANDLED;
	} else if (uiir == WOKOO_UART_IIR_FLOW) {
		spin_lock(&sport->port.lock);
		wokoo_uart_mctrl_check(sport);
		readl_relaxed(sport->port.membase + WOKOO_UART_TSR);
		spin_unlock(&sport->port.lock);

		ret = IRQ_HANDLED;
	} else {
		readl_relaxed(sport->port.membase + WOKOO_UART_TSR);
		ret = IRQ_HANDLED;
	}

	return ret;
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int wokoo_uart_tx_empty(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	unsigned int ret;

	ret = (readl_relaxed(sport->port.membase + WOKOO_UART_TSR) & WOKOO_UART_TSR_TXFULL) ? TIOCSER_TEMT : 0;
	return ret;
}

/*
 * wokoo_uart_get_mctrl - get mctrl
 */
static unsigned int wokoo_uart_get_mctrl(struct uart_port *port)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	unsigned int ret = wokoo_uart_get_hwmctrl(sport);

	mctrl_gpio_get(sport->gpios, &ret);

	return ret;
}

/*
 * wokoo_uart_set_mctrl - set mctrl
 */
static void wokoo_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;
	
	if ((mctrl & TIOCM_RTS) && (port->status & UPSTAT_AUTORTS))
		wokoo_clr_bits(port, WOKOO_UART_MCR, WOKOO_UART_MCR_RTS);
	else
		wokoo_set_bits(port, WOKOO_UART_MCR, WOKOO_UART_MCR_RTS);

	mctrl_gpio_set(sport->gpios, mctrl);
}

/*
 * Interrupts always disabled.
 */
static void wokoo_uart_break_ctl(struct uart_port *port, int break_state)
{
}

/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void wokoo_uart_timeout(struct timer_list *t)
{
	struct wokoo_port *sport = from_timer(sport, t, timer);
	unsigned long flags;

	if (sport->port.state) {
		spin_lock_irqsave(&sport->port.lock, flags);
		wokoo_uart_mctrl_check(sport);
		spin_unlock_irqrestore(&sport->port.lock, flags);

		mod_timer(&sport->timer, jiffies + MCTRL_TIMEOUT);
	}
}

static const struct uart_ops wokoo_uart_pops = {
	.tx_empty	= wokoo_uart_tx_empty,
	.set_mctrl	= wokoo_uart_set_mctrl,
	.get_mctrl	= wokoo_uart_get_mctrl,
	.stop_tx	= wokoo_uart_stop_tx,
	.start_tx	= wokoo_uart_start_tx,
	.stop_rx	= wokoo_uart_stop_rx,
	.enable_ms	= wokoo_uart_enable_ms,
	.break_ctl	= wokoo_uart_break_ctl,
	.startup	= wokoo_uart_startup,
	.shutdown	= wokoo_uart_shutdown,
	.flush_buffer	= wokoo_uart_flush_buffer,
	.set_termios	= wokoo_uart_set_termios,
	.type			= wokoo_uart_type,
	.config_port	= wokoo_uart_config_port,
	.verify_port	= wokoo_uart_verify_port,
#if defined(CONFIG_CONSOLE_POLL)
	.poll_init		= wokoo_uart_poll_init,
	.poll_get_char	= wokoo_uart_poll_get_char,
	.poll_put_char	= wokoo_uart_poll_put_char,
#endif
};

#ifdef CONFIG_SERIAL_WOKOO_CONSOLE
static void wokoo_uart_console_putchar(struct uart_port *port, int ch)
{
	struct  wokoo_port *sport = (struct  wokoo_port *)port;

	while (!(readl_relaxed(sport->port.membase + WOKOO_UART_TSR) & WOKOO_UART_TSR_TXFULL))
		barrier();

	writel_relaxed(ch, sport->port.membase + WOKOO_UART_THR);
}

/*
 * Interrupts are disabled on entering
 */
static void
wokoo_uart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct wokoo_port *sport = wokoo_uart_ports[co->index];
	unsigned long flags = 0;
	int locked = 1;

	clk_prepare_enable(sport->clk_pclk);

	if (sport->port.sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&sport->port.lock, flags);
	else
		spin_lock_irqsave(&sport->port.lock, flags);

	uart_console_write(&sport->port, s, count, wokoo_uart_console_putchar);

	if (locked)
		spin_unlock_irqrestore(&sport->port.lock, flags);

	clk_disable_unprepare(sport->clk_pclk);
}

/*
 * wokoo_uart_console_setup
 */
static int __init
wokoo_uart_console_setup(struct console *co, char *options)
{
	struct wokoo_port *sport;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int retval;

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(wokoo_uart_ports))
		co->index = 0;
	sport = wokoo_uart_ports[co->index];
	if (sport == NULL)
		return -ENODEV;

	/* For setting the registers, we only need to enable the ipg clock. */
	retval = clk_prepare_enable(sport->clk_pclk);
	if (retval) {
		goto error_console;
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	wokoo_uart_setup_ufcr(sport, TXMT_DEFAULT, RCVR_DEFAULT);

	retval = uart_set_options(&sport->port, co, baud, parity, bits, flow);

	//clk_disable(sport->clk_pclk);
	if (retval) {
		clk_unprepare(sport->clk_pclk);
		goto error_console;
	}

	retval = clk_prepare(sport->clk_sclk);
	if (retval) {
		clk_unprepare(sport->clk_pclk);
	}

error_console:
	return retval;
}

static struct uart_driver wokoo_uart_uart_driver;
static struct console wokoo_uart_console = {
	.name		= DEV_NAME,
	.write		= wokoo_uart_console_write,
	.device		= uart_console_device,
	.setup		= wokoo_uart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &wokoo_uart_uart_driver,
};

static int __init wokoo_serial_console_init(void)
{
	register_console(&wokoo_uart_console);
	return 0;
}
console_initcall(wokoo_serial_console_init);

#define WOKOO_CONSOLE	&wokoo_uart_console

#ifdef CONFIG_OF
static void wokoo_uart_console_early_putchar(struct uart_port *port, int ch)
{
	struct wokoo_port *sport = (struct wokoo_port *)port;

	while (!(readl_relaxed(sport->port.membase + WOKOO_UART_TSR) & WOKOO_UART_TSR_TXFULL))
		cpu_relax();

	writel_relaxed(ch, sport->port.membase + WOKOO_UART_THR);
}

static void wokoo_uart_console_early_write(struct console *con, const char *s,
					 unsigned count)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, count, wokoo_uart_console_early_putchar);
}

static int __init
wokoo_console_early_setup(struct earlycon_device *dev, const char *opt)
{
	if (!dev->port.membase)
		return -ENODEV;

	dev->con->write = wokoo_uart_console_early_write;

	return 0;
}
OF_EARLYCON_DECLARE(wokoo, "wokoo,wokoo-uart", wokoo_console_early_setup);
#endif

#else
#define WOKOO_CONSOLE	NULL
#endif

static struct uart_driver wokoo_uart_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
	.major          = SERIAL_WOKOO_MAJOR,
	.minor          = MINOR_START,
	.nr             = ARRAY_SIZE(wokoo_uart_ports),
	.cons           = WOKOO_CONSOLE,
};


#ifdef CONFIG_OF
/*
 * This function returns 1 iff pdev isn't a device instatiated by dt, 0 iff it
 * could successfully get all information from dt or a negative errno.
 */
static int wokoo_uart_probe_dt(struct wokoo_port *sport,
				struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		return ret;
	}
	sport->port.line = ret;

	if (of_get_property(np, "uart-has-rtscts", NULL) ||
		of_get_property(np, "wokoo,uart-has-rtscts", NULL) /* deprecated */)
		sport->have_rtscts = 1;

	if (of_get_property(np, "wokoo,dte-mode", NULL))
		sport->dte_mode = 1;

	if (of_get_property(np, "rts-gpios", NULL))
		sport->have_rtsgpio = 1;

	return 0;
}
#else
static inline int wokoo_uart_probe_dt(struct wokoo_port *sport,
					struct platform_device *pdev)
{
	return 1;
}
#endif

static int wokoo_uart_probe(struct platform_device *pdev)
{
	struct wokoo_port *sport;
	void __iomem *base;
	int ret = 0;
	struct resource *res;
	int txirq, rxirq, rtsirq;

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	ret = wokoo_uart_probe_dt(sport, pdev);
	if (ret < 0)
		return ret;

	if (sport->port.line >= ARRAY_SIZE(wokoo_uart_ports)) {
		dev_err(&pdev->dev, "serial%d out of range\n",
			sport->port.line);
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	rxirq = platform_get_irq(pdev, 0);
	txirq = platform_get_irq_optional(pdev, 1);
	rtsirq = platform_get_irq_optional(pdev, 2);

	sport->port.dev = &pdev->dev;
	sport->port.mapbase = res->start;
	sport->port.membase = base;
	sport->port.type = PORT_WOKOO,
	sport->port.iotype = UPIO_MEM;
	sport->port.irq = rxirq;
	sport->port.fifosize = 32;
	sport->port.rs485_config = wokoo_uart_rs485_config;
	sport->port.ops = &wokoo_uart_pops;
	sport->port.flags = UPF_BOOT_AUTOCONF;
	timer_setup(&sport->timer, wokoo_uart_timeout, 0);

	sport->gpios = mctrl_gpio_init(&sport->port, 0);
	if (IS_ERR(sport->gpios))
		return PTR_ERR(sport->gpios);

	sport->clk_pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(sport->clk_pclk)) {
		ret = PTR_ERR(sport->clk_pclk);
		dev_err(&pdev->dev, "failed to get ipg clk: %d\n", ret);
		return ret;
	}

	sport->clk_sclk = devm_clk_get(&pdev->dev, "sclk");
	if (IS_ERR(sport->clk_sclk)) {
		ret = PTR_ERR(sport->clk_sclk);
		dev_err(&pdev->dev, "failed to get per clk: %d\n", ret);
		return ret;
	}


	sport->port.uartclk = clk_get_rate(sport->clk_sclk);

	/* For register access, we only need to enable the ipg clock. */
	ret = clk_prepare_enable(sport->clk_pclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable per clk: %d\n", ret);
		return ret;
	}

	wokoo_uart_disable_irq(&sport->port);

	/*
	 * Allocate the IRQ(s) has three interrupts whereas later
	 * chips only have one interrupt.
	 */
	if (txirq > 0) {
		ret = devm_request_irq(&pdev->dev, rxirq, wokoo_uart_rxint, 0,
						dev_name(&pdev->dev), sport);
		if (ret) {
			dev_err(&pdev->dev, "failed to request rx irq: %d\n",
				ret);
			return ret;
		}

		ret = devm_request_irq(&pdev->dev, txirq, wokoo_uart_txint, 0,
							dev_name(&pdev->dev), sport);
		if (ret) {
			dev_err(&pdev->dev, "failed to request tx irq: %d\n",
				ret);
			return ret;
		}

		ret = devm_request_irq(&pdev->dev, rtsirq, wokoo_uart_rtsint, 0,
				       dev_name(&pdev->dev), sport);
		if (ret) {
			dev_err(&pdev->dev, "failed to request rts irq: %d\n",
				ret);
			return ret;
		}
	} else {
		ret = devm_request_irq(&pdev->dev, rxirq, wokoo_uart_int, 0,
						dev_name(&pdev->dev), sport);
		if (ret) {
			dev_err(&pdev->dev, "failed to request irq: %d\n", ret);
			return ret;
		}
	}

	device_init_wakeup(&pdev->dev, true);
	enable_irq_wake(rxirq);
	wokoo_uart_ports[sport->port.line] = sport;

	platform_set_drvdata(pdev, sport);
	ret = uart_add_one_port(&wokoo_uart_uart_driver, &sport->port);
	clk_disable_unprepare(sport->clk_pclk);

	return ret;
}

static int wokoo_uart_remove(struct platform_device *pdev)
{
	struct wokoo_port *sport = platform_get_drvdata(pdev);

	return uart_remove_one_port(&wokoo_uart_uart_driver, &sport->port);
}

#if 0
static void wokoo_uart_restore_context(struct wokoo_port *sport)
{
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	if (!sport->context_saved) {
		spin_unlock_irqrestore(&sport->port.lock, flags);
		return;
	}

	wokoo_uart_writel(sport, sport->saved_reg[0], WOKOO_UART_IER);
	wokoo_uart_writel(sport, sport->saved_reg[1], WOKOO_UART_DLL);
	wokoo_uart_writel(sport, sport->saved_reg[2], WOKOO_UART_DLH);
	wokoo_uart_writel(sport, sport->saved_reg[3], WOKOO_UART_FCR);
	wokoo_uart_writel(sport, sport->saved_reg[4], WOKOO_UART_TCR);
	wokoo_uart_writel(sport, sport->saved_reg[5], WOKOO_UART_TSR);

	sport->context_saved = false;
	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void wokoo_uart_save_context(struct wokoo_port *sport)
{
	unsigned long flags;

	/* Save necessary regs */
	spin_lock_irqsave(&sport->port.lock, flags);
	sport->saved_reg[0] = wokoo_uart_readl(sport, WOKOO_UART_IER);
	sport->saved_reg[1] = wokoo_uart_readl(sport, WOKOO_UART_DLL);
	sport->saved_reg[2] = wokoo_uart_readl(sport, WOKOO_UART_DLH);
	sport->saved_reg[3] = wokoo_uart_readl(sport, WOKOO_UART_FCR);
	sport->saved_reg[4] = wokoo_uart_readl(sport, WOKOO_UART_TCR);
	sport->saved_reg[5] = wokoo_uart_readl(sport, WOKOO_UART_TSR);
	sport->context_saved = true;
	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static int wokoo_uart_suspend_noirq(struct device *dev)
{
	struct wokoo_port *sport = dev_get_drvdata(dev);

	wokoo_uart_save_context(sport);

	clk_disable(sport->clk_ipg);

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int wokoo_uart_resume_noirq(struct device *dev)
{
	struct wokoo_port *sport = dev_get_drvdata(dev);
	int ret;

	pinctrl_pm_select_default_state(dev);

	ret = clk_enable(sport->clk_ipg);
	if (ret)
		return ret;

	wokoo_uart_restore_context(sport);

	return 0;
}
#endif

#if CONFIG_PM
static int wokoo_uart_suspend(struct device *dev)
{
	struct wokoo_port *sport = dev_get_drvdata(dev);

	enable_irq_wake(sport->port.irq);

	return 0;
}

static int wokoo_uart_resume(struct device *dev)
{
	struct wokoo_port *sport = dev_get_drvdata(dev);

	disable_irq_wake(sport->port.irq);

	return 0;
}

#if 0
static int wokoo_uart_freeze(struct device *dev)
{
	struct wokoo_port *sport = dev_get_drvdata(dev);

	uart_suspend_port(&wokoo_uart_uart_driver, &sport->port);

	return clk_prepare_enable(sport->clk_pclk);
}

static int wokoo_uart_thaw(struct device *dev)
{
	struct wokoo_port *sport = dev_get_drvdata(dev);

	uart_resume_port(&wokoo_uart_uart_driver, &sport->port);

	clk_disable_unprepare(sport->clk_pclk);

	return 0;
}
#endif

static const struct dev_pm_ops wokoo_uart_pm_ops = {
	.suspend = wokoo_uart_suspend,
	.resume = wokoo_uart_resume,
#if 0
	.freeze = wokoo_uart_freeze,
	.thaw = wokoo_uart_thaw,
	.restore = wokoo_uart_thaw,
#endif
};
#endif

static const struct of_device_id wokoo_uart_dt_ids[] = {
	{ .compatible = "wokoo,wokoo-uart"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wokoo_uart_dt_ids);

static struct platform_driver wokoo_uart_platform_driver = {
	.probe = wokoo_uart_probe,
	.remove = wokoo_uart_remove,

	.driver = {
		.name = "wokoo-uart",
		.of_match_table = wokoo_uart_dt_ids,
#if CONFIG_PM
		.pm = &wokoo_uart_pm_ops,
#endif
	},
};

static int __init wokoo_uart_init(void)
{
	int ret = uart_register_driver(&wokoo_uart_uart_driver);

	if (ret)
		return ret;

	ret = platform_driver_register(&wokoo_uart_platform_driver);
	if (ret != 0)
		uart_unregister_driver(&wokoo_uart_uart_driver);

	return ret;
}

static void __exit wokoo_uart_exit(void)
{
	platform_driver_unregister(&wokoo_uart_platform_driver);
	uart_unregister_driver(&wokoo_uart_uart_driver);
}

module_init(wokoo_uart_init);
module_exit(wokoo_uart_exit);

MODULE_ALIAS("platform:wokoo-uart");
MODULE_DESCRIPTION("Wokoo SoC Serial port driver");
MODULE_AUTHOR("GuangHuang Lin<linguanghuang@codefairsemi.com >");
MODULE_LICENSE("GPL v2");
