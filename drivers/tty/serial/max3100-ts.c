// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2008 Christian Pellegrin <chripell@evolware.org>
 *
 * Notes: the MAX3100 doesn't provide an interrupt on CTS so we have
 * to use polling for flow control. TX empty IRQ is unusable, since
 * writing conf clears FIFO buffer and we cannot have this interrupt
 * always asking us for attention.
 *
 * The initial minor number is 209 in the low-density serial port:
 * mknod /dev/ttyMAX0 c 204 209
 *
 * This driver is an offshoot of the base driver. It is designed for
 * FPGA IP in some embeddedTS platforms that can fit up to 64 emulated
 * MAX3100 devices under a single chip-select. This has some added
 * overhead to communications to select which UART is being used. There
 * are also changes to this driver to support multiple devices in a
 * different way than the base driver needs to support.
 */

#define MAX3100_MAJOR 204
#define MAX3100_MINOR 209
/* While the hardware can support up to 64 emulated MAX3100 devices, in practice
 * that many would not be functional and implementations never exceeded 3 of them.
 * Because of that, the original driver's max of 4 is plenty for us.
 */
/* 4 MAX3100s should be enough for everyone */
#define MAX_MAX3100 4

#include <linux/bitops.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/tty_flip.h>
#include <linux/tty.h>
#include <linux/types.h>

#include <linux/unaligned.h>

#define MAX3100_C    (1<<14)
#define MAX3100_D    (0<<14)
#define MAX3100_W    (1<<15)
#define MAX3100_RX   (0<<15)

#define MAX3100_WC   (MAX3100_W  | MAX3100_C)
#define MAX3100_RC   (MAX3100_RX | MAX3100_C)
#define MAX3100_WD   (MAX3100_W  | MAX3100_D)
#define MAX3100_RD   (MAX3100_RX | MAX3100_D)
#define MAX3100_CMD  (3 << 14)

#define MAX3100_T    (1<<14)
#define MAX3100_R    (1<<15)

#define MAX3100_FEN  (1<<13)
#define MAX3100_SHDN (1<<12)
#define MAX3100_TM   (1<<11)
#define MAX3100_RM   (1<<10)
#define MAX3100_PM   (1<<9)
#define MAX3100_RAM  (1<<8)
#define MAX3100_IR   (1<<7)
#define MAX3100_ST   (1<<6)
#define MAX3100_PE   (1<<5)
#define MAX3100_L    (1<<4)
#define MAX3100_BAUD (0xf)

#define MAX3100_TE   (1<<10)
#define MAX3100_RAFE (1<<10)
#define MAX3100_RTS  (1<<9)
#define MAX3100_CTS  (1<<9)
#define MAX3100_PT   (1<<8)
#define MAX3100_DATA (0xff)

#define MAX3100_RT   (MAX3100_R | MAX3100_T)
#define MAX3100_RTC  (MAX3100_RT | MAX3100_CTS | MAX3100_RAFE)

/* the following simulate a status reg for ignore_status_mask */
#define MAX3100_STATUS_PE 1
#define MAX3100_STATUS_FE 2
#define MAX3100_STATUS_OE 4

/* Needed to set the CS index */
#define MAX3100_CSI 0xc0

struct max3100_port {
	struct uart_port port;
	struct spi_device *spi;

	int cts;	        /* last CTS received for flow ctrl */
	int tx_empty;		/* last TX empty bit */

	spinlock_t conf_lock;	/* shared data */
	int conf_commit;	/* need to make changes */
	int conf;		/* configuration for the MAX31000
				 * (bits 0-7, bits 8-11 are irqs) */
	int rts_commit;	        /* need to change rts */
	int rts;		/* rts status */
	int baud;		/* current baud rate */

	int parity;		/* keeps track if we should send parity */
#define MAX3100_PARITY_ON 1
#define MAX3100_PARITY_ODD 2
#define MAX3100_7BIT 4
	int rx_enabled;	        /* if we should rx chars */

	int minor;		/* minor number */
	int loopback_commit;	/* need to change loopback */
	int loopback;		/* 1 if we are in loopback mode */

	/* for handling irqs: need workqueue since we do spi_sync */
	struct workqueue_struct *workqueue;
	struct work_struct work;
	/* set to 1 to make the workhandler exit as soon as possible */
	int  force_end_work;
	/* need to know we are suspending to avoid deadlock on workqueue */
	int suspending;

	struct timer_list	timer;
};

static struct max3100ts {
	/* portlock is used mostly to ensure that we can send complete SPI
	 * messages in max3100_sr() since communication with a different port
	 * than the previous message would need to have a preamble to switch
	 * the port. This could be handled by instead using the spi lock, or,
	 * reworking the max3100_sr() routines to be able to write these
	 * back to back transactions with a single call to spi_sync().
	 */
	struct mutex portlock;
	struct max3100_port *max3100s[MAX_MAX3100];
	int irq;
	int idx_q;		/* Last UART port/index we interacted with */
	int cnt;		/* Number of UARTs */
} max3100ts;


static inline struct max3100_port *to_max3100_port(struct uart_port *port)
{
	return container_of(port, struct max3100_port, port);
}

/* This lock is no longer really necessary with the modified driver, but, also
 * is not a hindrance as it is only used on probe.
 */
static DEFINE_MUTEX(max3100s_lock);               /* race on probe */

static int max3100_do_parity(struct max3100_port *s, u16 c)
{
	int parity;

	if (s->parity & MAX3100_PARITY_ODD)
		parity = 1;
	else
		parity = 0;

	if (s->parity & MAX3100_7BIT)
		c &= 0x7f;
	else
		c &= 0xff;

	parity = parity ^ parity8(c);
	return parity;
}

static int max3100_check_parity(struct max3100_port *s, u16 c)
{
	return max3100_do_parity(s, c) == ((c >> 8) & 1);
}

static void max3100_calc_parity(struct max3100_port *s, u16 *c)
{
	if (s->parity & MAX3100_7BIT)
		*c &= 0x7f;
	else
		*c &= 0xff;

	if (s->parity & MAX3100_PARITY_ON)
		*c |= max3100_do_parity(s, *c) << 8;
}

static int max3100_sr(struct max3100_port *s, u16 tx, u16 *rx)
{
	struct spi_message message;
	__be16 etx, erx;
	int status;
	struct spi_transfer tran = {
		.tx_buf = &etx,
		.rx_buf = &erx,
		.len = 2,
	};
	struct spi_transfer csidx = {
		.rx_buf = NULL,
		.len = 1,
	};

	/* If the last port we interacted with differs from the port we're
	 * about to interact with, preface the message with a command to
	 * the SPI peripheral that we are changing the index.
	 */
	if (max3100ts.idx_q != s->minor) {
		u8 cs = (s->minor | MAX3100_CSI);
		csidx.tx_buf = &cs;
		spi_message_init(&message);
		spi_message_add_tail(&csidx, &message);
		status = spi_sync(s->spi, &message);
		if (status) {
			dev_warn(&s->spi->dev, "error setting csidx\n");
			return status;
		}
		max3100ts.idx_q = s->minor;
	}

	etx = cpu_to_be16(tx);
	spi_message_init(&message);
	spi_message_add_tail(&tran, &message);
	status = spi_sync(s->spi, &message);
	if (status) {
		dev_warn(&s->spi->dev, "error while calling spi_sync\n");
		return -EIO;
	}
	*rx = be16_to_cpu(erx);
	s->tx_empty = (*rx & MAX3100_T) > 0;
	dev_dbg(&s->spi->dev, "%04x - %04x\n", tx, *rx);
	return 0;
}

static int max3100_handlerx_unlocked(struct max3100_port *s, u16 rx)
{
	unsigned int status = 0;
	int ret = 0, cts;
	u8 ch, flg;

	if (rx & MAX3100_R && s->rx_enabled) {
		dev_dbg(&s->spi->dev, "%s\n", __func__);
		ch = rx & (s->parity & MAX3100_7BIT ? 0x7f : 0xff);
		if (rx & MAX3100_RAFE) {
			s->port.icount.frame++;
			flg = TTY_FRAME;
			status |= MAX3100_STATUS_FE;
		} else {
			if (s->parity & MAX3100_PARITY_ON) {
				if (max3100_check_parity(s, rx)) {
					s->port.icount.rx++;
					flg = TTY_NORMAL;
				} else {
					s->port.icount.parity++;
					flg = TTY_PARITY;
					status |= MAX3100_STATUS_PE;
				}
			} else {
				s->port.icount.rx++;
				flg = TTY_NORMAL;
			}
		}
		uart_insert_char(&s->port, status, MAX3100_STATUS_OE, ch, flg);
		ret = 1;
	}

	cts = (rx & MAX3100_CTS) > 0;
	if (s->cts != cts) {
		s->cts = cts;
		uart_handle_cts_change(&s->port, cts);
	}

	return ret;
}

static int max3100_handlerx(struct max3100_port *s, u16 rx)
{
	unsigned long flags;
	int ret;

	uart_port_lock_irqsave(&s->port, &flags);
	ret = max3100_handlerx_unlocked(s, rx);
	uart_port_unlock_irqrestore(&s->port, flags);
	return ret;
}

/* The prototype of this function needs to change since we need to be able to
 * call this directly from an interrupt context.
 *
 * Since we have to poll every port, and an interrupt doesn't clear until all
 * interrupt sources are ACKed; we need to run this against every port to clear
 * the interrupts. Since this is normally a workqueue, this creates an irq storm
 * since it would queue all three ports in the queue, tell the kernel the IRQ was
 * handled, but it wouldn't be and the interrupt routine would fire again, re-queue,
 * and start giving bad data or wasting too much time being in an interrupt.
 *
 * This function can still be wq'ed, however, for things like changing config
 * options, baud, etc, that doesn't need to be run in an interrupt context.
 * In order to be okay with that, the max3100_work() function was also modified
 * to get the driver structure from the work structure.
 */
static void max3100_work_unlocked(struct max3100_port *s)
{
	struct tty_port *tport = &s->port.state->port;
	unsigned char ch;
	int conf, cconf, cloopback, loopback, crts;
	int rxchars;
	u16 tx, rx;
	unsigned long flags;
	int txcnt = 0;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	rxchars = 0;
	do {
		spin_lock(&s->conf_lock);
		conf = s->conf;
		cconf = s->conf_commit;
		s->conf_commit = 0;
		cloopback = s->loopback_commit;
		loopback = s->loopback;
		s->loopback_commit = 0;
		crts = s->rts_commit;
		s->rts_commit = 0;
		spin_unlock(&s->conf_lock);

		if (cconf)
			max3100_sr(s, MAX3100_WC | conf, &rx);
		if (cloopback)
			max3100_sr(s, 0x4000 | loopback, &rx);
		if (crts) {
			max3100_sr(s, MAX3100_WD | MAX3100_TE |
				   (s->rts ? MAX3100_RTS : 0), &rx);
			rxchars += max3100_handlerx(s, rx);
		}

		max3100_sr(s, MAX3100_RD, &rx);
		rxchars += max3100_handlerx(s, rx);

		if (rx & MAX3100_T) {
			tx = 0xffff;
			uart_port_lock_irqsave(&s->port, &flags);
			if (s->port.x_char) {
				tx = s->port.x_char;
				s->port.icount.tx++;
				s->port.x_char = 0;
			} else if (!uart_tx_stopped(&s->port) &&
					uart_fifo_get(&s->port, &ch)) {
				tx = ch;
			}
			uart_port_unlock_irqrestore(&s->port, flags);

			if (tx != 0xffff) {
				max3100_calc_parity(s, &tx);
				tx |= MAX3100_WD | (s->rts ? MAX3100_RTS : 0);
				max3100_sr(s, tx, &rx);
				rxchars += max3100_handlerx(s, rx);
				txcnt++;
			}
		}

		if (rxchars > 16) {
			tty_flip_buffer_push(&s->port.state->port);
			rxchars = 0;
		}
		if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS) {
			uart_port_lock_irqsave(&s->port, &flags);
			uart_write_wakeup(&s->port);
			uart_port_unlock_irqrestore(&s->port, flags);
		}

		/* Only transmit a maximum of 10 bytes. This prevents any one port
		 * that is slamming out TX data from hogging all of the bandwidth.
		 * If we need to transmit 1 byte, but the RX FIFO is full (and filling
		 * further), we still will cleanly pull all of that data out until
		 * both buffers are empty. But this voluntary preemption ensures
		 * that a hammering port won't cause dropped data elsewhere.
		 */
		if (txcnt > 10)
			break;

	} while (!s->force_end_work &&
		 !freezing(current) &&
		 ((rx & MAX3100_R) ||
		  (!kfifo_is_empty(&tport->xmit_fifo) &&
		   !uart_tx_stopped(&s->port))));

	if (rxchars > 0)
		tty_flip_buffer_push(&s->port.state->port);
}

static void max3100_work(struct work_struct *w)
{
	struct max3100_port *s = container_of(w, struct max3100_port, work);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
	mutex_lock(&max3100ts.portlock);
	max3100_work_unlocked(s);
	mutex_unlock(&max3100ts.portlock);
}

static void max3100_dowork(struct max3100_port *s)
{
	if (!s->force_end_work && !freezing(current) && !s->suspending)
		queue_work(s->workqueue, &s->work);
}

static void max3100_timeout(struct timer_list *t)
{
	struct max3100_port *s = timer_container_of(s, t, timer);

	max3100_dowork(s);
	mod_timer(&s->timer, jiffies + uart_poll_timeout(&s->port));
}

/* Threaded handler called in a sleepable context */
static irqreturn_t max3100_thread_irq(int irqno, void *dev_id)
{
	int i;

	/* We can't know what the IRQ source actually was until we start
	 * checking each port. However, for ports that are not yet open,
	 * there is not a valid workqueue, so only do work on ports that
	 * have a valid workqueue pointer. 
	 */
	for (i = 0; i < max3100ts.cnt; i++) {
		if (max3100ts.max3100s[i]->workqueue) {
			mutex_lock(&max3100ts.portlock);
			max3100_work_unlocked(max3100ts.max3100s[i]);
			mutex_unlock(&max3100ts.portlock);
		}
	}

	return IRQ_HANDLED;
}

static void max3100_enable_ms(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	mod_timer(&s->timer, jiffies);
	dev_dbg(&s->spi->dev, "%s\n", __func__);
}

static void max3100_start_tx(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	max3100_dowork(s);
}

static void max3100_stop_rx(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	s->rx_enabled = 0;
	spin_lock(&s->conf_lock);
	s->conf &= ~MAX3100_RM;
	s->conf_commit = 1;
	spin_unlock(&s->conf_lock);
	max3100_dowork(s);
}

static unsigned int max3100_tx_empty(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	/* may not be truly up-to-date */
	max3100_dowork(s);
	return s->tx_empty;
}

static unsigned int max3100_get_mctrl(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	/* may not be truly up-to-date */
	max3100_dowork(s);
	/* always assert DCD and DSR since these lines are not wired */
	return (s->cts ? TIOCM_CTS : 0) | TIOCM_DSR | TIOCM_CAR;
}

static void max3100_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct max3100_port *s = to_max3100_port(port);
	int loopback, rts;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	loopback = (mctrl & TIOCM_LOOP) > 0;
	rts = (mctrl & TIOCM_RTS) > 0;

	spin_lock(&s->conf_lock);
	if (s->loopback != loopback) {
		s->loopback = loopback;
		s->loopback_commit = 1;
	}
	if (s->rts != rts) {
		s->rts = rts;
		s->rts_commit = 1;
	}
	if (s->loopback_commit || s->rts_commit)
		max3100_dowork(s);
	spin_unlock(&s->conf_lock);
}

static void
max3100_set_termios(struct uart_port *port, struct ktermios *termios,
		    const struct ktermios *old)
{
	struct max3100_port *s = to_max3100_port(port);
	unsigned int baud = port->uartclk / 16;
	unsigned int baud230400 = (baud == 230400) ? 1 : 0;
	unsigned cflag;
	u32 param_new, param_mask, parity = 0;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	cflag = termios->c_cflag;
	param_mask = 0;

	baud = tty_termios_baud_rate(termios);
	param_new = s->conf & MAX3100_BAUD;
	switch (baud) {
	case 300:
		if (baud230400)
			baud = s->baud;
		else
			param_new = 15;
		break;
	case 600:
		param_new = 14 + baud230400;
		break;
	case 1200:
		param_new = 13 + baud230400;
		break;
	case 2400:
		param_new = 12 + baud230400;
		break;
	case 4800:
		param_new = 11 + baud230400;
		break;
	case 9600:
		param_new = 10 + baud230400;
		break;
	case 19200:
		param_new = 9 + baud230400;
		break;
	case 38400:
		param_new = 8 + baud230400;
		break;
	case 57600:
		param_new = 1 + baud230400;
		break;
	case 115200:
		param_new = 0 + baud230400;
		break;
	case 230400:
		if (baud230400)
			param_new = 0;
		else
			baud = s->baud;
		break;
	default:
		baud = s->baud;
	}
	tty_termios_encode_baud_rate(termios, baud, baud);
	s->baud = baud;
	param_mask |= MAX3100_BAUD;

	if ((cflag & CSIZE) == CS8) {
		param_new &= ~MAX3100_L;
		parity &= ~MAX3100_7BIT;
	} else {
		param_new |= MAX3100_L;
		parity |= MAX3100_7BIT;
		cflag = (cflag & ~CSIZE) | CS7;
	}
	param_mask |= MAX3100_L;

	if (cflag & CSTOPB)
		param_new |= MAX3100_ST;
	else
		param_new &= ~MAX3100_ST;
	param_mask |= MAX3100_ST;

	if (cflag & PARENB) {
		param_new |= MAX3100_PE;
		parity |= MAX3100_PARITY_ON;
	} else {
		param_new &= ~MAX3100_PE;
		parity &= ~MAX3100_PARITY_ON;
	}
	param_mask |= MAX3100_PE;

	if (cflag & PARODD)
		parity |= MAX3100_PARITY_ODD;
	else
		parity &= ~MAX3100_PARITY_ODD;

	/* mask termios capabilities we don't support */
	cflag &= ~CMSPAR;
	termios->c_cflag = cflag;

	s->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		s->port.ignore_status_mask |=
			MAX3100_STATUS_PE | MAX3100_STATUS_FE |
			MAX3100_STATUS_OE;

	timer_delete_sync(&s->timer);
	uart_update_timeout(port, termios->c_cflag, baud);

	spin_lock(&s->conf_lock);
	s->conf = (s->conf & ~param_mask) | (param_new & param_mask);
	s->conf_commit = 1;
	s->parity = parity;
	spin_unlock(&s->conf_lock);
	max3100_dowork(s);

	if (UART_ENABLE_MS(&s->port, termios->c_cflag))
		max3100_enable_ms(&s->port);
}

static void max3100_shutdown(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);
	u16 rx;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	if (s->suspending)
		return;

	s->force_end_work = 1;
	/* Cycle the lock to ensure any active work is completed */
	mutex_lock(&max3100ts.portlock);
	mutex_unlock(&max3100ts.portlock);

	timer_delete_sync(&s->timer);

	if (s->workqueue) {
		/* Flush the workqueue since we add delayed work */
		flush_workqueue(s->workqueue);
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
	}

	/* set shutdown mode to save power */
	mutex_lock(&max3100ts.portlock);
	max3100_sr(s, MAX3100_WC | MAX3100_SHDN, &rx);
	mutex_unlock(&max3100ts.portlock);
}

static int max3100_startup(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);
	char b[12];

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	s->conf = MAX3100_RM | MAX3100_TM;
	s->baud = port->uartclk / 16;
	s->rx_enabled = 1;

	if (s->suspending)
		return 0;

	s->force_end_work = 0;
	s->parity = 0;
	s->rts = 0;

	sprintf(b, "max3100-%d", s->minor);
	s->workqueue = create_freezable_workqueue(b);
	if (!s->workqueue) {
		dev_warn(&s->spi->dev, "cannot create workqueue\n");
		return -EBUSY;
	}
	INIT_WORK(&s->work, max3100_work);

	s->conf_commit = 1;
	max3100_dowork(s);
	/* wait for clock to settle */
	if (s->port.line == 0)
		msleep(50);

	max3100_enable_ms(&s->port);

	return 0;
}

static const char *max3100_type(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	return s->port.type == PORT_MAX3100 ? "MAX3100" : NULL;
}

static void max3100_release_port(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
}

static void max3100_config_port(struct uart_port *port, int flags)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	if (flags & UART_CONFIG_TYPE)
		s->port.type = PORT_MAX3100;
}

static int max3100_verify_port(struct uart_port *port,
			       struct serial_struct *ser)
{
	struct max3100_port *s = to_max3100_port(port);
	int ret = -EINVAL;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	if (ser->type == PORT_UNKNOWN || ser->type == PORT_MAX3100)
		ret = 0;
	return ret;
}

static void max3100_stop_tx(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
}

static int max3100_request_port(struct uart_port *port)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
	return 0;
}

static void max3100_break_ctl(struct uart_port *port, int break_state)
{
	struct max3100_port *s = to_max3100_port(port);

	dev_dbg(&s->spi->dev, "%s\n", __func__);
}

static const struct uart_ops max3100_ops = {
	.tx_empty	= max3100_tx_empty,
	.set_mctrl	= max3100_set_mctrl,
	.get_mctrl	= max3100_get_mctrl,
	.stop_tx        = max3100_stop_tx,
	.start_tx	= max3100_start_tx,
	.stop_rx	= max3100_stop_rx,
	.enable_ms      = max3100_enable_ms,
	.break_ctl      = max3100_break_ctl,
	.startup	= max3100_startup,
	.shutdown	= max3100_shutdown,
	.set_termios	= max3100_set_termios,
	.type		= max3100_type,
	.release_port   = max3100_release_port,
	.request_port   = max3100_request_port,
	.config_port	= max3100_config_port,
	.verify_port	= max3100_verify_port,
};

static struct uart_driver max3100_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = "ttyMAX",
	.dev_name       = "ttyMAX",
	.major          = MAX3100_MAJOR,
	.minor          = MAX3100_MINOR,
	.nr             = MAX_MAX3100,
};
static int uart_driver_registered;

static int max3100_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int i, retval;
	u16 rx;

	mutex_init(&max3100ts.portlock);

	mutex_lock(&max3100s_lock);

	if (!uart_driver_registered) {
		retval = uart_register_driver(&max3100_uart_driver);
		if (retval) {
			mutex_unlock(&max3100s_lock);
			return dev_err_probe(dev, retval, "Couldn't register max3100 uart driver\n");
		}

		uart_driver_registered = 1;
	}

	max3100ts.irq = spi->irq;
	max3100ts.idx_q = -1;
	max3100ts.cnt = 0;


	for (i = 0; i < MAX_MAX3100; i++) {
		max3100ts.max3100s[i] = devm_kzalloc(dev,
						     sizeof(struct max3100_port),
						     GFP_KERNEL);
		if (!max3100ts.max3100s[i]) {
			mutex_unlock(&max3100s_lock);
			return -ENOMEM;
		}

		spin_lock_init(&max3100ts.max3100s[i]->conf_lock);
		max3100ts.max3100s[i]->spi = spi;
		/* Note that the modifications to this driver basically discard
		 * any use of spi_get_drvdata() due to the shared struct. This
		 * could probably be cleaned up to not have the global max3100ts
		 * struct and make it more inline with kernel paradigms, but that
		 * would create more diffs from the base driver. So for now, we just
		 * set_drvdata with dummy data and don't use it anywhere.
		 */
		spi_set_drvdata(spi, max3100ts.max3100s[i]);
		max3100ts.max3100s[i]->minor = i;

		/* Before we continue, ensure the "port" actually exists in the
		 * FPGA. This is done by attempting to shutdown the port and write
		 * a valid baud rate; then doing a read configuration of the
		 * chip and checking that the baud has a value of 5. If not, then
		 * there is no port there and we should move on with initialization
		 * of the driver. If there is, configure it, add it to the system,
		 * and repeat for the next port.
		 */
		mutex_lock(&max3100ts.portlock);
		max3100_sr(max3100ts.max3100s[i], MAX3100_WC | MAX3100_SHDN | 5, &rx);
		max3100_sr(max3100ts.max3100s[i], MAX3100_RC, &rx);
		mutex_unlock(&max3100ts.portlock);
		if ((rx & MAX3100_BAUD) != 5) {
			kfree(max3100ts.max3100s[i]);
			max3100ts.max3100s[i] = NULL;
			break;
		} else {
			max3100ts.cnt++;
		}

		/* Continue with the per-port initialization. This is based on
		 * the original driver.
		 */
		max3100ts.max3100s[i]->spi = spi;

		timer_setup(&max3100ts.max3100s[i]->timer, max3100_timeout, 0);

		dev_dbg(&spi->dev, "%s: adding port %d\n", __func__, i);
		max3100ts.max3100s[i]->port.irq = max3100ts.irq;
		max3100ts.max3100s[i]->port.fifosize = 16;
		max3100ts.max3100s[i]->port.ops = &max3100_ops;
		max3100ts.max3100s[i]->port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
		max3100ts.max3100s[i]->port.line = i;
		max3100ts.max3100s[i]->port.type = PORT_MAX3100;
		max3100ts.max3100s[i]->port.dev = &spi->dev;

		/* Read clock frequency from a property, uart_add_one_port() will
		 * fail if it's not set */
		device_property_read_u32(dev, "clock-frequency",
					 &max3100ts.max3100s[i]->port.uartclk);

		retval = uart_add_one_port(&max3100_uart_driver,
					   &max3100ts.max3100s[i]->port);
		if (retval < 0)
			dev_warn(&spi->dev,
				 "uart_add_one_port failed for line %d with error %d\n",
				 i, retval);

		/* set shutdown mode to save power. Will be woken-up on open */
		mutex_lock(&max3100ts.portlock);
		max3100_sr(max3100ts.max3100s[i], MAX3100_WC | MAX3100_SHDN, &rx);
		mutex_unlock(&max3100ts.portlock);
	}

	/* This differs from the base driver in that we're not specifically
	 * taking advantage of threaded IRQ handling, but rather the sleepable
	 * context where we need to do work directly since we can't use the wq
	 * for interrupt handling.
	 */
	retval = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
					   max3100_thread_irq,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   "max3100-ts", &max3100ts);
	if (retval) {
		mutex_unlock(&max3100s_lock);
		return dev_err_probe(dev, retval, "Couldn't allocate IRQ %d\n", spi->irq);
	}

	dev_info(&spi->dev, "Detected %d UART ports\n", max3100ts.cnt);
	mutex_unlock(&max3100s_lock);

	return 0;
}

static void max3100_remove(struct spi_device *spi)
{
	struct max3100_port *s = spi_get_drvdata(spi);
	int i;

	mutex_lock(&max3100s_lock);

	/* find out the index for the chip we are removing */
	for (i = 0; i < MAX_MAX3100; i++)
		if (max3100ts.max3100s[i]) {
			s = max3100ts.max3100s[i];
			dev_dbg(&spi->dev, "%s: removing port %d\n", __func__, i);
			uart_remove_one_port(&max3100_uart_driver, &s->port);
			kfree(max3100ts.max3100s[i]);
			max3100ts.max3100s[i] = NULL;
		}

	if (max3100ts.irq) {
		free_irq(max3100ts.irq, &max3100ts);
		max3100ts.irq = 0;
	}

	pr_debug("removing max3100 driver\n");
	uart_unregister_driver(&max3100_uart_driver);
	uart_driver_registered = 0;

	mutex_unlock(&max3100s_lock);
}

static int max3100_suspend(struct device *dev)
{
	struct max3100_port *s = dev_get_drvdata(dev);
	u16 rx;
	int i;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	disable_irq(max3100ts.irq);

	for (i = 0; i < MAX_MAX3100; i++)
		if (max3100ts.max3100s[i]) {
			s = max3100ts.max3100s[i];
			s->suspending = 1;
			uart_suspend_port(&max3100_uart_driver, &s->port);

			/* no HW suspend, so do SW one */
			mutex_lock(&max3100ts.portlock);
			max3100_sr(s, MAX3100_WC | MAX3100_SHDN, &rx);
			mutex_unlock(&max3100ts.portlock);
		}

	return 0;
}

static int max3100_resume(struct device *dev)
{
	struct max3100_port *s = dev_get_drvdata(dev);
	int i;

	dev_dbg(&s->spi->dev, "%s\n", __func__);

	enable_irq(max3100ts.irq);

	for (i = 0; i < MAX_MAX3100; i++)
		if (max3100ts.max3100s[i]) {
			s = max3100ts.max3100s[i];
			uart_resume_port(&max3100_uart_driver, &s->port);
			s->suspending = 0;

			s->conf_commit = 1;
			if (s->workqueue)
				max3100_dowork(s);
		}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(max3100_pm_ops, max3100_suspend, max3100_resume);

static const struct spi_device_id max3100_spi_id[] = {
	{ "max3100-ts" },
	{ }
};
MODULE_DEVICE_TABLE(spi, max3100_spi_id);

static const struct of_device_id max3100_of_match[] = {
	{ .compatible = "technologic,max3100-ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, max3100_of_match);

static struct spi_driver max3100_driver = {
	.driver = {
		.name		= "max3100-ts",
		.of_match_table	= max3100_of_match,
		.pm		= pm_sleep_ptr(&max3100_pm_ops),
	},
	.probe		= max3100_probe,
	.remove		= max3100_remove,
	.id_table	= max3100_spi_id,
};

module_spi_driver(max3100_driver);

MODULE_DESCRIPTION("MAX3100 embeddedTS extended driver");
MODULE_AUTHOR("Christian Pellegrin <chripell@evolware.org>");
MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_AUTHOR("Kris Bahnsen <kris@embeddedTS.com>");
MODULE_LICENSE("GPL");
