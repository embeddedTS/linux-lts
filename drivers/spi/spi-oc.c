// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2007-2008 Avionic Design Development GmbH
 * Copyright (C) 2008-2009 Avionic Design GmbH
 * Written by Thierry Reding <thierry.reding@avionic-design.de>
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <asm/unaligned.h>

/* register definitions */
#define SPIOC_RX(i)	((i) * 4)
#define SPIOC_TX(i)	((i) * 4)
#define SPIOC_CTRL	0x10
#define SPIOC_DIV	0x14
#define SPIOC_SS	0x18
#define SPIOC_MAX_XFER_BYTES	16
#define SPIOC_MAX_CHIPSELECT	8
#define CTRL_LEN_MASK		0x7f

/* SPIOC_CTRL register */
#define CTRL_BUSY	BIT(8)
#define CTRL_RXNEG	BIT(9)
#define CTRL_TXNEG	BIT(10)
#define CTRL_LSB	BIT(11)
#define CTRL_IE		BIT(12)
#define CTRL_ASS	BIT(13)
#define CTRL_CPOL	BIT(14)
#define CTRL_CPHA	BIT(15)

/**
 * struct spioc - driver-specific context information
 * @ctlr:	SPI controller device
 * @refclk:	always-on input clock used as SPI rate reference
 * @base:	base of memory-mapped I/O
 * @transfer:	current transfer
 * @nx:		number of bytes sent/received for current transfer
 */
struct spioc {
	struct spi_controller *ctlr;
	struct clk *refclk;
	void __iomem *base;
	struct spi_transfer *transfer;
	unsigned long nx;
};

static inline u32 spioc_readl(struct spioc *spioc, unsigned long offset)
{
	return readl(spioc->base + offset);
}

static inline void spioc_writel(struct spioc *spioc, unsigned int offset,
				u32 value)
{
	writel(value, spioc->base + offset);
}

static inline u32 spioc_ctrl_len(u32 nbits)
{
	return nbits & CTRL_LEN_MASK;
}

static int spioc_clkdiv(const struct spioc *spioc, u32 speed_hz, u16 *clkdiv)
{
	unsigned long refclk_hz = clk_get_rate(spioc->refclk);

	if (!refclk_hz)
		return -EINVAL;

	if (!speed_hz) {
		*clkdiv = U16_MAX;
		return 0;
	}

	*clkdiv = clamp_val(DIV_ROUND_UP_ULL(refclk_hz, 2ULL * speed_hz) - 1,
			    0, U16_MAX);

	return 0;
}

static void spioc_chipselect(struct spioc *spioc, struct spi_device *spi)
{
	if (spi)
		spioc_writel(spioc, SPIOC_SS, BIT(spi->chip_select));
	else
		spioc_writel(spioc, SPIOC_SS, 0);
}

static int spioc_config(struct spioc *spioc, struct spi_device *spi,
			struct spi_transfer *transfer)
{
	u16 clkdiv;
	unsigned int speed_hz;
	unsigned int bits_per_word;
	u32 ctrl = spioc_readl(spioc, SPIOC_CTRL);
	int ret;

	if (ctrl & CTRL_BUSY)
		return -EBUSY;

	ctrl &= ~(CTRL_LEN_MASK | CTRL_BUSY | CTRL_IE | CTRL_ASS);

	bits_per_word = transfer && transfer->bits_per_word ?
			transfer->bits_per_word : spi->bits_per_word;
	if (!bits_per_word)
		bits_per_word = 8;
	if (bits_per_word != 8)
		return -EINVAL;

	if (spi->mode & SPI_LSB_FIRST)
		ctrl |= CTRL_LSB;
	else
		ctrl &= ~CTRL_LSB;

	ctrl &= ~(CTRL_RXNEG | CTRL_TXNEG | CTRL_CPOL | CTRL_CPHA);
	if (spi->mode & SPI_CPOL)
		ctrl |= CTRL_CPOL;
	if (spi->mode & SPI_CPHA)
		ctrl |= CTRL_CPHA;

	/*
	 * used on older versions of the controller without cpol/cpha, ignored
	 * on newer controllers
	 */
	if (spi->mode & SPI_CPHA)
		ctrl |= CTRL_RXNEG;
	else
		ctrl |= CTRL_TXNEG;

	speed_hz = transfer && transfer->speed_hz ?
		   transfer->speed_hz : spi->max_speed_hz;

	ret = spioc_clkdiv(spioc, speed_hz, &clkdiv);
	if (ret)
		return ret;

	spioc_writel(spioc, SPIOC_DIV, clkdiv);
	spioc_writel(spioc, SPIOC_CTRL, ctrl);

	return 0;
}

/*
 * The SPI core passes the inactive level here, so this controller selects
 * a device in the !enable case.
 */
static void spioc_set_cs(struct spi_device *spi, bool enable)
{
	struct spioc *spioc = spi_controller_get_devdata(spi->controller);

	if (enable)
		spioc_chipselect(spioc, NULL);
	else
		spioc_chipselect(spioc, spi);
}

/*
 * count is assumed to be less than or equal to the maximum number of bytes
 * that can be transferred in one go
 */
static void spioc_copy_tx(struct spioc *spioc, const void *src, size_t count)
{
	const u8 *buf = src;
	unsigned int reg = DIV_ROUND_UP(count, sizeof(u32));
	size_t head = count % sizeof(u32);

	if (head) {
		u32 val;

		switch (head) {
		case 1:
			val = *buf;
			break;
		case 2:
			val = get_unaligned_be16(buf);
			break;
		default:
			val = get_unaligned_be24(buf);
			break;
		}

		spioc_writel(spioc, SPIOC_TX(--reg), val);
		buf += head;
		count -= head;
	}

	while (count) {
		spioc_writel(spioc, SPIOC_TX(--reg), get_unaligned_be32(buf));
		buf += sizeof(u32);
		count -= sizeof(u32);
	}
}

static void spioc_copy_rx(struct spioc *spioc, void *dest, size_t count)
{
	u8 *buf = dest;
	unsigned int reg = DIV_ROUND_UP(count, sizeof(u32));
	size_t head = count % sizeof(u32);

	if (head) {
		u32 val = spioc_readl(spioc, SPIOC_RX(--reg));

		switch (head) {
		case 1:
			*buf = val;
			break;
		case 2:
			put_unaligned_be16(val, buf);
			break;
		default:
			put_unaligned_be24(val, buf);
			break;
		}

		buf += head;
		count -= head;
	}

	while (count) {
		put_unaligned_be32(spioc_readl(spioc, SPIOC_RX(--reg)), buf);
		buf += sizeof(u32);
		count -= sizeof(u32);
	}
}

static void spioc_start_transfer(struct spioc *spioc)
{
	struct spi_transfer *transfer = spioc->transfer;
	size_t rem;
	u32 ctrl;

	rem = min_t(size_t, transfer->len - spioc->nx, SPIOC_MAX_XFER_BYTES);
	spioc_copy_tx(spioc, (const u8 *)transfer->tx_buf + spioc->nx, rem);

	ctrl = spioc_readl(spioc, SPIOC_CTRL);
	ctrl &= ~CTRL_LEN_MASK;
	ctrl &= ~CTRL_ASS;
	ctrl |= CTRL_IE | spioc_ctrl_len(rem * 8);
	spioc_writel(spioc, SPIOC_CTRL, ctrl);

	ctrl |= CTRL_BUSY;
	spioc_writel(spioc, SPIOC_CTRL, ctrl);
}

static int spioc_setup(struct spi_device *spi)
{
	struct spioc *spioc = spi_controller_get_devdata(spi->controller);

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;
	else if (spi->bits_per_word != 8)
		return -EINVAL;

	return spioc_config(spioc, spi, NULL);
}

static int spioc_transfer_one(struct spi_controller *ctlr,
			      struct spi_device *spi,
			      struct spi_transfer *transfer)
{
	struct spioc *spioc = spi_controller_get_devdata(ctlr);
	int ret;

	if (WARN_ON(spioc->transfer))
		return -EBUSY;

	ret = spioc_config(spioc, spi, transfer);
	if (ret)
		return ret;

	spioc->transfer = transfer;
	spioc->nx = 0;
	spioc_start_transfer(spioc);

	return 1;
}

static void spioc_handle_err(struct spi_controller *ctlr,
			     struct spi_message *msg)
{
	struct spioc *spioc = spi_controller_get_devdata(ctlr);
	u32 ctrl = spioc_readl(spioc, SPIOC_CTRL);

	(void)msg;

	ctrl &= ~(CTRL_BUSY | CTRL_IE | CTRL_ASS);
	spioc_writel(spioc, SPIOC_CTRL, ctrl);
	spioc_chipselect(spioc, NULL);

	spioc->transfer = NULL;
	spioc->nx = 0;
}

static irqreturn_t spioc_interrupt(int irq, void *dev_id)
{
	struct spioc *spioc = (struct spioc *)dev_id;
	struct spi_transfer *transfer;
	size_t rem;
	u32 ctrl;

	transfer = spioc->transfer;
	if (WARN_ON_ONCE(!transfer))
		return IRQ_HANDLED;

	ctrl = spioc_readl(spioc, SPIOC_CTRL);
	if (WARN_ON_ONCE(ctrl & CTRL_BUSY))
		return IRQ_HANDLED;

	/* read data from registers */
	rem = min_t(size_t, transfer->len - spioc->nx, SPIOC_MAX_XFER_BYTES);
	if (transfer->rx_buf)
		spioc_copy_rx(spioc, transfer->rx_buf + spioc->nx, rem);
	spioc->nx += rem;

	if (spioc->nx < transfer->len) {
		spioc_start_transfer(spioc);
		return IRQ_HANDLED;
	}

	spioc->transfer = NULL;
	spioc->nx = 0;
	spi_finalize_current_transfer(spioc->ctlr);

	return IRQ_HANDLED;
}

static const struct of_device_id opencores_spi_match[] = {
	{ .compatible = "opencores,spi-oc" },
	{},
};
MODULE_DEVICE_TABLE(of, opencores_spi_match);

static int spioc_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct spioc *spioc;
	int irq, ret;
	u32 num_chipselect;

	if (device_property_read_u32(&pdev->dev, "num-cs", &num_chipselect))
		num_chipselect = 1;

	if (!num_chipselect) {
		dev_err(&pdev->dev, "invalid num-cs 0\n");
		return -EINVAL;
	}

	ctlr = devm_spi_alloc_master(&pdev->dev, sizeof(*spioc));
	if (!ctlr)
		return -ENOMEM;
	spioc = spi_controller_get_devdata(ctlr);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(&pdev->dev, irq, "failed to get IRQ\n");

	spioc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spioc->base))
		return dev_err_probe(&pdev->dev, PTR_ERR(spioc->base),
				     "failed to map registers\n");

	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->max_native_cs = SPIOC_MAX_CHIPSELECT;
	ctlr->flags = SPI_CONTROLLER_MUST_TX;
	ctlr->setup = spioc_setup;
	ctlr->set_cs = spioc_set_cs;
	ctlr->transfer_one = spioc_transfer_one;
	ctlr->handle_err = spioc_handle_err;
	ctlr->use_gpio_descriptors = true;
	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->num_chipselect = num_chipselect;
	spioc->ctlr = ctlr;

	/*
	 * The controller input clock is always on and only used as a divider
	 * reference for calculating SPI bus rates.
	 */
	spioc->refclk = devm_clk_get(&pdev->dev, "spi-oc-clk");
	if (IS_ERR(spioc->refclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(spioc->refclk),
				     "failed to get reference clock\n");

	ret = devm_request_irq(&pdev->dev, irq, spioc_interrupt, 0,
			       dev_name(&pdev->dev), spioc);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to request IRQ %d\n", irq);

	return devm_spi_register_controller(&pdev->dev, ctlr);
}

static struct platform_driver spioc_driver = {
	.probe = spioc_probe,
	.driver = {
		.name  = "spioc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(opencores_spi_match),
	},
};
module_platform_driver(spioc_driver);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("OpenCores SPI controller driver");
MODULE_LICENSE("GPL v2");
