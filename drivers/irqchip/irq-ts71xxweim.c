// SPDX-License-Identifier: GPL-2.0

#include <linux/types.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_device.h>
#include <linux/seq_file.h>

#define TSWEIM_IRQ_ACK		0x20
#define TSWEIM_IRQ_STATUS	0x24
#define TSWEIM_IRQ_POLARITY	0x28
#define TSWEIM_IRQ_ACK_MODE	0x2C
#define TSWEIM_IRQ_ACK_MODE_EN	BIT(0)
#define TSWEIM_IRQ_MASK		0x48
#define TSWEIM_NUM_FPGA_IRQ	32

#define TSWEIM_IRQ_SUPPORTED_FPGA_REV 70

struct tsweim_intc {
	void __iomem  *syscon;
	struct irq_domain *irqdomain;
	struct platform_device *pdev;
	raw_spinlock_t lock;
	u32 mask;
	bool ack_mode_en;
};

static void tsweim_intc_mask(struct irq_data *d)
{
	struct tsweim_intc *priv = irq_data_get_irq_chip_data(d);

	raw_spin_lock(&priv->lock);

	priv->mask = readl(priv->syscon + TSWEIM_IRQ_MASK) & ~BIT(d->hwirq);
	writel(priv->mask, priv->syscon + TSWEIM_IRQ_MASK);

	raw_spin_unlock(&priv->lock);
}

static void tsweim_intc_unmask(struct irq_data *d)
{
	struct tsweim_intc *priv = irq_data_get_irq_chip_data(d);

	raw_spin_lock(&priv->lock);

	priv->mask = readl(priv->syscon + TSWEIM_IRQ_MASK) | BIT(d->hwirq);
	writel(priv->mask, priv->syscon + TSWEIM_IRQ_MASK);

	raw_spin_unlock(&priv->lock);
}

static void tsweim_intc_print_chip(struct irq_data *d, struct seq_file *p)
{
	struct tsweim_intc *priv = irq_data_get_irq_chip_data(d);

	seq_printf(p, "%s", dev_name(&priv->pdev->dev));
}

static void tsweim_intc_irq_ack(struct irq_data *d)
{
	struct tsweim_intc *priv = irq_data_get_irq_chip_data(d);

	if (priv->ack_mode_en) {
		writel(BIT(d->hwirq), priv->syscon + TSWEIM_IRQ_ACK);
	} else {
		readl(priv->syscon + TSWEIM_IRQ_STATUS);
	}
}

static void tsweim_intc_irq_mask_ack(struct irq_data *d)
{
	struct tsweim_intc *priv = irq_data_get_irq_chip_data(d);

	raw_spin_lock(&priv->lock);

	priv->mask = readl(priv->syscon + TSWEIM_IRQ_MASK) & ~BIT(d->hwirq);
	writel(priv->mask, priv->syscon + TSWEIM_IRQ_MASK);

	raw_spin_unlock(&priv->lock);

	if (priv->ack_mode_en) {
		writel(BIT(d->hwirq), priv->syscon + TSWEIM_IRQ_ACK);
	} else {
		readl(priv->syscon + TSWEIM_IRQ_STATUS);
	}
}

static int tsweim_intc_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct tsweim_intc *priv = irq_data_get_irq_chip_data(d);
	u32 polarity, bit;

	raw_spin_lock(&priv->lock);

	polarity = readl(priv->syscon + TSWEIM_IRQ_POLARITY);
	bit = BIT(d->hwirq);

	switch (flow_type) {
	case IRQ_TYPE_LEVEL_LOW:
		polarity |= bit;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		polarity &= ~bit;
		break;
	default:
		raw_spin_unlock(&priv->lock);
		return -EINVAL;
	}

	writel(polarity, priv->syscon + TSWEIM_IRQ_POLARITY);

	raw_spin_unlock(&priv->lock);

	return 0;
}

static struct irq_chip tsweim_intc_chip = {
	.irq_mask	= tsweim_intc_mask,
	.irq_unmask	= tsweim_intc_unmask,
	.irq_print_chip	= tsweim_intc_print_chip,
	.irq_ack	= tsweim_intc_irq_ack,
	.irq_mask_ack	= tsweim_intc_irq_mask_ack,
};

static void tsweim_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct tsweim_intc *priv = irq_desc_get_handler_data(desc);
	unsigned long irq, status;

	chained_irq_enter(chip, desc);

	status = readl(priv->syscon + TSWEIM_IRQ_STATUS);

	/* before the ack mode functionality the mask was not applied in hardware
	 * requiring the mask to be applied in software */
	if (!priv->ack_mode_en)
		status &= priv->mask;

	for_each_set_bit(irq, &status, TSWEIM_NUM_FPGA_IRQ) {
		generic_handle_domain_irq(priv->irqdomain, irq);
	}

	chained_irq_exit(chip, desc);
}

static int tsweim_intc_irqdomain_map(struct irq_domain *d,
		unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &tsweim_intc_chip, handle_level_irq);
	irq_set_chip_data(irq, d->host_data);
	irq_clear_status_flags(irq, IRQ_NOREQUEST | IRQ_NOPROBE);
	irq_set_status_flags(irq, IRQ_LEVEL);

	return 0;
}

static const struct irq_domain_ops tsweim_intc_irqdomain_ops = {
	.map = tsweim_intc_irqdomain_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static int tsweim_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tsweim_intc *priv;
	int irq = 0;
	bool ack_mode_en;
	u32 revision;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	priv = devm_kzalloc(dev, sizeof(struct tsweim_intc), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->syscon = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->syscon))
		return PTR_ERR(priv->syscon);

	priv->pdev = pdev;

	raw_spin_lock_init(&priv->lock);

	priv->mask = readl(priv->syscon + TSWEIM_IRQ_MASK);

	if (of_property_read_bool(dev->of_node, "ts,haspolarity"))
		tsweim_intc_chip.irq_set_type = tsweim_intc_set_type;

	priv->irqdomain = irq_domain_add_linear(dev->of_node,
		TSWEIM_NUM_FPGA_IRQ, &tsweim_intc_irqdomain_ops, priv);
	if (!priv->irqdomain) {
		pr_err("unable to add irq domain\n");
		return -ENOMEM;
	}

	if (devm_request_irq(dev, irq, no_action, IRQF_NO_THREAD,
			     dev_name(dev), NULL)) {
		irq_domain_remove(priv->irqdomain);
		return -ENOENT;
	}

	irq_set_chained_handler_and_data(irq, tsweim_irq_handler, priv);

	platform_set_drvdata(pdev, priv);

	revision = readl(priv->syscon) & 0xFF;
	ack_mode_en = readl(priv->syscon + TSWEIM_IRQ_ACK_MODE) & TSWEIM_IRQ_ACK_MODE_EN;

	if (revision >= TSWEIM_IRQ_SUPPORTED_FPGA_REV && !ack_mode_en) {
		writel(TSWEIM_IRQ_ACK_MODE_EN, priv->syscon + TSWEIM_IRQ_ACK_MODE);

		ack_mode_en = readl(priv->syscon + TSWEIM_IRQ_ACK_MODE) & TSWEIM_IRQ_ACK_MODE_EN;
		if (ack_mode_en) {
			dev_info(dev, "ACK mode enabled with FPGA rev%u\n",
				revision);
			priv->ack_mode_en = true;
		} else {
			dev_warn(dev, "failed to enable ACK mode with FPGA rev%u\n",
				revision);
		}
	}

	return 0;
}

static int tsweim_intc_remove(struct platform_device *pdev)
{
	struct tsweim_intc *priv = platform_get_drvdata(pdev);

	if (priv->irqdomain) {
		int i, irq;

		for (i = 0; i < TSWEIM_NUM_FPGA_IRQ; i++) {
			irq = irq_find_mapping(priv->irqdomain, i);
			if (irq > 0)
				irq_dispose_mapping(irq);
		}
		irq_domain_remove(priv->irqdomain);
		priv->irqdomain = NULL;
	}

	return 0;
}

static const struct of_device_id tsweim_intc_of_match_table[] = {
	{.compatible = "technologic,ts71xxweim-intc", },
	{},
};
MODULE_DEVICE_TABLE(of, tsweim_intc_of_match_table);

static struct platform_driver tsweim_intc_driver = {
	.driver = {
		.name = "tsweim-intc",
		.of_match_table = of_match_ptr(tsweim_intc_of_match_table),
	},
	.probe = tsweim_intc_probe,
	.remove = tsweim_intc_remove,
};
module_platform_driver(tsweim_intc_driver);

MODULE_AUTHOR("embeddedTS");
MODULE_DESCRIPTION("Interrupt Controller for embeddedTS FPGA platforms connected with WEIM");
MODULE_LICENSE("GPL");
