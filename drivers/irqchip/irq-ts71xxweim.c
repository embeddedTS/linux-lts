// SPDX-License-Identifier: GPL-2.0

#include <linux/types.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_device.h>

#define TSWEIM_IRQ_STATUS	0x24
#define TSWEIM_IRQ_MASK		0x48
#define TSWEIM_NUM_FPGA_IRQ	32

struct tsweim_intc {
	void __iomem  *syscon;
	struct irq_domain *irqdomain;
	u32 mask;
};

static struct tsweim_intc *irq_data_to_priv(struct irq_data *data)
{
	return data->domain->host_data;
}

static const struct of_device_id tsweim_intc_of_match_table[] = {
	{.compatible = "technologic,ts71xxweim-intc", },
	{},
};
MODULE_DEVICE_TABLE(of, tsweim_intc_of_match_table);

static void tsweim_intc_mask(struct irq_data *d)
{
	struct tsweim_intc *priv = irq_data_to_priv(d);

	priv->mask = readl(priv->syscon + TSWEIM_IRQ_MASK) & ~BIT(d->hwirq);
	writel(priv->mask, priv->syscon + TSWEIM_IRQ_MASK);
}

static void tsweim_intc_unmask(struct irq_data *d)
{
	struct tsweim_intc *priv = irq_data_to_priv(d);

	priv->mask = readl(priv->syscon + TSWEIM_IRQ_MASK) | BIT(d->hwirq);
	writel(priv->mask, priv->syscon + TSWEIM_IRQ_MASK);
}

static void tsweim_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct tsweim_intc *priv = irq_desc_get_handler_data(desc);
	unsigned int irq;
	unsigned int status;

	chained_irq_enter(chip, desc);

	while ((status =
	  (priv->mask & readl(priv->syscon + TSWEIM_IRQ_STATUS)))) {
		irq = 0;
		do {
			if (status & 1) {
				generic_handle_irq(irq_linear_revmap(
				  priv->irqdomain, irq));
			}
			status >>= 1;
			irq++;
		} while (status);
	}

	chained_irq_exit(chip, desc);
}

static struct irq_chip tsweim_irq_chip = {
	.name		= "tsweim_intc",
	.irq_mask	= tsweim_intc_mask,
	.irq_unmask	= tsweim_intc_unmask,
};

static int tsweim_intc_irqdomain_map(struct irq_domain *d,
		unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &tsweim_irq_chip,
				 handle_level_irq);

	irq_clear_status_flags(irq, IRQ_NOREQUEST | IRQ_NOPROBE);
	irq_set_status_flags(irq, IRQ_LEVEL);

	return 0;
}

static const struct irq_domain_ops tsweim_intc_irqdomain_ops = {
	.map = tsweim_intc_irqdomain_map,
};

static int tsweim_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct device_node *np =  pdev->dev.of_node;
	struct tsweim_intc *priv;
	void __iomem  *membase;
	struct resource *res = 0;

	priv = devm_kzalloc(dev, sizeof(struct tsweim_intc), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	match = of_match_device(tsweim_intc_of_match_table, dev);
	if (!match)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL) {
		pr_err("Can't get device address\n");
		return -EFAULT;
	}

	membase = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(membase)) {
		pr_err("Could not map resource\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		pr_err("Can't get interrupt\n");
		return -EFAULT;
	}

	priv->syscon = membase;

	priv->irqdomain = irq_domain_add_linear(
		np, TSWEIM_NUM_FPGA_IRQ, &tsweim_intc_irqdomain_ops, priv);

	if (!priv->irqdomain) {
		pr_err("%s: unable to add irq domain\n", np->name);
		return -ENOMEM;
	}

	irq_set_handler_data(res->start, priv);
	irq_set_chained_handler(res->start, tsweim_irq_handler);

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int tsweim_intc_remove(struct platform_device *pdev)
{
	struct tsweim_intc *priv = dev_get_platdata(&pdev->dev);

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
