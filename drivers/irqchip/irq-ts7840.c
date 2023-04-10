// SPDX-License-Identifier: GPL-2.0
/*
 * Multiplexed-IRQs driver for TS-7840's FPGA
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#define IRQ_STATUS      0x0
#define IRQ_MASK        0x4

struct ts7840_irq_data {
	void __iomem            *base;
	struct irq_domain       *domain;
	struct irq_chip         irq_chip;
	raw_spinlock_t		mask_lock;
};

static void ts7840_irq_mask(struct irq_data *d)
{
	struct ts7840_irq_data *data = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	u32 reg;

	raw_spin_lock_irqsave(&data->mask_lock, flags);
	reg = readl(data->base + IRQ_MASK);
	writel(reg & ~(1 << d->hwirq), data->base + IRQ_MASK);
	raw_spin_unlock_irqrestore(&data->mask_lock, flags);
}

static void ts7840_irq_unmask(struct irq_data *d)
{
	struct ts7840_irq_data *data = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	u32 reg;

	raw_spin_lock_irqsave(&data->mask_lock, flags);
	reg = readl(data->base + IRQ_MASK);
	writel(reg | (1 << d->hwirq), data->base + IRQ_MASK);
	raw_spin_unlock_irqrestore(&data->mask_lock, flags);
}

static int ts7840_irqdomain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hwirq)
{
	struct ts7840_irq_data *data = d->host_data;

	irq_set_chip_and_handler(irq, &data->irq_chip, handle_level_irq);
	irq_set_chip_data(irq, data);
	irq_set_status_flags(irq, IRQ_LEVEL);
	irq_set_status_flags(irq, IRQ_IS_POLLED);

	return 0;
}

static const struct irq_domain_ops ts7840_ic_ops = {
	.map = ts7840_irqdomain_map,
	.xlate = irq_domain_xlate_onecell,
};

static void ts7840_ic_chained_handle_irq(struct irq_desc *desc)
{
	struct ts7840_irq_data *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 status = readl(data->base + IRQ_STATUS);

	if (status == 0)
		return;

	chained_irq_enter(chip, desc);

	while (status) {
		unsigned int bit = __ffs(status);
		int irq = irq_find_mapping(data->domain, bit);

		status &= ~(1 << bit);
		generic_handle_irq(irq);
	}

	chained_irq_exit(chip, desc);
}

static int ts7840_ic_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct ts7840_irq_data *data;
	struct irq_chip *irq_chip;
	struct resource *res;
	int parent_irq;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	/* Clear any enabled IRQs */
	writel(0x0, data->base + IRQ_MASK);

	parent_irq = irq_of_parse_and_map(node, 0);
	if (!parent_irq) {
		dev_err(&pdev->dev, "failed to get parent IRQ\n");
		return -EINVAL;
	}

	irq_chip = &data->irq_chip;
	irq_chip->name = dev_name(&pdev->dev);
	irq_chip->irq_mask = ts7840_irq_mask;
	irq_chip->irq_unmask = ts7840_irq_unmask;
	raw_spin_lock_init(&data->mask_lock);

	data->domain = irq_domain_add_linear(node, 32, &ts7840_ic_ops, data);
	if (!data->domain) {
		dev_err(&pdev->dev, "cannot add IRQ domain\n");
		return -ENOMEM;
	}

	irq_set_chained_handler_and_data(parent_irq,
					 ts7840_ic_chained_handle_irq, data);

	platform_set_drvdata(pdev, data);

	return 0;
}

static int ts7840_ic_remove(struct platform_device *pdev)
{
	struct ts7840_irq_data *data = platform_get_drvdata(pdev);

	irq_domain_remove(data->domain);

	return 0;
}

static const struct of_device_id ts7840_ic_of_match[] = {
	{ .compatible = "technologic,ts7840-irqc", },
	{},
};
MODULE_DEVICE_TABLE(of, ts7840_ic_of_match);

static struct platform_driver ts7840_ic_driver = {
	.probe  = ts7840_ic_probe,
	.remove = ts7840_ic_remove,
	.driver = {
		.name = "ts7840-irqc",
		.of_match_table = ts7840_ic_of_match,
	},
};
module_platform_driver(ts7840_ic_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ts7840_irqc");
