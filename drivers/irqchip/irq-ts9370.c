// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 - Technologic Systems, DBA embeddedTS
 *
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
#include <linux/seq_file.h>
#include <linux/regmap.h>
#include <linux/bitops.h>

#define IRQ_STATUS 0x0
#define IRQ_MASK_SET 0x4
#define IRQ_MASK_CLR 0x8

struct ts9370_irq_data {
	struct regmap *regmap;
	struct device *dev;
	struct irq_domain *domain;
	raw_spinlock_t lock;
	int irq;
};

static void ts9370_irq_mask(struct irq_data *d)
{
	struct ts9370_irq_data *data = irq_data_get_irq_chip_data(d);
	u32 reg = BIT(d->hwirq);

	regmap_write(data->regmap, IRQ_MASK_SET, reg);
}

static void ts9370_irq_unmask(struct irq_data *d)
{
	struct ts9370_irq_data *data = irq_data_get_irq_chip_data(d);
	u32 reg = BIT(d->hwirq);

	regmap_write(data->regmap, IRQ_MASK_CLR, reg);
}

static const struct irq_chip ts9370_chip = {
	.irq_mask = ts9370_irq_mask,
	.irq_unmask = ts9370_irq_unmask,
};

static int ts9370_irqdomain_map(struct irq_domain *d, unsigned int irq,
				 irq_hw_number_t hwirq)
{
	struct ts9370_irq_data *data = d->host_data;

	irq_set_chip_and_handler(irq, &ts9370_chip, handle_level_irq);
	irq_set_chip_data(irq, data);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops ts9370_ic_ops = {
	.map = ts9370_irqdomain_map,
	.xlate = irq_domain_xlate_onecell,
};

static irqreturn_t ts9370_irq_handler(int irq, void *priv)
{
	struct ts9370_irq_data *data = (struct ts9370_irq_data *)priv;
	unsigned long lock_flags;
	unsigned long status;
	int i;

	regmap_read(data->regmap, IRQ_STATUS, (u32 *)&status);

	for_each_set_bit(i, &status, 32) {
		raw_spin_lock_irqsave(&data->lock, lock_flags);
		generic_handle_domain_irq(data->domain, i);
		raw_spin_unlock_irqrestore(&data->lock,
					   lock_flags);
	}

	return IRQ_HANDLED;
}

static int ts9370_ic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct ts9370_irq_data *data;
	const struct regmap_config regmap_cfg = {
		.reg_bits = 32,
		.val_bits = 32,
	};
	void __iomem *base;
	int ret = 0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	data->regmap = devm_regmap_init_mmio(dev, base, &regmap_cfg);
	if (IS_ERR(data->regmap)) {
		dev_err(dev, "failed to initialize regmap\n");
		return PTR_ERR(data->regmap);
	}

	/* Disable all interrupts initially */
	regmap_write(data->regmap, IRQ_MASK_SET, 0xffffffff);

	raw_spin_lock_init(&data->lock);
	platform_set_drvdata(pdev, data);

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0)
		return data->irq;

	data->domain = irq_domain_add_linear(node, 32, &ts9370_ic_ops, data);
	if (!data->domain) {
		dev_err(dev, "cannot add IRQ domain\n");
		return -ENOMEM;
	}

	ret = request_irq(data->irq, ts9370_irq_handler,
			  0, dev_name(dev), data);
	if (ret)
		goto out_domain_remove;

	return 0;

out_domain_remove:
	irq_domain_remove(data->domain);
	irq_dispose_mapping(data->irq);

	return ret;
}

static void ts9370_ic_remove(struct platform_device *pdev)
{
	struct ts9370_irq_data *data = platform_get_drvdata(pdev);

	irq_domain_remove(data->domain);
}

static const struct of_device_id ts9370_ic_of_match[] = {
	{ .compatible = "technologic,ts9370-irqc", },
	{},
};
MODULE_DEVICE_TABLE(of, ts9370_ic_of_match);

static struct platform_driver ts9370_ic_driver = {
	.probe  = ts9370_ic_probe,
	.remove = ts9370_ic_remove,
	.driver = {
		.name = "ts9370-irqc",
		.of_match_table = ts9370_ic_of_match,
	},
};
module_platform_driver(ts9370_ic_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ts9370_irqc");
