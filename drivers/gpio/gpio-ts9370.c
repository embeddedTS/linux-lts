// SPDX-License-Identifier: GPL-2.0

#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/regmap.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mm.h>

/* Read Decodes */
#define TS9370_OE_IN		0x00
#define TS9370_OUT_DATA		0x08
#define TS9370_IN		0x0C

/* Write Decodes */
#define TS9370_OE_SET			0x00
#define TS9370_OE_CLR			0x04
#define TS9370_DAT_SET			0x08
#define TS9370_DAT_CLR			0x0C
#define TS9370_IRQ_PENDING		0x10 /* Read is pending */
#define TS9370_IRQ_ACK			0x10 /* Write is ack */
#define TS9370_IRQ_MASK_SET		0x14
#define TS9370_IRQ_MASK_CLR		0x18
#define TS9370_IRQ_MASK_AND_ACK		0x1C
#define TS9370_IRQ_EDGE_LEVEL		0x20
#define TS9370_IRQ_EDGE_SEL		0x24
#define TS9370_IRQ_POL			0x28

struct ts9370_gpio_priv {
	struct regmap *map;
	struct device *dev;
	struct gpio_chip chip;
	struct irq_chip irqchip;
	raw_spinlock_t lock;
	int irq;
};

static int ts9370_gpio_set(struct gpio_chip *chip, unsigned int pin, int val)
{
	int ret;
	struct ts9370_gpio_priv *p = gpiochip_get_data(chip);

	if (val)
		ret = regmap_write(p->map, TS9370_DAT_SET, BIT(pin));
	else
		ret = regmap_write(p->map, TS9370_DAT_CLR, BIT(pin));
	return ret;
}

static int ts9370_gpio_set_multiple(struct gpio_chip *chip,
				     unsigned long *mask,
				     unsigned long *bits)
{
	int ret;
	struct ts9370_gpio_priv *p = gpiochip_get_data(chip);

	ret = regmap_write(p->map, TS9370_DAT_SET, *mask & *bits);
	if (ret == 0)
		ret = regmap_write(p->map, TS9370_DAT_CLR, *mask & (~*bits));
	return ret;
}

static int ts9370_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct ts9370_gpio_priv *p = gpiochip_get_data(chip);
	u32 in;

	regmap_read(p->map, TS9370_IN, &in);

	return !!(in & BIT(pin));
}

static int ts9370_gpio_direction_input(struct gpio_chip *chip,
					unsigned int pin)
{
	struct ts9370_gpio_priv *p = gpiochip_get_data(chip);

	regmap_write(p->map, TS9370_OE_CLR, BIT(pin));
	return 0;
}


static int ts9370_gpio_direction_output(struct gpio_chip *chip,
	unsigned int pin, int val)
{
	struct ts9370_gpio_priv *p = gpiochip_get_data(chip);

	if (val)
		regmap_write(p->map, TS9370_DAT_SET, BIT(pin));
	else
		regmap_write(p->map, TS9370_DAT_CLR, BIT(pin));

	regmap_write(p->map, TS9370_OE_SET, BIT(pin));

	return 0;
}

static int ts9370_gpio_direction_get(struct gpio_chip *chip,
	unsigned int pin)
{
	struct ts9370_gpio_priv *p = gpiochip_get_data(chip);
	u32 oe_in;

	regmap_read(p->map, TS9370_OE_IN, &oe_in);
	return !(oe_in & BIT(pin));
}

static void gpio_ts9370_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts9370_gpio_priv *p = gpiochip_get_data(gc);
	u32 reg = BIT(irqd_to_hwirq(d));

	regmap_write(p->map, TS9370_IRQ_MASK_SET, reg);
	gpiochip_disable_irq(gc, d->hwirq);
}

static void gpio_ts9370_irq_mask_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts9370_gpio_priv *p = gpiochip_get_data(gc);
	u32 reg = BIT(irqd_to_hwirq(d));

	regmap_write(p->map, TS9370_IRQ_MASK_AND_ACK, reg);
}

static void gpio_ts9370_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts9370_gpio_priv *p = gpiochip_get_data(gc);
	u32 reg = BIT(irqd_to_hwirq(d));

	gpiochip_enable_irq(gc, d->hwirq);
	regmap_write(p->map, TS9370_IRQ_MASK_CLR, reg);
}

static void gpio_ts9370_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts9370_gpio_priv *p = gpiochip_get_data(gc);
	u32 reg = BIT(irqd_to_hwirq(d));

	regmap_write(p->map, TS9370_IRQ_ACK, reg);
}

static int gpio_ts9370_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts9370_gpio_priv *p = gpiochip_get_data(gc);
	unsigned int hwirq = irqd_to_hwirq(d);
	u32 polarity, edge, edge_sel;
	int ret = 0;

	regmap_read(p->map, TS9370_IRQ_POL, &polarity);
	regmap_read(p->map, TS9370_IRQ_EDGE_LEVEL, &edge);
	regmap_read(p->map, TS9370_IRQ_EDGE_SEL, &edge_sel);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_LEVEL_HIGH:
		edge &= ~BIT(hwirq);
		edge_sel &= ~BIT(hwirq);
		polarity |= BIT(hwirq);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge &= ~BIT(hwirq);
		polarity &= ~BIT(hwirq);
		edge_sel &= ~BIT(hwirq);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_EDGE_RISING:
		edge |= BIT(hwirq);
		edge_sel &= ~BIT(hwirq);
		polarity |= BIT(hwirq);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge |= BIT(hwirq);
		edge_sel &= ~BIT(hwirq);
		polarity &= ~BIT(hwirq);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		edge |= BIT(hwirq);
		edge_sel |= BIT(hwirq);
		polarity &= ~BIT(hwirq);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	default:
		ret = -EINVAL;
	}

	regmap_write(p->map, TS9370_IRQ_POL, polarity);
	regmap_write(p->map, TS9370_IRQ_EDGE_LEVEL, edge);
	regmap_write(p->map, TS9370_IRQ_EDGE_SEL, edge_sel);

	return ret;
}

static void gpio_ts9370_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct ts9370_gpio_priv *p = gpiochip_get_data(gc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned long pending;
	u32 reg;
	u32 bit;

	regmap_read(p->map, TS9370_IRQ_PENDING, &reg);
	pending = reg;
	chained_irq_enter(irqchip, desc);
	for_each_set_bit(bit, &pending, 32) {
		generic_handle_domain_irq(gc->irq.domain, bit);
	}
	chained_irq_exit(irqchip, desc);
}

static void gpio_ts9370_irq_print_chip(struct irq_data *data, struct seq_file *p)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);

	seq_printf(p, dev_name(gc->parent));
}

static const struct irq_chip ts9370_irq_chip = {
	.irq_ack		= gpio_ts9370_irq_ack,
	.irq_mask		= gpio_ts9370_irq_mask,
	.irq_mask_ack		= gpio_ts9370_irq_mask_ack,
	.irq_unmask		= gpio_ts9370_irq_unmask,
	.irq_set_type		= gpio_ts9370_irq_set_type,
	.irq_print_chip		= gpio_ts9370_irq_print_chip,
	.flags			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static const struct regmap_config ts9370_gpio_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x3F,
};

static int ts9370_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ts9370_gpio_priv *p;
	struct gpio_irq_chip *girq;
	void __iomem *base;

	p = devm_kzalloc(dev, sizeof(struct ts9370_gpio_priv), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->dev = dev;
	p->irq = platform_get_irq(pdev, 0);
	if (p->irq == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	raw_spin_lock_init(&p->lock);

	p->map = devm_regmap_init_mmio(dev, base, &ts9370_gpio_regmap_config);
	if (IS_ERR(p->map))
		return PTR_ERR(p->map);

	/* Default to mask all interrupts */
	regmap_write(p->map, TS9370_IRQ_MASK_AND_ACK, 0xFFFFFFFF);

	p->chip.label = dev_name(dev);
	p->chip.owner = THIS_MODULE;
	p->chip.direction_input = ts9370_gpio_direction_input;
	p->chip.direction_output = ts9370_gpio_direction_output;
	p->chip.get_direction = ts9370_gpio_direction_get;
	p->chip.set = ts9370_gpio_set;
	p->chip.set_multiple = ts9370_gpio_set_multiple;
	p->chip.get = ts9370_gpio_get;
	p->chip.base = -1;
	p->chip.ngpio = 32;
	p->chip.parent = dev;

	if (p->irq >= 0) {
		girq = &p->chip.irq;
		gpio_irq_chip_set_chip(girq, &ts9370_irq_chip);
		girq->parent_handler = gpio_ts9370_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(&pdev->dev, 1,
					sizeof(*girq->parents),
					GFP_KERNEL);
		if (!girq->parents)
			return -ENOMEM;
		girq->parents[0] = p->irq;
		girq->handler = handle_bad_irq;
	}

	return devm_gpiochip_add_data(dev, &p->chip, p);
}

static const struct of_device_id ts9370_gpio_of_match[] = {
	{ .compatible = "technologic,ts9370-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, ts9370_gpio_of_match);

static struct platform_driver ts9370_gpio_driver = {
	.probe = ts9370_gpio_probe,
	.driver = {
		.name = "ts9370-gpio",
		.of_match_table = ts9370_gpio_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ts9370_gpio_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_DESCRIPTION("TS-9370 FPGA GPIO driver");
MODULE_LICENSE("GPL");
