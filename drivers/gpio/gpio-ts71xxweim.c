// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital I/O driver for embeddedTS TS-7120, TS-7100, et al.
 * Copyright (C) 2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/seq_file.h>

/* Most that this driver can currently support in a single bank is 16. This is
 * due simply to how the FPGA used for these devices is structured.
 */
#define TSWEIM_NR_DIO		16

/* Register offsets from the 'reg' value passed in device tree source */
#define TSWEIM_SET_REG		0x00
#define TSWEIM_GET_REG		0x00
#define TSWEIM_EN_SET_REG	0x02
#define TSWEIM_CLR_REG		0x04
#define TSWEIM_EN_CLR_REG	0x06

/* The IRQ registers are from a second register range */
#define TSWEIM_GPIO_IRQ_ACK		0x0
#define TSWEIM_GPIO_IRQ_ACK_MASK	0x2
#define TSWEIM_GPIO_IRQ_EDGE_LEVEL	0x4
#define TSWEIM_GPIO_IRQ_EDGE_SEL	0x6
#define TSWEIM_GPIO_IRQ_POLARITY	0x8
#define TSWEIM_GPIO_IRQ_MASK		0xA
#define TSWEIM_GPIO_IRQ_PENDING		0xC

struct tsweim_gpio_priv {
	void __iomem *base;
	void __iomem *irqbase;
	struct gpio_chip gpio_chip;
	spinlock_t lock;
};

static inline struct tsweim_gpio_priv *to_gpio_tsweim(struct gpio_chip *chip)
{
	return container_of(chip, struct tsweim_gpio_priv, gpio_chip);
}

static int tsweim_gpio_direction_input(struct gpio_chip *chip,
						 unsigned int offset)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);

	if (!(offset < priv->gpio_chip.ngpio))
		return -EINVAL;

	writew((1 << offset), priv->base + TSWEIM_EN_CLR_REG);

	return 0;
}

static int tsweim_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);
	unsigned long flags;

	if (!(offset < priv->gpio_chip.ngpio))
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);

	if (value)
		writew((1 << offset), priv->base + TSWEIM_SET_REG);
	else
		writew((1 << offset), priv->base + TSWEIM_CLR_REG);

	writew((1 << offset), priv->base + TSWEIM_EN_SET_REG);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int tsweim_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);
	uint16_t reg;

	if (!(offset < priv->gpio_chip.ngpio))
		return -EINVAL;

	reg = readw(priv->base + TSWEIM_GET_REG);
	return !!(reg & (1 << offset));
}

static void tsweim_gpio_set(struct gpio_chip *chip, unsigned int offset,
				 int value)
{
	struct tsweim_gpio_priv *priv = to_gpio_tsweim(chip);

	if (!(offset < priv->gpio_chip.ngpio))
		return;

	if (value)
		writew((1 << offset), priv->base + TSWEIM_SET_REG);
	else
		writew((1 << offset), priv->base + TSWEIM_CLR_REG);
}

static const struct gpio_chip tsweim_gpio_chip = {
	.owner			= THIS_MODULE,
	.direction_input	= tsweim_gpio_direction_input,
	.direction_output	= tsweim_gpio_direction_output,
	.get			= tsweim_gpio_get,
	.set			= tsweim_gpio_set,
	.base			= -1,
	.can_sleep		= false,
};

static void gpio_tsweim_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct tsweim_gpio_priv *priv = gpiochip_get_data(gc);
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	unsigned long pending, bit;

	chained_irq_enter(irq_chip, desc);

	pending = readw(priv->irqbase + TSWEIM_GPIO_IRQ_PENDING);

	for_each_set_bit(bit, &pending, 16) {
		generic_handle_domain_irq(gc->irq.domain, bit);
	}

	chained_irq_exit(irq_chip, desc);
}

static void gpio_tsweim_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tsweim_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned long flags;
	u16 reg;

	spin_lock_irqsave(&priv->lock, flags);

	reg = readw(priv->irqbase + TSWEIM_GPIO_IRQ_ACK) | BIT(irqd_to_hwirq(d));
	writew(reg, priv->irqbase + TSWEIM_GPIO_IRQ_ACK);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static void gpio_tsweim_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tsweim_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned long flags;
	u16 mask;

	spin_lock_irqsave(&priv->lock, flags);

	mask = readw(priv->irqbase + TSWEIM_GPIO_IRQ_MASK) | BIT(irqd_to_hwirq(d));
	writew(mask, priv->irqbase + TSWEIM_GPIO_IRQ_MASK);

	spin_unlock_irqrestore(&priv->lock, flags);

	gpiochip_disable_irq(gc, d->hwirq);
}

static void gpio_tsweim_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tsweim_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned long flags;
	u16 mask;

	gpiochip_enable_irq(gc, d->hwirq);

	spin_lock_irqsave(&priv->lock, flags);

	mask = readw(priv->irqbase + TSWEIM_GPIO_IRQ_MASK) & ~BIT(irqd_to_hwirq(d));
	writew(mask, priv->irqbase + TSWEIM_GPIO_IRQ_MASK);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int gpio_tsweim_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tsweim_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned int hwirq = irqd_to_hwirq(d);
	u16 polarity, edge, edge_sel;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	polarity = readw(priv->irqbase + TSWEIM_GPIO_IRQ_POLARITY);
	edge = readw(priv->irqbase + TSWEIM_GPIO_IRQ_EDGE_LEVEL);
	edge_sel = readw(priv->irqbase + TSWEIM_GPIO_IRQ_EDGE_SEL);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_LEVEL_HIGH:
		edge &= ~BIT(hwirq);
		edge_sel &= ~BIT(hwirq);
		polarity |= BIT(hwirq);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge &= ~BIT(hwirq);
		edge_sel &= ~BIT(hwirq);
		polarity &= ~BIT(hwirq);
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
		spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	writew(polarity, priv->irqbase + TSWEIM_GPIO_IRQ_POLARITY);
	writew(edge, priv->irqbase + TSWEIM_GPIO_IRQ_EDGE_LEVEL);
	writew(edge_sel, priv->irqbase + TSWEIM_GPIO_IRQ_EDGE_SEL);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void gpio_tsweim_irq_print_chip(struct irq_data *data, struct seq_file *p)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);

	seq_printf(p, dev_name(gc->parent));
}

static const struct irq_chip tsweim_irq_chip = {
	.name			= "tsweim_gpio_irq",
	.irq_ack		= gpio_tsweim_irq_ack,
	.irq_mask		= gpio_tsweim_irq_mask,
	.irq_unmask		= gpio_tsweim_irq_unmask,
	.irq_set_type		= gpio_tsweim_irq_set_type,
	.irq_print_chip		= gpio_tsweim_irq_print_chip,
	.flags			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static const struct of_device_id tsweim_gpio_of_match_table[] = {
	{ .compatible = "technologic,ts71xxweim-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, tsweim_gpio_of_match_table);

static int tsweim_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tsweim_gpio_priv *priv;
	struct resource *res;
	struct gpio_irq_chip *girq;
	int irq;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -EFAULT;

	priv->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!priv->base)
		return -ENOMEM;

	priv->gpio_chip = tsweim_gpio_chip;
	priv->gpio_chip.label = dev_name(dev);
	priv->gpio_chip.ngpio = TSWEIM_NR_DIO;
	priv->gpio_chip.parent = dev;

	spin_lock_init(&priv->lock);

	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0) {
		return devm_gpiochip_add_data(dev, &priv->gpio_chip, priv);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -EFAULT;

	priv->irqbase = devm_ioremap(dev, res->start, resource_size(res));
	if (!priv->irqbase)
		return -ENOMEM;

	girq = &priv->gpio_chip.irq;
	gpio_irq_chip_set_chip(girq, &tsweim_irq_chip);
	girq->parent_handler = gpio_tsweim_irq_handler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents)
		return -ENOMEM;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;
	girq->parents[0] = irq;

	return devm_gpiochip_add_data(dev, &priv->gpio_chip, priv);
}

static struct platform_driver tsweim_gpio_driver = {
	.driver = {
		.name = "tsweim-gpio",
		.of_match_table = of_match_ptr(tsweim_gpio_of_match_table),
	},
	.probe = tsweim_gpio_probe,
};
module_platform_driver(tsweim_gpio_driver);

MODULE_AUTHOR("embeddedTS");
MODULE_DESCRIPTION("GPIO interface for embeddedTS WEIM FPGA");
MODULE_LICENSE("GPL");
