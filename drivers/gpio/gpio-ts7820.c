// SPDX-License-Identifier: GPL-2.0

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

/* Read Decodes */
#define TS7820_OE_IN		0x00
#define TS7820_OUT_DATA		0x08
#define TS7820_IN		0x0C
/*#define TS7820_IRQ_EN		0x10 */
#define TS7820_IRQSTATUS	0x14

/* Write Decodes */
#define TS7820_OE_SET		0x00
#define TS7820_OE_CLR		0x04
#define TS7820_DAT_SET		0x08
#define TS7820_DAT_CLR		0x0C
#define TS7820_IRQ_EN		0x10
#define TS7820_IRQ_NPOL		0x18 /* 1 = active low */

struct ts7820_gpio {
	void __iomem *base;
	struct device *dev;
	struct gpio_chip chip;
	struct irq_chip irqchip;
	raw_spinlock_t lock;
	uint32_t npol;
	int irq;
};

static void ts7820_gpio_set(struct gpio_chip *chip, unsigned int pin, int val)
{
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(chip);

	if (val)
		writel(BIT(pin), ts7820_gpio->base + TS7820_DAT_SET);
	else
		writel(BIT(pin), ts7820_gpio->base + TS7820_DAT_CLR);
}

static void ts7820_gpio_set_multiple(struct gpio_chip *chip,
				     unsigned long *mask,
				     unsigned long *bits)
{
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(chip);

	writel(*mask & *bits, ts7820_gpio->base + TS7820_DAT_SET);
	writel(*mask & (~*bits), ts7820_gpio->base + TS7820_DAT_CLR);
}

static int ts7820_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(chip);

	return !!(readl(ts7820_gpio->base + TS7820_IN) & BIT(pin));
}

static int ts7820_gpio_direction_input(struct gpio_chip *chip,
					unsigned int pin)
{
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(chip);

	writel(BIT(pin), ts7820_gpio->base + TS7820_OE_CLR);
	return 0;
}

static int ts7820_gpio_direction_output(struct gpio_chip *chip,
					unsigned int pin, int val)
{
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(chip);

	ts7820_gpio_set(chip, pin, val);
	writel(BIT(pin), ts7820_gpio->base + TS7820_OE_SET);
	return 0;
}

static int ts7820_gpio_direction_get(struct gpio_chip *chip,
				     unsigned int pin)
{
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(chip);

	return !(readl(ts7820_gpio->base + TS7820_OE_IN) & BIT(pin));
}

static void gpio_ts7820_irq_disable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(gc);
	u32 irqen;

	raw_spin_lock(&ts7820_gpio->lock);
	irqen = readl(ts7820_gpio->base + TS7820_IRQ_EN);
	irqen &= ~BIT(irqd_to_hwirq(d));
	writel(irqen, ts7820_gpio->base + TS7820_IRQ_EN);
	raw_spin_unlock(&ts7820_gpio->lock);
}

static void gpio_ts7820_irq_enable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(gc);
	u32 irqen;

	raw_spin_lock(&ts7820_gpio->lock);
	irqen = readl(ts7820_gpio->base + TS7820_IRQ_EN);
	irqen |= BIT(irqd_to_hwirq(d));
	writel(irqen, ts7820_gpio->base + TS7820_IRQ_EN);
	raw_spin_unlock(&ts7820_gpio->lock);
}

static int gpio_ts7820_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(gc);
	unsigned int hwirq = irqd_to_hwirq(d);
	int ret = 0;

	irq_set_status_flags(d->irq, IRQ_IS_POLLED);

	ts7820_gpio->npol &= ~hwirq;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_LEVEL_HIGH:
		ts7820_gpio->npol &= ~(1 << hwirq);
		writel(ts7820_gpio->npol, ts7820_gpio->base + TS7820_IRQ_NPOL);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		ts7820_gpio->npol |= (1 << hwirq);
		writel(ts7820_gpio->npol, ts7820_gpio->base + TS7820_IRQ_NPOL);
		break;
	case IRQ_TYPE_EDGE_FALLING:
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void gpio_ts7820_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct ts7820_gpio *ts7820_gpio = gpiochip_get_data(gc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned long status;
	u32 bit, girq;

	chained_irq_enter(irqchip, desc);
	status = readl(ts7820_gpio->base + TS7820_IRQSTATUS) &
		 readl(ts7820_gpio->base + TS7820_IRQ_EN);

	for_each_set_bit(bit, &status, 32) {
		girq = irq_find_mapping(ts7820_gpio->chip.irq.domain, bit);
		generic_handle_irq(girq);
	}
	chained_irq_exit(irqchip, desc);
}

static int ts7820_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ts7820_gpio *ts7820_gpio;
	struct gpio_irq_chip *girq;

	ts7820_gpio = devm_kzalloc(dev, sizeof(struct ts7820_gpio), GFP_KERNEL);
	if (!ts7820_gpio)
		return -ENOMEM;

	ts7820_gpio->irq = platform_get_irq(pdev, 0);

	ts7820_gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ts7820_gpio->base))
		return PTR_ERR(ts7820_gpio->base);

	ts7820_gpio->dev = dev;

	raw_spin_lock_init(&ts7820_gpio->lock);

	ts7820_gpio->chip.label = dev_name(dev);
	ts7820_gpio->chip.owner = THIS_MODULE;
	ts7820_gpio->chip.direction_input = ts7820_gpio_direction_input;
	ts7820_gpio->chip.direction_output = ts7820_gpio_direction_output;
	ts7820_gpio->chip.get_direction = ts7820_gpio_direction_get;
	ts7820_gpio->chip.set = ts7820_gpio_set;
	ts7820_gpio->chip.set_multiple = ts7820_gpio_set_multiple;
	ts7820_gpio->chip.get = ts7820_gpio_get;
	ts7820_gpio->chip.base = -1;
	ts7820_gpio->chip.ngpio = 32;
	ts7820_gpio->chip.parent = dev;

	if (ts7820_gpio->irq >= 0) {
		girq = &ts7820_gpio->chip.irq;
		girq->chip = &ts7820_gpio->irqchip;
		girq->chip->name = "ts7820-gpio";
		girq->chip->irq_mask = gpio_ts7820_irq_disable;
		girq->chip->irq_unmask = gpio_ts7820_irq_enable;
		girq->chip->irq_set_type = gpio_ts7820_irq_set_type;
		girq->handler = handle_level_irq;
		girq->parent_handler = gpio_ts7820_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(&pdev->dev, 1,
					sizeof(*girq->parents),
					GFP_KERNEL);
		if (!girq->parents)
			return -ENOMEM;
		girq->parents[0] = ts7820_gpio->irq;
	}

	return devm_gpiochip_add_data(dev, &ts7820_gpio->chip, ts7820_gpio);
}

static const struct of_device_id ts7820_gpio_of_match[] = {
	{ .compatible = "technologic,ts7820-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, ts7820_gpio_of_match);

static struct platform_driver ts7820_gpio_driver = {
	.probe = ts7820_gpio_probe,
	.driver = {
		.name = "ts7820-gpio",
		.of_match_table = ts7820_gpio_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ts7820_gpio_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_DESCRIPTION("TS-7820 FPGA GPIO driver");
MODULE_LICENSE("GPL v2");
