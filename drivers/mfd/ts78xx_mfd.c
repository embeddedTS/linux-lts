// SPDX-License-Identifier: GPL-2.0
/*
 * PCI device tree support
 *
 * Copyright (C) 2022 Mark Featherston <mark@embeddedTS.com>
 */

#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pci.h>

/* If present, IRQC base is 0x200
 * 0x0 = doorbell_adr
 * 0x4 = doorbell_msk (1 = enabled)
 * 0x8 = irqs
 */

#define TS7800V2_IRQC		0x200
#define TS7800V2_IRQC_MASK_REG	0x4
#define TS7800V2_IRQC_IRQS	0x8

#define TS7800V2_PC104_A_MUX	0x30
#define TS7800V2_PC104_B_MUX	0x34
#define TS7800V2_PC104_C_MUX	0x38
#define TS7800V2_PC104_D_MUX	0x3c

struct tsmfd_of_priv {
	struct irq_chip		chip;
	struct irq_domain	*domain;
	void __iomem		*fpgabase;
	int			irqnum;
	u32			mask;
	u32			map[32];
	spinlock_t		lock;
};

static int tsfpga_add_ranges(struct pci_dev *pdev, struct device_node *np)
{
	struct property *prop;
	u32 start, len;
	struct of_changeset ocs;
	uint32_t *val;
	int i;

	prop = devm_kcalloc(&pdev->dev, 1, sizeof(*prop), GFP_KERNEL);
	if (!prop)
		return -ENOMEM;

	val = devm_kcalloc(&pdev->dev, 1, sizeof(uint32_t)*4*DEVICE_COUNT_RESOURCE, GFP_KERNEL);
	if (!val)
		return -ENOMEM;

	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		start = pci_resource_start(pdev, i);
		if (!start)
			break;

		len = pci_resource_len(pdev, i);
		val[(i * 4)] = cpu_to_be32(i);
		val[(i * 4) + 1] = cpu_to_be32(0);
		val[(i * 4) + 2] = cpu_to_be32(start);
		val[(i * 4) + 3] = cpu_to_be32(len);
	}

	prop->name = devm_kstrdup(&pdev->dev, "ranges", GFP_KERNEL);
	prop->value = val;
	prop->length = sizeof(uint32_t) * 4 * i;

	of_changeset_init(&ocs);
	of_changeset_add_property(&ocs, np, prop);
	of_changeset_apply(&ocs);

	return 0;
}

static void tsmfd_chained_irq_handler(struct irq_desc *desc)
{
	struct tsmfd_of_priv *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 status = readl(priv->fpgabase + TS7800V2_IRQC + TS7800V2_IRQC_IRQS);

	status &= priv->mask;

	chained_irq_enter(chip, desc);
	if (!status)
		goto out;

	do {
		unsigned int bit = __ffs(status);
		int irq = irq_find_mapping(priv->domain, bit);

		status &= ~(1 << bit);
		generic_handle_irq(irq);
	} while (status);

out:
	chained_irq_exit(chip, desc);
}

static struct irq_chip pci_irq_chip = {
	.name = "tsmfd_irq",
};

static int tsmfd_irqdomain_map(struct irq_domain *d,
		unsigned int virq, irq_hw_number_t hwirq)
{
	struct tsmfd_of_priv *priv = d->host_data;

	irq_set_chip_and_handler(virq, &pci_irq_chip,
				 handle_simple_irq);
	irq_set_chip_data(virq, priv);
	irq_clear_status_flags(virq, IRQ_NOREQUEST);
	irq_set_status_flags(virq, IRQ_IS_POLLED);

	return 0;
}

static const struct irq_domain_ops tsmfd_irqdomain_ops = {
	.map = tsmfd_irqdomain_map,
	.xlate = irq_domain_xlate_onecell,
};

static int ts7800v2_irqc_enable(struct pci_dev *pdev, struct device_node *np, u32 irqnum)
{
	struct tsmfd_of_priv *priv;
	struct device *dev = &pdev->dev;
	int ret, i;

	priv = devm_kcalloc(dev, 1, sizeof(struct tsmfd_of_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = pci_alloc_irq_vectors(pdev, irqnum, irqnum, PCI_IRQ_MSI);
	if (ret != irqnum)
		return -ENODEV;

	spin_lock_init(&priv->lock);
	priv->fpgabase = pci_ioremap_bar(pdev, 2);
	priv->irqnum = irqnum;
	priv->domain = irq_domain_add_linear(np, irqnum, &tsmfd_irqdomain_ops, priv);
	if (!priv->domain) {
		dev_err(dev, "Couldn't create IRQ domain");
		return -ENODEV;
	}

	priv->mask = 0xFFF0000;
	writel(priv->mask, priv->fpgabase + TS7800V2_IRQC + TS7800V2_IRQC_MASK_REG);

	for (i = 0; i < irqnum; i++) {
		priv->map[i] =  pci_irq_vector(pdev, i);
		irq_set_chained_handler_and_data(pci_irq_vector(pdev, i),
			tsmfd_chained_irq_handler, priv);
	}

	return 0;
}

/* Switch mux registers on TS-7800-V2 from GPIO to a PC104 bus */
void ts7800v2_pc104on(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *base = pci_ioremap_bar(pdev, 2);

	dev_info(dev, "Enabling PC104");
	writel(0x55555555, base + TS7800V2_PC104_A_MUX);
	writel(0x55555555, base + TS7800V2_PC104_B_MUX);
	writel(0x55555, base + TS7800V2_PC104_C_MUX);
	writel(0x55555, base + TS7800V2_PC104_D_MUX);
}

static int tsfpga_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct device_node *np;
	struct device *dev = &pdev->dev;
	u32 irqnum;
	int ret = 0;

	if (pci_enable_device(pdev)) {
		ret = -ENODEV;
		goto out;
	}
	np = of_find_compatible_node(NULL, NULL, "technologic,ts78xx-mfd");
	if (np == NULL) {
		dev_err(dev, "Couldn't find the device tree node!\n");
		ret = -ENODEV;
		goto out_pci_disable_device;
	}

	ret = of_property_read_u32(np, "irqnum", &irqnum);
	if (ret < 0)
		irqnum = 0;

	pdev->dev.of_node = np;
	ret = tsfpga_add_ranges(pdev, np);
	if (ret)
		goto out_pci_disable_device;

	if (irqnum) {
		ret = ts7800v2_irqc_enable(pdev, np, irqnum);
		if (ret) {
			dev_err(dev, "Couldn't enable TS-7800-V2 IRQs\n");
			goto out_pci_disable_device;
		}
	}

	if (of_property_read_bool(np, "pc104"))
		ts7800v2_pc104on(pdev);

	return of_platform_default_populate(np, NULL, &pdev->dev);

out_pci_disable_device:
	pci_free_irq_vectors(pdev);
	pci_disable_device(pdev);
	pci_release_regions(pdev);

out:
	return ret;
}

static void tsfpga_pci_remove(struct pci_dev *pdev)
{
	pci_free_irq_vectors(pdev);
	pci_disable_device(pdev);
	pci_release_regions(pdev);
}

static int tsfpga_plat_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const char *name = dev_name(&pdev->dev);
	int vendor, device, ret;
	struct pci_driver *pci;
	struct pci_device_id *id_table;

	ret = of_property_read_u32(np, "vendor", &vendor);
	if (ret) {
		pr_err("Missing pci vendor id");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "device", &device);
	if (ret) {
		pr_err("Missing pci device id");
		return -EINVAL;
	}

	pci = devm_kzalloc(dev, sizeof(struct pci_driver), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	id_table = devm_kzalloc(dev, sizeof(struct pci_device_id)*2, GFP_KERNEL);
	if (!id_table)
		return -ENOMEM;

	id_table[0].vendor = vendor;
	id_table[0].device = device;
	id_table[0].subvendor = PCI_ANY_ID;
	id_table[0].subdevice = PCI_ANY_ID;

	pci->name = name;
	pci->probe = tsfpga_pci_probe;
	pci->remove = tsfpga_pci_remove;
	pci->id_table = id_table;

	ret = pci_register_driver(pci);
	if (ret) {
		pr_err("Could not register pci driver\n");
		return -EINVAL;
	}

	return 0;
}

static int tsfpga_plat_driver_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id tsmfd_of_match_table[] = {
	{ .compatible = "technologic,ts78xx-mfd", },
	{},
};

static struct platform_driver tsmfd_platform_driver = {
	.driver = {
		.name = "ts78xx_mfd",
		.of_match_table = of_match_ptr(tsmfd_of_match_table),
	},
	.probe = tsfpga_plat_driver_probe,
	.remove = tsfpga_plat_driver_remove,
};
module_platform_driver(tsmfd_platform_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_DESCRIPTION("TS-78XX Series FPGA MFD driver");
MODULE_LICENSE("GPL v2");
