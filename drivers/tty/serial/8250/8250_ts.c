// SPDX-License-Identifier: GPL-2.0

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/tspc104_bus.h>

static unsigned int tsisa_serial_in(struct uart_port *p, int offset)
{
	struct tspc104_bus *bus = (struct tspc104_bus *)p->private_data;
	unsigned int value;

	tspc104_io_read8(bus, p->mapbase + offset, &value);
	return value;
}

static void tsisa_serial_out(struct uart_port *p, int offset, int value)
{
	struct tspc104_bus *bus = (struct tspc104_bus *)p->private_data;
	tspc104_io_write8(bus, p->mapbase + offset, &value);
}

static int technologic_ts16550_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uart_8250_port uport;
	struct uart_port *port;
	const __be32 *addr_be;

	memset(&uport, 0, sizeof(uport));

	addr_be = of_get_property(dev->of_node, "reg", NULL);
	if (!addr_be)
		return -ENODEV;

	port = &uport.port;
	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	port->mapbase = be32_to_cpup(addr_be);
	port->flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF |
		      UPF_FIXED_PORT | UPF_FIXED_TYPE;
	port->iotype = UPIO_MEM;
	port->irqflags = IRQF_TRIGGER_HIGH;
	port->private_data = platform_get_drvdata(to_platform_device(pdev->dev.parent));
	port->uartclk = 1843200;
	port->type = PORT_16550A;
	port->serial_in = tsisa_serial_in;
	port->serial_out = tsisa_serial_out;

	return serial8250_register_8250_port(&uport);
}

static const struct of_device_id ts16550_of_match[] = {
	{ .compatible = "technologic,ts16550", },
	{},
};

static struct platform_driver ts16550_driver = {
	.probe = technologic_ts16550_probe,
	.driver = {
		.name = "ts16550",
		.of_match_table = ts16550_of_match,
	},
};
module_platform_driver(ts16550_driver);

MODULE_ALIAS("platform:ts16550");
MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_DESCRIPTION("embeddedTS 16550 PC104 driver");
MODULE_LICENSE("GPL v2");
