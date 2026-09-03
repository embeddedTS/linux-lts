// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Sitronix ST7571 connected via SPI bus.
 *
 * Copyright (C) 2025 Marcus Folkesson <marcus.folkesson@gmail.com>
 */

#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>

#include "st7571.h"

struct st7571_spi {
	struct spi_device *spi;
	struct gpio_desc *dc;
};

static int st7571_spi_write(void *context, const void *data, size_t count)
{
	struct st7571_spi *stspi = context;
	const u8 *buf = data;

	if (count < 2)
		return -EINVAL;

	gpiod_set_value_cansleep(stspi->dc, buf[0] == 0x40);
	return spi_write(stspi->spi, buf + 1, count - 1);
}

static const struct regmap_bus st7571_spi_regmap_bus = {
	.write = st7571_spi_write,
};

static const struct regmap_config st7571_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.can_multi_write = true,
};

static int st7571_spi_probe(struct spi_device *spi)
{
	struct st7571_device *st7571;
	struct st7571_spi *stspi;
	struct regmap *regmap;

	stspi = devm_kzalloc(&spi->dev, sizeof(*stspi), GFP_KERNEL);
	if (!stspi)
		return -ENOMEM;

	stspi->spi = spi;
	stspi->dc = devm_gpiod_get(&spi->dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(stspi->dc))
		return dev_err_probe(&spi->dev, PTR_ERR(stspi->dc),
				     "Failed to get command/data gpio\n");

	spi->mode = SPI_MODE_3;
	if (spi_setup(spi))
		return -EINVAL;

	regmap = devm_regmap_init(&spi->dev, &st7571_spi_regmap_bus, stspi,
				  &st7571_spi_regmap_config);
	if (IS_ERR(regmap)) {
		return dev_err_probe(&spi->dev, PTR_ERR(regmap),
				     "Failed to initialize regmap\n");
	}

	st7571 = st7571_probe(&spi->dev, regmap);
	if (IS_ERR(st7571))
		return dev_err_probe(&spi->dev, PTR_ERR(st7571),
				     "Failed to initialize regmap\n");

	spi_set_drvdata(spi, st7571);
	return 0;
}

static void st7571_spi_remove(struct spi_device *spi)
{
	struct st7571_device *st7571 = spi_get_drvdata(spi);

	st7571_remove(st7571);
}

static const struct of_device_id st7571_of_match[] = {
	{ .compatible = "sitronix,st7567", .data = &st7567_config },
	{ .compatible = "sitronix,st7565p", .data = &st7565p_config },
	{ .compatible = "sitronix,st7571", .data = &st7571_config },
	{},
};
MODULE_DEVICE_TABLE(of, st7571_of_match);

static const struct spi_device_id st7571_spi_id[] = {
	{ "st7567", 0 },
	{ "st7565p", 0 },
	{ "st7571", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, st7571_spi_id);

static struct spi_driver st7571_spi_driver = {
	.driver = {
		.name = "st7571-spi",
		.of_match_table = st7571_of_match,
	},
	.probe = st7571_spi_probe,
	.remove = st7571_spi_remove,
	.id_table = st7571_spi_id,
};

module_spi_driver(st7571_spi_driver);

MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@gmail.com>");
MODULE_DESCRIPTION("DRM Driver for Sitronix ST7571 LCD controller (SPI)");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DRM_ST7571");
