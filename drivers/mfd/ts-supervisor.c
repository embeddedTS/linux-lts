// SPDX-License-Identifier: GPL-2.0-only
/* embeddedTS supervisory microcontroller */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>

static const struct regmap_config ts_supervisor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = 0xffff,
};

static int ts_supervisor_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regmap *regmap;

	/* Gen 0 uses an unaddressed I2C snapshot, not the register map. */
	if (!of_device_is_compatible(dev->of_node, "technologic,ts4100-supervisor") &&
	    !of_device_is_compatible(dev->of_node, "technologic,ts7553v2-supervisor")) {
		regmap = devm_regmap_init_i2c(client, &ts_supervisor_regmap_config);
		if (IS_ERR(regmap))
			return dev_err_probe(dev, PTR_ERR(regmap),
					     "failed to initialize regmap\n");
	}

	return devm_of_platform_populate(dev);
}

static const struct of_device_id ts_supervisor_of_match[] = {
	{ .compatible = "technologic,ts4100-supervisor" },
	{ .compatible = "technologic,ts7100-supervisor" },
	{ .compatible = "technologic,ts7553v2-supervisor" },
	{ }
};
MODULE_DEVICE_TABLE(of, ts_supervisor_of_match);

static struct i2c_driver ts_supervisor_driver = {
	.driver = {
		.name = "ts-supervisor",
		.of_match_table = ts_supervisor_of_match,
	},
	.probe = ts_supervisor_probe,
};
module_i2c_driver(ts_supervisor_driver);

MODULE_DESCRIPTION("embeddedTS supervisory microcontroller core");
MODULE_LICENSE("GPL");
