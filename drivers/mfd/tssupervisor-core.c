// SPDX-License-Identifier: GPL-2.0-only

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/of_device.h>
#include <linux/mfd/ts_supervisor.h>

#define MODEL_TS_7250_V3 0x7250

static struct mfd_cell tssupervisor_devs[] = {
	{
		.name = "tssupervisor-reset",
		.id = -1,
	},
	{
		.name = "tssupervisor-temp",
		.of_compatible = "technologic,supervisor-temp",
		.id = -1,
	},
	{
		.name = "tssupervisor-adc",
		.of_compatible = "technologic,supervisor-adc",
		.id = -1,
	}
};

static const struct regmap_range ts_supervisor_read_regs[] = {
	regmap_reg_range(0, 3), /* model/version/advertisements */
	regmap_reg_range(16, 16), /* flags */
	regmap_reg_range(24, 24), /* inputs */
	regmap_reg_range(32, 32), /* reboot_reason */
	regmap_reg_range(128, 160), /* ADCs+temp */
};

static const struct regmap_range ts_supervisor_write_regs[] = {
	regmap_reg_range(8, 8), /* cmds */
	regmap_reg_range(16, 16), /* flags */
};

const struct regmap_access_table ts_supervisor_read_register_set = {
	.yes_ranges = ts_supervisor_read_regs,
	.n_yes_ranges = ARRAY_SIZE(ts_supervisor_read_regs),
};

const struct regmap_access_table ts_supervisor_write_register_set = {
	.yes_ranges = ts_supervisor_write_regs,
	.n_yes_ranges = ARRAY_SIZE(ts_supervisor_write_regs),
};

const struct regmap_config ts_supervisor_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 16,
	.can_multi_write = true,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,

	.wr_table = &ts_supervisor_write_register_set,
	.rd_table = &ts_supervisor_read_register_set,
	.volatile_table = &ts_supervisor_read_register_set,

	.disable_locking = true,
	.cache_type = REGCACHE_NONE,
};
EXPORT_SYMBOL_GPL(ts_supervisor_i2c_regmap);

static ssize_t vbus_present_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_supervisor *super = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(super->regmap, SUPER_INPUTS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & INPUTS_USB_VBUS));
	return ret;
}
static DEVICE_ATTR_RO(vbus_present);

static ssize_t wake_en_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct ts_supervisor *super = dev_get_drvdata(dev);
	unsigned int ctrl_reg = 0;
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	if (en)
		ctrl_reg |= FLG_WAKE_EN;

	ret = regmap_update_bits(super->regmap, SUPER_FLAGS,
				  FLG_WAKE_EN,
				  ctrl_reg);

	return ret ? ret : count;
}

static ssize_t wake_en_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_supervisor *super = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(super->regmap, SUPER_FLAGS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & FLG_WAKE_EN));
	return ret;
}
static DEVICE_ATTR_RW(wake_en);

static ssize_t console_cfg_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct ts_supervisor *super = dev_get_drvdata(dev);
	unsigned int ctrl_reg;
	int ret;

	if (sysfs_streq(buf, "auto"))
		ctrl_reg = 0;
	else if (sysfs_streq(buf, "always-usb"))
		ctrl_reg = FLG_FORCE_USB_CON;
	else
		return -EINVAL;

	ret = regmap_update_bits(super->regmap, SUPER_FLAGS, FLG_FORCE_USB_CON,
				 ctrl_reg);

	return ret ? ret : count;
}

static ssize_t console_cfg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_supervisor *super = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(super->regmap, SUPER_FLAGS, &reg);
	if (ret)
		return ret;

	if (reg & FLG_FORCE_USB_CON)
		ret = sprintf(buf, "auto [always-usb]\n");
	else
		ret = sprintf(buf, "[auto] always-usb\n");

	return ret;
}
static DEVICE_ATTR_RW(console_cfg);

static struct attribute *ts7250v3_sysfs_entries[] = {
	&dev_attr_vbus_present.attr,
	&dev_attr_wake_en.attr,
	&dev_attr_console_cfg.attr,
	NULL,
};

static struct attribute_group ts7250v3_attr_group = {
	.attrs	= ts7250v3_sysfs_entries,
};

static int ts_supervisor_i2c_probe(struct i2c_client *client)
{
	struct ts_supervisor *super;
	struct device *dev = &client->dev;
	int err = 0, i;
	uint32_t model, revision;

	super = devm_kzalloc(dev, sizeof(struct ts_supervisor),
			     GFP_KERNEL);
	if (!super)
		return -ENOMEM;

	dev_set_drvdata(dev, super);

	super->client = client;
	super->regmap = devm_regmap_init_i2c(client, &ts_supervisor_i2c_regmap);
	if (IS_ERR(super->regmap)) {
		err = PTR_ERR(super->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", err);
		return err;
	}

	err = regmap_read(super->regmap, SUPER_MODEL, &model);
	if (err < 0)
		dev_err(dev, "error reading reg %u", SUPER_MODEL);
	err = regmap_read(super->regmap, SUPER_REV_INFO, &revision);
	if (err < 0)
		dev_err(dev, "error reading reg %u", SUPER_REV_INFO);
	dev_info(&client->dev, "Model %04X rev %d%s\n",
		 model,
		 revision & 0x7fff,
		 revision & 0x8000 ? " (DIRTY)" : "");

	if (model == MODEL_TS_7250_V3) {
		err = sysfs_create_group(&dev->kobj, &ts7250v3_attr_group);
		if (err)
			dev_warn(dev, "error creating sysfs entries\n");
	}

	/* Set up and register the platform devices. */
	for (i = 0; i < ARRAY_SIZE(tssupervisor_devs); i++) {
		tssupervisor_devs[i].platform_data = super;
		tssupervisor_devs[i].pdata_size = sizeof(struct ts_supervisor);
	}

	return mfd_add_devices(dev, 0, tssupervisor_devs,
			      ARRAY_SIZE(tssupervisor_devs), NULL, 0, NULL);
}

static const struct i2c_device_id ts_supervisor_i2c_id[] = {
	{ "tssupervisor", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ts_supervisor_i2c_id);

static const struct of_device_id ts_supervisor_i2c_of_match[] = {
	{ .compatible = "technologic,supervisor", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ts_supervisor_i2c_of_match);

static struct i2c_driver ts_supervisor_i2c_driver = {
	.driver = {
		.name = "tssupervisor-core",
		.of_match_table = of_match_ptr(ts_supervisor_i2c_of_match),
	},
	.probe = ts_supervisor_i2c_probe,
	.id_table = ts_supervisor_i2c_id,
};
module_i2c_driver(ts_supervisor_i2c_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_DESCRIPTION("Core driver for embeddedTS Supervisory microcontroller");
MODULE_LICENSE("GPL v2");
