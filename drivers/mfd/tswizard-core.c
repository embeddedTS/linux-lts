// SPDX-License-Identifier: GPL-2.0-only

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/of_device.h>
#include <linux/mfd/ts_wizard.h>

#define MODEL_TS_7250_V3 0x7250
#define MODEL_TS_SILO_104 0x5104

static struct mfd_cell tswizard_devs[] = {
	{
		.name = "tswizard-reset",
        .of_compatible = "technologic,wizard-reset",
		.id = PLATFORM_DEVID_AUTO,
	},
	{
		.name = "tswizard-temp",
		.of_compatible = "technologic,wizard-temp",
		.id = PLATFORM_DEVID_AUTO,
	},
	{
		.name = "tswizard-adc",
		.of_compatible = "technologic,wizard-adc",
		.id = PLATFORM_DEVID_AUTO,
	}
};

static struct mfd_cell silo104_devs[] = {
        {
                .name = "tswizard-irq",
                .of_compatible = "technologic,wizard-irq",
                .id = PLATFORM_DEVID_AUTO,
        },
        {
                .name = "tswizard-reset",
                .of_compatible = "technologic,wizard-reset",
                .id = PLATFORM_DEVID_AUTO,
        },
        {
                .name = "tswizard-silo",
                .of_compatible = "technologic,wizard-silo",
                .id = PLATFORM_DEVID_NONE,
        },
        {
                .name = "tswizard-temp",
                .of_compatible = "technologic,wizard-temp",
                .id = PLATFORM_DEVID_AUTO,
        },
        {
                .name = "tswizard-adc",
                .of_compatible = "technologic,wizard-adc",
                .id = PLATFORM_DEVID_AUTO,
        },
};

static const struct regmap_range ts_wizard_read_regs[] = {
	regmap_reg_range(0, 3), /* model/version/advertisements */
	regmap_reg_range(16, 16), /* flags */
	regmap_reg_range(24, 24), /* inputs */
	regmap_reg_range(32, 32), /* reboot_reason */
	regmap_reg_range(34, 37), /* serial */
	regmap_reg_range(WIZ_SILO_BASE, WIZ_SILO_BASE +16), /* silo regs (64-80)*/
	regmap_reg_range(128, 160), /* ADCs+temp */
};

static const struct regmap_range ts_wizard_write_regs[] = {
	regmap_reg_range(8, 8), /* cmds */
	regmap_reg_range(16, 16), /* flags */
	regmap_reg_range(34, 37), /* serial */
	regmap_reg_range(WIZ_SILO_BASE +2, WIZ_SILO_BASE +2), /* silo control (66)*/
	regmap_reg_range(WIZ_SILO_BASE +4, WIZ_SILO_BASE +4), /* silo req chg current (68) */
	regmap_reg_range(WIZ_SILO_BASE +9, WIZ_SILO_BASE +9), /* CRITICAL_PCT (base+9) */
	regmap_reg_range(WIZ_SILO_BASE +12, WIZ_SILO_BASE +13), /* startup current (76), min pwr on pct (77) */
};

const struct regmap_access_table ts_wizard_read_register_set = {
	.yes_ranges = ts_wizard_read_regs,
	.n_yes_ranges = ARRAY_SIZE(ts_wizard_read_regs),
};

const struct regmap_access_table ts_wizard_write_register_set = {
	.yes_ranges = ts_wizard_write_regs,
	.n_yes_ranges = ARRAY_SIZE(ts_wizard_write_regs),
};

const struct regmap_config ts_wizard_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 16,
	.can_multi_write = true,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,

	.wr_table = &ts_wizard_write_register_set,
	.rd_table = &ts_wizard_read_register_set,
	.volatile_table = &ts_wizard_read_register_set,

	.disable_locking = true,
	.cache_type = REGCACHE_NONE,
};
EXPORT_SYMBOL_GPL(ts_wizard_i2c_regmap);

static ssize_t vbus_present_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_wizard *wiz = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wiz->regmap, WIZ_INPUTS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & INPUTS_USB_VBUS));
	return ret;
}
static DEVICE_ATTR_RO(vbus_present);

static ssize_t wake_en_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct ts_wizard *wiz = dev_get_drvdata(dev);
	unsigned int ctrl_reg = 0;
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	if (en)
		ctrl_reg |= FLG_WAKE_EN;

	ret = regmap_update_bits(wiz->regmap, WIZ_FLAGS,
				  FLG_WAKE_EN,
				  ctrl_reg);

	return ret ? ret : count;
}

static ssize_t wake_en_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_wizard *wiz = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wiz->regmap, WIZ_FLAGS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & FLG_WAKE_EN));
	return ret;
}
static DEVICE_ATTR_RW(wake_en);

static ssize_t serial_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct ts_wizard *wiz = dev_get_drvdata(dev);
	unsigned int ctrl = 0;
	u8 sn[6], verify[6];
	int ret;

	ret = regmap_read(wiz->regmap, WIZ_SERIAL_CTRL, &ctrl);
	if (ret)
		return ret;

	/* Dont write the SN if its already written */
	if (ctrl & WIZ_SN_LOCKED)
		return -EEXIST;

	if (count < 17)
		return -EIO;

	if (sscanf(buf, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
		   &sn[0], &sn[1], &sn[2], &sn[3], &sn[4], &sn[5]) != 6)
		return -EINVAL;

	ret = regmap_bulk_write(wiz->regmap, WIZ_SERIAL0, sn, 3);
	if (ret)
		return ret;

	/* Verify data before commiting OTP SN */
	ret = regmap_bulk_read(wiz->regmap, WIZ_SERIAL0, verify, 3);
	if (ret)
		return ret;

	if (memcmp(verify, sn, 6) != 0)
		return -EIO;

	ret = regmap_update_bits(wiz->regmap, WIZ_SERIAL_CTRL,
				 WIZ_SN_LOCKED,
				 WIZ_SN_LOCKED);

	return ret ? ret : count;
}

static ssize_t serial_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_wizard *wiz = dev_get_drvdata(dev);
	unsigned int ctrl;
	u8 sn[6];
	int ret;

	ret = regmap_read(wiz->regmap, WIZ_SERIAL_CTRL, &ctrl);
	if (ret)
		return ret;

	/* Dont show the serial number if its not written yet */
	if (!(ctrl & WIZ_SN_LOCKED))
		return -EINVAL;

	ret = regmap_bulk_read(wiz->regmap, WIZ_SERIAL0, sn, 3);
	if (ret)
		return ret;
	ret = sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		sn[0], sn[1], sn[2], sn[3], sn[4], sn[5]);

	return ret;
}
static DEVICE_ATTR_RW(serial);

static ssize_t console_cfg_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct ts_wizard *wiz = dev_get_drvdata(dev);
	unsigned int ctrl_reg;
	int ret;

	if (sysfs_streq(buf, "auto"))
		ctrl_reg = 0;
	else if (sysfs_streq(buf, "always-usb"))
		ctrl_reg = FLG_FORCE_USB_CON;
	else
		return -EINVAL;

	ret = regmap_update_bits(wiz->regmap, WIZ_FLAGS, FLG_FORCE_USB_CON,
				 ctrl_reg);

	return ret ? ret : count;
}

static ssize_t console_cfg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_wizard *wiz = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wiz->regmap, WIZ_FLAGS, &reg);
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

static struct attribute *serial_sysfs_entries[] = {
	&dev_attr_serial.attr,
	NULL,
};

static struct attribute_group serial_attr_group = {
	.attrs	= serial_sysfs_entries,
};

static int ts_wizard_i2c_probe(struct i2c_client *client)
{
	struct ts_wizard *wiz;
	struct device *dev = &client->dev;
	int err = 0, i, j;
	uint32_t model, revision, features;

	wiz = devm_kzalloc(dev, sizeof(struct ts_wizard),
			     GFP_KERNEL);
	if (!wiz)
		return -ENOMEM;

	dev_set_drvdata(dev, wiz);

	wiz->client = client;
	wiz->regmap = devm_regmap_init_i2c(client, &ts_wizard_i2c_regmap);
	if (IS_ERR(wiz->regmap)) {
		err = PTR_ERR(wiz->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", err);
		return err;
	}

	err = regmap_read(wiz->regmap, WIZ_MODEL, &model);
	if (err < 0)
		dev_err(dev, "error reading reg %u", WIZ_MODEL);
	err = regmap_read(wiz->regmap, WIZ_REV_INFO, &revision);
	if (err < 0)
		dev_err(dev, "error reading reg %u", WIZ_REV_INFO);
	err = regmap_read(wiz->regmap, WIZ_FEATURES0, &features);
	if (err < 0)
		dev_err(dev, "error reading reg %u", WIZ_FEATURES0);
	dev_info(&client->dev, "Model %04X rev %d%s\n",
		 model,
		 revision & 0x7fff,
		 revision & 0x8000 ? " (DIRTY)" : "");

	if (model == MODEL_TS_7250_V3) {
		err = sysfs_create_group(&dev->kobj, &ts7250v3_attr_group);
		if (err)
			dev_warn(dev, "error creating sysfs entries\n");
	}

	if (features & WIZ_FEAT_SN) {
		err = sysfs_create_group(&dev->kobj, &serial_attr_group);
		if (err)
			dev_warn(dev, "error creating sysfs entries\n");
	}

	if (model == MODEL_TS_SILO_104) {
		err = mfd_add_devices(dev, PLATFORM_DEVID_AUTO, silo104_devs,
				      ARRAY_SIZE(silo104_devs), NULL, 0, NULL);
		if (err) {
			dev_err(dev, "Failed to add SILO104 devices: %d\n", err);
		}
		return err;
	}

	/* Set up and register the platform devices. */
	for (i = 0; i < ARRAY_SIZE(tswizard_devs); i++) {
		tswizard_devs[i].platform_data = wiz;
		tswizard_devs[i].pdata_size = sizeof(struct ts_wizard);
	}

	return mfd_add_devices(dev, PLATFORM_DEVID_AUTO, tswizard_devs,
			      ARRAY_SIZE(tswizard_devs), NULL, 0, NULL);
};

static const struct i2c_device_id ts_wizard_i2c_id[] = {
	{ "tswizard", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ts_wizard_i2c_id);

static const struct of_device_id ts_wizard_i2c_of_match[] = {
	{ .compatible = "technologic,wizard", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ts_wizard_i2c_of_match);

static struct i2c_driver ts_wizard_i2c_driver = {
	.driver = {
		.name = "tswizard-core",
		.of_match_table = of_match_ptr(ts_wizard_i2c_of_match),
	},
	.probe = ts_wizard_i2c_probe,
	.id_table = ts_wizard_i2c_id,
};
module_i2c_driver(ts_wizard_i2c_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_DESCRIPTION("Core driver for embeddedTS Wizard microcontroller");
MODULE_LICENSE("GPL v2");
