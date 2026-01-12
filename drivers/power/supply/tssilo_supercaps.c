// SPDX-License-Identifier: GPL-2.0
/*
 * Supercaps driver for the embeddedTS SILO controller
 * Copyright (C) 2024-2025 Technologic Systems, Inc. dba embeddedTS
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/mfd/ts_wizard.h>

#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define SILO_RESERVED0					(WIZ_SILO_BASE + 0)
#define SILO_STATUS					(WIZ_SILO_BASE + 1)
#define SILO_CONTROL					(WIZ_SILO_BASE + 2)

#define SILO_REQUESTED_CHG_CURRENT_MA			(WIZ_SILO_BASE + 4)
#define SILO_MAX_SUPPORTED_CHRG_CURRENT_MA		(WIZ_SILO_BASE + 5)

#define SILO_PCT_CHARGED				(WIZ_SILO_BASE + 8)
#define SILO_CRITICAL_PCT				(WIZ_SILO_BASE + 9)

#define SILO_STARTUP_REQUESTED_CHG_CURRENT_MA		(WIZ_SILO_BASE + 12)
#define SILO_MIN_PWR_ON_PCT				(WIZ_SILO_BASE + 13)

#define SILO_STATUS_CHARGING		BIT(0)
#define SILO_STATUS_PWR_FAIL		BIT(15)
#define SILO_STATUS_MODE_MASK		0x3E
#define SILO_STATUS_MODE_SHIFT		1

#define SILO_STATUS_MODE_DISABLED	0
#define SILO_STATUS_MODE_CHARGING	1
#define SILO_STATUS_MODE_FULL		2
#define SILO_STATUS_MODE_DISCHARGING	3

#define SILO_CONTROL_CHRG_EN		BIT(0)
#define SILO_CONTROL_PWRUP		BIT(1)

struct silo_data {
	struct regmap *regmap;
	struct power_supply *psy;
};

static int get_pct_charged(struct silo_data *data)
{
	int ret;
	unsigned int val;

	ret = regmap_read(data->regmap, SILO_PCT_CHARGED, &val);
	if (ret) {
		dev_err(&data->psy->dev, "%s failed from regmap_read (rc=%d)\n", __func__, ret);
		return ret;
	}
	return val;
}

static ssize_t startup_charge_current_ma_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct silo_data *data = dev_get_drvdata(dev);
	int ret, val;

	ret = regmap_read(data->regmap, SILO_STARTUP_REQUESTED_CHG_CURRENT_MA, &val);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", val);
}

static ssize_t startup_charge_current_ma_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct silo_data *data = dev_get_drvdata(dev);
	unsigned int max;
	int ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, SILO_MAX_SUPPORTED_CHRG_CURRENT_MA, &max);
	if (ret)
		return ret;
	if (val < 0 || val > max)
		return -EINVAL;

	ret = regmap_write(data->regmap, SILO_STARTUP_REQUESTED_CHG_CURRENT_MA, val);
	if (ret)
		return ret;

	power_supply_changed(data->psy);

	return count;
}

static ssize_t min_power_on_pct_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct silo_data *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(data->regmap, SILO_MIN_PWR_ON_PCT, &val);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", val);
}

static ssize_t min_power_on_pct_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct silo_data *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 100)
		return -EINVAL;

	ret = regmap_write(data->regmap, SILO_MIN_PWR_ON_PCT, val);
	if (ret)
		return ret;

	power_supply_changed(data->psy);

	return count;
}

static DEVICE_ATTR_RW(startup_charge_current_ma);
static DEVICE_ATTR_RW(min_power_on_pct);

/*
 * Currently these are our properties that do not map to any standard
 * power supply properties.
 */
static struct attribute *silo_attrs[] = {
	&dev_attr_startup_charge_current_ma.attr,
	&dev_attr_min_power_on_pct.attr,
	NULL,
};

static const struct attribute_group silo_attr_group = {
	.attrs = silo_attrs,
};

static int silo_property_is_writable(struct power_supply *psy,
				       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		return 1;
	default:
		break;
	}
	return 0;
}

static enum power_supply_property silo_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN
};

static int silo_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	unsigned int reg;
	int ret;

	struct silo_data *data = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		ret = regmap_test_bits(data->regmap, SILO_CONTROL, SILO_CONTROL_CHRG_EN);
		if (ret < 0)
			return ret;
		val->intval = (ret ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO : POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE);
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(data->regmap, SILO_STATUS, &reg);
		val->intval = !(reg & SILO_STATUS_PWR_FAIL);
		return ret;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = get_pct_charged(data);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		return regmap_read(data->regmap, SILO_CRITICAL_PCT, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(data->regmap, SILO_STATUS, &reg);
		reg = (reg & SILO_STATUS_MODE_MASK) >> SILO_STATUS_MODE_SHIFT;

		switch (reg) {
		case SILO_STATUS_MODE_DISABLED:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case SILO_STATUS_MODE_CHARGING:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case SILO_STATUS_MODE_FULL:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case SILO_STATUS_MODE_DISCHARGING:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return regmap_read(data->regmap, SILO_REQUESTED_CHG_CURRENT_MA, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return regmap_read(data->regmap, SILO_MAX_SUPPORTED_CHRG_CURRENT_MA, &val->intval);
	default:
		return -EINVAL;
	}
	return -ENODATA;
}

static int silo_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct silo_data *data = power_supply_get_drvdata(psy);
	unsigned int value;
	int ret;

	if (psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT) {
		ret = regmap_read(data->regmap, SILO_MAX_SUPPORTED_CHRG_CURRENT_MA, &value);
		if (ret)
			return ret;
		if (val->intval < 0 || val->intval > value)
			return -EINVAL;
		ret = regmap_write(data->regmap, SILO_REQUESTED_CHG_CURRENT_MA, val->intval);
	} else if (psp == POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN) {
		ret = regmap_write(data->regmap, SILO_CRITICAL_PCT, val->intval);
	} else if (psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR) {
		value = ((val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) ? SILO_CONTROL_CHRG_EN : 0);
		ret = regmap_update_bits(data->regmap, SILO_CONTROL,
					 SILO_CONTROL_CHRG_EN, value);
	} else {
		return -EINVAL;
	}

	if (!ret)
		power_supply_changed(data->psy);

	return ret;
}

static const struct power_supply_desc silo_desc = {
	.name			= "silo",
	.type			= POWER_SUPPLY_TYPE_UPS,
	.properties		= silo_props,
	.num_properties		= ARRAY_SIZE(silo_props),
	.get_property		= silo_get_property,
	.set_property		= silo_set_property,
	.property_is_writeable	= silo_property_is_writable,
	.no_thermal		= true,
};

static irqreturn_t silo_irq_handler(int irq, void *dev_id)
{
	struct silo_data *data = dev_id;
	power_supply_changed(data->psy);
	return IRQ_HANDLED;
}

static int ts_silo_probe(struct platform_device *pdev)
{
	struct ts_wizard *wizard = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct silo_data *data;
	struct power_supply_config psy_cfg = {};
	int ret;
	int irq;
	unsigned int version;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	data->regmap = wizard->regmap;
	platform_set_drvdata(pdev, data);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	psy_cfg.drv_data = data;
	data->psy = devm_power_supply_register(dev, &silo_desc, &psy_cfg);
	if (IS_ERR(data->psy)) {
		dev_err(dev, "devm_power_supply_register failed (rc=%pe)", data->psy);
		return PTR_ERR(data->psy);
	}

	ret = sysfs_create_group(&dev->kobj, &silo_attr_group);
	if (ret) {
		dev_err(dev, "sysfs_create_group failed (rc=%d)\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, irq,
					NULL, silo_irq_handler,
					IRQF_ONESHOT, dev_name(dev), data);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, SILO_RESERVED0, &version);
	if (ret < 0)
		return ret;
	dev_info(dev, "TS-SILO version %d\n", version);
	if (version < 2)
		dev_warn(dev, "POWER_FAIL ignored without a Wizard interrupt controller.\n");
	return 0;
}

static int ts_silo_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &silo_attr_group);
	return 0;
}

static const struct of_device_id silo_of_match[] = {
	{ .compatible = "technologic,wizard-silo", },
	{}
};
MODULE_DEVICE_TABLE(of, silo_of_match);

static struct platform_driver silo_driver = {
	.driver = {
		.name = "silo",
		.of_match_table = silo_of_match,
	},
	.probe = ts_silo_probe,
	.remove = ts_silo_remove,
};

module_platform_driver(silo_driver);

MODULE_DESCRIPTION("embeddedTS SILO supercaps driver");
MODULE_AUTHOR("Lionel D. Hummel <lionel@embeddedTS.com>");
MODULE_LICENSE("GPL");
