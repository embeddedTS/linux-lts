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

#define SILO_RESERVED0					(WIZARD_SILO_BASE + 0)
#define SILO_STATUS					(WIZARD_SILO_BASE + 1)
#define SILO_CONTROL					(WIZARD_SILO_BASE + 2)

#define SILO_REQUESTED_CHG_CURRENT_MA			(WIZARD_SILO_BASE + 4)
#define SILO_MAX_SUPPORTED_CHRG_CURRENT_MA		(WIZARD_SILO_BASE + 5)

#define SILO_PCT_CHARGED				(WIZARD_SILO_BASE + 8)
#define SILO_CRITICAL_PCT				(WIZARD_SILO_BASE + 9)

#define SILO_STARTUP_REQUESTED_CHG_CURRENT_MA		(WIZARD_SILO_BASE + 12)
#define SILO_MIN_PWR_ON_PCT				(WIZARD_SILO_BASE + 13)

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
		val->intval = (ret ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO :
				POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE);
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
	} else if (psp == POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN)
		ret = regmap_write(data->regmap, SILO_CRITICAL_PCT, val->intval);
	else if (psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR) {
		value = ((val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) ?
				SILO_CONTROL_CHRG_EN : 0);
		ret = regmap_update_bits(data->regmap, SILO_CONTROL,
					 SILO_CONTROL_CHRG_EN, value);
	} else
		return -EINVAL;

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
	.charge_behaviours =
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

static irqreturn_t silo_irq_handler(int irq, void *dev_id)
{
	struct silo_data *data = dev_id;

	power_supply_changed(data->psy);
	return IRQ_HANDLED;
}

#define MICRO_VOLTAGE 16
#define MICRO_STATUS 22
#define MICRO_CURRENT 26
#define MICRO_POWER_FAIL BIT(0)
#define MICRO_ENABLED BIT(1)
#define MICRO_MET_MIN BIT(2)
#define MICRO_CHARGING BIT(3)

struct micro_supply {
	struct regmap *map;
	struct power_supply *psy;
	struct delayed_work work;
};

static int micro_read_word(struct micro_supply *s, unsigned int reg,
			   unsigned int *value)
{
	u8 bytes[2];
	int ret;

	ret = regmap_bulk_read(s->map, reg, bytes, sizeof(bytes));
	if (!ret)
		*value = (bytes[0] << 8) | bytes[1];
	return ret;
}

static int micro_get(struct power_supply *psy, enum power_supply_property prop,
		     union power_supply_propval *val)
{
	struct micro_supply *s = power_supply_get_drvdata(psy);
	unsigned int value;
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = micro_read_word(s, MICRO_VOLTAGE, &value);
		if (ret)
			return ret;
		if (prop == POWER_SUPPLY_PROP_VOLTAGE_NOW)
			val->intval = value * 1000;
		else
			/* Match the voltage estimate used by tsmicroctl. */
			val->intval = (clamp_val(value, 3680, 4800) - 3680) * 100 / 1120;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = micro_read_word(s, MICRO_CURRENT, &value);
		if (!ret)
			val->intval = value * 1000;
		return ret;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = 900000;
		return 0;
	default:
		break;
	}

	ret = regmap_read(s->map, MICRO_STATUS, &value);
	if (ret)
		return ret;
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !(value & MICRO_POWER_FAIL);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = value & MICRO_ENABLED ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO :
			POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		if (value & MICRO_POWER_FAIL)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!(value & MICRO_ENABLED))
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (value & MICRO_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	default:
		return -EINVAL;
	}
}

static int micro_set(struct power_supply *psy, enum power_supply_property prop,
		     const union power_supply_propval *val)
{
	struct micro_supply *s = power_supply_get_drvdata(psy);
	unsigned int value;
	u8 bytes[2];
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (val->intval < 50000 || val->intval > 900000 || val->intval % 1000)
			return -EINVAL;
		value = val->intval / 1000;
		bytes[0] = value >> 8;
		bytes[1] = value;
		ret = regmap_bulk_write(s->map, MICRO_CURRENT, bytes, sizeof(bytes));
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
			return -EINVAL;
		value = val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO ? MICRO_ENABLED : 0;
		ret = regmap_update_bits(s->map, MICRO_STATUS,
					 MICRO_ENABLED | MICRO_MET_MIN, value);
		break;
	default:
		return -EINVAL;
	}
	if (!ret)
		power_supply_changed(psy);
	return ret;
}

static int micro_writable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR ||
		prop == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
}

static enum power_supply_property micro_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static const struct power_supply_desc micro_desc = {
	.name = "silo",
	.type = POWER_SUPPLY_TYPE_UPS,
	.properties = micro_props,
	.num_properties = ARRAY_SIZE(micro_props),
	.get_property = micro_get,
	.set_property = micro_set,
	.property_is_writeable = micro_writable,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

static void micro_poll(struct work_struct *work)
{
	struct micro_supply *s = container_of(to_delayed_work(work),
					     struct micro_supply, work);

	power_supply_changed(s->psy);
	schedule_delayed_work(&s->work, HZ);
}

static void micro_stop(void *arg)
{
	struct micro_supply *s = arg;

	cancel_delayed_work_sync(&s->work);
}

static int micro_silo_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = {};
	struct device *dev = &pdev->dev;
	struct micro_supply *s;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->map = dev_get_regmap(dev->parent, NULL);
	if (!s->map)
		return -ENODEV;
	cfg.drv_data = s;
	s->psy = devm_power_supply_register(dev, &micro_desc, &cfg);
	if (IS_ERR(s->psy))
		return PTR_ERR(s->psy);
	INIT_DELAYED_WORK(&s->work, micro_poll);
	ret = devm_add_action_or_reset(dev, micro_stop, s);
	if (ret)
		return ret;
	schedule_delayed_work(&s->work, HZ);
	return 0;
}

static int ts_silo_probe(struct platform_device *pdev)
{
	struct ts_wizard *wizard;
	struct device *dev = &pdev->dev;
	struct silo_data *data;
	struct power_supply_config psy_cfg = {};
	int ret;
	int irq;

	if (of_device_is_compatible(dev->of_node, "technologic,ts7100-silo"))
		return micro_silo_probe(pdev);
	if (dev->of_node &&
	    !of_device_is_compatible(dev->of_node, "technologic,wizard-silo"))
		return -ENODEV;
	wizard = dev_get_drvdata(dev->parent);
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
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

	ret = devm_device_add_group(dev, &silo_attr_group);
	if (ret) {
		dev_err(dev, "failed to add sysfs group (rc=%d)\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, irq,
					NULL, silo_irq_handler,
					IRQF_ONESHOT, dev_name(dev), data);
	return ret;
}

static const struct of_device_id silo_of_match[] = {
	{ .compatible = "technologic,wizard-silo", },
	{ .compatible = "technologic,ts7100-silo", },
	{}
};
MODULE_DEVICE_TABLE(of, silo_of_match);

static struct platform_driver silo_driver = {
	.driver = {
		.name = "silo",
		.of_match_table = silo_of_match,
	},
	.probe = ts_silo_probe,
};

module_platform_driver(silo_driver);

MODULE_DESCRIPTION("embeddedTS SILO supercaps driver");
MODULE_AUTHOR("Lionel D. Hummel <lionel@embeddedTS.com>");
MODULE_LICENSE("GPL");
