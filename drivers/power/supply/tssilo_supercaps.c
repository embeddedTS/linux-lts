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
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/mfd/ts_wizard.h>

#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

/*
 * SILO controller generations:
 * Gen 2: TS-9370/TS-4300 Wizard controller.
 * Gen 1: TS-7100/TS-7180 supervisor.
 * Gen 0: TS-4100/TS-7553-V2 legacy controller.
 */

/* Gen 2 Wizard SILO registers. */
#define GEN2_SILO_RESERVED0				(WIZARD_SILO_BASE + 0)
#define GEN2_SILO_STATUS				(WIZARD_SILO_BASE + 1)
#define GEN2_SILO_CONTROL				(WIZARD_SILO_BASE + 2)
#define GEN2_SILO_REQUESTED_CHG_CURRENT_MA		(WIZARD_SILO_BASE + 4)
#define GEN2_SILO_MAX_SUPPORTED_CHRG_CURRENT_MA	(WIZARD_SILO_BASE + 5)
#define GEN2_SILO_PCT_CHARGED				(WIZARD_SILO_BASE + 8)
#define GEN2_SILO_CRITICAL_PCT				(WIZARD_SILO_BASE + 9)
#define GEN2_SILO_STARTUP_REQUESTED_CHG_CURRENT_MA	(WIZARD_SILO_BASE + 12)
#define GEN2_SILO_MIN_PWR_ON_PCT			(WIZARD_SILO_BASE + 13)

#define GEN2_SILO_STATUS_CHARGING	BIT(0)
#define GEN2_SILO_STATUS_PWR_FAIL	BIT(15)
#define GEN2_SILO_STATUS_MODE_MASK	0x3E
#define GEN2_SILO_STATUS_MODE_SHIFT	1
#define GEN2_SILO_STATUS_MODE_DISABLED	0
#define GEN2_SILO_STATUS_MODE_CHARGING	1
#define GEN2_SILO_STATUS_MODE_FULL	2
#define GEN2_SILO_STATUS_MODE_DISCHARGING	3
#define GEN2_SILO_CONTROL_CHRG_EN	BIT(0)
#define GEN2_SILO_CONTROL_PWRUP		BIT(1)

/* Gen 1 TS-7100/TS-7180 supervisor registers and status bits. */
#define GEN1_SILO_VOLTAGE		16
#define GEN1_SILO_STATUS		22
#define GEN1_SILO_CURRENT		26
#define GEN1_SILO_POWER_FAIL		BIT(0)
#define GEN1_SILO_ENABLED		BIT(1)
#define GEN1_SILO_MET_MIN		BIT(2)
#define GEN1_SILO_CHARGING		BIT(3)
#define GEN1_SILO_MIN_CHARGE_MV		3680
#define GEN1_SILO_MAX_CHARGE_MV		4800
#define GEN1_SILO_MIN_CHARGE_CURRENT_UA	50000
#define GEN1_SILO_MAX_CHARGE_CURRENT_UA	900000

/* Gen 0 TS-4100/TS-7553-V2 controller ADC and command values. */
#define GEN0_SILO_ADC_MAX			1023
#define GEN0_SILO_VOLTAGE_FULL_SCALE_UV		5000000
#define GEN0_SILO_CAPACITY_FULL_RAW		237
#define GEN0_SILO_CAPACITY_OFFSET		311
#define GEN0_SILO_ENABLE_COMMAND		1
#define GEN0_SILO_DISABLE_COMMAND		0

struct gen2_silo {
	struct regmap *regmap;
	struct power_supply *psy;
};

static int gen2_get_pct_charged(struct gen2_silo *data)
{
	int ret;
	unsigned int val;

	ret = regmap_read(data->regmap, GEN2_SILO_PCT_CHARGED, &val);
	if (ret) {
		dev_err(&data->psy->dev, "%s failed from regmap_read (rc=%d)\n", __func__, ret);
		return ret;
	}
	return val;
}

static ssize_t gen2_startup_charge_current_ma_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	struct gen2_silo *data = dev_get_drvdata(dev);
	int ret, val;

	ret = regmap_read(data->regmap, GEN2_SILO_STARTUP_REQUESTED_CHG_CURRENT_MA, &val);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", val);
}

static ssize_t gen2_startup_charge_current_ma_store(struct device *dev,
						   struct device_attribute *attr,
						   const char *buf, size_t count)
{
	struct gen2_silo *data = dev_get_drvdata(dev);
	unsigned int max;
	int ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, GEN2_SILO_MAX_SUPPORTED_CHRG_CURRENT_MA, &max);
	if (ret)
		return ret;
	if (val < 0 || val > max)
		return -EINVAL;

	ret = regmap_write(data->regmap, GEN2_SILO_STARTUP_REQUESTED_CHG_CURRENT_MA, val);
	if (ret)
		return ret;

	power_supply_changed(data->psy);

	return count;
}

static ssize_t gen2_min_power_on_pct_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct gen2_silo *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(data->regmap, GEN2_SILO_MIN_PWR_ON_PCT, &val);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", val);
}

static ssize_t gen2_min_power_on_pct_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct gen2_silo *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 100)
		return -EINVAL;

	ret = regmap_write(data->regmap, GEN2_SILO_MIN_PWR_ON_PCT, val);
	if (ret)
		return ret;

	power_supply_changed(data->psy);

	return count;
}

static DEVICE_ATTR(startup_charge_current_ma, 0644,
		   gen2_startup_charge_current_ma_show,
		   gen2_startup_charge_current_ma_store);
static DEVICE_ATTR(min_power_on_pct, 0644, gen2_min_power_on_pct_show,
		   gen2_min_power_on_pct_store);

/*
 * Currently these are our properties that do not map to any standard
 * power supply properties.
 */
static struct attribute *gen2_attrs[] = {
	&dev_attr_startup_charge_current_ma.attr,
	&dev_attr_min_power_on_pct.attr,
	NULL,
};

static const struct attribute_group gen2_attr_group = {
	.attrs = gen2_attrs,
};

static int gen2_property_is_writable(struct power_supply *psy,
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

static enum power_supply_property gen2_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN
};

static int gen2_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	unsigned int reg;
	int ret;

	struct gen2_silo *data = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		ret = regmap_test_bits(data->regmap, GEN2_SILO_CONTROL, GEN2_SILO_CONTROL_CHRG_EN);
		if (ret < 0)
			return ret;
		val->intval = (ret ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO :
				POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE);
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(data->regmap, GEN2_SILO_STATUS, &reg);
		val->intval = !(reg & GEN2_SILO_STATUS_PWR_FAIL);
		return ret;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = gen2_get_pct_charged(data);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		return regmap_read(data->regmap, GEN2_SILO_CRITICAL_PCT, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(data->regmap, GEN2_SILO_STATUS, &reg);
		reg = (reg & GEN2_SILO_STATUS_MODE_MASK) >> GEN2_SILO_STATUS_MODE_SHIFT;

		switch (reg) {
		case GEN2_SILO_STATUS_MODE_DISABLED:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case GEN2_SILO_STATUS_MODE_CHARGING:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case GEN2_SILO_STATUS_MODE_FULL:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case GEN2_SILO_STATUS_MODE_DISCHARGING:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return regmap_read(data->regmap, GEN2_SILO_REQUESTED_CHG_CURRENT_MA, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return regmap_read(data->regmap, GEN2_SILO_MAX_SUPPORTED_CHRG_CURRENT_MA, &val->intval);
	default:
		return -EINVAL;
	}
	return -ENODATA;
}

static int gen2_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *val)
{
	struct gen2_silo *data = power_supply_get_drvdata(psy);
	unsigned int value;
	int ret;

	if (psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT) {
		ret = regmap_read(data->regmap, GEN2_SILO_MAX_SUPPORTED_CHRG_CURRENT_MA, &value);
		if (ret)
			return ret;
		if (val->intval < 0 || val->intval > value)
			return -EINVAL;
		ret = regmap_write(data->regmap, GEN2_SILO_REQUESTED_CHG_CURRENT_MA, val->intval);
	} else if (psp == POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN)
		ret = regmap_write(data->regmap, GEN2_SILO_CRITICAL_PCT, val->intval);
	else if (psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR) {
		value = ((val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) ?
				GEN2_SILO_CONTROL_CHRG_EN : 0);
		ret = regmap_update_bits(data->regmap, GEN2_SILO_CONTROL,
					 GEN2_SILO_CONTROL_CHRG_EN, value);
	} else
		return -EINVAL;

	if (!ret)
		power_supply_changed(data->psy);

	return ret;
}

static const struct power_supply_desc gen2_desc = {
	.name			= "silo",
	.type			= POWER_SUPPLY_TYPE_UPS,
	.properties		= gen2_props,
	.num_properties		= ARRAY_SIZE(gen2_props),
	.get_property		= gen2_get_property,
	.set_property		= gen2_set_property,
	.property_is_writeable	= gen2_property_is_writable,
	.no_thermal		= true,
	.charge_behaviours =
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

static irqreturn_t gen2_irq_handler(int irq, void *dev_id)
{
	struct gen2_silo *data = dev_id;

	power_supply_changed(data->psy);
	return IRQ_HANDLED;
}


struct gen1_silo {
	struct regmap *map;
	struct power_supply *psy;
	struct delayed_work work;
};

static int gen1_read_word(struct gen1_silo *s, unsigned int reg,
			  unsigned int *value)
{
	u8 bytes[2];
	int ret;

	ret = regmap_bulk_read(s->map, reg, bytes, sizeof(bytes));
	if (!ret)
		*value = (bytes[0] << 8) | bytes[1];
	return ret;
}

static int gen1_get(struct power_supply *psy, enum power_supply_property prop,
		    union power_supply_propval *val)
{
	struct gen1_silo *s = power_supply_get_drvdata(psy);
	unsigned int value;
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = gen1_read_word(s, GEN1_SILO_VOLTAGE, &value);
		if (ret)
			return ret;
		if (prop == POWER_SUPPLY_PROP_VOLTAGE_NOW)
			val->intval = value * 1000;
		else
			/* Match the voltage estimate used by tsmicroctl. */
			val->intval = (clamp_val(value, GEN1_SILO_MIN_CHARGE_MV,
						 GEN1_SILO_MAX_CHARGE_MV) -
				       GEN1_SILO_MIN_CHARGE_MV) * 100 /
				(GEN1_SILO_MAX_CHARGE_MV - GEN1_SILO_MIN_CHARGE_MV);
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = gen1_read_word(s, GEN1_SILO_CURRENT, &value);
		if (!ret)
			val->intval = value * 1000;
		return ret;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = GEN1_SILO_MAX_CHARGE_CURRENT_UA;
		return 0;
	default:
		break;
	}

	ret = regmap_read(s->map, GEN1_SILO_STATUS, &value);
	if (ret)
		return ret;
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !(value & GEN1_SILO_POWER_FAIL);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = value & GEN1_SILO_ENABLED ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO :
			POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		if (value & GEN1_SILO_POWER_FAIL)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!(value & GEN1_SILO_ENABLED))
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (value & GEN1_SILO_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gen1_set(struct power_supply *psy, enum power_supply_property prop,
		    const union power_supply_propval *val)
{
	struct gen1_silo *s = power_supply_get_drvdata(psy);
	unsigned int value;
	u8 bytes[2];
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (val->intval < GEN1_SILO_MIN_CHARGE_CURRENT_UA ||
		    val->intval > GEN1_SILO_MAX_CHARGE_CURRENT_UA || val->intval % 1000)
			return -EINVAL;
		value = val->intval / 1000;
		bytes[0] = value >> 8;
		bytes[1] = value;
		ret = regmap_bulk_write(s->map, GEN1_SILO_CURRENT, bytes, sizeof(bytes));
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
			return -EINVAL;
		value = val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO ? GEN1_SILO_ENABLED : 0;
		ret = regmap_update_bits(s->map, GEN1_SILO_STATUS,
					 GEN1_SILO_ENABLED | GEN1_SILO_MET_MIN, value);
		break;
	default:
		return -EINVAL;
	}
	if (!ret)
		power_supply_changed(psy);
	return ret;
}

static int gen1_writable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR ||
		prop == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
}

static enum power_supply_property gen1_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static const struct power_supply_desc gen1_desc = {
	.name = "silo",
	.type = POWER_SUPPLY_TYPE_UPS,
	.properties = gen1_props,
	.num_properties = ARRAY_SIZE(gen1_props),
	.get_property = gen1_get,
	.set_property = gen1_set,
	.property_is_writeable = gen1_writable,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

static void gen1_poll(struct work_struct *work)
{
	struct gen1_silo *s = container_of(to_delayed_work(work),
					    struct gen1_silo, work);

	power_supply_changed(s->psy);
	schedule_delayed_work(&s->work, HZ);
}

static void gen1_stop(void *arg)
{
	struct gen1_silo *s = arg;

	cancel_delayed_work_sync(&s->work);
}

static int gen1_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = {};
	struct device *dev = &pdev->dev;
	struct gen1_silo *s;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->map = dev_get_regmap(dev->parent, NULL);
	if (!s->map)
		return -ENODEV;
	cfg.drv_data = s;
	s->psy = devm_power_supply_register(dev, &gen1_desc, &cfg);
	if (IS_ERR(s->psy))
		return PTR_ERR(s->psy);
	INIT_DELAYED_WORK(&s->work, gen1_poll);
	ret = devm_add_action_or_reset(dev, gen1_stop, s);
	if (ret)
		return ret;
	schedule_delayed_work(&s->work, HZ);
	return 0;
}

/* Gen 0 controllers send an unaddressed snapshot; a byte write is a command. */
struct gen0_silo {
	struct i2c_client *client;
	struct gpio_desc *power_fail;
	struct power_supply *psy;
	struct delayed_work work;
};

static int gen0_get(struct power_supply *psy, enum power_supply_property prop,
		     union power_supply_propval *val)
{
	struct gen0_silo *s = power_supply_get_drvdata(psy);
	unsigned int raw;
	u8 data[4];
	int ret;

	if (prop == POWER_SUPPLY_PROP_ONLINE || prop == POWER_SUPPLY_PROP_STATUS) {
		ret = gpiod_get_value_cansleep(s->power_fail);
		if (ret < 0)
			return ret;
		val->intval = prop == POWER_SUPPLY_PROP_ONLINE ? !ret :
			(ret ? POWER_SUPPLY_STATUS_DISCHARGING : POWER_SUPPLY_STATUS_UNKNOWN);
		return 0;
	}
	if (prop == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR)
		return -ENODATA;
	ret = i2c_master_recv(s->client, data, sizeof(data));
	if (ret != sizeof(data))
		return ret < 0 ? ret : -EIO;
	raw = (data[2] << 8) | data[3];
	if (raw > GEN0_SILO_ADC_MAX)
		return -ERANGE;
	if (prop == POWER_SUPPLY_PROP_VOLTAGE_NOW)
		val->intval = div_u64((u64)raw * GEN0_SILO_VOLTAGE_FULL_SCALE_UV,
				      GEN0_SILO_ADC_MAX);
	else if (prop == POWER_SUPPLY_PROP_CAPACITY)
		val->intval = clamp_val((int)(raw * 100 / GEN0_SILO_CAPACITY_FULL_RAW) -
				       GEN0_SILO_CAPACITY_OFFSET, 0, 100);
	else
		return -EINVAL;
	return 0;
}

static int gen0_set(struct power_supply *psy, enum power_supply_property prop,
		     const union power_supply_propval *val)
{
	struct gen0_silo *s = power_supply_get_drvdata(psy);
	u8 command;
	int ret;

	if (prop != POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR)
		return -EINVAL;
	if (val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO)
		command = GEN0_SILO_ENABLE_COMMAND;
	else if (val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
		command = GEN0_SILO_DISABLE_COMMAND;
	else
		return -EINVAL;
	/* On TS-4100, either command also clears push-switch reset enable. */
	ret = i2c_master_send(s->client, &command, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;
	power_supply_changed(psy);
	return 0;
}

static int gen0_writable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static enum power_supply_property gen0_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

static const struct power_supply_desc gen0_desc = {
	.name = "silo",
	.type = POWER_SUPPLY_TYPE_UPS,
	.properties = gen0_props,
	.num_properties = ARRAY_SIZE(gen0_props),
	.get_property = gen0_get,
	.set_property = gen0_set,
	.property_is_writeable = gen0_writable,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
	.no_thermal = true,
};

static void gen0_poll(struct work_struct *work)
{
	struct gen0_silo *s = container_of(to_delayed_work(work),
					     struct gen0_silo, work);

	power_supply_changed(s->psy);
	schedule_delayed_work(&s->work, HZ);
}

static void gen0_stop(void *arg)
{
	struct gen0_silo *s = arg;

	cancel_delayed_work_sync(&s->work);
}

static int gen0_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	struct gen0_silo *s;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->client = to_i2c_client(dev->parent);
	s->power_fail = devm_gpiod_get(dev, "power-fail", GPIOD_IN);
	if (IS_ERR(s->power_fail))
		return dev_err_probe(dev, PTR_ERR(s->power_fail), "power-fail GPIO\n");
	cfg.drv_data = s;
	s->psy = devm_power_supply_register(dev, &gen0_desc, &cfg);
	if (IS_ERR(s->psy))
		return PTR_ERR(s->psy);
	INIT_DELAYED_WORK(&s->work, gen0_poll);
	ret = devm_add_action_or_reset(dev, gen0_stop, s);
	if (ret)
		return ret;
	schedule_delayed_work(&s->work, HZ);
	return 0;
}

static int ts_silo_probe(struct platform_device *pdev)
{
	struct ts_wizard *wizard;
	struct device *dev = &pdev->dev;
	struct gen2_silo *data;
	struct power_supply_config psy_cfg = {};
	int ret;
	int irq;

	if (of_device_is_compatible(dev->of_node, "technologic,ts4100-silo") ||
	    of_device_is_compatible(dev->of_node, "technologic,ts7553v2-silo"))
		return gen0_probe(pdev);
	if (of_device_is_compatible(dev->of_node, "technologic,ts7100-silo"))
		return gen1_probe(pdev);
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
	data->psy = devm_power_supply_register(dev, &gen2_desc, &psy_cfg);
	if (IS_ERR(data->psy)) {
		dev_err(dev, "devm_power_supply_register failed (rc=%pe)", data->psy);
		return PTR_ERR(data->psy);
	}

	ret = devm_device_add_group(dev, &gen2_attr_group);
	if (ret) {
		dev_err(dev, "failed to add sysfs group (rc=%d)\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, irq,
					NULL, gen2_irq_handler,
					IRQF_ONESHOT, dev_name(dev), data);
	return ret;
}

static const struct of_device_id silo_of_match[] = {
	{ .compatible = "technologic,wizard-silo", },
	{ .compatible = "technologic,ts7100-silo", },
	{ .compatible = "technologic,ts4100-silo", },
	{ .compatible = "technologic,ts7553v2-silo", },
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
