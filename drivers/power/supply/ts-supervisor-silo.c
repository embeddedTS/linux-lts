// SPDX-License-Identifier: GPL-2.0-only
/*
 * Supercaps driver for embeddedTS legacy supervisors.
 * Copyright (C) 2024-2025 Technologic Systems, Inc. dba embeddedTS
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

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

static int ts_supervisor_silo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (of_device_is_compatible(dev->of_node, "technologic,ts4100-silo") ||
	    of_device_is_compatible(dev->of_node, "technologic,ts7553v2-silo"))
		return gen0_probe(pdev);
	if (of_device_is_compatible(dev->of_node, "technologic,ts7100-silo"))
		return gen1_probe(pdev);

	return -ENODEV;
}

static const struct of_device_id ts_supervisor_silo_of_match[] = {
	{ .compatible = "technologic,ts7100-silo", },
	{ .compatible = "technologic,ts4100-silo", },
	{ .compatible = "technologic,ts7553v2-silo", },
	{}
};
MODULE_DEVICE_TABLE(of, ts_supervisor_silo_of_match);

static struct platform_driver ts_supervisor_silo_driver = {
	.driver = {
		.name = "ts-supervisor-silo",
		.of_match_table = ts_supervisor_silo_of_match,
	},
	.probe = ts_supervisor_silo_probe,
};
module_platform_driver(ts_supervisor_silo_driver);

MODULE_DESCRIPTION("embeddedTS legacy supervisor SILO driver");
MODULE_AUTHOR("Lionel D. Hummel <lionel@embeddedTS.com>");
MODULE_LICENSE("GPL");
