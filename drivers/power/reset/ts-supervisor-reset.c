// SPDX-License-Identifier: GPL-2.0-only
/* embeddedTS legacy supervisory controller system restart. */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

#define TS_SUPERVISOR_MAX_GEN1_SECONDS (U32_MAX / 100)
#define TS_SUPERVISOR_MAX_GEN0_SECONDS 0xffffff
#define TS_SUPERVISOR_MAX_SLEEP_SECONDS 1847297

enum ts_supervisor_reset_mode {
	TS_SUPERVISOR_RESET,
	TS_SUPERVISOR_SLEEP,
	TS_SUPERVISOR_STANDBY,
};

struct ts_supervisor_reset {
	struct i2c_client *client;
	bool gen1;
	enum ts_supervisor_reset_mode mode;
};

static int ts_supervisor_power_cycle(struct ts_supervisor_reset *reset, u32 seconds)
{
	u8 command[7];
	int len, ret;

	if (!seconds || (reset->gen1 &&
			 seconds > TS_SUPERVISOR_MAX_GEN1_SECONDS) ||
	    (!reset->gen1 && reset->mode == TS_SUPERVISOR_SLEEP &&
	     seconds > TS_SUPERVISOR_MAX_SLEEP_SECONDS) ||
	    (!reset->gen1 && reset->mode != TS_SUPERVISOR_SLEEP &&
	     seconds > TS_SUPERVISOR_MAX_GEN0_SECONDS))
		return -ERANGE;

	if (reset->gen1) {
		u32 ticks = seconds * 100;

		command[0] = 0x04;
		command[1] = 0x00;
		command[2] = ticks;
		command[3] = ticks >> 8;
		command[4] = ticks >> 16;
		command[5] = ticks >> 24;
		command[6] = BIT(1);
		len = 7;
	} else if (reset->mode == TS_SUPERVISOR_SLEEP) {
		command[0] = 0x41;
		command[1] = seconds >> 16;
		command[2] = seconds >> 8;
		command[3] = seconds;
		len = 4;
	} else if (reset->mode == TS_SUPERVISOR_STANDBY) {
		command[0] = 0x10;
		command[1] = seconds >> 16;
		command[2] = seconds >> 8;
		command[3] = seconds;
		len = 4;
	} else {
		command[0] = 0;
		command[1] = seconds >> 16;
		command[2] = seconds >> 8;
		command[3] = seconds;
		len = 4;
	}

	ret = i2c_master_send(reset->client, command, len);
	return ret < 0 ? ret : ret == len ? 0 : -EIO;
}

static ssize_t power_cycle_seconds_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct ts_supervisor_reset *reset = dev_get_drvdata(dev);
	unsigned int seconds;
	int ret;

	ret = kstrtouint(buf, 10, &seconds);
	if (ret)
		return ret;
	ret = ts_supervisor_power_cycle(reset, seconds);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(power_cycle_seconds);

static struct attribute *ts_supervisor_reset_attrs[] = {
	&dev_attr_power_cycle_seconds.attr,
	NULL,
};
static const struct attribute_group ts_supervisor_reset_group = {
	.attrs = ts_supervisor_reset_attrs,
};

static int ts_supervisor_restart(struct sys_off_data *data)
{
	struct ts_supervisor_reset *reset = data->cb_data;
	int ret;

	dev_info(data->dev, "requesting supervisor restart\n");
	ret = ts_supervisor_power_cycle(reset, 1);
	if (ret)
		dev_err(data->dev, "failed to request restart: %d\n", ret);
	else
		mdelay(2000);

	return NOTIFY_DONE;
}

static int ts_supervisor_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ts_supervisor_reset *reset;
	int ret;

	if (!dev->of_node)
		return -ENODEV;
	reset = devm_kzalloc(dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;
	reset->client = to_i2c_client(dev->parent);
	reset->gen1 = of_device_is_compatible(dev->parent->of_node,
					      "technologic,ts-supervisor-gen1");
	if (of_device_is_compatible(dev->of_node,
				    "technologic,ts-supervisor-sleep"))
		reset->mode = TS_SUPERVISOR_SLEEP;
	else if (of_device_is_compatible(dev->of_node,
					 "technologic,ts-supervisor-standby"))
		reset->mode = TS_SUPERVISOR_STANDBY;
	else
		reset->mode = TS_SUPERVISOR_RESET;
	platform_set_drvdata(pdev, reset);
	ret = devm_device_add_group(dev, &ts_supervisor_reset_group);
	if (ret)
		return ret;
	if (of_property_read_bool(dev->of_node, "technologic,skip-micro-handler"))
		return 0;
	if (!reset->client->adapter->algo->xfer_atomic)
		return dev_err_probe(dev, -EOPNOTSUPP, "atomic I2C required for restart\n");

	return devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART, 192,
					     ts_supervisor_restart, reset);
}

static const struct of_device_id ts_supervisor_reset_match[] = {
	{ .compatible = "technologic,ts-supervisor-reset" },
	{ .compatible = "technologic,ts-supervisor-sleep" },
	{ .compatible = "technologic,ts-supervisor-standby" },
	{ }
};
MODULE_DEVICE_TABLE(of, ts_supervisor_reset_match);

static struct platform_driver ts_supervisor_reset_driver = {
	.driver = {
		.name = "ts-supervisor-reset",
		.of_match_table = ts_supervisor_reset_match,
	},
	.probe = ts_supervisor_reset_probe,
};
module_platform_driver(ts_supervisor_reset_driver);

MODULE_DESCRIPTION("embeddedTS legacy supervisor restart driver");
MODULE_LICENSE("GPL");
