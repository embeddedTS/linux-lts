// SPDX-License-Identifier: GPL-2.0-only
/* embeddedTS supervisory controller system restart. */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

#define TS_SUPERVISOR_MAX_GEN1_SECONDS (U32_MAX / 100)
#define TS_SUPERVISOR_MAX_GEN0_SECONDS 0xffffff
#define TS_SUPERVISOR_GEN0_SLEEP_COMMAND 0x41
#define TS_SUPERVISOR_GEN1_COMMAND_ADDR 0x0400
#define TS_SUPERVISOR_GEN1_CMD_SLEEP BIT(1)
#define TS_SUPERVISOR_GEN1_TICKS_PER_SECOND 100
#define TS_SUPERVISOR_PUSH_SW_WAKE_EN BIT(1)

struct ts_supervisor_reset {
	struct i2c_client *client;
	int (*power_cycle)(struct ts_supervisor_reset *reset, u32 seconds);
	bool pushsw_wake_en;
};

static u8 ts_supervisor_pushsw_flags(struct ts_supervisor_reset *reset)
{
	if (READ_ONCE(reset->pushsw_wake_en))
		return TS_SUPERVISOR_PUSH_SW_WAKE_EN;
	return 0;
}

static int ts_supervisor_power_cycle_gen1(struct ts_supervisor_reset *reset,
					  u32 seconds)
{
	u32 ticks;
	u8 command[7] = { TS_SUPERVISOR_GEN1_COMMAND_ADDR >> 8,
			  TS_SUPERVISOR_GEN1_COMMAND_ADDR & 0xff };
	int ret;

	if (!seconds || seconds > TS_SUPERVISOR_MAX_GEN1_SECONDS)
		return -ERANGE;

	ticks = seconds * TS_SUPERVISOR_GEN1_TICKS_PER_SECOND;
	command[2] = ticks;
	command[3] = ticks >> 8;
	command[4] = ticks >> 16;
	command[5] = ticks >> 24;
	command[6] = TS_SUPERVISOR_GEN1_CMD_SLEEP;

	ret = i2c_master_send(reset->client, command, sizeof(command));
	if (ret < 0)
		return ret;
	if (ret != sizeof(command))
		return -EIO;

	return 0;
}

static int ts_supervisor_power_cycle_gen0(struct ts_supervisor_reset *reset,
					  u32 seconds)
{
	u8 command[4] = { TS_SUPERVISOR_GEN0_SLEEP_COMMAND |
			  ts_supervisor_pushsw_flags(reset),
			  seconds >> 16, seconds >> 8, seconds };
	int ret;

	if (!seconds || seconds > TS_SUPERVISOR_MAX_GEN0_SECONDS)
		return -ERANGE;

	ret = i2c_master_send(reset->client, command, sizeof(command));
	if (ret < 0)
		return ret;
	if (ret != sizeof(command))
		return -EIO;

	return 0;
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
	ret = reset->power_cycle(reset, seconds);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(power_cycle_seconds);

static ssize_t pushsw_wake_en_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ts_supervisor_reset *reset = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(reset->pushsw_wake_en));
}

static ssize_t pushsw_wake_en_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct ts_supervisor_reset *reset = dev_get_drvdata(dev);
	bool value;
	int ret;

	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;

	WRITE_ONCE(reset->pushsw_wake_en, value);
	return count;
}
static DEVICE_ATTR_RW(pushsw_wake_en);

static struct attribute *ts_supervisor_reset_attrs[] = {
	&dev_attr_power_cycle_seconds.attr,
	NULL,
};

static const struct attribute_group ts_supervisor_reset_group = {
	.attrs = ts_supervisor_reset_attrs,
};

static struct attribute *ts_supervisor_reset_pushsw_attrs[] = {
	&dev_attr_pushsw_wake_en.attr,
	NULL,
};

static const struct attribute_group ts_supervisor_reset_pushsw_group = {
	.attrs = ts_supervisor_reset_pushsw_attrs,
};

static int ts_supervisor_restart(struct sys_off_data *data)
{
	struct ts_supervisor_reset *reset = data->cb_data;
	int ret;

	dev_info(data->dev, "requesting supervisor restart\n");
	ret = reset->power_cycle(reset, 1);
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
	bool pushsw_wake_supported;
	int ret;

	if (!dev->of_node)
		return -ENODEV;
	reset = devm_kzalloc(dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;
	reset->client = to_i2c_client(dev->parent);
	pushsw_wake_supported =
		of_property_read_bool(dev->of_node,
				      "technologic,pushsw-wake-supported");
	reset->pushsw_wake_en = pushsw_wake_supported;
	if (of_device_is_compatible(dev->parent->of_node,
				    "technologic,ts-supervisor-gen1"))
		reset->power_cycle = ts_supervisor_power_cycle_gen1;
	else
		reset->power_cycle = ts_supervisor_power_cycle_gen0;
	platform_set_drvdata(pdev, reset);
	ret = devm_device_add_group(dev, &ts_supervisor_reset_group);
	if (ret)
		return ret;
	if (pushsw_wake_supported) {
		ret = devm_device_add_group(dev,
					   &ts_supervisor_reset_pushsw_group);
		if (ret)
			return ret;
	}
	if (of_property_read_bool(dev->of_node, "technologic,skip-micro-handler"))
		return 0;
	if (!reset->client->adapter->algo->xfer_atomic)
		return dev_err_probe(dev, -EOPNOTSUPP, "atomic I2C required for restart\n");

	return devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART, 192,
					     ts_supervisor_restart, reset);
}

static const struct of_device_id ts_supervisor_reset_match[] = {
	{ .compatible = "technologic,ts-supervisor-reset" },
	{ .compatible = "technologic,ts-supervisor-gen0-reset" },
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

MODULE_DESCRIPTION("embeddedTS supervisor restart driver");
MODULE_LICENSE("GPL");
