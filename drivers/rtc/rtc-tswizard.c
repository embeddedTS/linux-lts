// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTC for embeddedts's wizard microcontroller
 * Alarm does not have an IRQ, controls wakeup or reset from the external
 * wizard microcontroller.
 */

#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc/ds1307.h>
#include <linux/rtc.h>
#include <linux/string.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regmap.h>

#define RTC_MAGIC       0x0
#define     RTC_MAGIC_VALUE 0xAA
#define RTC_RESERVED0   0x1
#define RTC_FLAGS       0x2
#define     RTC_FLAGS_OF            (1 << 7) /* Cleared writing any new epoch value */
#define     RTC_FLAGS_ALARM_EN      (1 << 6)
#define     RTC_FLAGS_ALARM_TRIPPED (1 << 5) /* Clear by writing RTC Alarm regs */
#define     RTC_FLAGS_ALARM_REBOOT  (1 << 4)
#define     RTC_FLAGS_BATT_PRESENT  (1 << 3)
#define RTC_RESERVED1   0x3
#define RTC_EPOCH_0     0x4
#define RTC_EPOCH_1     0x5
#define RTC_EPOCH_2     0x6
#define RTC_EPOCH_3     0x7
#define RTC_ALARM_0     0x12
#define RTC_ALARM_1     0x13
#define RTC_ALARM_2     0x14
#define RTC_ALARM_3     0x15
#define RTC_PPB_0       0x1A
#define RTC_PPB_1       0x1B
#define RTC_PPB_2       0x1C
#define RTC_PPB_3       0x1D
#define RTC_PPB_CTL     0x1E
#define     RTC_PPB_CTL_SIGN (1 << 1) /* 1 = positive, 0 = negative */
#define     RTC_PPB_CTL_EN   (1 << 0) /* Calibration is applied anytime this is 1*/

struct wizard_rtc {
	struct device		*dev;
	struct regmap		*regmap;
	const char		*name;
	struct rtc_device	*rtc;
};

static int wizard_rtc_get_time(struct device *dev, struct rtc_time *tm)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	time64_t timestamp = 0;
	unsigned int reg;
	int ret;

	ret = regmap_read(wiz->regmap, RTC_FLAGS, &reg);
	if (ret)
		return ret;

	/* RTC has invalid time */
	if (reg & RTC_FLAGS_OF)
		return -EINVAL;

	ret = regmap_bulk_read(wiz->regmap, RTC_EPOCH_0, &timestamp, 4);
	if (ret)
		return ret;

	rtc_time64_to_tm(timestamp, tm);

	return 0;
}

static int wizard_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	time64_t timestamp = rtc_tm_to_time64(tm);

	return regmap_bulk_write(wiz->regmap, RTC_EPOCH_0, &timestamp, 4);
}

static int wizard_rtc_read_offset(struct device *dev, long *offset)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	unsigned int ctrl_reg;
	uint32_t ppb;
	int ret;

	ret = regmap_bulk_read(wiz->regmap, RTC_PPB_0, &ppb, sizeof(ppb));
	if (ret)
		return ret;

	ret = regmap_read(wiz->regmap, RTC_PPB_CTL, &ctrl_reg);
	if (ret)
		return ret;

	/* check if positive */
	if (ctrl_reg & RTC_PPB_CTL_SIGN)
		*offset = (ppb);
	else
		*offset = -(ppb);

	return 0;
}

static int wizard_rtc_set_offset(struct device *dev, long offset)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	unsigned int ctrl_reg = RTC_PPB_CTL_EN;
	u32 ppb = (uint32_t)offset;
	int ret;

	ret = regmap_bulk_write(wiz->regmap, RTC_PPB_0, &ppb, sizeof(ppb));
	if (ret)
		return ret;

	if (offset >= 0)
		ctrl_reg |= RTC_PPB_CTL_SIGN;

	return regmap_update_bits(wiz->regmap, RTC_FLAGS,
				  RTC_PPB_CTL_SIGN | RTC_PPB_CTL_EN,
				  ctrl_reg);
}

static const struct rtc_class_ops wizard_rtc_ops = {
	.read_time      = wizard_rtc_get_time,
	.set_time       = wizard_rtc_set_time,
	.read_offset	= wizard_rtc_read_offset,
	.set_offset	= wizard_rtc_set_offset,
};

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 32,
};

static ssize_t alarm_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	unsigned long timestamp;
	int ret;

	ret = kstrtoul(buf, 0, &timestamp);
	if (ret)
		return ret;

	ret = regmap_bulk_write(wiz->regmap, RTC_ALARM_0, &timestamp, 4);
	if (ret)
		return ret;

	return count;
}

static ssize_t alarm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	u32 timestamp;
	int len;
	int ret;

	ret = regmap_bulk_read(wiz->regmap, RTC_ALARM_0, &timestamp, 4);
	if (ret)
		return ret;

	len = sprintf(buf, "%d\n", timestamp);

	return len;
}
static DEVICE_ATTR_RW(alarm);

static ssize_t alarm_en_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	unsigned int ctrl_reg = 0;
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	if (en)
		ctrl_reg |= RTC_FLAGS_ALARM_EN;

	ret = regmap_update_bits(wiz->regmap, RTC_FLAGS,
				  RTC_FLAGS_ALARM_EN,
				  ctrl_reg);

	return ret ? ret : count;
}

static ssize_t alarm_en_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wiz->regmap, RTC_FLAGS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & RTC_FLAGS_ALARM_EN));
	return ret;
}
static DEVICE_ATTR_RW(alarm_en);

static ssize_t alarm_cause_reboot_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	unsigned int ctrl_reg = 0;
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	if (en)
		ctrl_reg |= RTC_FLAGS_ALARM_REBOOT;

	ret = regmap_update_bits(wiz->regmap, RTC_FLAGS,
				  RTC_FLAGS_ALARM_REBOOT,
				  ctrl_reg);

	return ret ? ret : count;
}

static ssize_t alarm_cause_reboot_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wiz->regmap, RTC_FLAGS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & RTC_FLAGS_ALARM_REBOOT));
	return ret;
}
static DEVICE_ATTR_RW(alarm_cause_reboot);

static ssize_t batt_present_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct wizard_rtc *wiz = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wiz->regmap, RTC_FLAGS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & RTC_FLAGS_BATT_PRESENT));
	return ret;
}
static DEVICE_ATTR_RO(batt_present);

static struct attribute *wizard_rtc_sysfs_entries[] = {
	&dev_attr_alarm_en.attr,
	&dev_attr_alarm.attr,
	&dev_attr_alarm_cause_reboot.attr,
	&dev_attr_batt_present.attr,
	NULL,
};

static struct attribute_group wizard_rtc_attr_group = {
	.attrs	= wizard_rtc_sysfs_entries,
};

static int wizard_rtc_probe(struct i2c_client *client)
{
	struct wizard_rtc *wiz;
	struct device *dev = &client->dev;
	int err = -ENODEV;

	wiz = devm_kzalloc(dev, sizeof(struct wizard_rtc), GFP_KERNEL);
	if (!wiz)
		return -ENOMEM;

	dev_set_drvdata(dev, wiz);
	wiz->dev = dev;
	wiz->name = client->name;
	wiz->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(wiz->regmap)) {
		dev_err(dev, "regmap allocation failed\n");
		return PTR_ERR(wiz->regmap);
	}

	i2c_set_clientdata(client, wiz);

	wiz->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(wiz->rtc))
		return PTR_ERR(wiz->rtc);

	err = sysfs_create_group(&dev->kobj, &wizard_rtc_attr_group);
	if (err)
		dev_warn(dev, "error creating sysfs entries\n");

	wiz->rtc->ops = &wizard_rtc_ops;
	err = devm_rtc_register_device(wiz->rtc);
	if (err)
		return err;

	return 0;
}

static const struct i2c_device_id tswizard_rtc_id[] = {
	{ "tswizard_rtc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tswizard_rtc_id);

static const struct of_device_id tswizard_rtc_of_match[] = {
	{ .compatible = "technologic,wizard-rtc", },
	{ }
};
MODULE_DEVICE_TABLE(of, tswizard_rtc_of_match);

static struct i2c_driver wizard_rtc_driver = {
	.driver = {
		.name	= "rtc-tswizard",
		.of_match_table = of_match_ptr(tswizard_rtc_of_match),
	},
	.probe		= wizard_rtc_probe,
	.id_table	= tswizard_rtc_id,
};

module_i2c_driver(wizard_rtc_driver);

MODULE_DESCRIPTION("RTC driver for embeddedTS wizard microcontroller");
MODULE_LICENSE("GPL");
