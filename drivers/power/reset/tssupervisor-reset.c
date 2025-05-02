// SPDX-License-Identifier: GPL-2.0-only
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/reboot.h>
#include <linux/mfd/ts_supervisor.h>

/* We need a static device to support this for shutdown/reboot hooks */
static struct device *ts_rstc_device;
static atomic_t ts_restart_nb_refcnt = ATOMIC_INIT(0);

static ssize_t reboot_reason_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_supervisor *super = dev_get_drvdata(dev);
	uint32_t reason;
	int len, err;

	err = regmap_read(super->regmap, SUPER_REBOOT_REASON, &reason);
	if (err < 0)
		dev_err(dev, "error reading reg %u", SUPER_REBOOT_REASON);

	switch (reason) {
	case REBOOT_REASON_POR:
		len = sprintf(buf, "POR\n");
		break;
	case REBOOT_REASON_CPU_WDT:
		len = sprintf(buf, "CPU WDT\n");
		break;
	case REBOOT_REASON_SOFTWARE_REBOOT:
		len = sprintf(buf, "Software Reboot\n");
		break;
	case REBOOT_REASON_BROWNOUT:
		len = sprintf(buf, "Brownout\n");
		break;
	case REBOOT_REASON_RTC_ALARM_REBOOT:
		len = sprintf(buf, "RTC Alarm Reboot\n");
		break;
	case REBOOT_REASON_WAKE_FROM_PWR_CYCLE:
		len = sprintf(buf, "Wake from PWR Cycle\n");
		break;
	case REBOOT_REASON_WAKE_FROM_WAKE_SIGNAL:
		len = sprintf(buf, "Wake from WAKE_EN\n");
		break;
	case REBOOT_REASON_WAKE_FROM_RTC_ALARM:
		len = sprintf(buf, "Wake from RTC Alarm\n");
		break;
	case REBOOT_REASON_WAKE_FROM_USB_VBUS:
		len = sprintf(buf, "Wake from USB VBUS\n");
		break;
	default:
		len = sprintf(buf, "Unknown\n");
		break;
	}

	return len;
}
static DEVICE_ATTR_RO(reboot_reason);

static struct attribute *ts_supervisor_sysfs_entries[] = {
	&dev_attr_reboot_reason.attr,
	NULL,
};

static struct attribute_group ts_supervisor_attr_group = {
	.attrs	= ts_supervisor_sysfs_entries,
};

static int ts_supervisor_restart(struct notifier_block *this,
				 unsigned long mode,
				 void *cmd)
{
	int err = -ENOENT;
	struct ts_supervisor *super = dev_get_drvdata(ts_rstc_device);

	if (super) {
		err = regmap_write(super->regmap, SUPER_CMDS, I2C_REBOOT);
		if (!err)
			mdelay(1000);
	}

	dev_emerg(ts_rstc_device, "reset controller could not cause a reset!");

	return NOTIFY_DONE;
}

static struct notifier_block ts_supervisor_restart_nb = {
	.notifier_call = ts_supervisor_restart,
	.priority = 128,
};

static void ts_supervisor_poweroff(void)
{
	int err = -ENOENT;
	struct ts_supervisor *super = dev_get_drvdata(ts_rstc_device);

	if (super) {
		err = regmap_write(super->regmap, SUPER_CMDS, I2C_HALT);
		if (!err)
			mdelay(1000);
	}

	dev_emerg(ts_rstc_device, "Unable to call halt (%d)", err);
}

static int ts_supervisor_rstc_probe(struct platform_device *pdev)
{
	struct ts_supervisor *super = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	uint32_t features;
	int err = 0;

	err = regmap_read(super->regmap, SUPER_FEATURES0, &features);
	if (err < 0)
		dev_err(dev, "error reading reg %u", SUPER_FEATURES0);

	if ((features & SUPER_FEAT_RSTC) == 0) {
		/* Reset controller not supported on this supervisor */
		return 0;
	}

	dev_set_drvdata(dev, super);
	if (atomic_inc_return(&ts_restart_nb_refcnt) == 1) {
		ts_rstc_device = dev;
		pm_power_off = ts_supervisor_poweroff;

		err = register_restart_handler(&ts_supervisor_restart_nb);
		if (err) {
			dev_err(dev, "cannot register restart handler (err=%d)\n", err);
			atomic_dec(&ts_restart_nb_refcnt);
			return err;
		}
	} else {
		err = EEXIST;
		dev_err(dev, "rstc already registered");
	}

	err = sysfs_create_group(&dev->kobj, &ts_supervisor_attr_group);
	if (err)
		dev_warn(dev, "error creating sysfs entries\n");

	dev_info(dev, "Using supervisor for reset controller");

	return 0;
}

static struct platform_driver tssupervisor_rstc_driver = {
	.driver = {
		.name = "tssupervisor-reset",
	},
	.probe = ts_supervisor_rstc_probe,
};

module_platform_driver(tssupervisor_rstc_driver);

MODULE_DESCRIPTION("embeddedTS supervisor reset controller driver");
MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_LICENSE("GPL");
