// SPDX-License-Identifier: GPL-2.0+
/*
 * embeddedTS supervisory microcontroller watchdog driver
 *
 * Copyright (C) 2017, 2023 Technologic Systems, Inc. dba embeddedTS
 *
 * Originally written for kernel 4.1, this has been updated with more
 * modern paradigms and does not behave quite the same as the original.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

#define TS_DEFAULT_TIMEOUT 30

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started default="
		__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct ts_wdt {
	struct i2c_client *client;
	struct watchdog_device wdd;
	bool gen1;
};

static struct ts_wdt *ts_wdt_poweroff_data;

/* The WDT expects 3 values:
 * 0 (command to feed)
 * and two bytes for the feed length in deciseconds
 * <MSB>
 * <LSB>
 * there are also 4 special values if they are specified
 * in the LSB with a 0 MSB:
 * 0 - 400 ms
 * 1 - 2.7 s
 * 2 - 10 s
 * 3 - disable watchdog
 *
 * Additionally, some platforms support a special poweroff/halt command
 * 0x40 (command to poweroff)
 * 0xAA
 * 0xAA
 */

static int ts_wdt_gen1_write(struct ts_wdt *ts_wdt, u32 ticks, u8 flags)
{
	u8 out[7] = {
		0x04, 0x00,
		ticks, ticks >> 8, ticks >> 16, ticks >> 24,
		flags,
	};
	int ret;

	ret = i2c_master_send(ts_wdt->client, out, sizeof(out));
	if (ret != sizeof(out))
		return ret < 0 ? ret : -EIO;
	return 0;
}

static int ts_wdt_write(struct ts_wdt *ts_wdt, u16 deciseconds)
{
	struct i2c_client *client = ts_wdt->client;
	u8 out[3];
	int ret;

	if (ts_wdt->gen1)
		return ts_wdt_gen1_write(ts_wdt, deciseconds * 10, BIT(0));

	out[0] = 0;
	out[1] = deciseconds >> 8;
	out[2] = deciseconds;

	ret = i2c_master_send(client, out, sizeof(out));
	if (ret != sizeof(out)) {
		dev_err(&client->dev, "%s: write error, ret=%d\n",
			__func__, ret);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int ts_wdt_start(struct watchdog_device *wdd)
{
	struct ts_wdt *ts_wdt = watchdog_get_drvdata(wdd);

	return ts_wdt_write(ts_wdt, wdd->timeout * 10);
}

static int ts_wdt_stop(struct watchdog_device *wdd)
{
	struct ts_wdt *ts_wdt = watchdog_get_drvdata(wdd);

	return ts_wdt_write(ts_wdt, ts_wdt->gen1 ? 0 : 3);
}

static int ts_wdt_restart(struct watchdog_device *wdd,
			unsigned long a, void *b)
{
	struct ts_wdt *ts_wdt = watchdog_get_drvdata(wdd);

	ts_wdt_write(ts_wdt, ts_wdt->gen1 ? 10 : 0);
	while (1)
		;

	return 0;
}

static void ts_wdt_poweroff(void)
{
	struct ts_wdt *ts_wdt = ts_wdt_poweroff_data;
	struct i2c_client *client = ts_wdt->client;

	u8 out[3];
	int ret;
	struct i2c_msg msg;

	if (ts_wdt->gen1) {
		ts_wdt_gen1_write(ts_wdt, 0, BIT(1));
		while (1)
			;
	}

	dev_dbg(&client->dev, "%s\n", __func__);

	ts_wdt_write(ts_wdt, 3);

	out[0] = 0x40;
	out[1] = 0xAA;
	out[2] = 0xAA;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = out;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "%s: write error, ret=%d\n",
			__func__, ret);
	}

	while (1)
		;
}

static int ts_set_timeout(struct watchdog_device *wdd,
				   unsigned int timeout)
{
	wdd->timeout = timeout;
	return 0;
}

static struct watchdog_info ts_wdt_info = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
				WDIOF_MAGICCLOSE,
	.identity	= "embeddedTS Micro Watchdog",
};

static struct watchdog_ops ts_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ts_wdt_start,
	.stop		= ts_wdt_stop,
	.set_timeout	= ts_set_timeout,
	.restart	= ts_wdt_restart,
};

static int ts_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ts_wdt *ts_wdt;
	bool enable_early = false;
	struct watchdog_device *wdd;
	bool skip_micro_handler;
	int err;

	ts_wdt = devm_kzalloc(dev, sizeof(*ts_wdt), GFP_KERNEL);
	if (!ts_wdt)
		return -ENOMEM;
	ts_wdt->client = client;
	ts_wdt->gen1 = of_device_is_compatible(dev->parent->of_node,
						       "technologic,ts7100-supervisor");
	wdd = &ts_wdt->wdd;

	wdd->info = &ts_wdt_info;
	wdd->ops = &ts_wdt_ops;
	wdd->min_timeout = 1;
	wdd->max_timeout = 6553;
	wdd->timeout = TS_DEFAULT_TIMEOUT;
	wdd->parent = dev;
	watchdog_init_timeout(wdd, 0, dev);

	if (of_property_read_bool(dev->of_node, "enable-early"))
		enable_early = true;
	skip_micro_handler =
		of_property_read_bool(dev->of_node,
				      "technologic,skip-micro-handler");

	watchdog_set_nowayout(wdd, nowayout);
	watchdog_set_drvdata(wdd, ts_wdt);

	/* We want this handler to be the first priority handler for reboots */
	if (!skip_micro_handler)
		watchdog_set_restart_priority(wdd, 255);

	/*
	 * On supported platforms either:
	 * The bootloader has already armed the WDT.
	 * -or-
	 * This platform's bootloader doesn't support starting the WDT and we
	 * want it started as quickly as possible at boot.
	 *
	 * In either case, there is no mechanism to query the WDT running status
	 * in hardware so we want to ensure the WDT is started and let the WDT
	 * core know that it is. If CONFIG_WATCHDOG_HANDLE_BOOT_ENABLED is set,
	 * then the WDT core will know to automatically feed this until
	 * userspace can take over. If not set, a single feed takes place at
	 * this point in time.
	 */
	if (enable_early)
		set_bit(WDOG_HW_RUNNING, &wdd->status);

	err = devm_watchdog_register_device(dev, wdd);
	if (err)
		return err;

	/* This is the power-off handler on platforms that support the command. */
	if (pm_power_off) {
		dev_err(dev,
			"%s: pm_power_off function already registered, overwriting",
			__func__);
	}
	pm_power_off = ts_wdt_poweroff;
	ts_wdt_poweroff_data = ts_wdt;

	dev_info(dev, "Registered embeddedTS microcontroller watchdog\n");
	return 0;
}

static const struct of_device_id ts_wdt_of_match[] = {
	{ .compatible = "technologic,ts-wdt", },
	{ },
};
MODULE_DEVICE_TABLE(of, ts_wdt_of_match);

static struct platform_driver ts_wdt_driver = {
	.driver = {
		.name	= "ts-wdt",
		.of_match_table = ts_wdt_of_match,
	},
	.probe		= ts_wdt_probe,
};
module_platform_driver(ts_wdt_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_AUTHOR("Kris Bahnsen <kris@embeddedTS.com>");
MODULE_DESCRIPTION("embeddedTS watchdog driver");
MODULE_LICENSE("GPL");
