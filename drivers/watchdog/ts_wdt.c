// SPDX-License-Identifier: GPL-2.0+
/*
 * embeddedTS uC i2c watchdog driver
 *
 * Copyright (C) 2017, 2023 Technologic Systems, Inc. dba embeddedTS
 *
 * Originally written for kernel 4.1, this has been updated with more
 * modern paradigms and does not behave quite the same as the original.
 */

#include <asm/system_misc.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/watchdog.h>

#define TS_DEFAULT_TIMEOUT 30

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started default="
		__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/* Global for ts_wdt_poweroff to access needed data */
static struct i2c_client *ts_wdt_poweroff_dev;
static struct watchdog_device *ts_wdt_restart_wdd;

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

static int ts_wdt_write(struct i2c_client *client, u16 deciseconds)
{
	u8 out[3];
	int ret;
	struct i2c_msg msg;

	out[0] = 0;
	out[1] = (deciseconds & 0xff00) >> 8;
	out[2] = deciseconds & 0xff;
	dev_dbg(&client->dev, "Writing 0x00, 0x%02x, 0x%02x\n",
		out[1],
		out[2]);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = out;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "%s: write error, ret=%d\n",
			__func__, ret);
		return ret;
	}
	return 0;
}

static int ts_wdt_start(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	dev_dbg(&client->dev, "%s\n", __func__);

	return ts_wdt_write(client, wdd->timeout * 10);
}

static int ts_wdt_stop(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	dev_dbg(&client->dev, "%s\n", __func__);
	return ts_wdt_write(client, 3);
}

static int ts_wdt_restart(struct watchdog_device *wdd,
			unsigned long a, void *b)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	dev_dbg(&client->dev, "%s\n", __func__);
	printk(KERN_ERR "KRIS: wdt reboot hit\n");

	ts_wdt_write(client, 0);
	while (1);

	return 0;
}

static void ts_wdt_restart_shim(enum reboot_mode mode, const char *cmd)
{
	ts_wdt_restart(ts_wdt_restart_wdd, 0, NULL);
}

void ts_wdt_poweroff(void)
{
	struct i2c_client *client = ts_wdt_poweroff_dev;

	u8 out[3];
	int ret;
	struct i2c_msg msg;

	dev_dbg(&client->dev, "%s\n", __func__);

	ts_wdt_write(client, 3);

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


	while (1);
}

static int ts_set_timeout(struct watchdog_device *wdd,
				   unsigned int timeout)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	dev_dbg(&client->dev, "%s\n", __func__);
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

static int ts_wdt_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int err;
	bool enable_early = false;
	struct watchdog_device *wdd;

	dev_dbg(&client->dev, "%s\n", __func__);

	wdd = devm_kzalloc(&client->dev, sizeof(*wdd), GFP_KERNEL);
	if (!wdd)
		return -ENOMEM;

	wdd->info = &ts_wdt_info;
	wdd->ops = &ts_wdt_ops;
	wdd->min_timeout = 1;
	wdd->max_timeout = 6553;
	wdd->timeout = TS_DEFAULT_TIMEOUT;
	wdd->parent = &client->dev;

	if (of_property_read_bool(client->dev.of_node, "enable-early"))
		enable_early = true;

	watchdog_set_nowayout(wdd, nowayout);

	i2c_set_clientdata(client, wdd);

	/* We want this handler to be the first priority handler for reboots */
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

	/*
	 * On supported platforms, this will generally be the only way to
	 * correctly power_off the system. So, clobber any other handlers that
	 * may have already been set but generate some noise if this happens.
	 */
	if (pm_power_off != NULL) {
		dev_err(&client->dev,
			 "%s: pm_power_off function already registered, overwriting",
			__func__);
	}
	pm_power_off = ts_wdt_poweroff;

	/* While the watchdog subsystem has hooks for running a restart through
	 * the WDT itself with varying levels of priority, the imx28 plat registers
	 * its own reboot handler under arm_pm_restart. Which the arm reboot core
	 * will use instead of the reboot handler chain that this watchdog driver
	 * would register with.
	 *
	 * The watchdog restart handler worked in 4.9, but, appears to have changed
	 * going in to 5.10.
	 */
	if (arm_pm_restart != NULL) {
		dev_err(&client->dev,
			 "%s: arm_pm_restart function already registered, overwriting",
			__func__);
	}
	arm_pm_restart = ts_wdt_restart_shim;
	ts_wdt_poweroff_dev = client;
	ts_wdt_restart_wdd = wdd;

	err = watchdog_register_device(wdd);
	if (err)
		return err;

	dev_info(&client->dev, "Registered embeddedTS microcontroller watchdog\n");

	return 0;
}

static const struct i2c_device_id ts_wdt_id[] = {
	{ "ts-wdt", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts_wdt_id);

static const struct of_device_id ts_wdt_of_match[] = {
	{ .compatible = "technologic,ts-wdt", },
	{ .compatible = "ts_wdt", },
	{ },
};
MODULE_DEVICE_TABLE(of, ts_wdt_of_match);

MODULE_ALIAS("platform:ts-wdt");

static struct i2c_driver ts_wdt_driver = {
	.driver = {
		.name	= "ts-wdt",
		.owner	= THIS_MODULE,
	},
	.probe		= ts_wdt_probe,
	.id_table	= ts_wdt_id,
};

static int __init ts_wdt_init(void)
{
	return i2c_add_driver(&ts_wdt_driver);
}
subsys_initcall(ts_wdt_init);

static void __exit ts_wdt_exit(void)
{
	i2c_del_driver(&ts_wdt_driver);
}
module_exit(ts_wdt_exit);

MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_AUTHOR("Kris Bahnsen <kris@embeddedTS.com>");
MODULE_DESCRIPTION("embeddedTS watchdog driver");
MODULE_LICENSE("GPL");
