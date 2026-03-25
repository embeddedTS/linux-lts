// SPDX-License-Identifier: GPL-2.0+

#include <linux/reboot.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/watchdog.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/system_misc.h>

#define TS_DEFAULT_TIMEOUT 30

static int wdt_timeout;
module_param(wdt_timeout, int, 0);
MODULE_PARM_DESC(wdt_timeout, "Watchdog timeout in seconds");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started default="
		__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/* This driver supports the embeddedTS 2nd generation WDT in
 * microcontroller.
 *
 * The 2nd gen WDT is still in I2C microcontroller, but follows more "standard"
 * WDT layouts. There are separate registers for the timeout, and a single
 * register bit for a feed. Additionally, this 2nd gen microcontroller setup
 * uses a register mapped address space, similar to a generic I2C EEPROM.
 */

/* The relevant WDT registers are:
 *
 * Timeout in centiseconds:
 *   0x400: LSB
 *   0x401:
 *   0x402:
 *   0x403: MSB
 *
 * Control:
 *   0x404:
 *     7   - Set if last reboot was caused by WDT (RO)
 *     6:2 - Reserved
 *     1:0 -
 *         Set to 0x1 to cause a feed of <Timeout>
 *         Set to 0x2 to sleep for <Timeout>
 *
 *
 * Writing a value of 0 to the Timeout registers and issuing a feed will disable
 * the WDT completely.
 */

#define	WDT_TIMEOUT	0x400
#define	WDT_CTRL	0x404
#define	WDT_FEED_CMD	0x01
#define	WDT_SLEEP_CMD	0x02

static int ts7100_wdt_write(struct i2c_client *client, u16 reg, u8 *data,
			    int len)
{
	struct i2c_msg msg;
	u8 data_buf[6];
	int ret;

	BUG_ON(len > 4);

	/* MSB of reg is written on the bus first */
	data_buf[0] = ((reg >> 8) & 0xFF);
	data_buf[1] = (reg & 0xFF);
	memcpy(&data_buf[2], data, len);

	/* Write 16-bit register address */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len+2;
	msg.buf = data_buf;

	dev_dbg(&client->dev, "Writing %d byte(s) to 0x%02X%02X\n",
	  len, data_buf[0], data_buf[1]);

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret != 1) {
		dev_err(&client->dev, "%s: write error, ret=%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/* This sets the timeout to <timeout> seconds. Does not issue a feed after. */
static int ts7100_wdt_set_timeout(struct watchdog_device *wdt,
				  unsigned int timeout)
{
	struct i2c_client *client = to_i2c_client(wdt->parent);
	u8 tmp[4];

	wdt->timeout = timeout;
	/*
	 * WDT has centisecond granularity, kernel is easier with whole
	 * second increments
	 */
	timeout *= 100;
	tmp[0] = timeout & 0xFF;
	tmp[1] = (timeout >> 8) & 0xFF;
	tmp[2] = (timeout >> 16) & 0xFF;
	tmp[3] = (timeout >> 24) & 0xFF;

	return ts7100_wdt_write(client, WDT_TIMEOUT, tmp, 4);
}

static int ts7100_wdt_start(struct watchdog_device *wdt)
{
	struct i2c_client *client = to_i2c_client(wdt->parent);
	u8 buf = WDT_FEED_CMD;

	dev_dbg(&client->dev, "Feeding WDT with %d s\n", wdt->timeout);

	return ts7100_wdt_write(client, WDT_CTRL, &buf, 1);
}

static int ts7100_wdt_stop(struct watchdog_device *wdt)
{
	struct i2c_client *client = to_i2c_client(wdt->parent);
	u8 buf = WDT_FEED_CMD;

	dev_dbg(&client->dev, "%s\n", __func__);

	/* Feeding with a timeout of 0 will disable WDT */
	ts7100_wdt_set_timeout(wdt, 0);
	return ts7100_wdt_write(client, WDT_CTRL, &buf, 1);
}

static int ts7100_wdt_restart(struct watchdog_device *wdt,
			      unsigned long a, void *b)
{
	struct i2c_client *client = to_i2c_client(wdt->parent);
	u8 buf = WDT_FEED_CMD;

	dev_dbg(&client->dev, "%s\n", __func__);

	/* Set WDT for 1 s timeout and stall CPU */
	ts7100_wdt_set_timeout(wdt, 1);
	ts7100_wdt_write(client, WDT_CTRL, &buf, 1);

	while (1)
		;

	return 0;
}

static struct watchdog_info ts7100_wdt_info = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
	.identity	= "TS-7100 uC Watchdog",
};

/* NOTE:
 * If .ping is not provided, .start is used. Both would end up doing the same
 * operation anyway.
 * WDIOC_KEEPALIVE ioctl() still functions when .ping is not defined (.start is
 * called instead), so long as WDIOF_KEEPALIVEPING is set as an option
 */
static struct watchdog_ops ts7100_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ts7100_wdt_start,
	.stop		= ts7100_wdt_stop,
	.set_timeout	= ts7100_wdt_set_timeout,
	.restart	= ts7100_wdt_restart,
};

static struct watchdog_device ts_wdt_wdd = {
	.info		= &ts7100_wdt_info,
	.ops		= &ts7100_wdt_ops,
	/*
	 * A timeout of 0 means disable. Due to how the validity of the timeout
	 * is checked, we need to spec a minimum timeout of 0. This allows the
	 * FDT to spec a timeout of 0, disabling the WDT on startup. This may
	 * cause issues in specific scenarios.
	 */
	.min_timeout	= 0,
	.max_timeout	= 4294967,
	.status		= WATCHDOG_NOWAYOUT_INIT_STATUS,
	.timeout	= TS_DEFAULT_TIMEOUT,
};

static int ts7100_wdt_probe(struct i2c_client *client)
{
	ts_wdt_wdd.parent = &client->dev;

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
	 * this point in time.we
	 */

	if (of_property_read_bool(client->dev.of_node, "enable-early"))
		set_bit(WDOG_HW_RUNNING, &ts_wdt_wdd.status);

	watchdog_set_nowayout(&ts_wdt_wdd, nowayout);
	watchdog_set_restart_priority(&ts_wdt_wdd, 128);

	return watchdog_register_device(&ts_wdt_wdd);
}

static const struct i2c_device_id ts7100_wdt_id[] = {
	{ "ts7100-wdt", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts7100_wdt_id);

static const struct of_device_id ts7100_wdt_of_match[] = {
	{ .compatible = "technologic,ts7100-wdt", },
	{ },
};
MODULE_DEVICE_TABLE(of, ts7100_wdt_of_match);

MODULE_ALIAS("platform:ts7100_wdt");

static struct i2c_driver ts7100_wdt_driver = {
	.driver = {
		.name	= "ts7100_wdt",
		.of_match_table = ts7100_wdt_of_match,
		.owner	= THIS_MODULE,
	},
	.probe		= ts7100_wdt_probe,
	.id_table	= ts7100_wdt_id,
};
module_i2c_driver(ts7100_wdt_driver);

MODULE_AUTHOR("Kris Bahnsen <kris@embeddedTS.com>");
MODULE_DESCRIPTION("embeddedTS' TS-7100 (and compat.) WDT driver");
MODULE_LICENSE("GPL");
