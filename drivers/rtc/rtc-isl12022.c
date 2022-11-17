// SPDX-License-Identifier: GPL-2.0-only
/*
 * An I2C driver for the Intersil ISL 12022
 *
 * Author: Roman Fietze <roman.fietze@telemotive.de>
 *
 * Based on the Philips PCF8563 RTC
 * by Alessandro Zummo <a.zummo@towertech.it>.
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/hwmon.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

/* ISL register offsets */
#define ISL12022_REG_SC		0x00
#define ISL12022_REG_MN		0x01
#define ISL12022_REG_HR		0x02
#define ISL12022_REG_DT		0x03
#define ISL12022_REG_MO		0x04
#define ISL12022_REG_YR		0x05
#define ISL12022_REG_DW		0x06

#define ISL12022_REG_SR		0x07
#define ISL12022_REG_INT	0x08
#define ISL12022_REG_VBAT	0x0a
#define ISL12022_REG_BETA	0x0d
#define ISL12022_REG_FATR	0x0e
#define ISL12022_REG_FDTR	0x0f
#define ISL12022_REG_TEMP	0x28

/*
 * These 3 registers only exist in the emulated device, they are unused dst
 * registers on the real RTC.
 */
#define ISL12022_REG_OFF_VAL	0x21
#define ISL12022_REG_OFF_CTL	0x25

/* ISL register bits */
#define ISL12022_HR_MIL		(1 << 7)	/* military or 24 hour time */

#define ISL12022_SR_LBAT85	(1 << 2)
#define ISL12022_SR_LBAT75	(1 << 1)
#define ISL12022_SR_RTCF	(1 << 0)

#define ISL12022_INT_WRTC	(1 << 6)

#define ISL12022_BETA_TSE	(1 << 7) /* Enable temp sensor compensation */
#define ISL12022_BETA_BTSE	(1 << 6) /* Temp sensor enabled on VBAT */
#define ISL12022_BETA_BTSR	(1 << 5) /* Sample Frequency (1=1min,0=10min) */
#define ISL12022_VBAT_VB85_MASK 0x38
#define ISL12022_VBAT_VB85_SHFT	3
#define ISL12022_VBAT_VB75_MASK 0x7
#define ISL12022_VBAT_VB75_SHFT	0
#define ISL12022_OFF_CTL_APPLY	(1 << 0) /* Make value take affect now */
#define ISL12022_OFF_CTL_ADD	(1 << 1) /* 1 if the value is add, 0 if subtract */
#define ISL12022_OFF_CTL_FLASH	(1 << 2) /* 1 to commit to flash, 0 to just ram */

/* Detect embeddedTS emulated ISL12022.  This is always 0 on the real RTC. */
#define ISL12022_FDTR_EMULATED	(1 << 7)

static struct i2c_driver isl12022_driver;

struct isl12022 {
	struct rtc_device *rtc;
	struct regmap *regmap;
	bool enable_btse;
	uint32_t btse_minutes;
	bool set_trip_thresh;
	uint32_t vb75_threshold;
	uint32_t vb85_threshold;
};

/*
 * In the routines that deal directly with the isl12022 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int isl12022_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct isl12022 *isl12022 = dev_get_drvdata(dev);
	struct regmap *regmap = isl12022->regmap;
	uint8_t buf[ISL12022_REG_INT + 1];
	int ret;

	ret = regmap_bulk_read(regmap, ISL12022_REG_SC, buf, sizeof(buf));
	if (ret)
		return ret;

	if (buf[ISL12022_REG_SR] & ISL12022_SR_RTCF) {
		dev_err(dev, "Total power failure, RTC data is invalid.\n");
		return -EINVAL;
	}

	/*
	 * Check battery voltage trip points.
	 * These have default settings but may also be set by isl12022_setup()
	 * which also cycles TSE and forces a re-read of the temperature and
	 * battery voltages. Temperature conversions take 22 ms, battery voltage
	 * conversions do not have a listed time.
	 *
	 * Warn only once if the battery voltage is found to be under the set
	 * thresholds to prevent spamming on every RTC read.
	 */
	if (buf[ISL12022_REG_SR] & ISL12022_SR_LBAT75) {
		dev_warn_once(dev,
			"battery voltage dropped below 75%%, date and time may not be reliable.\n");
	} else if (buf[ISL12022_REG_SR] & ISL12022_SR_LBAT85) {
		dev_warn_once(dev,
			"battery voltage dropped below 85%%.\n");
	}

	dev_dbg(dev,
		"%s: raw data is sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, mon=%02x, year=%02x, wday=%02x, "
		"sr=%02x, int=%02x",
		__func__,
		buf[ISL12022_REG_SC],
		buf[ISL12022_REG_MN],
		buf[ISL12022_REG_HR],
		buf[ISL12022_REG_DT],
		buf[ISL12022_REG_MO],
		buf[ISL12022_REG_YR],
		buf[ISL12022_REG_DW],
		buf[ISL12022_REG_SR],
		buf[ISL12022_REG_INT]);

	tm->tm_sec = bcd2bin(buf[ISL12022_REG_SC] & 0x7F);
	tm->tm_min = bcd2bin(buf[ISL12022_REG_MN] & 0x7F);
	tm->tm_hour = bcd2bin(buf[ISL12022_REG_HR] & 0x3F);
	tm->tm_mday = bcd2bin(buf[ISL12022_REG_DT] & 0x3F);
	tm->tm_wday = buf[ISL12022_REG_DW] & 0x07;
	tm->tm_mon = bcd2bin(buf[ISL12022_REG_MO] & 0x1F) - 1;
	tm->tm_year = bcd2bin(buf[ISL12022_REG_YR]) + 100;

	dev_dbg(dev, "%s: %ptR\n", __func__, tm);

	return 0;
}

static int isl12022_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct isl12022 *isl12022 = dev_get_drvdata(dev);
	struct regmap *regmap = isl12022->regmap;
	int ret;
	uint8_t buf[ISL12022_REG_DW + 1];

	dev_dbg(dev, "%s: %ptR\n", __func__, tm);

	/* Ensure the write enable bit is set. */
	ret = regmap_update_bits(regmap, ISL12022_REG_INT,
				 ISL12022_INT_WRTC, ISL12022_INT_WRTC);
	if (ret)
		return ret;

	/* hours, minutes and seconds */
	buf[ISL12022_REG_SC] = bin2bcd(tm->tm_sec);
	buf[ISL12022_REG_MN] = bin2bcd(tm->tm_min);
	buf[ISL12022_REG_HR] = bin2bcd(tm->tm_hour) | ISL12022_HR_MIL;

	buf[ISL12022_REG_DT] = bin2bcd(tm->tm_mday);

	/* month, 1 - 12 */
	buf[ISL12022_REG_MO] = bin2bcd(tm->tm_mon + 1);

	/* year and century */
	buf[ISL12022_REG_YR] = bin2bcd(tm->tm_year % 100);

	buf[ISL12022_REG_DW] = tm->tm_wday & 0x07;

	return regmap_bulk_write(isl12022->regmap, ISL12022_REG_SC,
				 buf, sizeof(buf));
}

static int isl12022_set_offset(struct device *dev, long offset)
{
	struct isl12022 *isl12022 = dev_get_drvdata(dev);
	uint32_t ppb = abs(offset);
	uint8_t data;
	int ret;

	ret = regmap_bulk_write(isl12022->regmap, ISL12022_REG_OFF_VAL, &ppb, sizeof(ppb));
	if (ret)
		return ret;
	data = ISL12022_OFF_CTL_APPLY |
	       ((offset > 0) ? ISL12022_OFF_CTL_ADD : 0) |
	       ISL12022_OFF_CTL_FLASH;
	ret = regmap_bulk_write(isl12022->regmap, ISL12022_REG_OFF_CTL, &data, 1);
	if (ret)
		return ret;

	return ret;
}

static int isl12022_read_offset(struct device *dev, long *offset)
{
	struct isl12022 *isl12022 = dev_get_drvdata(dev);
	int ret;
	uint32_t ppb;
	uint8_t data;

	ret = regmap_bulk_read(isl12022->regmap, ISL12022_REG_OFF_VAL, &ppb, sizeof(ppb));
	if (ret)
		return ret;

	ret = regmap_bulk_read(isl12022->regmap, ISL12022_REG_OFF_CTL, &data, 1);
	if (ret)
		return -EIO;

	*offset = ppb;

	if ((data & ISL12022_OFF_CTL_ADD) == 0)
		*offset *= -1;

	return ret;
}

static int isl12022_hwmon_read_temp(struct device *dev, long *mC)
{
	struct isl12022 *isl12022 = dev_get_drvdata(dev);
	u8 data[2];
	int ret;

	ret = regmap_bulk_read(isl12022->regmap, ISL12022_REG_TEMP, data,
			       sizeof(data));
	if (ret < 0)
		return ret;

	/* Convert from Kelvin */
	*mC = ((data[0]|(data[1]<<8))*500)-273150;

	return 0;
}

static int isl12022_hwmon_read(struct device *dev,
			     enum hwmon_sensor_types type,
			     u32 attr, int channel, long *temp)
{
	int err;

	switch (attr) {
	case hwmon_temp_input:
		err = isl12022_hwmon_read_temp(dev, temp);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	return err;
}

static umode_t isl12022_hwmon_is_visible(const void *data,
				       enum hwmon_sensor_types type,
				       u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
		return 0444;
	default:
		return 0;
	}
}

static const struct rtc_class_ops isl12022_rtc_ops = {
	.read_time	= isl12022_rtc_read_time,
	.set_time	= isl12022_rtc_set_time,
};

static const struct rtc_class_ops isl12022_emulated_rtc_ops = {
	.read_time	= isl12022_rtc_read_time,
	.set_time	= isl12022_rtc_set_time,
	.set_offset	= isl12022_set_offset,
	.read_offset	= isl12022_read_offset,
};

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.use_single_write = true,
};

static u32 isl12022_hwmon_chip_config[] = {
	HWMON_C_REGISTER_TZ,
	0
};

static const struct hwmon_channel_info isl12022_hwmon_chip = {
	.type = hwmon_chip,
	.config = isl12022_hwmon_chip_config,
};

static u32 isl12022_hwmon_temp_config[] = {
	HWMON_T_INPUT,
	0
};

static const struct hwmon_channel_info isl12022_hwmon_temp = {
	.type = hwmon_temp,
	.config = isl12022_hwmon_temp_config,
};

static const struct hwmon_channel_info *isl12022_hwmon_info[] = {
	&isl12022_hwmon_chip,
	&isl12022_hwmon_temp,
	NULL
};

static const struct hwmon_ops isl12022_hwmon_hwmon_ops = {
	.is_visible = isl12022_hwmon_is_visible,
	.read = isl12022_hwmon_read,
};

static const struct hwmon_chip_info isl12022_hwmon_chip_info = {
	.ops = &isl12022_hwmon_hwmon_ops,
	.info = isl12022_hwmon_info,
};

static void isl12022_hwmon_register(struct device *dev)
{
	struct isl12022 *isl12022 = dev_get_drvdata(dev);
	struct device *hwmon_dev;

	if (!IS_ENABLED(CONFIG_RTC_DRV_ISL12022_HWMON))
		return;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "isl12022", isl12022,
							&isl12022_hwmon_chip_info,
							NULL);
	if (IS_ERR(hwmon_dev)) {
		dev_err(dev, "unable to register hwmon device %ld\n",
			PTR_ERR(hwmon_dev));
	}
}

int isl12022_setup(struct i2c_client *client, struct isl12022 *isl12022)
{
	struct regmap *regmap = isl12022->regmap;
	struct device *dev = &client->dev;
	uint32_t data = 0;
	int ret = 0;

	/*
	 * All changes to ALPHA, BETA, IDTR, and IATR registers must
	 * be done with TSE disabled according to the datasheet.
	 * It's possible for BTSE settings to be touched one way or another
	 * below, so disable TSE for now and re-enable later just in case.
	 */
	ret = regmap_update_bits(regmap, ISL12022_REG_BETA,
				ISL12022_BETA_TSE,
				0);
	if (ret)
		return ret;

	/* Setup temperature sensing on battery power. */
	if (isl12022->enable_btse) {
		if (isl12022->btse_minutes != 10)
			data = ISL12022_BETA_BTSR;

		data |= ISL12022_BETA_BTSE;

		ret = regmap_update_bits(regmap, ISL12022_REG_BETA,
					ISL12022_BETA_BTSE | ISL12022_BETA_BTSR,
					data);
		if (ret)
			return ret;
	} else {
		ret = regmap_update_bits(regmap, ISL12022_REG_BETA,
					ISL12022_BETA_BTSE,
					0);
		if (ret)
			return ret;
	}

	/* Set battery voltage trip thresholds */
	if (isl12022->set_trip_thresh) {
		ret = regmap_update_bits(regmap,
					 ISL12022_REG_VBAT,
					 ISL12022_VBAT_VB85_MASK | ISL12022_VBAT_VB75_MASK,
					 isl12022->vb75_threshold << ISL12022_VBAT_VB75_SHFT |
					 isl12022->vb85_threshold << ISL12022_VBAT_VB85_SHFT);
		if (ret)
			return ret;
	}

	/*
	 * (Re)Enable TSE after BETA and VB75/85T were potentially modified.
	 * Setting TSE will also force a manual battery voltage and temperature
	 * read.
	 */
	ret = regmap_update_bits(regmap, ISL12022_REG_BETA, ISL12022_BETA_TSE,
				ISL12022_BETA_TSE);
	if (ret)
		return ret;

	isl12022_hwmon_register(dev);

	return ret;
}

static int isl12022_probe(struct i2c_client *client,
			  const struct i2c_device_id *id) {
	struct isl12022 *isl12022;
	struct device *dev = &client->dev;
	uint32_t data;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	isl12022 = devm_kzalloc(&client->dev, sizeof(struct isl12022),
				GFP_KERNEL);
	if (!isl12022)
		return -ENOMEM;
	dev_set_drvdata(&client->dev, isl12022);

	isl12022->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(isl12022->regmap)) {
		dev_err(&client->dev, "regmap allocation failed\n");
		return PTR_ERR(isl12022->regmap);
	}

	isl12022->rtc = devm_rtc_allocate_device(&client->dev);
	if (IS_ERR(isl12022->rtc))
		return PTR_ERR(isl12022->rtc);

	/* Detect emulated isl12022 */
	ret = regmap_bulk_read(isl12022->regmap, ISL12022_REG_FDTR, &data, 1);
	if (ret)
		return ret;

	if (data & ISL12022_FDTR_EMULATED) {
		dev_info(dev, "Emulated isl12022 detected");
		isl12022->rtc->ops = &isl12022_emulated_rtc_ops;
	} else {
		isl12022->rtc->ops = &isl12022_rtc_ops;
	}

	if (dev->of_node) {
		ret = of_property_read_u32(dev->of_node,
					"btse-minutes",
					&isl12022->btse_minutes);
		if (!ret)
			isl12022->enable_btse = 1;

		/* Both vb75t and vb85t must be passed simultaneously. */
		ret = of_property_read_u32(dev->of_node,
					"vb75t",
					&isl12022->vb75_threshold);
		if (!ret)
			isl12022->set_trip_thresh = 1;

		ret = of_property_read_u32(dev->of_node,
					"vb85t",
					&isl12022->vb85_threshold);
		if (ret || !isl12022->set_trip_thresh)
			isl12022->set_trip_thresh = 0;
	}

	ret = isl12022_setup(client, isl12022);
	if (ret)
		return ret;

	return rtc_register_device(isl12022->rtc);
}

#ifdef CONFIG_OF
static const struct of_device_id isl12022_dt_match[] = {
	{ .compatible = "isl,isl12022" }, /* for backward compat., don't use */
	{ .compatible = "isil,isl12022" },
	{ },
};
MODULE_DEVICE_TABLE(of, isl12022_dt_match);
#endif

static const struct i2c_device_id isl12022_id[] = {
	{ "isl12022", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl12022_id);

static struct i2c_driver isl12022_driver = {
	.driver		= {
		.name	= "rtc-isl12022",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(isl12022_dt_match),
#endif
	},
	.probe		= isl12022_probe,
	.id_table	= isl12022_id,
};

module_i2c_driver(isl12022_driver);

MODULE_AUTHOR("roman.fietze@telemotive.de");
MODULE_DESCRIPTION("ISL 12022 RTC driver");
MODULE_LICENSE("GPL");
