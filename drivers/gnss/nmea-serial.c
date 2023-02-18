// SPDX-License-Identifier: GPL-2.0
/*
 * GNSS receiver driver for NMEA-0183 over serial
 *
 * Copyright (C) 2023 Technologic Systems, Inc. dba embeddedTS <support@embeddedTS.com>
 *
 * Based on mtk driver
 */

#include <linux/errno.h>
#include <linux/gnss.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>

#include "serial.h"

struct nmea_ser_data {
	struct regulator *vbackup;
	struct regulator *vcc;
};

static int nmea_ser_set_active(struct gnss_serial *gserial)
{
	struct nmea_ser_data *data = gnss_serial_get_drvdata(gserial);
	int ret;

	ret = regulator_enable(data->vcc);
	if (ret)
		return ret;

	return 0;
}

static int nmea_ser_set_standby(struct gnss_serial *gserial)
{
	struct nmea_ser_data *data = gnss_serial_get_drvdata(gserial);
	int ret;

	ret = regulator_disable(data->vcc);
	if (ret)
		return ret;

	return 0;
}

static int nmea_ser_set_power(struct gnss_serial *gserial,
			 enum gnss_serial_pm_state state)
{
	switch (state) {
	case GNSS_SERIAL_ACTIVE:
		return nmea_ser_set_active(gserial);
	case GNSS_SERIAL_OFF:
	case GNSS_SERIAL_STANDBY:
		return nmea_ser_set_standby(gserial);
	}

	return -EINVAL;
}

static const struct gnss_serial_ops nmea_ser_gserial_ops = {
	.set_power = nmea_ser_set_power,
};

static int nmea_ser_probe(struct serdev_device *serdev)
{
	struct gnss_serial *gserial;
	struct nmea_ser_data *data;
	int ret;

	gserial = gnss_serial_allocate(serdev, sizeof(*data));
	if (IS_ERR(gserial)) {
		ret = PTR_ERR(gserial);
		return ret;
	}

	gserial->ops = &nmea_ser_gserial_ops;

	gserial->gdev->type = GNSS_TYPE_NMEA;

	data = gnss_serial_get_drvdata(gserial);

	data->vcc = devm_regulator_get(&serdev->dev, "vcc");
	if (IS_ERR(data->vcc)) {
		ret = PTR_ERR(data->vcc);
		goto err_free_gserial;
	}

	if (data->vbackup) {
		ret = regulator_enable(data->vbackup);
		if (ret)
			goto err_free_gserial;
	}

	ret = gnss_serial_register(gserial);
	if (ret)
		goto err_disable_vbackup;

	return 0;

err_disable_vbackup:
	if (data->vbackup)
		regulator_disable(data->vbackup);
err_free_gserial:
	gnss_serial_free(gserial);

	return ret;
}

static void nmea_ser_remove(struct serdev_device *serdev)
{
	struct gnss_serial *gserial = serdev_device_get_drvdata(serdev);
	struct nmea_ser_data *data = gnss_serial_get_drvdata(gserial);

	gnss_serial_deregister(gserial);
	if (data->vbackup)
		regulator_disable(data->vbackup);
	gnss_serial_free(gserial);
};

#ifdef CONFIG_OF
static const struct of_device_id nmea_ser_of_match[] = {
	{ .compatible = "gnss,nmea-serial" },
	{},
};
MODULE_DEVICE_TABLE(of, nmea_ser_of_match);
#endif

static struct serdev_device_driver nmea_ser_driver = {
	.driver	= {
		.name		= "gnss-nmea-serial",
		.of_match_table	= of_match_ptr(nmea_ser_of_match),
		.pm		= &gnss_serial_pm_ops,
	},
	.probe	= nmea_ser_probe,
	.remove	= nmea_ser_remove,
};
module_serdev_device_driver(nmea_ser_driver);

MODULE_AUTHOR("Kris Bahnsen <kris@embeddedTS.com>");
MODULE_DESCRIPTION("Generic GNSS NMEA-0183 serial receiver driver");
MODULE_LICENSE("GPL v2");
