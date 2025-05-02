// SPDX-License-Identifier: GPL-2.0
/*
 * ADC driver for simple 8-bit ADC on TS-7250-V3
 * Copyright (C) 2021-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/mfd/ts_supervisor.h>

struct ts_temp_adc {
	struct ts_supervisor *super;
};

static const struct iio_chan_spec ts_temp_channel =
{
	.type = IIO_TEMP,
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_PROCESSED),
};

static int ts_temp_iio_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan, int *val,
				int *val2, long mask)
{
	struct ts_temp_adc *adc = iio_priv(iio_dev);
	int32_t data;
	int ret;

	ret = regmap_read(adc->super->regmap, SUPER_TEMPERATURE, &data);
	if (ret < 0)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		*val = (int32_t)data*196551/1000-277439;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ts_adc_info = {
	.read_raw = &ts_temp_iio_read_raw,
};

static int ts_supervisor_temp_probe(struct platform_device *pdev)
{
	struct ts_supervisor *super = dev_get_drvdata(pdev->dev.parent);
	struct ts_temp_adc *adc;
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (indio_dev == NULL)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	adc->super = super;

	/* ADC Channels + 1 temperature sensor */
	indio_dev->num_channels = 1;
	indio_dev->channels = &ts_temp_channel;

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &ts_adc_info;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver tsadc_driver = {
	.driver = {
		.name   = "tssupervisor-temp",
	},
	.probe	= ts_supervisor_temp_probe,
};
module_platform_driver(tsadc_driver);

MODULE_DESCRIPTION("embeddedTS supervisor temperature sensor");
MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_LICENSE("GPL");
