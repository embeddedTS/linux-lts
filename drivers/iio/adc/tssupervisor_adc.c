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

#define TS_SUPERVISOR_MAX_ADC 32

struct ts_adc {
	struct ts_supervisor *super;
	uint16_t channel_count;
};

#define SUPERVISOR_CHAN(index)					\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)		\
			      | BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 12,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,				\
	},							\
}

static const struct iio_chan_spec tssupervisor_channels[] = {
	SUPERVISOR_CHAN(0),
	SUPERVISOR_CHAN(1),
	SUPERVISOR_CHAN(2),
	SUPERVISOR_CHAN(3),
	SUPERVISOR_CHAN(4),
	SUPERVISOR_CHAN(5),
	SUPERVISOR_CHAN(6),
	SUPERVISOR_CHAN(7),
	SUPERVISOR_CHAN(8),
	SUPERVISOR_CHAN(9),
	SUPERVISOR_CHAN(10),
	SUPERVISOR_CHAN(11),
	SUPERVISOR_CHAN(12),
	SUPERVISOR_CHAN(13),
	SUPERVISOR_CHAN(14),
	SUPERVISOR_CHAN(15),
	SUPERVISOR_CHAN(16),
	SUPERVISOR_CHAN(17),
	SUPERVISOR_CHAN(18),
	SUPERVISOR_CHAN(19),
	SUPERVISOR_CHAN(20),
	SUPERVISOR_CHAN(21),
	SUPERVISOR_CHAN(22),
	SUPERVISOR_CHAN(23),
	SUPERVISOR_CHAN(24),
	SUPERVISOR_CHAN(25),
	SUPERVISOR_CHAN(26),
	SUPERVISOR_CHAN(27),
	SUPERVISOR_CHAN(28),
	SUPERVISOR_CHAN(29),
	SUPERVISOR_CHAN(30),
	SUPERVISOR_CHAN(31),
};

static int ts_adc_iio_read_raw(struct iio_dev *iio_dev,
			struct iio_chan_spec const *chan, int *val,
			int *val2, long mask)
{
	struct ts_adc *adc = iio_priv(iio_dev);
	int addr;
	uint32_t data;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		addr = SUPER_ADC_BASE + chan->channel;
		ret = regmap_read(adc->super->regmap, addr, &data);
		if (ret < 0)
			return ret;
		*val = data;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 3300;
		*val2 = 12;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		break;
	}
	return -EINVAL;
}

static const struct iio_info ts_adc_info = {
	.read_raw = &ts_adc_iio_read_raw,
};

static int ts_supervisor_adc_probe(struct platform_device *pdev)
{
	struct ts_supervisor *super = dev_get_drvdata(pdev->dev.parent);
	struct ts_adc *adc;
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	uint32_t chan_count;
	int ret;

	ret = regmap_read(super->regmap, SUPER_ADC_CHAN_ADV, &chan_count);
	if (ret < 0) {
		dev_err(dev, "error reading reg %u", SUPER_ADC_CHAN_ADV);
		return ret;
	}

	/* This supervisor does not support ADC channels */
	if (chan_count == 0)
		return 0;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (indio_dev == NULL)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	adc->super = super;
	adc->channel_count = chan_count;

	/*
	 * The microcontroller advertises how many ADC are present.  This can
	 * be up to 32 channels depending on muxes onboard and channels that
	 * need to be sampled, but most will be < 7 channels.
	 */
	if (adc->channel_count > TS_SUPERVISOR_MAX_ADC) {
		dev_warn(dev, "The ADC device is advertising more ADC than supported!");
		adc->channel_count = TS_SUPERVISOR_MAX_ADC;
	}
	indio_dev->num_channels = adc->channel_count;
	indio_dev->channels = tssupervisor_channels;

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &ts_adc_info;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id tssupervisor_of_match[] = {
	{ .compatible = "technologic,tssupervisor-adc", },
	{ }
};
MODULE_DEVICE_TABLE(of, tsadc_of_match);

static struct platform_driver tsadc_driver = {
	.driver = {
		.name   = "tssupervisor-adc",
		.of_match_table = tssupervisor_of_match,
	},
	.probe	= ts_supervisor_adc_probe,
};
module_platform_driver(tsadc_driver);

MODULE_DESCRIPTION("embeddedTS supervisor adc controller driver");
MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_LICENSE("GPL");
