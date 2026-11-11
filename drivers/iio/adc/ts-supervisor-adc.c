// SPDX-License-Identifier: GPL-2.0-only
/* embeddedTS supervisory microcontroller ADC register interface. */
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

struct ts_supervisor_adc {
	struct regmap *regmap;
	struct i2c_client *client;
	const struct ts_supervisor_adc_chip *chip;
};

struct ts_supervisor_adc_channel {
	const char *label;
	unsigned int reg;
	bool temperature;
	bool raw_temperature;
	int raw_format;
	int scale;
	int scale2;
	int scale_format;
};

struct ts_supervisor_adc_chip {
	const struct ts_supervisor_adc_channel *channels;
	const struct iio_chan_spec *iio_channels;
	unsigned int num_channels;
	unsigned int snapshot_size;
	bool gen0;
};

#define TS_SUPERVISOR_CHAN(n) { \
	.type = IIO_VOLTAGE, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), \
}

#define TS_SUPERVISOR_TEMP_CHAN(n) { \
	.type = IIO_TEMP, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), \
}

#define TS_SUPERVISOR_RAW_ONLY_CHAN(n) { \
	.type = IIO_VOLTAGE, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
}

#define TS_SUPERVISOR_RAW_TEMP_CHAN(n) { \
	.type = IIO_TEMP, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
}

static const struct iio_chan_spec ts_supervisor_channels[] = {
	TS_SUPERVISOR_CHAN(0), TS_SUPERVISOR_CHAN(1), TS_SUPERVISOR_CHAN(2),
	TS_SUPERVISOR_CHAN(3), TS_SUPERVISOR_TEMP_CHAN(4),
	TS_SUPERVISOR_CHAN(7), TS_SUPERVISOR_CHAN(8),
	TS_SUPERVISOR_TEMP_CHAN(10),
};

#define TS_SUPERVISOR_MV_CHANNEL(n, name) { \
	.label = (name), .reg = (n) * 2, .raw_format = IIO_VAL_INT, \
	.scale = 1, .scale_format = IIO_VAL_INT, \
}

#define TS_SUPERVISOR_RAW_CHANNEL(n, name) { \
	.label = (name), .reg = (n) * 2, .raw_format = IIO_VAL_INT, \
	.scale = 2500, .scale2 = 1023, .scale_format = IIO_VAL_FRACTIONAL, \
}

#define TS_SUPERVISOR_TEMP_CHANNEL(n, name) { \
	.label = (name), .reg = (n) * 2, .temperature = true, \
}

static const struct ts_supervisor_adc_channel ts7100_channels[] = {
	[0] = TS_SUPERVISOR_MV_CHANNEL(0, "5V_A"),
	[1] = TS_SUPERVISOR_MV_CHANNEL(1, "CHARGE"),
	[2] = TS_SUPERVISOR_MV_CHANNEL(2, "3V3"),
	[3] = TS_SUPERVISOR_MV_CHANNEL(3, "VIN"),
	[4] = TS_SUPERVISOR_TEMP_CHANNEL(4, "TEMP_INITIAL"),
	[7] = TS_SUPERVISOR_MV_CHANNEL(7, "SUP_CAP_1"),
	[8] = TS_SUPERVISOR_MV_CHANNEL(8, "SUP_CAP_2"),
	[10] = TS_SUPERVISOR_TEMP_CHANNEL(10, "TEMP"),
};

static const struct ts_supervisor_adc_channel ts7180_channels[] = {
	[0] = TS_SUPERVISOR_RAW_CHANNEL(0, "5V_A"),
	[1] = TS_SUPERVISOR_RAW_CHANNEL(1, "CHARGE"),
	[2] = TS_SUPERVISOR_RAW_CHANNEL(2, "3V3"),
	[3] = TS_SUPERVISOR_RAW_CHANNEL(3, "VIN"),
	[4] = TS_SUPERVISOR_TEMP_CHANNEL(4, "TEMP_INITIAL"),
	[7] = TS_SUPERVISOR_MV_CHANNEL(7, "SUP_CAP_1"),
	[8] = TS_SUPERVISOR_MV_CHANNEL(8, "SUP_CAP_2"),
	[10] = TS_SUPERVISOR_TEMP_CHANNEL(10, "TEMP"),
};

#define TS_SUPERVISOR_GEN0_CHAN(name, offset, factor) { \
	.label = (name), .reg = (offset), .raw_format = IIO_VAL_INT, \
	.scale = 2500 * (factor), .scale2 = 1023, \
	.scale_format = IIO_VAL_FRACTIONAL, \
}

#define TS_SUPERVISOR_GEN0_RAW_CHANNEL(name, offset) { \
	.label = (name), .reg = (offset), .raw_format = IIO_VAL_INT, \
}

static const struct ts_supervisor_adc_channel ts7553v2_channels[] = {
	[0] = TS_SUPERVISOR_GEN0_CHAN("SUP_CAP_1", 0, 1),
	[1] = TS_SUPERVISOR_GEN0_CHAN("SUP_CAP", 2, 2),
	[2] = TS_SUPERVISOR_GEN0_CHAN("4V7", 4, 2),
	[3] = {
		.label = "VIN", .reg = 6, .raw_format = IIO_VAL_INT,
		.scale = 2500 * 2082, .scale2 = 1023 * 172,
		.scale_format = IIO_VAL_FRACTIONAL,
	},
	[4] = {
		.label = "5V_A", .reg = 8, .raw_format = IIO_VAL_INT,
		.scale = 2500 * 958, .scale2 = 1023 * 422,
		.scale_format = IIO_VAL_FRACTIONAL,
	},
	[5] = TS_SUPERVISOR_GEN0_CHAN("3V3", 10, 2),
	[6] = TS_SUPERVISOR_GEN0_CHAN("RAM_V1P35", 12, 1),
	[7] = TS_SUPERVISOR_GEN0_CHAN("VCORE", 14, 1),
	[8] = TS_SUPERVISOR_GEN0_CHAN("CHARGE", 16, 2),
	[9] = TS_SUPERVISOR_GEN0_CHAN("VSOC", 18, 1),
	[10] = TS_SUPERVISOR_GEN0_CHAN("VARM", 20, 1),
};

static const struct ts_supervisor_adc_channel ts7680_channels[] = {
	[0] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("SUP_CAP_LOWER", 0),
	[1] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("SUP_CAP", 2),
	[2] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("P1_4", 4),
	[3] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("VIN", 6),
	[4] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("5V_A", 8),
	[5] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("P2_2", 10),
	[6] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("P2_3", 12),
	[7] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("P2_4", 18),
	[8] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("CHARGE", 20),
	[9] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("P2_6", 22),
	[10] = TS_SUPERVISOR_GEN0_RAW_CHANNEL("P2_7", 24),
	[11] = {
		.label = "TEMP_RAW", .reg = 26, .temperature = true,
		.raw_temperature = true, .raw_format = IIO_VAL_INT,
	},
};

static const struct iio_chan_spec ts7553v2_iio_channels[] = {
	TS_SUPERVISOR_CHAN(0), TS_SUPERVISOR_CHAN(1),
	TS_SUPERVISOR_CHAN(2), TS_SUPERVISOR_CHAN(3),
	TS_SUPERVISOR_CHAN(4), TS_SUPERVISOR_CHAN(5),
	TS_SUPERVISOR_CHAN(6), TS_SUPERVISOR_CHAN(7),
	TS_SUPERVISOR_CHAN(8), TS_SUPERVISOR_CHAN(9),
	TS_SUPERVISOR_CHAN(10),
};

static const struct iio_chan_spec ts7680_iio_channels[] = {
	TS_SUPERVISOR_RAW_ONLY_CHAN(0), TS_SUPERVISOR_RAW_ONLY_CHAN(1),
	TS_SUPERVISOR_RAW_ONLY_CHAN(2), TS_SUPERVISOR_RAW_ONLY_CHAN(3),
	TS_SUPERVISOR_RAW_ONLY_CHAN(4), TS_SUPERVISOR_RAW_ONLY_CHAN(5),
	TS_SUPERVISOR_RAW_ONLY_CHAN(6), TS_SUPERVISOR_RAW_ONLY_CHAN(7),
	TS_SUPERVISOR_RAW_ONLY_CHAN(8), TS_SUPERVISOR_RAW_ONLY_CHAN(9),
	TS_SUPERVISOR_RAW_ONLY_CHAN(10), TS_SUPERVISOR_RAW_TEMP_CHAN(11),
};

static const struct ts_supervisor_adc_chip ts7100_chip = {
	.channels = ts7100_channels,
	.iio_channels = ts_supervisor_channels,
	.num_channels = ARRAY_SIZE(ts_supervisor_channels),
};

static const struct ts_supervisor_adc_chip ts7180_chip = {
	.channels = ts7180_channels,
	.iio_channels = ts_supervisor_channels,
	.num_channels = ARRAY_SIZE(ts_supervisor_channels),
};

static const struct ts_supervisor_adc_chip ts7553v2_chip = {
	.channels = ts7553v2_channels,
	.iio_channels = ts7553v2_iio_channels,
	.num_channels = ARRAY_SIZE(ts7553v2_iio_channels),
	.snapshot_size = 32,
	.gen0 = true,
};

static const struct ts_supervisor_adc_chip ts7680_chip = {
	.channels = ts7680_channels,
	.iio_channels = ts7680_iio_channels,
	.num_channels = ARRAY_SIZE(ts7680_iio_channels),
	.snapshot_size = 28,
	.gen0 = true,
};

static int ts_supervisor_adc_xlate(struct iio_dev *indio,
				   const struct fwnode_reference_args *spec)
{
	unsigned int i;

	if (spec->nargs != 1)
		return -EINVAL;

	for (i = 0; i < indio->num_channels; i++)
		if (indio->channels[i].channel == spec->args[0])
			return i;

	return -EINVAL;
}

static int ts_supervisor_adc_read_label(struct iio_dev *indio,
					const struct iio_chan_spec *chan,
					char *label)
{
	struct ts_supervisor_adc *adc = iio_priv(indio);

	return sysfs_emit(label, "%s\n",
			  adc->chip->channels[chan->channel].label);
}

static int ts_supervisor_adc_read_sample(struct ts_supervisor_adc *adc,
					 const struct ts_supervisor_adc_channel *channel,
					 u16 *sample)
{
	u8 data[32];
	int ret;

	if (adc->chip->gen0) {
		ret = i2c_master_recv(adc->client, data,
				      adc->chip->snapshot_size);
		if (ret != adc->chip->snapshot_size)
			return ret < 0 ? ret : -EIO;
	} else {
		ret = regmap_bulk_read(adc->regmap, channel->reg, data, 2);
		if (ret)
			return ret;
	}

	if (adc->chip->gen0)
		*sample = (data[channel->reg] << 8) |
			data[channel->reg + 1];
	else
		*sample = (data[0] << 8) | data[1];
	return 0;
}

static int ts_supervisor_adc_read(struct iio_dev *indio,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ts_supervisor_adc *adc = iio_priv(indio);
	const struct ts_supervisor_adc_channel *channel =
		&adc->chip->channels[chan->channel];
	u16 sample;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		if (!channel->temperature)
			return -EINVAL;
		ret = ts_supervisor_adc_read_sample(adc, channel, &sample);
		if (ret)
			return ret;
		*val = sample * 1000;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (channel->temperature && !channel->raw_temperature)
			return -EINVAL;
		ret = ts_supervisor_adc_read_sample(adc, channel, &sample);
		if (ret)
			return ret;
		*val = sample;
		*val2 = 0;
		return channel->raw_format;
	case IIO_CHAN_INFO_SCALE:
		*val = channel->scale;
		*val2 = channel->scale2;
		return channel->scale_format;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ts_supervisor_adc_info = {
	.read_raw = ts_supervisor_adc_read,
	.read_label = ts_supervisor_adc_read_label,
	.fwnode_xlate = ts_supervisor_adc_xlate,
};

static int ts_supervisor_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio;
	struct ts_supervisor_adc *adc;

	indio = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio)
		return -ENOMEM;
	adc = iio_priv(indio);
	adc->chip = device_get_match_data(dev);
	if (!adc->chip)
		return -EINVAL;
	if (adc->chip->gen0) {
		adc->client = to_i2c_client(dev->parent);
	} else {
		adc->regmap = dev_get_regmap(dev->parent, NULL);
		if (!adc->regmap)
			return -ENODEV;
	}
	indio->name = "ts-supervisor-adc";
	indio->dev.of_node = dev->of_node;
	indio->info = &ts_supervisor_adc_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = adc->chip->iio_channels;
	indio->num_channels = adc->chip->num_channels;
	return devm_iio_device_register(dev, indio);
}

static const struct of_device_id ts_supervisor_adc_match[] = {
	{ .compatible = "technologic,ts7100-supervisor-adc", .data = &ts7100_chip },
	{ .compatible = "technologic,ts7180-supervisor-adc", .data = &ts7180_chip },
	{ .compatible = "technologic,ts7553v2-supervisor-adc", .data = &ts7553v2_chip },
	{ .compatible = "technologic,ts7680-supervisor-adc", .data = &ts7680_chip },
	{ }
};
MODULE_DEVICE_TABLE(of, ts_supervisor_adc_match);

static struct platform_driver ts_supervisor_adc_driver = {
	.driver = {
		.name = "ts-supervisor-adc",
		.of_match_table = ts_supervisor_adc_match,
	},
	.probe = ts_supervisor_adc_probe,
};
module_platform_driver(ts_supervisor_adc_driver);

MODULE_DESCRIPTION("embeddedTS supervisory controller ADC");
MODULE_LICENSE("GPL");
