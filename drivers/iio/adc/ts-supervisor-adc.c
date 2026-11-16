// SPDX-License-Identifier: GPL-2.0-only
/* embeddedTS supervisory microcontroller ADC register interface. */
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/math64.h>
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
	bool little_endian;
	u16 sample_mask;
	bool micro_temperature;
};

#define TS_SUPERVISOR_CHAN(n) { \
	.type = IIO_VOLTAGE, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), \
}

#define TS_SUPERVISOR_TEMP_CHAN(n) { \
	.type = IIO_TEMP, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), \
}

#define TS_SUPERVISOR_RAW_TEMP_CHAN(n) { \
	.type = IIO_TEMP, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
}

#define TS_SUPERVISOR_RAW_CURRENT_CHAN(n) { \
	.type = IIO_CURRENT, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), \
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
	[1] = TS_SUPERVISOR_MV_CHANNEL(1, "AN_SUP_CHRG"),
	[2] = TS_SUPERVISOR_MV_CHANNEL(2, "3.3V"),
	[3] = TS_SUPERVISOR_MV_CHANNEL(3, "8V_48V"),
	[4] = TS_SUPERVISOR_TEMP_CHANNEL(4, "TEMP_INITIAL"),
	[7] = TS_SUPERVISOR_MV_CHANNEL(7, "AN_SUP_CAP_1"),
	[8] = TS_SUPERVISOR_MV_CHANNEL(8, "AN_SUP_CAP_2"),
	[10] = TS_SUPERVISOR_TEMP_CHANNEL(10, "TEMP"),
};

static const struct ts_supervisor_adc_channel ts7180_channels[] = {
	[0] = TS_SUPERVISOR_RAW_CHANNEL(0, "5V_A"),
	[1] = TS_SUPERVISOR_RAW_CHANNEL(1, "AN_CHRG"),
	[2] = TS_SUPERVISOR_RAW_CHANNEL(2, "3.3V"),
	[3] = TS_SUPERVISOR_RAW_CHANNEL(3, "VIN"),
	[4] = TS_SUPERVISOR_TEMP_CHANNEL(4, "TEMP_INITIAL"),
	[7] = TS_SUPERVISOR_MV_CHANNEL(7, "AN_SUP_CAP_1"),
	[8] = TS_SUPERVISOR_MV_CHANNEL(8, "AN_SUP_CAP_2"),
	[10] = TS_SUPERVISOR_TEMP_CHANNEL(10, "TEMP"),
};

#define TS7800V2_REG(n) (1280 + (n) * 2)

#define TS7800V2_CHANNEL(name, n, scale_val) { \
	.label = (name), .reg = TS7800V2_REG(n), .raw_format = IIO_VAL_INT, \
	.scale = (scale_val), .scale2 = 1024, \
	.scale_format = IIO_VAL_FRACTIONAL, \
}

static const struct iio_chan_spec ts7800v2_iio_channels[] = {
	TS_SUPERVISOR_CHAN(0), TS_SUPERVISOR_CHAN(1),
	TS_SUPERVISOR_CHAN(2), TS_SUPERVISOR_CHAN(3),
	TS_SUPERVISOR_CHAN(4), TS_SUPERVISOR_CHAN(5),
	TS_SUPERVISOR_CHAN(6), TS_SUPERVISOR_CHAN(7),
	TS_SUPERVISOR_CHAN(8), TS_SUPERVISOR_CHAN(9),
	TS_SUPERVISOR_CHAN(10), TS_SUPERVISOR_CHAN(11),
	TS_SUPERVISOR_TEMP_CHAN(12),
};

static const struct ts_supervisor_adc_channel ts7800v2_channels[] = {
	[0] = TS7800V2_CHANNEL("CPU_CORE", 0, 2500),
	[1] = TS7800V2_CHANNEL("RAM_1.35V", 1, 2500),
	[2] = TS7800V2_CHANNEL("1.2V", 2, 2500),
	[3] = TS7800V2_CHANNEL("1.8V", 3, 2500),
	[4] = TS7800V2_CHANNEL("8V_30V", 4, 2500),
	[5] = TS7800V2_CHANNEL("5V_A", 5, 2500),
	[6] = TS7800V2_CHANNEL("AN_3.3V", 6, 2500),
	[7] = TS7800V2_CHANNEL("ADC_4", 7, 2500),
	[8] = TS7800V2_CHANNEL("ADC_3", 8, 2500),
	[9] = TS7800V2_CHANNEL("ADC_2", 9, 2500),
	[10] = TS7800V2_CHANNEL("ADC_1", 10, 2500),
	[11] = TS7800V2_CHANNEL("ADC_0", 11, 2500),
	[12] = { .label = "TEMP_SENS", .reg = TS7800V2_REG(12),
		 .temperature = true, .raw_format = IIO_VAL_INT },
};

#define TS_SUPERVISOR_GEN0_CHAN(name, offset) { \
	.label = (name), .reg = (offset), .raw_format = IIO_VAL_INT, \
	.scale = 2500, .scale2 = 1023, \
	.scale_format = IIO_VAL_FRACTIONAL, \
}

#define TS_SUPERVISOR_GEN0_CURRENT_CHANNEL(name, offset, shunt) { \
	.label = (name), .reg = (offset), .raw_format = IIO_VAL_INT, \
	.scale = 2500, .scale2 = 1023 * (shunt), \
	.scale_format = IIO_VAL_FRACTIONAL, \
}

static const struct ts_supervisor_adc_channel ts7553v2_channels[] = {
	[0] = TS_SUPERVISOR_GEN0_CHAN("AN_SUP_CAP_1", 0),
	[1] = TS_SUPERVISOR_GEN0_CHAN("AN_SUP_CAP_2", 2),
	[2] = TS_SUPERVISOR_GEN0_CHAN("AN_MAIN_4.7V", 4),
	[3] = TS_SUPERVISOR_GEN0_CHAN("VIN", 6),
	[4] = TS_SUPERVISOR_GEN0_CHAN("5V_A", 8),
	[5] = TS_SUPERVISOR_GEN0_CHAN("3.3V", 10),
	[6] = TS_SUPERVISOR_GEN0_CHAN("RAM_1.35V", 12),
	[7] = TS_SUPERVISOR_GEN0_CHAN("VDD_6UL_CORE", 14),
	[8] = TS_SUPERVISOR_GEN0_CHAN("AN_CHRG", 16),
	[9] = TS_SUPERVISOR_GEN0_CHAN("VDD_SOC_CAP", 18),
	[10] = TS_SUPERVISOR_GEN0_CHAN("VDD_ARM_CAP", 20),
};

static const struct ts_supervisor_adc_channel ts4100_channels[] = {
	[0] = TS_SUPERVISOR_GEN0_CHAN("AN_SUP_CAP_1", 0),
	[1] = TS_SUPERVISOR_GEN0_CHAN("AN_SUP_CAP_2", 2),
	[2] = TS_SUPERVISOR_GEN0_CHAN("AN_MAIN_4.7V", 4),
	[3] = TS_SUPERVISOR_GEN0_CHAN("MAIN_5V", 6),
	[4] = TS_SUPERVISOR_GEN0_CHAN("USB_OTG_5V", 8),
	[5] = TS_SUPERVISOR_GEN0_CHAN("3.3V", 10),
	[6] = TS_SUPERVISOR_GEN0_CHAN("RAM_1.5V", 12),
	[9] = TS_SUPERVISOR_GEN0_CHAN("VDD_6UL_CORE", 18),
	[10] = TS_SUPERVISOR_GEN0_CHAN("AN_CHRG", 20),
	[11] = TS_SUPERVISOR_GEN0_CHAN("VDD_SOC_CAP", 22),
	[12] = TS_SUPERVISOR_GEN0_CHAN("VDD_ARM_CAP", 24),
};

static const struct ts_supervisor_adc_channel ts7680_channels[] = {
	[0] = TS_SUPERVISOR_GEN0_CHAN("AN_SCAP_1", 0),
	[1] = TS_SUPERVISOR_GEN0_CHAN("AN_SCAP_2", 2),
	[2] = TS_SUPERVISOR_GEN0_CHAN("AN_VINT", 4),
	[3] = TS_SUPERVISOR_GEN0_CHAN("AN_VIN", 6),
	[4] = TS_SUPERVISOR_GEN0_CHAN("5V_BOOST", 8),
	[5] = TS_SUPERVISOR_GEN0_CHAN("AN_AUX_3.3V", 10),
	[6] = TS_SUPERVISOR_GEN0_CHAN("CPU_1.8V", 12),
	[7] = TS_SUPERVISOR_GEN0_CHAN("CPU_CORE", 18),
	[8] = TS_SUPERVISOR_GEN0_CHAN("AN_CHRG_2", 20),
	[9] = TS_SUPERVISOR_GEN0_CHAN("RAM_1.8V", 22),
	[10] = TS_SUPERVISOR_GEN0_CHAN("AN_CPU_3.3V", 24),
	[11] = {
		.label = "TEMP_RAW", .reg = 26, .temperature = true,
		.raw_temperature = true, .raw_format = IIO_VAL_INT,
	},
};

static const struct ts_supervisor_adc_channel ts7970_channels[] = {
	[0] = TS_SUPERVISOR_GEN0_CHAN("VDD_ARM_CAP", 0),
	[1] = TS_SUPERVISOR_GEN0_CHAN("VDD_HIGH_CAP", 2),
	[2] = TS_SUPERVISOR_GEN0_CHAN("VDD_SOC_CAP", 4),
	[3] = TS_SUPERVISOR_GEN0_CHAN("VDD_ARM", 6),
	[4] = TS_SUPERVISOR_GEN0_CURRENT_CHANNEL("AD_P10", 8, 110),
	[5] = TS_SUPERVISOR_GEN0_CURRENT_CHANNEL("AD_P11", 10, 110),
	[6] = TS_SUPERVISOR_GEN0_CURRENT_CHANNEL("AD_P12", 12, 110),
	[7] = TS_SUPERVISOR_GEN0_CHAN("VIN", 14),
	[8] = TS_SUPERVISOR_GEN0_CHAN("5V_A", 16),
	[9] = TS_SUPERVISOR_GEN0_CHAN("3.1V", 18),
	[10] = TS_SUPERVISOR_GEN0_CHAN("DDR_1.5V", 20),
	[11] = TS_SUPERVISOR_GEN0_CHAN("1.8V", 22),
	[13] = TS_SUPERVISOR_GEN0_CHAN("RAM_VREF", 26),
	[14] = TS_SUPERVISOR_GEN0_CHAN("3.3V", 28),
};

static const struct ts_supervisor_adc_channel ts7990_channels[] = {
	[0] = TS_SUPERVISOR_GEN0_CHAN("VIN", 0),
	[1] = TS_SUPERVISOR_GEN0_CHAN("5V_A", 2),
	[2] = TS_SUPERVISOR_GEN0_CHAN("AN_LCD_20V", 4),
	[3] = TS_SUPERVISOR_GEN0_CHAN("DDR_1.5V", 6),
	[4] = TS_SUPERVISOR_GEN0_CHAN("1.8V", 8),
	[5] = TS_SUPERVISOR_GEN0_CHAN("SUP_CAP_ANALOG", 10),
	[6] = TS_SUPERVISOR_GEN0_CHAN("BACK_LT-", 12),
	[7] = TS_SUPERVISOR_GEN0_CHAN("3.3V", 14),
	[8] = TS_SUPERVISOR_GEN0_CHAN("VDD_ARM_CAP", 16),
	[9] = TS_SUPERVISOR_GEN0_CHAN("VDD_SOC_CAP", 18),
};

static const struct iio_chan_spec ts7553v2_iio_channels[] = {
	TS_SUPERVISOR_CHAN(0), TS_SUPERVISOR_CHAN(1),
	TS_SUPERVISOR_CHAN(2), TS_SUPERVISOR_CHAN(3),
	TS_SUPERVISOR_CHAN(4), TS_SUPERVISOR_CHAN(5),
	TS_SUPERVISOR_CHAN(6), TS_SUPERVISOR_CHAN(7),
	TS_SUPERVISOR_CHAN(8), TS_SUPERVISOR_CHAN(9),
	TS_SUPERVISOR_CHAN(10),
};

static const struct iio_chan_spec ts4100_iio_channels[] = {
	TS_SUPERVISOR_CHAN(0), TS_SUPERVISOR_CHAN(1),
	TS_SUPERVISOR_CHAN(2), TS_SUPERVISOR_CHAN(3),
	TS_SUPERVISOR_CHAN(4), TS_SUPERVISOR_CHAN(5),
	TS_SUPERVISOR_CHAN(6), TS_SUPERVISOR_CHAN(9),
	TS_SUPERVISOR_CHAN(10), TS_SUPERVISOR_CHAN(11),
	TS_SUPERVISOR_CHAN(12),
};

static const struct iio_chan_spec ts7680_iio_channels[] = {
	TS_SUPERVISOR_CHAN(0), TS_SUPERVISOR_CHAN(1),
	TS_SUPERVISOR_CHAN(2), TS_SUPERVISOR_CHAN(3),
	TS_SUPERVISOR_CHAN(4), TS_SUPERVISOR_CHAN(5),
	TS_SUPERVISOR_CHAN(6), TS_SUPERVISOR_CHAN(7),
	TS_SUPERVISOR_CHAN(8), TS_SUPERVISOR_CHAN(9),
	TS_SUPERVISOR_CHAN(10), TS_SUPERVISOR_RAW_TEMP_CHAN(11),
};

static const struct iio_chan_spec ts7970_iio_channels[] = {
	TS_SUPERVISOR_CHAN(0), TS_SUPERVISOR_CHAN(1),
	TS_SUPERVISOR_CHAN(2), TS_SUPERVISOR_CHAN(3),
	TS_SUPERVISOR_RAW_CURRENT_CHAN(4), TS_SUPERVISOR_RAW_CURRENT_CHAN(5),
	TS_SUPERVISOR_RAW_CURRENT_CHAN(6), TS_SUPERVISOR_CHAN(7),
	TS_SUPERVISOR_CHAN(8), TS_SUPERVISOR_CHAN(9),
	TS_SUPERVISOR_CHAN(10), TS_SUPERVISOR_CHAN(11),
	TS_SUPERVISOR_CHAN(13),
	TS_SUPERVISOR_CHAN(14),
};

static const struct iio_chan_spec ts7990_iio_channels[] = {
	TS_SUPERVISOR_CHAN(0), TS_SUPERVISOR_CHAN(1),
	TS_SUPERVISOR_CHAN(2), TS_SUPERVISOR_CHAN(3),
	TS_SUPERVISOR_CHAN(4), TS_SUPERVISOR_CHAN(5),
	TS_SUPERVISOR_CHAN(6), TS_SUPERVISOR_CHAN(7),
	TS_SUPERVISOR_CHAN(8), TS_SUPERVISOR_CHAN(9),
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

static const struct ts_supervisor_adc_chip ts7800v2_chip = {
	.channels = ts7800v2_channels,
	.iio_channels = ts7800v2_iio_channels,
	.num_channels = ARRAY_SIZE(ts7800v2_iio_channels),
	.little_endian = true,
	.sample_mask = GENMASK(9, 0),
	.micro_temperature = true,
};

static const struct ts_supervisor_adc_chip ts7553v2_chip = {
	.channels = ts7553v2_channels,
	.iio_channels = ts7553v2_iio_channels,
	.num_channels = ARRAY_SIZE(ts7553v2_iio_channels),
	.snapshot_size = 32,
	.gen0 = true,
};

static const struct ts_supervisor_adc_chip ts4100_chip = {
	.channels = ts4100_channels,
	.iio_channels = ts4100_iio_channels,
	.num_channels = ARRAY_SIZE(ts4100_iio_channels),
	.snapshot_size = 28,
	.gen0 = true,
};

static const struct ts_supervisor_adc_chip ts7680_chip = {
	.channels = ts7680_channels,
	.iio_channels = ts7680_iio_channels,
	.num_channels = ARRAY_SIZE(ts7680_iio_channels),
	.snapshot_size = 28,
	.gen0 = true,
};

static const struct ts_supervisor_adc_chip ts7970_chip = {
	.channels = ts7970_channels,
	.iio_channels = ts7970_iio_channels,
	.num_channels = ARRAY_SIZE(ts7970_iio_channels),
	.snapshot_size = 32,
	.gen0 = true,
};

static const struct ts_supervisor_adc_chip ts7990_chip = {
	.channels = ts7990_channels,
	.iio_channels = ts7990_iio_channels,
	.num_channels = ARRAY_SIZE(ts7990_iio_channels),
	.snapshot_size = 32,
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
	else if (adc->chip->little_endian)
		*sample = (data[1] << 8) | data[0];
	else
		*sample = (data[0] << 8) | data[1];
	if (adc->chip->little_endian && *sample == U16_MAX)
		return -EAGAIN;
	if (adc->chip->sample_mask)
		*sample &= adc->chip->sample_mask;
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
		if (adc->chip->micro_temperature) {
			s64 temp = (s64)sample * 160156 - (764LL << 16);

			*val = div_s64(temp * 1000, 188088);
			return IIO_VAL_INT;
		}
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
	if (adc->chip->gen0)
		adc->client = to_i2c_client(dev->parent);
	else {
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
	{ .compatible = "technologic,ts4100-supervisor-adc", .data = &ts4100_chip },
	{ .compatible = "technologic,ts7680-supervisor-adc", .data = &ts7680_chip },
	{ .compatible = "technologic,ts7800v2-supervisor-adc", .data = &ts7800v2_chip },
	{ .compatible = "technologic,ts7970-supervisor-adc", .data = &ts7970_chip },
	{ .compatible = "technologic,ts7990-supervisor-adc", .data = &ts7990_chip },
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
