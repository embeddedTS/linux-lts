// SPDX-License-Identifier: GPL-2.0

/*
 * Driver for TS-7840's FPGA based RNG.  This outputs random data
 * based on a core that outputs continuous data based on metastability
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/hw_random.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

struct ts7840_rng_priv {
	struct hwrng rng;
	void __iomem *base;
};

static int ts7840_rng_data_read(struct hwrng *rng, u32 *buffer)
{
	struct ts7840_rng_priv *priv = (struct ts7840_rng_priv *)rng->priv;
	*buffer = readl(priv->base);
	return 4;
}

static int ts7840_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *name = dev_name(dev);
	struct ts7840_rng_priv *priv;
	struct hwrng *rng;

	priv = devm_kzalloc(dev, sizeof(struct ts7840_rng_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	rng = &priv->rng;

	platform_set_drvdata(pdev, priv);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base)) {
		dev_err(dev, "failed to remap I/O memory\n");
		return PTR_ERR(priv->base);
	}

	rng->name = name;
	rng->data_read = ts7840_rng_data_read;
	rng->priv = (unsigned long)priv;
	rng->quality = 1000;

	return hwrng_register(rng);
}

static const struct of_device_id ts7840_rng_of_match[] = {
	{ .compatible = "technologic,ts7840-rng", },
	{},
};
MODULE_DEVICE_TABLE(of, ts7840_rng_of_match);

static struct platform_driver ts7840_rng_driver = {
	.probe = ts7840_rng_probe,
	.driver = {
		.name = "ts7840-rng",
		.of_match_table = ts7840_rng_of_match,
	},
};
module_platform_driver(ts7840_rng_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_DESCRIPTION("TS-7840 RNG driver");
MODULE_LICENSE("GPL v2");
