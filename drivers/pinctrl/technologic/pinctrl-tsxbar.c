// SPDX-License-Identifier: GPL-2.0
/*
 * embeddedTS FPGA XBAR pinmux driver
 *
 * Copyright (C) 2025 embeddedTS
 *
 * Mark Featherston <mark@embeddedts.com>
 */

#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "../pinctrl-utils.h"
#include "pinctrl-tsxbar.h"

#define XBAR_PIN_WIDTH 4
#define XBAR_PIN_HIGHZ BIT(4)

#define TSXBAR_PIN_CELL_SIZE 16  // 4 * 4 bytes (4 cells per pin)

struct tsxbar_pinctrl {
	struct regmap *regmap;
	struct device *dev;
	const struct tsxbar_pinctrl_desc *desc;
	struct tsxbar_pinctrl_function *functions;
	unsigned int nfunctions;
	struct pinctrl_dev *pctrl_dev;
};

static int tsxbar_pinctrl_get_group_count(struct pinctrl_dev *pctrl_dev)
{
	struct tsxbar_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pctrl->desc->ngroups;
}

static const char *tsxbar_pinctrl_get_group_name(struct pinctrl_dev *pctrl_dev,
						 unsigned int group)
{
	struct tsxbar_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pctrl->desc->groups[group].name;
}

static int tsxbar_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
					 struct device_node *np,
					 struct pinctrl_map **map,
					 unsigned int *num_maps)
{
	const struct tsxbar_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct tsxbar_desc_group *group;
	const struct tsxbar_desc_function *func;
	const __be32 *list;
	int size, num_pins;
	struct pinctrl_map *new_map;
	int i;
	u32 pinidx, funcsel;

	list = of_get_property(np, "xbar,pins", &size);
	if (!list || size % (4 * sizeof(u32)))
		return -EINVAL;

	num_pins = size / (4 * sizeof(u32));
	new_map = kcalloc(num_pins, sizeof(*new_map), GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	for (i = 0; i < num_pins; i++) {
		/* <pin_idx> <mux_mode> <highz_pin_idx> <highz_en> */
		pinidx = be32_to_cpu(*list++);
		funcsel = be32_to_cpu(*list++);
		list += 2; /* skip high-Z info; driver handles it at runtime */
		group = NULL;
		func = NULL;

		for (int g = 0; g < pctrl->desc->ngroups; g++) {
			if (pctrl->desc->groups[g].pinidx == pinidx) {
				group = &pctrl->desc->groups[g];
				for (int f = 0; group->functions[f].name; f++) {
					if (group->functions[f].funcsel == funcsel) {
						func = &group->functions[f];
						break;
					}
				}
				break;
			}
		}

		if (!group || !func) {
			dev_err(pctrl->dev, "No matching group/function for pinidx=0x%x funcsel=0x%x\n",
				pinidx, funcsel);
			kfree(new_map);
			return -EINVAL;
		}

		new_map[i].type = PIN_MAP_TYPE_MUX_GROUP;
		new_map[i].data.mux.group = group->name;
		new_map[i].data.mux.function = func->name;
	}

	*map = new_map;
	*num_maps = num_pins;

	return 0;
}

static const struct pinctrl_ops tsxbar_pinctrl_ops = {
	.get_groups_count	= &tsxbar_pinctrl_get_group_count,
	.get_group_name		= &tsxbar_pinctrl_get_group_name,
	.dt_node_to_map		= &tsxbar_pinctrl_dt_node_to_map,
	.dt_free_map		= &pinctrl_utils_free_map,
};

static int tsxbar_pinmux_get_functions_count(struct pinctrl_dev *pctrl_dev)
{
	struct tsxbar_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pctrl->nfunctions;
}

static const char *tsxbar_pinmux_get_function_name(struct pinctrl_dev *pctrl_dev,
						   unsigned int function)
{
	struct tsxbar_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pctrl->functions[function].name;
}

static int tsxbar_pinmux_get_function_groups(struct pinctrl_dev *pctrl_dev,
					     unsigned int function,
					     const char *const **groups,
					     unsigned *const num_groups)
{
	struct tsxbar_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	*groups = pctrl->functions[function].groups;
	*num_groups = pctrl->functions[function].ngroups;

	return 0;
}

static struct tsxbar_desc_function *
tsxbar_pinctrl_find_function_by_name(struct tsxbar_pinctrl *pctrl,
				     const struct tsxbar_desc_group *group,
				     const char *fname)
{
	struct tsxbar_desc_function *function = group->functions;

	while (function->name) {
		if (!strcmp(function->name, fname))
			return function;

		function++;
	}

	return NULL;
}

static int tsxbar_pinmux_set(struct pinctrl_dev *pctrl_dev,
			     unsigned int function,
			     unsigned int group)
{
	struct tsxbar_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	const struct tsxbar_desc_group *gdesc = pctrl->desc->groups + group;
	struct tsxbar_pinctrl_function *func = pctrl->functions + function;
	struct tsxbar_desc_function *fdesc;
	u32 shift;
	u32 reg;
	u32 mask;
	u32 val;

	fdesc = tsxbar_pinctrl_find_function_by_name(pctrl, gdesc, func->name);
	if (!fdesc)
		return -EINVAL;

	reg   = XBAR_PIN_WIDTH * (gdesc->pinidx / 8);
	shift = XBAR_PIN_WIDTH * (gdesc->pinidx % 8);
	mask  = GENMASK(shift + XBAR_PIN_WIDTH - 1, shift);
	val   = fdesc->funcsel << shift;

	dev_dbg(pctrl->dev,
		"tsxbar: mux group=%s func=%s pinidx=%u reg=0x%03x shift=%u mask=0x%08x val=0x%08x\n",
		gdesc->name, func->name, gdesc->pinidx,
		reg, shift, mask, val);

	regmap_update_bits(pctrl->regmap, reg, mask, val);

	/* If high-z is set, we need to set the input pin as high-z */
	if (fdesc->highz_en) {
		reg   = XBAR_PIN_WIDTH * (fdesc->highz_pinidx / 8);
		shift = XBAR_PIN_WIDTH * (fdesc->highz_pinidx % 8);
		mask  = GENMASK(shift + XBAR_PIN_WIDTH - 1, shift);
		val   = XBAR_PIN_HIGHZ << shift;

		dev_dbg(pctrl->dev,
			"tsxbar: high-z pinidx=%u reg=0x%03x shift=%u mask=0x%08x val=0x%08x\n",
			fdesc->highz_pinidx,
			reg, shift, mask, val);
		regmap_update_bits(pctrl->regmap, reg, mask, val);
	}

	return 0;
}

static const struct pinmux_ops tsxbar_pinmux_ops = {
	.get_functions_count = &tsxbar_pinmux_get_functions_count,
	.get_function_name = &tsxbar_pinmux_get_function_name,
	.get_function_groups = &tsxbar_pinmux_get_function_groups,
	.set_mux = &tsxbar_pinmux_set,
};

static struct pinctrl_desc tsxbar_pctrl_desc = {
	.name = "tsxbar-pinctrl",
	.pctlops = &tsxbar_pinctrl_ops,
	.pmxops = &tsxbar_pinmux_ops,
	.owner = THIS_MODULE,
};

static const struct regmap_config txbar_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x3FF,
};

static void tsxbar_pinctrl_add_function(struct tsxbar_pinctrl *pctrl,
					const char *fname)
{
	struct tsxbar_pinctrl_function *func = pctrl->functions;

	/* Skip functions that exist */
	while (func->name) {
		if (!strcmp(func->name, fname)) {
			func->ngroups++;
			return;
		}
		func++;
	}

	func->name = fname;
	func->ngroups = 1;
	pctrl->nfunctions++;
}

static unsigned int tsxbar_count_functions(const struct tsxbar_pinctrl_desc *d)
{
	unsigned int total = 0;
	unsigned int i;

	for (i = 0; i < d->ngroups; i++)
		if (d->groups[i].functions)
			for (const struct tsxbar_desc_function *f =
			     d->groups[i].functions; f->name; f++)
				total++;

	return total;
}

static int tsxbar_add_unique_funcs(struct tsxbar_pinctrl *pctrl)
{
	const struct tsxbar_desc_function *f;
	unsigned int i;

	for (i = 0; i < pctrl->desc->ngroups; i++) {
		f = pctrl->desc->groups[i].functions;

		while (f && f->name) {
			tsxbar_pinctrl_add_function(pctrl, f->name);
			f++;
		}
	}

	return 0;
}

static int tsxbar_alloc_groups(struct platform_device *pdev,
			       struct tsxbar_pinctrl *pctrl)
{
	struct tsxbar_pinctrl_function *fn;
	unsigned int i;

	for (i = 0; i < pctrl->nfunctions; i++) {
		fn = &pctrl->functions[i];

		if (!fn->ngroups) /* If a function is never referenced */
			continue;

		fn->groups = devm_kcalloc(&pdev->dev, fn->ngroups,
					  sizeof(char *), GFP_KERNEL);
		if (!fn->groups)
			return -ENOMEM;

		fn->ngroups = 0;
	}
	return 0;
}

static void tsxbar_map_funcs_to_groups(struct tsxbar_pinctrl *pctrl)
{
	struct tsxbar_pinctrl_function *pinctrl_fn;
	const struct tsxbar_desc_function *func;
	const struct tsxbar_desc_group *grp;
	unsigned int g, i;

	for (g = 0; g < pctrl->desc->ngroups; g++) {
		grp = &pctrl->desc->groups[g];
		func = grp->functions;

		while (func && func->name) {
			for (i = 0; i < pctrl->nfunctions; i++)
				if (!strcmp(pctrl->functions[i].name, func->name)) {
					pinctrl_fn = &pctrl->functions[i];
					pinctrl_fn->groups[pinctrl_fn->ngroups++] = grp->name;
					break;
				}
			func++;
		}
	}
}

static int tsxbar_pinctrl_build_state(struct platform_device *pdev)
{
	struct tsxbar_pinctrl *pctrl = platform_get_drvdata(pdev);
	int max_functions = tsxbar_count_functions(pctrl->desc);
	int ret;

	BUG_ON(max_functions == 0);

	pctrl->functions = devm_kcalloc(&pdev->dev, max_functions,
					sizeof(*pctrl->functions), GFP_KERNEL);
	if (!pctrl->functions)
		return -ENOMEM;

	tsxbar_add_unique_funcs(pctrl);

	ret = tsxbar_alloc_groups(pdev, pctrl);
	if (ret)
		return ret;

	tsxbar_map_funcs_to_groups(pctrl);

	return 0;
}

int tsxbar_pinctrl_probe(struct platform_device *pdev,
			 const struct tsxbar_pinctrl_desc *desc)
{
	struct device *dev = &pdev->dev;
	struct tsxbar_pinctrl *pctrl;
	void __iomem *base;
	int ret;

	pctrl = devm_kzalloc(dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pctrl);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pctrl->regmap = devm_regmap_init_mmio(dev, base, &txbar_regmap_config);
	if (IS_ERR(pctrl->regmap))
		return PTR_ERR(pctrl->regmap);

	pctrl->dev = &pdev->dev;
	pctrl->desc = desc;

	ret = tsxbar_pinctrl_build_state(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot build crossbar state: %d\n", ret);
		return ret;
	}

	pctrl->pctrl_dev = devm_pinctrl_register(dev, &tsxbar_pctrl_desc,
						 pctrl);
	if (IS_ERR(pctrl->pctrl_dev)) {
		dev_err(dev, "failed to register pinctrl driver\n");
		return PTR_ERR(pctrl->pctrl_dev);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tsxbar_pinctrl_probe);
