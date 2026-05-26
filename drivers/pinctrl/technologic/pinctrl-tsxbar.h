/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __PINCTRL_TSXBAR_H
#define __PINCTRL_TSXBAR_H

struct tsxbar_desc_function {
	const char *name;
	u8 funcsel;
	u8 highz_pinidx;
	u8 highz_en;
};

struct tsxbar_desc_group {
	const char *name;
	u8 pinidx;
	struct tsxbar_desc_function *functions;
};

struct tsxbar_pinctrl_desc {
	const struct tsxbar_desc_group *groups;
	unsigned int ngroups;
};

struct tsxbar_pinctrl_function {
	const char *name;
	const char **groups;
	unsigned int ngroups;
};

#define TSXBAR_PINCTRL_GROUP(_name, _pinidx, ...)		\
	{								\
		.name = _name,						\
		.pinidx = _pinidx,					\
		.functions = (struct tsxbar_desc_function[]){		\
			__VA_ARGS__, { } },				\
	}

#define TSXBAR_PINCTRL_FUNCTION(_name, _funcsel, _hz_pinidx, _hz_en)	\
	{								\
		.name         = (_name),				\
		.funcsel      = (_funcsel),				\
		.highz_pinidx = (_hz_pinidx),				\
		.highz_en     = (_hz_en),				\
	}

int tsxbar_pinctrl_probe(struct platform_device *pdev,
			 const struct tsxbar_pinctrl_desc *desc);

#endif /* __PINCTRL_TSXBAR_H */
