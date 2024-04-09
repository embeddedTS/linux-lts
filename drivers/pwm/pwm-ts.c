// SPDX-License-Identifier: GPL-2.0
/*
 * PWM for embeddedTS TS-7250-V3, TS-7120, et al.
 * Copyright (C) 2021-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/slab.h>

/* Enabled bit is the only one that applies immediately.  All other registers
 * take effect when apply is set
 */
#define REG_CONFIG		0x0
/* Enable PWM Output */
#define ENABLED			(1 << 0)
/* 0 = idle high, active low.  1 = idle low, active high */
#define INVERSED		(1 << 1)

#define REG_PERIOD		0x2
#define REG_DUTY		0x4
#define PWM_DUTY_WIDTH		10
#define CYCLE_MASK		0x3ff
#define REG_SHIFT		0x6
#define SHIFT_MAX		12

struct ts_pwm {
	struct pwm_chip chip;
	void __iomem *base;
	struct clk *clk;
};

static inline struct ts_pwm *to_ts_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct ts_pwm, chip);
}

static int ts_pwm_calc(struct ts_pwm *ts,
		       unsigned int duty,
		       unsigned int period)
{
	unsigned long clk_rate = clk_get_rate(ts->clk);
	unsigned long long cycle;
	unsigned int  cnt, duty_cnt;
	u16 duty_reg;
	u8 shift;

	/* Calc shift & period reg */
	for (shift = 0; shift < SHIFT_MAX; shift++) {
		cycle = DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC,
					      (clk_rate / 100) >> shift);
		cnt = DIV_ROUND_CLOSEST(period * 100, (unsigned int)cycle);
		if (cnt <= CYCLE_MASK)
			break;
	}

	if (cnt > CYCLE_MASK)
		return -EINVAL;

	dev_dbg(ts->chip.dev, "cycle=%llu shift=%u cnt=%u\n",
		cycle, shift, cnt);


	if (duty == period) {
		duty_reg = cnt;
	} else if (duty == 0) {
		duty_reg = 0;
	} else {
		duty_cnt = DIV_ROUND_CLOSEST(duty * 100, (unsigned int)cycle);
		if (duty_cnt > CYCLE_MASK) {
			dev_err(ts->chip.dev, "unable to get duty cycle\n");
			return -EINVAL;
		}

		dev_dbg(ts->chip.dev, "shift=%u cnt=%u duty_cnt=%u\n",
			shift, cnt, duty_cnt);
		duty_reg = cnt - duty_cnt;
	}

	writew(cnt, ts->base + REG_PERIOD);
	writew(duty_reg, ts->base + REG_DUTY);
	writew(shift, ts->base + REG_SHIFT);

	return 0;
}

static int ts_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			const struct pwm_state *state)
{
	struct ts_pwm *ts = to_ts_pwm(chip);
	int err;
	u16 ctrl = 0;

	if (state->polarity == PWM_POLARITY_NORMAL)
		ctrl &= ~INVERSED;
	else
		ctrl |= INVERSED;

	if (state->enabled)
		ctrl |= ENABLED;

	err = ts_pwm_calc(ts, state->duty_cycle, state->period);
	if (err < 0)
		return err;

	writew(ctrl, ts->base + REG_CONFIG);

	return 0;
}

static const struct pwm_ops ts_pwm_ops = {
	.apply = ts_pwm_apply,
	.owner = THIS_MODULE,
};

static const struct of_device_id ts_pwm_matches[] = {
	{ .compatible = "technologic,pwm", },
	{},
};
MODULE_DEVICE_TABLE(of, ts_pwm_matches);

static int ts_pwm_probe(struct platform_device *pdev)
{
	struct ts_pwm *ts;
	struct resource *regs;
	int err;

	ts = devm_kzalloc(&pdev->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ts->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(ts->base))
		return PTR_ERR(ts->base);

	ts->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ts->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(ts->clk);
	}

	platform_set_drvdata(pdev, ts);

	ts->chip.dev = &pdev->dev;
	ts->chip.ops = &ts_pwm_ops;
	ts->chip.base = -1;
	ts->chip.npwm = 1;

	pm_runtime_enable(&pdev->dev);

	err = pwmchip_add(&ts->chip);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register PWM chip: %d\n", err);
		return err;
	}

	return 0;
}

static int ts_pwm_remove(struct platform_device *pdev)
{
	struct ts_pwm *ts = platform_get_drvdata(pdev);

	return pwmchip_remove(&ts->chip);
}

static struct platform_driver ts_pwm_driver = {
	.driver = {
		.name = "ts-pwm",
		.of_match_table = ts_pwm_matches,
	},
	.probe = ts_pwm_probe,
	.remove = ts_pwm_remove,
};
module_platform_driver(ts_pwm_driver);

MODULE_ALIAS("platform:ts-pwm");
MODULE_DESCRIPTION("embeddedTS PS");
MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_LICENSE("GPL");
