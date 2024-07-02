/* LICENSE HERE */

/* Simple pinctrl driver for the Mikrobus socket on the TS-7250-V3. The FPGA
 * provides all of the resources connected to the Mikrobus socket, however, the
 * signal MUXing takes place in an arbitrary register with one bit per group of
 * pins for any given function.
 * e.g. bit 4 set puts SPI pins as GPIO, clear has them as SPI. same with bit 5
 * for I2C, and so on.
 *
 * There is minimal flexibility here as it is intended for this one specific
 * platform which has a register layout that will not change over time.
 */

#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

#include "core.h"
#include "pinctrl-utils.h"
#include "pinmux.h"

struct mikro_group {
	const struct pingroup group;
	const unsigned int bit;
};

#define MIKRO_PIN(pin) PINCTRL_PIN(pin, #pin)
static const struct pinctrl_pin_desc mikro_pins[] = {
	MIKRO_PIN(0), /* Reset is GPIO only */
	MIKRO_PIN(1), /* AN unsure at this time */
	MIKRO_PIN(2), /* INT is gpio only */
	MIKRO_PIN(3), /* 3 is 180 rotation indicator only */
	MIKRO_PIN(4),
	MIKRO_PIN(5),
	MIKRO_PIN(6),
	MIKRO_PIN(7),
	MIKRO_PIN(8),
	MIKRO_PIN(9),
	MIKRO_PIN(10),
	MIKRO_PIN(11),
	MIKRO_PIN(12),
};

static const unsigned int gpio_pins[] = {0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
static const unsigned int pwm_pins[] = { 4 };
static const unsigned int uart_pins[] = { 9, 10 };
static const unsigned int i2c_pins[] = { 11, 12 };
static const unsigned int spi_pins[] = { 5, 6, 7, 8 };

static const struct mikro_group mikro_groups[] = {
	{ PINCTRL_PINGROUP("gpio_grp", gpio_pins, ARRAY_SIZE(gpio_pins)),	0 },
	{ PINCTRL_PINGROUP("pwm_grp", pwm_pins, ARRAY_SIZE(pwm_pins)),		6 },
	{ PINCTRL_PINGROUP("uart_grp", uart_pins, ARRAY_SIZE(uart_pins)),	7 },
	{ PINCTRL_PINGROUP("i2c_grp", i2c_pins, ARRAY_SIZE(i2c_pins)),		5 },
	{ PINCTRL_PINGROUP("spi_grp", spi_pins, ARRAY_SIZE(spi_pins)),		4 },
};

/* Setting up pin functions */
static const char * const gpio_groups[] = {
	"pwm_grp",
	"uart_grp",
	"i2c_grp",
	"spi_grp",
};
static const char * const pwm_groups[] = { "pwm_grp" };
static const char * const uart_groups[] = { "uart_grp" };
static const char * const i2c_groups[] = { "i2c_grp" };
static const char * const spi_groups[] = { "spi_grp" };

/* TODO: Investigate "generic" pinctrl/conf/mux functions, e.g. this could be
 * handled by struct function_desc but is only a part of the generic system.
 * I'm not sure what requirements or other offloading we can do with that, the
 * documentation doesn't really talk about it.
 */
struct mikro_function {
	const char *name;
	const char * const *groups;
	const unsigned int ngroups;
};

static const struct mikro_function mikro_functions[] = {
	{ "gpio", gpio_groups, ARRAY_SIZE(gpio_groups) },
	{ "pwm", pwm_groups, ARRAY_SIZE(pwm_groups) },
	{ "uart", uart_groups, ARRAY_SIZE(uart_groups) },
	{ "i2c", i2c_groups, ARRAY_SIZE(i2c_groups) },
	{ "spi", spi_groups, ARRAY_SIZE(spi_groups) },
};

/*
 * Pin group handling
 */
static int mikro_get_groups_count(struct pinctrl_dev *pctldev)
{
	pr_debug("%s: %d", __func__, ARRAY_SIZE(mikro_groups));
	return ARRAY_SIZE(mikro_groups);
}

static const char *mikro_get_group_name(struct pinctrl_dev *pctrldev, unsigned int sel)
{
	pr_debug("%s: %s\n", __func__, mikro_groups[sel].group.name);
	return mikro_groups[sel].group.name;
}

static int mikro_get_group_pins(struct pinctrl_dev *pctrldev, unsigned int sel,
				const unsigned **pins,
				unsigned *num_pins)
{
	*pins = mikro_groups[sel].group.pins;
	*num_pins = mikro_groups[sel].group.npins;

	pr_debug("%s: pins %p (%p), npins %d\n", __func__,
		mikro_groups[sel].group.pins,
		pins,
		mikro_groups[sel].group.npins);

	return 0;
}

static void mikro_pin_dbg_show(struct pinctrl_dev *pctrldev, struct seq_file *s,
				unsigned int offs)
{
	seq_printf(s, " %s", dev_name(pctrldev->dev));
}

static const struct pinctrl_ops mikro_pinctrl_ops = {
	.get_groups_count	= mikro_get_groups_count,
	.get_group_name		= mikro_get_group_name,
	.get_group_pins		= mikro_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinctrl_utils_free_map,
	.pin_dbg_show		= mikro_pin_dbg_show,
};

/*
 * Pin function handling
 */
static int mikro_get_functions_count(struct pinctrl_dev *pctrldev)
{
	pr_debug("%s: %d", __func__, ARRAY_SIZE(mikro_functions));
	return ARRAY_SIZE(mikro_functions);
}

static const char *mikro_get_function_name(struct pinctrl_dev *pctrldev,
					unsigned int sel)
{
	pr_debug("%s: %s\n", __func__, mikro_functions[sel].name);
	return mikro_functions[sel].name;
}

static int mikro_get_function_groups(struct pinctrl_dev *pctrldev, unsigned int sel,
				const char * const **groups,
				unsigned * const num_groups)
{
	*groups = mikro_functions[sel].groups;
	*num_groups = mikro_functions[sel].ngroups;

	pr_debug("%s: groups %p (%p), ngroups %d, sel %d\n", __func__,
		mikro_functions[sel].groups,
		groups,
		mikro_functions[sel].ngroups,
		sel);

	return 0;
}

static int mikro_set_mux(struct pinctrl_dev *pctrldev, unsigned int sel,
			unsigned int group)
{
	void __iomem *syscon_reg08 = pinctrl_dev_get_drvdata(pctrldev);
	unsigned int val;
	unsigned int mux_bit = mikro_groups[group].bit;

	/* TODO: This probably should have a lock since we do need to RMW */
	/* TODO: With a lock, we probably also want a pointer to the struct
	 * data we would be using dynamically, rather than using the static
	 * structs defined here for futureproofing.
	 */
	val = readl(syscon_reg08);
	if (!sel)
		val |= BIT(mux_bit);
	else
		val &= ~BIT(mux_bit);
	writel(val, syscon_reg08);

	pr_debug("new reg08 val: 0x%X\n", val);

	return 0;
}

static const struct pinmux_ops mikro_pinmux_ops = {
	.get_functions_count	= mikro_get_functions_count,
	.get_function_name	= mikro_get_function_name,
	.get_function_groups	= mikro_get_function_groups,
	.set_mux		= mikro_set_mux,
	.strict			= true,
};

static struct pinctrl_desc mikro_pinctrl_desc = {
	.name		= "ts7250v3-mikro-pinctrl",
	.owner		= THIS_MODULE,
	.pctlops	= &mikro_pinctrl_ops,
	.pmxops		= &mikro_pinmux_ops,
	.pins		= mikro_pins,
	.npins		= ARRAY_SIZE(mikro_pins),
};

/* TODO: Would this benefit from a spinlock? */
static int mikro_pinctrl_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	void __iomem *membase;
	struct pinctrl_dev *pctrldev = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("Cannot get device address\n");
		return -EFAULT;
	}

	membase = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(membase)) {
		pr_err("Could not map resource\n");
		return -ENXIO;
	}

	pctrldev = devm_pinctrl_register(&pdev->dev, &mikro_pinctrl_desc, membase);
	if (IS_ERR(pctrldev))
		return dev_err_probe(&pdev->dev, PTR_ERR(pctrldev),
					"Failed to register pinctrl device");

	return 0;
}

static const struct of_device_id mikro_pinctrl_dt_match[] = {
	{ .compatible = "technologic,ts7250v3-mikro-pinctrl", },
	{ },
};

static struct platform_driver mikro_pinctrl_driver = {
	.probe			= mikro_pinctrl_probe,
	.driver = {
		.name		= "ts7250v3-mikro-pinctrl",
		.of_match_table	= mikro_pinctrl_dt_match,
	},
};

static int __init mikro_pinctrl_init(void)
{
	return platform_driver_register(&mikro_pinctrl_driver);
}
module_init(mikro_pinctrl_init);

static void __exit mikro_pinctrl_exit(void)
{
	platform_driver_unregister(&mikro_pinctrl_driver);
}
module_exit(mikro_pinctrl_exit);

MODULE_DESCRIPTION("TS-7250-V3 Mikrobus pinctrl");
MODULE_LICENSE("GPL");

