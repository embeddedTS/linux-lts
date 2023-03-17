// SPDX-License-Identifier: GPL-2.0
/*
 * Digital I/O driver for Technologic Systems TS-7800-V2
 */

#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#define TS7800V2_NR_DIO	   121

#define LCD_DIO_DATA	0x4
#define LCD_DIO_OUT	0X8
#define MISC_IO		0xC
#define ROW_A_DATA	0x10
#define ROW_B_DATA	0x14
#define ROW_C_DATA	0x18
#define ROW_D_DATA	0x1C
#define ROW_A_DIR	0x20
#define ROW_B_DIR	0x24
#define ROW_C_DIR	0x28
#define ROW_D_DIR	0x2c

struct ts7800v2_gpio_priv {
	void __iomem  *syscon;
	struct gpio_chip gpio_chip;
	spinlock_t lock;
};

/* DIO number : bit position in relevant syscon reg or regs */
static unsigned int dio_bitpositions[] = {
	0,   // 0 DIO_01,    CN8 pin 1
	2,   // 1 DIO_03,    CN8 pin 3
	3,   // 2 DIO_04,    CN8 pin 4
	4,   // 3 DIO_05,    CN8 pin 5
	5,   // 4 SPI_FRAME, CN8 pin 6
	6,   // 5 DIO_07,    CN8 pin 7
	7,   // 6 DIO_08,    CN8 pin 8
	8,   // 7 DIO_09,    CN8 pin 9
	9,   // 8 SPI_MISO,  CN8 pin 10  (read-only)
	10,  // 9 DIO_11,    CN8 pin 11
	11, // 10 SPI_MOSI,  CN8 pin 12
	12, // 11 DIO_13,    CN8 pin 13
	13, // 12 SPI_CLK,   CN8 pin 14
	14, // 13 DIO_15,    CN8 pin 15

	18, // 14 RS,        CN7 pin 3
	19, // 15 BIAS,      CN7 pin 4
	20, // 16 EN,        CN7 pin 5
	21, // 17 RW,        CN7 pin 6
	22, // 18 DB1,       CN7 pin 7
	23, // 19 DB0,       CN7 pin 8
	24, // 20 DB3,       CN7 pin 9
	25, // 21 DB2,       CN7 pin 10
	26, // 22 DB5,       CN7 pin 11
	27, // 23 DB4,       CN7 pin 12
	28, // 24 DB7,       CN7 pin 13
	29, // 25 DB6,       CN7 pin 14

	0, // 26 A[0]        CN5 pin A1
	1, // 27 A[1]
	2, // 28 A[2]
	3, // 29 A[3]
	4, // 30 A[4]
	5, // 31 A[5]
	6, // 32 A[6]
	7, // 33 A[7]
	8, // 34 A[8]
	9, // 35 A[9]
	10, // 36 A[10]
	11, // 37 A[11]
	12, // 38 A[12]
	13, // 39 A[13]
	14, // 40 A[14]
	15, // 41 A[15]
	16, // 42 A[16]
	17, // 43 A[17]
	18, // 44 A[18]
	19, // 45 A[19]
	20, // 46 A[20]
	21, // 47 A[21]
	22, // 48 A[22]
	23, // 49 A[23]
	24, // 50 A[24]
	25, // 51 A[25]
	26, // 52 A[26]
	27, // 53 A[27]
	28, // 54 A[28]
	29, // 55 A[28]
	30, // 56 A[30]    CN5 pin 31
	31, // 57 A[31]    whoops, there is no A[31] on CN5

	1, // 58 B[1]       CN5 pin B2
	3, // 59 B[3]       CN5 pin B4
	5, // 60 B[5]       CN5 pin B6
	6, // 61 B[6]       CN5 pin B7
	7, // 62 B[7]       CN5 pin B8
	10, // 63 B[10]     CN5 pin B11
	11, // 64 B[11]     CN5 pin B12
	12, // 65 B[12]     CN5 pin B13
	13, // 66 B[13]     CN5 pin B14
	14, // 67 B[14]     CN5 pin B15
	15, // 68 B[15]     CN5 pin B16
	16, // 69 B[16]     CN5 pin B17
	17, // 70 B[17]     CN5 pin B18
	18, // 71 B[18]     CN5 pin B19
	19, // 72 B[19]     CN5 pin B20
	20, // 73 B[20]     CN5 pin B21
	21, // 74 B[21]     CN5 pin B22
	22, // 75 B[22]     CN5 pin B23
	23, // 76 B[23]     CN5 pin B24
	24, // 77 B[24]     CN5 pin B25
	25, // 78 B[25]     CN5 pin B26
	26, // 79 B[26]     CN5 pin B27
	27, // 80 B[27]     CN5 pin B28
	29, // 81 B[29]     CN5 pin B30
	31, // 82 B[31]     CN5 pin B32

	1, // 83 C[1]       CN6 pin C1
	2, // 84 C[2]       CN6 pin C2
	3, // 85 C[3]       CN6 pin C3
	4, // 86 C[4]       CN6 pin C4
	5, // 87 C[5]       CN6 pin C5
	6, // 88 C[6]       CN6 pin C6
	7, // 89 C[7]       CN6 pin C7
	8, // 90 C[8]       CN6 pin C8
	9, // 91 C[9]       CN6 pin C9
	10, // 92 C[10]     CN6 pin C10
	11, // 93 C[11]     CN6 pin C11
	12, // 94 C[12]     CN6 pin C12
	13, // 95 C[13]     CN6 pin C13
	14, // 96 C[14]     CN6 pin C14
	15, // 97 C[15]     CN6 pin C15
	16, // 98 C[16]     CN6 pin C16
	17, // 99 C[17]     CN6 pin C17
	18, // 100 C[18]    CN6 pin C18

	1, // 101 D[1]      CN6 pin D1
	2, // 102 D[2]      CN6 pin D2
	3, // 103 D[3]      CN6 pin D3
	4, // 104 D[4]      CN6 pin D4  (read-only)
	5, // 105 D[5]      CN6 pin D5  (read-only)
	6, // 106 D[6]      CN6 pin D6  (read-only)
	7, // 107 D[7]      CN6 pin D7  (read-only)
	9, // 108 D[9]      CN6 pin D9
	10, // 109 D[10]    CN6 pin D10
	11, // 110 D[11]    CN6 pin D11
	12, // 111 D[12]    CN6 pin D12
	13, // 112 D[13]    CN6 pin D13
	14, // 113 D[14]    CN6 pin D14
	15, // 114 D[15]    CN6 pin D15
	17, // 115 D[17]    CN6 pin D17

	21, // 116 EN_WIFI_PWR
	22, // 117 WIFI_RESET

	30, // 118 Green LED Register 0x8
	20, // 119 Red LED   Register 0xC

	31, // 120 CPU_ACCESS_FPGA_FLASH# Register 0x8
};

static inline struct ts7800v2_gpio_priv *to_gpio_ts7800v2(struct gpio_chip *chip)
{
	return container_of(chip, struct ts7800v2_gpio_priv, gpio_chip);
}

static int ts7800v2_gpio_get_direction(struct gpio_chip *chip,
	unsigned int offset)
{
	struct ts7800v2_gpio_priv *priv = to_gpio_ts7800v2(chip);
	unsigned int reg, bit, ret;

	bit = BIT(dio_bitpositions[offset]);

	if (offset >= TS7800V2_NR_DIO)
		return -EINVAL;

	/* offset < 26 is special because open drain. */
	if (offset < 26) {
		reg = readl(priv->syscon + LCD_DIO_OUT);
		ret = !(reg & bit);
	} else if (offset < 58) {
		reg = readl(priv->syscon + ROW_A_DIR);
		ret = !!(reg & bit);
	} else if (offset < 83) {
		reg = readl(priv->syscon + ROW_B_DIR);
		ret = !!(reg & bit);
	} else if (offset < 101) {
		reg = readl(priv->syscon + ROW_C_DIR);
		ret = !!(reg & bit);
	} else if (offset < 116) {
		reg = readl(priv->syscon + ROW_D_DIR);
		ret = !!(reg & bit);
	}
	/* offset > 116 are all output-only. */
	else
		ret = 0;

	return ret;
}

static int ts7800v2_gpio_direction_input(struct gpio_chip *chip,
	unsigned int offset)
{
	struct ts7800v2_gpio_priv *priv = to_gpio_ts7800v2(chip);
	unsigned int reg, bit;
	unsigned long flags;
	int ret = 0;

	if (offset >= TS7800V2_NR_DIO)
		return -EINVAL;

	bit = BIT(dio_bitpositions[offset]);

	spin_lock_irqsave(&priv->lock, flags);

	if (offset < 26) {
		/*
		 * The LCD/DIO pins are open-drain with pull-ups, so making
		 * one an 'input' is the same as setting the pin high
		 */
		reg = readl(priv->syscon + LCD_DIO_OUT);
		reg |= bit;
		writel(reg, priv->syscon + LCD_DIO_OUT);
	} else if (offset < 58) {
		reg = readl(priv->syscon + ROW_A_DIR);
		reg &= ~bit;
		writel(reg, priv->syscon + ROW_A_DIR);
	} else if (offset < 83) {
		reg = readl(priv->syscon + ROW_B_DIR);
		reg &= ~bit;
		writel(reg, priv->syscon + ROW_B_DIR);
	} else if (offset < 101) {
		reg = readl(priv->syscon + ROW_C_DIR);
		reg &= ~bit;
		writel(reg, priv->syscon + ROW_C_DIR);
	} else if (offset < 116) {
		reg = readl(priv->syscon + ROW_D_DIR);

		reg &= ~bit;
		writel(reg, priv->syscon + ROW_D_DIR);
	} else {
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static unsigned int ts7800v2_gpio_rmw(unsigned int reg, unsigned int bit,
	int value)
{
	if (value)
		reg |= bit;
	else
		reg &= ~bit;
	return reg;
}

static int ts7800v2_gpio_direction_output(struct gpio_chip *chip,
	unsigned int offset, int value)
{
	struct ts7800v2_gpio_priv *priv = to_gpio_ts7800v2(chip);
	unsigned int reg, bit;
	unsigned long flags;
	int ret = 0;

	bit = BIT(dio_bitpositions[offset]);

	spin_lock_irqsave(&priv->lock, flags);

	if (offset < 26) {
		/* SPI_MISO, read-only pin, can't make an output */
		if (offset == 8) {
			ret = -EINVAL;
		} else {
			reg = readl(priv->syscon + LCD_DIO_OUT);
			reg = ts7800v2_gpio_rmw(reg, bit, value);
			writel(reg, priv->syscon + LCD_DIO_OUT);
		}
	} else if (offset < 58) {  /* pc/104 Row A */
		reg = readl(priv->syscon + ROW_A_DATA);
		reg = ts7800v2_gpio_rmw(reg, bit, value);
		writel(reg, priv->syscon + ROW_A_DATA);
		reg = readl(priv->syscon + ROW_A_DIR);
		reg |= bit;
		writel(reg, priv->syscon + ROW_A_DIR);
	} else if (offset < 83) {  /* pc/104 Row B */
		reg = readl(priv->syscon + ROW_B_DATA);
		reg = ts7800v2_gpio_rmw(reg, bit, value);
		writel(reg, priv->syscon + ROW_B_DATA);
		reg = readl(priv->syscon + ROW_B_DIR);
		reg |= bit;
		writel(reg, priv->syscon + ROW_B_DIR);
	} else if (offset < 101) { /* pc/104 Row C */
		reg = readl(priv->syscon + ROW_C_DATA);
		reg = ts7800v2_gpio_rmw(reg, bit, value);
		writel(reg, priv->syscon + ROW_C_DATA);
		reg = readl(priv->syscon + ROW_C_DIR);
		reg |= bit;
		writel(reg, priv->syscon + ROW_C_DIR);
	} else if (offset < 116) { /* pc/104 Row D */
		if (offset >= 104 && offset <= 107) {
			/* D[4..7], read-only pins */
			ret = -EINVAL;
		} else {
			reg = readl(priv->syscon + ROW_D_DATA);
			reg = ts7800v2_gpio_rmw(reg, bit, value);
			writel(reg, priv->syscon + ROW_D_DATA);
			reg = readl(priv->syscon + ROW_D_DIR);
			reg |= bit;
			writel(reg, priv->syscon + ROW_D_DIR);
		}
	} else if (offset < 118)  { /* WIFI control bits, nothing to do */
		reg = readl(priv->syscon + MISC_IO);
		reg = ts7800v2_gpio_rmw(reg, bit, value);
		writel(reg, priv->syscon + MISC_IO);
	} else if (offset == 118) { /* Green LED */
		reg = readl(priv->syscon + LCD_DIO_OUT);
		reg = ts7800v2_gpio_rmw(reg, bit, value);
		writel(reg, priv->syscon + LCD_DIO_OUT);
	} else if (offset == 119) { /* Red LED */
		reg = readl(priv->syscon + MISC_IO);
		reg = ts7800v2_gpio_rmw(reg, bit, value);
		writel(reg, priv->syscon + MISC_IO);
	} else if (offset == 120) { /* CPU_ACCESS_FPGA_FLASH */
		reg = readl(priv->syscon + LCD_DIO_OUT);
		reg = ts7800v2_gpio_rmw(reg, bit, value);
		writel(reg, priv->syscon + LCD_DIO_OUT);
	} else {
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static int ts7800v2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ts7800v2_gpio_priv *priv = to_gpio_ts7800v2(chip);
	unsigned int reg_num, reg, bit;

	bit = BIT(dio_bitpositions[offset]);

	if (offset < 26)
		reg_num = LCD_DIO_DATA;
	else if (offset < 58)
		reg_num = ROW_A_DATA;
	else if (offset < 83)
		reg_num = ROW_B_DATA;
	else if (offset < 101)
		reg_num = ROW_C_DATA;
	else if (offset < 116)
		reg_num = ROW_D_DATA;
	else if (offset < 118)
		reg_num = MISC_IO;
	else
		return -1;

	reg = readl(priv->syscon + reg_num);
	return !!(reg & bit);
}

static void ts7800v2_gpio_set(struct gpio_chip *chip, unsigned int offset,
	int value)
{
	struct ts7800v2_gpio_priv *priv = to_gpio_ts7800v2(chip);
	unsigned int reg_num, reg, bit;
	unsigned long flags;

	bit = BIT(dio_bitpositions[offset]);

	if (offset < 26) {   /* DIO or LCD header,  */
		if (offset == 8)  { /* SPI_MISO, read-only pin, can't set */
			dev_info(priv->gpio_chip.parent, "error: DIO #%d, read-only pin, can't be set\n",
						priv->gpio_chip.base + offset);
			return;
		}
		reg_num = LCD_DIO_OUT;
	} else if (offset < 58) {  /* pc/104 Row A */
		reg_num = ROW_A_DATA;
	} else if (offset < 83) {  /* pc/104 Row B */
		reg_num = ROW_B_DATA;
	} else if (offset < 101) { /* pc/104 Row C */
		if (offset >= 104 && offset <= 107) {  /* D[4..7], read-only pins */
			return;
		}
		reg_num = ROW_C_DATA;
	} else if (offset < 116) { /* pc/104 Row D */
		reg_num = ROW_D_DATA;
	}  else if (offset < 118)  { /* WIFI control bits */
		reg_num = MISC_IO;
	} else if (offset == 118) { /* Green LED */
		reg_num = LCD_DIO_OUT;
	} else if (offset == 119) { /* Red LED */
		reg_num = MISC_IO;
	} else if (offset == 120) { /* CPU_ACCESS_FPGA_FLASH */
		reg_num = LCD_DIO_OUT;
	} else {
		return;
	}

	spin_lock_irqsave(&priv->lock, flags);
	reg = readl(priv->syscon + reg_num);
	if (value)
		reg |= bit;
	else
		reg &= ~bit;
	writel(reg, priv->syscon + reg_num);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static const struct gpio_chip template_chip = {
	.label			= "ts7800v2-gpio",
	.owner			= THIS_MODULE,
	.get_direction		= ts7800v2_gpio_get_direction,
	.direction_input	= ts7800v2_gpio_direction_input,
	.direction_output	= ts7800v2_gpio_direction_output,
	.get			= ts7800v2_gpio_get,
	.set			= ts7800v2_gpio_set,
	.base			= -1,
	.can_sleep		= false,
};

static const struct of_device_id ts7800v2_gpio_of_match_table[] = {
	{
		.compatible = "technologic,ts7800v2-gpio",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ts7800v2_gpio_of_match_table);

static int ts7800v2_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ts7800v2_gpio_priv *priv;
	void __iomem  *membase;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	membase = devm_ioremap(dev, res->start, resource_size(res));

	if (IS_ERR(membase)) {
		pr_err("Could not map resource\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->syscon = membase;
	spin_lock_init(&priv->lock);
	priv->gpio_chip = template_chip;
	priv->gpio_chip.label = "ts7800v2-gpio";
	priv->gpio_chip.ngpio = TS7800V2_NR_DIO;
	priv->gpio_chip.base = -1;
	pdev->dev.platform_data = &priv;
	priv->gpio_chip.parent = dev;

	platform_set_drvdata(pdev, priv);

	return devm_gpiochip_add_data(dev, &priv->gpio_chip, priv);
}

static struct platform_driver ts7800v2_gpio_driver = {
	.driver = {
		.name = "ts7800v2-gpio",
		.of_match_table = of_match_ptr(ts7800v2_gpio_of_match_table),
	},
	.probe = ts7800v2_gpio_probe,
};
module_platform_driver(ts7800v2_gpio_driver);

MODULE_AUTHOR("Technologic Systems");
MODULE_DESCRIPTION("GPIO interface for Technologic Systems TS-7800-V2 DIO");
MODULE_LICENSE("GPL");
