// SPDX-License-Identifier: GPL-2.0+

/*
 * NXP FlexSPI(FSPI) BUS controller driver.
 *
 * This driver is written based on the spi-nxp-fspi.c, but instead of a SPI
 * peripheral this FlexSPI core can be used as a memory mapped FPGA bus
 * interface.
 *
 * This currently is hard coded to support memory mapped reads and writes to a
 * single chip select:
 * Read:
 *	[7:0] "0x9" (identifies ddr read)
 *	[15:0] address
 *	8 dummy clocks
 *	[(8*n)-1:0] read data
 *
 * Write:
 *	[7:0] "0x8" (identifies write)
 *	[15:0] address
 *	[(8*n)-1:0] write data
 *
 * This driver is planned to attempt to upstream, in which case we should be able
 * to refactor the flexspi driver to share common register definitions. This uses
 * the existing defines from flexspi but adds ~4 definitions.
 */

#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/sys_soc.h>

/* Registers used by the driver */
#define FSPI_MCR0			0x00
#define FSPI_MCR0_AHB_TIMEOUT(x)	((x) << 24)
#define FSPI_MCR0_IP_TIMEOUT(x)		((x) << 16)
#define FSPI_MCR0_LEARN_EN		BIT(15)
#define FSPI_MCR0_SCRFRUN_EN		BIT(14)
#define FSPI_MCR0_OCTCOMB_EN		BIT(13)
#define FSPI_MCR0_DOZE_EN		BIT(12)
#define FSPI_MCR0_HSEN			BIT(11)
#define FSPI_MCR0_SERCLKDIV		BIT(8)
#define FSPI_MCR0_ATDF_EN		BIT(7)
#define FSPI_MCR0_ARDF_EN		BIT(6)
#define FSPI_MCR0_RXCLKSRC(x)		((x) << 4)
#define FSPI_MCR0_END_CFG(x)		((x) << 2)
#define FSPI_MCR0_MDIS			BIT(1)
#define FSPI_MCR0_SWRST			BIT(0)

#define FSPI_MCR1			0x04
#define FSPI_MCR1_SEQ_TIMEOUT(x)	((x) << 16)
#define FSPI_MCR1_AHB_TIMEOUT(x)	(x)

#define FSPI_MCR2			0x08
#define FSPI_MCR2_IDLE_WAIT(x)		((x) << 24)
#define FSPI_MCR2_SAMEDEVICEEN		BIT(15)
#define FSPI_MCR2_CLRLRPHS		BIT(14)
#define FSPI_MCR2_CLRAHBBUFOPT		BIT(11)
#define FSPI_MCR2_ABRDATSZ		BIT(8)
#define FSPI_MCR2_ABRLEARN		BIT(7)
#define FSPI_MCR2_ABR_READ		BIT(6)
#define FSPI_MCR2_ABRWRITE		BIT(5)
#define FSPI_MCR2_ABRDUMMY		BIT(4)
#define FSPI_MCR2_ABR_MODE		BIT(3)
#define FSPI_MCR2_ABRCADDR		BIT(2)
#define FSPI_MCR2_ABRRADDR		BIT(1)
#define FSPI_MCR2_ABR_CMD		BIT(0)

#define FSPI_AHBCR			0x0c
#define FSPI_AHBCR_RDADDROPT		BIT(6)
#define FSPI_AHBCR_PREF_EN		BIT(5)
#define FSPI_AHBCR_BUFF_EN		BIT(4)
#define FSPI_AHBCR_CACH_EN		BIT(3)
#define FSPI_AHBCR_CLRTXBUF		BIT(2)
#define FSPI_AHBCR_CLRRXBUF		BIT(1)
#define FSPI_AHBCR_PAR_EN		BIT(0)

#define FSPI_INTEN			0x10
#define FSPI_INTEN_SCLKSBWR		BIT(9)
#define FSPI_INTEN_SCLKSBRD		BIT(8)
#define FSPI_INTEN_DATALRNFL		BIT(7)
#define FSPI_INTEN_IPTXWE		BIT(6)
#define FSPI_INTEN_IPRXWA		BIT(5)
#define FSPI_INTEN_AHBCMDERR		BIT(4)
#define FSPI_INTEN_IPCMDERR		BIT(3)
#define FSPI_INTEN_AHBCMDGE		BIT(2)
#define FSPI_INTEN_IPCMDGE		BIT(1)
#define FSPI_INTEN_IPCMDDONE		BIT(0)

#define FSPI_INTR			0x14
#define FSPI_INTR_SCLKSBWR		BIT(9)
#define FSPI_INTR_SCLKSBRD		BIT(8)
#define FSPI_INTR_DATALRNFL		BIT(7)
#define FSPI_INTR_IPTXWE		BIT(6)
#define FSPI_INTR_IPRXWA		BIT(5)
#define FSPI_INTR_AHBCMDERR		BIT(4)
#define FSPI_INTR_IPCMDERR		BIT(3)
#define FSPI_INTR_AHBCMDGE		BIT(2)
#define FSPI_INTR_IPCMDGE		BIT(1)
#define FSPI_INTR_IPCMDDONE		BIT(0)

#define FSPI_LUTKEY			0x18
#define FSPI_LUTKEY_VALUE		0x5AF05AF0

#define FSPI_LCKCR			0x1C

#define FSPI_LCKER_LOCK			0x1
#define FSPI_LCKER_UNLOCK		0x2

#define FSPI_BUFXCR_INVALID_MSTRID	0xE
#define FSPI_AHBRX_BUF0CR0		0x20
#define FSPI_AHBRX_BUF1CR0		0x24
#define FSPI_AHBRX_BUF2CR0		0x28
#define FSPI_AHBRX_BUF3CR0		0x2C
#define FSPI_AHBRX_BUF4CR0		0x30
#define FSPI_AHBRX_BUF5CR0		0x34
#define FSPI_AHBRX_BUF6CR0		0x38
#define FSPI_AHBRX_BUF7CR0		0x3C
#define FSPI_AHBRXBUF0CR7_MSTRID(x)	(((u32)(x) << 16))
#define FSPI_AHBRXBUF0CR7_BUFSZ(x)	(((u32)(x) << 0))
#define FSPI_AHBRXBUF0CR7_PREF		BIT(31)

#define FSPI_AHBRX_BUF0CR1		0x40
#define FSPI_AHBRX_BUF1CR1		0x44
#define FSPI_AHBRX_BUF2CR1		0x48
#define FSPI_AHBRX_BUF3CR1		0x4C
#define FSPI_AHBRX_BUF4CR1		0x50
#define FSPI_AHBRX_BUF5CR1		0x54
#define FSPI_AHBRX_BUF6CR1		0x58
#define FSPI_AHBRX_BUF7CR1		0x5C

#define FSPI_FLSHA1CR0			0x60
#define FSPI_FLSHA2CR0			0x64
#define FSPI_FLSHB1CR0			0x68
#define FSPI_FLSHB2CR0			0x6C
#define FSPI_FLSHXCR0_SZ_KB		10
#define FSPI_FLSHXCR0_SZ(x)		((x) >> FSPI_FLSHXCR0_SZ_KB)

#define FSPI_FLSHA1CR1			0x70
#define FSPI_FLSHA2CR1			0x74
#define FSPI_FLSHB1CR1			0x78
#define FSPI_FLSHB2CR1			0x7C
#define FSPI_FLSHXCR1_CSINTR(x)		((x) << 16)
#define FSPI_FLSHXCR1_CAS(x)		((x) << 11)
#define FSPI_FLSHXCR1_WA		BIT(10)
#define FSPI_FLSHXCR1_TCSH(x)		((x) << 5)
#define FSPI_FLSHXCR1_TCSS(x)		(x)

#define FSPI_FLSHA1CR2			0x80
#define FSPI_FLSHA2CR2			0x84
#define FSPI_FLSHB1CR2			0x88
#define FSPI_FLSHB2CR2			0x8C
#define FSPI_FLSHXCR2_CLRINSP		BIT(24)
#define FSPI_FLSHXCR2_AWRWAIT		BIT(16)
#define FSPI_FLSHXCR2_AWRSEQN_SHIFT	13
#define FSPI_FLSHXCR2_AWRSEQI_SHIFT	8
#define FSPI_FLSHXCR2_ARDSEQN_SHIFT	5
#define FSPI_FLSHXCR2_ARDSEQI_SHIFT	0

#define FSPI_FLSHACR4			0x94

#define FSPI_IPCR0			0xA0

#define FSPI_IPCR1			0xA4
#define FSPI_IPCR1_IPAREN		BIT(31)
#define FSPI_IPCR1_SEQNUM_SHIFT		24
#define FSPI_IPCR1_SEQID_SHIFT		16
#define FSPI_IPCR1_IDATSZ(x)		(x)

#define FSPI_IPCMD			0xB0
#define FSPI_IPCMD_TRG			BIT(0)

#define FSPI_DLPR			0xB4

#define FSPI_IPRXFCR			0xB8
#define FSPI_IPRXFCR_CLR		BIT(0)
#define FSPI_IPRXFCR_DMA_EN		BIT(1)
#define FSPI_IPRXFCR_WMRK(x)		((x) << 2)

#define FSPI_IPTXFCR			0xBC
#define FSPI_IPTXFCR_CLR		BIT(0)
#define FSPI_IPTXFCR_DMA_EN		BIT(1)
#define FSPI_IPTXFCR_WMRK(x)		((x) << 2)

#define FSPI_DLLACR			0xC0
#define FSPI_DLLACR_OVRDEN		BIT(8)
#define FSPI_DLLACR_SLVDLY(x)		((x) << 3)
#define FSPI_DLLACR_DLLRESET		BIT(1)
#define FSPI_DLLACR_DLLEN		BIT(0)

#define FSPI_DLLBCR			0xC4
#define FSPI_DLLBCR_OVRDEN		BIT(8)
#define FSPI_DLLBCR_SLVDLY(x)		((x) << 3)
#define FSPI_DLLBCR_DLLRESET		BIT(1)
#define FSPI_DLLBCR_DLLEN		BIT(0)

#define FSPI_STS0			0xE0
#define FSPI_STS0_DLPHB(x)		((x) << 8)
#define FSPI_STS0_DLPHA(x)		((x) << 4)
#define FSPI_STS0_CMD_SRC(x)		((x) << 2)
#define FSPI_STS0_ARB_IDLE		BIT(1)
#define FSPI_STS0_SEQ_IDLE		BIT(0)

#define FSPI_STS1			0xE4
#define FSPI_STS1_IP_ERRCD(x)		((x) << 24)
#define FSPI_STS1_IP_ERRID(x)		((x) << 16)
#define FSPI_STS1_AHB_ERRCD(x)		((x) << 8)
#define FSPI_STS1_AHB_ERRID(x)		(x)

#define FSPI_STS2			0xE8
#define FSPI_STS2_BREFLOCK		BIT(17)
#define FSPI_STS2_BSLVLOCK		BIT(16)
#define FSPI_STS2_AREFLOCK		BIT(1)
#define FSPI_STS2_ASLVLOCK		BIT(0)
#define FSPI_STS2_AB_LOCK		(FSPI_STS2_BREFLOCK | \
					 FSPI_STS2_BSLVLOCK | \
					 FSPI_STS2_AREFLOCK | \
					 FSPI_STS2_ASLVLOCK)

#define FSPI_AHBSPNST			0xEC
#define FSPI_AHBSPNST_DATLFT(x)		((x) << 16)
#define FSPI_AHBSPNST_BUFID(x)		((x) << 1)
#define FSPI_AHBSPNST_ACTIVE		BIT(0)

#define FSPI_IPRXFSTS			0xF0
#define FSPI_IPRXFSTS_RDCNTR(x)		((x) << 16)
#define FSPI_IPRXFSTS_FILL(x)		(x)

#define FSPI_IPTXFSTS			0xF4
#define FSPI_IPTXFSTS_WRCNTR(x)		((x) << 16)
#define FSPI_IPTXFSTS_FILL(x)		(x)

#define FSPI_RFDR			0x100
#define FSPI_TFDR			0x180

#define FSPI_LUT_BASE			0x200

/* register map end */

/* Instruction set for the LUT register. */
#define LUT_STOP			0x00
#define LUT_CMD				0x01
#define LUT_ADDR			0x02
#define LUT_CADDR_SDR			0x03
#define LUT_MODE			0x04
#define LUT_MODE2			0x05
#define LUT_MODE4			0x06
#define LUT_MODE8			0x07
#define LUT_NXP_WRITE			0x08
#define LUT_NXP_READ			0x09
#define LUT_LEARN_SDR			0x0A
#define LUT_DATSZ_SDR			0x0B
#define LUT_DUMMY			0x0C
#define LUT_DUMMY_RWDS_SDR		0x0D
#define LUT_JMP_ON_CS			0x1F
#define LUT_CMD_DDR			0x21
#define LUT_ADDR_DDR			0x22
#define LUT_CADDR_DDR			0x23
#define LUT_MODE_DDR			0x24
#define LUT_MODE2_DDR			0x25
#define LUT_MODE4_DDR			0x26
#define LUT_MODE8_DDR			0x27
#define LUT_WRITE_DDR			0x28
#define LUT_READ_DDR			0x29
#define LUT_LEARN_DDR			0x2A
#define LUT_DATSZ_DDR			0x2B
#define LUT_DUMMY_DDR			0x2C
#define LUT_DUMMY_RWDS_DDR		0x2D

/*
 * Calculate number of required PAD bits for LUT register.
 *
 * The pad stands for the number of IO lines [0:7].
 * For example, the octal read needs eight IO lines,
 * so you should use LUT_PAD(8). This macro
 * returns 3 i.e. use eight (2^3) IP lines for read.
 */
#define LUT_PAD(x) (fls(x) - 1)

/*
 * Macro for constructing the LUT entries with the following
 * register layout:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define PAD_SHIFT		8
#define INSTR_SHIFT		10
#define OPRND_SHIFT		16

/* Macros for constructing the LUT register. */
#define LUT_DEF(idx, ins, pad, opr)			  \
	((((ins) << INSTR_SHIFT) | ((pad) << PAD_SHIFT) | \
	(opr)) << (((idx) % 2) * OPRND_SHIFT))

#define POLL_TOUT		5000
#define NXP_FSPI_MAX_CHIPSELECT		4
#define NXP_FSPI_MIN_IOMAP	SZ_4M

#define DCFG_RCWSR1		0x100
#define SYS_PLL_RAT		GENMASK(6, 2)

struct nxp_fspi {
	void __iomem *iobase;
	struct clk *clk, *clk_en;
	struct device *dev;
	int clk_rate;
};

static void fspi_writel(struct nxp_fspi *f, u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static u32 fspi_readl(struct nxp_fspi *f, void __iomem *addr)
{
	return ioread32(addr);
}

/* Instead of busy looping invoke readl_poll_timeout functionality. */
static int fspi_readl_poll_tout(struct nxp_fspi *f, void __iomem *base,
				u32 mask, u32 delay_us,
				u32 timeout_us, bool c)
{
	u32 reg;

	if (c)
		return readl_poll_timeout(base, reg, (reg & mask),
					  delay_us, timeout_us);
	else
		return readl_poll_timeout(base, reg, !(reg & mask),
					  delay_us, timeout_us);
}

static int nxp_fspi_clk_prep_enable(struct nxp_fspi *f)
{
	int ret;

	ret = clk_prepare_enable(f->clk_en);
	if (ret)
		return ret;

	ret = clk_prepare_enable(f->clk);
	if (ret) {
		clk_disable_unprepare(f->clk_en);
		return ret;
	}

	return 0;
}

static void nxp_fspi_clk_disable_unprep(struct nxp_fspi *f)
{
	clk_disable_unprepare(f->clk);
	clk_disable_unprepare(f->clk_en);
}

static int nxp_fspi_ahb_bus_setup(struct nxp_fspi *f)
{
	const int cs = 0;
	const int pads = 4;
	void __iomem *base = f->iobase;
	uint64_t size_kb;
	u32 seq0[4] = {0, 0, 0, 0};
	u32 seq1[4] = {0, 0, 0, 0};
	u32 reg, mcr0;
	int ret;
	int i;

	ret = clk_set_rate(f->clk, f->clk_rate);
	if (ret)
		return ret;

	ret = nxp_fspi_clk_prep_enable(f);
	if (ret)
		return ret;

	/* Reset the module */
	/* w1c register, wait unit clear */
	ret = fspi_readl_poll_tout(f, f->iobase + FSPI_MCR0,
				   FSPI_MCR0_SWRST, 0, POLL_TOUT, false);
	WARN_ON(ret);

	/* Disable the module */
	fspi_writel(f, FSPI_MCR0_MDIS, base + FSPI_MCR0);

	/*
	 * Config the DLL register to default value, enable the target clock delay
	 * line delay cell override mode, and use 1 fixed delay cell in DLL delay
	 * chain, this is the suggested setting when clock rate < 100MHz.
	 */
	fspi_writel(f, FSPI_DLLACR_OVRDEN, base + FSPI_DLLACR);
	fspi_writel(f, FSPI_DLLBCR_OVRDEN, base + FSPI_DLLBCR);

	/*
	 * Disable same device enable bit and configure all target devices
	 * independently.
	 */
	reg = fspi_readl(f, f->iobase + FSPI_MCR2);
	reg = reg & ~(FSPI_MCR2_SAMEDEVICEEN);
	fspi_writel(f, reg, base + FSPI_MCR2);

	/* AHB configuration for access buffer 0~7. */
	for (i = 0; i < 7; i++)
		fspi_writel(f, 0, base + FSPI_AHBRX_BUF0CR0 + 4 * i);

	/*
	 * Set ADATSZ with the maximum AHB buffer size to improve the read
	 * performance.
	 */
	fspi_writel(f, (SZ_2K / 8 | FSPI_AHBRXBUF0CR7_PREF),
		    base + FSPI_AHBRX_BUF7CR0);

	/* prefetch and no start address alignment limitation */
	fspi_writel(f, FSPI_AHBCR_PREF_EN | FSPI_AHBCR_RDADDROPT,
		 base + FSPI_AHBCR);

	/* Reset the FLSHxCR1 registers. */
	reg = FSPI_FLSHXCR1_TCSH(0x3) | FSPI_FLSHXCR1_TCSS(0x3);
	fspi_writel(f, reg, base + FSPI_FLSHA1CR1);
	fspi_writel(f, reg, base + FSPI_FLSHA2CR1);
	fspi_writel(f, reg, base + FSPI_FLSHB1CR1);
	fspi_writel(f, reg, base + FSPI_FLSHB2CR1);

	/*
	 * The driver only uses one single LUT entry, that is updated on
	 * each call of exec_op(). Index 0 is preset at boot with a basic
	 * read operation, so let's use the last entry.
	 */
	/* AHB Read - Set lut sequence ID for all CS. */
	fspi_writel(f, 31, base + FSPI_FLSHA1CR2);
	fspi_writel(f, 31, base + FSPI_FLSHA2CR2);
	fspi_writel(f, 31, base + FSPI_FLSHB1CR2);
	fspi_writel(f, 31, base + FSPI_FLSHB2CR2);

	/* Setup MCR0 config */
	mcr0 = (u32)(FSPI_MCR0_RXCLKSRC(0x0) |
		     FSPI_MCR0_DOZE_EN |
		     FSPI_MCR0_SCRFRUN_EN |
		     FSPI_MCR0_IP_TIMEOUT(0xFF) |
		     FSPI_MCR0_AHB_TIMEOUT(0xFF));

	fspi_writel(f, mcr0, base + FSPI_MCR0);

	/* Setup MCR1 config */
	reg = FSPI_MCR1_SEQ_TIMEOUT(0xFFFF) |
	      FSPI_MCR1_AHB_TIMEOUT(0xFFFF);
	fspi_writel(f, reg, base + FSPI_MCR1);

	/* Setup MCR2 config */
	reg = FSPI_MCR2_IDLE_WAIT(20) |
	      FSPI_MCR2_CLRAHBBUFOPT;
	fspi_writel(f, reg, base + FSPI_MCR2);

	/* Setup AHB config */
	reg = FSPI_AHBCR_BUFF_EN;
	fspi_writel(f, reg, base + FSPI_AHBCR);

	/* Set up ahb buffers */
	reg = FSPI_AHBRXBUF0CR7_MSTRID(FSPI_BUFXCR_INVALID_MSTRID);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF0CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF1CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF2CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF3CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF4CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF5CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF6CR0);

	reg = FSPI_AHBRXBUF0CR7_BUFSZ(1);// in units of 64-bits // Max burst size is 32-bytes
	fspi_writel(f, 0, base + FSPI_AHBRX_BUF7CR0);

	/* Reset FLSHxxCR0 registers */
	fspi_writel(f, 0, f->iobase + FSPI_FLSHA1CR0);
	fspi_writel(f, 0, f->iobase + FSPI_FLSHA2CR0);
	fspi_writel(f, 0, f->iobase + FSPI_FLSHB1CR0);
	fspi_writel(f, 0, f->iobase + FSPI_FLSHB2CR0);

	/* Assign controller memory mapped space as size, KBytes, of flash. */
	size_kb = FSPI_FLSHXCR0_SZ(0x10000000);

	fspi_writel(f, size_kb, f->iobase + FSPI_FLSHA1CR0 + 4 * cs);

	/* Set bus parameters */
	reg = FSPI_FLSHXCR1_CSINTR(2) | /* 2 is minimum */
	      FSPI_FLSHXCR1_TCSH(0) |
	      FSPI_FLSHXCR1_TCSS(0);
	fspi_writel(f, reg, base + FSPI_FLSHA1CR1);
	fspi_writel(f, reg, base + FSPI_FLSHA2CR1);
	fspi_writel(f, reg, base + FSPI_FLSHB1CR1);
	fspi_writel(f, reg, base + FSPI_FLSHB2CR1);

	reg = fspi_readl(f, base + FSPI_FLSHA1CR2) & 0xffff0000;
	reg |= (0 << FSPI_FLSHXCR2_AWRSEQN_SHIFT) |
	      (1 << FSPI_FLSHXCR2_AWRSEQI_SHIFT) |
	      (0 << FSPI_FLSHXCR2_ARDSEQI_SHIFT) |
	      (0 << FSPI_FLSHXCR2_ARDSEQN_SHIFT);
	fspi_writel(f, reg, base + FSPI_FLSHA1CR2);

	reg = 0x1;
	fspi_writel(f, reg, base + FSPI_FLSHACR4);

	/* Magic number, finish testing and see what we need here*/
	fspi_writel(f, FSPI_DLLACR_DLLEN | FSPI_DLLACR_SLVDLY(0x37),
		    f->iobase + FSPI_DLLACR);

	/*
	 * LUT Sequence 0 is an SDR Read.
	 */
	seq0[0] = LUT_DEF(0, LUT_CMD, LUT_PAD(pads), LUT_NXP_READ) |
		  LUT_DEF(1, LUT_ADDR, LUT_PAD(pads), 16);
	seq0[1] = LUT_DEF(2, LUT_DUMMY, LUT_PAD(pads), 8) |
		  LUT_DEF(3, LUT_NXP_READ, LUT_PAD(pads), 0);
	seq0[2] = LUT_DEF(4, LUT_STOP, 0, 0);

	/*
	 * LUT Sequence 1 is a SDR Write
	 */
	seq1[0] = LUT_DEF(0, LUT_CMD, LUT_PAD(pads), LUT_NXP_WRITE) |
		  LUT_DEF(1, LUT_ADDR, LUT_PAD(pads), 16);
	seq1[1] = LUT_DEF(2, LUT_NXP_WRITE, LUT_PAD(pads), 0) |
		  LUT_DEF(3, LUT_STOP, 0, 0);

	/* unlock LUT */
	fspi_writel(f, FSPI_LUTKEY_VALUE, f->iobase + FSPI_LUTKEY);
	fspi_writel(f, FSPI_LCKER_UNLOCK, f->iobase + FSPI_LCKCR);

	/* configure read lut */
	for (i = 0; i < ARRAY_SIZE(seq0); i++)
		fspi_writel(f, seq0[i], base + FSPI_LUT_BASE + (i*4));

	/* configure write lut */
	for (i = 0; i < ARRAY_SIZE(seq1); i++)
		fspi_writel(f, seq1[i], base + FSPI_LUT_BASE + 0x10 + (i*4));

	/* lock LUT */
	fspi_writel(f, FSPI_LUTKEY_VALUE, f->iobase + FSPI_LUTKEY);
	fspi_writel(f, FSPI_LCKER_LOCK, f->iobase + FSPI_LCKCR);

	/* Deassert disable */
	fspi_writel(f, mcr0, base + FSPI_MCR0);

	/* Wait for controller being ready. */
	ret = fspi_readl_poll_tout(f, f->iobase + FSPI_STS0,
				   FSPI_STS0_ARB_IDLE, 1, POLL_TOUT, true);
	if (ret)
		return ret;

	return 0;
}

static void nxp_fspi_cleanup(void *data)
{
	struct nxp_fspi *f = data;
	int ret;

	/* enable clock first since there is reigster access */
	ret = pm_runtime_get_sync(f->dev);
	if (ret < 0)
		dev_err(f->dev, "Failed to enable clock %d\n", __LINE__);

	/* disable the hardware */
	fspi_writel(f, FSPI_MCR0_MDIS, f->iobase + FSPI_MCR0);

	nxp_fspi_clk_disable_unprep(f);
}

static int nxp_fspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct nxp_fspi *f;
	int ret;

	f = devm_kzalloc(dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;

	f->dev = dev;
	platform_set_drvdata(pdev, f);

	f->iobase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(f->iobase)) {
		ret = PTR_ERR(f->iobase);
		return ret;
	}

	/* find the clocks */
	if (dev_of_node(&pdev->dev)) {
		f->clk_en = devm_clk_get(dev, "fspi_en");
		if (IS_ERR(f->clk_en)) {
			ret = PTR_ERR(f->clk_en);
			return ret;
		}

		f->clk = devm_clk_get(dev, "fspi");
		if (IS_ERR(f->clk)) {
			ret = PTR_ERR(f->clk);
			return ret;
		}
	}

	ret = of_property_read_u32(np, "clock-frequency", &f->clk_rate);
	if (ret)
		return ret;
	pm_runtime_enable(dev);

	/* enable clock */
	ret = pm_runtime_get_sync(f->dev);
	if (ret < 0) {
		dev_err(f->dev, "Failed to enable clock %d\n", __LINE__);
		return ret;
	}

	ret = nxp_fspi_ahb_bus_setup(f);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, nxp_fspi_cleanup, f);
	if (ret)
		return ret;


	return of_platform_default_populate(np, NULL, NULL);
}

static const struct of_device_id nxp_fspi_dt_ids[] = {
	{ .compatible = "nxp,imx93-fspi-bus", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nxp_fspi_dt_ids);

static struct platform_driver nxp_fspi_driver = {
	.driver = {
		.name	= "nxp-fspi-bus",
		.of_match_table = nxp_fspi_dt_ids,
	},
	.probe          = nxp_fspi_probe,
};
module_platform_driver(nxp_fspi_driver);

MODULE_DESCRIPTION("NXP FSPI BUS Driver");
MODULE_AUTHOR("embeddedTS");
MODULE_LICENSE("GPL");
