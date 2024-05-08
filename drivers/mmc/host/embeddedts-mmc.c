// SPDX-License-Identifier: GPL-2.0
/*
 * embeddedTS SD controller driver
 *
 * Copyright (C) 2022 Mark Featherston
 * 
 * This SD controller that bitbangs all access until it gets to a read/write 
 * multiple command, and the controller automates those accesses with the 128-bit
 * FIFO.
 * 
 * 
 * This core has different registers at the same offset depending on the bus access size
 * 
 * 8-bit:
 * 
 * 
 * 
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/pci.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#include <asm/byteorder.h>

#define SDPEEK8(sd, x) readb((uint32_t *)((sd)->sd_regstart + (x)))
#define SDPOKE8(sd, x, y) writeb(y, (uint32_t *)((sd)->sd_regstart + (x)))
#define SDPEEK16(sd, x) readw((uint32_t *)((sd)->sd_regstart + (x)))
#define SDPOKE16(sd, x, y) writew(y, (uint32_t *)((sd)->sd_regstart + (x)))
#define SDPEEK32(sd, x) readl((uint32_t *)((sd)->sd_regstart + (x)))
#define SDPOKE32(sd, x, y) writel(y, (uint32_t *)((sd)->sd_regstart + (x)))


/* Disable probing for eMMC, we only connect this core here to SD */
#define SD_NOMMC
#define SD_NOAUTOMMC

/* Layer includes low level hardware support */
#include "tssdcore2-embedded-ts.c"

struct ts_mmc_priv {
	struct mutex sdlock;
	struct mmc_host *mmc;
	struct sdcore sdcore;
	void __iomem *sdmmc_base;
	struct mmc_request *req;
	struct mmc_command *cmd;
	struct device *dev;
	unsigned long timeout;
	unsigned long lasttimeout;
	int sdio_irq;
	int busy_gpio;
	int debug_gpio;
	int active_lun;
};

static void debug_trigger_low(struct mmc_host *mmc)
{
	struct ts_mmc_priv *priv = mmc_priv(mmc);
	if(gpio_is_valid(priv->debug_gpio))
		gpio_direction_output(priv->debug_gpio, 0);

	/*u32 reg = readl(priv->syscon + 0x08);
	writel(reg & ~(1 << 25), priv->syscon + 0x08);*/
}
static void debug_trigger_high(struct mmc_host *mmc)
{
	struct ts_mmc_priv *priv = mmc_priv(mmc);
	if(gpio_is_valid(priv->debug_gpio))
		gpio_direction_output(priv->debug_gpio, 1);

	/*u32 reg = readl(priv->syscon + 0x08);
	writel(reg | (1 << 25), priv->syscon + 0x08);*/
}

static void ts_mmc_delay(void *data, unsigned int us)
{
	if (us > 50000)
		msleep_interruptible(us/1000);
	else
		udelay(us);
	printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
}

static void ts_mmc_reset_timeout(void *data)
{
	struct ts_mmc_priv *priv = (struct ts_mmc_priv *)data;
	priv->timeout = jiffies + HZ;
}

static void ts_mmc_irqwait(void *data, unsigned int x)
{
	struct ts_mmc_priv *priv = (struct ts_mmc_priv *)data;

	/*while (!gpio_get_value(priv->busy_gpio))  {
#ifdef CONFIG_PREEMPT_NONE
		cond_resched();
#endif
	}*/
	struct sdcore *sd = &priv->sdcore;
	uint32_t reg;

	do {
#ifdef CONFIG_PREEMPT_NONE
		cond_resched();
#endif
		reg = readl((uint32_t *)(sd->sd_regstart + 0x108));
//printk(KERN_INFO "HERE: %s, %s, %d - 0x%X\n", __FILE__, __FUNCTION__, __LINE__, reg);
	} while ((reg & 4) == 0);
	//printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	mdelay(1);
}

static int ts_mmc_timeout(void *data)
{
	struct ts_mmc_priv *priv = (struct ts_mmc_priv *)data;
	int ret;
	//printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);

	priv->lasttimeout = jiffies;
	ret = time_is_before_jiffies(priv->timeout);
	return ret;
}

static void ts_mmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct ts_mmc_priv *priv = mmc_priv(mmc);
	struct mmc_command *cmd = req->cmd;
	struct mmc_data *data = cmd->data;
	struct sdcore *sd = &priv->sdcore;
	uint16_t sd_req;
	int ret;
	u32 resp[17];
	memset(resp, 0, 17 * sizeof(u32));
	unsigned int crc;
	int i;
	int debug_on = 0;

	mutex_lock(&priv->sdlock);

//	printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	/*if (cmd->opcode != 52 &&
		cmd->opcode != 8 &&
		cmd->opcode != 5 &&
		cmd->opcode != 55 && // APP_CMD
		cmd->opcode != 41 &&
		cmd->opcode != 2 &&
		cmd->opcode != 3 &&
		cmd->opcode != 9 &&
		cmd->opcode != 7 &&
		cmd->opcode != 51 && // SEND_SCR
		cmd->opcode != 13 && // SD_STATUS
		cmd->opcode != 6 && // SWITCH_FUNC
		cmd->opcode != 17 && // Single block read
		cmd->opcode != 0){

		debug_on = 1;
	}*/

/*	if(cmd->opcode == 52 && cmd->arg == 0x80022000){
		printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		debug_on = 1;
	}

	if(cmd->opcode == 53)
		debug_on = 1;*/

	/*if(debug_on | (cmd->opcode == 41) | (cmd->opcode == 51) || 1){
		printk(KERN_INFO "%d bit MMC Request OP: %d, arg 0x%X, type 0x%X - hasdata %d\n",
		sd->sd_state & DATSSP_4BIT ? 4 : 1,
		cmd->opcode,
		cmd->arg,
		mmc_resp_type(req->cmd),
		data != NULL);
	}*/

	sd->os_irqwait(sd->os_arg, 3);

	switch (mmc_resp_type(req->cmd)) {
	case MMC_RSP_NONE:
		sd_req = CMD(cmd->opcode, TYPE_NORESP);
		if(debug_on) printk(KERN_INFO "MMC_RSP_NONE");
		break;
	case MMC_RSP_R1B:
		sd_req = CMD(cmd->opcode, TYPE_BSYRESP);
		if(debug_on) printk(KERN_INFO "MMC_RSP_R1B");
		break;
	case MMC_RSP_R2:
		sd_req = CMD(cmd->opcode, TYPE_LONGRESP);
		if(debug_on) printk(KERN_INFO "MMC_RSP_R2");
		break;
	case MMC_RSP_R3:
	case MMC_RSP_R1: /* Also R4, R5, R6, and R7 */
		if(debug_on) printk(KERN_INFO "MMC_RSP_R1/3/4/5/6/7");
		if(data) {
			if(data->flags & MMC_DATA_READ) {
				sd_req = CMD(cmd->opcode, TYPE_RXDAT);
			} else {
				sd_req = CMD(cmd->opcode, TYPE_TXDAT);
			}
		} else {
			sd_req = CMD(cmd->opcode, TYPE_SHORTRESP);
		}
		break;
	default:
		printk(KERN_INFO "Received impossible response type? 0x%X\n", 
			mmc_resp_type(req->cmd));
		mmc_request_done(mmc, req);
		return;
	}

	if(debug_on)
		printk(KERN_INFO "sd->sdstate: 0x%X\n", sd->sd_state);

	if (debug_on) debug_trigger_low(mmc);
	reset_timeout(sd);
	ret = sdcmd(sd, sd_req, cmd->arg, (unsigned int *)&resp, NULL);
	//if (debug_on) debug_trigger_high(mmc);
	if(ret) {
		printk(KERN_INFO "HERE ETIMEDOUT: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		cmd->error = -ETIMEDOUT;
		mutex_unlock(&priv->sdlock);
		mmc_request_done(mmc, req);
		return;
	}

	if(cmd->opcode == 41) { // There is no CMD41, just ACMD41
		unsigned long done = jiffies + jiffies_to_msecs(40);
		/* This loop runs at ~300khz */
		while(!time_after(jiffies, done)) {
			SDPOKE8(sd, SDGPIO, 0xdf);
			SDPEEK8(sd, SDGPIO);       // delay 
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPEEK8(sd, SDGPIO);       // delay
		}

		/*i = 0;
		while ((resp[1] & 0x80) == 0x0) {
			sdcmd2(sd, CMD_APP_CMD, 0, NULL, NULL);
			sdcmd2(sd, ACMD_SD_SEND_OP_COND, cmd->arg, resp, NULL);
			if (timeout(sd)){ 			
				cmd->error = -ETIMEDOUT;
				mmc_request_done(mmc, req);
			}
			printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		}*/
	}

	if(mmc_resp_type(cmd) & MMC_RSP_PRESENT) {
		/* opcode 41 is the only one that does not echo back its opcode */
		if(resp[0] != cmd->opcode && mmc_resp_type(req->cmd) & MMC_RSP_OPCODE) {
			printk(KERN_INFO "sd op is %d, not expected %d?\n", resp[0], cmd->opcode);
			cmd->error = -EILSEQ;
			mutex_unlock(&priv->sdlock);
			mmc_request_done(mmc, req);
			return;
		}

		if (mmc_resp_type(cmd) & MMC_RSP_136) {
			cmd->resp[0] = resp[1] << 24;
			cmd->resp[0] |= resp[2] << 16;
			cmd->resp[0] |= resp[3] << 8;
			cmd->resp[0] |= resp[4];

			cmd->resp[1] = resp[5] << 24;
			cmd->resp[1] |= resp[6] << 16;
			cmd->resp[1] |= resp[7] << 8;
			cmd->resp[1] |= resp[8];

			cmd->resp[2] = resp[9] << 24;
			cmd->resp[2] |= resp[10] << 16;
			cmd->resp[2] |= resp[11] << 8;
			cmd->resp[2] |= resp[12];

			cmd->resp[3] = resp[13] << 24;
			cmd->resp[3] |= resp[14] << 16;
			cmd->resp[3] |= resp[15] << 8;
			cmd->resp[3] |= resp[16];

			if(debug_on) {
				for (i = 0; i < 17; i++){
					printk(KERN_INFO "RAW_RESP[%d]=0x%X\n", i, resp[i]);
				}
				printk(KERN_INFO "RESP[0]: 0x%X\n", cmd->resp[0]);
				printk(KERN_INFO "RESP[1]: 0x%X\n", cmd->resp[1]);
				printk(KERN_INFO "RESP[2]: 0x%X\n", cmd->resp[2]);
				printk(KERN_INFO "RESP[3]: 0x%X\n", cmd->resp[3]);
			}
		} else {
			cmd->resp[0] = resp[1] << 24;
			cmd->resp[0] |= resp[2] << 16;
			cmd->resp[0] |= resp[3] << 8;
			cmd->resp[0] |= resp[4];
			if(debug_on) {
				printk(KERN_INFO "RESP[0]: 0x%X\n", cmd->resp[0]);
			}
			/*printk(KERN_INFO "RAW_RESP[0]: 0x%X\n", resp[0]);
			printk(KERN_INFO "RAW_RESP[1]: 0x%X\n", resp[1]);
			printk(KERN_INFO "RAW_RESP[2]: 0x%X\n", resp[2]);
			printk(KERN_INFO "RAW_RESP[3]: 0x%X\n", resp[3]);
			printk(KERN_INFO "RAW_RESP[4]: 0x%X\n", resp[4]);
			printk(KERN_INFO "RAW_RESP[5]: 0x%X\n", resp[5]);*/

			crc = (0x1 | (crc7(0, resp, 5) << 1));
			if (crc != resp[5] && mmc_resp_type(req->cmd) & MMC_RSP_CRC) {
				printk(KERN_INFO "CRC Failed!\n");
				//DBG(SD_RESP_BAD_CRC, req & 0x3f, crc, resp[5]);
				cmd->error = -EILSEQ;
				mutex_unlock(&priv->sdlock);
				mmc_request_done(mmc, req);
				return;
			}
		}
	}

	if (data) {
		uint8_t crc[2];
		//if (dly) 
		/*} else {*/
			/*SDPOKE8(sd, SDGPIO, 0xff);
			SDPOKE8(sd, SDCMD2, 0xff);*/
		//}


		if(debug_on) {
			printk(KERN_INFO "RESP[0]: 0x%X\n", cmd->resp[0]);
			printk(KERN_INFO "data->sg_len: %d\n", data->sg_len);
			printk(KERN_INFO "data->sg->length: %d\n", data->sg->length);
			printk(KERN_INFO "data->sg->offset: %d\n", data->sg->offset);
		}
		u8 *buffer = sg_virt(data->sg);

		sd->os_irqwait(sd->os_arg, 3);

		if(data->flags & MMC_DATA_READ) {
			unsigned int s;

			if(debug_on) printk(KERN_INFO "Read\n");
			reset_timeout(sd);
			if(sd->sd_state & DATSSP_4BIT){
				do {
					if (timeout(sd)) {
						printk(KERN_INFO "timeout queueing up to start bit\n");
						//DBG(READ_FAIL, sector);
						cmd->error = -EILSEQ;
						sd->sd_state &= ~(SDCMD_RX | SDCMD_TX);	
						mutex_unlock(&priv->sdlock);
						mmc_request_done(mmc, req);
						return;
					}
					SDPOKE8(sd, SDGPIO, 0xdf);
					s = SDPEEK8(sd, SDGPIO);
					SDPOKE8(sd, SDGPIO, 0xff);
				} while ((s & 0xf) != 0x0);
				reset_timeout(sd);
				SDPOKE8(sd, SDGPIO, 0xdf);
				sd->sd_state |= SDDAT_RX;
				SDPOKE8(sd, SDSTAT2, SDPEEK8(sd, SDSTAT2) & ~0x8);
				datssp_stream2(sd, &buffer, data->sg->length);
				data->bytes_xfered = data->sg->length;
				if(debug_on) {
					for (i = 0; i < data->sg->length; i++) {
						printk(KERN_INFO "RX[%d]=0x%X\n", i, buffer[i]);
					}
				}
			} else {

				/* queue up to start bit */
				do {
					if (timeout(sd)) {
						printk(KERN_INFO "timeout queueing up to start bit\n");
						//DBG(READ_FAIL, sector);
						cmd->error = -EILSEQ;
						sd->sd_state &= ~(SDCMD_RX | SDCMD_TX);	
						mutex_unlock(&priv->sdlock);
						mmc_request_done(mmc, req);
						return;
					}
					SDPOKE8(sd, SDGPIO, 0xdf);
					SDPEEK8(sd, SDGPIO); // delay
					s = SDPEEK8(sd, SDGPIO);
					SDPOKE8(sd, SDGPIO, 0xff);
					SDPEEK8(sd, SDGPIO); // delay
				} while ((s & 0x1) != 0x0);
				printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
				SDPOKE8(sd, SDGPIO, 0xdf);
				sd->sd_state |= SDDAT_RX;
				datssp_stream2_1bit(sd, &buffer, data->sg->length);
				data->bytes_xfered = data->sg->length;
				if(debug_on) {
					for (i = 0; i < data->sg->length; i++) {
						printk(KERN_INFO "RX[%d]=0x%X\n", i, buffer[i]);
					}
				}

			}
	//printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	//printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		} else { /* Write */

			sd->sd_state |= SDDAT_TX;
			if(debug_on){
				printk(KERN_INFO "Write %d bytes\n", data->sg->length);
				printk(KERN_INFO "SD STATE 0x%X\n", sd->sd_state);
			}

			reset_timeout(sd);
			uint8_t stat = SDPEEK8(sd, SDSTAT2); // Reset CRC

			if(debug_on) {
				for (i = 0; i < data->sg->length; i++) {
					printk(KERN_INFO "TX[%d]=0x%X\n", i, buffer[i]);
				}
			}

			if(sd->sd_state & DATSSP_4BIT){
				stat = SDPEEK8(sd, SDSTAT2);
				SDPOKE8(sd, SDSTAT2, stat & ~0x10); /* Disable multiwrite */

				datssp_stream2(sd, &buffer, data->sg->length);
				SDPOKE8(sd, SDCTRL2, 0x0); /* busy wait */
				if (sd->os_irqwait)
					sd->os_irqwait(sd->os_arg, 2);
				SDPOKE8(sd, SDGPIO, 0xff);
				data->bytes_xfered = data->sg->length;

				if (stat & 0x10) {
					stat = SDPEEK8(sd, SDSTAT2);
					SDPOKE8(sd, SDSTAT2, stat | 0x10); /* reenab multiwrite */
				} else {
					stat = SDPEEK8(sd, SDSTAT2);
				}

			} else {
				datssp_stream2_1bit(sd, &buffer, data->sg->length);
				data->bytes_xfered = data->sg->length;
			}
		}

		sd->sd_state &= ~(SDCMD_RX | SDCMD_TX | SDDAT_TX | SDDAT_RX);	
	} else {
		for (i = 0; i < 8; i++) {
			SDPOKE8(sd, SDGPIO, 0xdf);
			SDPEEK8(sd, SDGPIO);       // delay
			SDPEEK8(sd, SDGPIO);       // delay 
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPEEK8(sd, SDGPIO);       // delay
			SDPEEK8(sd, SDGPIO);       // delay
		}
	}

/*	if(cmd->opcode == 52) {
		uint8_t data = cmd->resp[0] & 0xff;
		uint32_t addr = (cmd->arg >> 9) & 0x1FFFF;
		uint8_t is_write = cmd->arg >> 31;
		uint8_t raw = cmd->resp[0] >> 27;

		if(is_write) {
			printk(KERN_INFO "CMD52: WR Addr 0x%X=0x%X\n", addr, data);
		} else {
			printk(KERN_INFO "CMD52: RD Addr 0x%X=0x%X (%d)\n", addr, data, raw);
		}
	}*/
	mutex_unlock(&priv->sdlock);



	/*if(debug_on)
		return;*/

//printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	mmc_request_done(mmc, req);

	///* rsptype=7 only valid for SPI commands - should be =2 for SD */
	//if (rsptype == 7)
	//	rsptype = 2;
	///* rsptype=21 is R1B, convert for controller */
	//if (rsptype == 21)
	//	rsptype = 9;

}

static void ts_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ts_mmc_priv *priv = mmc_priv(mmc);
	struct sdcore *sd = &priv->sdcore;

	printk(KERN_INFO "SD Clock speed %d\n", ios->clock);

	mutex_lock(&priv->sdlock);

	sd->sd_state &= ~(SD_LOSPEED | SD_HISPEED | MMC_BUS_WIDTH_4);
	
	if (ios->clock > 400000) {
		//sd->sd_state |= SD_HISPEED;
	} else if (ios->clock != 0) {
		sd->sd_state |= SD_LOSPEED;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4) {
		printk(KERN_INFO "4-bit SD mode\n");
		sd->sd_state |= DATSSP_4BIT;
	} else {
		printk(KERN_INFO "1-bit SD mode\n");
		sd->sd_state &= ~DATSSP_4BIT;
	}

	mutex_unlock(&priv->sdlock);
}

static int ts_mmc_get_ro(struct mmc_host *mmc)
{
	return 0;
}

static int ts_mmc_get_cd(struct mmc_host *mmc)
{
	return 1;
}

static void ts_mmc_enable_sdio_irq(struct mmc_host *host, int enable)
{
	printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
}

static const struct mmc_host_ops ts_mmc_ops = {
	.request = ts_mmc_request,
	.set_ios = ts_mmc_set_ios,
	.get_ro = ts_mmc_get_ro,
	.get_cd = ts_mmc_get_cd,
	.enable_sdio_irq = ts_mmc_enable_sdio_irq,
};

static int ts_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc;
	struct ts_mmc_priv *priv;
	struct sdcore *sd;
	struct device_node *np = pdev->dev.of_node;
	/*const struct of_device_id *of_id =
		of_match_device(ts_mmc_dt_ids, &pdev->dev);*/
	struct resource *base;
	//struct resource *irq;
	int ret;
	struct pci_dev *pcidev;

	if(gpio_direction_output(85, 1) < 0)
		return -EPROBE_DEFER;

	/* The pcie must be enabled before we can access the fpga registers! */
	/*pcidev = pci_get_device(0x1204, 0x0001, NULL);
	if (!pcidev) {
		pr_err("Cannot find FPGA at PCI 1204:0001\n");
		ret = -EINVAL;
		goto fail;
	}

	if (pci_enable_device(pcidev)) {
		pr_err("Cannot enable FPGA at PCI 1204:0001\n");
		ret = -EINVAL;
		goto fail;
	}*/

	mmc = mmc_alloc_host(sizeof(struct ts_mmc_priv), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "Failed to allocate mmc_host\n");
		return -ENOMEM;
	}
	priv = mmc_priv(mmc);
	sd = &priv->sdcore;

	base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!base) {
		dev_err(dev, "missing IOMEM\n");
		ret = -ENOMEM;
		goto fail;
	}

	printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);

	if(!np)
		printk(KERN_INFO "No DTB node?\n");

	priv->debug_gpio = of_get_named_gpio(np, "debug-gpios", 0);
	//priv->debug_gpio = 85;
	//priv->debug_gpio = 460;
	/*if(!gpio_is_valid(priv->debug_gpio)) {
		dev_err(dev, "Couldn't get debug gpio: %d", priv->debug_gpio);
		ret = -ENOMEM;
		goto fail;
	}*/

	if(gpio_is_valid(priv->debug_gpio)) {
		devm_gpio_request(dev, priv->debug_gpio, "debug");
		gpio_direction_output(priv->debug_gpio, 1);
	}

	printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	//priv->busy_gpio = 495;
	//priv->busy_gpio = of_get_named_gpio(np, "busy-gpios", 0);
	/*if(!gpio_is_valid(priv->busy_gpio)) {
		dev_err(dev, "Couldn't get busy gpio");
		ret -ENOMEM;
		goto fail;
	}
	devm_gpio_request(dev, priv->busy_gpio, "busy");
	gpio_direction_input(priv->busy_gpio);*/


	printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	//pm_runtime_enable(dev);

	priv->sdmmc_base = devm_ioremap(dev, base->start, 
					resource_size(base));
	if (!priv->sdmmc_base) {
		dev_err(dev, "failed to remap I/O memory\n");
		goto fail;
	}
	//irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	mutex_init(&priv->sdlock);

	mmc->ops = &ts_mmc_ops;
	mmc->f_min = 0;
	mmc->f_max = 25000000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;

	mmc->max_req_size = 512;
	mmc->max_seg_size = 512;
	mmc->max_segs = 1;
	mmc->max_blk_count = 1;
	mmc->max_blk_size = 512;

	//priv->mmc = mmc;
	//priv->dev = dev;

	/*priv->sdio_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (devm_request_irq(dev, irq->start, ts_mmc_sdio_irq,
			     IRQF_SHARED, name, priv)) {
		dev_err(&pdev->dev, "Register SDIO IRQ fail\n");
		goto fail;
	}*/

	/*ret = request_irq(priv->sdio_irq, ts_mmc_dma_isr, 0, "sdmmc", priv);
	if (ret) {
		dev_err(&pdev->dev, "Register DMA IRQ fail\n");
		goto fail;
	}*/

	platform_set_drvdata(pdev, mmc);

	sd->sd_regstart = (unsigned int)priv->sdmmc_base;
	sd->sd_lun = 0;
	sd->os_arg = priv;
	sd->sd_writeparking = 1;
	sd->debug_arg = priv;
	sd->os_timeout = ts_mmc_timeout;
	sd->os_reset_timeout = ts_mmc_reset_timeout;
	sd->os_delay = ts_mmc_delay;
	sd->os_irqwait = ts_mmc_irqwait;

	//sd->sd_state |= DATSSP_4BIT|SD_HISPEED|SD_HC;
	sd->sd_state = SD_LOSPEED;
	sd->hw_version = 2; // TODO: Remove when we can
	reset_common(sd);
	/* This physically resets the card, allow 100ms for the card to boot */
	//mdelay(1000);
	//sd->debug = tssdcard_debug;

	mmc_add_host(mmc);

	dev_info(&pdev->dev, "TS MMC Controller initialized\n");

	return 0;
fail:
	mmc_free_host(mmc);
	return ret;
}

static int ts_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct ts_mmc_priv *priv;

	mmc = platform_get_drvdata(pdev);
	priv = mmc_priv(mmc);

	/* release the dma buffers */
	mmc_remove_host(mmc);
	mmc_free_host(mmc);

	dev_info(&pdev->dev, "TS MMC device removed\n");

	return 0;
}

static const struct of_device_id ts_mmc_dt_ids[] = {
	{ .compatible = "embeddedts,tssdcore2", NULL },
	{ /* Sentinel */ },
};

static struct platform_driver ts_mmc_driver = {
	.probe = ts_mmc_probe,
	.remove = ts_mmc_remove,
	.driver = {
		.name = "ts_mmc",
		.of_match_table = ts_mmc_dt_ids,
	},
};

module_platform_driver(ts_mmc_driver);

MODULE_DESCRIPTION("Technologic Systems MMC/SD/SDIO Driver");
MODULE_AUTHOR("Mark featherston");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, ts_mmc_dt_ids);
