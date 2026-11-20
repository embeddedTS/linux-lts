// SPDX-License-Identifier: GPL-2.0
/* Host driver for the TS-7800-V2 FPGA SD core. */
#include <linux/crc-itu-t.h>
#include <linux/crc7.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#define R_GPIO 0x00
#define R_STATUS 0x01
#define R_CLKSEL 0x02
#define R_DATA 0x04
#define R_WRITE_BUSY 0x0c
#define R_FPGA_IRQ_STATUS 0x108

#define GPIO_DAT GENMASK(3, 0)
#define GPIO_CMD BIT(4)
#define GPIO_CLK BIT(5)
#define GPIO_OE_DAT0 BIT(6)
#define GPIO_OE_CMD BIT(7)
#define GPIO_OE_INPUT GENMASK(7, 6)
#define GPIO_IDLE (GPIO_OE_INPUT | GPIO_CMD | GPIO_DAT)
#define GPIO_DAT_INPUT (GPIO_OE_CMD | GPIO_CMD | GPIO_DAT)
#define GPIO_WRITE_START GPIO_CMD

#define TSSD_BURST_FREQ 25000000
#define TSSD_GPIO_MAX_FREQ 400000
#define TSSD_IDLE_TIMEOUT_US USEC_PER_SEC
#define TSSD_BLOCK_SIZE 512
#define TSSD_CMD_RESPONSE_CLOCKS 64
#define TSSD_INITIAL_CLOCKS 80
#define S_CRCERR BIT(2)
#define S_TIMEOUT BIT(6)
#define S_MULTIRD BIT(3)
#define S_MULTIWR BIT(4)

struct tssd_host {
	void __iomem *base;
	struct mmc_host *mmc;
	struct mutex lock;
	bool bus_width_4;
	bool burst;
	u8 power_mode;
	unsigned int clock_delay_ns;
};

struct tssd_read_prefix {
	u8 samples[48];
	unsigned int count;
	unsigned int next;
	bool started;
};

static inline u8 rd(struct tssd_host *h, unsigned int r)
{
	return readb(h->base + r);
}

static inline void wr(struct tssd_host *h, unsigned int r, u8 v)
{
	writeb(v, h->base + r);
	/* GPIO clock transitions are software paced; stream mode runs at 25 MHz. */
	if (r == R_GPIO && h->clock_delay_ns)
		ndelay(h->clock_delay_ns);
}

static int wait_idle(struct tssd_host *h, u64 timeout_us)
{
	u32 status;

	/* The FPGA routes this window to the IRQ controller's SD idle bit. */
	return readl_poll_timeout(h->base + R_FPGA_IRQ_STATUS, status, status & BIT(2),
			  1, timeout_us);
}

static void gpio_delay(struct tssd_host *h)
{
	rd(h, R_GPIO);
	rd(h, R_GPIO);
}

/* Drive the given pin values for one rising clock edge. */
static void clock_out(struct tssd_host *h, u8 value)
{
	wr(h, R_GPIO, value);
	gpio_delay(h);
	wr(h, R_GPIO, value | GPIO_CLK);
	gpio_delay(h);
}

static void clock_idle(struct tssd_host *h, unsigned int clocks)
{
	while (clocks--)
		clock_out(h, GPIO_IDLE);
}

static u8 clock_sample_data(struct tssd_host *h)
{
	wr(h, R_GPIO, GPIO_IDLE);
	rd(h, R_GPIO);
	wr(h, R_GPIO, GPIO_IDLE | GPIO_CLK);
	rd(h, R_GPIO);
	return rd(h, R_GPIO) & GPIO_DAT;
}

static u8 read_data_sample(struct tssd_host *h, struct tssd_read_prefix *prefix)
{
	if (prefix->next < prefix->count)
		return prefix->samples[prefix->next++];

	return clock_sample_data(h);
}

static u64 data_timeout_ns(struct tssd_host *h, const struct mmc_data *data)
{
	unsigned int clock = min(h->mmc->ios.clock, TSSD_GPIO_MAX_FREQ);
	u64 timeout = data->timeout_ns;

	/* Data framing and software busy polling use the GPIO clock. */
	if (data->timeout_clks)
		timeout += DIV_ROUND_UP_ULL((u64)data->timeout_clks * NSEC_PER_SEC,
					    clock ?: h->mmc->f_min);

	if (timeout)
		return timeout;

	return data->flags & MMC_DATA_WRITE ? NSEC_PER_SEC : 100 * NSEC_PER_MSEC;
}

static int wait_data_start(struct tssd_host *h, u8 mask, u64 timeout_ns)
{
	ktime_t deadline = ktime_add_ns(ktime_get(), timeout_ns);

	do {
		if (!(clock_sample_data(h) & mask))
			return 0;
	} while (ktime_before(ktime_get(), deadline));

	return -ETIMEDOUT;
}

static u16 data_crc16_bit(u16 crc, u8 bit)
{
	bool feedback = ((crc >> 15) ^ bit) & 1;

	crc <<= 1;
	if (feedback)
		crc ^= 0x1021;
	return crc;
}

static int read_1bit_data(struct tssd_host *h, u8 *buf, unsigned int blksz,
			  unsigned int blocks, bool stop_pending,
			  struct tssd_read_prefix *prefix, u64 timeout_ns)
{
	unsigned int b, i, bit;
	u16 crc, card_crc;
	u8 value, end;

	for (b = 0; b < blocks; b++) {
		if ((b || !prefix->started) && wait_data_start(h, BIT(0), timeout_ns))
			return -ETIMEDOUT;

		for (i = 0; i < blksz; i++) {
			value = 0;
			for (bit = 0; bit < 8; bit++)
				value = (value << 1) | (read_data_sample(h, prefix) & BIT(0));
			buf[b * blksz + i] = value;
		}
		crc = crc_itu_t(0, buf + b * blksz, blksz);
		card_crc = 0;
		for (bit = 0; bit < 16; bit++) {
			card_crc = (card_crc << 1) | (read_data_sample(h, prefix) & BIT(0));
		}
		end = (read_data_sample(h, prefix) & BIT(0));
		if (!stop_pending)
			clock_idle(h, 8);
		if (end != BIT(0) || crc != card_crc) {
			dev_err_ratelimited(mmc_dev(h->mmc),
					    "1-bit read block %u end %u CRC %04x/%04x\n",
					    b, !!end, crc, card_crc);
			return -EILSEQ;
		}
	}
	return 0;
}

static void data_crc16_4bit(const u8 *buf, unsigned int len, u16 crc[4])
{
	unsigned int i, lane;

	memset(crc, 0, sizeof(u16) * 4);
	for (i = 0; i < len; i++) {
		for (lane = 0; lane < 4; lane++) {
			crc[lane] = data_crc16_bit(crc[lane],
						  (buf[i] >> (lane + 4)) & 1);
			crc[lane] = data_crc16_bit(crc[lane], (buf[i] >> lane) & 1);
		}
	}
}

static void write_gpio_block(struct tssd_host *h, const u8 *buf,
			     unsigned int blksz)
{
	unsigned int i, bit, lane;
	u16 crc[4];
	u8 sample;

	if (h->bus_width_4) {
		data_crc16_4bit(buf, blksz, crc);
		clock_out(h, GPIO_WRITE_START);
		for (i = 0; i < blksz; i++) {
			clock_out(h, GPIO_CMD | (buf[i] >> 4));
			clock_out(h, GPIO_CMD | (buf[i] & GPIO_DAT));
		}
		for (bit = 0; bit < 16; bit++) {
			sample = GPIO_CMD;
			for (lane = 0; lane < 4; lane++)
				sample |= ((crc[lane] >> (15 - bit)) & 1) << lane;
			clock_out(h, sample);
		}
		clock_out(h, GPIO_CMD | GPIO_DAT);
	} else {
		/* In one-bit mode, leave DAT1-DAT3 as inputs. */
		u8 pins = GPIO_OE_DAT0 | GPIO_CMD | (GPIO_DAT & ~BIT(0));

		crc[0] = crc_itu_t(0, buf, blksz);
		clock_out(h, pins);
		for (i = 0; i < blksz; i++)
			for (bit = 0; bit < 8; bit++)
				clock_out(h, pins | ((buf[i] >> (7 - bit)) & 1));
		for (bit = 0; bit < 16; bit++)
			clock_out(h, pins | ((crc[0] >> (15 - bit)) & 1));
		clock_out(h, pins | BIT(0));
	}
}

static int write_gpio_data(struct tssd_host *h, const u8 *buf,
			   unsigned int blksz, unsigned int blocks,
			   bool stop_pending, u64 timeout_ns)
{
	unsigned int block, bit;
	u8 response, value;
	ktime_t deadline;

	/* Allow the card's two nWR clocks after the command response. */
	clock_idle(h, 2);
	for (block = 0; block < blocks; block++, buf += blksz) {
		write_gpio_block(h, buf, blksz);

		/* Native SD returns an 8-clock CRC status; 010 means accepted. */
		response = 0;
		for (bit = 0; bit < 8; bit++) {
			wr(h, R_GPIO, GPIO_DAT_INPUT);
			gpio_delay(h);
			response = (response << 1) |
				   !!(rd(h, R_GPIO) & BIT(0));
			wr(h, R_GPIO, GPIO_DAT_INPUT | GPIO_CLK);
			gpio_delay(h);
		}
		if ((response & 0x07) != 0x02) {
			dev_err_ratelimited(mmc_dev(h->mmc),
					    "GPIO write CRC status %02x, expected ..02\n",
					    response);
			return -EILSEQ;
		}
		/* The card holds DAT0 low while programming the block. */
		deadline = ktime_add_ns(ktime_get(), timeout_ns);
		response = 0;
		while (response < 3 && ktime_before(ktime_get(), deadline)) {
			wr(h, R_GPIO, GPIO_DAT_INPUT);
			gpio_delay(h);
			value = rd(h, R_GPIO);
			wr(h, R_GPIO, GPIO_DAT_INPUT | GPIO_CLK);
			gpio_delay(h);
			if (value & BIT(0))
				response++;
			else {
				response = 0;
			}
		}
		if (response < 3)
			return -ETIMEDOUT;
	}

	if (!stop_pending)
		clock_idle(h, 8);
	return 0;
}

static u8 read_4bit_byte(struct tssd_host *h, struct tssd_read_prefix *prefix)
{
	u8 hi = read_data_sample(h, prefix);

	return (hi << 4) | read_data_sample(h, prefix);
}

static int read_4bit_data(struct tssd_host *h, u8 *buf, unsigned int blksz,
			 unsigned int blocks, bool stop_pending,
			 struct tssd_read_prefix *prefix, u64 timeout_ns)
{
	unsigned int b, i, bit, lane;
	u16 crc[4], card_crc[4];
	u8 sample;

	for (b = 0; b < blocks; b++, buf += blksz) {
		if ((b || !prefix->started) && wait_data_start(h, GPIO_DAT, timeout_ns))
			return -ETIMEDOUT;

		/*
		 * Capture the response overlap in GPIO mode, then stream whole
		 * words. Leave at least one word to GPIO mode so the FPGA's
		 * 1024-nibble counter cannot roll over into its CRC state.
		 */
		i = 0;
		if (h->burst) {
			unsigned int prefix_bytes = DIV_ROUND_UP(prefix->count - prefix->next, 2);
			unsigned int gpio_bytes = min(blksz, max(4U, ALIGN(prefix_bytes, 4)));

			for (; i < gpio_bytes; i++)
				buf[i] = read_4bit_byte(h, prefix);
			if (blksz - i >= sizeof(u32)) {
				wr(h, R_GPIO, GPIO_IDLE);
				gpio_delay(h);
				for (; blksz - i >= sizeof(u32); i += sizeof(u32))
					put_unaligned_le32(readl(h->base + R_DATA), buf + i);
			}
		}
		for (; i < blksz; i++)
			buf[i] = read_4bit_byte(h, prefix);

		data_crc16_4bit(buf, blksz, crc);
		memset(card_crc, 0, sizeof(card_crc));
		for (bit = 0; bit < 16; bit++) {
			sample = read_data_sample(h, prefix);
			for (lane = 0; lane < 4; lane++)
				card_crc[lane] = (card_crc[lane] << 1) |
						 ((sample >> lane) & 1);
		}
		if (read_data_sample(h, prefix) != GPIO_DAT ||
		    memcmp(crc, card_crc, sizeof(crc))) {
			dev_err_ratelimited(mmc_dev(h->mmc),
					    "4-bit read block %u CRC %4ph/%4ph\n",
					    b, crc, card_crc);
			return -EILSEQ;
		}
	}

	if (!stop_pending)
		clock_idle(h, 8);
	return 0;
}

static int send_cmd(struct tssd_host *h, struct mmc_command *cmd,
		    struct mmc_data *data, u8 *resp,
		    struct tssd_read_prefix *prefix, u64 busy_timeout_ns)
{
	unsigned int i, n;
	u8 v, crc, data_mask = h->bus_width_4 ? GPIO_DAT : BIT(0);

	v = S_MULTIRD | S_MULTIWR;
	if (data && (data->flags & MMC_DATA_READ))
		v &= ~S_MULTIRD;
	if (data && (data->flags & MMC_DATA_WRITE) && !h->burst)
		v &= ~S_MULTIWR;
	wr(h, R_STATUS, v);

	{
		u8 frame[6];

		frame[0] = 0x40 | cmd->opcode;
		put_unaligned_be32(cmd->arg, frame + 1);
		frame[5] = crc7_be(0, frame, 5) | 1;
		for (i = 0; i < sizeof(frame); i++) {
			u8 b = frame[i];
			int bit;

			for (bit = 0; bit < 8; bit++) {
				v = GPIO_OE_CMD | GPIO_DAT;
				if (b & BIT(7))
					v |= GPIO_CMD;
				clock_out(h, v);
				b <<= 1;
			}
		}
	}
	if (!(cmd->flags & MMC_RSP_PRESENT)) {
		if (!data)
			clock_idle(h, 8);
		return 0;
	}

	n = (cmd->flags & MMC_RSP_136) ? 17 : 6;
	for (i = 0; i < TSSD_CMD_RESPONSE_CLOCKS; i++) {
		wr(h, R_GPIO, GPIO_IDLE);
		v = rd(h, R_GPIO);
		if (!(v & GPIO_CMD))
			break;
		wr(h, R_GPIO, GPIO_IDLE | GPIO_CLK);
	}
	if (i == TSSD_CMD_RESPONSE_CLOCKS)
		return -ETIMEDOUT;
	for (i = 0; i < n; i++) {
		unsigned int bit;

		v = 0;
		for (bit = 0; bit < 8; bit++) {
			u8 sample;

			wr(h, R_GPIO, GPIO_IDLE);
			rd(h, R_GPIO);
			sample = rd(h, R_GPIO);
			v = (v << 1) | !!(sample & GPIO_CMD);
			/* Data can start while CMD is still returning its response. */
			if (prefix && n == 6) {
				if (prefix->started) {
					prefix->samples[prefix->count++] = sample & data_mask;
				} else if (!(sample & data_mask)) {
					prefix->started = true;
				}
			}
			wr(h, R_GPIO, GPIO_IDLE | GPIO_CLK);
			gpio_delay(h);
		}
		resp[i] = v;
	}
	crc = crc7_be(0, resp + !!(cmd->flags & MMC_RSP_136),
		       n - 1 - !!(cmd->flags & MMC_RSP_136)) | 1;
	if ((cmd->flags & MMC_RSP_CRC) && crc != resp[n - 1]) {
		dev_err_ratelimited(mmc_dev(h->mmc),
				    "CMD%u response CRC mismatch: got %02x expected %02x, bytes %*ph\n",
				    cmd->opcode, resp[n - 1], crc, (int)n, resp);
		return -EILSEQ;
	}
	if (cmd->flags & MMC_RSP_136) {
		/* R2 carries 120 payload bits followed by CRC/end bits. */
		resp[16] = 0;
		for (i = 0; i < 4; i++)
			cmd->resp[i] = get_unaligned_be32(resp + 1 + i * 4);
	} else {
		if ((cmd->flags & MMC_RSP_OPCODE) &&
		    (resp[0] & 0x3f) != cmd->opcode)
			return -EILSEQ;
		cmd->resp[0] = get_unaligned_be32(resp + 1);
		cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
	}
	if (cmd->flags & MMC_RSP_BUSY) {
		unsigned int released = 0;
		ktime_t deadline;

		if (cmd->busy_timeout)
			busy_timeout_ns = (u64)cmd->busy_timeout * NSEC_PER_MSEC;
		deadline = ktime_add_ns(ktime_get(), busy_timeout_ns ?: NSEC_PER_SEC);

		while (released < 3 && ktime_before(ktime_get(), deadline)) {
			wr(h, R_GPIO, GPIO_DAT_INPUT);
			if (rd(h, R_GPIO) & BIT(0))
				released++;
			else
				released = 0;
			wr(h, R_GPIO, GPIO_DAT_INPUT | GPIO_CLK);
		}
		if (released < 3)
			return -ETIMEDOUT;
	}
	if (!data)
		clock_idle(h, 8);
	return 0;
}

static int fpga_status(struct tssd_host *h)
{
	/* Reading status acknowledges the latched FPGA errors. */
	u8 status = rd(h, R_STATUS);

	if (!(status & (S_CRCERR | S_TIMEOUT)))
		return 0;

	dev_err_ratelimited(mmc_dev(h->mmc),
			    "data engine status error %02x\n", status);
	return status & S_TIMEOUT ? -ETIMEDOUT : -EILSEQ;
}

static int write_4bit_data(struct tssd_host *h, const u8 *buf,
			   unsigned int blocks, bool stop_pending,
			   bool *finish_write, u64 timeout_ns)
{
	unsigned int block, i;
	u8 status;
	int ret;

	clock_idle(h, 2);
	clock_out(h, GPIO_WRITE_START);
	status = rd(h, R_STATUS);

	for (block = 0; block < blocks; block++, buf += TSSD_BLOCK_SIZE) {
		if (block + 1 == blocks)
			wr(h, R_STATUS, status & ~S_MULTIWR);
		for (i = 0; i < TSSD_BLOCK_SIZE; i += sizeof(u32))
			writel(get_unaligned_le32(buf + i), h->base + R_DATA);
	}
	wr(h, R_GPIO, GPIO_IDLE | GPIO_CLK);

	if (stop_pending && (status & S_MULTIWR)) {
		*finish_write = true;
		return 0;
	}

	wr(h, R_WRITE_BUSY, 0);
	ret = wait_idle(h, DIV_ROUND_UP_ULL(timeout_ns, NSEC_PER_USEC));
	if (!ret && (status & S_MULTIWR))
		wr(h, R_STATUS, status | S_MULTIWR);

	return ret;
}

static int do_data(struct tssd_host *h, struct mmc_data *data,
		   unsigned int bytes, bool stop_pending,
		   struct tssd_read_prefix *prefix, bool *finish_write,
		   u64 timeout_ns)
{
	u8 *buf;
	int ret;

	buf = kmalloc(bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (data->flags & MMC_DATA_READ) {
		ret = wait_idle(h, TSSD_IDLE_TIMEOUT_US);
		if (ret)
			goto out;
		if (h->bus_width_4)
			ret = read_4bit_data(h, buf, data->blksz,
					     data->blocks, stop_pending, prefix, timeout_ns);
		else
			ret = read_1bit_data(h, buf, data->blksz,
					     data->blocks, stop_pending, prefix, timeout_ns);
		if (ret)
			goto out;
	} else {
		if (sg_copy_to_buffer(data->sg, data->sg_len, buf, bytes) != bytes) {
			ret = -EINVAL;
			goto out;
		}
		if (h->burst)
			ret = write_4bit_data(h, buf, data->blocks, stop_pending,
					      finish_write, timeout_ns);
		else
			ret = write_gpio_data(h, buf, data->blksz, data->blocks,
					      stop_pending, timeout_ns);
		if (ret)
			goto out;
	}
	ret = fpga_status(h);
	if (ret)
		goto out;
	if ((data->flags & MMC_DATA_READ) &&
	    sg_copy_from_buffer(data->sg, data->sg_len, buf, bytes) != bytes) {
		ret = -EINVAL;
		goto out;
	}
out:
	kfree(buf);
	return ret;
}

static void tssd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct tssd_host *h = mmc_priv(mmc);
	struct tssd_read_prefix prefix = {};
	struct mmc_data *data = mrq->data;
	unsigned int bytes = 0;
	u64 timeout_ns;
	bool finish_write = false;
	u8 resp[17];
	int ret;

	mutex_lock(&h->lock);
	if (data &&
	    (!data->blksz || data->blksz > mmc->max_blk_size || !data->blocks ||
	     check_mul_overflow(data->blksz, data->blocks, &bytes) ||
	     bytes > mmc->max_req_size ||
	     (h->burst && (data->flags & MMC_DATA_WRITE) &&
	      data->blksz != TSSD_BLOCK_SIZE))) {
		data->error = -EINVAL;
		goto done;
	}
	ret = wait_idle(h, TSSD_IDLE_TIMEOUT_US);
	if (ret) {
		mrq->cmd->error = ret;
		goto done;
	}

	/* Clear latched FPGA CRC/timeout status from the previous request. */
	rd(h, R_STATUS);
	if (mrq->sbc) {
		ret = send_cmd(h, mrq->sbc, NULL, resp, NULL, 0);
		if (ret) {
			mrq->sbc->error = ret;
			goto done;
		}
	}

	ret = send_cmd(h, mrq->cmd, data, resp,
		       data && (data->flags & MMC_DATA_READ) ? &prefix : NULL, 0);
	if (ret) {
		mrq->cmd->error = ret;
		dev_err_ratelimited(mmc_dev(mmc), "CMD%u failed: %d\n",
				    mrq->cmd->opcode, ret);
		goto done;
	}
	if (!data)
		goto done;

	timeout_ns = data_timeout_ns(h, data);
	data->error = do_data(h, data, bytes, mrq->stop != NULL,
			      &prefix, &finish_write, timeout_ns);
	if (data->error)
		dev_err_ratelimited(mmc_dev(mmc),
				    "CMD%u arg %08x blocks %u data phase failed: %d\n",
				    mrq->cmd->opcode, mrq->cmd->arg,
				    data->blocks, data->error);

	if (mrq->stop) {
		/* A write stop can wait for the card to finish programming. */
		ret = send_cmd(h, mrq->stop, NULL, resp, NULL,
			       data->flags & MMC_DATA_WRITE ? timeout_ns : 0);
		mrq->stop->error = ret;
		if (ret)
			dev_err_ratelimited(mmc_dev(mmc), "CMD%u stop failed: %d\n",
					    mrq->stop->opcode, ret);
	}
	if (finish_write) {
		wr(h, R_WRITE_BUSY, 0);
		ret = wait_idle(h, DIV_ROUND_UP_ULL(timeout_ns, NSEC_PER_USEC));
		if (!ret)
			ret = fpga_status(h);
		if (ret && !data->error)
			data->error = ret;
		wr(h, R_STATUS, S_MULTIRD | S_MULTIWR);
	}
	if (!data->error && (!mrq->stop || !mrq->stop->error))
		data->bytes_xfered = bytes;
done:
	mutex_unlock(&h->lock);
	mmc_request_done(mmc, mrq);
}

static void tssd_power_up(struct tssd_host *h)
{
	unsigned int i;

	if (wait_idle(h, TSSD_IDLE_TIMEOUT_US))
		dev_warn(mmc_dev(h->mmc), "FPGA stream engine did not idle before reset\n");

	/* The FPGA turns off card power when the host drives all pins low. */
	writew(0, h->base + R_CLKSEL);
	wr(h, R_STATUS, S_MULTIRD | S_MULTIWR);
	wr(h, R_GPIO, 0);
	for (i = 0; i < 8; i++)
		writew(i, h->base + R_CLKSEL);
	msleep(100);

	writew(0, h->base + R_CLKSEL);
	wr(h, R_GPIO, GPIO_IDLE | GPIO_CLK);
	usleep_range(5000, 6000);
	rd(h, R_STATUS);
}

static void tssd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct tssd_host *h = mmc_priv(mmc);
	u8 old_power_mode = h->power_mode;
	unsigned int gpio_clock = min(ios->clock, TSSD_GPIO_MAX_FREQ);

	h->power_mode = ios->power_mode;
	if (ios->power_mode == MMC_POWER_OFF) {
		h->clock_delay_ns = 0;
		wr(h, R_GPIO, 0);
		return;
	}

	/* Keep the GPIO phases at or below the tested software rate. */
	h->clock_delay_ns = gpio_clock ? DIV_ROUND_UP(500000000U, gpio_clock) : 0;
	if (old_power_mode == MMC_POWER_OFF)
		tssd_power_up(h);

	/* The MMC core supplies the identification rate at MMC_POWER_ON. */
	if (ios->power_mode == MMC_POWER_ON && old_power_mode != MMC_POWER_ON)
		clock_idle(h, TSSD_INITIAL_CLOCKS);

	h->bus_width_4 = ios->bus_width == MMC_BUS_WIDTH_4;
	h->burst = h->bus_width_4 && ios->clock >= TSSD_BURST_FREQ;
	wr(h, R_STATUS, S_MULTIRD | S_MULTIWR);
	if (!ios->clock)
		wr(h, R_GPIO, GPIO_IDLE);
}

static int tssd_card_busy(struct mmc_host *mmc)
{
	struct tssd_host *h = mmc_priv(mmc);
	int busy;

	mutex_lock(&h->lock);
	if (h->power_mode != MMC_POWER_ON) {
		busy = 0;
	} else {
		/* Clock DAT0 once so cards that need clocks while busy can progress. */
		wr(h, R_GPIO, GPIO_IDLE);
		busy = !(rd(h, R_GPIO) & BIT(0));
		wr(h, R_GPIO, GPIO_IDLE | GPIO_CLK);
	}
	mutex_unlock(&h->lock);

	return busy;
}

static const struct mmc_host_ops tssd_ops = {
	.request = tssd_request,
	.set_ios = tssd_set_ios,
	.card_busy = tssd_card_busy,
};

static int tssd_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct tssd_host *h;
	int ret;

	mmc = devm_mmc_alloc_host(&pdev->dev, sizeof(*h));
	if (!mmc)
		return -ENOMEM;
	h = mmc_priv(mmc);
	h->mmc = mmc;
	h->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(h->base))
		return PTR_ERR(h->base);
	mutex_init(&h->lock);
	mmc->ops = &tssd_ops;
	mmc->f_min = 100000;
	mmc->f_max = TSSD_BURST_FREQ;
	/* Longer busy periods are polled by the MMC core through card_busy(). */
	mmc->max_busy_timeout = 1000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps2 = MMC_CAP2_NO_MMC | MMC_CAP2_NO_SDIO;
	ret = mmc_of_parse(mmc);
	if (ret)
		return ret;
	mmc->f_max = min(mmc->f_max, TSSD_BURST_FREQ);
	mmc->max_blk_size = TSSD_BLOCK_SIZE;
	mmc->max_req_size = 4096;
	mmc->max_blk_count = mmc->max_req_size / mmc->max_blk_size;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_segs = 1;
	platform_set_drvdata(pdev, mmc);
	ret = mmc_add_host(mmc);
	if (ret)
		return ret;
	dev_info(&pdev->dev, "TS SD core registered with MMC\n");
	return 0;
}

static void tssd_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	mmc_remove_host(mmc);
}

static const struct of_device_id tssd_of_match[] = {
	{ .compatible = "technologic,tssdcard" }, { }
};
MODULE_DEVICE_TABLE(of, tssd_of_match);

static struct platform_driver tssd_driver = {
	.probe = tssd_probe,
	.remove = tssd_remove,
	.driver = { .name = "tssdcard", .of_match_table = tssd_of_match },
};
module_platform_driver(tssd_driver);

MODULE_DESCRIPTION("TS-7800-V2 FPGA SD/MMC host controller");
MODULE_AUTHOR("Technologic Systems");
MODULE_LICENSE("GPL");
