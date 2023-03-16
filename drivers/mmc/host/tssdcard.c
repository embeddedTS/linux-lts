// SPDX-License-Identifier: GPL-2.0

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/blkdev.h>
#include <linux/dma-mapping.h>
#include <linux/major.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/hdreg.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/pci.h>

static int poll_rate = 1;
module_param(poll_rate, int, 0644);
MODULE_PARM_DESC(poll_rate,
		 "Rate in seconds to poll for SD card.  Defaults to 1\n");

static int disable_poll;
module_param(disable_poll, int, 0644);
MODULE_PARM_DESC(disable_poll,
		 "Set to non-zero to only check for SD once on startup\n");

#define DRIVER_NAME "tssdcard"

/*
 * Current use of this driver only includes TS-7800-V2
 * but TS-7250-V2 could use up to 3
 */
#define MAX_SDS	3

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
#include "tssdcore2.c"

static DEFINE_MUTEX(tssdcore_lock);
static struct semaphore sem;
static atomic_t busy;

struct tssdcard_dev {
	struct device *dev;
	struct sdcore tssdcore;
	char *devname;
	sector_t sectors;
	struct gendisk *gd;
	struct bio *bio;
	struct bio *biotail;
	spinlock_t lock;
	atomic_t users;    /* How many users */
	struct task_struct *thread;
	wait_queue_head_t event;
	struct work_struct diskpoll_work;
	struct workqueue_struct *diskpoll_queue;
	struct timer_list cd_timer;
	unsigned long timeout;
	unsigned long lasttimeout;
	int cardpresent;
	int lasterr;
	int major;
};

struct tssdcard_host {
	struct platform_device *pdev;
	struct resource *mem_res;
	void __iomem *base;
	int numluns;
	struct tssdcard_dev luns[MAX_SDS];
};

void tssdcard_debug(void *arg,
		    unsigned int code,
		    const char *func,
		    unsigned int line, ...)
{
	struct tssdcard_dev *dev = (struct tssdcard_dev *)arg;
	va_list ap;
	unsigned int s, x, y, z;

	/* Only print each message once */
	if (dev->lasterr == code)
		return;
	dev->lasterr = code;

	va_start(ap, line);
	switch (code) {
	case SD_HW_TMOUT:
		s = va_arg(ap, unsigned int); /* sector */
		x = va_arg(ap, unsigned int); /* reg val */
		pr_info("SD hardware timeout, sect=%u (0x%x)\n", s, x);
		break;
	case SD_DAT_BAD_CRC:
		s = va_arg(ap, unsigned int); /* sector */
		x = va_arg(ap, unsigned int); /* reg val */
		pr_info("SD hw detected bad CRC16, sect=%u (0x%x)\n", s, x);
		break;
	case READ_FAIL:
		s = va_arg(ap, unsigned int); /* sector */
		pr_info("SD read failed, sect=%u\n", s);
		break;
	case WRITE_FAIL:
		s = va_arg(ap, unsigned int); /* sector */
		x = va_arg(ap, unsigned int); /* sdcmd() ret status */
		pr_info("SD write failed, sect=%u (0x%x)\n", s, x);
		break;
	case SD_STOP_FAIL:
		x = va_arg(ap, unsigned int); /* sdcmd() ret status */
		pr_info("SD stop transmission failed (0x%x)\n", x);
		break;
	case SD_RESP_FAIL:
		x = va_arg(ap, unsigned int); /* SD cmd */
		y = va_arg(ap, unsigned int); /* response status */
		pr_info("SD cmd 0x%x resp code has err bits 0x%x\n", x, y);
		break;
	case SD_RESP_BAD_CRC:
		x = va_arg(ap, unsigned int); /* SD cmd */
		y = va_arg(ap, unsigned int); /* calculated */
		z = va_arg(ap, unsigned int); /* rx'ed */
		pr_info("SD cmd 0x%x resp bad crc (0x%x != 0x%x)\n", x, y, z);
		break;
	case SD_RESP_WRONG_REQ:
		x = va_arg(ap, unsigned int); /* SD cmd */
		y = va_arg(ap, unsigned int); /* cmd in response */
		pr_info("SD response for wrong cmd. (0x%x != 0x%x)\n", x, y);
		break;
	case SD_SW_TMOUT:
		if (dev->cardpresent)
			pr_info("SD soft timeout(%pS)\n",
				__builtin_return_address(0));
		break;
	}
	va_end(ap);
}

static void tssdcard_add_bio(struct tssdcard_dev *dev, struct bio *bio)
{
	spin_lock(&dev->lock);
	if (dev->biotail) {
		dev->biotail->bi_next = bio;
		dev->biotail = bio;
	} else {
		dev->bio = dev->biotail = bio;
	}
	spin_unlock(&dev->lock);
}

static struct bio *tssdcard_get_bio(struct tssdcard_dev *dev)
{
	struct bio *bio;

	spin_lock(&dev->lock);
	bio = dev->bio;
	if (bio) {
		if (bio == dev->biotail)
			dev->biotail = NULL;
		dev->bio = bio->bi_next;
		bio->bi_next = NULL;
	}
	spin_unlock(&dev->lock);
	return bio;
}

static void tssdcard_reset_timeout(void *data)
{
	struct sdcore *sd = (struct sdcore *)data;
	struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;

	/* SD Spec allows 1 second for cards to answer */
	dev->timeout = jiffies + HZ;
}

static int tssdcard_transfer(struct tssdcard_dev *dev, unsigned long sector,
			      unsigned long nsect, char *buffer, int rw)
{
	int ret = 0;

	dev_dbg(dev->dev, "%s size:%lld sector:%lu nsect:%lu rw:%d\n",
		__func__, (long long)dev->sectors, sector, nsect, rw);

	tssdcard_reset_timeout(&dev->tssdcore);
	switch (rw) {
	case WRITE:
		ret = sdwrite(&dev->tssdcore, sector, buffer, nsect);
		if (ret && !dev->tssdcore.sd_wprot) {
			if (sdreset(&dev->tssdcore) != 0) {
				tssdcard_reset_timeout(&dev->tssdcore);
				ret = sdwrite(&dev->tssdcore, sector,
					      buffer, nsect);
			}
		}
		break;

	case READ:
	//case READA:
		ret = sdread(&dev->tssdcore, sector, buffer, nsect);
		if (ret) {
			if (sdreset(&dev->tssdcore) != 0) {
				tssdcard_reset_timeout(&dev->tssdcore);
				ret = sdread(&dev->tssdcore, sector,
					     buffer, nsect);
			}
		}
	}

	return ret;
}

static void tssdcard_handle_bio(struct tssdcard_dev *dev, struct bio *bio)
{
	struct bio_vec bvec;
	struct bvec_iter iter;
	sector_t sector, end_sector, n_sectors;
	char *buffer;
	int ret = 0;

	sector = bio->bi_iter.bi_sector;
	end_sector = (bio->bi_iter.bi_sector) + (bio->bi_iter.bi_size >> 9) - 1;

	if ((bio->bi_iter.bi_size % 512) != 0)
		panic("Invalid transfer, bi_size 512 != 0\n");

	bio_for_each_segment(bvec, bio, iter) {
		if ((sector + (bvec.bv_len >> 9)) > end_sector)
			n_sectors = end_sector - sector + 1;
		else
			n_sectors = bvec.bv_len >> 9;
		if (n_sectors == 0)
			continue;

		buffer = kmap(bvec.bv_page) + bvec.bv_offset;
		ret = tssdcard_transfer(dev, sector, n_sectors, buffer,
					bio_data_dir(bio));
		sector += n_sectors;
		kunmap(bvec.bv_page);
	}

	bio_endio(bio);

	if (ret) {
		dev->cardpresent = 0;
		queue_work(dev->diskpoll_queue,
			   &dev->diskpoll_work);
	}
}

static void tssdcard_delay(void *data, unsigned int us)
{
	if (us > 50000)
		msleep_interruptible(us/1000);
	else
		udelay(us);
}

static int tssdcard_timeout_relaxed(void *data)
{
	struct sdcore *sd = (struct sdcore *)data;
	struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;
	int ret;

	dev->lasttimeout = jiffies;

	if (jiffies_to_msecs(dev->timeout - jiffies) > 50)
		msleep_interruptible(10);

	ret = time_is_before_jiffies(dev->timeout);

	return ret;
}

static int tssdcard_timeout(void *data)
{
	struct sdcore *sd = (struct sdcore *)data;
	struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;
	int ret;

	dev->lasttimeout = jiffies;
	ret = time_is_before_jiffies(dev->timeout);

	return ret;
}

static void tssdcard_irqwait(void *data, unsigned int x)
{
	struct tssdcard_dev *dev = (struct tssdcard_dev *)data;
	uint32_t reg;

	do {
#ifdef CONFIG_PREEMPT_NONE
		/* Default marvell kernel config has no preempt, so
		 * to support that:
		 */
		cond_resched();
#endif
		reg = readl((uint32_t *)(dev->tssdcore.sd_regstart + 0x108));
	} while ((reg & 4) == 0);
}

static void tssdcard_release(struct gendisk *disk)
{
	struct tssdcard_dev *dev = disk->private_data;

	atomic_dec(&dev->users);
	if (atomic_read(&dev->users) == 0) {
		if (dev->thread != NULL) {
			char buffer[512];
			int ret;

			kthread_stop(dev->thread);
			dev->thread = NULL;
			if (dev->sectors) {
				tssdcard_reset_timeout(&dev->tssdcore);
				ret = sdread(&dev->tssdcore, 0, buffer, 1);
			}
		}
	}
}

static int tssdcard_peek_bio(struct tssdcard_dev *dev)
{
	int ret = 0;

	spin_lock(&dev->lock);
	if (dev->bio != NULL)
		ret = 1;
	spin_unlock(&dev->lock);

	return ret;
}

static int tssdcard_thread(void *data)
{
	struct tssdcard_dev *dev = data;
	struct bio *bio;

	while (!kthread_should_stop()) {
		wait_event_interruptible(dev->event,
					 tssdcard_peek_bio(dev) ||
					 kthread_should_stop());

		if (down_interruptible(&sem))
			continue;

		atomic_inc(&busy);
		if (atomic_read(&busy) > 1)
			panic("recursive make_request!\n");

		bio = tssdcard_get_bio(dev);
		if (bio)
			tssdcard_handle_bio(dev, bio);

		atomic_dec(&busy);
		up(&sem);
	}

	return 0;
}

static int tssdcard_open(struct gendisk *gd, fmode_t mode)
{
	struct tssdcard_dev *dev = gd->private_data;

	if (!atomic_read(&dev->users))
		disk_check_media_change(gd);
	atomic_inc(&dev->users);
	if (dev->thread == NULL && atomic_read(&dev->users)) {
		dev->thread = kthread_create(tssdcard_thread,
					     dev, dev->devname);
		if (IS_ERR(dev->thread))
			dev->thread = NULL;
		else
			wake_up_process(dev->thread);
	}

	return 0;
}

static void tssdcard_make_request(struct bio *bio)
{
	struct tssdcard_dev *dev = bio->bi_bdev->bd_disk->private_data;

	tssdcard_add_bio(dev, bio);
	wake_up(&dev->event);
}

static int tssdcard_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct tssdcard_dev *dev = bdev->bd_disk->private_data;

	geo->cylinders = dev->sectors >> 9 / (4 * 16);
	geo->heads = 4;
	geo->sectors = 16;
	return 0;
}

static const struct block_device_operations tssdcard_ops = {
	.owner		= THIS_MODULE,
	.open		= tssdcard_open,
	.release	= tssdcard_release,
	.getgeo		= tssdcard_getgeo,
	.submit_bio	= tssdcard_make_request,
};

static void tssdcard_alloc_disk(struct tssdcard_dev *dev)
{
	dev->bio = dev->biotail = NULL;

	dev->gd = blk_alloc_disk(CONFIG_MMC_BLOCK_MINORS);
	if (dev->gd == NULL) {
		pr_err(DRIVER_NAME ": Failed to alloc_disk");
		return;
	}

	strcpy(dev->gd->disk_name, dev->devname);

	blk_queue_flag_set(QUEUE_FLAG_NONROT, dev->gd->queue);

	set_capacity(dev->gd, dev->sectors);
	dev->gd->flags = 0;
	dev->gd->fops = &tssdcard_ops;
	dev->gd->private_data = dev;

	/* Check disk WP */
	set_disk_ro(dev->gd, dev->tssdcore.sd_wprot);

	/* FIXME: This return should be checked, generates warning right now. */
	add_disk(dev->gd);
}
static void tssdcard_cleanup_disk(struct tssdcard_dev *dev)
{
	pr_info("SD card was removed!\n");
	del_gendisk(dev->gd);
	put_disk(dev->gd);
	dev->sectors = 0;
}

static void diskpoll_thread(struct work_struct *work)
{
	struct tssdcard_dev *dev = container_of(work, struct tssdcard_dev,
		diskpoll_work);

	if (!dev->cardpresent && dev->sectors != 0)
		tssdcard_cleanup_disk(dev);
	else
		dev->sectors = sdreset(&dev->tssdcore);

	if (dev->sectors == 0) {
		dev->tssdcore.os_timeout = tssdcard_timeout_relaxed;
		if (!disable_poll)
			mod_timer(&dev->cd_timer, jiffies + (HZ * poll_rate));
	} else {
		dev->cardpresent = 1;
		dev->tssdcore.os_timeout = tssdcard_timeout;
		tssdcard_alloc_disk(dev);
	}
}

static void tssdcard_card_poll(struct timer_list *t)
{
	struct tssdcard_dev *dev = from_timer(dev, t, cd_timer);

	queue_work(dev->diskpoll_queue, &dev->diskpoll_work);
}

static int setup_device(struct tssdcard_host *host, int lun)
{
	int ret = 0;
	struct tssdcard_dev *dev = &host->luns[lun];

	dev->dev = &host->pdev->dev;
	/* IO Remapping (use the same virtual address for all LUNs) */
	dev->tssdcore.sd_regstart = (unsigned int)host->base;
	dev->tssdcore.sd_lun = lun;
	dev->tssdcore.os_timeout = tssdcard_timeout;
	dev->tssdcore.os_reset_timeout = tssdcard_reset_timeout;
	dev->tssdcore.os_arg = dev;
	dev->tssdcore.os_delay = tssdcard_delay;
	dev->tssdcore.os_irqwait = tssdcard_irqwait;
	dev->tssdcore.sd_writeparking = 1;
	dev->tssdcore.debug = tssdcard_debug;
	dev->tssdcore.debug_arg = dev;
	dev->major = register_blkdev(UNNAMED_MAJOR, DRIVER_NAME);

	dev->devname = kmalloc(32, GFP_KERNEL);
	if (!dev->devname)
		return -ENOMEM;
	snprintf(dev->devname, 32, "%s%c", DRIVER_NAME, lun + 'a');

	init_waitqueue_head(&dev->event);
	sema_init(&sem, 1);
	atomic_set(&busy, 0);
	spin_lock_init(&dev->lock);

	/* sdreset sleeps so we need our own workqueue */
	dev->diskpoll_queue = alloc_ordered_workqueue(dev->devname, 0);
	if (!dev->diskpoll_queue)
		return -ENOMEM;

	INIT_WORK(&dev->diskpoll_work, diskpoll_thread);

	timer_setup(&dev->cd_timer, tssdcard_card_poll, 0);

	/* Start polling for the card */
	queue_work(dev->diskpoll_queue,
	   &dev->diskpoll_work);

	return ret;
}

static int tssdcard_probe(struct platform_device *pdev)
{
	int i, ret = 0;
	struct tssdcard_host *host;
	struct resource *res = 0;

	host = kzalloc(sizeof(struct tssdcard_host), GFP_KERNEL);
	if (host == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	host->numluns = 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!devm_request_mem_region(&pdev->dev, res->start,
	    resource_size(res), pdev->name)) {
		ret = -EBUSY;
		goto out;
	}

	host->base = devm_ioremap(&pdev->dev, res->start,
				  resource_size(res));
	if (!host->base) {
		ret = -EFAULT;
		goto out;
	}

	for (i = 0; i < host->numluns; i++) {
		ret = setup_device(host, i);
		if (ret)
			goto out;
	}

	platform_set_drvdata(pdev, host);

	return 0;

out:
	return ret;
}

static int tssdcard_remove(struct platform_device *pdev)
{
	struct tssdcard_host *host = (struct tssdcard_host *)pdev->dev.p;
	int i;

	for (i = 0; i < host->numluns; i++) {
		struct tssdcard_dev *dev = &host->luns[i];

		dev_dbg(dev->dev, "dev[%d] ...\n", i);

		if (dev->sectors == 0)
			continue;

		if (dev->gd)
			put_disk(dev->gd);

		unregister_blkdev(NBD_MAJOR, "nbd");
		kfree(dev->devname);
	}
	return 0;
}

static const struct platform_device_id tssdcard_devtype[] = {
	{
		.name = "tssdcard-mmc",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, tssdcard_devtype);

static const struct of_device_id tssdcard_of_match[] = {
	{
		.compatible = "technologic,tssdcard",
	}
};

static struct platform_driver tssdcard_driver = {
	.probe =  tssdcard_probe,
	.remove = tssdcard_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = tssdcard_of_match,
	}
};

module_platform_driver(tssdcard_driver);

MODULE_DESCRIPTION("TS-7800-V2 SDHC Driver");
MODULE_AUTHOR("Ian Coughlan, Technologic Systems");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tssdcard");
