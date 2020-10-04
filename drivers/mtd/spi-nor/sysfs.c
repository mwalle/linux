// SPDX-License-Identifier: GPL-2.0
/*
 * sysfs properties for SPI NOR flashes
 *
 * Copyright 2020 Michael Walle <michael@walle.cc>
 */

#include <linux/mtd/spi-nor.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/sysfs.h>

#include "core.h"

static ssize_t status_reg_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);
	int len = nor->flags & SNOR_F_HAS_16BIT_SR ? 2 : 1;
	int ret;

	ret = spi_nor_lock_and_prep(nor);
	if (ret)
		return ret;

	ret = spi_nor_read_sr(nor, nor->bouncebuf, len);
	if (ret)
		goto out;

	if (nor->flags & SNOR_F_HAS_16BIT_SR)
		ret = sprintf(buf, "%02x%02x\n", nor->bouncebuf[1], nor->bouncebuf[0]);
	else
		ret = sprintf(buf, "%02x\n", nor->bouncebuf[0]);

out:
	spi_nor_unlock_and_unprep(nor);
	return ret;
}
static DEVICE_ATTR_RO(status_reg);

static ssize_t configuration_reg_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);
	int ret;

	ret = spi_nor_lock_and_prep(nor);
	if (ret)
		return ret;

	ret = spi_nor_read_cr(nor, nor->bouncebuf);
	spi_nor_unlock_and_unprep(nor);

	return ret ? ret : sprintf(buf, "%02x\n", nor->bouncebuf[0]);
}
static DEVICE_ATTR_RO(configuration_reg);

static struct attribute *spi_nor_sysfs_entries[] = {
	&dev_attr_status_reg.attr,
	&dev_attr_configuration_reg.attr,
	NULL
};

static ssize_t sfdp_table_read(struct file *filp, struct kobject *kobj,
                               struct bin_attribute *bin_attr, char *buf,
                               loff_t off, size_t count)
{
	struct spi_device *spi = to_spi_device(kobj_to_dev(kobj));
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);
	int ret;

	ret = spi_nor_lock_and_prep(nor);
	if (ret)
	    return ret;

        if (off >= nor->sfdp_size)
                count = 0;
        else {
                if (off + count > nor->sfdp_size)
                        count = nor->sfdp_size - off;

		ret = spi_nor_read_sfdp_dma_unsafe(nor, off, count, buf);
		if (ret < 0)
			ret = -EIO;
		else
			ret = count;
        }

	spi_nor_unlock_and_unprep(nor);
	return ret;
}
static BIN_ATTR_RO(sfdp_table, PAGE_SIZE);

static struct bin_attribute *spi_nor_sysfs_bin_entries[] = {
	&bin_attr_sfdp_table,
	NULL
};

static umode_t spi_nor_sysfs_is_visible(struct kobject *kobj,
					struct attribute *attr, int n)
{
	struct spi_device *spi = to_spi_device(kobj_to_dev(kobj));
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);

	if (attr == &dev_attr_configuration_reg.attr &&
	    nor->flags & SNOR_F_NO_READ_CR)
		return 0;

	return attr->mode;
}

static umode_t spi_nor_sysfs_is_bin_visible(struct kobject *kobj,
					    struct bin_attribute *attr, int n)
{
	struct spi_device *spi = to_spi_device(kobj_to_dev(kobj));
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);

	if (attr == &bin_attr_sfdp_table && !nor->sfdp_size)
		return 0;

	return 0444;
}

static struct attribute_group spi_nor_sysfs_attr_group = {
	.name		= NULL,
	.is_visible	= spi_nor_sysfs_is_visible,
	.is_bin_visible	= spi_nor_sysfs_is_bin_visible,
	.attrs		= spi_nor_sysfs_entries,
	.bin_attrs	= spi_nor_sysfs_bin_entries,
};

int spi_nor_sysfs_create(struct spi_nor *nor)
{
	return sysfs_create_group(&nor->dev->kobj, &spi_nor_sysfs_attr_group);
}

void spi_nor_sysfs_remove(struct spi_nor *nor)
{
	sysfs_remove_group(&nor->dev->kobj, &spi_nor_sysfs_attr_group);
}
