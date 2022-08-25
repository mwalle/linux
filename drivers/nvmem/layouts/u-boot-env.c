// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Rafał Miłecki <rafal@milecki.pl>
 */

#include <linux/crc32.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/slab.h>
#include <linux/of.h>

enum u_boot_env_format {
	U_BOOT_FORMAT_SINGLE,
	U_BOOT_FORMAT_REDUNDANT,
};

struct u_boot_env_image_single {
	__le32 crc32;
	u8 data[];
} __packed;

struct u_boot_env_image_redundant {
	__le32 crc32;
	u8 mark;
	u8 data[];
} __packed;

static int u_boot_env_add_cells(struct device *dev,
				struct nvmem_device *nvmem, uint8_t *buf,
				size_t data_offset, size_t data_len)
{
	struct nvmem_cell_info info = {0};
	char *data = buf + data_offset;
	char *var, *value, *eq;
	int err;

	for (var = data;
	     var < data + data_len && *var;
	     var = value + strlen(value) + 1) {
		eq = strchr(var, '=');
		if (!eq)
			break;
		*eq = '\0';
		value = eq + 1;

		info.name = var;
		info.offset = data_offset + value - data;
		info.bytes = strlen(value);
		info.np = of_get_child_by_name(dev->of_node, var);

		err = nvmem_add_one_cell(nvmem, &info);
		if (err)
			return err;
	}

	return 0;
}

static int u_boot_add_cells(struct device *dev, struct nvmem_device *nvmem,
			    struct nvmem_layout *layout)
{
	size_t size = nvmem_device_size(nvmem);
	enum u_boot_env_format format;
	size_t crc32_data_offset;
	size_t crc32_data_len;
	size_t crc32_offset;
	size_t data_offset;
	size_t data_len;
	u32 crc32;
	u32 calc;
	u8 *buf;
	int err;

	format = (uintptr_t)nvmem_layout_get_match_data(nvmem, layout);

	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		err = -ENOMEM;
		goto err_out;
	}

	err = nvmem_device_read(nvmem, 0, size, buf);
	if (err < 0) {
		dev_err(dev, "Failed to read from nvmem device (%pe)\n",
			ERR_PTR(err));
		goto err_kfree;
	} else if (err != size) {
		dev_err(dev, "Short read from nvmem device.\n");
		err = -EIO;
		goto err_kfree;
	}

	switch (format) {
	case U_BOOT_FORMAT_SINGLE:
		crc32_offset = offsetof(struct u_boot_env_image_single, crc32);
		crc32_data_offset = offsetof(struct u_boot_env_image_single, data);
		data_offset = offsetof(struct u_boot_env_image_single, data);
		break;
	case U_BOOT_FORMAT_REDUNDANT:
		crc32_offset = offsetof(struct u_boot_env_image_redundant, crc32);
		crc32_data_offset = offsetof(struct u_boot_env_image_redundant, mark);
		data_offset = offsetof(struct u_boot_env_image_redundant, data);
		break;
	}
	crc32 = le32_to_cpu(*(u32 *)(buf + crc32_offset));
	crc32_data_len = size - crc32_data_offset;
	data_len = size - data_offset;

	calc = crc32(~0, buf + crc32_data_offset, crc32_data_len) ^ ~0L;
	if (calc != crc32) {
		dev_err(dev, "Invalid calculated CRC32: 0x%08x (expected: 0x%08x)\n", calc, crc32);
		err = -EINVAL;
		goto err_kfree;
	}

	buf[size - 1] = '\0';
	err = u_boot_env_add_cells(dev, nvmem, buf, data_offset, data_len);
	if (err)
		dev_err(dev, "Failed to add cells: %d\n", err);

err_kfree:
	kfree(buf);
err_out:
	return err;
}

static const struct of_device_id u_boot_env_of_match_table[] = {
	{ .compatible = "u-boot,env", .data = (void *)U_BOOT_FORMAT_SINGLE, },
	{ .compatible = "u-boot,env-redundant-bool", .data = (void *)U_BOOT_FORMAT_REDUNDANT, },
	{ .compatible = "u-boot,env-redundant-count", .data = (void *)U_BOOT_FORMAT_REDUNDANT, },
	{},
};

static struct nvmem_layout u_boot_env_layout = {
	.name = "u_boot_env",
	.of_match_table = u_boot_env_of_match_table,
	.add_cells = u_boot_add_cells,
};

static int __init u_boot_env_init(void)
{
	return nvmem_layout_register(&u_boot_env_layout);
}
subsys_initcall(u_boot_env_init);
