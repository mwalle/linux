// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

#define ATMEL_SR_GLOBAL_PROTECT_MASK GENMASK(5, 2)

/**
 * atmel_set_global_protection - Do a Global Protect or Unprotect command
 * @nor:	pointer to 'struct spi_nor'
 * @ofs:	offset in bytes
 * @len:	len in bytes
 * @is_protect:	if true do a Global Protect otherwise it is a Global Unprotect
 *
 * Return: 0 on success, -error otherwise.
 */
static int atmel_set_global_protection(struct spi_nor *nor, loff_t ofs,
				       uint64_t len, bool is_protect)
{
	int ret;
	u8 sr;

	/* We only support locking the whole flash array */
	if (ofs || len != nor->params->size)
		return -EINVAL;

	ret = spi_nor_read_sr(nor, nor->bouncebuf, 1);
	if (ret)
		return ret;
	sr = nor->bouncebuf[0];

	/* SRWD bit needs to be cleared, otherwise the protection doesn't change */
	if (sr & SR_SRWD) {
		sr &= ~SR_SRWD;
		ret = spi_nor_write_sr_and_check(nor, sr);
		if (ret) {
			dev_err(nor->dev, "unable to clear SRWD bit, WP# asserted?\n");
			return ret;
		}
	}

	if (is_protect)
		sr |= ATMEL_SR_GLOBAL_PROTECT_MASK;
	else
		sr &= ~ATMEL_SR_GLOBAL_PROTECT_MASK;

	return spi_nor_write_sr_and_check(nor, sr);
}

static int atmel_global_protect(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	return atmel_set_global_protection(nor, ofs, len, true);
}

static int atmel_global_unprotect(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	return atmel_set_global_protection(nor, ofs, len, false);
}

static int atmel_is_global_protected(struct spi_nor *nor, loff_t ofs, uint64_t len)
{
	int ret;

	if (ofs >= nor->params->size || (ofs + len) > nor->params->size)
		return -EINVAL;

	ret = spi_nor_read_sr(nor, nor->bouncebuf, 1);
	if (ret)
		return ret;

	return ((nor->bouncebuf[0] & ATMEL_SR_GLOBAL_PROTECT_MASK) == ATMEL_SR_GLOBAL_PROTECT_MASK);
}

static const struct spi_nor_locking_ops atmel_global_protection_ops = {
	.lock = atmel_global_protect,
	.unlock = atmel_global_unprotect,
	.is_locked = atmel_is_global_protected,
};

static void atmel_global_protection_default_init(struct spi_nor *nor)
{
	nor->params->locking_ops = &atmel_global_protection_ops;
}

static const struct spi_nor_fixups atmel_global_protection_fixups = {
	.default_init = atmel_global_protection_default_init,
};

static const struct flash_info atmel_parts[] = {
	/* Atmel -- some are (confusingly) marketed as "DataFlash" */
	{ "at25fs010",  INFO(0x1f6601, 0, 32 * 1024,   4, SECT_4K | SPI_NOR_HAS_LOCK) },
	{ "at25fs040",  INFO(0x1f6604, 0, 64 * 1024,   8, SECT_4K | SPI_NOR_HAS_LOCK) },

	{ "at25df041a", INFO(0x1f4401, 0, 64 * 1024,   8,
			     SECT_4K | SPI_NOR_HAS_LOCK | SPI_NOR_WP_IS_VOLATILE)
			.fixups = &atmel_global_protection_fixups },
	{ "at25df321",  INFO(0x1f4700, 0, 64 * 1024,  64,
			     SECT_4K | SPI_NOR_HAS_LOCK | SPI_NOR_WP_IS_VOLATILE)
			.fixups = &atmel_global_protection_fixups },
	{ "at25df321a", INFO(0x1f4701, 0, 64 * 1024,  64,
			     SECT_4K | SPI_NOR_HAS_LOCK | SPI_NOR_WP_IS_VOLATILE)
			.fixups = &atmel_global_protection_fixups },
	{ "at25df641",  INFO(0x1f4800, 0, 64 * 1024, 128,
			     SECT_4K | SPI_NOR_HAS_LOCK | SPI_NOR_WP_IS_VOLATILE)
			.fixups = &atmel_global_protection_fixups },

	{ "at25sl321",	INFO(0x1f4216, 0, 64 * 1024, 64,
			     SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },

	{ "at26f004",   INFO(0x1f0400, 0, 64 * 1024,  8, SECT_4K) },
	{ "at26df081a", INFO(0x1f4501, 0, 64 * 1024, 16,
			     SECT_4K | SPI_NOR_HAS_LOCK | SPI_NOR_WP_IS_VOLATILE)
			.fixups = &atmel_global_protection_fixups },
	{ "at26df161a", INFO(0x1f4601, 0, 64 * 1024, 32,
			     SECT_4K | SPI_NOR_HAS_LOCK | SPI_NOR_WP_IS_VOLATILE)
			.fixups = &atmel_global_protection_fixups },
	{ "at26df321",  INFO(0x1f4700, 0, 64 * 1024, 64,
			     SECT_4K | SPI_NOR_HAS_LOCK | SPI_NOR_WP_IS_VOLATILE)
			.fixups = &atmel_global_protection_fixups },

	{ "at45db081d", INFO(0x1f2500, 0, 64 * 1024, 16, SECT_4K) },
};

const struct spi_nor_manufacturer spi_nor_atmel = {
	.name = "atmel",
	.parts = atmel_parts,
	.nparts = ARRAY_SIZE(atmel_parts),
};
