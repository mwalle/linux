// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

static int
w25q256_post_bfpt_fixups(struct spi_nor *nor,
			 const struct sfdp_parameter_header *bfpt_header,
			 const struct sfdp_bfpt *bfpt,
			 struct spi_nor_flash_parameter *params)
{
	/*
	 * W25Q256JV supports 4B opcodes but W25Q256FV does not.
	 * Unfortunately, Winbond has re-used the same JEDEC ID for both
	 * variants which prevents us from defining a new entry in the parts
	 * table.
	 * To differentiate between W25Q256JV and W25Q256FV check SFDP header
	 * version: only JV has JESD216A compliant structure (version 5).
	 */
	if (bfpt_header->major == SFDP_JESD216_MAJOR &&
	    bfpt_header->minor == SFDP_JESD216A_MINOR)
		nor->flags |= SNOR_F_4B_OPCODES;

	return 0;
}

static struct spi_nor_fixups w25q256_fixups = {
	.post_bfpt = w25q256_post_bfpt_fixups,
};

static const struct flash_info winbond_parts[] = {
	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25x05", INFO(0xef3010, 0, 64 * 1024,  1,  SECT_4K) },
	{ "w25x10", INFO(0xef3011, 0, 64 * 1024,  2,  SECT_4K) },
	{ "w25x20", INFO(0xef3012, 0, 64 * 1024,  4,  SECT_4K) },
	{ "w25x40", INFO(0xef3013, 0, 64 * 1024,  8,  SECT_4K) },
	{ "w25x80", INFO(0xef3014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25x16", INFO(0xef3015, 0, 64 * 1024,  32, SECT_4K) },
	{ "w25q16dw", INFO(0xef6015, 0, 64 * 1024,  32,
			   SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			   SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB) },
	{ "w25x32", INFO(0xef3016, 0, 64 * 1024,  64, SECT_4K) },
	{ "w25q16jv-im/jm", INFO(0xef7015, 0, 64 * 1024,  32,
				 SECT_4K | SPI_NOR_DUAL_READ |
				 SPI_NOR_QUAD_READ | SPI_NOR_HAS_LOCK |
				 SPI_NOR_HAS_TB) },
	{ "w25q20cl", INFO(0xef4012, 0, 64 * 1024,  4, SECT_4K) },
	{ "w25q20bw", INFO(0xef5012, 0, 64 * 1024,  4, SECT_4K) },
	{ "w25q20ew", INFO(0xef6012, 0, 64 * 1024,  4, SECT_4K) },
	{ "w25q32", INFO(0xef4016, 0, 64 * 1024,  64, SECT_4K) },
	{ "w25q32dw", INFO(0xef6016, 0, 64 * 1024,  64,
			   SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			   SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB) },
	{ "w25q32jv", INFO(0xef7016, 0, 64 * 1024,  64,
			   SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			   SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{ "w25q32jwm", INFO(0xef8016, 0, 64 * 1024,  64,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			    SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
		       OTP_INFO(256, 3, 0x1000, 0x1000)
	},
	{ "w25q64jwm", INFO(0xef8017, 0, 64 * 1024, 128,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			    SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB) },
	{ "w25q128jwm", INFO(0xef8018, 0, 64 * 1024, 256,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			    SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB) },
	{ "w25q256jwm", INFO(0xef8019, 0, 64 * 1024, 512,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			    SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB) },
	{ "w25x64", INFO(0xef3017, 0, 64 * 1024, 128, SECT_4K) },
	{ "w25q64", INFO(0xef4017, 0, 64 * 1024, 128,
			 SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "w25q64dw", INFO(0xef6017, 0, 64 * 1024, 128,
			   SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			   SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB) },
	{ "w25q64jvm", INFO(0xef7017, 0, 64 * 1024, 128, SECT_4K) },
	{ "w25q128fw", INFO(0xef6018, 0, 64 * 1024, 256,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			    SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB) },
	{ "w25q128jv", INFO(0xef7018, 0, 64 * 1024, 256,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			    SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB) },
	{ "w25q80", INFO(0xef5014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25q80bl", INFO(0xef4014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25q128", INFO(0xef4018, 0, 64 * 1024, 256, SECT_4K) },
	{ "w25q256", INFO(0xef4019, 0, 64 * 1024, 512,
			  SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
	  .fixups = &w25q256_fixups },
	{ "w25q256jvm", INFO(0xef7019, 0, 64 * 1024, 512,
			     SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "w25q256jw", INFO(0xef6019, 0, 64 * 1024, 512,
			     SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "w25m512jv", INFO(0xef7119, 0, 64 * 1024, 1024,
			    SECT_4K | SPI_NOR_QUAD_READ | SPI_NOR_DUAL_READ) },
};

/**
 * winbond_set_4byte_addr_mode() - Set 4-byte address mode for Winbond flashes.
 * @nor:	pointer to 'struct spi_nor'.
 * @enable:	true to enter the 4-byte address mode, false to exit the 4-byte
 *		address mode.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int winbond_set_4byte_addr_mode(struct spi_nor *nor, bool enable)
{
	int ret;

	ret = spi_nor_set_4byte_addr_mode(nor, enable);
	if (ret || enable)
		return ret;

	/*
	 * On Winbond W25Q256FV, leaving 4byte mode causes the Extended Address
	 * Register to be set to 1, so all 3-byte-address reads come from the
	 * second 16M. We must clear the register to enable normal behavior.
	 */
	ret = spi_nor_write_enable(nor);
	if (ret)
		return ret;

	ret = spi_nor_write_ear(nor, 0);
	if (ret)
		return ret;

	return spi_nor_write_disable(nor);
}

static int winbond_otp_read(struct spi_nor *nor, loff_t addr, uint64_t len,
			    u8 *buf)
{
	u8 addr_width, read_opcode, read_dummy;
	enum spi_nor_protocol read_proto;
	int ret;

	read_opcode = nor->read_opcode;
	addr_width = nor->addr_width;
	read_dummy = nor->read_dummy;
	read_proto = nor->read_proto;

	nor->read_opcode = SPINOR_OP_WB_RSECR;
	nor->addr_width = 3;
	nor->read_dummy = 8;
	nor->read_proto = SNOR_PROTO_1_1_1;

	ret = spi_nor_read_data(nor, addr, len, buf);

	nor->read_opcode = read_opcode;
	nor->addr_width = addr_width;
	nor->read_dummy = read_dummy;
	nor->read_proto = read_proto;

	return ret;
}

static int winbond_otp_write(struct spi_nor *nor, loff_t addr, uint64_t len,
			     u8 *buf)
{
	u8 addr_width, program_opcode;
	enum spi_nor_protocol write_proto;
	int ret;

	program_opcode = nor->program_opcode;
	addr_width = nor->addr_width;
	write_proto = nor->write_proto;

	nor->program_opcode = SPINOR_OP_WB_PSECR;
	nor->addr_width = 3;
	nor->write_proto = SNOR_PROTO_1_1_1;

	/*
	 * We only support a write to one single page. For now all winbond
	 * flashes only have one page per OTP region.
	 */
	ret = spi_nor_write_enable(nor);
	if (ret)
		goto out;

	ret = spi_nor_write_data(nor, addr, len, buf);
	if (ret < 0)
		goto out;

	ret = spi_nor_wait_till_ready(nor);

out:
	nor->program_opcode = program_opcode;
	nor->addr_width = addr_width;
	nor->write_proto = write_proto;

	return ret;
}

static int _winbond_otp_lock_bit(unsigned int region)
{
	static const int lock_bits[] = { SR2_WB_LB1, SR2_WB_LB2, SR2_WB_LB3 };

	if (region >= ARRAY_SIZE(lock_bits))
		return -EINVAL;

	return lock_bits[region];
}

static int winbond_otp_lock(struct spi_nor *nor, unsigned int region)
{
	int lock_bit;
	u8 *sr2 = nor->bouncebuf;
	int ret;

	lock_bit = _winbond_otp_lock_bit(region);
	if (lock_bit < 0)
		return lock_bit;

	ret = spi_nor_read_cr(nor, sr2);
	if (ret)
		return ret;

	/* check if its already locked */
	if (*sr2 & lock_bit)
		return 0;

	return spi_nor_write_16bit_cr_and_check(nor, *sr2 | lock_bit);
}

static int winbond_otp_is_locked(struct spi_nor *nor, unsigned int region)
{
	int lock_bit;
	u8 *sr2 = nor->bouncebuf;
	int ret;

	lock_bit = _winbond_otp_lock_bit(region);
	if (lock_bit < 0)
		return lock_bit;

	ret = spi_nor_read_cr(nor, sr2);
	if (ret)
		return ret;

	return (*sr2 & lock_bit);
}

static const struct spi_nor_otp_ops winbond_otp_ops = {
	.read = winbond_otp_read,
	.write = winbond_otp_write,
	.lock = winbond_otp_lock,
	.is_locked = winbond_otp_is_locked,
};

static void winbond_default_init(struct spi_nor *nor)
{
	nor->params->set_4byte_addr_mode = winbond_set_4byte_addr_mode;
	if (nor->params->otp_info.n_otps)
		nor->params->otp_ops = &winbond_otp_ops;
}

static const struct spi_nor_fixups winbond_fixups = {
	.default_init = winbond_default_init,
};

const struct spi_nor_manufacturer spi_nor_winbond = {
	.name = "winbond",
	.parts = winbond_parts,
	.nparts = ARRAY_SIZE(winbond_parts),
	.fixups = &winbond_fixups,
};
