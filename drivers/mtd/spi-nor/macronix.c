// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

static int
mx25u3235f_post_bfpt_fixups(struct spi_nor *nor,
			    const struct sfdp_parameter_header *bfpt_header,
			    const struct sfdp_bfpt *bfpt,
			    struct spi_nor_flash_parameter *params)
{
	/*
	 * MX25U3235F has a JESD216 rev A BFPT table. MX25U3232F uses
	 * a later version. Thus we use this to differentiate between
	 * MX25U3235F and MX25U3232F.
	 *
	 * The MX25U3232F has 4 block protection bits and one TB bit
	 * which is OTP.
	 */
	if (bfpt_header->length == BFPT_DWORD_MAX)
		nor->flags |= (SNOR_F_HAS_LOCK | SNOR_F_HAS_4BIT_BP |
			       SNOR_F_HAS_OTP_TB | SNOR_F_HAS_CR_TB);

	return 0;
}

static struct spi_nor_fixups mx25u3235f_fixups = {
	.post_bfpt = mx25u3235f_post_bfpt_fixups,
};

static int
mx25l25635_post_bfpt_fixups(struct spi_nor *nor,
			    const struct sfdp_parameter_header *bfpt_header,
			    const struct sfdp_bfpt *bfpt,
			    struct spi_nor_flash_parameter *params)
{
	/*
	 * MX25L25635F supports 4B opcodes but MX25L25635E does not.
	 * Unfortunately, Macronix has re-used the same JEDEC ID for both
	 * variants which prevents us from defining a new entry in the parts
	 * table.
	 * We need a way to differentiate MX25L25635E and MX25L25635F, and it
	 * seems that the F version advertises support for Fast Read 4-4-4 in
	 * its BFPT table.
	 */
	if (bfpt->dwords[BFPT_DWORD(5)] & BFPT_DWORD5_FAST_READ_4_4_4)
		nor->flags |= SNOR_F_4B_OPCODES;

	return 0;
}

static struct spi_nor_fixups mx25l25635_fixups = {
	.post_bfpt = mx25l25635_post_bfpt_fixups,
};

static const struct flash_info macronix_parts[] = {
	/* Macronix */
	{ "mx25l512e",   INFO(0xc22010, 0, 64 * 1024,   1, SECT_4K) },
	{ "mx25l2005a",  INFO(0xc22012, 0, 64 * 1024,   4, SECT_4K) },
	{ "mx25l4005a",  INFO(0xc22013, 0, 64 * 1024,   8, SECT_4K) },
	{ "mx25l8005",   INFO(0xc22014, 0, 64 * 1024,  16, 0) },
	{ "mx25l1606e",  INFO(0xc22015, 0, 64 * 1024,  32, SECT_4K) },
	{ "mx25l3205d",  INFO(0xc22016, 0, 64 * 1024,  64, SECT_4K) },
	{ "mx25l3255e",  INFO(0xc29e16, 0, 64 * 1024,  64, SECT_4K) },
	{ "mx25l6405d",  INFO(0xc22017, 0, 64 * 1024, 128, SECT_4K)
			 OTP_INFO1(64, 0) },
	{ "mx25u2033e",  INFO(0xc22532, 0, 64 * 1024,   4, SECT_4K) },
	{ "mx25u3235f",	 INFO(0xc22536, 0, 64 * 1024,  64,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ)
		.fixups = &mx25u3235f_fixups },
	{ "mx25u4035",   INFO(0xc22533, 0, 64 * 1024,   8, SECT_4K) },
	{ "mx25u8035",   INFO(0xc22534, 0, 64 * 1024,  16, SECT_4K) },
	{ "mx25u6435f",  INFO(0xc22537, 0, 64 * 1024, 128, SECT_4K) },
	{ "mx25l12805d", INFO(0xc22018, 0, 64 * 1024, 256, SECT_4K) },
	{ "mx25l12855e", INFO(0xc22618, 0, 64 * 1024, 256, 0) },
	{ "mx25r1635f",  INFO(0xc22815, 0, 64 * 1024,  32,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25r3235f",  INFO(0xc22816, 0, 64 * 1024,  64,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25u12835f", INFO(0xc22538, 0, 64 * 1024, 256,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25l25635e", INFO(0xc22019, 0, 64 * 1024, 512,
			      SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		.fixups = &mx25l25635_fixups },
	{ "mx25u25635f", INFO(0xc22539, 0, 64 * 1024, 512,
			      SECT_4K | SPI_NOR_4B_OPCODES) },
	{ "mx25u51245g", INFO(0xc2253a, 0, 64 * 1024, 1024,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ "mx25v8035f",  INFO(0xc22314, 0, 64 * 1024,  16,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25l25655e", INFO(0xc22619, 0, 64 * 1024, 512, 0) },
	{ "mx25l51245g", INFO(0xc2201a, 0, 64 * 1024, 1024,
			      SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			      SPI_NOR_4B_OPCODES) },
	{ "mx66l51235l", INFO(0xc2201a, 0, 64 * 1024, 1024,
			      SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			      SPI_NOR_4B_OPCODES) },
	{ "mx66u51235f", INFO(0xc2253a, 0, 64 * 1024, 1024,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ "mx66l1g45g",  INFO(0xc2201b, 0, 64 * 1024, 2048,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx66l1g55g",  INFO(0xc2261b, 0, 64 * 1024, 2048,
			      SPI_NOR_QUAD_READ) },
	{ "mx66u2g45g",	 INFO(0xc2253c, 0, 64 * 1024, 4096,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
};

/**
 * macronix_set_secured_otp_mode() - Set secured OTP mode for Macronix flashes.
 * @nor:	pointer to 'struct spi_nor'.
 * @enable:	true to enter the secured OTP mode, false to exit the secured
 *		OTP mode.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int macronix_set_secured_otp_mode(struct spi_nor *nor, bool enable)
{
	u8 cmd = enable ? SPINOR_OP_ENSO : SPINOR_OP_EXSO;
	int ret;

	ret = spi_nor_simple_cmd(nor, cmd);
	if (ret)
		dev_dbg(nor->dev, "error %d setting secured OTP mode\n", ret);

	return ret;
}

/**
 * spi_nor_read_scur() - Read the Security Register using the
 * SPINOR_OP_RDSCUR (2Bh) command.
 * @nor:	pointer to 'struct spi_nor'
 * @scur:	pointer to a DMA-able buffer where the value of the
 *		Security Register will be written.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int macronix_read_scur(struct spi_nor *nor, u8 *scur)
{
       int ret;

       ret = spi_nor_simple_cmd_din(nor, SPINOR_OP_RDSCUR, scur, 1);
       if (ret)
               dev_dbg(nor->dev, "error %d reading SCUR\n", ret);

       return ret;
}

/**
 * spi_nor_write_scur() - Write the Security Register using the
 * SPINOR_OP_WRSCUR (2Fh) command.
 * @nor:	pointer to 'struct spi_nor'
 *
 * This register contains only one OTP bit. The command doesn't take any
 * arguments. In fact it _must not_ take any arugments. Otherwise the command
 * is ignored.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int macronix_write_scur(struct spi_nor *nor)
{
       int ret;

       ret = spi_nor_simple_cmd(nor, SPINOR_OP_WRSCUR);
       if (ret)
               dev_dbg(nor->dev, "error %d writing SCUR\n", ret);

       return ret;
}

static int macronix_otp_read(struct spi_nor *nor, loff_t addr, uint64_t len,
			     u8 *buf)
{
	int ret;

	ret = macronix_set_secured_otp_mode(nor, true);
	if (ret)
	    return ret;

	ret = spi_nor_read_data(nor, addr, len, buf);

	macronix_set_secured_otp_mode(nor, false);

	return ret;
}

static int macronix_otp_write(struct spi_nor *nor, loff_t addr, uint64_t len,
			      u8 *buf)
{
	int ret;

	ret = macronix_set_secured_otp_mode(nor, true);
	if (ret)
	    return ret;

	ret = spi_nor_write_enable(nor);
	if (ret)
		goto out;

	ret = spi_nor_write_data(nor, addr, len, buf);
	if (ret < 0)
		goto out;

	ret = spi_nor_wait_till_ready(nor);

out:
	macronix_set_secured_otp_mode(nor, false);

	return ret;
}

static int macronix_otp_lock(struct spi_nor *nor, unsigned int region)
{
	if (region != 0)
		return -EINVAL;

	return macronix_write_scur(nor);
}

static int macronix_otp_is_locked(struct spi_nor *nor, unsigned int region)
{
	u8 *scur = nor->bouncebuf;
	int ret;

	if (region != 0)
		return -EINVAL;

	ret = macronix_read_scur(nor, scur);
	if (ret)
		return ret;

	return *scur & SCUR_LDSO;
}

static const struct spi_nor_otp_ops macronix_otp_ops = {
	.read = macronix_otp_read,
	.write = macronix_otp_write,
	.lock = macronix_otp_lock,
	.is_locked = macronix_otp_is_locked,
};

static void macronix_default_init(struct spi_nor *nor)
{
	nor->params->quad_enable = spi_nor_sr1_bit6_quad_enable;
	nor->params->set_4byte_addr_mode = spi_nor_set_4byte_addr_mode;

	if (nor->params->otp_info.n_otps)
	    nor->params->otp_ops = &macronix_otp_ops;
}

static const struct spi_nor_fixups macronix_fixups = {
	.default_init = macronix_default_init,
};

const struct spi_nor_manufacturer spi_nor_macronix = {
	.name = "macronix",
	.parts = macronix_parts,
	.nparts = ARRAY_SIZE(macronix_parts),
	.fixups = &macronix_fixups,
};
