// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence MHDP DP bridge common mailbox functions.
 *
 * Copyright (C) 2020 Cadence Design Systems, Inc.
 *
 * Authors: Quentin Schulz <quentin.schulz@free-electrons.com>
 *          Swapnil Jakhade <sjakhade@cadence.com>
 *          Yuti Amonkar <yamonkar@cadence.com>
 *          Tomi Valkeinen <tomi.valkeinen@ti.com>
 *          Jyri Sarha <jsarha@ti.com>
 */

#include <linux/mutex.h>
#include <linux/io.h>
#include <asm/unaligned.h>
#include <linux/iopoll.h>

#include "cdns-mhdp-common.h"

static int cdns_mhdp_mailbox_read(struct cdns_mhdp_mbox *mbox)
{
	int ret, empty;

	WARN_ON(!mutex_is_locked(&mbox->mutex));

	ret = readx_poll_timeout(readl, mbox->regs + CDNS_MAILBOX_EMPTY,
				 empty, !empty, MAILBOX_RETRY_US,
				 MAILBOX_TIMEOUT_US);
	if (ret < 0)
		return ret;

	return readl(mbox->regs + CDNS_MAILBOX_RX_DATA) & 0xff;
}

static int cdns_mhdp_mailbox_write(struct cdns_mhdp_mbox *mbox, u8 val)
{
	int ret, full;

	WARN_ON(!mutex_is_locked(&mbox->mutex));

	ret = readx_poll_timeout(readl, mbox->regs + CDNS_MAILBOX_FULL,
				 full, !full, MAILBOX_RETRY_US,
				 MAILBOX_TIMEOUT_US);
	if (ret < 0)
		return ret;

	writel(val, mbox->regs + CDNS_MAILBOX_TX_DATA);

	return 0;
}

static int cdns_mhdp_mailbox_recv_header(struct cdns_mhdp_mbox *mbox,
					 u8 module_id, u8 opcode, u16 req_size)
{
	u32 mbox_size, i;
	u8 header[4];
	int ret;

	/* read the header of the message */
	for (i = 0; i < sizeof(header); i++) {
		ret = cdns_mhdp_mailbox_read(mbox);
		if (ret < 0)
			return ret;

		header[i] = ret;
	}

	mbox_size = get_unaligned_be16(header + 2);

	if (opcode != header[0] || module_id != header[1] ||
	    req_size != mbox_size) {
		/*
		 * If the message in mailbox is not what we want, we need to
		 * clear the mailbox by reading its contents.
		 */
		for (i = 0; i < mbox_size; i++)
			if (cdns_mhdp_mailbox_read(mbox) < 0)
				break;

		return -EINVAL;
	}

	return 0;
}

static int cdns_mhdp_mailbox_recv_data(struct cdns_mhdp_mbox *mbox,
				       u8 *buff, u16 buff_size)
{
	u32 i;
	int ret;

	for (i = 0; i < buff_size; i++) {
		ret = cdns_mhdp_mailbox_read(mbox);
		if (ret < 0)
			return ret;

		buff[i] = ret;
	}

	return 0;
}

static int cdns_mhdp_mailbox_send(struct cdns_mhdp_mbox *mbox,
				  u8 module_id, u8 opcode, u16 size,
				  u8 *message)
{
	u8 header[4];
	int ret, i;

	header[0] = opcode;
	header[1] = module_id;
	put_unaligned_be16(size, header + 2);

	for (i = 0; i < sizeof(header); i++) {
		ret = cdns_mhdp_mailbox_write(mbox, header[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < size; i++) {
		ret = cdns_mhdp_mailbox_write(mbox, message[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int cdns_mhdp_reg_read(struct cdns_mhdp_mbox *mbox, u32 addr, u32 *value)
{
	u8 msg[4], resp[8];
	int ret;

	put_unaligned_be32(addr, msg);

	mutex_lock(&mbox->mutex);

	ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_GENERAL,
				     GENERAL_REGISTER_READ,
				     sizeof(msg), msg);
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_header(mbox, MB_MODULE_ID_GENERAL,
					    GENERAL_REGISTER_READ,
					    sizeof(resp));
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_data(mbox, resp, sizeof(resp));
	if (ret)
		goto out;

	/* Returned address value should be the same as requested */
	if (memcmp(msg, resp, sizeof(msg))) {
		ret = -EINVAL;
		goto out;
	}

	*value = get_unaligned_be32(resp + 4);

out:
	mutex_unlock(&mbox->mutex);
	if (ret) {
		dev_err(mbox->dev, "Failed to read register\n");
		*value = 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_reg_read);

int cdns_mhdp_reg_write(struct cdns_mhdp_mbox *mbox, u32 addr, u32 val)
{
	u8 msg[8];
	int ret;

	put_unaligned_be32(addr, msg);
	put_unaligned_be32(val, msg + 4);

	mutex_lock(&mbox->mutex);

	ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_GENERAL,
				     GENERAL_REGISTER_WRITE, sizeof(msg), msg);

	mutex_unlock(&mbox->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_reg_write);

int cdns_mhdp_reg_write_bit(struct cdns_mhdp_mbox *mbox,
			    u16 addr, u8 start_bit, u8 bits_no, u32 val)
{
	u8 field[8];
	int ret;

	put_unaligned_be16(addr, field);
	field[2] = start_bit;
	field[3] = bits_no;
	put_unaligned_be32(val, field + 4);

	mutex_lock(&mbox->mutex);

	ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_DP_TX,
				     DPTX_WRITE_FIELD, sizeof(field), field);

	mutex_unlock(&mbox->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_reg_write_bit);

int cdns_mhdp_dpcd_read(struct cdns_mhdp_mbox *mbox,
			u32 addr, u8 *data, u16 len)
{
	u8 msg[5], reg[5];
	int ret;

	put_unaligned_be16(len, msg);
	put_unaligned_be24(addr, msg + 2);

	mutex_lock(&mbox->mutex);

	ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_DP_TX,
				     DPTX_READ_DPCD, sizeof(msg), msg);
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_header(mbox, MB_MODULE_ID_DP_TX,
					    DPTX_READ_DPCD,
					    sizeof(reg) + len);
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_data(mbox, reg, sizeof(reg));
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_data(mbox, data, len);

out:
	mutex_unlock(&mbox->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_dpcd_read);

int cdns_mhdp_dpcd_write(struct cdns_mhdp_mbox *mbox, u32 addr, u8 value)
{
	u8 msg[6], reg[5];
	int ret;

	put_unaligned_be16(1, msg);
	put_unaligned_be24(addr, msg + 2);
	msg[5] = value;

	mutex_lock(&mbox->mutex);

	ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_DP_TX,
				     DPTX_WRITE_DPCD, sizeof(msg), msg);
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_header(mbox, MB_MODULE_ID_DP_TX,
					    DPTX_WRITE_DPCD, sizeof(reg));
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_data(mbox, reg, sizeof(reg));
	if (ret)
		goto out;

	if (addr != get_unaligned_be24(reg + 2))
		ret = -EINVAL;

out:
	mutex_unlock(&mbox->mutex);

	if (ret)
		dev_err(mbox->dev, "dpcd write failed: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_dpcd_write);

int cdns_mhdp_set_firmware_active(struct cdns_mhdp_mbox *mbox, bool enable)
{
	u8 msg[5];
	int ret, i;

	msg[0] = GENERAL_MAIN_CONTROL;
	msg[1] = MB_MODULE_ID_GENERAL;
	msg[2] = 0;
	msg[3] = 1;
	msg[4] = enable ? FW_ACTIVE : FW_STANDBY;

	mutex_lock(&mbox->mutex);

	for (i = 0; i < sizeof(msg); i++) {
		ret = cdns_mhdp_mailbox_write(mbox, msg[i]);
		if (ret)
			goto out;
	}

	/* read the firmware state */
	ret = cdns_mhdp_mailbox_recv_data(mbox, msg, sizeof(msg));
	if (ret)
		goto out;

	ret = 0;

out:
	mutex_unlock(&mbox->mutex);

	if (ret < 0)
		dev_err(mbox->dev, "set firmware active failed\n");
	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_set_firmware_active);

int cdns_mhdp_get_edid_block(void *data, u8 *edid, unsigned int block,
			     size_t length)
{
	struct cdns_mhdp_mbox *mbox = data;
	u8 msg[2], reg[2], i;
	int ret;

	mutex_lock(&mbox->mutex);

	for (i = 0; i < 4; i++) {
		msg[0] = block / 2;
		msg[1] = block % 2;

		ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_DP_TX,
					     DPTX_GET_EDID, sizeof(msg), msg);
		if (ret)
			continue;

		ret = cdns_mhdp_mailbox_recv_header(mbox, MB_MODULE_ID_DP_TX,
						    DPTX_GET_EDID,
						    sizeof(reg) + length);
		if (ret)
			continue;

		ret = cdns_mhdp_mailbox_recv_data(mbox, reg, sizeof(reg));
		if (ret)
			continue;

		ret = cdns_mhdp_mailbox_recv_data(mbox, edid, length);
		if (ret)
			continue;

		if (reg[0] == length && reg[1] == block / 2)
			break;
	}

	mutex_unlock(&mbox->mutex);

	if (ret)
		dev_err(mbox->dev, "get block[%d] edid failed: %d\n",
			block, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_get_edid_block);

int cdns_mhdp_get_hpd_status(struct cdns_mhdp_mbox *mbox)
{
	u8 status;
	int ret;

	mutex_lock(&mbox->mutex);

	ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_DP_TX,
				     DPTX_HPD_STATE, 0, NULL);
	if (ret)
		goto err_get_hpd;

	ret = cdns_mhdp_mailbox_recv_header(mbox, MB_MODULE_ID_DP_TX,
					    DPTX_HPD_STATE,
					    sizeof(status));
	if (ret)
		goto err_get_hpd;

	ret = cdns_mhdp_mailbox_recv_data(mbox, &status, sizeof(status));
	if (ret)
		goto err_get_hpd;

	mutex_unlock(&mbox->mutex);

	dev_dbg(mbox->dev, "%s: HPD %splugged\n", __func__,
		status ? "" : "un");

	return status;

err_get_hpd:
	mutex_unlock(&mbox->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_get_hpd_status);

int cdns_mhdp_read_hpd_event(struct cdns_mhdp_mbox *mbox)
{
	u8 event = 0;
	int ret;

	mutex_lock(&mbox->mutex);

	ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_DP_TX,
				     DPTX_READ_EVENT, 0, NULL);
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_header(mbox, MB_MODULE_ID_DP_TX,
					    DPTX_READ_EVENT, sizeof(event));
	if (ret < 0)
		goto out;

	ret = cdns_mhdp_mailbox_recv_data(mbox, &event, sizeof(event));
out:
	mutex_unlock(&mbox->mutex);

	if (ret < 0)
		return ret;

	dev_dbg(mbox->dev, "%s: %s%s%s%s\n", __func__,
		(event & DPTX_READ_EVENT_HPD_TO_HIGH) ? "TO_HIGH " : "",
		(event & DPTX_READ_EVENT_HPD_TO_LOW) ? "TO_LOW " : "",
		(event & DPTX_READ_EVENT_HPD_PULSE) ? "PULSE " : "",
		(event & DPTX_READ_EVENT_HPD_STATE) ? "HPD_STATE " : "");

	return event;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_read_hpd_event);

int cdns_mhdp_adjust_lt(struct cdns_mhdp_mbox *mbox, unsigned int nlanes,
			unsigned int udelay, const u8 *lanes_data,
			u8 link_status[DP_LINK_STATUS_SIZE])
{
	u8 payload[7];
	u8 hdr[5]; /* For DPCD read response header */
	u32 addr;
	int ret;

	if (nlanes != 4 && nlanes != 2 && nlanes != 1) {
		dev_err(mbox->dev, "invalid number of lanes: %u\n", nlanes);
		ret = -EINVAL;
		goto out;
	}

	payload[0] = nlanes;
	put_unaligned_be16(udelay, payload + 1);
	memcpy(payload + 3, lanes_data, nlanes);

	mutex_lock(&mbox->mutex);

	ret = cdns_mhdp_mailbox_send(mbox, MB_MODULE_ID_DP_TX,
				     DPTX_ADJUST_LT,
				     sizeof(payload), payload);
	if (ret)
		goto out;

	/* Yes, read the DPCD read command response */
	ret = cdns_mhdp_mailbox_recv_header(mbox, MB_MODULE_ID_DP_TX,
					    DPTX_READ_DPCD,
					    sizeof(hdr) + DP_LINK_STATUS_SIZE);
	if (ret)
		goto out;

	ret = cdns_mhdp_mailbox_recv_data(mbox, hdr, sizeof(hdr));
	if (ret)
		goto out;

	addr = get_unaligned_be24(hdr + 2);
	if (addr != DP_LANE0_1_STATUS)
		goto out;

	ret = cdns_mhdp_mailbox_recv_data(mbox, link_status,
					  DP_LINK_STATUS_SIZE);

out:
	mutex_unlock(&mbox->mutex);

	if (ret)
		dev_err(mbox->dev, "Failed to adjust Link Training.\n");

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_adjust_lt);

void cdns_mhdp_mailbox_init(struct cdns_mhdp_mbox *mbox, struct device *dev,
			    void __iomem *regs)
{
	mbox->dev = dev;
	mbox->regs = regs;
	mutex_init(&mbox->mutex);
}
EXPORT_SYMBOL_GPL(cdns_mhdp_mailbox_init);

MODULE_DESCRIPTION("Cadence MHDP DP Bridge Mailbox Driver");
MODULE_LICENSE("GPL");
