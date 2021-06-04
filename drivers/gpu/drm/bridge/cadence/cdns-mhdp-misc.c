// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence MHDP DP bridge common DP link functions.
 *
 * Copyright (C) 2020 Cadence Design Systems, Inc.
 *
 * Authors: Quentin Schulz <quentin.schulz@free-electrons.com>
 *          Swapnil Jakhade <sjakhade@cadence.com>
 *          Yuti Amonkar <yamonkar@cadence.com>
 *          Tomi Valkeinen <tomi.valkeinen@ti.com>
 *          Jyri Sarha <jsarha@ti.com>
 */

#include <drm/drm_dp_helper.h>

#include "cdns-mhdp-common.h"

/**
 * cdns_mhdp_link_power_up() - power up a DisplayPort link
 * @aux: DisplayPort AUX channel
 * @link: pointer to a structure containing the link configuration
 *
 * Returns 0 on success or a negative error code on failure.
 */
int cdns_mhdp_link_power_up(struct drm_dp_aux *aux, struct cdns_mhdp_link *link)
{
	u8 value;
	int err;

	/* DP_SET_POWER register is only available on DPCD v1.1 and later */
	if (link->revision < 0x11)
		return 0;

	err = drm_dp_dpcd_readb(aux, DP_SET_POWER, &value);
	if (err < 0)
		return err;

	value &= ~DP_SET_POWER_MASK;
	value |= DP_SET_POWER_D0;

	err = drm_dp_dpcd_writeb(aux, DP_SET_POWER, value);
	if (err < 0)
		return err;

	/*
	 * According to the DP 1.1 specification, a "Sink Device must exit the
	 * power saving state within 1 ms" (Section 2.5.3.1, Table 5-52, "Sink
	 * Control Field" (register 0x600).
	 */
	usleep_range(1000, 2000);

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_link_power_up);

/**
 * cdns_mhdp_link_power_down() - power down a DisplayPort link
 * @aux: DisplayPort AUX channel
 * @link: pointer to a structure containing the link configuration
 *
 * Returns 0 on success or a negative error code on failure.
 */
int cdns_mhdp_link_power_down(struct drm_dp_aux *aux,
			      struct cdns_mhdp_link *link)
{
	u8 value;
	int err;

	/* DP_SET_POWER register is only available on DPCD v1.1 and later */
	if (link->revision < 0x11)
		return 0;

	err = drm_dp_dpcd_readb(aux, DP_SET_POWER, &value);
	if (err < 0)
		return err;

	value &= ~DP_SET_POWER_MASK;
	value |= DP_SET_POWER_D3;

	err = drm_dp_dpcd_writeb(aux, DP_SET_POWER, value);
	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_link_power_down);

/**
 * cdns_mhdp_link_configure() - configure a DisplayPort link
 * @aux: DisplayPort AUX channel
 * @link: pointer to a structure containing the link configuration
 *
 * Returns 0 on success or a negative error code on failure.
 */
int cdns_mhdp_link_configure(struct drm_dp_aux *aux,
			     struct cdns_mhdp_link *link)
{
	u8 values[2];
	int err;

	values[0] = drm_dp_link_rate_to_bw_code(link->rate);
	values[1] = link->num_lanes;

	if (link->capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		values[1] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;

	err = drm_dp_dpcd_write(aux, DP_LINK_BW_SET, values, sizeof(values));
	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_link_configure);

ssize_t cdns_mhdp_aux_transfer(struct cdns_mhdp_mbox *mbox,
			       struct drm_dp_aux *aux,
			       struct drm_dp_aux_msg *msg)
{
	int ret;

	if (msg->request != DP_AUX_NATIVE_WRITE &&
	    msg->request != DP_AUX_NATIVE_READ)
		return -EOPNOTSUPP;

	if (msg->request == DP_AUX_NATIVE_WRITE) {
		const u8 *buf = msg->buffer;
		unsigned int i;

		for (i = 0; i < msg->size; ++i) {
			ret = cdns_mhdp_dpcd_write(mbox,
						   msg->address + i, buf[i]);
			if (!ret)
				continue;

			dev_err(mbox->dev,
				"Failed to write DPCD addr %u\n",
				msg->address + i);

			return ret;
		}
	} else {
		ret = cdns_mhdp_dpcd_read(mbox, msg->address,
					  msg->buffer, msg->size);
		if (ret) {
			dev_err(mbox->dev,
				"Failed to read DPCD addr %u\n",
				msg->address);

			return ret;
		}
	}

	return msg->size;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_aux_transfer);
