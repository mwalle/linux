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
#include <drm/drm_modes.h>

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

u32 cdns_mhdp_get_bpp(struct cdns_mhdp_display_fmt *fmt)
{
	u32 bpp;

	if (fmt->y_only)
		return fmt->bpc;

	switch (fmt->color_format) {
	case DRM_COLOR_FORMAT_RGB444:
	case DRM_COLOR_FORMAT_YCRCB444:
		bpp = fmt->bpc * 3;
		break;
	case DRM_COLOR_FORMAT_YCRCB422:
		bpp = fmt->bpc * 2;
		break;
	case DRM_COLOR_FORMAT_YCRCB420:
		bpp = fmt->bpc * 3 / 2;
		break;
	default:
		bpp = fmt->bpc * 3;
		WARN_ON(1);
	}
	return bpp;
}
EXPORT_SYMBOL_GPL(cdns_mhdp_get_bpp);

void cdns_mhdp_configure_video(struct cdns_mhdp_mbox *mbox, int stream_id,
			       struct cdns_mhdp_display_fmt *display_fmt,
			       const struct drm_display_mode *mode)
{
	unsigned int dp_framer_sp = 0, msa_horizontal_1,
		msa_vertical_1, bnd_hsync2vsync, hsync2vsync_pol_ctrl,
		misc0 = 0, misc1 = 0, pxl_repr,
		front_porch, back_porch, msa_h0, msa_v0, hsync, vsync,
		dp_vertical_1;
	u32 bpp, bpc, pxlfmt, framer;
	int ret;

	pxlfmt = display_fmt->color_format;
	bpc = display_fmt->bpc;

	/*
	 * If YCBCR supported and stream not SD, use ITU709
	 * Need to handle ITU version with YCBCR420 when supported
	 */
	if ((pxlfmt == DRM_COLOR_FORMAT_YCRCB444 ||
	     pxlfmt == DRM_COLOR_FORMAT_YCRCB422) && mode->crtc_vdisplay >= 720)
		misc0 = DP_YCBCR_COEFFICIENTS_ITU709;

	bpp = cdns_mhdp_get_bpp(display_fmt);

	switch (pxlfmt) {
	case DRM_COLOR_FORMAT_RGB444:
		pxl_repr = CDNS_DP_FRAMER_RGB << CDNS_DP_FRAMER_PXL_FORMAT;
		misc0 |= DP_COLOR_FORMAT_RGB;
		break;
	case DRM_COLOR_FORMAT_YCRCB444:
		pxl_repr = CDNS_DP_FRAMER_YCBCR444 << CDNS_DP_FRAMER_PXL_FORMAT;
		misc0 |= DP_COLOR_FORMAT_YCbCr444 | DP_TEST_DYNAMIC_RANGE_CEA;
		break;
	case DRM_COLOR_FORMAT_YCRCB422:
		pxl_repr = CDNS_DP_FRAMER_YCBCR422 << CDNS_DP_FRAMER_PXL_FORMAT;
		misc0 |= DP_COLOR_FORMAT_YCbCr422 | DP_TEST_DYNAMIC_RANGE_CEA;
		break;
	case DRM_COLOR_FORMAT_YCRCB420:
		pxl_repr = CDNS_DP_FRAMER_YCBCR420 << CDNS_DP_FRAMER_PXL_FORMAT;
		break;
	default:
		pxl_repr = CDNS_DP_FRAMER_Y_ONLY << CDNS_DP_FRAMER_PXL_FORMAT;
	}

	switch (bpc) {
	case 6:
		misc0 |= DP_TEST_BIT_DEPTH_6;
		pxl_repr |= CDNS_DP_FRAMER_6_BPC;
		break;
	case 8:
		misc0 |= DP_TEST_BIT_DEPTH_8;
		pxl_repr |= CDNS_DP_FRAMER_8_BPC;
		break;
	case 10:
		misc0 |= DP_TEST_BIT_DEPTH_10;
		pxl_repr |= CDNS_DP_FRAMER_10_BPC;
		break;
	case 12:
		misc0 |= DP_TEST_BIT_DEPTH_12;
		pxl_repr |= CDNS_DP_FRAMER_12_BPC;
		break;
	case 16:
		misc0 |= DP_TEST_BIT_DEPTH_16;
		pxl_repr |= CDNS_DP_FRAMER_16_BPC;
		break;
	}

	bnd_hsync2vsync = CDNS_IP_BYPASS_V_INTERFACE;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		bnd_hsync2vsync |= CDNS_IP_DET_INTERLACE_FORMAT;
	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_BND_HSYNC2VSYNC(0)
					    : CDNS_BND_HSYNC2VSYNC(stream_id),
			    bnd_hsync2vsync);

	hsync2vsync_pol_ctrl = 0;
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		hsync2vsync_pol_ctrl |= CDNS_H2V_HSYNC_POL_ACTIVE_LOW;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		hsync2vsync_pol_ctrl |= CDNS_H2V_VSYNC_POL_ACTIVE_LOW;
	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_HSYNC2VSYNC_POL_CTRL(0)
					    : CDNS_HSYNC2VSYNC_POL_CTRL(stream_id),
			    hsync2vsync_pol_ctrl);

	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_FRAMER_PXL_REPR
					    : CDNS_DP_MST_FRAMER_PXL_REPR(stream_id),
			    pxl_repr);

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		dp_framer_sp |= CDNS_DP_FRAMER_INTERLACE;
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		dp_framer_sp |= CDNS_DP_FRAMER_HSYNC_POL_LOW;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		dp_framer_sp |= CDNS_DP_FRAMER_VSYNC_POL_LOW;
	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_FRAMER_SP
					    : CDNS_DP_MST_FRAMER_SP(stream_id),
			    dp_framer_sp);

	front_porch = mode->crtc_hsync_start - mode->crtc_hdisplay;
	back_porch = mode->crtc_htotal - mode->crtc_hsync_end;
	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_FRONT_BACK_PORCH
					    : CDNS_DP_MST_FRONT_BACK_PORCH(stream_id),
			    CDNS_DP_FRONT_PORCH(front_porch) |
			    CDNS_DP_BACK_PORCH(back_porch));

	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_BYTE_COUNT
					    : CDNS_DP_MST_BYTE_COUNT(stream_id),
			    mode->crtc_hdisplay * bpp / 8);

	msa_h0 = mode->crtc_htotal - mode->crtc_hsync_start;
	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_MSA_HORIZONTAL_0
					    : CDNS_DP_MST_MSA_HORIZONTAL_0(stream_id),
			    CDNS_DP_MSAH0_H_TOTAL(mode->crtc_htotal) |
			    CDNS_DP_MSAH0_HSYNC_START(msa_h0));

	hsync = mode->crtc_hsync_end - mode->crtc_hsync_start;
	msa_horizontal_1 = CDNS_DP_MSAH1_HSYNC_WIDTH(hsync) |
			   CDNS_DP_MSAH1_HDISP_WIDTH(mode->crtc_hdisplay);
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		msa_horizontal_1 |= CDNS_DP_MSAH1_HSYNC_POL_LOW;
	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_MSA_HORIZONTAL_1
					    : CDNS_DP_MST_MSA_HORIZONTAL_1(stream_id),
			    msa_horizontal_1);

	msa_v0 = mode->crtc_vtotal - mode->crtc_vsync_start;
	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_MSA_VERTICAL_0
					    : CDNS_DP_MST_MSA_VERTICAL_0(stream_id),
			    CDNS_DP_MSAV0_V_TOTAL(mode->crtc_vtotal) |
			    CDNS_DP_MSAV0_VSYNC_START(msa_v0));

	vsync = mode->crtc_vsync_end - mode->crtc_vsync_start;
	msa_vertical_1 = CDNS_DP_MSAV1_VSYNC_WIDTH(vsync) |
			 CDNS_DP_MSAV1_VDISP_WIDTH(mode->crtc_vdisplay);
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		msa_vertical_1 |= CDNS_DP_MSAV1_VSYNC_POL_LOW;
	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_MSA_VERTICAL_1
					    : CDNS_DP_MST_MSA_VERTICAL_1(stream_id),
			    msa_vertical_1);

	if ((mode->flags & DRM_MODE_FLAG_INTERLACE) &&
	    mode->crtc_vtotal % 2 == 0)
		misc1 = DP_TEST_INTERLACED;
	if (display_fmt->y_only)
		misc1 |= CDNS_DP_TEST_COLOR_FORMAT_RAW_Y_ONLY;
	/* Use VSC SDP for Y420 */
	if (pxlfmt == DRM_COLOR_FORMAT_YCRCB420)
		misc1 = CDNS_DP_TEST_VSC_SDP;

	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_MSA_MISC
					    : CDNS_DP_MST_MSA_MISC(stream_id),
			    misc0 | (misc1 << 8));

	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_HORIZONTAL
					    : CDNS_DP_MST_HORIZONTAL(stream_id),
			    CDNS_DP_H_HSYNC_WIDTH(hsync) |
			    CDNS_DP_H_H_TOTAL(mode->crtc_hdisplay));

	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_VERTICAL_0
					    : CDNS_DP_MST_VERTICAL_0(stream_id),
			    CDNS_DP_V0_VHEIGHT(mode->crtc_vdisplay) |
			    CDNS_DP_V0_VSTART(msa_v0));

	dp_vertical_1 = CDNS_DP_V1_VTOTAL(mode->crtc_vtotal);
	if ((mode->flags & DRM_MODE_FLAG_INTERLACE) &&
	    mode->crtc_vtotal % 2 == 0)
		dp_vertical_1 |= CDNS_DP_V1_VTOTAL_EVEN;

	cdns_mhdp_reg_write(mbox,
			    (stream_id < 0) ? CDNS_DP_VERTICAL_1
					    : CDNS_DP_MST_VERTICAL_1(stream_id),
			    dp_vertical_1);

	cdns_mhdp_reg_write_bit(mbox,
				(stream_id < 0) ? CDNS_DP_VB_ID
						: CDNS_DP_MST_VB_ID(stream_id),
				2, 1, (mode->flags & DRM_MODE_FLAG_INTERLACE) ?
				CDNS_DP_VB_ID_INTERLACED : 0);

	ret = cdns_mhdp_reg_read(mbox, CDNS_DP_FRAMER_GLOBAL_CONFIG,
				 &framer);
	if (ret < 0) {
		dev_err(mbox->dev,
			"Failed to read CDNS_DP_FRAMER_GLOBAL_CONFIG %d\n",
			ret);
		return;
	}
	framer |= CDNS_DP_FRAMER_EN;
	framer &= ~CDNS_DP_NO_VIDEO_MODE;
	cdns_mhdp_reg_write(mbox, CDNS_DP_FRAMER_GLOBAL_CONFIG, framer);
}
EXPORT_SYMBOL_GPL(cdns_mhdp_configure_video);
