/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cadence MHDP8501 DP bridge driver.
 *
 * Copyright (C) 2020 Cadence Design Systems, Inc.
 *
 * Author: Quentin Schulz <quentin.schulz@free-electrons.com>
 *         Swapnil Jakhade <sjakhade@cadence.com>
 */

#ifndef CDNS_MHDP8501_CORE_H
#define CDNS_MHDP8501_CORE_H

#include <linux/bits.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_dp_helper.h>

#include "cdns-mhdp-common.h"

struct clk;
struct device;
struct phy;

/* REMOVE ME */
enum vic_pxl_encoding_format {
	PXL_RGB = 0x1,
	YCBCR_4_4_4 = 0x2,
	YCBCR_4_2_2 = 0x4,
	YCBCR_4_2_0 = 0x8,
	Y_ONLY = 0x10,
};

struct video_info {
	bool h_sync_polarity;
	bool v_sync_polarity;
	bool interlaced;
	int color_depth;
	enum vic_pxl_encoding_format color_fmt;
};

struct cdns_mhdp_sink {
	unsigned int link_rate;
	u8 lanes_cnt;
	u8 pattern_supp;
	bool fast_link;
	bool enhanced;
	bool ssc;
};

/*
 * These enums present MHDP hw initialization state
 * Legal state transitions are:
 * MHDP_HW_READY <-> MHDP_HW_STOPPED
 */
enum mhdp_hw_state {
	MHDP_HW_READY = 1,	/* HW ready, FW active */
	MHDP_HW_STOPPED		/* Driver removal FW to be stopped */
};

struct cdns_mhdp8501_device;

struct cdns_mhdp8501_platform_info {
	const char *fwname;
	u8 lane_mapping;
};

struct cdns_mhdp8501_device {
	void __iomem *regs;

	struct device *dev;
	struct clk *clk;
	struct phy *phy;
	struct reset_control *phy_reset;

	struct cdns_mhdp_mbox mbox;

	const struct cdns_mhdp8501_platform_info *info;

	/*
	 * "link_mutex" protects the access to all the link parameters
	 * including the link training process. Link training will be
	 * invoked both from threaded interrupt handler and from atomic
	 * callbacks when link_up is not set. So this mutex protects
	 * flags such as link_up, bridge_enabled, link.num_lanes,
	 * link.rate etc.
	 */
	struct mutex link_mutex;

	struct drm_encoder encoder;
	struct drm_connector *connector;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;

	struct cdns_mhdp_link link;
	struct drm_dp_aux aux;

	struct cdns_mhdp_host host;
	struct cdns_mhdp_display_fmt display_fmt;
	u8 stream_id;

	/*
	 * "start_lock" protects the access to bridge_attached and
	 * hw_state data members that control the delayed firmware
	 * loading and attaching the bridge. They are accessed from
	 * both the DRM core and cdns_mhdp8501_fw_cb(). In most cases just
	 * protecting the data members is enough, but the irq mask
	 * setting needs to be protected when enabling the FW.
	 */
	spinlock_t start_lock;
	bool bridge_attached;
	bool bridge_enabled;
	enum mhdp_hw_state hw_state;
	wait_queue_head_t fw_load_wq;

	/* Work struct to schedule a uevent on link train failure */
	struct work_struct modeset_retry_work;
	struct work_struct hpd_work;

	wait_queue_head_t sw_events_wq;
	u32 sw_events;

	bool hdcp_supported;

	/* REMOVE ME */
	struct video_info video_info;
	struct drm_display_mode mode;
#if 0
	bool power_up;
	bool force_mode_set;
	bool is_hpd;
	struct mutex lock;
	struct delayed_work hotplug_work;
#endif
};

int cdns_mhdp8501_encoder_init(struct platform_device *pdev);
#define bridge_to_mhdp8501(x) container_of(x, struct cdns_mhdp8501_device, bridge)

#define TU_CNT_RST_EN				BIT(15)
//#define VIF_BYPASS_INTERLACE			BIT(13)
#define INTERLACE_FMT_DET			BIT(12)
#define INTERLACE_DTCT_WIN			0x20

#define DP_FRAMER_SP_INTERLACE_EN		BIT(2)
#define DP_FRAMER_SP_HSP			BIT(1)
#define DP_FRAMER_SP_VSP			BIT(0)

#define TU_SIZE					30
#define CDNS_DP_MAX_LINK_RATE	540000

/* TODO(mw) */
#define FW_NAME				"cadence/mhdp8501.bin"
#define FW_NAME_LS1028A			"cadence/mhdp8501-ls1028a.bin"
#define CDNS_MHDP8501_IMEM		0x10000

enum vic_color_depth {
	BCS_6 = 0x1,
	BCS_8 = 0x2,
	BCS_10 = 0x4,
	BCS_12 = 0x8,
	BCS_16 = 0x10,
};

enum vic_bt_type {
	BT_601 = 0x0,
	BT_709 = 0x1,
};

/* source vif addr */
//#define BND_HSYNC2VSYNC			0x0b00
#define HSYNC2VSYNC_F1_L1		0x0b04
#define HSYNC2VSYNC_F2_L1		0x0b08
#define HSYNC2VSYNC_STATUS		0x0b0c
//#define HSYNC2VSYNC_POL_CTRL		0x0b10

/* dpyx framer addr */
#define DP_FRAMER_GLOBAL_CONFIG		0x2200
#define DP_SW_RESET			0x2204
#define DP_FRAMER_TU			0x2208
#define DP_FRAMER_PXL_REPR		0x220c
#define DP_FRAMER_SP			0x2210
#define AUDIO_PACK_CONTROL		0x2214
#define DP_VC_TABLE(x)			(0x2218 + ((x) << 2))
#define DP_VB_ID			0x2258
#define DP_MTPH_LVP_CONTROL		0x225c
#define DP_MTPH_SYMBOL_VALUES		0x2260
#define DP_MTPH_ECF_CONTROL		0x2264
#define DP_MTPH_ACT_CONTROL		0x2268
#define DP_MTPH_STATUS			0x226c
#define DP_INTERRUPT_SOURCE		0x2270
#define DP_INTERRUPT_MASK		0x2274
#define DP_FRONT_BACK_PORCH		0x2278
#define DP_BYTE_COUNT			0x227c

/* dptx stream addr */
#define MSA_HORIZONTAL_0		0x2280
#define MSA_HORIZONTAL_1		0x2284
#define MSA_VERTICAL_0			0x2288
#define MSA_VERTICAL_1			0x228c
#define MSA_MISC			0x2290
#define STREAM_CONFIG			0x2294
#define AUDIO_PACK_STATUS		0x2298
#define VIF_STATUS			0x229c
#define PCK_STUFF_STATUS_0		0x22a0
#define PCK_STUFF_STATUS_1		0x22a4
#define INFO_PACK_STATUS		0x22a8
#define RATE_GOVERNOR_STATUS		0x22ac
#define DP_HORIZONTAL			0x22b0
#define DP_VERTICAL_0			0x22b4
#define DP_VERTICAL_1			0x22b8
#define DP_BLOCK_SDP			0x22bc

//#define ADDR_PHY_AFE	0x80000

/* source car addr */
#define SOURCE_HDTX_CAR			0x0900
#define SOURCE_DPTX_CAR			0x0904
#define SOURCE_PHY_CAR			0x0908
#define SOURCE_CEC_CAR			0x090c
#define SOURCE_CBUS_CAR			0x0910
#define SOURCE_PKT_CAR			0x0918
#define SOURCE_AIF_CAR			0x091c
#define SOURCE_CIPHER_CAR		0x0920
#define SOURCE_CRYPTO_CAR		0x0924

#define LANES_CONFIG			0x0814

#endif
