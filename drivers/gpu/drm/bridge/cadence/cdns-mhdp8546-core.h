/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cadence MHDP8546 DP bridge driver.
 *
 * Copyright (C) 2020 Cadence Design Systems, Inc.
 *
 * Author: Quentin Schulz <quentin.schulz@free-electrons.com>
 *         Swapnil Jakhade <sjakhade@cadence.com>
 */

#ifndef CDNS_MHDP8546_CORE_H
#define CDNS_MHDP8546_CORE_H

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

/* firmware and opcodes */
#define FW_NAME					"cadence/mhdp8546.bin"
#define CDNS_MHDP_IMEM				0x10000

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

struct cdns_mhdp_device;

struct mhdp_platform_ops {
	int (*init)(struct cdns_mhdp_device *mhdp);
	void (*exit)(struct cdns_mhdp_device *mhdp);
	void (*enable)(struct cdns_mhdp_device *mhdp);
	void (*disable)(struct cdns_mhdp_device *mhdp);
};

struct cdns_mhdp_bridge_state {
	struct drm_bridge_state base;
	struct drm_display_mode *current_mode;
};

struct cdns_mhdp_platform_info {
	const struct drm_bridge_timings *timings;
	const struct mhdp_platform_ops *ops;
};

#define to_cdns_mhdp_bridge_state(s) \
		container_of(s, struct cdns_mhdp_bridge_state, base)

struct cdns_mhdp_hdcp {
	struct delayed_work check_work;
	struct work_struct prop_work;
	struct mutex mutex; /* mutex to protect hdcp.value */
	u32 value;
	u8 hdcp_content_type;
};

struct cdns_mhdp_device {
	void __iomem *regs;
	void __iomem *sapb_regs;
	void __iomem *j721e_regs;

	struct device *dev;
	struct clk *clk;
	struct phy *phy;

	struct cdns_mhdp_mbox mbox;

	const struct cdns_mhdp_platform_info *info;

	/* This is to protect mailbox communications with the firmware */
	struct mutex secure_mbox_mutex;

	/*
	 * "link_mutex" protects the access to all the link parameters
	 * including the link training process. Link training will be
	 * invoked both from threaded interrupt handler and from atomic
	 * callbacks when link_up is not set. So this mutex protects
	 * flags such as link_up, bridge_enabled, link.num_lanes,
	 * link.rate etc.
	 */
	struct mutex link_mutex;

	struct drm_connector connector;
	struct drm_bridge bridge;

	struct cdns_mhdp_link link;
	struct drm_dp_aux aux;

	struct cdns_mhdp_host host;
	struct cdns_mhdp_sink sink;
	struct cdns_mhdp_display_fmt display_fmt;
	u8 stream_id;

	bool link_up;
	bool plugged;

	/*
	 * "start_lock" protects the access to bridge_attached and
	 * hw_state data members that control the delayed firmware
	 * loading and attaching the bridge. They are accessed from
	 * both the DRM core and cdns_mhdp_fw_cb(). In most cases just
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

	struct cdns_mhdp_hdcp hdcp;
	bool hdcp_supported;
};

#define connector_to_mhdp(x) container_of(x, struct cdns_mhdp_device, connector)
#define bridge_to_mhdp(x) container_of(x, struct cdns_mhdp_device, bridge)

u32 cdns_mhdp_wait_for_sw_event(struct cdns_mhdp_device *mhdp, uint32_t event);

#endif
