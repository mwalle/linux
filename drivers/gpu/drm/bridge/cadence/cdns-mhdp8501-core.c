#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <drm/drm_of.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <linux/firmware.h>
#include <linux/reset.h>

#include "cdns-mhdp8501-afe.h"
#include "cdns-mhdp-common.h"

/***************/

#include <linux/iopoll.h>
#include <asm/unaligned.h>
#include "cdns-mhdp8501-core.h"

static ssize_t cdns_mhdp8501_aux_transfer(struct drm_dp_aux *aux,
					  struct drm_dp_aux_msg *msg)
{
	struct cdns_mhdp8501_device *mhdp = dev_get_drvdata(aux->dev);

	return cdns_mhdp_aux_transfer(&mhdp->mbox, aux, msg);
}

static int cdns_mhdp8501_train_link(struct cdns_mhdp8501_device *mhdp)
{
	int ret;

	ret = cdns_mhdp_training_start(&mhdp->mbox);
	if (ret) {
		dev_err(mhdp->dev, "Failed to start training %d\n", ret);
		return ret;
	}

	ret = cdns_mhdp_get_training_status(&mhdp->mbox, &mhdp->link);
	if (ret) {
		dev_err(mhdp->dev, "Failed to get training stat %d\n", ret);
		return ret;
	}

	dev_err(mhdp->dev, "rate:0x%x, lanes:%d\n", mhdp->link.rate,
			  mhdp->link.num_lanes);
	return ret;
}

static void cdns_mhdp8501_sst_enable(struct cdns_mhdp8501_device *mhdp,
				     const struct drm_display_mode *mode)
{
	u32 rate, vs, required_bandwidth, available_bandwidth;
	s32 line_thresh1, line_thresh2, line_thresh = 0;
	int pxlclock = mode->crtc_clock;
	u32 tu_size = 64;
	u32 bpp;

	/* Get rate in MSymbols per second per lane */
	rate = mhdp->link.rate / 1000;

	bpp = cdns_mhdp_get_bpp(&mhdp->display_fmt);

	required_bandwidth = pxlclock * bpp / 8;
	available_bandwidth = mhdp->link.num_lanes * rate;

	vs = tu_size * required_bandwidth / available_bandwidth;
	vs /= 1000;

	if (vs == tu_size)
		vs = tu_size - 1;

	line_thresh1 = ((vs + 1) << 5) * 8 / bpp;
	line_thresh2 = (pxlclock << 5) / 1000 / rate * (vs + 1) - (1 << 5);
	line_thresh = line_thresh1 - line_thresh2 / (s32)mhdp->link.num_lanes;
	line_thresh = (line_thresh >> 5) + 2;

	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_DP_FRAMER_TU,
			    CDNS_DP_FRAMER_TU_VS(vs) |
			    CDNS_DP_FRAMER_TU_SIZE(tu_size) |
			    CDNS_DP_FRAMER_TU_CNT_RST_EN);

	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_DP_LINE_THRESH,
			    line_thresh & GENMASK(5, 0));


	cdns_mhdp_configure_video(&mhdp->mbox, -1, &mhdp->display_fmt, mode);
}

static enum drm_connector_status cdns_mhdp8501_detect(struct drm_bridge *bridge)
{
	struct cdns_mhdp8501_device *mhdp = bridge_to_mhdp8501(bridge);
	int status;

	status = cdns_mhdp_get_hpd_status(&mhdp->mbox);
	if (status < 0) {
		dev_warn(mhdp->dev, "getting HPD status failed (%d)\n",
			 status);
		return connector_status_disconnected;
	}

	return status ? connector_status_connected
		      : connector_status_disconnected;
}

static struct edid *cdns_mhdp8501_get_edid(struct drm_bridge *bridge,
						  struct drm_connector *connector)
{
	struct cdns_mhdp8501_device *mhdp = bridge_to_mhdp8501(bridge);

	return drm_do_get_edid(connector, cdns_mhdp_get_edid_block, &mhdp->mbox);
}

static int cdns_mhdp8501_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags)
{
	struct cdns_mhdp8501_device *mhdp = bridge_to_mhdp8501(bridge);
	int ret;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
		return -EINVAL;

	mhdp->aux.dev = mhdp->dev;
	mhdp->aux.drm_dev = bridge->dev;
	mhdp->aux.transfer = cdns_mhdp8501_aux_transfer;
	ret = drm_dp_aux_register(&mhdp->aux);
	if (ret < 0)
		return ret;

	ret = drm_bridge_attach(bridge->encoder, mhdp->next_bridge, bridge,
				flags);
	if (ret)
		dev_err(mhdp->dev, "failed to attach next bridge\n");

	return ret;
}

static void cdns_mhdp8501_detach(struct drm_bridge *bridge)
{
	struct cdns_mhdp8501_device *mhdp = bridge_to_mhdp8501(bridge);

	drm_dp_aux_unregister(&mhdp->aux);
}


static enum drm_mode_status
cdns_mhdp8501_mode_valid(struct drm_bridge *bridge,
			 const struct drm_display_info *info,
			 const struct drm_display_mode *mode)
{
	enum drm_mode_status mode_status = MODE_OK;

	/* We don't support double-clocked modes */
	if (mode->flags & DRM_MODE_FLAG_DBLCLK ||
			mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_BAD;

	/* MAX support pixel clock rate 594MHz */
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;

	/* 4096x2160 is not supported now */
	if (mode->hdisplay > 1920)
		return MODE_BAD_HVALUE;

	if (mode->vdisplay > 1080)
		return MODE_BAD_VVALUE;

	return mode_status;
}

static void cdns_mhdp8501_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adjusted_mode)
{
	struct cdns_mhdp8501_device *mhdp = bridge_to_mhdp8501(bridge);

	memcpy(&mhdp->mode, adjusted_mode, sizeof(mhdp->mode));
}

static void cdns_mhdp8501_enable(struct drm_bridge *bridge)
{
	struct cdns_mhdp8501_device *mhdp = bridge_to_mhdp8501(bridge);
	struct cdns_mhdp_link *link = &mhdp->link;
	union phy_configure_opts phy_cfg;
	u8 dpcd[DP_RECEIVER_CAP_SIZE];
	u32 resp, interval, interval_us;
	u8 ext_cap_chk = 0;
	unsigned int addr;
	int ret;
	int err;

	//WARN_ON(!mutex_is_locked(&mhdp->link_mutex));

	ret = drm_dp_read_dpcd_caps(&mhdp->aux, dpcd);
	if (ret < 0)
		return ret;

	mhdp->link.revision = dpcd[0];
	mhdp->link.rate = drm_dp_bw_code_to_link_rate(dpcd[1]);
	mhdp->link.num_lanes = dpcd[2] & DP_MAX_LANE_COUNT_MASK;

	if (dpcd[2] & DP_ENHANCED_FRAME_CAP)
		mhdp->link.capabilities |= DP_LINK_CAP_ENHANCED_FRAMING;

	dev_dbg(mhdp->dev, "Set sink device power state via DPCD\n");

	/* check the max link rate */
	if (link->rate > 540000)
		link->rate = 540000;

	cdns_mhdp_link_power_up(&mhdp->aux, link);
	if (ret < 0) {
		printk("Failed to power DP link: %d\n", ret);
		return;
	}

	/* Initialize link rate/num_lanes as panel max link rate/max_num_lanes */
	phy_cfg.dp.lanes = mhdp->link.num_lanes;
	phy_cfg.dp.link_rate = mhdp->link.rate / 100;
	//phy_cfg.dp.ssc = cdns_mhdp8501_get_ssc_supported(mhdp);
	phy_cfg.dp.ssc = false;
	phy_cfg.dp.set_lanes = true;
	phy_cfg.dp.set_rate = true;
	phy_cfg.dp.set_voltages = false;
	ret = phy_configure(mhdp->phy, &phy_cfg);
	if (ret) {
		dev_err(mhdp->dev, "phy config valid %d\n", ret);
		return;
	}

	ret = cdns_mhdp_set_video_status(&mhdp->mbox, CONTROL_VIDEO_IDLE);
	if (ret) {
		dev_err(mhdp->dev, "Failed to valid video %d\n", ret);
		return;
	}

	ret = cdns_mhdp_set_host_cap(&mhdp->mbox, &mhdp->host);
	if (ret) {
		dev_err(mhdp->dev, "Failed to set host cap %d\n", ret);
		return;
	}

	cdns_mhdp8501_sst_enable(mhdp, &mhdp->mode);

	ret = cdns_mhdp8501_train_link(mhdp);
	if (ret) {
		dev_err(mhdp->dev, "Failed link train %d\n", ret);
		return;
	}

	ret = cdns_mhdp_set_video_status(&mhdp->mbox, CONTROL_VIDEO_VALID);
	if (ret) {
		dev_err(mhdp->dev, "Failed to valid video %d\n", ret);
		return;
	}
}

static void cdns_mhdp8501_disable(struct drm_bridge *bridge)
{
	struct cdns_mhdp8501_device *mhdp = bridge_to_mhdp8501(bridge);

	cdns_mhdp_set_video_status(&mhdp->mbox, CONTROL_VIDEO_IDLE);
}

static const struct drm_bridge_funcs cdns_mhdp8501_bridge_funcs = {
	.attach = cdns_mhdp8501_attach,
	.detach = cdns_mhdp8501_detach,
	.enable = cdns_mhdp8501_enable,
	.disable = cdns_mhdp8501_disable,
	.mode_set = cdns_mhdp8501_mode_set,
	.mode_valid = cdns_mhdp8501_mode_valid,
	.detect = cdns_mhdp8501_detect,
	.get_edid = cdns_mhdp8501_get_edid,
};

static void cdns_mhdp8501_fill_host_caps(struct cdns_mhdp8501_device *mhdp)
{
	unsigned int link_rate;

	/* Get source capabilities based on PHY attributes */

	//mhdp->host.lanes_cnt = mhdp->phy->attrs.bus_width;
	mhdp->host.lanes_cnt = 4;
	if (!mhdp->host.lanes_cnt)
		mhdp->host.lanes_cnt = 4;

	//link_rate = mhdp->phy->attrs.max_link_rate;
	link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_8_1);
	if (!link_rate)
		link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_8_1);
	else
		/* PHY uses Mb/s, DRM uses tens of kb/s. */
		link_rate *= 100;

	mhdp->host.link_rate = link_rate;
	mhdp->host.volt_swing = CDNS_VOLT_SWING(3);
	mhdp->host.pre_emphasis = CDNS_PRE_EMPHASIS(3);
	mhdp->host.pattern_supp = CDNS_SUPPORT_TPS(1) | CDNS_SUPPORT_TPS(2) |
				  CDNS_SUPPORT_TPS(3) | CDNS_SUPPORT_TPS(4);
	mhdp->host.lane_mapping = CDNS_LANE_MAPPING_NORMAL;
	mhdp->host.fast_link = false;
	mhdp->host.enhanced = true;
	mhdp->host.scrambler = true;
	mhdp->host.ssc = false;
}

static int cdns_mhdp8501_check_fw_version(struct cdns_mhdp8501_device *mhdp)
{
	u32 major_num, minor_num, revision;
	u32 fw_ver, lib_ver;

	fw_ver = (readl(mhdp->regs + CDNS_VER_H) << 8)
	       | readl(mhdp->regs + CDNS_VER_L);

	lib_ver = (readl(mhdp->regs + CDNS_LIB_H_ADDR) << 8)
		| readl(mhdp->regs + CDNS_LIB_L_ADDR);

	/*
	 * TODO(mw): unverified, but ls1028a commit says:
	 *
	 * mhdp_fw_1_0_60-dptx-hdcp-mcu2 which reports fw_ver=32955 lib_ver=20691
	 */
	if (lib_ver < 33984) {
		/*
		 * Older FW versions with major number 1, used to store FW
		 * version information by storing repository revision number
		 * in registers. This is for identifying these FW versions.
		 */
		major_num = 1;
		minor_num = 0;
		if (fw_ver == 32955) {
			revision = 60;
		} else {
			dev_err(mhdp->dev, "Unsupported FW version: fw_ver = %u, lib_ver = %u\n",
				fw_ver, lib_ver);
			return -ENODEV;
		}
	} else {
		/* To identify newer FW versions with major number 2 onwards. */
		major_num = fw_ver / 10000;
		minor_num = (fw_ver / 100) % 100;
		revision = (fw_ver % 10000) % 100;
	}

	dev_dbg(mhdp->dev, "FW version: v%u.%u.%u\n", major_num, minor_num,
		revision);
	return 0;
}

static int cdns_mhdp8501_fw_activate(const struct firmware *fw,
				 struct cdns_mhdp8501_device *mhdp)
{
	/* TODO(mw): unverified */
	unsigned int reg;
	int ret;

	writel(CDNS_XT_RESET | CDNS_DRAM_PATH | CDNS_IRAM_PATH,
	       mhdp->regs + CDNS_APB_CTRL);

	memcpy_toio(mhdp->regs + CDNS_MHDP8501_IMEM, fw->data, fw->size);

	/* Leave debug mode, release stall */
	writel(0, mhdp->regs + CDNS_APB_CTRL);

	/*
	 * Wait for the KEEP_ALIVE "message" on the first 8 bits.
	 * Updated each sched "tick" (~2ms)
	 */
	ret = readl_poll_timeout(mhdp->regs + CDNS_KEEP_ALIVE, reg,
				 reg & CDNS_KEEP_ALIVE_MASK, 500,
				 CDNS_KEEP_ALIVE_TIMEOUT);
	if (ret) {
		dev_err(mhdp->dev,
			"device didn't give any life sign: reg %d\n", reg);
		return ret;
	}

	ret = cdns_mhdp8501_check_fw_version(mhdp);
	if (ret)
		return ret;

	/* Init events to 0 as it's not cleared by FW at boot but on read */
	readl(mhdp->regs + CDNS_SW_EVENT0);
	readl(mhdp->regs + CDNS_SW_EVENT1);
	readl(mhdp->regs + CDNS_SW_EVENT2);
	readl(mhdp->regs + CDNS_SW_EVENT3);

	/* Activate uCPU */
	ret = cdns_mhdp_set_firmware_active(&mhdp->mbox, true);
	if (ret)
		return ret;

	spin_lock(&mhdp->start_lock);

	mhdp->hw_state = MHDP_HW_READY;

	/*
	 * Here we must keep the lock while enabling the interrupts
	 * since it would otherwise be possible that interrupt enable
	 * code is executed after the bridge is detached. The similar
	 * situation is not possible in attach()/detach() callbacks
	 * since the hw_state changes from MHDP_HW_READY to
	 * MHDP_HW_STOPPED happens only due to driver removal when
	 * bridge should already be detached.
	 */
	if (mhdp->bridge_attached)
		writel(~(u32)CDNS_APB_INT_MASK_SW_EVENT_INT,
		       mhdp->regs + CDNS_APB_INT_MASK);

	spin_unlock(&mhdp->start_lock);

	wake_up(&mhdp->fw_load_wq);
	dev_dbg(mhdp->dev, "DP FW activated\n");

	return 0;
}

#if 0
static void cdns_mhdp8501_fw_cb(const struct firmware *fw, void *context)
{
	/* TODO(mw): unverified */
	struct cdns_mhdp8501_device *mhdp = context;
	bool bridge_attached;
	int ret;

	dev_dbg(mhdp->dev, "firmware callback\n");

	if (!fw || !fw->data) {
		dev_err(mhdp->dev, "%s: No firmware.\n", __func__);
		return;
	}


	if (ret)
		return;

	/*
	 *  XXX how to make sure the bridge is still attached when
	 *      calling drm_kms_helper_hotplug_event() after releasing
	 *      the lock? We should not hold the spin lock when
	 *      calling drm_kms_helper_hotplug_event() since it may
	 *      cause a dead lock. FB-dev console calls detect from the
	 *      same thread just down the call stack started here.
	 */
	spin_lock(&mhdp->start_lock);
	bridge_attached = mhdp->bridge_attached;
	spin_unlock(&mhdp->start_lock);
#if 0
	if (bridge_attached)
		drm_bridge_hpd_notify(&mhdp->bridge, cdns_mhdp8501_detect(mhdp));
#endif
}
#endif

static int cdns_mhdp8501_load_firmware(struct cdns_mhdp8501_device *mhdp)
{
	const char *fwname = mhdp->info->fwname;
	const struct firmware *fw;
	int ret;

	ret = request_firmware_direct(&fw, fwname, mhdp->dev);
	if (ret) {
		dev_err(mhdp->dev, "Unable to load firmware %s\n", fwname);
		return ret;
	}

	ret = cdns_mhdp8501_fw_activate(fw, mhdp);
	release_firmware(fw);

	return ret;
}

static const struct cdns_mhdp8501_platform_info mhdp8501_platform_info = {
	.fwname = FW_NAME,
};

static int cdns_mhdp8501_probe(struct platform_device *pdev)
{
	struct cdns_mhdp8501_device *mhdp;
	struct device *dev = &pdev->dev;
	struct device_node *remote;
	unsigned long rate;
	struct clk *clk;
	int ret;
	int irq;

	mhdp = devm_kzalloc(dev, sizeof(*mhdp), GFP_KERNEL);
	if (!mhdp)
		return -ENOMEM;

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "couldn't get clk: %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

        remote = of_graph_get_remote_node(dev->of_node, 1, -1);
        if (!remote)
                return -EINVAL;

        mhdp->next_bridge = of_drm_find_bridge(remote);
        of_node_put(remote);

	mhdp->clk = clk;
	mhdp->dev = dev;
	mutex_init(&mhdp->link_mutex);
	spin_lock_init(&mhdp->start_lock);

	mhdp->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mhdp->regs)) {
		dev_err(dev, "Failed to get memory resource\n");
		return PTR_ERR(mhdp->regs);
	}

#if 0
	mhdp->phy = devm_of_phy_get_by_index(dev, pdev->dev.of_node, 0);
	if (IS_ERR(mhdp->phy)) {
		dev_err(dev, "no PHY configured\n");
		return PTR_ERR(mhdp->phy);
	}
#endif

	platform_set_drvdata(pdev, mhdp);

	cdns_mhdp_mailbox_init(&mhdp->mbox, dev, mhdp->regs);

	mhdp->info = of_device_get_match_data(dev);
	if (!mhdp->info)
		mhdp->info = &mhdp8501_platform_info;

	ret = cdns_mhdp8501_phy_probe(pdev);
	if (ret)
		return ret;

	clk_prepare_enable(clk);

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		pm_runtime_disable(dev);
		goto clk_disable;
	}

	rate = clk_get_rate(clk);
	writel(rate % 1000000, mhdp->regs + CDNS_SW_CLK_L);
	writel(rate / 1000000, mhdp->regs + CDNS_SW_CLK_H);

	dev_dbg(dev, "func clk rate %lu Hz\n", rate);

	writel(~0, mhdp->regs + CDNS_APB_INT_MASK);

#if 0
	irq = platform_get_irq(pdev, 0);
	ret = devm_request_threaded_irq(mhdp->dev, irq, NULL,
					cdns_mhdp8501_irq_handler, IRQF_ONESHOT,
					"mhdp8546", mhdp);
	if (ret) {
		dev_err(dev, "cannot install IRQ %d\n", irq);
		ret = -EIO;
		goto plat_fini;
	}
#endif

	cdns_mhdp8501_fill_host_caps(mhdp);

	/* Initialize link rate and num of lanes to host values */
	mhdp->link.rate = mhdp->host.link_rate;
	mhdp->link.num_lanes = mhdp->host.lanes_cnt;

	/* The only currently supported format */
	mhdp->display_fmt.y_only = false;
	mhdp->display_fmt.color_format = DRM_COLOR_FORMAT_RGB444;
	mhdp->display_fmt.bpc = 8;


	/* Initialize the work for modeset in case of link train failure */
	//INIT_WORK(&mhdp->modeset_retry_work, cdns_mhdp8501_modeset_retry_fn);
	//INIT_WORK(&mhdp->hpd_work, cdns_mhdp8501_hpd_work);

	init_waitqueue_head(&mhdp->fw_load_wq);
	init_waitqueue_head(&mhdp->sw_events_wq);

	ret = cdns_mhdp8501_load_firmware(mhdp);
	if (ret)
		goto phy_exit;

	/* Line swapping */
	if (mhdp->info->lane_mapping)
		cdns_mhdp_reg_write(&mhdp->mbox, LANES_CONFIG,
				    0x00400000 | mhdp->info->lane_mapping);

	ret = phy_init(mhdp->phy);
	if (ret) {
		dev_err(mhdp->dev, "Failed to initialize PHY: %d\n", ret);
		goto plat_fini;
	}

printk("%s:%d\n", __func__, __LINE__);
	mhdp->bridge.of_node = pdev->dev.of_node;
	mhdp->bridge.funcs = &cdns_mhdp8501_bridge_funcs;
	mhdp->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID;
#if 0
	mhdp->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID |
			   DRM_BRIDGE_OP_HPD;
#endif
	mhdp->bridge.type = DRM_MODE_CONNECTOR_DisplayPort;
	drm_bridge_add(&mhdp->bridge);

	cdns_mhdp8501_encoder_init(pdev);

	return 0;

phy_exit:
	phy_exit(mhdp->phy);
plat_fini:
runtime_put:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
clk_disable:
	clk_disable_unprepare(mhdp->clk);

	return ret;
}

static int cdns_mhdp8501_remove(struct platform_device *pdev)
{
	struct cdns_mhdp8501_device *mhdp = platform_get_drvdata(pdev);
	unsigned long timeout = msecs_to_jiffies(100);
	bool stop_fw = false;
	int ret;

	drm_bridge_remove(&mhdp->bridge);

	ret = wait_event_timeout(mhdp->fw_load_wq,
				 mhdp->hw_state == MHDP_HW_READY,
				 timeout);
	if (ret == 0)
		dev_err(mhdp->dev, "%s: Timeout waiting for fw loading\n",
			__func__);
	else
		stop_fw = true;

	spin_lock(&mhdp->start_lock);
	mhdp->hw_state = MHDP_HW_STOPPED;
	spin_unlock(&mhdp->start_lock);

	if (stop_fw)
		ret = cdns_mhdp_set_firmware_active(&mhdp->mbox, false);

	phy_exit(mhdp->phy);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	//cancel_work_sync(&mhdp->modeset_retry_work);
	flush_scheduled_work();

	clk_disable_unprepare(mhdp->clk);

	return ret;
}

static const struct cdns_mhdp8501_platform_info ls1028a_platform_info = {
	.fwname = FW_NAME_LS1028A,
	.lane_mapping = 0x4e,
};

static const struct of_device_id cdns_mhdp8501_ids[] = {
	{ .compatible = "cdns,mhdp8501", },
	{ .compatible = "fsl,ls1028a-mhdp8501", .data = &ls1028a_platform_info },
	{ }
};
MODULE_DEVICE_TABLE(of, cdns_mhdp8501_ids);

static struct platform_driver cdns_mhdp8501_driver = {
	.driver	= {
		.name		= "cdns-mhdp8501",
		.of_match_table	= cdns_mhdp8501_ids,
	},
	.probe	= cdns_mhdp8501_probe,
	.remove	= cdns_mhdp8501_remove,
};

module_platform_driver(cdns_mhdp8501_driver);

MODULE_AUTHOR("Michael Walle <michael@walle.cc>");
MODULE_LICENSE("GPL");
