// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) 2016-2017 Hisilicon Limited.

#include "hclge_main.h"
#include "hclge_dcb.h"
#include "hclge_tm.h"
#include "hnae3.h"

#define BW_PERCENT	100

static int hclge_ieee_ets_to_tm_info(struct hclge_dev *hdev,
				     struct ieee_ets *ets)
{
	u8 i;

	for (i = 0; i < HNAE3_MAX_TC; i++) {
		switch (ets->tc_tsa[i]) {
		case IEEE_8021QAZ_TSA_STRICT:
			hdev->tm_info.tc_info[i].tc_sch_mode =
				HCLGE_SCH_MODE_SP;
			hdev->tm_info.pg_info[0].tc_dwrr[i] = 0;
			break;
		case IEEE_8021QAZ_TSA_ETS:
			hdev->tm_info.tc_info[i].tc_sch_mode =
				HCLGE_SCH_MODE_DWRR;
			hdev->tm_info.pg_info[0].tc_dwrr[i] =
				ets->tc_tx_bw[i];
			break;
		default:
			/* Hardware only supports SP (strict priority)
			 * or ETS (enhanced transmission selection)
			 * algorithms, if we receive some other value
			 * from dcbnl, then throw an error.
			 */
			return -EINVAL;
		}
	}

	hclge_tm_prio_tc_info_update(hdev, ets->prio_tc);

	return 0;
}

static void hclge_tm_info_to_ieee_ets(struct hclge_dev *hdev,
				      struct ieee_ets *ets)
{
	u32 i;

	memset(ets, 0, sizeof(*ets));
	ets->willing = 1;
	ets->ets_cap = hdev->tc_max;

	for (i = 0; i < HNAE3_MAX_TC; i++) {
		ets->prio_tc[i] = hdev->tm_info.prio_tc[i];
		ets->tc_tx_bw[i] = hdev->tm_info.pg_info[0].tc_dwrr[i];

		if (hdev->tm_info.tc_info[i].tc_sch_mode ==
		    HCLGE_SCH_MODE_SP)
			ets->tc_tsa[i] = IEEE_8021QAZ_TSA_STRICT;
		else
			ets->tc_tsa[i] = IEEE_8021QAZ_TSA_ETS;
	}
}

/* IEEE std */
static int hclge_ieee_getets(struct hnae3_handle *h, struct ieee_ets *ets)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct hclge_dev *hdev = vport->back;

	hclge_tm_info_to_ieee_ets(hdev, ets);

	return 0;
}

static int hclge_dcb_common_validate(struct hclge_dev *hdev, u8 num_tc,
				     u8 *prio_tc)
{
	int i;

	if (num_tc > hdev->tc_max) {
		dev_err(&hdev->pdev->dev,
			"tc num checking failed, %u > tc_max(%u)\n",
			num_tc, hdev->tc_max);
		return -EINVAL;
	}

	for (i = 0; i < HNAE3_MAX_USER_PRIO; i++) {
		if (prio_tc[i] >= num_tc) {
			dev_err(&hdev->pdev->dev,
				"prio_tc[%d] checking failed, %u >= num_tc(%u)\n",
				i, prio_tc[i], num_tc);
			return -EINVAL;
		}
	}

	if (num_tc > hdev->vport[0].alloc_tqps) {
		dev_err(&hdev->pdev->dev,
			"allocated tqp checking failed, %u > tqp(%u)\n",
			num_tc, hdev->vport[0].alloc_tqps);
		return -EINVAL;
	}

	return 0;
}

static u8 hclge_ets_tc_changed(struct hclge_dev *hdev, struct ieee_ets *ets,
			       bool *changed)
{
	u8 max_tc_id = 0;
	u8 i;

	for (i = 0; i < HNAE3_MAX_USER_PRIO; i++) {
		if (ets->prio_tc[i] != hdev->tm_info.prio_tc[i])
			*changed = true;

		if (ets->prio_tc[i] > max_tc_id)
			max_tc_id = ets->prio_tc[i];
	}

	/* return max tc number, max tc id need to plus 1 */
	return max_tc_id + 1;
}

static int hclge_ets_sch_mode_validate(struct hclge_dev *hdev,
				       struct ieee_ets *ets, bool *changed)
{
	bool has_ets_tc = false;
	u32 total_ets_bw = 0;
	u8 i;

	for (i = 0; i < HNAE3_MAX_TC; i++) {
		switch (ets->tc_tsa[i]) {
		case IEEE_8021QAZ_TSA_STRICT:
			if (hdev->tm_info.tc_info[i].tc_sch_mode !=
				HCLGE_SCH_MODE_SP)
				*changed = true;
			break;
		case IEEE_8021QAZ_TSA_ETS:
			/* The hardware will switch to sp mode if bandwidth is
			 * 0, so limit ets bandwidth must be greater than 0.
			 */
			if (!ets->tc_tx_bw[i]) {
				dev_err(&hdev->pdev->dev,
					"tc%u ets bw cannot be 0\n", i);
				return -EINVAL;
			}

			if (hdev->tm_info.tc_info[i].tc_sch_mode !=
				HCLGE_SCH_MODE_DWRR)
				*changed = true;

			total_ets_bw += ets->tc_tx_bw[i];
			has_ets_tc = true;
			break;
		default:
			return -EINVAL;
		}
	}

	if (has_ets_tc && total_ets_bw != BW_PERCENT)
		return -EINVAL;

	return 0;
}

static int hclge_ets_validate(struct hclge_dev *hdev, struct ieee_ets *ets,
			      u8 *tc, bool *changed)
{
	u8 tc_num;
	int ret;

	tc_num = hclge_ets_tc_changed(hdev, ets, changed);

	ret = hclge_dcb_common_validate(hdev, tc_num, ets->prio_tc);
	if (ret)
		return ret;

	ret = hclge_ets_sch_mode_validate(hdev, ets, changed);
	if (ret)
		return ret;

	*tc = tc_num;
	if (*tc != hdev->tm_info.num_tc)
		*changed = true;

	return 0;
}

static int hclge_map_update(struct hclge_dev *hdev)
{
	int ret;

	ret = hclge_tm_schd_setup_hw(hdev);
	if (ret)
		return ret;

	ret = hclge_pause_setup_hw(hdev, false);
	if (ret)
		return ret;

	ret = hclge_buffer_alloc(hdev);
	if (ret)
		return ret;

	hclge_comm_rss_indir_init_cfg(hdev->ae_dev, &hdev->rss_cfg);

	return hclge_rss_init_hw(hdev);
}

static int hclge_notify_down_uinit(struct hclge_dev *hdev)
{
	int ret;

	ret = hclge_notify_client(hdev, HNAE3_DOWN_CLIENT);
	if (ret)
		return ret;

	return hclge_notify_client(hdev, HNAE3_UNINIT_CLIENT);
}

static int hclge_notify_init_up(struct hclge_dev *hdev)
{
	int ret;

	ret = hclge_notify_client(hdev, HNAE3_INIT_CLIENT);
	if (ret)
		return ret;

	return hclge_notify_client(hdev, HNAE3_UP_CLIENT);
}

static int hclge_ieee_setets(struct hnae3_handle *h, struct ieee_ets *ets)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct net_device *netdev = h->kinfo.netdev;
	struct hclge_dev *hdev = vport->back;
	bool map_changed = false;
	u8 num_tc = 0;
	int ret;

	if (!(hdev->dcbx_cap & DCB_CAP_DCBX_VER_IEEE) ||
	    hdev->flag & HCLGE_FLAG_MQPRIO_ENABLE)
		return -EINVAL;

	ret = hclge_ets_validate(hdev, ets, &num_tc, &map_changed);
	if (ret)
		return ret;

	if (map_changed) {
		netif_dbg(h, drv, netdev, "set ets\n");

		ret = hclge_notify_down_uinit(hdev);
		if (ret)
			return ret;
	}

	hclge_tm_schd_info_update(hdev, num_tc);
	if (num_tc > 1)
		hdev->flag |= HCLGE_FLAG_DCB_ENABLE;
	else
		hdev->flag &= ~HCLGE_FLAG_DCB_ENABLE;

	ret = hclge_ieee_ets_to_tm_info(hdev, ets);
	if (ret)
		goto err_out;

	if (map_changed) {
		ret = hclge_map_update(hdev);
		if (ret)
			goto err_out;

		return hclge_notify_init_up(hdev);
	}

	return hclge_tm_dwrr_cfg(hdev);

err_out:
	if (!map_changed)
		return ret;

	hclge_notify_init_up(hdev);

	return ret;
}

static int hclge_ieee_getpfc(struct hnae3_handle *h, struct ieee_pfc *pfc)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct hclge_dev *hdev = vport->back;
	int ret;

	memset(pfc, 0, sizeof(*pfc));
	pfc->pfc_cap = hdev->pfc_max;
	pfc->pfc_en = hdev->tm_info.pfc_en;

	ret = hclge_mac_update_stats(hdev);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"failed to update MAC stats, ret = %d.\n", ret);
		return ret;
	}

	hclge_pfc_tx_stats_get(hdev, pfc->requests);
	hclge_pfc_rx_stats_get(hdev, pfc->indications);

	return 0;
}

static int hclge_ieee_setpfc(struct hnae3_handle *h, struct ieee_pfc *pfc)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct net_device *netdev = h->kinfo.netdev;
	struct hclge_dev *hdev = vport->back;
	u8 i, j, pfc_map, *prio_tc;
	int ret;

	if (!(hdev->dcbx_cap & DCB_CAP_DCBX_VER_IEEE))
		return -EINVAL;

	if (pfc->pfc_en == hdev->tm_info.pfc_en)
		return 0;

	prio_tc = hdev->tm_info.prio_tc;
	pfc_map = 0;

	for (i = 0; i < hdev->tm_info.num_tc; i++) {
		for (j = 0; j < HNAE3_MAX_USER_PRIO; j++) {
			if ((prio_tc[j] == i) && (pfc->pfc_en & BIT(j))) {
				pfc_map |= BIT(i);
				break;
			}
		}
	}

	hdev->tm_info.hw_pfc_map = pfc_map;
	hdev->tm_info.pfc_en = pfc->pfc_en;

	netif_dbg(h, drv, netdev,
		  "set pfc: pfc_en=%x, pfc_map=%x, num_tc=%u\n",
		  pfc->pfc_en, pfc_map, hdev->tm_info.num_tc);

	hclge_tm_pfc_info_update(hdev);

	ret = hclge_pause_setup_hw(hdev, false);
	if (ret)
		return ret;

	ret = hclge_notify_client(hdev, HNAE3_DOWN_CLIENT);
	if (ret)
		return ret;

	ret = hclge_buffer_alloc(hdev);
	if (ret) {
		hclge_notify_client(hdev, HNAE3_UP_CLIENT);
		return ret;
	}

	return hclge_notify_client(hdev, HNAE3_UP_CLIENT);
}

static int hclge_ieee_setapp(struct hnae3_handle *h, struct dcb_app *app)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct net_device *netdev = h->kinfo.netdev;
	struct hclge_dev *hdev = vport->back;
	struct dcb_app old_app;
	int ret;

	if (app->selector != IEEE_8021QAZ_APP_SEL_DSCP ||
	    app->protocol >= HCLGE_MAX_DSCP ||
	    app->priority >= HNAE3_MAX_USER_PRIO)
		return -EINVAL;

	dev_info(&hdev->pdev->dev, "setapp dscp=%u priority=%u\n",
		 app->protocol, app->priority);

	if (app->priority == hdev->tm_info.dscp_prio[app->protocol])
		return 0;

	ret = dcb_ieee_setapp(netdev, app);
	if (ret)
		return ret;

	old_app.selector = IEEE_8021QAZ_APP_SEL_DSCP;
	old_app.protocol = app->protocol;
	old_app.priority = hdev->tm_info.dscp_prio[app->protocol];

	hdev->tm_info.dscp_prio[app->protocol] = app->priority;
	ret = hclge_dscp_to_tc_map(hdev);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"failed to set dscp to tc map, ret = %d\n", ret);
		hdev->tm_info.dscp_prio[app->protocol] = old_app.priority;
		(void)dcb_ieee_delapp(netdev, app);
		return ret;
	}

	vport->nic.kinfo.tc_map_mode = HNAE3_TC_MAP_MODE_DSCP;
	if (old_app.priority == HCLGE_PRIO_ID_INVALID)
		hdev->tm_info.dscp_app_cnt++;
	else
		ret = dcb_ieee_delapp(netdev, &old_app);

	return ret;
}

static int hclge_ieee_delapp(struct hnae3_handle *h, struct dcb_app *app)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct net_device *netdev = h->kinfo.netdev;
	struct hclge_dev *hdev = vport->back;
	int ret;

	if (app->selector != IEEE_8021QAZ_APP_SEL_DSCP ||
	    app->protocol >= HCLGE_MAX_DSCP ||
	    app->priority >= HNAE3_MAX_USER_PRIO ||
	    app->priority != hdev->tm_info.dscp_prio[app->protocol])
		return -EINVAL;

	dev_info(&hdev->pdev->dev, "delapp dscp=%u priority=%u\n",
		 app->protocol, app->priority);

	ret = dcb_ieee_delapp(netdev, app);
	if (ret)
		return ret;

	hdev->tm_info.dscp_prio[app->protocol] = HCLGE_PRIO_ID_INVALID;
	ret = hclge_dscp_to_tc_map(hdev);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"failed to del dscp to tc map, ret = %d\n", ret);
		hdev->tm_info.dscp_prio[app->protocol] = app->priority;
		(void)dcb_ieee_setapp(netdev, app);
		return ret;
	}

	if (hdev->tm_info.dscp_app_cnt)
		hdev->tm_info.dscp_app_cnt--;

	if (!hdev->tm_info.dscp_app_cnt) {
		vport->nic.kinfo.tc_map_mode = HNAE3_TC_MAP_MODE_PRIO;
		ret = hclge_up_to_tc_map(hdev);
	}

	return ret;
}

/* DCBX configuration */
static u8 hclge_getdcbx(struct hnae3_handle *h)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct hclge_dev *hdev = vport->back;

	if (hdev->flag & HCLGE_FLAG_MQPRIO_ENABLE)
		return 0;

	return hdev->dcbx_cap;
}

static u8 hclge_setdcbx(struct hnae3_handle *h, u8 mode)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct net_device *netdev = h->kinfo.netdev;
	struct hclge_dev *hdev = vport->back;

	netif_dbg(h, drv, netdev, "set dcbx: mode=%u\n", mode);

	/* No support for LLD_MANAGED modes or CEE */
	if ((mode & DCB_CAP_DCBX_LLD_MANAGED) ||
	    (mode & DCB_CAP_DCBX_VER_CEE) ||
	    !(mode & DCB_CAP_DCBX_HOST))
		return 1;

	hdev->dcbx_cap = mode;

	return 0;
}

static int hclge_mqprio_qopt_check(struct hclge_dev *hdev,
				   struct tc_mqprio_qopt_offload *mqprio_qopt)
{
	u16 queue_sum = 0;
	int ret;
	int i;

	if (!mqprio_qopt->qopt.num_tc) {
		mqprio_qopt->qopt.num_tc = 1;
		return 0;
	}

	ret = hclge_dcb_common_validate(hdev, mqprio_qopt->qopt.num_tc,
					mqprio_qopt->qopt.prio_tc_map);
	if (ret)
		return ret;

	for (i = 0; i < mqprio_qopt->qopt.num_tc; i++) {
		if (!is_power_of_2(mqprio_qopt->qopt.count[i])) {
			dev_err(&hdev->pdev->dev,
				"qopt queue count must be power of 2\n");
			return -EINVAL;
		}

		if (mqprio_qopt->qopt.count[i] > hdev->pf_rss_size_max) {
			dev_err(&hdev->pdev->dev,
				"qopt queue count should be no more than %u\n",
				hdev->pf_rss_size_max);
			return -EINVAL;
		}

		if (mqprio_qopt->qopt.offset[i] != queue_sum) {
			dev_err(&hdev->pdev->dev,
				"qopt queue offset must start from 0, and being continuous\n");
			return -EINVAL;
		}

		if (mqprio_qopt->min_rate[i] || mqprio_qopt->max_rate[i]) {
			dev_err(&hdev->pdev->dev,
				"qopt tx_rate is not supported\n");
			return -EOPNOTSUPP;
		}

		queue_sum = mqprio_qopt->qopt.offset[i];
		queue_sum += mqprio_qopt->qopt.count[i];
	}
	if (hdev->vport[0].alloc_tqps < queue_sum) {
		dev_err(&hdev->pdev->dev,
			"qopt queue count sum should be less than %u\n",
			hdev->vport[0].alloc_tqps);
		return -EINVAL;
	}

	return 0;
}

static void hclge_sync_mqprio_qopt(struct hnae3_tc_info *tc_info,
				   struct tc_mqprio_qopt_offload *mqprio_qopt)
{
	memset(tc_info, 0, sizeof(*tc_info));
	tc_info->num_tc = mqprio_qopt->qopt.num_tc;
	memcpy(tc_info->prio_tc, mqprio_qopt->qopt.prio_tc_map,
	       sizeof_field(struct hnae3_tc_info, prio_tc));
	memcpy(tc_info->tqp_count, mqprio_qopt->qopt.count,
	       sizeof_field(struct hnae3_tc_info, tqp_count));
	memcpy(tc_info->tqp_offset, mqprio_qopt->qopt.offset,
	       sizeof_field(struct hnae3_tc_info, tqp_offset));
}

static int hclge_config_tc(struct hclge_dev *hdev,
			   struct hnae3_tc_info *tc_info)
{
	int i;

	hclge_tm_schd_info_update(hdev, tc_info->num_tc);
	for (i = 0; i < HNAE3_MAX_USER_PRIO; i++)
		hdev->tm_info.prio_tc[i] = tc_info->prio_tc[i];

	return hclge_map_update(hdev);
}

/* Set up TC for hardware offloaded mqprio in channel mode */
static int hclge_setup_tc(struct hnae3_handle *h,
			  struct tc_mqprio_qopt_offload *mqprio_qopt)
{
	struct hclge_vport *vport = hclge_get_vport(h);
	struct hnae3_knic_private_info *kinfo;
	struct hclge_dev *hdev = vport->back;
	struct hnae3_tc_info old_tc_info;
	u8 tc = mqprio_qopt->qopt.num_tc;
	int ret;

	/* if client unregistered, it's not allowed to change
	 * mqprio configuration, which may cause uninit ring
	 * fail.
	 */
	if (!test_bit(HCLGE_STATE_NIC_REGISTERED, &hdev->state))
		return -EBUSY;

	if (hdev->flag & HCLGE_FLAG_DCB_ENABLE)
		return -EINVAL;

	ret = hclge_mqprio_qopt_check(hdev, mqprio_qopt);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"failed to check mqprio qopt params, ret = %d\n", ret);
		return ret;
	}

	ret = hclge_notify_down_uinit(hdev);
	if (ret)
		return ret;

	kinfo = &vport->nic.kinfo;
	memcpy(&old_tc_info, &kinfo->tc_info, sizeof(old_tc_info));
	hclge_sync_mqprio_qopt(&kinfo->tc_info, mqprio_qopt);
	kinfo->tc_info.mqprio_active = tc > 0;

	ret = hclge_config_tc(hdev, &kinfo->tc_info);
	if (ret)
		goto err_out;

	hdev->flag &= ~HCLGE_FLAG_DCB_ENABLE;

	if (tc > 1)
		hdev->flag |= HCLGE_FLAG_MQPRIO_ENABLE;
	else
		hdev->flag &= ~HCLGE_FLAG_MQPRIO_ENABLE;

	return hclge_notify_init_up(hdev);

err_out:
	if (!tc) {
		dev_warn(&hdev->pdev->dev,
			 "failed to destroy mqprio, will active after reset, ret = %d\n",
			 ret);
	} else {
		/* roll-back */
		memcpy(&kinfo->tc_info, &old_tc_info, sizeof(old_tc_info));
		if (hclge_config_tc(hdev, &kinfo->tc_info))
			dev_err(&hdev->pdev->dev,
				"failed to roll back tc configuration\n");
	}
	hclge_notify_init_up(hdev);

	return ret;
}

static const struct hnae3_dcb_ops hns3_dcb_ops = {
	.ieee_getets	= hclge_ieee_getets,
	.ieee_setets	= hclge_ieee_setets,
	.ieee_getpfc	= hclge_ieee_getpfc,
	.ieee_setpfc	= hclge_ieee_setpfc,
	.ieee_setapp    = hclge_ieee_setapp,
	.ieee_delapp    = hclge_ieee_delapp,
	.getdcbx	= hclge_getdcbx,
	.setdcbx	= hclge_setdcbx,
	.setup_tc	= hclge_setup_tc,
};

void hclge_dcb_ops_set(struct hclge_dev *hdev)
{
	struct hclge_vport *vport = hdev->vport;
	struct hnae3_knic_private_info *kinfo;

	/* Hdev does not support DCB or vport is
	 * not a pf, then dcb_ops is not set.
	 */
	if (!hnae3_dev_dcb_supported(hdev) ||
	    vport->vport_id != 0)
		return;

	kinfo = &vport->nic.kinfo;
	kinfo->dcb_ops = &hns3_dcb_ops;
	hdev->dcbx_cap = DCB_CAP_DCBX_VER_IEEE | DCB_CAP_DCBX_HOST;
}
