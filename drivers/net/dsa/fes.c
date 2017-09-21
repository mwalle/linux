
/*
 * TODO:
 * - locking
 * - timestamps and VLAN
 * - endianess ?!
 * - suspend/resume
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/ptp_classify.h>
#include <linux/etherdevice.h>
#include <net/dsa.h>

#include "fes.h"

#define FES_DRV_NAME "fes"

/* find cpu port by pci_get_slot(bus, 0) */
/* from pch_gbe_main.c
 * adapter->ptp_pdev = pci_get_bus_and_slot(adapter->pdev->bus->number,
				                                               PCI_DEVFN(12, 4));
*/

struct fes_counters_desc {
	const char *name;
	u16 reg_low;
	u16 reg_high;
};

static const struct fes_counters_desc fes_counters_desc[] = {
	{ "rx_good_octets", FES_PORT_RX_GOOD_OCTETS_L, FES_PORT_RX_GOOD_OCTETS_H },
	{ "rx_bad_octets", FES_PORT_RX_BAD_OCTETS_L, FES_PORT_RX_BAD_OCTETS_H },
	{ "rx_unicast", FES_PORT_RX_UNICAST_L, FES_PORT_RX_UNICAST_H },
	{ "rx_broadcast", FES_PORT_RX_BROADCAST_L, FES_PORT_RX_BROADCAST_H },
	{ "rx_multicast", FES_PORT_RX_MULTICAST_L, FES_PORT_RX_MULTICAST_H },
	{ "rx_undersize", FES_PORT_RX_UNDERSIZE_L, FES_PORT_RX_UNDERSIZE_H },
	{ "rx_fragments", FES_PORT_RX_FRAGMENTS_L, FES_PORT_RX_FRAGMENTS_H },
	{ "rx_oversize", FES_PORT_RX_OVERSIZE_L, FES_PORT_RX_OVERSIZE_H },
	{ "rx_jabber", FES_PORT_RX_JABBER_L, FES_PORT_RX_JABBER_H },
	{ "rx_err", FES_PORT_RX_ERR_L, FES_PORT_RX_ERR_H },
	{ "rx_crc", FES_PORT_RX_CRC_L, FES_PORT_RX_CRC_H },
	{ "rx_64", FES_PORT_RX_64_L, FES_PORT_RX_64_H },
	{ "rx_65_127", FES_PORT_RX_65_127_L, FES_PORT_RX_65_127_H },
	{ "rx_128_255", FES_PORT_RX_128_255_L, FES_PORT_RX_128_255_H },
	{ "rx_256_511", FES_PORT_RX_256_511_L, FES_PORT_RX_256_511_H },
	{ "rx_512_1023", FES_PORT_RX_512_1023_L, FES_PORT_RX_512_1023_H },
	{ "rx_1024_1536", FES_PORT_RX_1024_1536_L, FES_PORT_RX_1024_1536_H },
	{ "rx_hsrprp", FES_PORT_RX_HSRPRP_L, FES_PORT_RX_HSRPRP_H },
	{ "rx_wronglan", FES_PORT_RX_WRONGLAN_L, FES_PORT_RX_WRONGLAN_H },
	{ "rx_duplicate", FES_PORT_RX_DUPLICATE_L, FES_PORT_RX_DUPLICATE_H },
	{ "full_drop", FES_PORT_FULL_DROP_L, FES_PORT_FULL_DROP_H },
	{ "rx_policed", FES_PORT_RX_POLICED_L, FES_PORT_RX_POLICED_H },
	{ "rx_preempt_smd_err", FES_PORT_RX_PREEMPT_SMD_ERR_L, FES_PORT_RX_PREEMPT_SMD_ERR_H },
	{ "rx_macsec_untagged", FES_PORT_RX_MACSEC_UNTAGGED_L, FES_PORT_RX_MACSEC_UNTAGGED_H },
	{ "rx_macsec_notsupp", FES_PORT_RX_MACSEC_NOTSUPP_L, FES_PORT_RX_MACSEC_NOTSUPP_H },
	{ "rx_macsec_unknownsci", FES_PORT_RX_MACSEC_UNKNOWNSCI_L, FES_PORT_RX_MACSEC_UNKNOWNSCI_H },
	{ "rx_macsec_notvalid", FES_PORT_RX_MACSEC_NOTVALID_L, FES_PORT_RX_MACSEC_NOTVALID_H },
	{ "rx_macsec_late", FES_PORT_RX_MACSEC_LATE_L, FES_PORT_RX_MACSEC_LATE_H },
	{ "rx_preempt_ass_err", FES_PORT_RX_PREEMPT_ASS_ERR_L, FES_PORT_RX_PREEMPT_ASS_ERR_H },
	{ "rx_preempt_ass_ok", FES_PORT_RX_PREEMPT_ASS_OK_L, FES_PORT_RX_PREEMPT_ASS_OK_H },
	{ "rx_preempt_frag_cnt", FES_PORT_RX_PREEMPT_FRAG_CNT_L, FES_PORT_RX_PREEMPT_FRAG_CNT_H },
	{ "tx_octets", FES_PORT_TX_OCTETS_L, FES_PORT_TX_OCTETS_H },
	{ "tx_unicast", FES_PORT_TX_UNICAST_L, FES_PORT_TX_UNICAST_H },
	{ "tx_broadcast", FES_PORT_TX_BROADCAST_L, FES_PORT_TX_BROADCAST_H },
	{ "tx_multicast", FES_PORT_TX_MULTICAST_L, FES_PORT_TX_MULTICAST_H },
	{ "tx_hsrprp", FES_PORT_TX_HSRPRP_L, FES_PORT_TX_HSRPRP_H },
	{ "tx_overrun", FES_PORT_TX_OVERRUN_L, FES_PORT_TX_OVERRUN_H },
	{ "tx_preemptfrag_cnt", FES_PORT_TX_PREEMPTFRAG_CNT_L, FES_PORT_TX_PREEMPTFRAG_CNT_H },
	{ "priq_drop", FES_PORT_PRIQ_DROP_L, FES_PORT_PRIQ_DROP_H },
	{ "early_drop", FES_PORT_EARLY_DROP_L, FES_PORT_EARLY_DROP_H },
};

struct fes_chip_data {
	/* must be first because DSA will cast it to dsa_chip_data */
	struct dsa_chip_data cd;
	int phyad[DSA_MAX_PORTS];
};

struct fes_priv {
	struct device *dev;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_info;
	int phc_index;
	void __iomem *regs;
	struct fes_chip_data *cd;
	struct delayed_work ptp_tx_work;
	struct sk_buff *ptp_tx_skb;
	u64 counters[DSA_MAX_PORTS][ARRAY_SIZE(fes_counters_desc)];
};

static u16 fes_read(struct fes_priv *priv, u32 offset)
{
	void __iomem *addr = priv->regs + offset;

	return ioread16(addr);
}

static void fes_write(struct fes_priv *priv, u32 offset, u16 value)
{
	void __iomem *addr = priv->regs + offset;

	iowrite16(value, addr);
}

static int fes_cmd(struct fes_priv *priv, u32 offset, u16 cmd)
{
	int timeout = 10;

	fes_write(priv, offset, cmd);
	do {
		if (fes_read(priv, offset) & cmd)
			break;
		cpu_relax();
	} while (timeout-- > 0);

	return (timeout) ? 0 : -EIO;
}

static u16 fes_port_read(struct fes_priv *priv, int port, u32 offset)
{
	return fes_read(priv, FES_PORT_OFFSET(port) + offset);
}

static void fes_port_write(struct fes_priv *priv, int port, u32 offset, u16 value)
{
	fes_write(priv, FES_PORT_OFFSET(port) + offset, value);
}

static int fes_port_cmd(struct fes_priv *priv, int port, u32 offset, u16 cmd)
{
	return fes_cmd(priv, FES_PORT_OFFSET(port) + offset, cmd);
}

static enum dsa_tag_protocol fes_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_FES;
}

static struct fes_chip_data fes_chip_data = {
	.cd = {
		.port_names[0] = "cpu",
		.port_names[1] = "lan1",
		//.port_names[2] = "lan2",
	},
	.phyad[0] = 0,
	.phyad[1] = 1,
	//.phyad[2] = -1,
};

static int fes_setup(struct dsa_switch *ds)
{
	struct fes_priv *priv = ds->priv;
	int ret;
	u32 id;
	u16 val;
	int cpu_port = 0;

	/* software reset */
	ret = fes_cmd(priv, FES_GENERAL, GENERAL_SOFTWARE_RESET);
	if (ret)
		return -ENODEV;

	id = fes_read(priv, FES_ID0) << 8;
	id |= (fes_read(priv, FES_ID1) >> 8) & 0xff;
	dev_info(priv->dev, "FES ID %06Xh\n", id);

	/* enable management trailer */
	val = fes_port_read(priv, cpu_port, FES_PORT_STATE);
	val &= PORT_STATE_MGMT_STATE_MASK;
	val |= PORT_STATE_MGMT_STATE_MGMT;
	fes_port_write(priv, cpu_port, FES_PORT_STATE, val);

	/* enable timestamp slots */
	fes_write(priv, FES_TS_CTRL_TX, 0xf);
	fes_write(priv, FES_TS_CTRL_RX, 0xf);

#if 1
	/* enable L2 ptp timestamping */
	fes_write(priv, FES_GENERAL, GENERAL_PTP_MODE_L2);
#endif

	/* flush fdb */

	return 0;
}

static int fes_phy_read(struct dsa_switch *ds, int port, int reg)
{
	struct fes_priv *priv = ds->priv;
	void __iomem *addr = priv->regs + FES_PHY_OFFSET;
	int phyad = priv->cd->phyad[port];

	if (phyad < 0)
		return 0xffff;

	phyad &= 0x1f;
	reg &= 0x1f;

	iowrite32(phyad, addr + FES_MDIO_ADDR);
	return ioread32(addr + (reg << 2)) & 0xffff;
}

static int fes_phy_write(struct dsa_switch *ds, int port, int reg, u16 val)
{
	struct fes_priv *priv = ds->priv;
	void __iomem *addr = priv->regs + FES_PHY_OFFSET;
	int phyad = priv->cd->phyad[port];

	phyad &= 0x1f;
	reg &= 0x1f;

	if (phyad < 0)
		return 0;

	iowrite32(phyad, addr + FES_MDIO_ADDR);
	iowrite32(val, addr + (reg << 2));

	return 0;
}

static int fes_get_ts_info(struct dsa_switch *ds, int port,
			   struct ethtool_ts_info *info)
{
	struct fes_priv *priv = ds->priv;

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
	info->phc_index = priv->phc_index;
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ONESTEP_SYNC);
	info->rx_filters =
		(1 << HWTSTAMP_FILTER_NONE) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT);
	return 0;
}

static int fes_ptp_match(struct sk_buff *skb, u8 ts_msgtype, u16 ts_seqid)
{
	unsigned int ptp_class;
	u8 *msgtype, *data = skb->data;
	u16 *seqid;
	int offset = 0;

	ptp_class = ptp_classify_raw(skb);

	switch (ptp_class) {
	case PTP_CLASS_V2_IPV4:
		offset = ETH_HLEN + IPV4_HLEN(data + offset) + UDP_HLEN;
		break;
	case PTP_CLASS_V2_L2:
		offset = ETH_HLEN;
		break;
	default:
		/* we don't support any other variants */
		return 0;
	}

	if (skb->len + ETH_HLEN < offset + OFF_PTP_SEQUENCE_ID + sizeof(*seqid))
		return 0;

	msgtype = data + offset;
	seqid = (u16 *)(data + offset + OFF_PTP_SEQUENCE_ID);

	return (ts_msgtype == (*msgtype & 0xf) && ts_seqid == ntohs(*seqid));
}

static ktime_t fes_fetch_timestamps(struct fes_priv *priv, struct sk_buff *skb)
{
	int i,j;
	int mask = 0;
	u32 ns, s;
	u16 ctrl_tx;
	u8 ptp_message[64];
	u8 msgtype;
	u16 seqid;
	ktime_t ret = 0;
	int match;

	ctrl_tx = fes_read(priv, FES_TS_CTRL_TX);
	for (i = 0; i <= 3; i++) {
		if (!(ctrl_tx & BIT(i))) {
			ns = fes_read(priv, FES_TX_TS_NS_LO(i));
			ns |= fes_read(priv, FES_TX_TS_NS_HI(i)) << 16;
			s = fes_read(priv, FES_TX_TS_S_LO(i));
			s |= fes_read(priv, FES_TX_TS_S_HI(i)) << 16;

			/*
			 * From the reference manual:
			 *   The nanoseconds value can at some cases be more than
			 *   999999999. The software using the timestamp can handle
			 *   the situation by substracting 1000000000 from the
			 *   nanoseconds value and adding 1 to seconds.
			 */
			if (ns >= NSEC_PER_SEC) {
				ns -= NSEC_PER_SEC;
				s += 1;
			}

			msgtype = fes_read(priv, FES_TX_TS_HDR_BASE(i)) & 0xf;
			seqid = be16_to_cpu(fes_read(priv, FES_TX_TS_HDR_BASE(i) + OFF_PTP_SEQUENCE_ID));

#if 0
			printk("rx timestamp %d.%d\n", s, ns);
			for (j = 0; j <= 63; j += 2) {
				ptp_message[j] = fes_read(priv, FES_TX_TS_HDR_BASE(i) + j) & 0xff;
				ptp_message[j+1] = fes_read(priv, FES_TX_TS_HDR_BASE(i) + j) >> 8;
			}
			print_hex_dump(KERN_INFO, "ptp: ", DUMP_PREFIX_NONE, 16, 1, ptp_message, 64, false);
			printk("  msgtype=%d seqid=%d\n", msgtype, seqid);
#endif

			if (!ret) {
				__skb_push(skb, ETH_HLEN);
				match = fes_ptp_match(skb, msgtype, seqid);
				__skb_pull(skb, ETH_HLEN);
				if (match)
					ret = ktime_set(s, ns);
			}

			mask |= BIT(i);
		}
	}
	fes_write(priv, FES_TS_CTRL_TX, mask);

	return ret;
}

static void fes_rx_timestamp(struct dsa_switch *ds, int port, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *ssh;

	ssh = skb_hwtstamps(skb);
	memset(ssh, 0, sizeof(*ssh));
	ssh->hwtstamp = fes_fetch_timestamps(ds->priv, skb);
}

static void fes_tx_timestamp_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fes_priv *priv = container_of(dwork, struct fes_priv,
			ptp_tx_work);
	int i,j;
	int mask = 0;
	u32 ns, s;
	u16 ctrl_rx;
	u8 ptp_message[64];
	u8 msgtype;
	u16 seqid;
	ktime_t ts = 0;

	if (!priv->ptp_tx_skb)
		return;

	ctrl_rx = fes_read(priv, FES_TS_CTRL_RX);
	for (i = 0; i <= 3; i++) {
		if (!(ctrl_rx & BIT(i))) {
			ns = fes_read(priv, FES_RX_TS_NS_LO(i));
			ns |= fes_read(priv, FES_RX_TS_NS_HI(i)) << 16;
			s = fes_read(priv, FES_RX_TS_S_LO(i));
			s |= fes_read(priv, FES_RX_TS_S_HI(i)) << 16;

			/*
			 * From the reference manual:
			 *   The nanoseconds value can at some cases be more than
			 *   999999999. The software using the timestamp can handle
			 *   the situation by substracting 1000000000 from the
			 *   nanoseconds value and adding 1 to seconds.
			 */
			if (ns >= NSEC_PER_SEC) {
				ns -= NSEC_PER_SEC;
				s += 1;
			}

			msgtype = fes_read(priv, FES_RX_TS_HDR_BASE(i)) & 0xf;
			seqid = be16_to_cpu(fes_read(priv, FES_RX_TS_HDR_BASE(i) + OFF_PTP_SEQUENCE_ID));

#if 0
			printk("tx timestamp %d.%d\n", s, ns);
			for (j = 0; j <= 63; j += 2) {
				ptp_message[j] = fes_read(priv, FES_RX_TS_HDR_BASE(i) + j) & 0xff;
				ptp_message[j+1] = fes_read(priv, FES_RX_TS_HDR_BASE(i) + j) >> 8;
			}
			print_hex_dump(KERN_INFO, "ptp: ", DUMP_PREFIX_NONE, 16, 1, ptp_message, 64, false);
			printk("  msgtype=%d seqid=%d\n", msgtype, seqid);
#endif

			if (!ts && fes_ptp_match(priv->ptp_tx_skb, msgtype, seqid)) {
				ts = ktime_set(s, ns);
			}

			mask |= BIT(i);
		}
	}
	fes_write(priv, FES_TS_CTRL_RX, mask);

	if (ts) {
		struct skb_shared_hwtstamps shhwtstamps = { 0 };
		shhwtstamps.hwtstamp = ts;
		skb_tstamp_tx(priv->ptp_tx_skb, &shhwtstamps);
		dev_kfree_skb_any(priv->ptp_tx_skb);
		priv->ptp_tx_skb = NULL;
	} else {
		schedule_delayed_work(&priv->ptp_tx_work, 100);
	}


}

static void fes_tx_timestamp(struct dsa_switch *ds, int port, struct sk_buff *skb)
{
	struct fes_priv *priv = ds->priv;
	if (priv->ptp_tx_skb)
		return;

	fes_write(priv, FES_TS_CTRL_TX, 0xf);
	skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	priv->ptp_tx_skb = skb_get(skb);
	schedule_delayed_work(&priv->ptp_tx_work, 1);
}

static int fes_hwtstamp_set(struct dsa_switch *ds, int port, struct ifreq *ifr)
{
	struct fes_priv *priv = ds->priv;
	struct hwtstamp_config cfg;
	u16 general;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags)
		return -EINVAL;

	general = fes_read(priv, FES_GENERAL);
	general &= ~GENERAL_MODIFY_SYNC_FRAMES;
	general &= ~GENERAL_PTP_MODE_MASK;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_ONESTEP_SYNC:
		general |= GENERAL_MODIFY_SYNC_FRAMES;
		break;
	case HWTSTAMP_TX_ON:
	case HWTSTAMP_TX_OFF:
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
		general |= GENERAL_PTP_MODE_L4;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_L2_EVENT;
		general |= GENERAL_PTP_MODE_L2;
		break;
	default:
		return -ERANGE;
	}

	fes_write(priv, FES_GENERAL, general);

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int fes_hwtstamp_get(struct dsa_switch *ds, int port, struct ifreq *ifr)
{
	return -EOPNOTSUPP;
}

void fes_port_update_counters(struct fes_priv *priv, int port)
{
	int ret;
	int i;

	/* capture */
	ret = fes_port_cmd(priv, port, FES_PORT_CNT_CTRL,
			   PORT_CNT_CTRL_CAPTURE);
	if (ret)
		return;

	for (i = 0; i < ARRAY_SIZE(fes_counters_desc); i++) {
		const struct fes_counters_desc *desc = &fes_counters_desc[i];
		u32 val;
		val = fes_port_read(priv, port, desc->reg_high);
		val <<= 16;
		val |= fes_port_read(priv, port, desc->reg_low);
		priv->counters[port][i] += val;
	}
}

void fes_get_strings(struct dsa_switch *ds, int port, u8 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(fes_counters_desc); i++) {
		strlcpy(data, fes_counters_desc[i].name, ETH_GSTRING_LEN);
		data += ETH_GSTRING_LEN;
	}
}

void fes_get_ethtool_stats(struct dsa_switch *ds, int port, u64 *data)
{
	struct fes_priv *priv = ds->priv;
	int i;

	fes_port_update_counters(priv, port);

	for (i = 0; i < ARRAY_SIZE(fes_counters_desc); i++)
		*(data++) = priv->counters[port][i];
}

int fes_get_sset_count(struct dsa_switch *ds)
{
	return ARRAY_SIZE(fes_counters_desc);
}


struct fes_fdb_entry {
	int port;
	u8 mac[ETH_ALEN];
};

static int fes_fdb_next(struct fes_priv *priv, struct fes_fdb_entry *fdb)
{
	u16 reg[4];
	int i;

	fes_write(priv, FES_MAC_TABLE(0), MAC_TABLE0_TRANSFER);
	for (i = 0; i <= 3; i++)
		reg[i] = fes_read(priv, FES_MAC_TABLE(i));

	fdb->port = reg[0] & MAC_TABLE0_PORT_MASK;
	fdb->mac[0] = reg[1] & 0xff;
	fdb->mac[1] = (reg[1] >> 8) & 0xff;
	fdb->mac[2] = reg[2] & 0xff;
	fdb->mac[3] = (reg[2] >> 8) & 0xff;
	fdb->mac[4] = reg[3] & 0xff;
	fdb->mac[5] = (reg[3] >> 8) & 0xff;

	return !is_broadcast_ether_addr(fdb->mac);
}

#define FES_NUM_FDB_RECORDS  1024
static int fes_port_fdb_dump(struct dsa_switch *ds, int port,
			     struct switchdev_obj_port_fdb *fdb,
			     switchdev_obj_dump_cb_t *cb)
{
	struct fes_priv *priv = ds->priv;
	struct fes_fdb_entry _fdb;
	int cnt = FES_NUM_FDB_RECORDS;
	int ret = 0;

	while (cnt -- && fes_fdb_next(priv, &_fdb)) {
		ether_addr_copy(fdb->addr, _fdb.mac);
		fdb->vid = 0;
		fdb->ndm_state = NUD_REACHABLE;
		ret = cb(&fdb->obj);
		if (ret)
			break;
	}

	return 0;
}

static struct dsa_switch_ops fes_switch_ops = {
	.get_tag_protocol = fes_get_tag_protocol,
	.setup = fes_setup,
	.phy_read = fes_phy_read,
	.phy_write = fes_phy_write,
	.get_ts_info = fes_get_ts_info,
	.hwtstamp_set = fes_hwtstamp_set,
	.hwtstamp_get = fes_hwtstamp_get,
	.rx_timestamp = fes_rx_timestamp,
	.tx_timestamp = fes_tx_timestamp,

	.get_strings = fes_get_strings,
	.get_ethtool_stats = fes_get_ethtool_stats,
	.get_sset_count = fes_get_sset_count,

	.port_fdb_dump = fes_port_fdb_dump,
};

static u64 frtc_read(struct fes_priv *priv, u32 offset)
{
	void __iomem *addr = priv->regs + FRTC_OFFSET + offset;

	return readq(addr);
}

static void frtc_write(struct fes_priv *priv, u32 offset, u64 value)
{
	void __iomem *addr = priv->regs + FRTC_OFFSET + offset;

	return writeq(value, addr);
}

static int frtc_cmd(struct fes_priv *priv, u32 offset, u64 cmd)
{
	int timeout = 10;

	frtc_write(priv, offset, cmd);
	do {
		if (frtc_read(priv, offset) & cmd)
			break;
		cpu_relax();
	} while (timeout-- > 0);

	return (timeout) ? 0 : -EIO;
}

static int fes_phc_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	int ret;

	ret = frtc_cmd(priv, FRTC_TIME_CMD, TIME_CMD_READ_TIME);
	if (ret)
		return ret;

	ts->tv_sec = frtc_read(priv, FRTC_CUR_SEC);
	ts->tv_nsec = frtc_read(priv, FRTC_CUR_NSEC) >> 32;

	return 0;
}

static int fes_phc_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

static int fes_phc_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	struct timespec ts = ns_to_timespec(delta);

	frtc_write(priv, FRTC_ADJUST_NSEC, ts.tv_nsec << 32);
	frtc_write(priv, FRTC_ADJUST_SEC, ts.tv_sec);
	return frtc_cmd(priv, FRTC_TIME_CMD, TIME_CMD_ADJUST_TIME);
}

static int fes_phc_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	struct timespec orig_ts, delta;
	int ret;

	ret = fes_phc_gettime(ptp, &orig_ts);
	if (ret)
		return ret;

	delta = timespec64_sub(*ts, orig_ts);

	frtc_write(priv, FRTC_ADJUST_NSEC, delta.tv_nsec << 32);
	frtc_write(priv, FRTC_ADJUST_SEC, delta.tv_sec);
	return frtc_cmd(priv, FRTC_TIME_CMD, TIME_CMD_ADJUST_TIME);
}

static int fes_adjust_step(struct fes_priv *priv, u64 subnsec)
{
	frtc_write(priv, FRTC_STEP_SIZE, subnsec);
	return frtc_cmd(priv, FRTC_TIME_CMD, TIME_CMD_ADJUST_STEP);
}

#define NS_STEP_SIZE 8ULL
#define INITIAL_SUBNS (NS_STEP_SIZE << 32)
#define ADJFREQ_SCALING_FACTOR ((1ULL<<32) * NS_STEP_SIZE / 1000000000)
static int fes_phc_adjfreq(struct ptp_clock_info *ptp, s32 delta)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	u64 subnsec = INITIAL_SUBNS + delta * ADJFREQ_SCALING_FACTOR;

	return fes_adjust_step(priv, subnsec);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
static int fes_phc_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	u64 subnsec;

	s64 delta = (NS_STEP_SIZE << 32) * scaled_ppm;
	delta = div_s64(delta, 1000000);
	delta >>= 16;

	subnsec = INITIAL_SUBNS + delta;
	return fes_adjust_step(priv, subnsec);
}
#endif

static struct ptp_clock_info fes_ptp_clock_info = {
	.owner = THIS_MODULE,
	.name = "fes",
	.max_adj = 99999999,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0,
	.n_pins = 0,
	.pps = 0,
	.gettime64 = fes_phc_gettime,
	.settime64 = fes_phc_settime,
	.adjtime = fes_phc_adjtime,
	.adjfreq = fes_phc_adjfreq,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
	.adjfine = fes_phc_adjfine,
#endif
	.enable = fes_phc_enable,
};

static int fes_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret, i;
	struct fes_priv *priv;
	struct dsa_switch *ds;
	struct net_device *netdev;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: ERROR: failed to enable device\n",
				__func__);
		return ret;
	}

	for (i = 0; i <= PCI_STD_RESOURCE_END; i++) {
		if (pci_resource_len(pdev, i) == 0)
			continue;
		ret = pcim_iomap_regions(pdev, BIT(i), FES_DRV_NAME);
		if (ret)
			return ret;
		break;
	}

	pci_set_master(pdev);

	priv->dev = &pdev->dev;
	priv->regs = pcim_iomap_table(pdev)[i];
	INIT_DELAYED_WORK(&priv->ptp_tx_work, fes_tx_timestamp_work);

	ds = dsa_switch_alloc(&pdev->dev, DSA_MAX_PORTS);
	if (!ds)
		return -ENOMEM;

	netdev = dev_get_by_name(&init_net, "eth0");
	if (!netdev)
		return -EPROBE_DEFER;

	fes_chip_data.cd.netdev[0] = &netdev->dev;

	priv->cd = &fes_chip_data;
	ds->dev = &pdev->dev;
	ds->ops = &fes_switch_ops;
	ds->priv = priv;
	pdev->dev.platform_data = &fes_chip_data;

	dev_set_drvdata(&pdev->dev, priv);

	priv->ptp_clock_info = fes_ptp_clock_info;
	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_info, &pdev->dev);
	if (IS_ERR(priv->ptp_clock)) {
		priv->ptp_clock = NULL;
		dev_err(&pdev->dev, "ptp_clock_register failed\n");
	} else if (priv->ptp_clock) {
		dev_info(&pdev->dev, "added PHC\n");
	}
	priv->phc_index = ptp_clock_index(priv->ptp_clock);

	return dsa_register_switch(ds);
}

static void fes_remove(struct pci_dev *pdev)
{
}

static const struct pci_device_id fes_id_table[] = {
	{0x1172, 0xe001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{}
};
MODULE_DEVICE_TABLE(pci, fes_id_table);

static struct pci_driver fes_pci_driver = {
	.name = FES_DRV_NAME,
	.id_table = fes_id_table,
	.probe = fes_probe,
	.remove = fes_remove,
};

module_pci_driver(fes_pci_driver);

MODULE_DESCRIPTION("Flexibilis Ethernet Switch PCI driver");
MODULE_AUTHOR("Michael Walle <michael.walle@kontron.com>");
MODULE_LICENSE("GPL");
