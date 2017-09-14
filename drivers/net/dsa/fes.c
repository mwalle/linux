#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/ptp_clock_kernel.h>
#include <net/dsa.h>

#define FES_DRV_NAME "fes"

/* find cpu port by pci_get_slot(bus, 0) */
/* from pch_gbe_main.c
 * adapter->ptp_pdev = pci_get_bus_and_slot(adapter->pdev->bus->number,
				                                               PCI_DEVFN(12, 4));
*/

struct fes_priv {
	struct device *dev;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_info;
	void __iomem *regs;
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

#define FES_PORT_OFFSET(i)  (i * 0x10000)
static u16 fes_port_read(struct fes_priv *priv, int port, u32 offset)
{
	return fes_read(priv, FES_PORT_OFFSET(port) + offset);
}

static void fes_port_write(struct fes_priv *priv, int port, u32 offset, u16 value)
{
	fes_write(priv, FES_PORT_OFFSET(port) + offset, value);
}

static enum dsa_tag_protocol fes_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_FES;
}

static struct dsa_chip_data fes_chip_data = {
	.port_names[0] = "cpu",
	.port_names[1] = "lan1",
	.port_names[2] = "lan2",
};

#define FES_ID0 0x0000
#define FES_ID1 0x0002

static int fes_setup(struct dsa_switch *ds)
{
	struct fes_priv *priv = ds->priv;
	u32 id;

	id = fes_read(priv, FES_ID0) << 8;
	id |= (fes_read(priv, FES_ID1) >> 8) & 0xff;
	dev_info(priv->dev, "FES ID %06Xh\n", id);

	/* flush fdb */

	return 0;
}

#define FES_PHY_OFFSET 0x200000
#define FES_MDIO_ADDR 0x84

static int fes_phy_read(struct dsa_switch *ds, int phyad, int reg)
{
	struct fes_priv *priv = ds->priv;
	void __iomem *addr = priv->regs + FES_PHY_OFFSET;
	int val;

	phyad &= 0x1f;
	reg &= 0x1f;

	iowrite32(phyad, addr + FES_MDIO_ADDR);
	return ioread32(addr + (reg << 2)) & 0xffff;
}

static int fes_phy_write(struct dsa_switch *ds, int phyad, int reg, u16 val)
{
	struct fes_priv *priv = ds->priv;
	void __iomem *addr = priv->regs + FES_PHY_OFFSET;

	phyad &= 0x1f;
	reg &= 0x1f;

	iowrite32(phyad, addr + FES_MDIO_ADDR);
	iowrite32(val, addr + (reg << 2));

	return 0;
}

static struct dsa_switch_ops fes_switch_ops = {
	.get_tag_protocol = fes_get_tag_protocol,
	.setup = fes_setup,
	.phy_read = fes_phy_read,
	.phy_write = fes_phy_write,
};

#define FRTC_OFFSET 0x80000
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

#define FRTC_GENERAL     0x0000
#define FRTC_CUR_NSEC    0x1000
#define FRTC_CUR_SEC     0x1008
#define FRTC_TIME_CC     0x1010
#define FRTC_STEP_SIZE   0x1020
#define FRTC_ADJUST_NSEC 0x1030
#define FRTC_ADJUST_SEC  0x1038
#define FRTC_TIME_CMD    0x1040

#define TIME_CMD_ADJUST_TIME      BIT(0)
#define TIME_CMD_ADJUST_STEP      BIT(1)
#define TIME_CMD_READ_TIME        BIT(2)
#define TIME_CMD_ADJUST_TIME_MODE BIT(3)

static int fes_phc_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);

	frtc_write(priv, FRTC_TIME_CMD, TIME_CMD_READ_TIME);

	ts->tv_sec = frtc_read(priv, FRTC_CUR_SEC);
	ts->tv_nsec = frtc_read(priv, FRTC_CUR_NSEC);

	return 0;
}

static int fes_phc_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);

	return 0;
}

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

	ds = dsa_switch_alloc(&pdev->dev, DSA_MAX_PORTS);
	if (!ds)
		return -ENOMEM;

	netdev = dev_get_by_name(&init_net, "eth0");
	if (!netdev)
		return -EPROBE_DEFER;

	fes_chip_data.netdev[0] = &netdev->dev;

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
