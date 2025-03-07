// SPDX-License-Identifier: GPL-2.0
/* Microchip LAN937X switch driver main logic
 * Copyright (C) 2019-2022 Microchip Technology Inc.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include <linux/phy.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/math.h>
#include <net/dsa.h>
#include <net/switchdev.h>

#include "lan937x_reg.h"
#include "ksz_common.h"
#include "lan937x.h"

#define LAN937x_PNIRQS 6

static int lan937x_cfg(struct ksz_device *dev, u32 addr, u8 bits, bool set)
{
	return regmap_update_bits(dev->regmap[0], addr, bits, set ? bits : 0);
}

static int lan937x_port_cfg(struct ksz_device *dev, int port, int offset,
			    u8 bits, bool set)
{
	return regmap_update_bits(dev->regmap[0], PORT_CTRL_ADDR(port, offset),
				  bits, set ? bits : 0);
}

static int lan937x_enable_spi_indirect_access(struct ksz_device *dev)
{
	u16 data16;
	int ret;

	/* Enable Phy access through SPI */
	ret = lan937x_cfg(dev, REG_GLOBAL_CTRL_0, SW_PHY_REG_BLOCK, false);
	if (ret < 0)
		return ret;

	ret = ksz_read16(dev, REG_VPHY_SPECIAL_CTRL__2, &data16);
	if (ret < 0)
		return ret;

	/* Allow SPI access */
	data16 |= VPHY_SPI_INDIRECT_ENABLE;

	return ksz_write16(dev, REG_VPHY_SPECIAL_CTRL__2, data16);
}

static int lan937x_vphy_ind_addr_wr(struct ksz_device *dev, int addr, int reg)
{
	u16 addr_base = REG_PORT_T1_PHY_CTRL_BASE;
	u16 temp;

	/* get register address based on the logical port */
	temp = PORT_CTRL_ADDR(addr, (addr_base + (reg << 2)));

	return ksz_write16(dev, REG_VPHY_IND_ADDR__2, temp);
}

static int lan937x_internal_phy_write(struct ksz_device *dev, int addr, int reg,
				      u16 val)
{
	unsigned int value;
	int ret;

	/* Check for internal phy port */
	if (!dev->info->internal_phy[addr])
		return -EOPNOTSUPP;

	ret = lan937x_vphy_ind_addr_wr(dev, addr, reg);
	if (ret < 0)
		return ret;

	/* Write the data to be written to the VPHY reg */
	ret = ksz_write16(dev, REG_VPHY_IND_DATA__2, val);
	if (ret < 0)
		return ret;

	/* Write the Write En and Busy bit */
	ret = ksz_write16(dev, REG_VPHY_IND_CTRL__2,
			  (VPHY_IND_WRITE | VPHY_IND_BUSY));
	if (ret < 0)
		return ret;

	ret = regmap_read_poll_timeout(dev->regmap[1], REG_VPHY_IND_CTRL__2,
				       value, !(value & VPHY_IND_BUSY), 10,
				       1000);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to write phy register\n");
		return ret;
	}

	return 0;
}

static int lan937x_internal_phy_read(struct ksz_device *dev, int addr, int reg,
				     u16 *val)
{
	unsigned int value;
	int ret;

	/* Check for internal phy port, return 0xffff for non-existent phy */
	if (!dev->info->internal_phy[addr])
		return 0xffff;

	ret = lan937x_vphy_ind_addr_wr(dev, addr, reg);
	if (ret < 0)
		return ret;

	/* Write Read and Busy bit to start the transaction */
	ret = ksz_write16(dev, REG_VPHY_IND_CTRL__2, VPHY_IND_BUSY);
	if (ret < 0)
		return ret;

	ret = regmap_read_poll_timeout(dev->regmap[1], REG_VPHY_IND_CTRL__2,
				       value, !(value & VPHY_IND_BUSY), 10,
				       1000);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to read phy register\n");
		return ret;
	}

	/* Read the VPHY register which has the PHY data */
	return ksz_read16(dev, REG_VPHY_IND_DATA__2, val);
}

int lan937x_r_phy(struct ksz_device *dev, u16 addr, u16 reg, u16 *data)
{
	return lan937x_internal_phy_read(dev, addr, reg, data);
}

int lan937x_w_phy(struct ksz_device *dev, u16 addr, u16 reg, u16 val)
{
	return lan937x_internal_phy_write(dev, addr, reg, val);
}

static int lan937x_sw_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct ksz_device *dev = bus->priv;
	u16 val;
	int ret;

	ret = lan937x_internal_phy_read(dev, addr, regnum, &val);
	if (ret < 0)
		return ret;

	return val;
}

static int lan937x_sw_mdio_write(struct mii_bus *bus, int addr, int regnum,
				 u16 val)
{
	struct ksz_device *dev = bus->priv;

	return lan937x_internal_phy_write(dev, addr, regnum, val);
}

static int lan937x_irq_phy_setup(struct ksz_device *dev)
{
	struct dsa_switch *ds = dev->ds;
	int phy, err_phy;
	int irq;
	int ret;

	for (phy = 0; phy < KSZ_MAX_NUM_PORTS; phy++) {
		if (BIT(phy) & ds->phys_mii_mask) {
			irq = irq_find_mapping(dev->ports[phy].pirq.domain,
					       PORT_SRC_PHY_INT);
			if (irq < 0) {
				ret = irq;
				goto out;
			}
			ds->slave_mii_bus->irq[phy] = irq;
		}
	}
	return 0;
out:
	err_phy = phy;

	for (phy = 0; phy < err_phy; phy++)
		if (BIT(phy) & ds->phys_mii_mask)
			irq_dispose_mapping(ds->slave_mii_bus->irq[phy]);

	return ret;
}

static void lan937x_irq_phy_free(struct ksz_device *dev)
{
	struct dsa_switch *ds = dev->ds;
	int phy;

	for (phy = 0; phy < KSZ_MAX_NUM_PORTS; phy++)
		if (BIT(phy) & ds->phys_mii_mask)
			irq_dispose_mapping(ds->slave_mii_bus->irq[phy]);
}

static int lan937x_mdio_register(struct ksz_device *dev)
{
	struct dsa_switch *ds = dev->ds;
	struct device_node *mdio_np;
	struct mii_bus *bus;
	int ret;

	mdio_np = of_get_child_by_name(dev->dev->of_node, "mdio");
	if (!mdio_np) {
		dev_err(ds->dev, "no MDIO bus node\n");
		return -ENODEV;
	}

	bus = devm_mdiobus_alloc(ds->dev);
	if (!bus) {
		of_node_put(mdio_np);
		return -ENOMEM;
	}

	bus->priv = dev;
	bus->read = lan937x_sw_mdio_read;
	bus->write = lan937x_sw_mdio_write;
	bus->name = "lan937x slave smi";
	snprintf(bus->id, MII_BUS_ID_SIZE, "SMI-%d", ds->index);
	bus->parent = ds->dev;
	bus->phy_mask = ~ds->phys_mii_mask;

	ds->slave_mii_bus = bus;

	ret = lan937x_irq_phy_setup(dev);
	if (ret) {
		of_node_put(mdio_np);
		return ret;
	}

	ret = devm_of_mdiobus_register(ds->dev, bus, mdio_np);
	if (ret) {
		dev_err(ds->dev, "unable to register MDIO bus %s\n",
			bus->id);
		lan937x_irq_phy_free(dev);
	}

	of_node_put(mdio_np);

	return ret;
}

int lan937x_reset_switch(struct ksz_device *dev)
{
	u32 data32;
	int ret;

	/* reset switch */
	ret = lan937x_cfg(dev, REG_SW_OPERATION, SW_RESET, true);
	if (ret < 0)
		return ret;

	/* Enable Auto Aging */
	ret = lan937x_cfg(dev, REG_SW_LUE_CTRL_1, SW_LINK_AUTO_AGING, true);
	if (ret < 0)
		return ret;

	/* disable interrupts */
	ret = ksz_write32(dev, REG_SW_INT_MASK__4, SWITCH_INT_MASK);
	if (ret < 0)
		return ret;

	ret = ksz_write32(dev, REG_SW_INT_STATUS__4, POR_READY_INT);
	if (ret < 0)
		return ret;

	ret = ksz_write32(dev, REG_SW_PORT_INT_MASK__4, 0xFF);
	if (ret < 0)
		return ret;

	return ksz_read32(dev, REG_SW_PORT_INT_STATUS__4, &data32);
}

void lan937x_port_setup(struct ksz_device *dev, int port, bool cpu_port)
{
	const u32 *masks = dev->info->masks;
	const u16 *regs = dev->info->regs;
	struct dsa_switch *ds = dev->ds;
	u8 member;

	/* enable tag tail for host port */
	if (cpu_port)
		lan937x_port_cfg(dev, port, REG_PORT_CTRL_0,
				 PORT_TAIL_TAG_ENABLE, true);

	/* disable frame check length field */
	lan937x_port_cfg(dev, port, REG_PORT_MAC_CTRL_0, PORT_CHECK_LENGTH,
			 false);

	/* set back pressure for half duplex */
	lan937x_port_cfg(dev, port, REG_PORT_MAC_CTRL_1, PORT_BACK_PRESSURE,
			 true);

	/* enable 802.1p priority */
	lan937x_port_cfg(dev, port, P_PRIO_CTRL, PORT_802_1P_PRIO_ENABLE, true);

	if (!dev->info->internal_phy[port])
		lan937x_port_cfg(dev, port, regs[P_XMII_CTRL_0],
				 masks[P_MII_TX_FLOW_CTRL] |
				 masks[P_MII_RX_FLOW_CTRL],
				 true);

	if (cpu_port)
		member = dsa_user_ports(ds);
	else
		member = BIT(dsa_upstream_port(ds, port));

	dev->dev_ops->cfg_port_member(dev, port, member);
}

void lan937x_config_cpu_port(struct dsa_switch *ds)
{
	struct ksz_device *dev = ds->priv;
	struct dsa_port *dp;

	dsa_switch_for_each_cpu_port(dp, ds) {
		if (dev->info->cpu_ports & (1 << dp->index)) {
			dev->cpu_port = dp->index;

			/* enable cpu port */
			lan937x_port_setup(dev, dp->index, true);
		}
	}

	dsa_switch_for_each_user_port(dp, ds) {
		ksz_port_stp_state_set(ds, dp->index, BR_STATE_DISABLED);
	}
}

int lan937x_change_mtu(struct ksz_device *dev, int port, int new_mtu)
{
	struct dsa_switch *ds = dev->ds;
	int ret;

	new_mtu += VLAN_ETH_HLEN + ETH_FCS_LEN;

	if (dsa_is_cpu_port(ds, port))
		new_mtu += LAN937X_TAG_LEN;

	if (new_mtu >= FR_MIN_SIZE)
		ret = lan937x_port_cfg(dev, port, REG_PORT_MAC_CTRL_0,
				       PORT_JUMBO_PACKET, true);
	else
		ret = lan937x_port_cfg(dev, port, REG_PORT_MAC_CTRL_0,
				       PORT_JUMBO_PACKET, false);
	if (ret < 0) {
		dev_err(ds->dev, "failed to enable jumbo\n");
		return ret;
	}

	/* Write the frame size in PORT_MAX_FR_SIZE register */
	ksz_pwrite16(dev, port, PORT_MAX_FR_SIZE, new_mtu);

	return 0;
}

int lan937x_set_ageing_time(struct ksz_device *dev, unsigned int msecs)
{
	u32 secs = msecs / 1000;
	u32 value;
	int ret;

	value = FIELD_GET(SW_AGE_PERIOD_7_0_M, secs);

	ret = ksz_write8(dev, REG_SW_AGE_PERIOD__1, value);
	if (ret < 0)
		return ret;

	value = FIELD_GET(SW_AGE_PERIOD_19_8_M, secs);

	return ksz_write16(dev, REG_SW_AGE_PERIOD__2, value);
}

static void lan937x_set_tune_adj(struct ksz_device *dev, int port,
				 u16 reg, u8 val)
{
	u16 data16;

	ksz_pread16(dev, port, reg, &data16);

	/* Update tune Adjust */
	data16 |= FIELD_PREP(PORT_TUNE_ADJ, val);
	ksz_pwrite16(dev, port, reg, data16);

	/* write DLL reset to take effect */
	data16 |= PORT_DLL_RESET;
	ksz_pwrite16(dev, port, reg, data16);
}

static void lan937x_set_rgmii_tx_delay(struct ksz_device *dev, int port)
{
	u8 val;

	/* Apply different codes based on the ports as per characterization
	 * results
	 */
	val = (port == LAN937X_RGMII_1_PORT) ? RGMII_1_TX_DELAY_2NS :
		RGMII_2_TX_DELAY_2NS;

	lan937x_set_tune_adj(dev, port, REG_PORT_XMII_CTRL_5, val);
}

static void lan937x_set_rgmii_rx_delay(struct ksz_device *dev, int port)
{
	u8 val;

	val = (port == LAN937X_RGMII_1_PORT) ? RGMII_1_RX_DELAY_2NS :
		RGMII_2_RX_DELAY_2NS;

	lan937x_set_tune_adj(dev, port, REG_PORT_XMII_CTRL_4, val);
}

void lan937x_phylink_get_caps(struct ksz_device *dev, int port,
			      struct phylink_config *config)
{
	config->mac_capabilities = MAC_100FD;

	if (dev->info->supports_rgmii[port]) {
		/* MII/RMII/RGMII ports */
		config->mac_capabilities |= MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
					    MAC_100HD | MAC_10 | MAC_1000FD;
	}
}

void lan937x_setup_rgmii_delay(struct ksz_device *dev, int port)
{
	struct ksz_port *p = &dev->ports[port];

	if (p->rgmii_tx_val) {
		lan937x_set_rgmii_tx_delay(dev, port);
		dev_info(dev->dev, "Applied rgmii tx delay for the port %d\n",
			 port);
	}

	if (p->rgmii_rx_val) {
		lan937x_set_rgmii_rx_delay(dev, port);
		dev_info(dev->dev, "Applied rgmii rx delay for the port %d\n",
			 port);
	}
}

int lan937x_switch_init(struct ksz_device *dev)
{
	dev->port_mask = (1 << dev->info->port_cnt) - 1;

	return 0;
}

static void lan937x_girq_mask(struct irq_data *d)
{
	struct ksz_device *dev = irq_data_get_irq_chip_data(d);
	unsigned int n = d->hwirq;

	dev->girq.masked |= (1 << n);
}

static void lan937x_girq_unmask(struct irq_data *d)
{
	struct ksz_device *dev = irq_data_get_irq_chip_data(d);
	unsigned int n = d->hwirq;

	dev->girq.masked &= ~(1 << n);
}

static void lan937x_girq_bus_lock(struct irq_data *d)
{
	struct ksz_device *dev = irq_data_get_irq_chip_data(d);

	mutex_lock(&dev->lock_irq);
}

static void lan937x_girq_bus_sync_unlock(struct irq_data *d)
{
	struct ksz_device *dev = irq_data_get_irq_chip_data(d);
	int ret;

	ret = ksz_write32(dev, REG_SW_PORT_INT_MASK__4, dev->girq.masked);
	if (ret)
		dev_err(dev->dev, "failed to change IRQ mask\n");

	mutex_unlock(&dev->lock_irq);
}

static const struct irq_chip lan937x_girq_chip = {
	.name			= "lan937x-global",
	.irq_mask		= lan937x_girq_mask,
	.irq_unmask		= lan937x_girq_unmask,
	.irq_bus_lock		= lan937x_girq_bus_lock,
	.irq_bus_sync_unlock	= lan937x_girq_bus_sync_unlock,
};

static int lan937x_girq_domain_map(struct irq_domain *d,
				   unsigned int irq, irq_hw_number_t hwirq)
{
	struct ksz_device *dev = d->host_data;

	irq_set_chip_data(irq, d->host_data);
	irq_set_chip_and_handler(irq, &dev->girq.chip, handle_level_irq);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops lan937x_girq_domain_ops = {
	.map	= lan937x_girq_domain_map,
	.xlate	= irq_domain_xlate_twocell,
};

static void lan937x_girq_free(struct ksz_device *dev)
{
	int irq, virq;

	free_irq(dev->irq, dev);

	for (irq = 0; irq < dev->girq.nirqs; irq++) {
		virq = irq_find_mapping(dev->girq.domain, irq);
		irq_dispose_mapping(virq);
	}

	irq_domain_remove(dev->girq.domain);
}

static irqreturn_t lan937x_girq_thread_fn(int irq, void *dev_id)
{
	struct ksz_device *dev = dev_id;
	unsigned int nhandled = 0;
	unsigned int sub_irq;
	unsigned int n;
	u32 data;
	int ret;

	/* Read global interrupt status register */
	ret = ksz_read32(dev, REG_SW_PORT_INT_STATUS__4, &data);
	if (ret)
		goto out;

	for (n = 0; n < dev->girq.nirqs; ++n) {
		if (data & (1 << n)) {
			sub_irq = irq_find_mapping(dev->girq.domain, n);
			handle_nested_irq(sub_irq);
			++nhandled;
		}
	}
out:
	return (nhandled > 0 ? IRQ_HANDLED : IRQ_NONE);
}

static int lan937x_girq_setup(struct ksz_device *dev)
{
	int ret, irq;

	dev->girq.nirqs = dev->info->port_cnt;
	dev->girq.domain = irq_domain_add_simple(NULL, dev->girq.nirqs, 0,
						 &lan937x_girq_domain_ops, dev);
	if (!dev->girq.domain)
		return -ENOMEM;

	for (irq = 0; irq < dev->girq.nirqs; irq++)
		irq_create_mapping(dev->girq.domain, irq);

	dev->girq.chip = lan937x_girq_chip;
	dev->girq.masked = ~0;

	ret = request_threaded_irq(dev->irq, NULL, lan937x_girq_thread_fn,
				   IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				   dev_name(dev->dev), dev);
	if (ret)
		goto out;

	return 0;

out:
	lan937x_girq_free(dev);

	return ret;
}

static void lan937x_pirq_mask(struct irq_data *d)
{
	struct ksz_port *port = irq_data_get_irq_chip_data(d);
	unsigned int n = d->hwirq;

	port->pirq.masked |= (1 << n);
}

static void lan937x_pirq_unmask(struct irq_data *d)
{
	struct ksz_port *port = irq_data_get_irq_chip_data(d);
	unsigned int n = d->hwirq;

	port->pirq.masked &= ~(1 << n);
}

static void lan937x_pirq_bus_lock(struct irq_data *d)
{
	struct ksz_port *port = irq_data_get_irq_chip_data(d);
	struct ksz_device *dev = port->ksz_dev;

	mutex_lock(&dev->lock_irq);
}

static void lan937x_pirq_bus_sync_unlock(struct irq_data *d)
{
	struct ksz_port *port = irq_data_get_irq_chip_data(d);
	struct ksz_device *dev = port->ksz_dev;

	ksz_pwrite8(dev, port->num, REG_PORT_INT_MASK, port->pirq.masked);
	mutex_unlock(&dev->lock_irq);
}

static const struct irq_chip lan937x_pirq_chip = {
	.name			= "lan937x-port",
	.irq_mask		= lan937x_pirq_mask,
	.irq_unmask		= lan937x_pirq_unmask,
	.irq_bus_lock		= lan937x_pirq_bus_lock,
	.irq_bus_sync_unlock	= lan937x_pirq_bus_sync_unlock,
};

static int lan937x_pirq_domain_map(struct irq_domain *d, unsigned int irq,
				   irq_hw_number_t hwirq)
{
	struct ksz_port *port = d->host_data;

	irq_set_chip_data(irq, d->host_data);
	irq_set_chip_and_handler(irq, &port->pirq.chip, handle_level_irq);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops lan937x_pirq_domain_ops = {
	.map	= lan937x_pirq_domain_map,
	.xlate	= irq_domain_xlate_twocell,
};

static void lan937x_pirq_free(struct ksz_device *dev, u8 p)
{
	struct ksz_port *port = &dev->ports[p];
	int irq, virq;
	int irq_num;

	irq_num = irq_find_mapping(dev->girq.domain, p);
	if (irq_num < 0)
		return;

	free_irq(irq_num, port);

	for (irq = 0; irq < port->pirq.nirqs; irq++) {
		virq = irq_find_mapping(port->pirq.domain, irq);
		irq_dispose_mapping(virq);
	}

	irq_domain_remove(port->pirq.domain);
}

static irqreturn_t lan937x_pirq_thread_fn(int irq, void *dev_id)
{
	struct ksz_port *port = dev_id;
	unsigned int nhandled = 0;
	struct ksz_device *dev;
	unsigned int sub_irq;
	unsigned int n;
	u8 data;

	dev = port->ksz_dev;

	/* Read port interrupt status register */
	ksz_pread8(dev, port->num, REG_PORT_INT_STATUS, &data);

	for (n = 0; n < port->pirq.nirqs; ++n) {
		if (data & (1 << n)) {
			sub_irq = irq_find_mapping(port->pirq.domain, n);
			handle_nested_irq(sub_irq);
			++nhandled;
		}
	}

	return (nhandled > 0 ? IRQ_HANDLED : IRQ_NONE);
}

static int lan937x_pirq_setup(struct ksz_device *dev, u8 p)
{
	struct ksz_port *port = &dev->ports[p];
	int ret, irq;
	int irq_num;

	port->pirq.nirqs = LAN937x_PNIRQS;
	port->pirq.domain = irq_domain_add_simple(dev->dev->of_node,
						  port->pirq.nirqs, 0,
						  &lan937x_pirq_domain_ops,
						  port);
	if (!port->pirq.domain)
		return -ENOMEM;

	for (irq = 0; irq < port->pirq.nirqs; irq++)
		irq_create_mapping(port->pirq.domain, irq);

	port->pirq.chip = lan937x_pirq_chip;
	port->pirq.masked = ~0;

	irq_num = irq_find_mapping(dev->girq.domain, p);
	if (irq_num < 0)
		return irq_num;

	snprintf(port->pirq.name, sizeof(port->pirq.name), "port_irq-%d", p);

	ret = request_threaded_irq(irq_num, NULL, lan937x_pirq_thread_fn,
				   IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				   port->pirq.name, port);
	if (ret)
		goto out;

	return 0;

out:
	lan937x_pirq_free(dev, p);

	return ret;
}

int lan937x_setup(struct dsa_switch *ds)
{
	struct ksz_device *dev = ds->priv;
	struct dsa_port *dp;
	int ret;

	/* enable Indirect Access from SPI to the VPHY registers */
	ret = lan937x_enable_spi_indirect_access(dev);
	if (ret < 0) {
		dev_err(dev->dev, "failed to enable spi indirect access");
		return ret;
	}

	if (dev->irq > 0) {
		ret = lan937x_girq_setup(dev);
		if (ret)
			return ret;

		dsa_switch_for_each_user_port(dp, dev->ds) {
			ret = lan937x_pirq_setup(dev, dp->index);
			if (ret)
				goto out_girq;
		}
	}

	ret = lan937x_mdio_register(dev);
	if (ret < 0) {
		dev_err(dev->dev, "failed to register the mdio");
		goto out_pirq;
	}

	/* The VLAN aware is a global setting. Mixed vlan
	 * filterings are not supported.
	 */
	ds->vlan_filtering_is_global = true;

	/* Enable aggressive back off for half duplex & UNH mode */
	lan937x_cfg(dev, REG_SW_MAC_CTRL_0,
		    (SW_PAUSE_UNH_MODE | SW_NEW_BACKOFF | SW_AGGR_BACKOFF),
		    true);

	/* If NO_EXC_COLLISION_DROP bit is set, the switch will not drop
	 * packets when 16 or more collisions occur
	 */
	lan937x_cfg(dev, REG_SW_MAC_CTRL_1, NO_EXC_COLLISION_DROP, true);

	/* enable global MIB counter freeze function */
	lan937x_cfg(dev, REG_SW_MAC_CTRL_6, SW_MIB_COUNTER_FREEZE, true);

	/* disable CLK125 & CLK25, 1: disable, 0: enable */
	lan937x_cfg(dev, REG_SW_GLOBAL_OUTPUT_CTRL__1,
		    (SW_CLK125_ENB | SW_CLK25_ENB), true);

	return 0;

out_pirq:
	if (dev->irq > 0)
		dsa_switch_for_each_user_port(dp, dev->ds)
			lan937x_pirq_free(dev, dp->index);
out_girq:
	if (dev->irq > 0)
		lan937x_girq_free(dev);

	return ret;
}

void lan937x_teardown(struct dsa_switch *ds)
{
	struct ksz_device *dev = ds->priv;
	struct dsa_port *dp;

	if (dev->irq > 0) {
		dsa_switch_for_each_user_port(dp, dev->ds)
			lan937x_pirq_free(dev, dp->index);

		lan937x_girq_free(dev);
	}
}

void lan937x_switch_exit(struct ksz_device *dev)
{
	lan937x_reset_switch(dev);
}

MODULE_AUTHOR("Arun Ramadoss <arun.ramadoss@microchip.com>");
MODULE_DESCRIPTION("Microchip LAN937x Series Switch DSA Driver");
MODULE_LICENSE("GPL");
