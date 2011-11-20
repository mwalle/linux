/*
 * arch/arm/mach-kirkwood/mpp.c
 *
 * MPP functions for Marvell Kirkwood SoCs
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <mach/hardware.h>
#include <plat/mpp.h>
#include "common.h"
#include "mpp.h"

static unsigned int __init kirkwood_variant(void)
{
	u32 dev, rev;

	kirkwood_pcie_id(&dev, &rev);

	if ((dev == MV88F6281_DEV_ID && rev >= MV88F6281_REV_A0) ||
	    (dev == MV88F6282_DEV_ID))
		return MPP_F6281_MASK;
	if (dev == MV88F6192_DEV_ID && rev >= MV88F6192_REV_A0)
		return MPP_F6192_MASK;
	if (dev == MV88F6180_DEV_ID)
		return MPP_F6180_MASK;

	printk(KERN_ERR "MPP setup: unknown kirkwood variant "
			"(dev %#x rev %#x)\n", dev, rev);
	return 0;
}

void __init kirkwood_mpp_conf(unsigned int *mpp_list)
{
	orion_mpp_conf(mpp_list, kirkwood_variant(),
		       MPP_MAX, DEV_BUS_VIRT_BASE);
}

#define MPP_MAX 49
void __init kirkwood_dt_mpp_conf()
{
	struct device_node *np;
	unsigned int mpp_nr_regs = (1 + MPP_MAX/8);
	unsigned int mpp_ctrl[mpp_nr_regs];
	const u32 *cfg, *basep;
	u64 base, size;
	void __iomem *iobase;
	
	int len;
	int i;

	np = of_find_compatible_node(NULL, NULL, "marvell,kirkwood-mpp");
	if (!np)
		return;

	basep = of_get_address(np, 0, &size, NULL);
	if (!basep) {
		printk(KERN_ERR "kirkwood-mpp: no 'reg' property");
		return;
	}

	base = of_translate_address(np, basep);
	iobase = ioremap(base, size);

	cfg = of_get_property(np, "gpio-conf", &len);
	if (!cfg) {
		printk(KERN_ERR "kirkwood-mpp: no 'gpio-conf' property");
		return;
	}

	printk(KERN_DEBUG "initial MPP regs:");
	for (i = 0; i < mpp_nr_regs; i++) {
		mpp_ctrl[i] = readl(iobase + i * 4);
		printk(" %08x", mpp_ctrl[i]);
	}
	printk("\n");

	len /= sizeof(u32);
	len /= 2;

	while (len--) {
		u32 num = be32_to_cpup(cfg++);
		u32 sel = be32_to_cpup(cfg++);
		int shift;

		shift = (num & 7) << 2;
		mpp_ctrl[num / 8] &= ~(0xf << shift);
		mpp_ctrl[num / 8] |= sel << shift;
	}

	printk(KERN_DEBUG "  final MPP regs:");
	for (i = 0; i < mpp_nr_regs; i++) {
		writel(mpp_ctrl[i], iobase + i * 4);
		printk(" %08x", mpp_ctrl[i]);
	}
	printk("\n");
}
