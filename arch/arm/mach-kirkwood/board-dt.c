/*
 * Marvell Kirkwood device tree board support
 *
 * Copyright (C) 2011 Michael Walle <michael@walle.cc>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/bridge-regs.h>

#include "common.h"
#include "mpp.h"

static struct of_device_id kirkwood_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus", },
	{}
};

static struct of_device_id kirkwood_dt_int_match[] __initdata = {
	{ .compatible = "marvell,kirkwood-intc", },
	{}
};

static void __init kirkwood_board_dt_init(void)
{
	struct device_node *node;

	/*
	 * Basic setup. Needs to be called early.
	 */
	kirkwood_init_dt();

	node = of_find_matching_node_by_address(NULL, kirkwood_dt_int_match,
					IRQ_PHYS_BASE);
	if (node)
		irq_domain_add_simple(node, 0);

	kirkwood_dt_mpp_conf();
	/*
	 * Finished with the static registrations now; fill in the missing
	 * devices
	 */
	of_platform_populate(NULL, kirkwood_dt_match_table, NULL, NULL);
}

static const char * kirkwood_dt_board_compat[] = {
	"marvell,lsxhl",
	"marvell,lschlv2",
	NULL
};

DT_MACHINE_START(KIRKWOOD_DT, "Marvell Kirkwood (Flattened Device Tree)")
	.map_io		= kirkwood_map_io,
	.init_early	= kirkwood_init_early,
	.init_irq	= kirkwood_init_irq,
	.timer		= &kirkwood_timer,
	.init_machine	= kirkwood_board_dt_init,
	.dt_compat	= kirkwood_dt_board_compat,
MACHINE_END
