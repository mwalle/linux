/*
 * arch/arm/mach-kirkwood/lsxhl-setup.c
 *
 * Buffalo LS-XHL Series Setup
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/ata_platform.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/mv643xx_eth.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/gpio-fan.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/kirkwood.h>
#include <plat/mvsdio.h>
#include "common.h"
#include "mpp.h"

/*****************************************************************************
 * 512KB SPI Flash on BOOT Device
 ****************************************************************************/
static struct mtd_partition lsxhl_partitions[] = {
	{
		.name		= "u-boot",
		.size		= 0x70000,
		.offset		= 0x00000,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "u-boot env",
		.size		= 0x10000,
		.offset		= 0x70000,
	}
};

static struct flash_platform_data lsxhl_spi_slave_data = {
	.type		= "m25p40",
	.parts		= lsxhl_partitions,
	.nr_parts	= ARRAY_SIZE(lsxhl_partitions),
};

static struct spi_board_info __initdata lsxhl_spi_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &lsxhl_spi_slave_data,
		.irq		= -1,
		.max_speed_hz	= 20000000,
		.bus_num	= 0,
		.chip_select	= 0,
	}
};

/*****************************************************************************
 * Ethernet
 ****************************************************************************/
static struct mv643xx_eth_platform_data lsxhl_ge00_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(0),
};

static struct mv643xx_eth_platform_data lsxhl_ge01_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(8),
};

/*****************************************************************************
 * SATA
 ****************************************************************************/
static struct mv_sata_platform_data lsxhl_sata_data = {
	.n_ports	= 1,
};

/*****************************************************************************
 * LEDs attached to GPIO
 ****************************************************************************/
#define LSXHL_GPIO_LED_ALARM		37
#define LSXHL_GPIO_LED_INFO		38
#define LSXHL_GPIO_LED_PWR		39
#define LSXHL_GPIO_LED_FUNC_BLUE	36
#define LSXHL_GPIO_LED_FUNC_RED		48

static struct gpio_led lsxhl_led_pins[] = {
	{
		.name			= "alarm:red",
		.gpio			= LSXHL_GPIO_LED_ALARM,
		.active_low		= 1,
	},
	{
		.name			= "info:amber",
		.gpio			= LSXHL_GPIO_LED_INFO,
		.active_low		= 1,
	},
	{
		.name			= "power:blue",
		.default_trigger	= "default-on",
		.gpio			= LSXHL_GPIO_LED_PWR,
		.active_low		= 1,
	},
	{
		.name			= "func:blue:bottom",
		.gpio			= LSXHL_GPIO_LED_FUNC_BLUE,
		.active_low		= 1,
	},
	{
		.name			= "func:red:bottom",
		.gpio			= LSXHL_GPIO_LED_FUNC_RED,
		.active_low		= 1,
	},
};

static struct gpio_led_platform_data lsxhl_led_data = {
	.leds		= lsxhl_led_pins,
	.num_leds	= ARRAY_SIZE(lsxhl_led_pins),
};

static struct platform_device lsxhl_leds = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &lsxhl_led_data,
	}
};

/*****************************************************************************
 * General Setup
 ****************************************************************************/
#define LSXHL_GPIO_HDD_POWER 10
#define LSXHL_GPIO_USB_POWER 11

/*****************************************************************************
 * GPIO Attached Keys
 ****************************************************************************/
#define LSXHL_GPIO_KEY_FUNC		41
#define LSXHL_GPIO_KEY_AUTOPOWER	42
#define LSXHL_GPIO_KEY_POWER		43
#define LSXHL_SW_POWER		0x00
#define LSXHL_SW_AUTOPOWER	0x01
#define LSXHL_SW_FUNC		0x02

static struct gpio_keys_button lsxhl_buttons[] = {
	{
		.type = EV_SW,
		.code = LSXHL_SW_POWER,
		.gpio = LSXHL_GPIO_KEY_POWER,
		.desc = "Power-on Switch",
		.active_low = 1,
	}, {
		.type = EV_SW,
		.code = LSXHL_SW_AUTOPOWER,
		.gpio = LSXHL_GPIO_KEY_AUTOPOWER,
		.desc = "Power-auto Switch",
		.active_low = 1,
	}, {
		.type = EV_SW,
		.code = LSXHL_SW_POWER,
		.gpio = LSXHL_GPIO_KEY_FUNC,
		.desc = "Function Button",
		.active_low = 1,
	},
};

static struct gpio_keys_platform_data lsxhl_button_data = {
	.buttons = lsxhl_buttons,
	.nbuttons = ARRAY_SIZE(lsxhl_buttons),
};

static struct platform_device lsxhl_button_device = {
	.name = "gpio-keys",
	.id = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &lsxhl_button_data,
	},
};

/*****************************************************************************
 * GPIO Fan
 ****************************************************************************/
#define LSXHL_GPIO_FAN_HIGH	18
#define LSXHL_GPIO_FAN_LOW	19
#define LSXHL_GPIO_FAN_LOCK	40

static struct gpio_fan_alarm lsxhl_alarm = {
	.gpio = LSXHL_GPIO_FAN_LOCK,
};

static struct gpio_fan_speed lsxhl_speeds[] = {
	{
		.rpm = 0,
		.ctrl_val = 3,
	}, {
		.rpm = 1500,
		.ctrl_val = 1,
	}, {
		.rpm = 3250,
		.ctrl_val = 2,
	}, {
		.rpm = 5000,
		.ctrl_val = 0,
	}
};

static int lsxhl_gpio_list[] = {
	LSXHL_GPIO_FAN_HIGH, LSXHL_GPIO_FAN_LOW,
};

static struct gpio_fan_platform_data lsxhl_fan_data = {
	.num_ctrl = ARRAY_SIZE(lsxhl_gpio_list),
	.ctrl = lsxhl_gpio_list,
	.alarm = &lsxhl_alarm,
	.num_speed = ARRAY_SIZE(lsxhl_speeds),
	.speed = lsxhl_speeds,
};

static struct platform_device lsxhl_fan_device = {
	.name = "gpio-fan",
	.id = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &lsxhl_fan_data,
	},
};

/*****************************************************************************
 * GPIO Data
 ****************************************************************************/

static unsigned int lsxhl_mpp_config[] __initdata = {
	MPP10_GPO,	/* HDD Power Enable */
	MPP11_GPIO,	/* USB Vbus Enable */
	MPP18_GPO,	/* FAN High Enable# */
	MPP19_GPO,	/* FAN Low Enable# */
	MPP36_GPIO,	/* Function Blue LED */
	MPP37_GPIO,	/* Alarm LED */
	MPP38_GPIO,	/* Info LED */
	MPP39_GPIO,	/* Power LED */
	MPP40_GPIO,	/* Fan Lock */
	MPP41_GPIO,	/* Function Button */
	MPP42_GPIO,	/* Power Switch */
	MPP43_GPIO,	/* Power Auto Switch */
	MPP48_GPIO,	/* Function Red LED */
	0
};

static void __init lsxhl_init(void)
{
	/*
	 * Basic setup. Needs to be called early.
	 */
	kirkwood_init();
	kirkwood_mpp_conf(lsxhl_mpp_config);

	/*
	 * Configure peripherals.
	 */
	kirkwood_uart0_init();
	kirkwood_ehci_init();
	kirkwood_ge00_init(&lsxhl_ge00_data);
	kirkwood_ge01_init(&lsxhl_ge01_data);
	kirkwood_sata_init(&lsxhl_sata_data);
	kirkwood_spi_init();

	platform_device_register(&lsxhl_leds);
	platform_device_register(&lsxhl_button_device);
	platform_device_register(&lsxhl_fan_device);

	spi_register_board_info(lsxhl_spi_slave_info,
				ARRAY_SIZE(lsxhl_spi_slave_info));

	/* usb power on */
	gpio_set_value(LSXHL_GPIO_USB_POWER, 1);

	pr_info("%s: finished\n", __func__);
}

MACHINE_START(LSXHL, "Buffalo LS-XHL Series")
	.atag_offset	= 0x100,
	.init_machine	= lsxhl_init,
	.map_io		= kirkwood_map_io,
	.init_early	= kirkwood_init_early,
	.init_irq	= kirkwood_init_irq,
	.timer		= &kirkwood_timer,
	.restart	= kirkwood_restart,
MACHINE_END

MACHINE_START(LINKSTATION_CHLV2, "Buffalo LS-CHLv2 Series")
	.atag_offset	= 0x100,
	.init_machine	= lsxhl_init,
	.map_io		= kirkwood_map_io,
	.init_early	= kirkwood_init_early,
	.init_irq	= kirkwood_init_irq,
	.timer		= &kirkwood_timer,
	.restart	= kirkwood_restart,
MACHINE_END
