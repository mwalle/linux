/* TODO(mw) */
#define DEBUG
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>

#include "cdns-mhdp8501-core.h"
#include "cdns-mhdp8501-afe.h"

struct phy_pll_reg {
	u16 val[7];
	u32 addr;
};

static const struct phy_pll_reg phy_pll_27m_cfg[] = {
	/*  1.62    2.16    2.43    2.7     3.24    4.32    5.4      register address */
	{{ 0x010E, 0x010E, 0x010E, 0x010E, 0x010E, 0x010E, 0x010E }, CDNS_AFE_CMN_PLL0_VCOCAL_INIT_TMR },
	{{ 0x001B, 0x001B, 0x001B, 0x001B, 0x001B, 0x001B, 0x001B }, CDNS_AFE_CMN_PLL0_VCOCAL_ITER_TMR },
	{{ 0x30B9, 0x3087, 0x3096, 0x30B4, 0x30B9, 0x3087, 0x30B4 }, CDNS_AFE_CMN_PLL0_VCOCAL_START },
	{{ 0x0077, 0x009F, 0x00B3, 0x00C7, 0x0077, 0x009F, 0x00C7 }, CDNS_AFE_CMN_PLL0_INTDIV },
	{{ 0xF9DA, 0xF7CD, 0xF6C7, 0xF5C1, 0xF9DA, 0xF7CD, 0xF5C1 }, CDNS_AFE_CMN_PLL0_FRACDIV },
	{{ 0x001E, 0x0028, 0x002D, 0x0032, 0x001E, 0x0028, 0x0032 }, CDNS_AFE_CMN_PLL0_HIGH_THR },
	{{ 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020 }, CDNS_AFE_CMN_PLL0_DSM_DIAG },
	{{ 0x0000, 0x1000, 0x1000, 0x1000, 0x0000, 0x1000, 0x1000 }, CDNS_AFE_CMN_PLLSM0_USER_DEF_CTRL },
	{{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }, CDNS_AFE_CMN_DIAG_PLL0_OVRD },
	{{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }, CDNS_AFE_CMN_DIAG_PLL0_FBH_OVRD },
	{{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }, CDNS_AFE_CMN_DIAG_PLL0_FBL_OVRD },
	{{ 0x0006, 0x0007, 0x0007, 0x0007, 0x0006, 0x0007, 0x0007 }, CDNS_AFE_CMN_DIAG_PLL0_V2I_TUNE },
	{{ 0x0043, 0x0043, 0x0043, 0x0042, 0x0043, 0x0043, 0x0042 }, CDNS_AFE_CMN_DIAG_PLL0_CP_TUNE },
	{{ 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008 }, CDNS_AFE_CMN_DIAG_PLL0_LF_PROG },
	{{ 0x0100, 0x0001, 0x0001, 0x0001, 0x0100, 0x0001, 0x0001 }, CDNS_AFE_CMN_DIAG_PLL0_PTATIS_TUNE1 },
	{{ 0x0007, 0x0001, 0x0001, 0x0001, 0x0007, 0x0001, 0x0001 }, CDNS_AFE_CMN_DIAG_PLL0_PTATIS_TUNE2 },
	{{ 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020 }, CDNS_AFE_CMN_DIAG_PLL0_TEST_MODE},
	{{ 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016 }, CDNS_AFE_CMN_PSM_CLK_CTRL }
};

static const struct phy_pll_reg phy_pll_24m_cfg[] = {
	/*  1.62    2.16    2.43    2.7     3.24    4.32    5.4      register address */
	{{ 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0 }, CDNS_AFE_CMN_PLL0_VCOCAL_INIT_TMR },
	{{ 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018 }, CDNS_AFE_CMN_PLL0_VCOCAL_ITER_TMR },
	{{ 0x3061, 0x3092, 0x30B3, 0x30D0, 0x3061, 0x3092, 0x30D0 }, CDNS_AFE_CMN_PLL0_VCOCAL_START },
	{{ 0x0086, 0x00B3, 0x00CA, 0x00E0, 0x0086, 0x00B3, 0x00E0 }, CDNS_AFE_CMN_PLL0_INTDIV },
	{{ 0xF917, 0xF6C7, 0x75A1, 0xF479, 0xF917, 0xF6C7, 0xF479 }, CDNS_AFE_CMN_PLL0_FRACDIV },
	{{ 0x0022, 0x002D, 0x0033, 0x0038, 0x0022, 0x002D, 0x0038 }, CDNS_AFE_CMN_PLL0_HIGH_THR },
	{{ 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020 }, CDNS_AFE_CMN_PLL0_DSM_DIAG },
	{{ 0x0000, 0x1000, 0x1000, 0x1000, 0x0000, 0x1000, 0x1000 }, CDNS_AFE_CMN_PLLSM0_USER_DEF_CTRL },
	{{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }, CDNS_AFE_CMN_DIAG_PLL0_OVRD },
	{{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }, CDNS_AFE_CMN_DIAG_PLL0_FBH_OVRD },
	{{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }, CDNS_AFE_CMN_DIAG_PLL0_FBL_OVRD },
	{{ 0x0006, 0x0007, 0x0007, 0x0007, 0x0006, 0x0007, 0x0007 }, CDNS_AFE_CMN_DIAG_PLL0_V2I_TUNE },
	{{ 0x0026, 0x0029, 0x0029, 0x0029, 0x0026, 0x0029, 0x0029 }, CDNS_AFE_CMN_DIAG_PLL0_CP_TUNE },
	{{ 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008 }, CDNS_AFE_CMN_DIAG_PLL0_LF_PROG },
	{{ 0x008C, 0x008C, 0x008C, 0x008C, 0x008C, 0x008C, 0x008C }, CDNS_AFE_CMN_DIAG_PLL0_PTATIS_TUNE1 },
	{{ 0x002E, 0x002E, 0x002E, 0x002E, 0x002E, 0x002E, 0x002E }, CDNS_AFE_CMN_DIAG_PLL0_PTATIS_TUNE2 },
	{{ 0x0022, 0x0022, 0x0022, 0x0022, 0x0022, 0x0022, 0x0022 }, CDNS_AFE_CMN_DIAG_PLL0_TEST_MODE},
	{{ 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016 }, CDNS_AFE_CMN_PSM_CLK_CTRL }
};


static int link_rate_index(unsigned int link_rate)
{
	switch (link_rate) {
	case 1620:
		return 0;
	case 2160:
		return 1;
	case 2430:
		return 2;
	case 2700:
		return 3;
	case 3240:
		return 4;
	case 4320:
		return 5;
	case 5400:
		return 6;
	default:
		return -EINVAL;
	}
}

static void dp_aux_cfg(struct cdns_mhdp8501_device *mhdp)
{
	/* Power up Aux */
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TXDA_CYA_AUXDA_CYA, 1);

	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_DIG_CTRL_REG_1, 0x3);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_DIG_CTRL_REG_2, 36);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_2, 0x0100);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_2, 0x0300);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_3, 0x0000);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_1, 0x2008);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_1, 0x2018);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_1, 0xA018);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_2, 0x030C);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_5, 0x0000);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_4, 0x1001);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_1, 0xA098);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_1, 0xA198);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_2, 0x030d);
	ndelay(150);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_ANA_CTRL_REG_2, 0x030f);
}

/* PMA common configuration for 24MHz */
static void dp_phy_pma_cmn_cfg_24mhz(struct cdns_mhdp8501_device *mhdp, unsigned int lanes)
{
	int k;
	u32 val;

	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_PMA_CMN_CTRL1, &val);
	val &= 0xFFF7;
	val |= 0x0008;
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_PMA_CMN_CTRL1, val);

	for (k = 0; k < lanes; k++) {
		/* Transceiver control and diagnostic registers */
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_XCVR_DIAG_LANE_FCM_EN_MGN_TMR | (k << 9), 0x0090);
		/* Transmitter receiver detect registers */
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_RCVDET_EN_TMR | (k << 9), 0x0960);
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_RCVDET_ST_TMR | (k << 9), 0x0030);
	}
}

/* Valid for 24 MHz only */
static void dp_phy_pma_cmn_pll0_24mhz(struct cdns_mhdp8501_device *mhdp, unsigned int lanes, unsigned int link_rate)
{
	u32 val;
	int index, i, k;

	/*
	 * PLL reference clock source select
	 * for single ended reference clock val |= 0x0030;
	 * for differential clock  val |= 0x0000;
	 */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_PMA_CMN_CTRL1, &val);
	val = val & 0xFF8F;
	val = val | 0x0030;
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_PMA_CMN_CTRL1, val);

	/* DP PLL data rate 0/1 clock divider value */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
	val &= 0x00FF;
	if (link_rate <= 2700)
		val |= 0x2400;
	else
		val |= 0x1200;
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, val);

	/* High speed clock 0/1 div */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_CMN_DIAG_HSCLK_SEL, &val);
	val &= 0xFFCC;
	if (link_rate <= 2700)
		val |= 0x0011;
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_DIAG_HSCLK_SEL, val);

	for (k = 0; k < lanes; k = k + 1) {
		cdns_mhdp_reg_read(&mhdp->mbox, (CDNS_AFE_XCVR_DIAG_HSCLK_SEL | (k << 9)), &val);
		val &= 0xCFFF;
		if (link_rate <= 2700)
			val |= 0x1000;
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_XCVR_DIAG_HSCLK_SEL | (k << 9)), val);
	}

	/* DP PHY PLL 24MHz configuration */
	index = link_rate_index(link_rate);
	if (index < 0) {
		dev_err(mhdp->dev, "wrong link rate index\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(phy_pll_24m_cfg); i++)
		cdns_mhdp_reg_write(&mhdp->mbox, phy_pll_24m_cfg[i].addr, phy_pll_24m_cfg[i].val[index]);

	/* Transceiver control and diagnostic registers */
	for (k = 0; k < lanes; k = k + 1) {
		cdns_mhdp_reg_read(&mhdp->mbox, (CDNS_AFE_XCVR_DIAG_PLLDRC_CTRL | (k << 9)), &val);
		val &= 0x8FFF;
		if (link_rate <= 2700)
			val |= 0x2000;
		else
			val |= 0x1000;
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_XCVR_DIAG_PLLDRC_CTRL | (k << 9)), val);
	}

	for (k = 0; k < lanes; k = k + 1) {
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_XCVR_PSM_RCTRL | (k << 9)), 0xBEFC);
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_TX_PSC_A0 | (k << 9)), 0x6799);
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_TX_PSC_A1 | (k << 9)), 0x6798);
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_TX_PSC_A2 | (k << 9)), 0x0098);
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_TX_PSC_A3 | (k << 9)), 0x0098);
	}
}

/* PMA common configuration for 27MHz */
static void dp_phy_pma_cmn_cfg_27mhz(struct cdns_mhdp8501_device *mhdp, int lanes)
{
	u32 val;
	int k;

	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_PMA_CMN_CTRL1, &val);
	val &= 0xFFF7;
	val |= 0x0008;
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_PMA_CMN_CTRL1, val);

	/* Startup state machine registers */
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_SSM_BIAS_TMR, 0x0087);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_PLLSM0_PLLEN_TMR, 0x001B);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_PLLSM0_PLLPRE_TMR, 0x0036);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_PLLSM0_PLLVREF_TMR, 0x001B);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_PLLSM0_PLLLOCK_TMR, 0x006C);

	/* Current calibration registers */
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_ICAL_INIT_TMR, 0x0044);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_ICAL_ITER_TMR, 0x0006);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_ICAL_ADJ_INIT_TMR, 0x0022);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_ICAL_ADJ_ITER_TMR, 0x0006);

	/* Resistor calibration registers */
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_TXPUCAL_INIT_TMR, 0x0022);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_TXPUCAL_ITER_TMR, 0x0006);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_TXPU_ADJ_INIT_TMR, 0x0022);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_TXPU_ADJ_ITER_TMR, 0x0006);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_TXPDCAL_INIT_TMR, 0x0022);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_TXPDCAL_ITER_TMR, 0x0006);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_TXPD_ADJ_INIT_TMR, 0x0022);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_TXPD_ADJ_ITER_TMR, 0x0006);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_RXCAL_INIT_TMR, 0x0022);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_RXCAL_ITER_TMR, 0x0006);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_RX_ADJ_INIT_TMR, 0x0022);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_RX_ADJ_ITER_TMR, 0x0006);

	for (k = 0; k < lanes; k = k + 1) {
		/* Power state machine registers */
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_XCVR_PSM_CAL_TMR  | (k << 9), 0x016D);
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_XCVR_PSM_A0IN_TMR | (k << 9), 0x016D);
		/* Transceiver control and diagnostic registers */
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_XCVR_DIAG_LANE_FCM_EN_MGN_TMR | (k << 9), 0x00A2);
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_DIAG_BGREF_PREDRV_DELAY    | (k << 9), 0x0097);
		/* Transmitter receiver detect registers */
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_RCVDET_EN_TMR | (k << 9), 0x0A8C);
		cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_RCVDET_ST_TMR | (k << 9), 0x0036);
	}
}

static void dp_phy_pma_cmn_pll0_27mhz(struct cdns_mhdp8501_device *mhdp,
				      unsigned int lanes,
				      unsigned int link_rate)
{
	int index, i, k;
	u32 val;

	/*
	 * PLL reference clock source select
	 * for single ended reference clock val |= 0x0030;
	 * for differential clock  val |= 0x0000;
	 */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_PMA_CMN_CTRL1, &val);
	val &= 0xFF8F;
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_PMA_CMN_CTRL1, val);

	/* for differential clock on the refclk_p and refclk_m off chip pins:
	 * CMN_DIAG_ACYA[8]=1'b1
	 */
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_DIAG_ACYA, 0x0100);

	/* DP PLL data rate 0/1 clock divider value */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
	val &= 0x00FF;
	if (link_rate <= 2700)
		val |= 0x2400;
	else
		val |= 0x1200;
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, val);

	/* High speed clock 0/1 div */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_CMN_DIAG_HSCLK_SEL, &val);
	val &= 0xFFCC;
	if (link_rate <= 2700)
		val |= 0x0011;
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_CMN_DIAG_HSCLK_SEL, val);

	for (k = 0; k < lanes; k++) {
		cdns_mhdp_reg_read(&mhdp->mbox, (CDNS_AFE_XCVR_DIAG_HSCLK_SEL | (k << 9)), &val);
		val = val & 0xCFFF;
		if (link_rate <= 2700)
			val |= 0x1000;
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_XCVR_DIAG_HSCLK_SEL | (k << 9)), val);
	}

	/* DP PHY PLL 27MHz configuration */
	index = link_rate_index(link_rate);
	if (index < 0) {
		dev_err(mhdp->dev, "wrong link rate index\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(phy_pll_27m_cfg); i++)
		cdns_mhdp_reg_write(&mhdp->mbox, phy_pll_27m_cfg[i].addr, phy_pll_27m_cfg[i].val[index]);

	/* Transceiver control and diagnostic registers */
	for (k = 0; k < lanes; k++) {
		cdns_mhdp_reg_read(&mhdp->mbox, (CDNS_AFE_XCVR_DIAG_PLLDRC_CTRL | (k << 9)), &val);
		val = val & 0x8FFF;
		if (link_rate <= 2700)
			val |= 0x2000;
		else
			val |= 0x1000;
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_XCVR_DIAG_PLLDRC_CTRL | (k << 9)), val);
	}

	for (k = 0; k < lanes; k = k + 1) {
		/* Power state machine registers */
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_XCVR_PSM_RCTRL | (k << 9)),  0xBEFC);
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_TX_PSC_A0 | (k << 9)), 0x6799);
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_TX_PSC_A1 | (k << 9)), 0x6798);
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_TX_PSC_A2 | (k << 9)), 0x0098);
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_TX_PSC_A3 | (k << 9)), 0x0098);
		/* Receiver calibration power state definition register */
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_RX_PSC_CAL | (k << 9), &val);
		val &= 0xFFBB;
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_RX_PSC_CAL | (k << 9)), val);
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_RX_PSC_A0  | (k << 9), &val);
		val &= 0xFFBB;
		cdns_mhdp_reg_write(&mhdp->mbox, (CDNS_AFE_RX_PSC_A0  | (k << 9)), val);
	}
}

static void dp_phy_power_down(struct cdns_mhdp8501_device *mhdp)
{
	u32 val;
	int i;

#if 0
	if (!mhdp->power_up)
		return;
#endif

	/* Place the PHY lanes in the A3 power state. */
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_MODE_CTRL, 0x8);
	/* Wait for Power State A3 Ack */
	for (i = 0; i < 10; i++) {
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_MODE_CTRL, &val);
		if (val & (1 << 7))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait A3 Ack failed\n");
		return;
	}

	/* Disable HDP PLL’s data rate and full rate clocks out of PMA. */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
	val &= ~(1 << 2);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, val);
	/* Wait for PLL clock gate ACK */
	for (i = 0; i < 10; i++) {
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
		if (!(val & (1 << 3)))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait PLL clock gate Ack failed\n");
		return;
	}

	/* Disable HDP PLL’s for high speed clocks */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
	val &= ~(1 << 0);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, val);
	/* Wait for PLL disable ACK */
	for (i = 0; i < 10; i++) {
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
		if (!(val & (1 << 1)))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait PLL disable Ack failed\n");
		return;
	}
}

static int dp_phy_power_up(struct cdns_mhdp8501_device *mhdp)
{
	u32 val, i;

	/* Enable HDP PLL’s for high speed clocks */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
	val |= (1 << 0);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, val);
	/* Wait for PLL ready ACK */
	for (i = 0; i < 10; i++) {
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
		if (val & (1 << 1))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait PLL Ack failed\n");
		return -1;
	}

	/* Enable HDP PLL’s data rate and full rate clocks out of PMA. */
	cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
	val |= (1 << 2);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, val);
	/* Wait for PLL clock enable ACK */
	for (i = 0; i < 10; i++) {
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_CLK_CTL, &val);
		if (val & (1 << 3))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait PLL clock enable ACk failed\n");
		return -1;
	}

	/* Configure PHY in A2 Mode */
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_MODE_CTRL, 0x0004);
	/* Wait for Power State A2 Ack */
	for (i = 0; i < 10; i++) {
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_MODE_CTRL, &val);
		if (val & (1 << 6))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait A2 Ack failed\n");
		return -1;
	}

	/* Configure PHY in A0 mode (PHY must be in the A0 power
	 * state in order to transmit data)
	 */
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_PHY_HDP_MODE_CTRL, 0x0101);

	/* Wait for Power State A0 Ack */
	for (i = 0; i < 10; i++) {
		cdns_mhdp_reg_read(&mhdp->mbox, CDNS_AFE_PHY_HDP_MODE_CTRL, &val);
		if (val & (1 << 4))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait A0 Ack failed\n");
		return -1;
	}

	//mhdp->power_up = true;

	return 0;
}

static int cdns_afe_init(struct phy *phy)
{
	struct cdns_mhdp8501_device *mhdp = phy_get_drvdata(phy);
printk("%s\n", __func__);
	int ret;

	reset_control_deassert(mhdp->phy_reset);

	/* Disable phy clock if PHY in power up state */
	dp_phy_power_down(mhdp);

	dp_phy_pma_cmn_cfg_27mhz(mhdp, mhdp->link.num_lanes);

	dp_phy_pma_cmn_pll0_27mhz(mhdp, mhdp->link.num_lanes, mhdp->link.rate);

	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_DIAG_ACYA_0, 1);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_DIAG_ACYA_1, 1);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_DIAG_ACYA_2, 1);
	cdns_mhdp_reg_write(&mhdp->mbox, CDNS_AFE_TX_DIAG_ACYA_3, 1);

	/* PHY power up */
	ret = dp_phy_power_up(mhdp);
	if (ret < 0)
		return ret;

	dp_aux_cfg(mhdp);

	return ret;
}

static int cdns_afe_exit(struct phy *phy)
{
	struct cdns_mhdp8501_device *mhdp = phy_get_drvdata(phy);

	reset_control_assert(mhdp->phy_reset);

	return 0;
}

static int cdns_afe_power_on(struct phy *phy)
{
	struct cdns_mhdp8501_device *mhdp = phy_get_drvdata(phy);
printk("%s\n", __func__);

	return 0;
}

static int cdns_afe_power_off(struct phy *phy)
{
	struct cdns_mhdp8501_device *mhdp = phy_get_drvdata(phy);
printk("%s\n", __func__);

	return 0;
}

static int cdns_afe_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct cdns_mhdp8501_device *mhdp = phy_get_drvdata(phy);
	int ret;

printk("%s num_lanes=%d link_rate=%d\n", __func__, mhdp->link.num_lanes, mhdp->link.rate);
printk("%s lanes=%d link_rate=%d\n", __func__, opts->dp.lanes, opts->dp.link_rate);

	dp_phy_power_down(mhdp);

	dp_phy_pma_cmn_cfg_27mhz(mhdp, opts->dp.lanes);
	dp_phy_pma_cmn_pll0_27mhz(mhdp, opts->dp.lanes, opts->dp.link_rate);

	/* PHY power up */
	ret = dp_phy_power_up(mhdp);
	if (ret < 0)
		return ret;

	dp_aux_cfg(mhdp);

	return 0;
}

static const struct phy_ops cdns_afe_phy_ops = {
	.init		= cdns_afe_init,
	.exit		= cdns_afe_exit,
	.power_on	= cdns_afe_power_on,
	.power_off	= cdns_afe_power_off,
	.configure	= cdns_afe_configure,
	.owner		= THIS_MODULE,
};

int cdns_mhdp8501_phy_probe(struct platform_device *pdev)
{
	struct cdns_mhdp8501_device *mhdp = platform_get_drvdata(pdev);

	mhdp->phy_reset = devm_reset_control_get_optional_exclusive(&pdev->dev, "phy");
	if (IS_ERR(mhdp->phy_reset))
	    return PTR_ERR(mhdp->phy_reset);

	mhdp->phy = devm_phy_create(&pdev->dev, NULL, &cdns_afe_phy_ops);
	if (IS_ERR(mhdp->phy))
		return PTR_ERR(mhdp->phy);
	phy_set_drvdata(mhdp->phy, mhdp);

	return 0;
}
