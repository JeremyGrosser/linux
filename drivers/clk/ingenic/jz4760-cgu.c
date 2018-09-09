// SPDX-License-Identifier: GPL-2.0
/*
 * JZ4760 SoC CGU driver
 * Copyright 2018, Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/syscore_ops.h>
#include <dt-bindings/clock/jz4760-cgu.h>
#include "cgu.h"

/* Clock Control Register */
#define CGU_REG_CPCCR		0x00	// Clock Control
#define CGU_REG_LCR			0x04	// Low Power Control
#define CGU_REG_CPPCR0		0x10	// PLL0 Control
#define CGU_REG_CPPSR		0x14	// PLL Switch and Status
#define CGU_REG_CPPCR1		0x30	// PLL1 Control
#define CGU_REG_CPSPR		0x34	// Scratch Pad
#define CGU_REG_CPSPPR		0x38	// Scratch Pad Protected
#define CGU_REG_USBPCR		0x3C	// USB Parameter Control
#define CGU_REG_USBRDT		0x40	// USB Reset Detect Timer
#define CGU_REG_USBVBFIL	0x44	// USB Jitter Filter
#define CGU_REG_USBCDR		0x50	// OTG PHY clock divider
#define CGU_REG_I2SCDR		0x60	// I2S clock divider
#define CGU_REG_LPCDR		0x64	// LCD pixel clock divider
#define CGU_REG_MSCCDR		0x68	// MSC clock divider
#define CGU_REG_UHCCDR		0x6C	// UHC 48M clock divider
#define CGU_REG_SSICDR		0x74	// SSI clock divider
#define CGU_REG_CIMCDR		0x7C	// CIM MCLK clock divider
#define CGU_REG_GPSCDR		0x80	// GPS clock divider
#define CGU_REG_PCMCDR		0x84	// PCM device clock divider
#define CGU_REG_GPUCDR		0x88	// GPU clock divider
#define CGU_REG_PSWC0ST		0x90	// Power Switch Chain 0 Start Time
#define CGU_REG_PSWC1ST		0x94	// Power Switch Chain 1 Start Time
#define CGU_REG_PSWC2ST		0x98	// Power Switch Chain 2 Start Time
#define CGU_REG_PSWC3ST		0x9c	// Power Switch Chain 3 Start Time
#define CGU_REG_CLKGR0		0x20	// Clock Gate Register 0
#define CGU_REG_OPCR		0x24	// Oscillator and Power Control
#define CGU_REG_CLKGR1		0x28	// Clock Gate Register 1 (VPU/AHB1)

/* bits within the LCR register */
#define LCR_LPM				BIT(0)

/* bits within the OPCR register */
#define OPCR_SPENDH			BIT(5)

/* bits within the USBPCR register */
#define USBPCR_UHC_POWER	BIT(20)		/* UHC PHY power down */

static struct ingenic_cgu *cgu;

static int jz4760_uhc_phy_enable(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;
	void __iomem *reg_usbpcr	= cgu->base + CGU_REG_USBPCR;

	writel(readl(reg_opcr) & ~OPCR_SPENDH, reg_opcr);
	writel(readl(reg_usbpcr) | USBPCR_UHC_POWER, reg_usbpcr);
	return 0;
}

static void jz4760_uhc_phy_disable(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;
	void __iomem *reg_usbpcr	= cgu->base + CGU_REG_USBPCR;

	writel(readl(reg_usbpcr) & ~USBPCR_UHC_POWER, reg_usbpcr);
	writel(readl(reg_opcr) | OPCR_SPENDH, reg_opcr);
}

static int jz4760_uhc_phy_is_enabled(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;
	void __iomem *reg_usbpcr	= cgu->base + CGU_REG_USBPCR;

	return !(readl(reg_opcr) & OPCR_SPENDH) &&
		(readl(reg_usbpcr) & USBPCR_UHC_POWER);
}

static const struct clk_ops jz4760_uhc_phy_ops = {
	.enable = jz4760_uhc_phy_enable,
	.disable = jz4760_uhc_phy_disable,
	.is_enabled = jz4760_uhc_phy_is_enabled,
};

static const s8 pll_od_encoding[8] = {
	0x0, 0x1, -1, 0x2, -1, -1, -1, 0x3,
};

static const struct ingenic_cgu_clk_info jz4760_cgu_clocks[] = {

	/* External clocks */

	[JZ4760_CLK_EXT] = { "ext", CGU_CLK_EXT },
	[JZ4760_CLK_RTC] = { "rtc", CGU_CLK_EXT },

	/* PLLs */

	[JZ4760_CLK_PLL0] = {
		"pll0", CGU_CLK_PLL,
		.parents = { JZ4760_CLK_EXT },
		.pll = {
			.reg = CGU_REG_CPPCR0,
			.m_shift = 24,
			.m_bits = 7,
			.m_offset = 2,
			.n_shift = 18,
			.n_bits = 4,
			.n_offset = 1,
			.od_shift = 16,
			.od_bits = 2,
			.od_max = 8,
			.od_encoding = pll_od_encoding,
			.bypass_bit = 9,
			.enable_bit = 8,
			.stable_bit = 10,
		},
	},

	[JZ4760_CLK_PLL1] = {
		/* TODO: PLL1 can depend on PLL0 */
		"pll1", CGU_CLK_PLL,
		.parents = { JZ4760_CLK_EXT },
		.pll = {
			.reg = CGU_REG_CPPCR1,
			.m_shift = 24,
			.m_bits = 7,
			.m_offset = 1,
			.n_shift = 18,
			.n_bits = 4,
			.n_offset = 1,
			.od_shift = 16,
			.od_bits = 2,
			.od_max = 8,
			.od_encoding = pll_od_encoding,
			.enable_bit = 7,
			.stable_bit = 6,
			.no_bypass_bit = true,
		},
	},

	/* Main clocks */

	[JZ4760_CLK_CCLK] = {
		"cclk", CGU_CLK_DIV,
		.parents = { JZ4760_CLK_PLL0, },
		.div = { CGU_REG_CPCCR, 0, 1, 4, 22, -1, -1 },
	},
	[JZ4760_CLK_HCLK] = {
		"hclk", CGU_CLK_DIV,
		.parents = { JZ4760_CLK_PLL0, },
		.div = { CGU_REG_CPCCR, 4, 1, 4, 22, -1, -1 },
	},
	[JZ4760_CLK_H2CLK] = {
		"h2clk", CGU_CLK_DIV,
		.parents = { JZ4760_CLK_PLL0, },
		.div = { CGU_REG_CPCCR, 16, 1, 4, 22, -1, -1 },
	},
	[JZ4760_CLK_PCLK] = {
		"pclk", CGU_CLK_DIV,
		.parents = { JZ4760_CLK_PLL0, },
		.div = { CGU_REG_CPCCR, 8, 1, 4, 22, -1, -1 },
	},
	[JZ4760_CLK_MCLK] = {
		"mclk", CGU_CLK_DIV,
		.parents = { JZ4760_CLK_PLL0, },
		.div = { CGU_REG_CPCCR, 12, 1, 4, 22, -1, -1 },
	},
	[JZ4760_CLK_SCLK] = {
		"sclk", CGU_CLK_DIV,
		.parents = { JZ4760_CLK_PLL0, },
		.div = { CGU_REG_CPCCR, 24, 1, 4, 22, -1, -1 },
	},

	/* Those divided clocks can connect to PLL0 or PLL1 */

	[JZ4760_CLK_I2S] = {
		"i2s", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, -1,
			JZ4760_CLK_PLL0, JZ4760_CLK_PLL1 },
		.mux = { CGU_REG_I2SCDR, 30, 2 },
		.div = { CGU_REG_I2SCDR, 0, 1, 9, -1, -1, -1 },
	},
	[JZ4760_CLK_OTG] = {
		"usb", CGU_CLK_DIV | CGU_CLK_GATE | CGU_CLK_MUX,
		.parents = { JZ4760_CLK_EXT, -1,
			JZ4760_CLK_PLL0, JZ4760_CLK_PLL1 },
		.mux = { CGU_REG_USBCDR, 30, 2 },
		.div = { CGU_REG_USBCDR, 0, 1, 8, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 2 },
	},
	[JZ4760_CLK_LP] = {
		"lcd", CGU_CLK_DIV | CGU_CLK_GATE | CGU_CLK_MUX,
		.parents = { JZ4760_CLK_PLL0, JZ4760_CLK_PLL1, },
		.mux = { CGU_REG_LPCDR, 29, 1 },
		.div = { CGU_REG_LPCDR, 0, 1, 11, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 28 },
	},
	[JZ4760_CLK_MSC0_MUX] = {
		"mmc0_mux", CGU_CLK_DIV | CGU_CLK_GATE | CGU_CLK_MUX,
		.parents = { JZ4760_CLK_PLL0, JZ4760_CLK_PLL1, },
		.mux = { CGU_REG_MSCCDR, 31, 1 },
		.div = { CGU_REG_MSCCDR, 0, 1, 5, -1, -1, 31 },
		.gate = { CGU_REG_CLKGR0, 3 },
	},
	[JZ4760_CLK_UHC] = {
		"uhc", CGU_CLK_DIV | CGU_CLK_GATE | CGU_CLK_MUX,
		.parents = { JZ4760_CLK_PLL0, JZ4760_CLK_PLL1, },
		.mux = { CGU_REG_UHCCDR, 31, 1 },
		.div = { CGU_REG_UHCCDR, 0, 1, 4, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 24 },
	},
	[JZ4760_CLK_SSI0_MUX] = {
		"ssi0_mux", CGU_CLK_DIV | CGU_CLK_MUX | CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, JZ4760_CLK_PLL0 },
		.mux = { CGU_REG_SSICDR, 31, 1 },
		.div = { CGU_REG_SSICDR, 0, 1, 6, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 4 },
	},
	[JZ4760_CLK_CIM] = {
		"cim", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4760_CLK_PLL0 },
		.div = { CGU_REG_CIMCDR, 0, 1, 8, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 26 },
	},
	[JZ4760_CLK_GPS] = {
		"gps", CGU_CLK_DIV | CGU_CLK_GATE | CGU_CLK_MUX,
		.parents = { JZ4760_CLK_PLL0, JZ4760_CLK_PLL1, },
		.mux = { CGU_REG_GPSCDR, 31, 1 },
		.div = { CGU_REG_GPSCDR, 0, 1, 4, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 22 },
	},
	[JZ4760_CLK_PCM] = {
		"pcm", CGU_CLK_DIV | CGU_CLK_MUX | CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, -1,
			JZ4760_CLK_PLL0, JZ4760_CLK_PLL1 },
		.mux = { CGU_REG_PCMCDR, 30, 2 },
		.div = { CGU_REG_PCMCDR, 0, 1, 9, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR1, 8 },
	},
	[JZ4760_CLK_GPU] = {
		"gpu", CGU_CLK_DIV | CGU_CLK_GATE | CGU_CLK_MUX,
		.parents = { JZ4760_CLK_EXT, -1,
			JZ4760_CLK_PLL0, JZ4760_CLK_PLL1 },
		.mux = { CGU_REG_GPUCDR, 30, 2 },
		.div = { CGU_REG_GPUCDR, 0, 1, 9, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR1, 9 },
	},

	/* Gate-only clocks */
	[JZ4760_CLK_NEMC] = {
		"nemc", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 0 },
	},
	[JZ4760_CLK_BCH] = {
		"bch", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 1 },
	},
	[JZ4760_CLK_I2C0] = {
		"i2c0", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 5 },
	},
	[JZ4760_CLK_I2C1] = {
		"i2c1", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 6 },
	},
	[JZ4760_CLK_SCC] = {
		"scc", CGU_CLK_SCC,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 7 },
	},
	[JZ4760_CLK_AIC] = {
		"aic", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 8 },
	},
	[JZ4760_CLK_TSSI] = {
		"tssi", CGU_CLK_TSSI,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 9 },
	},
	[JZ4760_CLK_OWI] = {
		"w1", CGU_CLK_OWI,
		.parents = { JZ4760_CLK_EXT },
		.gate = { CGU_REG_CLKGR0, 10 },
	},
	[JZ4760_CLK_MSC1] = {
		"mmc1", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_MSC0_MUX, },
		.gate = { CGU_REG_CLKGR0, 11 },
	},
	[JZ4760_CLK_MSC2] = {
		"mmc2", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_MSC0_MUX, },
		.gate = { CGU_REG_CLKGR0, 12 },
	},
	[JZ4760_CLK_KBC] = {
		"kbd", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 13 },
	},
	[JZ4760_CLK_SADC] = {
		"sadc", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 14 },
	},
	[JZ4760_CLK_UART0] = {
		"uart0", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 15 },
	},
	[JZ4760_CLK_UART1] = {
		"uart1", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 16 },
	},
	[JZ4760_CLK_UART2] = {
		"uart2", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 17 },
	},
	[JZ4760_CLK_UART3] = {
		"uart3", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT, },
		.gate = { CGU_REG_CLKGR0, 18 },
	},
	[JZ4760_CLK_SSI1] = {
		"ssi1", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_SSI0_MUX, },
		.gate = { CGU_REG_CLKGR0, 19 },
	},
	[JZ4760_CLK_SSI2] = {
		"ssi2", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_SSI0_MUX, },
		.gate = { CGU_REG_CLKGR0, 20 },
	},
	[JZ4760_CLK_DMA] = {
		"dma", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_H2CLK, },
		.gate = { CGU_REG_CLKGR0, 21 },
	},
	[JZ4760_CLK_MDMA] = {
		"mdma", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_H2CLK, },
		.gate = { CGU_REG_CLKGR0, 25 },
	},
	[JZ4760_CLK_TVE] = {
		"tve", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_EXT },
		.gate = { CGU_REG_CLKGR0, 27 },
	},
	[JZ4760_CLK_IPU] = {
		"ipu", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR0, 29 },
	},
	[JZ4760_CLK_DDR] = {
		"ddr", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR0, 30 },
	},
	[JZ4760_CLK_BDMA] = {
		"vpu_bdma", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR1, 0 },
	},
	[JZ4760_CLK_MC] = {
		"vpu_mc", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR1, 1 },
	},
	[JZ4760_CLK_DBLK] = {
		"vpu_dblk", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR1, 2 },
	},
	[JZ4760_CLK_ME] = {
		"vpu_me", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR1, 3 },
	},
	[JZ4760_CLK_DCT] = {
		"vpu_dct", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR1, 4 },
	},
	[JZ4760_CLK_SRAM] = {
		"vpu_sram", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR1, 5 },
	},
	[JZ4760_CLK_CABAC] = {
		"vpu_cabac", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR1, 6 },
	},
	[JZ4760_CLK_AHB1] = {
		"ahb1", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_HCLK, },
		.gate = { CGU_REG_CLKGR1, 7 },
	},

	},
	[JZ4760_CLK_OTG_PHY] = {
		"usb_phy", CGU_CLK_GATE,
		.parents = { JZ4760_CLK_OTG },
		.gate = { CGU_REG_OPCR, 7, true, 50 },
	},

	/* Custom clocks */

	[JZ4760_CLK_UHC_PHY] = {
		"uhc_phy", CGU_CLK_CUSTOM,
		.parents = { JZ4760_CLK_UHC, -1, -1, -1 },
		.custom = { &jz4760_uhc_phy_ops },
	},

	[JZ4760_CLK_EXT512] = {
		"ext/512", CGU_CLK_FIXDIV,
		.parents = { JZ4760_CLK_EXT },
		.fixdiv = { 512 },
	},

	[JZ4760_CLK_RTC] = {
		"rtc", CGU_CLK_MUX,
		.parents = { JZ4760_CLK_EXT512, JZ4760_CLK_OSC32K, },
		.mux = { CGU_REG_OPCR, 2, 1},
	},
};

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int jz4760_cgu_pm_suspend(void)
{
	u32 val;

	val = readl(cgu->base + CGU_REG_LCR);
	writel(val | LCR_LPM, cgu->base + CGU_REG_LCR);
	return 0;
}

static void jz4760_cgu_pm_resume(void)
{
	u32 val;

	val = readl(cgu->base + CGU_REG_LCR);
	writel(val & ~LCR_LPM, cgu->base + CGU_REG_LCR);
}

static struct syscore_ops jz4760_cgu_pm_ops = {
	.suspend = jz4760_cgu_pm_suspend,
	.resume = jz4760_cgu_pm_resume,
};
#endif /* CONFIG_PM_SLEEP */

static void __init jz4760_cgu_init(struct device_node *np)
{
	int retval;

	cgu = ingenic_cgu_new(jz4760_cgu_clocks,
			      ARRAY_SIZE(jz4760_cgu_clocks), np);
	if (!cgu)
		pr_err("%s: failed to initialise CGU\n", __func__);

	retval = ingenic_cgu_register_clocks(cgu);
	if (retval)
		pr_err("%s: failed to register CGU Clocks\n", __func__);

#if IS_ENABLED(CONFIG_PM_SLEEP)
	register_syscore_ops(&jz4760_cgu_pm_ops);
#endif
}

/* We only probe via devicetree, no need for a platform driver */
CLK_OF_DECLARE(jz4760_cgu, "ingenic,jz4760-cgu", jz4760_cgu_init);
