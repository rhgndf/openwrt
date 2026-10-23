// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom BCM47622 / BCM6755 pinctrl driver
 *
 * Based on pinctrl-bcm4908.c, Copyright (C) 2021 Rafał Miłecki
 *
 * Pins are configured through the GPIO block "test port" interface: the pin
 * number and data are written to TEST_PORT_BLOCK_DATA_LSB and a command is
 * issued: LOAD_MUX_REG selects the pin function, LOAD_PAD_CTRL loads the
 * (complete) pad configuration. Pin/function combinations are taken from the
 * Broadcom SDK 47622-pinctrl.dtsi, function 4 selects plain GPIO.
 *
 * The register block starts at the GPIO PAD_CTRL register, which holds the
 * voltage mode of the xMII (RGMII) pads.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/lockdep.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string_helpers.h>

#include "../core.h"
#include "../pinmux.h"

#define BCM47622_NUM_PINS			108

#define BCM47622_GPIO_FUNC			4

#define BCM47622_PAD_CTRL			0x00
#define  BCM47622_PAD_CTRL_XMII_MODEHV		BIT(8)
#define  BCM47622_PAD_CTRL_XMII_SEL_GMII	BIT(9)
#define  BCM47622_PAD_CTRL_XMII_AMP_EN		BIT(10)
#define BCM47622_TEST_PORT_BLOCK_DATA_MSB	0x14
#define BCM47622_TEST_PORT_BLOCK_DATA_LSB	0x18
#define  BCM47622_TEST_PORT_LSB_PIN		GENMASK(11, 0)
#define  BCM47622_TEST_PORT_LSB_PINMUX		GENMASK(14, 12)
#define  BCM47622_TEST_PORT_LSB_PAD_DRIVE	GENMASK(14, 12)
#define  BCM47622_TEST_PORT_LSB_PAD_AMP_EN	BIT(15)
#define  BCM47622_TEST_PORT_LSB_PAD_IND		BIT(16)
#define  BCM47622_TEST_PORT_LSB_PAD_SEL_GMII	BIT(17)
#define BCM47622_TEST_PORT_COMMAND		0x1c
#define  BCM47622_TEST_PORT_CMD_LOAD_MUX_REG	0x00000021
#define  BCM47622_TEST_PORT_CMD_LOAD_PAD_CTRL	0x00000022

/*
 * RGMII pads (pins of the vendor "rgmii" group). Broadcom code programs the
 * first six of them as RX pads (pad_ind = 0) and the other six as TX pads
 * (pad_ind = 1), see bcm_misc_hw_xmii_pads_init() in the SDK U-Boot
 * (bcm_ethsw_ext.c uses pin_offset 56 for pads 0..11).
 */
#define BCM47622_RGMII_RX_PIN_FIRST		56	/* 56..61: RX pads */
#define BCM47622_RGMII_TX_PIN_FIRST		62	/* 62..67: TX pads */
#define BCM47622_RGMII_PIN_LAST			67

/* Drive strength used when only the power source gets configured */
#define BCM47622_PAD_RGMII_DEFAULT_DRIVE_MA	14
#define BCM47622_PAD_DEFAULT_DRIVE_MA		8

struct bcm47622_pinctrl_pad {
	u8 drive_ma;	/* 0: not configured */
	u16 power_mv;	/* 0: not configured */
};

struct bcm47622_pinctrl {
	struct device *dev;
	void __iomem *base;
	struct mutex mutex; /* serializes test port accesses, protects pads */
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc pctldesc;
	struct bcm47622_pinctrl_pad pads[BCM47622_NUM_PINS];
};

/*
 * Groups & functions
 *
 * Every entry describes one function of the vendor pinctrl table together
 * with the pin group it can be muxed to. The function keeps the vendor name,
 * the group is named "<function>_grp".
 */

struct bcm47622_pinctrl_grp {
	const char *function;
	const char *name;
	const unsigned int *pins;
	unsigned int num_pins;
	unsigned int mux;
};

#define BCM47622_GRP(_func, _mux, ...)					\
	{								\
		.function = #_func,					\
		.name = #_func "_grp",					\
		.pins = (const unsigned int []){ __VA_ARGS__ },		\
		.num_pins = ARRAY_SIZE(((const unsigned int []){ __VA_ARGS__ })), \
		.mux = _mux,						\
	}

static const struct bcm47622_pinctrl_grp bcm47622_pinctrl_grps[] = {
	BCM47622_GRP(spim_group, 1, 72, 73, 74),
	BCM47622_GRP(usb0_pwr, 1, 79, 80),
	BCM47622_GRP(usb1_pwr, 1, 81, 82),
	BCM47622_GRP(rgmii, 1, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67),
	BCM47622_GRP(a_rgmii_mdc, 1, 68),
	BCM47622_GRP(a_rgmii_mdio, 1, 69),
	BCM47622_GRP(nand_data, 1, 43, 44, 45, 46, 47, 48, 49, 50),
	BCM47622_GRP(nand_ctrl, 1, 40, 41, 42, 51, 52, 53),
	BCM47622_GRP(emmc_ctrl, 1, 54, 55),
	BCM47622_GRP(a_pcm_clk, 1, 24),
	BCM47622_GRP(a_pcm_fs, 1, 25),
	BCM47622_GRP(a_pcm_sdin, 1, 22),
	BCM47622_GRP(a_pcm_sdout, 1, 23),
	BCM47622_GRP(a_per_led_00, 2, 0),
	BCM47622_GRP(a_per_led_01, 2, 1),
	BCM47622_GRP(a_per_led_02, 2, 2),
	BCM47622_GRP(a_per_led_03, 2, 3),
	BCM47622_GRP(a_per_led_04, 2, 4),
	BCM47622_GRP(a_per_led_05, 2, 5),
	BCM47622_GRP(a_per_led_06, 2, 6),
	BCM47622_GRP(a_per_led_07, 2, 7),
	BCM47622_GRP(a_per_led_08, 2, 8),
	BCM47622_GRP(a_per_led_09, 2, 9),
	BCM47622_GRP(a_per_led_10, 2, 10),
	BCM47622_GRP(a_per_led_11, 2, 11),
	BCM47622_GRP(a_per_led_12, 2, 12),
	BCM47622_GRP(a_per_led_13, 2, 13),
	BCM47622_GRP(a_per_led_14, 2, 14),
	BCM47622_GRP(a_per_led_15, 2, 15),
	BCM47622_GRP(a_per_led_16, 2, 16),
	BCM47622_GRP(a_per_led_17, 2, 17),
	BCM47622_GRP(a_per_led_18, 2, 18),
	BCM47622_GRP(a_per_led_19, 2, 19),
	BCM47622_GRP(a_per_led_20, 2, 20),
	BCM47622_GRP(a_per_led_21, 2, 21),
	BCM47622_GRP(a_per_led_22, 2, 22),
	BCM47622_GRP(a_per_led_23, 2, 23),
	BCM47622_GRP(a_per_led_24, 2, 24),
	BCM47622_GRP(a_per_led_25, 2, 25),
	BCM47622_GRP(a_per_led_26, 2, 26),
	BCM47622_GRP(a_per_led_27, 2, 27),
	BCM47622_GRP(a_per_led_28, 2, 28),
	BCM47622_GRP(a_per_led_29, 2, 29),
	BCM47622_GRP(a_per_led_30, 2, 30),
	BCM47622_GRP(a_per_led_31, 2, 31),
	BCM47622_GRP(b_pcm_clk, 0, 13),
	BCM47622_GRP(b_pcm_fs, 0, 14),
	BCM47622_GRP(b_pcm_sdin, 0, 11),
	BCM47622_GRP(b_pcm_sdout, 0, 12),
	BCM47622_GRP(b_per_led_00, 2, 40),
	BCM47622_GRP(b_per_led_01, 2, 41),
	BCM47622_GRP(b_per_led_02, 2, 42),
	BCM47622_GRP(b_per_led_03, 2, 43),
	BCM47622_GRP(b_per_led_04, 2, 44),
	BCM47622_GRP(b_per_led_05, 2, 45),
	BCM47622_GRP(b_per_led_06, 2, 46),
	BCM47622_GRP(b_per_led_07, 2, 47),
	BCM47622_GRP(b_per_led_08, 2, 48),
	BCM47622_GRP(b_per_led_09, 2, 49),
	BCM47622_GRP(b_per_led_10, 2, 50),
	BCM47622_GRP(b_per_led_11, 2, 51),
	BCM47622_GRP(b_per_led_12, 2, 52),
	BCM47622_GRP(b_per_led_13, 2, 53),
	BCM47622_GRP(b_per_led_14, 2, 54),
	BCM47622_GRP(b_per_led_15, 2, 55),
	BCM47622_GRP(b_per_led_16, 2, 56),
	BCM47622_GRP(b_per_led_17, 2, 57),
	BCM47622_GRP(b_per_led_18, 2, 58),
	BCM47622_GRP(b_per_led_19, 2, 59),
	BCM47622_GRP(b_per_led_20, 2, 60),
	BCM47622_GRP(b_per_led_21, 2, 61),
	BCM47622_GRP(b_per_led_22, 2, 62),
	BCM47622_GRP(b_per_led_23, 2, 63),
	BCM47622_GRP(b_per_led_24, 2, 64),
	BCM47622_GRP(b_per_led_25, 2, 65),
	BCM47622_GRP(b_per_led_26, 2, 66),
	BCM47622_GRP(b_per_led_27, 2, 67),
	BCM47622_GRP(b_per_led_28, 2, 68),
	BCM47622_GRP(b_per_led_29, 2, 69),
	BCM47622_GRP(ddr_uart_rx, 3, 26),
	BCM47622_GRP(ddr_uart_tx, 3, 25),
	BCM47622_GRP(ext_bp_0, 3, 37),
	BCM47622_GRP(ext_bp_1, 3, 29),
	BCM47622_GRP(hs_uart_cts, 1, 3),
	BCM47622_GRP(hs_uart_rts, 1, 4),
	BCM47622_GRP(hs_uart_sin, 1, 5),
	BCM47622_GRP(hs_uart_sout, 1, 6),
	BCM47622_GRP(i2c_scl, 1, 17),
	BCM47622_GRP(i2c_sda, 1, 16),
	BCM47622_GRP(i2s_lrck, 1, 12),
	BCM47622_GRP(i2s_mclk, 1, 14),
	BCM47622_GRP(i2s_rx_sdata, 1, 13),
	BCM47622_GRP(i2s_sclk, 1, 11),
	BCM47622_GRP(i2s_tx_sdata, 1, 15),
	BCM47622_GRP(nand_wp, 1, 29),
	BCM47622_GRP(ntr_pulse_in, 1, 37),
	BCM47622_GRP(pcie_clkreq_b, 1, 77),
	BCM47622_GRP(pcie_rst_b, 1, 78),
	BCM47622_GRP(pll_byp_clk_00, 5, 84),
	BCM47622_GRP(pll_byp_clk_01, 5, 85),
	BCM47622_GRP(pll_byp_clk_02, 5, 86),
	BCM47622_GRP(pll_byp_clk_03, 5, 87),
	BCM47622_GRP(pll_byp_clk_04, 5, 88),
	BCM47622_GRP(pll_byp_clk_05, 5, 89),
	BCM47622_GRP(pll_byp_clk_06, 5, 90),
	BCM47622_GRP(pll_byp_clk_07, 5, 91),
	BCM47622_GRP(pll_byp_clk_08, 5, 92),
	BCM47622_GRP(pll_byp_clk_09, 5, 93),
	BCM47622_GRP(pll_byp_clk_10, 5, 94),
	BCM47622_GRP(pll_byp_clk_11, 5, 95),
	BCM47622_GRP(reset_out_b, 1, 83),
	BCM47622_GRP(rgmii_rx_ok, 3, 8),
	BCM47622_GRP(rgmii_start_stop, 3, 7),
	BCM47622_GRP(ser_led_clk, 1, 1),
	BCM47622_GRP(ser_led_data, 1, 0),
	BCM47622_GRP(ser_led_mask, 1, 2),
	BCM47622_GRP(sgmii_fiber_detect, 1, 26),
	BCM47622_GRP(sgmii_serdes_mod_def, 3, 9),
	BCM47622_GRP(spim_ss0_b, 1, 75),
	BCM47622_GRP(spim_ss1_b, 1, 76),
	BCM47622_GRP(spim_ss2_b, 1, 10),
	BCM47622_GRP(spim_ss3_b, 1, 9),
	BCM47622_GRP(spim_ss4_b, 1, 8),
	BCM47622_GRP(spim_ss5_b, 1, 7),
	BCM47622_GRP(spis_clk, 1, 18),
	BCM47622_GRP(spis_miso, 1, 20),
	BCM47622_GRP(spis_mosi, 1, 21),
	BCM47622_GRP(spis_ss_b, 1, 19),
	BCM47622_GRP(sys_irq_out, 0, 10),
	BCM47622_GRP(uart0_sdin, 1, 70),
	BCM47622_GRP(uart0_sdout, 1, 71),
	BCM47622_GRP(upi_clk, 3, 3),
	BCM47622_GRP(upi_ena, 3, 1),
	BCM47622_GRP(upi_enb, 3, 2),
	BCM47622_GRP(upi_out, 3, 0),
	BCM47622_GRP(upi_sw_en, 3, 4),
	BCM47622_GRP(usbd_vbus_present, 2, 37),
	BCM47622_GRP(wifi0_c0_femctrl_0, 1, 84),
	BCM47622_GRP(wifi0_c0_femctrl_1, 1, 85),
	BCM47622_GRP(wifi0_c0_femctrl_2, 1, 86),
	BCM47622_GRP(wifi0_c0_femctrl_3, 1, 87),
	BCM47622_GRP(wifi0_c0_femctrl_4, 1, 88),
	BCM47622_GRP(wifi0_c0_femctrl_5, 1, 89),
	BCM47622_GRP(wifi0_c1_femctrl_0, 1, 90),
	BCM47622_GRP(wifi0_c1_femctrl_1, 1, 91),
	BCM47622_GRP(wifi0_c1_femctrl_2, 1, 92),
	BCM47622_GRP(wifi0_c1_femctrl_3, 1, 93),
	BCM47622_GRP(wifi0_c1_femctrl_4, 1, 94),
	BCM47622_GRP(wifi0_c1_femctrl_5, 1, 95),
	BCM47622_GRP(wifi0_gci_gpio_0, 1, 30),
	BCM47622_GRP(wifi0_gci_gpio_1, 1, 31),
	BCM47622_GRP(wifi0_gci_gpio_2, 1, 32),
	BCM47622_GRP(wifi0_gci_gpio_3, 1, 33),
	BCM47622_GRP(wifi0_gci_gpio_4, 1, 34),
	BCM47622_GRP(wifi0_gpio_0, 3, 10),
	BCM47622_GRP(wifi0_gpio_1, 3, 11),
	BCM47622_GRP(wifi0_gpio_10, 3, 24),
	BCM47622_GRP(wifi0_gpio_11, 3, 76),
	BCM47622_GRP(wifi0_gpio_12, 3, 79),
	BCM47622_GRP(wifi0_gpio_13, 3, 80),
	BCM47622_GRP(wifi0_gpio_14, 3, 81),
	BCM47622_GRP(wifi0_gpio_15, 3, 82),
	BCM47622_GRP(wifi0_gpio_2, 3, 12),
	BCM47622_GRP(wifi0_gpio_3, 3, 13),
	BCM47622_GRP(wifi0_gpio_4, 3, 14),
	BCM47622_GRP(wifi0_gpio_5, 3, 15),
	BCM47622_GRP(wifi0_gpio_6, 3, 16),
	BCM47622_GRP(wifi0_gpio_7, 3, 17),
	BCM47622_GRP(wifi0_gpio_8, 3, 22),
	BCM47622_GRP(wifi0_gpio_9, 3, 23),
	BCM47622_GRP(wifi0_scl, 1, 38),
	BCM47622_GRP(wifi0_sda, 1, 39),
	BCM47622_GRP(wifi0_seci_in, 1, 36),
	BCM47622_GRP(wifi0_seci_out, 1, 35),
	BCM47622_GRP(wifi1_c0_femctrl_0, 1, 96),
	BCM47622_GRP(wifi1_c0_femctrl_1, 1, 97),
	BCM47622_GRP(wifi1_c0_femctrl_2, 1, 98),
	BCM47622_GRP(wifi1_c0_femctrl_3, 1, 99),
	BCM47622_GRP(wifi1_c0_femctrl_4, 1, 100),
	BCM47622_GRP(wifi1_c0_femctrl_5, 1, 101),
	BCM47622_GRP(wifi1_c1_femctrl_0, 1, 102),
	BCM47622_GRP(wifi1_c1_femctrl_1, 1, 103),
	BCM47622_GRP(wifi1_c1_femctrl_2, 1, 104),
	BCM47622_GRP(wifi1_c1_femctrl_3, 1, 105),
	BCM47622_GRP(wifi1_c1_femctrl_4, 1, 106),
	BCM47622_GRP(wifi1_c1_femctrl_5, 1, 107),
	BCM47622_GRP(wifi1_gci_gpio_0, 0, 30),
	BCM47622_GRP(wifi1_gci_gpio_1, 0, 31),
	BCM47622_GRP(wifi1_gci_gpio_2, 0, 32),
	BCM47622_GRP(wifi1_gci_gpio_3, 0, 33),
	BCM47622_GRP(wifi1_gci_gpio_4, 0, 34),
	BCM47622_GRP(wifi1_gpio_0, 0, 0),
	BCM47622_GRP(wifi1_gpio_1, 0, 1),
	BCM47622_GRP(wifi1_gpio_10, 0, 25),
	BCM47622_GRP(wifi1_gpio_11, 0, 26),
	BCM47622_GRP(wifi1_gpio_12, 0, 29),
	BCM47622_GRP(wifi1_gpio_13, 0, 37),
	BCM47622_GRP(wifi1_gpio_14, 0, 54),
	BCM47622_GRP(wifi1_gpio_15, 0, 55),
	BCM47622_GRP(wifi1_gpio_2, 0, 2),
	BCM47622_GRP(wifi1_gpio_3, 0, 3),
	BCM47622_GRP(wifi1_gpio_4, 0, 4),
	BCM47622_GRP(wifi1_gpio_5, 0, 5),
	BCM47622_GRP(wifi1_gpio_6, 0, 6),
	BCM47622_GRP(wifi1_gpio_7, 0, 7),
	BCM47622_GRP(wifi1_gpio_8, 0, 8),
	BCM47622_GRP(wifi1_gpio_9, 0, 9),
	BCM47622_GRP(wifi1_scl, 0, 27),
	BCM47622_GRP(wifi1_sda, 0, 28),
	BCM47622_GRP(wifi1_seci_in, 0, 36),
	BCM47622_GRP(wifi1_seci_out, 0, 35),
};

/*
 * Test port
 */

static void bcm47622_pinctrl_test_port_cmd(struct bcm47622_pinctrl *bcm47622_pinctrl,
					   u32 lsb, u32 cmd)
{
	lockdep_assert_held(&bcm47622_pinctrl->mutex);

	writel(0x0, bcm47622_pinctrl->base + BCM47622_TEST_PORT_BLOCK_DATA_MSB);
	writel(lsb, bcm47622_pinctrl->base + BCM47622_TEST_PORT_BLOCK_DATA_LSB);
	writel(cmd, bcm47622_pinctrl->base + BCM47622_TEST_PORT_COMMAND);
}

/*
 * Groups code
 */

static const struct pinctrl_ops bcm47622_pinctrl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

/*
 * Functions code
 */

static void bcm47622_pinctrl_mux_pin(struct bcm47622_pinctrl *bcm47622_pinctrl,
				     unsigned int pin, unsigned int mux)
{
	u32 lsb = FIELD_PREP(BCM47622_TEST_PORT_LSB_PIN, pin) |
		  FIELD_PREP(BCM47622_TEST_PORT_LSB_PINMUX, mux);

	dev_dbg(bcm47622_pinctrl->dev, "setting pin %u to function %u\n", pin, mux);

	guard(mutex)(&bcm47622_pinctrl->mutex);

	bcm47622_pinctrl_test_port_cmd(bcm47622_pinctrl, lsb,
				       BCM47622_TEST_PORT_CMD_LOAD_MUX_REG);
}

static int bcm47622_pinctrl_set_mux(struct pinctrl_dev *pctrl_dev,
				    unsigned int func_selector,
				    unsigned int group_selector)
{
	struct bcm47622_pinctrl *bcm47622_pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	const struct bcm47622_pinctrl_grp *grp;
	const struct function_desc *func_desc;
	const struct group_desc *group_desc;
	unsigned int i;

	func_desc = pinmux_generic_get_function(pctrl_dev, func_selector);
	group_desc = pinctrl_generic_get_group(pctrl_dev, group_selector);
	if (!func_desc || !group_desc)
		return -EINVAL;

	grp = func_desc->data;
	for (i = 0; i < group_desc->grp.npins; i++)
		bcm47622_pinctrl_mux_pin(bcm47622_pinctrl, group_desc->grp.pins[i],
					 grp->mux);

	return 0;
}

static int bcm47622_pinctrl_gpio_request_enable(struct pinctrl_dev *pctrl_dev,
						struct pinctrl_gpio_range *range,
						unsigned int pin)
{
	struct bcm47622_pinctrl *bcm47622_pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	bcm47622_pinctrl_mux_pin(bcm47622_pinctrl, pin, BCM47622_GPIO_FUNC);

	return 0;
}

static const struct pinmux_ops bcm47622_pinctrl_pmxops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = bcm47622_pinctrl_set_mux,
	.gpio_request_enable = bcm47622_pinctrl_gpio_request_enable,
};

/*
 * Pad configuration code
 */

static bool bcm47622_pinctrl_pin_is_rgmii(unsigned int pin)
{
	return pin >= BCM47622_RGMII_RX_PIN_FIRST && pin <= BCM47622_RGMII_PIN_LAST;
}

/* Voltage mode of the xMII pads, shared by all RGMII pins */
static void bcm47622_pinctrl_set_xmii_power(struct bcm47622_pinctrl *bcm47622_pinctrl,
					    unsigned int power_mv)
{
	u32 val;

	lockdep_assert_held(&bcm47622_pinctrl->mutex);

	val = readl(bcm47622_pinctrl->base + BCM47622_PAD_CTRL);
	switch (power_mv) {
	case 1800:
		val &= ~BCM47622_PAD_CTRL_XMII_MODEHV;
		val |= BCM47622_PAD_CTRL_XMII_AMP_EN;
		break;
	case 2500:
		val |= BCM47622_PAD_CTRL_XMII_MODEHV;
		val &= ~(BCM47622_PAD_CTRL_XMII_SEL_GMII | BCM47622_PAD_CTRL_XMII_AMP_EN);
		break;
	case 3300:
		val |= BCM47622_PAD_CTRL_XMII_MODEHV | BCM47622_PAD_CTRL_XMII_SEL_GMII;
		val &= ~BCM47622_PAD_CTRL_XMII_AMP_EN;
		break;
	}
	writel(val, bcm47622_pinctrl->base + BCM47622_PAD_CTRL);
}

/* LOAD_PAD_CTRL always loads the complete pad configuration */
static void bcm47622_pinctrl_write_pad(struct bcm47622_pinctrl *bcm47622_pinctrl,
				       unsigned int pin)
{
	const struct bcm47622_pinctrl_pad *pad = &bcm47622_pinctrl->pads[pin];
	unsigned int drive_ma = pad->drive_ma;
	u32 lsb;

	lockdep_assert_held(&bcm47622_pinctrl->mutex);

	if (!drive_ma)
		drive_ma = bcm47622_pinctrl_pin_is_rgmii(pin) ?
			   BCM47622_PAD_RGMII_DEFAULT_DRIVE_MA :
			   BCM47622_PAD_DEFAULT_DRIVE_MA;

	lsb = FIELD_PREP(BCM47622_TEST_PORT_LSB_PIN, pin) |
	      FIELD_PREP(BCM47622_TEST_PORT_LSB_PAD_DRIVE, drive_ma / 2 - 1);
	if (pad->power_mv == 1800)
		lsb |= BCM47622_TEST_PORT_LSB_PAD_AMP_EN;
	if (pad->power_mv == 3300)
		lsb |= BCM47622_TEST_PORT_LSB_PAD_SEL_GMII;
	if (pin >= BCM47622_RGMII_TX_PIN_FIRST && pin <= BCM47622_RGMII_PIN_LAST)
		lsb |= BCM47622_TEST_PORT_LSB_PAD_IND;

	dev_dbg(bcm47622_pinctrl->dev, "setting pin %u pad control (lsb 0x%08x)\n",
		pin, lsb);

	bcm47622_pinctrl_test_port_cmd(bcm47622_pinctrl, lsb,
				       BCM47622_TEST_PORT_CMD_LOAD_PAD_CTRL);
}

static int bcm47622_pinconf_get(struct pinctrl_dev *pctrl_dev, unsigned int pin,
				unsigned long *config)
{
	struct bcm47622_pinctrl *bcm47622_pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	const struct bcm47622_pinctrl_pad *pad;
	u32 arg;

	guard(mutex)(&bcm47622_pinctrl->mutex);

	pad = &bcm47622_pinctrl->pads[pin];
	switch (param) {
	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = pad->drive_ma;
		break;
	case PIN_CONFIG_POWER_SOURCE:
		arg = pad->power_mv;
		break;
	default:
		return -ENOTSUPP;
	}

	if (!arg)
		return -EINVAL;

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int bcm47622_pinconf_set(struct pinctrl_dev *pctrl_dev, unsigned int pin,
				unsigned long *configs, unsigned int num_configs)
{
	struct bcm47622_pinctrl *bcm47622_pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	struct bcm47622_pinctrl_pad pad;
	bool power_set = false;
	unsigned int i;

	guard(mutex)(&bcm47622_pinctrl->mutex);

	pad = bcm47622_pinctrl->pads[pin];

	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_DRIVE_STRENGTH:
			if (arg < 2 || arg > 16 || arg % 2) {
				dev_err(bcm47622_pinctrl->dev,
					"pin %u: unsupported drive strength %u mA\n", pin, arg);
				return -EINVAL;
			}
			pad.drive_ma = arg;
			break;
		case PIN_CONFIG_POWER_SOURCE:
			if (arg != 1800 && arg != 2500 && arg != 3300) {
				dev_err(bcm47622_pinctrl->dev,
					"pin %u: unsupported power source %u mV\n", pin, arg);
				return -EINVAL;
			}
			pad.power_mv = arg;
			power_set = true;
			break;
		default:
			return -ENOTSUPP;
		}
	}

	bcm47622_pinctrl->pads[pin] = pad;

	if (power_set && bcm47622_pinctrl_pin_is_rgmii(pin))
		bcm47622_pinctrl_set_xmii_power(bcm47622_pinctrl, pad.power_mv);

	bcm47622_pinctrl_write_pad(bcm47622_pinctrl, pin);

	return 0;
}

static int bcm47622_pinconf_group_set(struct pinctrl_dev *pctrl_dev,
				      unsigned int group_selector,
				      unsigned long *configs,
				      unsigned int num_configs)
{
	const unsigned int *pins;
	unsigned int num_pins;
	unsigned int i;
	int err;

	err = pinctrl_generic_get_group_pins(pctrl_dev, group_selector, &pins, &num_pins);
	if (err)
		return err;

	for (i = 0; i < num_pins; i++) {
		err = bcm47622_pinconf_set(pctrl_dev, pins[i], configs, num_configs);
		if (err)
			return err;
	}

	return 0;
}

static const struct pinconf_ops bcm47622_pinctrl_confops = {
	.is_generic = true,
	.pin_config_get = bcm47622_pinconf_get,
	.pin_config_set = bcm47622_pinconf_set,
	.pin_config_group_set = bcm47622_pinconf_group_set,
};

/*
 * Controller code
 */

static const struct pinctrl_desc bcm47622_pinctrl_desc = {
	.name = "bcm47622-pinctrl",
	.pctlops = &bcm47622_pinctrl_ops,
	.pmxops = &bcm47622_pinctrl_pmxops,
	.confops = &bcm47622_pinctrl_confops,
	.owner = THIS_MODULE,
};

static const struct of_device_id bcm47622_pinctrl_of_match_table[] = {
	{ .compatible = "brcm,bcm6755-pinctrl", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm47622_pinctrl_of_match_table);

static int bcm47622_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm47622_pinctrl *bcm47622_pinctrl;
	struct pinctrl_desc *pctldesc;
	struct pinctrl_pin_desc *pins;
	char **pin_names;
	int err;
	unsigned int i;

	bcm47622_pinctrl = devm_kzalloc(dev, sizeof(*bcm47622_pinctrl), GFP_KERNEL);
	if (!bcm47622_pinctrl)
		return -ENOMEM;
	pctldesc = &bcm47622_pinctrl->pctldesc;

	/* Set basic properties */

	bcm47622_pinctrl->dev = dev;

	bcm47622_pinctrl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bcm47622_pinctrl->base))
		return PTR_ERR(bcm47622_pinctrl->base);

	err = devm_mutex_init(dev, &bcm47622_pinctrl->mutex);
	if (err)
		return err;

	memcpy(pctldesc, &bcm47622_pinctrl_desc, sizeof(*pctldesc));

	/* Set pinctrl properties */

	pin_names = devm_kasprintf_strarray(dev, "pin", BCM47622_NUM_PINS);
	if (IS_ERR(pin_names))
		return PTR_ERR(pin_names);

	pins = devm_kcalloc(dev, BCM47622_NUM_PINS, sizeof(*pins), GFP_KERNEL);
	if (!pins)
		return -ENOMEM;
	for (i = 0; i < BCM47622_NUM_PINS; i++) {
		pins[i].number = i;
		pins[i].name = pin_names[i];
	}
	pctldesc->pins = pins;
	pctldesc->npins = BCM47622_NUM_PINS;

	/* Register (groups and functions have to be added before enabling) */

	err = devm_pinctrl_register_and_init(dev, pctldesc, bcm47622_pinctrl,
					     &bcm47622_pinctrl->pctldev);
	if (err)
		return dev_err_probe(dev, err, "Failed to register pinctrl\n");

	/* Groups & functions from the vendor table */

	for (i = 0; i < ARRAY_SIZE(bcm47622_pinctrl_grps); i++) {
		const struct bcm47622_pinctrl_grp *grp = &bcm47622_pinctrl_grps[i];

		err = pinctrl_generic_add_group(bcm47622_pinctrl->pctldev, grp->name,
						grp->pins, grp->num_pins, (void *)grp);
		if (err < 0)
			return dev_err_probe(dev, err, "Failed to add group %s\n", grp->name);

		err = pinmux_generic_add_function(bcm47622_pinctrl->pctldev,
						  grp->function, &grp->name, 1,
						  (void *)grp);
		if (err < 0)
			return dev_err_probe(dev, err, "Failed to add function %s\n",
					     grp->function);
	}

	/* GPIO requests mux their pin through gpio_request_enable */

	return pinctrl_enable(bcm47622_pinctrl->pctldev);
}

static struct platform_driver bcm47622_pinctrl_driver = {
	.probe = bcm47622_pinctrl_probe,
	.driver = {
		.name = "bcm47622-pinctrl",
		.of_match_table = bcm47622_pinctrl_of_match_table,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(bcm47622_pinctrl_driver);

MODULE_DESCRIPTION("Broadcom BCM47622 pinmux driver");
MODULE_LICENSE("GPL");
