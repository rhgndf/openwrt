// SPDX-License-Identifier: GPL-2.0-only
/*
 * Broadcom BCA SYSTEMPORT v2 Ethernet driver
 *
 * SYSTEMPORT v2 is used in several BCA SoCs (BCM47622 / BCM6755, BCM63178,
 * BCM6756), each with small register differences. It is a different
 * register layout than the STB SYSTEMPORT of bcmsysport.c. Only the
 * BCM47622 is supported so far.
 *
 * The BCM47622 has two SYSTEMPORT v2 instances, each with its own UniMAC.
 * A shared "misc" block controls the internal GPHY, the RGMII interface and
 * the crossbar that routes the GPHY or RGMII port to a SYSTEMPORT, and it
 * hosts the MDIO controller.
 *
 * Register layout and init/stop sequences follow the Broadcom BCA U-Boot
 * driver (bcmbca_sysport_v2.c, bcm_ethsw_phy.c, brcm_rgmii.c). This driver
 * only uses RDMA ring 0 with descriptors in local RAM and TDMA queue 0 fed
 * through its descriptor write port.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/u64_stats_sync.h>
#include <linux/workqueue.h>

#include "unimac.h"

/* Shared misc block */
#define MISC_XBAR_CNTRL			0x04
#define  MISC_XBAR_SYSPORT_RGMII(i)	BIT(i)
#define MISC_RGMII_CNTRL		0x20
#define  RGMII_MODE_EN			BIT(0)
#define  RGMII_ID_MODE_DIS		BIT(1)
#define  RGMII_PORT_MODE		GENMASK(4, 2)
#define  RGMII_PORT_MODE_RGMII		3
#define MISC_RGMII_IB_STATUS		0x24
#define  IB_STATUS_SPEED		GENMASK(1, 0)
#define  IB_STATUS_SPEED_10		0
#define  IB_STATUS_SPEED_100		1
#define  IB_STATUS_SPEED_1000		2
#define  IB_STATUS_DUPLEX		BIT(2)
#define  IB_STATUS_LINK			BIT(3)
#define  IB_STATUS_OVRD			BIT(4)
#define MISC_RGMII_RX_CLOCK_DELAY	0x28
#define  RX_CLOCK_DELAY_ON		0xc8
#define  RX_CLOCK_DELAY_BYPASS		0xf8	/* IDDQ | BYPASS */
#define MISC_SPHY_TEST_CNTRL		0xbc
#define MISC_SPHY_CNTRL			0xc0
#define  SPHY_IDDQ_BIAS			BIT(0)
#define  SPHY_EXT_PWR_DOWN		BIT(1)
#define  SPHY_IDDQ_GLOBAL_PWR		BIT(3)
#define  SPHY_RESET			BIT(5)
#define  SPHY_PHYAD			GENMASK(12, 8)
#define MISC_MDIO_CFG			0x304
#define  MDIO_CFG_C22			BIT(0)
#define  MDIO_CFG_FREE_RUN_CLK		BIT(13)

/* GPHY power-up workaround delay (phy_wkard_timeout in the vendor DT) */
#define GPHY_WKARD_DELAY_MS		25

/* SYSTEMPORT blocks */
#define SP_TOPCTRL			0x0000
#define  TOPCTRL_REV			0x00
#define  TOPCTRL_RX_FLUSH		0x04
#define  TOPCTRL_TX_FLUSH		0x08

#define SP_RBUF				0x0400
#define  RBUF_CONTROL			0x00
#define   RBUF_RSB_MODE			GENMASK(1, 0)
#define   RBUF_4B_ALIGN			BIT(2)
#define   RBUF_BTAG_STRIP		BIT(3)
#define   RBUF_BAD_PKT_DISCARD		BIT(4)
#define  RBUF_PKT_READY_THRESH		0x04

#define SP_UMAC				0x0800
/* 47622 specific UMAC_CMD bits, set by the vendor flow control code */
#define  CMD_SW_OVERRIDE_TX		BIT(17)
#define  CMD_SW_OVERRIDE_RX		BIT(18)

#define SP_RDMA				0x2000
#define  RDMA_DESC(i)			((i) * 8)
#define  RDMA_CONTROL			0x1080
#define   RDMA_EN			BIT(0)
#define  RDMA_STATUS			0x1084
#define   RDMA_STATUS_DISABLED		BIT(0)
#define   RDMA_STATUS_DESC_RAM_BUSY	BIT(1)
#define  RDMA_LOCRAM_DESCRING_SIZE(r)	(0x10b0 + (r) * 4)
#define   RDMA_RING_EN			BIT(17)
#define  RDMA_PKTBUF_SIZE(r)		(0x10d0 + (r) * 4)
#define  RDMA_PINDEX(r)			(0x10f0 + (r) * 4)
#define  RDMA_CINDEX(r)			(0x1110 + (r) * 4)
#define   RDMA_INDEX			GENMASK(15, 0)
#define  RDMA_BUFDONE_INTR_CFG(r)	(0x1130 + (r) * 4)
#define  RDMA_DDR_DESC_RING_START(i)	(0x1170 + (i) * 4)
#define  RDMA_DDR_DESC_RING_SIZE(r)	(0x11b0 + (r) * 4)

#define SP_TDMA				0x4000
#define  TDMA_WRITE_PORT_LO(q)		((q) * 8)
#define  TDMA_WRITE_PORT_HI(q)		((q) * 8 + 4)
#define  TDMA_DESC_RING_CONTROL(q)	(0x200 + (q) * 4)
#define   TDMA_RING_EN			BIT(1)
#define  TDMA_MAX_THRESHOLD(q)		(0x380 + (q) * 4)
#define   TDMA_MAX_THRESHOLD_DEFAULT	0x00100009
#define  TDMA_INTR_CONTROL(q)		(0x400 + (q) * 4)
#define  TDMA_PROD_CONS(q)		(0x480 + (q) * 4)
#define   TDMA_PROD			GENMASK(15, 0)
#define   TDMA_CONS			GENMASK(31, 16)
#define  TDMA_MAPPING(q)		(0x500 + (q) * 4)
#define   TDMA_MAP_IGNORE_STATUS	BIT(6)
#define  TDMA_DDR_DESC_RING_START(i)	(0x600 + (i) * 4)
#define  TDMA_DDR_DESC_RING_SIZE(q)	(0x700 + (q) * 4)
#define  TDMA_DDR_DESC_RING_PUSH_TIMER(q) (0x800 + (q) * 4)
#define  TDMA_CONTROL			0x884
#define   TDMA_EN			BIT(0)
#define   TDMA_TSB_EN			BIT(1)
#define   TDMA_RING_CFG			BIT(19)
#define   TDMA_ACB_EN			BIT(27)
#define  TDMA_STATUS			0x888
#define   TDMA_STATUS_DISABLED		BIT(0)
#define   TDMA_STATUS_LL_RAM_INIT_BUSY	BIT(1)
#define  TDMA_TIER1_ARB_CTRL(i)		(0x890 + (i) * 4)
#define  TDMA_TIER1_ARB_QUEUE_EN(i)	(0x8b0 + (i) * 4)
#define  TDMA_TIER2_ARB_CTRL		0x8d0

/* INTRL2[k] / INTC[k] drive interrupt line k of the SYSTEMPORT */
#define SP_NUM_INTRL2			5
#define SP_INTRL2(k)			(0x8200 + (k) * 0x20)
#define  INTRL2_STATUS			0x00
#define  INTRL2_CLEAR			0x08
#define  INTRL2_MASK_STATUS		0x0c
#define  INTRL2_MASK_SET		0x10
#define  INTRL2_MASK_CLEAR		0x14
#define SP_INTC(k)			(0x8300 + (k) * 0x20)
#define  INTC_MAPPING_1			0x00
#define  INTC_MAPPING_0			0x04
#define  INTC_PSB_EN			0x08
#define  INTC_CSB_EN			0x0c
#define  INTC_MAP_TDMA			0
#define  INTC_MAP_RDMA			1

#define SP_IRQ_RX			0
#define SP_IRQ_TX			1
#define SP_INTR_RING0			BIT(0)

/* DMA descriptor word 1, status bits as in the upstream bcmsysport.h */
#define DESC_ADDR_HI			GENMASK(7, 0)
#define DESC_TX_APPEND_CRC		BIT(11)
#define DESC_RX_ERR			BIT(12)
#define DESC_RX_OVERFLOW		BIT(13)
#define DESC_SOP			BIT(16)
#define DESC_EOP			BIT(17)
#define DESC_LEN			GENMASK(31, 18)

#define SP_RX_RING_SIZE			128	/* local descriptor RAM limit */
#define SP_RX_BUF_SIZE			2048
#define SP_RX_BUF_OFFSET		NET_SKB_PAD
/* RBUF_4B_ALIGN puts 2 bytes in front of each frame, so the IP header is aligned */
#define SP_RX_ALIGN_PAD			2
#define SP_RX_FRAG_SIZE			(SKB_DATA_ALIGN(SP_RX_BUF_OFFSET + SP_RX_BUF_SIZE) + \
					 SKB_DATA_ALIGN(sizeof(struct skb_shared_info)))
#define SP_TX_RING_SIZE			8
/* Including FCS, a second VLAN tag on top of it still fits one buffer */
#define SP_MAX_FRAME_LEN		2040

struct bcm47622_misc {
	void __iomem *base;
};

struct bcmbca_sp_rx_slot {
	void *buf;
	dma_addr_t dma;
};

struct bcmbca_sp_tx_slot {
	struct sk_buff *skb;
	dma_addr_t dma;
};

struct bcmbca_sp {
	struct device *dev;
	struct net_device *ndev;
	void __iomem *base;
	struct bcm47622_misc *misc;

	struct phylink *phylink;
	struct phylink_config phylink_config;

	struct napi_struct napi;
	int rx_irq;
	int tx_irq;
	bool up;
	struct work_struct reset_work;

	struct bcmbca_sp_rx_slot rx_slots[SP_RX_RING_SIZE];
	u16 rx_cons;
	bool crc_fwd;

	spinlock_t tx_lock;	/* protects tx_slots, tx_prod and tx_cons */
	struct bcmbca_sp_tx_slot tx_slots[SP_TX_RING_SIZE];
	u16 tx_prod;
	u16 tx_cons;
};

/*
 * Misc block
 */

/* The PHY driver resets the GPHY and sets up its AFE */
static void bcm47622_misc_gphy_init(struct bcm47622_misc *misc, u32 addr)
{
	void __iomem *ctrl = misc->base + MISC_SPHY_CNTRL;
	void __iomem *test = misc->base + MISC_SPHY_TEST_CNTRL;
	u32 val;

	/* Power-on workaround */
	writel(readl(ctrl) | SPHY_RESET, ctrl);
	msleep(GPHY_WKARD_DELAY_MS);
	writel(1, test);
	writel(readl(ctrl) & ~(SPHY_IDDQ_BIAS | SPHY_IDDQ_GLOBAL_PWR), ctrl);
	msleep(GPHY_WKARD_DELAY_MS);
	writel(readl(ctrl) | SPHY_IDDQ_BIAS | SPHY_IDDQ_GLOBAL_PWR, ctrl);
	msleep(GPHY_WKARD_DELAY_MS);
	writel(readl(ctrl) & ~SPHY_RESET, ctrl);
	msleep(GPHY_WKARD_DELAY_MS);
	writel(0, test);

	/* Power up */
	val = readl(ctrl);
	val &= ~(SPHY_IDDQ_BIAS | SPHY_EXT_PWR_DOWN | SPHY_PHYAD);
	val |= SPHY_RESET | FIELD_PREP(SPHY_PHYAD, addr);
	writel(val, ctrl);
	udelay(1);
	val &= ~SPHY_IDDQ_GLOBAL_PWR;
	writel(val, ctrl);
	usleep_range(1000, 2000);
	val &= ~SPHY_RESET;
	writel(val, ctrl);
	usleep_range(1000, 2000);
}

static void bcm47622_misc_rgmii_init(struct bcm47622_misc *misc, u32 sysport,
				     bool tx_delay, bool rx_delay)
{
	u32 val;

	val = readl(misc->base + MISC_XBAR_CNTRL);
	val &= ~(MISC_XBAR_SYSPORT_RGMII(0) | MISC_XBAR_SYSPORT_RGMII(1));
	val |= MISC_XBAR_SYSPORT_RGMII(sysport);
	writel(val, misc->base + MISC_XBAR_CNTRL);

	val = readl(misc->base + MISC_RGMII_CNTRL);
	val |= RGMII_MODE_EN;
	val &= ~RGMII_PORT_MODE;
	val |= FIELD_PREP(RGMII_PORT_MODE, RGMII_PORT_MODE_RGMII);
	if (tx_delay)
		val &= ~RGMII_ID_MODE_DIS;
	else
		val |= RGMII_ID_MODE_DIS;
	writel(val, misc->base + MISC_RGMII_CNTRL);

	writel(rx_delay ? RX_CLOCK_DELAY_ON : RX_CLOCK_DELAY_BYPASS,
	       misc->base + MISC_RGMII_RX_CLOCK_DELAY);
}

static void bcm47622_misc_set_ib_status(struct bcm47622_misc *misc, bool link,
					int speed, int duplex)
{
	u32 val = IB_STATUS_OVRD;

	if (link) {
		val |= IB_STATUS_LINK;
		if (duplex == DUPLEX_FULL)
			val |= IB_STATUS_DUPLEX;
		if (speed == SPEED_1000)
			val |= FIELD_PREP(IB_STATUS_SPEED, IB_STATUS_SPEED_1000);
		else if (speed == SPEED_100)
			val |= FIELD_PREP(IB_STATUS_SPEED, IB_STATUS_SPEED_100);
		else
			val |= FIELD_PREP(IB_STATUS_SPEED, IB_STATUS_SPEED_10);
	}

	writel(val, misc->base + MISC_RGMII_IB_STATUS);
}

static int bcm47622_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct bcm47622_misc *misc;
	u32 val;
	int err;

	misc = devm_kzalloc(dev, sizeof(*misc), GFP_KERNEL);
	if (!misc)
		return -ENOMEM;

	misc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(misc->base))
		return PTR_ERR(misc->base);

	/*
	 * Clause 22 mode is needed by the internal GPHY and by
	 * mdio-bcm-unimac, which never sets it.
	 */
	val = readl(misc->base + MISC_MDIO_CFG);
	val |= MDIO_CFG_C22 | MDIO_CFG_FREE_RUN_CLK;
	writel(val, misc->base + MISC_MDIO_CFG);

	if (!of_property_read_u32(np, "brcm,gphy-address", &val)) {
		if (val >= PHY_MAX_ADDR)
			return dev_err_probe(dev, -EINVAL,
					     "invalid brcm,gphy-address %u\n", val);
		bcm47622_misc_gphy_init(misc, val);
	}

	if (!of_property_read_u32(np, "brcm,rgmii-sysport", &val)) {
		if (val > 1)
			return dev_err_probe(dev, -EINVAL,
					     "invalid brcm,rgmii-sysport %u\n", val);
		bcm47622_misc_rgmii_init(misc, val,
					 of_property_read_bool(np, "brcm,rgmii-tx-delay"),
					 of_property_read_bool(np, "brcm,rgmii-rx-delay"));
	}

	/* Probe the MDIO controller only once the GPHY is powered */
	err = devm_of_platform_populate(dev);
	if (err)
		return dev_err_probe(dev, err, "failed to populate children\n");

	/* The SYSTEMPORT driver waits for this */
	platform_set_drvdata(pdev, misc);

	return 0;
}

static const struct of_device_id bcm47622_misc_of_match[] = {
	{ .compatible = "brcm,bcm6755-systemport-misc" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm47622_misc_of_match);

static struct platform_driver bcm47622_misc_driver = {
	.driver = {
		.name = "bcm47622-systemport-misc",
		.of_match_table = bcm47622_misc_of_match,
	},
	.probe = bcm47622_misc_probe,
};

/*
 * SYSTEMPORT
 */

static inline u32 sp_read(struct bcmbca_sp *priv, u32 offset)
{
	return readl(priv->base + offset);
}

static inline void sp_write(struct bcmbca_sp *priv, u32 offset, u32 val)
{
	writel(val, priv->base + offset);
}

/* For per-packet accesses, ordered with the other accesses to the block */
static inline u32 sp_read_relaxed(struct bcmbca_sp *priv, u32 offset)
{
	return readl_relaxed(priv->base + offset);
}

static inline void sp_write_relaxed(struct bcmbca_sp *priv, u32 offset,
				    u32 val)
{
	writel_relaxed(val, priv->base + offset);
}

static inline void sp_maskset(struct bcmbca_sp *priv, u32 offset, u32 mask,
			      u32 set)
{
	u32 val;

	val = sp_read(priv, offset);
	val &= ~mask;
	val |= set;
	sp_write(priv, offset, val);
}

static void bcmbca_sp_set_umac_addr(struct bcmbca_sp *priv, const u8 *addr)
{
	sp_write(priv, SP_UMAC + UMAC_MAC0,
		 (addr[0] << 24) | (addr[1] << 16) | (addr[2] << 8) | addr[3]);
	sp_write(priv, SP_UMAC + UMAC_MAC1, (addr[4] << 8) | addr[5]);
}

static void bcmbca_sp_umac_init(struct bcmbca_sp *priv)
{
	u32 cmd;

	cmd = sp_read(priv, SP_UMAC + UMAC_CMD);
	sp_write(priv, SP_UMAC + UMAC_CMD, cmd | CMD_SW_RESET);

	cmd &= ~(CMD_TX_EN | CMD_RX_EN | CMD_SPEED_MASK << CMD_SPEED_SHIFT |
		 CMD_PAD_EN | CMD_CRC_FWD | CMD_PAUSE_FWD |
		 CMD_RX_PAUSE_IGNORE | CMD_TX_ADDR_INS | CMD_HD_EN |
		 CMD_LCL_LOOP_EN | CMD_AUTO_CONFIG | CMD_RMT_LOOP_EN |
		 CMD_RX_ERR_DISC | CMD_PRBL_EN | CMD_TX_PAUSE_IGNORE);
	cmd |= CMD_SPEED_1000 << CMD_SPEED_SHIFT | CMD_PROMISC |
	       CMD_CNTL_FRM_EN | CMD_NO_LEN_CHK | CMD_SW_RESET;
	sp_write(priv, SP_UMAC + UMAC_CMD, cmd);

	sp_write(priv, SP_UMAC + UMAC_MAX_FRAME_LEN, SP_MAX_FRAME_LEN);
	bcmbca_sp_set_umac_addr(priv, priv->ndev->dev_addr);

	sp_write(priv, SP_UMAC + UMAC_CMD, cmd & ~CMD_SW_RESET);

	priv->crc_fwd = !!(sp_read(priv, SP_UMAC + UMAC_CMD) & CMD_CRC_FWD);
}

static void bcmbca_sp_rx_desc_set(struct bcmbca_sp *priv, unsigned int idx)
{
	dma_addr_t dma = priv->rx_slots[idx].dma;

	/* The RDMA only looks at it once RDMA_CINDEX is updated */
	sp_write_relaxed(priv, SP_RDMA + RDMA_DESC(idx), lower_32_bits(dma));
	sp_write_relaxed(priv, SP_RDMA + RDMA_DESC(idx) + 4,
			 FIELD_PREP(DESC_LEN, SP_RX_BUF_SIZE) |
			 FIELD_PREP(DESC_ADDR_HI, upper_32_bits(dma)));
}

static void bcmbca_sp_dma_init(struct bcmbca_sp *priv)
{
	unsigned int i;
	u32 val;

	/* RBUF: no status block, IP alignment, keep tags, drop bad frames */
	val = sp_read(priv, SP_RBUF + RBUF_CONTROL);
	val &= ~(RBUF_RSB_MODE | RBUF_BTAG_STRIP);
	val |= RBUF_4B_ALIGN | RBUF_BAD_PKT_DISCARD;
	sp_write(priv, SP_RBUF + RBUF_CONTROL, val);
	sp_write(priv, SP_RBUF + RBUF_PKT_READY_THRESH, 0x80);

	/* RDMA ring 0, descriptors in local RAM */
	sp_write(priv, SP_RDMA + RDMA_LOCRAM_DESCRING_SIZE(0),
		 RDMA_RING_EN | SP_RX_RING_SIZE);
	sp_write(priv, SP_RDMA + RDMA_PKTBUF_SIZE(0), ilog2(SP_RX_BUF_SIZE));
	sp_write(priv, SP_RDMA + RDMA_CINDEX(0), 0);
	sp_write(priv, SP_RDMA + RDMA_PINDEX(0), 0);
	sp_write(priv, SP_RDMA + RDMA_DDR_DESC_RING_START(0), 0);
	sp_write(priv, SP_RDMA + RDMA_DDR_DESC_RING_START(1), 0);
	for (i = 0; i < SP_RX_RING_SIZE; i++)
		bcmbca_sp_rx_desc_set(priv, i);
	sp_write(priv, SP_RDMA + RDMA_DDR_DESC_RING_SIZE(0),
		 ilog2(SP_RX_RING_SIZE));
	/* Interrupt after every buffer, no timeout */
	sp_write(priv, SP_RDMA + RDMA_BUFDONE_INTR_CFG(0), 1);
	priv->rx_cons = 0;

	/* TDMA queue 0, fed through the descriptor write port */
	sp_write(priv, SP_TDMA + TDMA_DESC_RING_CONTROL(0), TDMA_RING_EN);
	sp_write(priv, SP_TDMA + TDMA_PROD_CONS(0), 0);
	sp_write(priv, SP_TDMA + TDMA_DDR_DESC_RING_START(0), 0);
	sp_write(priv, SP_TDMA + TDMA_DDR_DESC_RING_START(1), 0);
	sp_write(priv, SP_TDMA + TDMA_MAPPING(0), TDMA_MAP_IGNORE_STATUS);
	sp_write(priv, SP_TDMA + TDMA_DDR_DESC_RING_PUSH_TIMER(0), 1);
	sp_write(priv, SP_TDMA + TDMA_DDR_DESC_RING_SIZE(0), 0);
	/* Interrupt after every completed descriptor */
	sp_write(priv, SP_TDMA + TDMA_INTR_CONTROL(0), 1);
	sp_write(priv, SP_TDMA + TDMA_MAX_THRESHOLD(0),
		 TDMA_MAX_THRESHOLD_DEFAULT);

	/* Round robin arbitration */
	sp_write(priv, SP_TDMA + TDMA_TIER2_ARB_CTRL, 1);
	for (i = 0; i < 4; i++) {
		sp_write(priv, SP_TDMA + TDMA_TIER1_ARB_CTRL(i), 1);
		sp_write(priv, SP_TDMA + TDMA_TIER1_ARB_QUEUE_EN(i),
			 0xffU << (8 * i));
	}

	val = sp_read(priv, SP_TDMA + TDMA_CONTROL);
	val &= ~(TDMA_TSB_EN | TDMA_RING_CFG | TDMA_ACB_EN);
	sp_write(priv, SP_TDMA + TDMA_CONTROL, val);

	/* Only queue 0 is used */
	sp_write(priv, SP_TDMA + TDMA_TIER1_ARB_QUEUE_EN(0), 1);
	priv->tx_prod = 0;
	priv->tx_cons = 0;
}

static int bcmbca_sp_dma_enable(struct bcmbca_sp *priv)
{
	u32 val;
	int err;

	sp_maskset(priv, SP_RDMA + RDMA_CONTROL, 0, RDMA_EN);
	err = readl_poll_timeout(priv->base + SP_RDMA + RDMA_STATUS, val,
				 !(val & (RDMA_STATUS_DISABLED |
					  RDMA_STATUS_DESC_RAM_BUSY)),
				 10, 10000);
	if (err) {
		netdev_err(priv->ndev, "RDMA not ready (%08x)\n", val);
		return err;
	}

	sp_maskset(priv, SP_TDMA + TDMA_CONTROL, 0, TDMA_EN);
	err = readl_poll_timeout(priv->base + SP_TDMA + TDMA_STATUS, val,
				 !(val & (TDMA_STATUS_DISABLED |
					  TDMA_STATUS_LL_RAM_INIT_BUSY)),
				 10, 10000);
	if (err)
		netdev_err(priv->ndev, "TDMA not ready (%08x)\n", val);

	return err;
}

/* Returns an error if a DMA engine could not be confirmed stopped */
static int bcmbca_sp_hw_stop(struct bcmbca_sp *priv)
{
	int ret = 0;
	u32 val;

	/* RX: stop the MAC, let a pending DMA complete, stop the DMA */
	sp_maskset(priv, SP_UMAC + UMAC_CMD, CMD_RX_EN, 0);
	fsleep(1000);
	sp_maskset(priv, SP_RDMA + RDMA_CONTROL, RDMA_EN, 0);
	if (readl_poll_timeout(priv->base + SP_RDMA + RDMA_STATUS, val,
			       val & RDMA_STATUS_DISABLED, 10, 100000)) {
		dev_warn(priv->dev, "RDMA did not stop (%08x)\n", val);
		ret = -ETIMEDOUT;
	}
	sp_write(priv, SP_TOPCTRL + TOPCTRL_RX_FLUSH, 1);

	/*
	 * TX: nothing gets queued anymore, let the queued frames go out while
	 * the MAC is still enabled, then stop the DMA and the MAC.
	 */
	if (readl_poll_timeout(priv->base + SP_TDMA + TDMA_PROD_CONS(0), val,
			       FIELD_GET(TDMA_PROD, val) == FIELD_GET(TDMA_CONS, val),
			       100, 100000))
		dev_warn(priv->dev, "TDMA did not drain (%08x)\n", val);
	sp_maskset(priv, SP_TDMA + TDMA_CONTROL, TDMA_EN, 0);
	if (readl_poll_timeout(priv->base + SP_TDMA + TDMA_STATUS, val,
			       val & TDMA_STATUS_DISABLED, 10, 100000)) {
		dev_warn(priv->dev, "TDMA did not stop (%08x)\n", val);
		ret = -ETIMEDOUT;
	}
	sp_write(priv, SP_TDMA + TDMA_PROD_CONS(0), 0);
	sp_maskset(priv, SP_UMAC + UMAC_CMD, CMD_TX_EN, 0);
	sp_write(priv, SP_TOPCTRL + TOPCTRL_TX_FLUSH, 1);
	fsleep(100);

	/* Disable the rings, dma_init sets them up again */
	sp_maskset(priv, SP_TDMA + TDMA_DESC_RING_CONTROL(0), TDMA_RING_EN, 0);
	sp_maskset(priv, SP_RDMA + RDMA_LOCRAM_DESCRING_SIZE(0), RDMA_RING_EN, 0);

	sp_write(priv, SP_TOPCTRL + TOPCTRL_RX_FLUSH, 0);
	sp_write(priv, SP_TOPCTRL + TOPCTRL_TX_FLUSH, 0);

	return ret;
}

static void bcmbca_sp_irq_mask_all(struct bcmbca_sp *priv)
{
	unsigned int k;

	for (k = 0; k < SP_NUM_INTRL2; k++) {
		sp_write(priv, SP_INTRL2(k) + INTRL2_MASK_SET, ~0U);
		sp_write(priv, SP_INTRL2(k) + INTRL2_CLEAR, ~0U);
	}
}

static void bcmbca_sp_irq_unmask(struct bcmbca_sp *priv)
{
	sp_write(priv, SP_INTRL2(SP_IRQ_RX) + INTRL2_MASK_CLEAR, SP_INTR_RING0);
	sp_write(priv, SP_INTRL2(SP_IRQ_TX) + INTRL2_MASK_CLEAR, SP_INTR_RING0);
}

static void bcmbca_sp_irq_enable(struct bcmbca_sp *priv)
{
	unsigned int k;

	/*
	 * INTC[k] selects, for each bit of INTRL2[k], which DMA event of the
	 * ring/queue with the same index it reports. Route RDMA ring 0 to
	 * the RX line and TDMA queue 0 to the TX line.
	 */
	for (k = SP_IRQ_RX; k <= SP_IRQ_TX; k++) {
		u32 map = k == SP_IRQ_RX ? INTC_MAP_RDMA : INTC_MAP_TDMA;

		sp_write(priv, SP_INTC(k) + INTC_MAPPING_0, map);
		sp_write(priv, SP_INTC(k) + INTC_MAPPING_1, map);
		sp_write(priv, SP_INTC(k) + INTC_PSB_EN, 0);
		sp_write(priv, SP_INTC(k) + INTC_CSB_EN, 0);
		sp_write(priv, SP_INTRL2(k) + INTRL2_CLEAR, ~0U);
	}

	bcmbca_sp_irq_unmask(priv);
}

static irqreturn_t bcmbca_sp_isr(int irq, void *dev_id)
{
	struct bcmbca_sp *priv = dev_id;
	unsigned int k = irq == priv->rx_irq ? SP_IRQ_RX : SP_IRQ_TX;
	u32 stat;

	/* Ring 0 is the only source ever unmasked */
	stat = sp_read_relaxed(priv, SP_INTRL2(k) + INTRL2_STATUS) &
	       SP_INTR_RING0;
	if (!stat)
		return IRQ_NONE;

	sp_write(priv, SP_INTRL2(k) + INTRL2_MASK_SET, stat);
	napi_schedule(&priv->napi);

	return IRQ_HANDLED;
}

static int bcmbca_sp_rx_buf_alloc(struct bcmbca_sp *priv,
				  struct bcmbca_sp_rx_slot *slot, bool napi)
{
	dma_addr_t dma;
	void *buf;

	if (napi)
		buf = napi_alloc_frag(SP_RX_FRAG_SIZE);
	else
		buf = netdev_alloc_frag(SP_RX_FRAG_SIZE);
	if (!buf)
		return -ENOMEM;

	dma = dma_map_single(priv->dev, buf + SP_RX_BUF_OFFSET, SP_RX_BUF_SIZE,
			     DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->dev, dma)) {
		skb_free_frag(buf);
		return -ENOMEM;
	}

	slot->buf = buf;
	slot->dma = dma;

	return 0;
}

static void bcmbca_sp_rx_free(struct bcmbca_sp *priv)
{
	unsigned int i;

	for (i = 0; i < SP_RX_RING_SIZE; i++) {
		struct bcmbca_sp_rx_slot *slot = &priv->rx_slots[i];

		if (!slot->buf)
			continue;

		dma_unmap_single(priv->dev, slot->dma, SP_RX_BUF_SIZE,
				 DMA_FROM_DEVICE);
		skb_free_frag(slot->buf);
		slot->buf = NULL;
	}
}

static int bcmbca_sp_rx_alloc(struct bcmbca_sp *priv)
{
	unsigned int i;
	int err;

	for (i = 0; i < SP_RX_RING_SIZE; i++) {
		err = bcmbca_sp_rx_buf_alloc(priv, &priv->rx_slots[i], false);
		if (err) {
			bcmbca_sp_rx_free(priv);
			return err;
		}
	}

	return 0;
}

static bool bcmbca_sp_rx_frame_ok(struct bcmbca_sp *priv, u32 desc,
				  unsigned int len)
{
	struct net_device *ndev = priv->ndev;

	/* Frames never span buffers as SP_MAX_FRAME_LEN < SP_RX_BUF_SIZE */
	if ((desc & (DESC_SOP | DESC_EOP)) != (DESC_SOP | DESC_EOP) ||
	    FIELD_GET(DESC_LEN, desc) > SP_RX_BUF_SIZE || len < ETH_HLEN)
		DEV_STATS_INC(ndev, rx_length_errors);
	else if (desc & DESC_RX_OVERFLOW)
		DEV_STATS_INC(ndev, rx_over_errors);
	else if (!(desc & DESC_RX_ERR))
		return true;

	if (net_ratelimit())
		netdev_warn(ndev, "bad RX descriptor %08x\n", desc);
	DEV_STATS_INC(ndev, rx_errors);

	return false;
}

static int bcmbca_sp_rx(struct bcmbca_sp *priv, int budget)
{
	unsigned int trim = SP_RX_ALIGN_PAD + (priv->crc_fwd ? ETH_FCS_LEN : 0);
	struct net_device *ndev = priv->ndev;
	unsigned int packets = 0, bytes = 0;
	struct pcpu_sw_netstats *tstats;
	int done = 0;
	u16 pidx;

	pidx = FIELD_GET(RDMA_INDEX, sp_read(priv, SP_RDMA + RDMA_PINDEX(0)));
	if (unlikely((u16)(pidx - priv->rx_cons) > SP_RX_RING_SIZE)) {
		if (net_ratelimit())
			netdev_err(ndev,
				   "bogus RDMA producer index %u (consumer %u), restarting\n",
				   pidx, priv->rx_cons);
		schedule_work(&priv->reset_work);
		return 0;
	}

	while (done < budget && priv->rx_cons != pidx) {
		unsigned int idx = priv->rx_cons % SP_RX_RING_SIZE;
		struct bcmbca_sp_rx_slot *slot = &priv->rx_slots[idx];
		struct bcmbca_sp_rx_slot old = *slot;
		struct sk_buff *skb;
		unsigned int len;
		u32 desc;

		desc = sp_read_relaxed(priv, SP_RDMA + RDMA_DESC(idx) + 4);
		len = FIELD_GET(DESC_LEN, desc);
		priv->rx_cons++;
		done++;

		/* The length includes the alignment pad, and the FCS if forwarded */
		len = len > trim ? len - trim : 0;

		if (unlikely(!bcmbca_sp_rx_frame_ok(priv, desc, len))) {
			bcmbca_sp_rx_desc_set(priv, idx);
			continue;
		}

		/* On allocation failure, drop the frame and reuse the buffer */
		if (unlikely(bcmbca_sp_rx_buf_alloc(priv, slot, true))) {
			DEV_STATS_INC(ndev, rx_dropped);
			bcmbca_sp_rx_desc_set(priv, idx);
			continue;
		}
		bcmbca_sp_rx_desc_set(priv, idx);

		/* Only the frame needs to be synced, not the whole buffer */
		dma_sync_single_for_cpu(priv->dev, old.dma, SP_RX_ALIGN_PAD + len,
					DMA_FROM_DEVICE);
		dma_unmap_single_attrs(priv->dev, old.dma, SP_RX_BUF_SIZE,
				       DMA_FROM_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);

		skb = napi_build_skb(old.buf, SP_RX_FRAG_SIZE);
		if (unlikely(!skb)) {
			skb_free_frag(old.buf);
			DEV_STATS_INC(ndev, rx_dropped);
			continue;
		}

		skb_reserve(skb, SP_RX_BUF_OFFSET + SP_RX_ALIGN_PAD);
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, ndev);

		packets++;
		bytes += len;

		napi_gro_receive(&priv->napi, skb);
	}

	if (!done)
		return 0;

	sp_write(priv, SP_RDMA + RDMA_CINDEX(0), priv->rx_cons);

	tstats = this_cpu_ptr(ndev->tstats);
	u64_stats_update_begin(&tstats->syncp);
	u64_stats_add(&tstats->rx_packets, packets);
	u64_stats_add(&tstats->rx_bytes, bytes);
	u64_stats_update_end(&tstats->syncp);

	return done;
}

/* Returns the number of frames still owned by the hardware */
static unsigned int bcmbca_sp_tx_reclaim(struct bcmbca_sp *priv,
					 int budget)
{
	unsigned int packets = 0, bytes = 0;
	u16 hw_cons, pending;

	spin_lock(&priv->tx_lock);

	hw_cons = FIELD_GET(TDMA_CONS, sp_read(priv, SP_TDMA + TDMA_PROD_CONS(0)));
	if (unlikely((u16)(hw_cons - priv->tx_cons) >
		     (u16)(priv->tx_prod - priv->tx_cons))) {
		netdev_err_once(priv->ndev,
				"bogus TDMA consumer index %u (sw %u..%u)\n",
				hw_cons, priv->tx_cons, priv->tx_prod);
		hw_cons = priv->tx_cons;
	}

	while (priv->tx_cons != hw_cons) {
		struct bcmbca_sp_tx_slot *slot;

		slot = &priv->tx_slots[priv->tx_cons % SP_TX_RING_SIZE];
		dma_unmap_single(priv->dev, slot->dma, slot->skb->len,
				 DMA_TO_DEVICE);
		packets++;
		bytes += slot->skb->len;
		napi_consume_skb(slot->skb, budget);
		slot->skb = NULL;
		priv->tx_cons++;
	}

	pending = priv->tx_prod - priv->tx_cons;

	if (packets) {
		netdev_completed_queue(priv->ndev, packets, bytes);
		if (netif_queue_stopped(priv->ndev))
			netif_wake_queue(priv->ndev);
	}

	spin_unlock(&priv->tx_lock);

	if (packets)
		dev_sw_netstats_tx_add(priv->ndev, packets, bytes);

	return pending;
}

static void bcmbca_sp_tx_free(struct bcmbca_sp *priv)
{
	spin_lock_bh(&priv->tx_lock);

	while (priv->tx_cons != priv->tx_prod) {
		struct bcmbca_sp_tx_slot *slot;

		slot = &priv->tx_slots[priv->tx_cons % SP_TX_RING_SIZE];
		dma_unmap_single(priv->dev, slot->dma, slot->skb->len,
				 DMA_TO_DEVICE);
		dev_kfree_skb_any(slot->skb);
		slot->skb = NULL;
		priv->tx_cons++;
	}

	spin_unlock_bh(&priv->tx_lock);

	netdev_reset_queue(priv->ndev);
}

static int bcmbca_sp_poll(struct napi_struct *napi, int budget)
{
	struct bcmbca_sp *priv = container_of(napi, struct bcmbca_sp, napi);
	int done;

	/* Ack before looking at the rings so that new events are not lost */
	sp_write(priv, SP_INTRL2(SP_IRQ_RX) + INTRL2_CLEAR, SP_INTR_RING0);
	sp_write(priv, SP_INTRL2(SP_IRQ_TX) + INTRL2_CLEAR, SP_INTR_RING0);

	bcmbca_sp_tx_reclaim(priv, budget);
	done = bcmbca_sp_rx(priv, budget);

	if (done < budget && napi_complete_done(napi, done))
		bcmbca_sp_irq_unmask(priv);

	return done;
}

static netdev_tx_t bcmbca_sp_start_xmit(struct sk_buff *skb,
					struct net_device *ndev)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);
	struct bcmbca_sp_tx_slot *slot;
	unsigned int len;
	dma_addr_t dma;
	u16 used;

	/* The UniMAC does not pad short frames (PAD_EN is off) */
	if (eth_skb_pad(skb)) {
		dev_core_stats_tx_dropped_inc(ndev);
		return NETDEV_TX_OK;
	}
	len = skb->len;

	dma = dma_map_single(priv->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, dma)) {
		dev_kfree_skb_any(skb);
		dev_core_stats_tx_dropped_inc(ndev);
		return NETDEV_TX_OK;
	}

	spin_lock(&priv->tx_lock);

	used = priv->tx_prod - priv->tx_cons;
	if (unlikely(used >= SP_TX_RING_SIZE)) {
		netif_stop_queue(ndev);
		spin_unlock(&priv->tx_lock);
		dma_unmap_single(priv->dev, dma, len, DMA_TO_DEVICE);
		return NETDEV_TX_BUSY;
	}

	slot = &priv->tx_slots[priv->tx_prod % SP_TX_RING_SIZE];
	slot->skb = skb;
	slot->dma = dma;

	netdev_sent_queue(ndev, len);
	skb_tx_timestamp(skb);

	/* The TDMA takes the descriptor with the write of its second word */
	sp_write_relaxed(priv, SP_TDMA + TDMA_WRITE_PORT_LO(0),
			 lower_32_bits(dma));
	sp_write(priv, SP_TDMA + TDMA_WRITE_PORT_HI(0),
		 FIELD_PREP(DESC_LEN, len) | DESC_SOP | DESC_EOP |
		 DESC_TX_APPEND_CRC |
		 FIELD_PREP(DESC_ADDR_HI, upper_32_bits(dma)));
	priv->tx_prod++;

	if (used + 1 >= SP_TX_RING_SIZE)
		netif_stop_queue(ndev);

	spin_unlock(&priv->tx_lock);

	return NETDEV_TX_OK;
}

static void bcmbca_sp_dma_stop(struct bcmbca_sp *priv)
{
	if (bcmbca_sp_hw_stop(priv)) {
		/* Rather leak the buffers than free memory the DMA may access */
		netdev_err(priv->ndev, "DMA did not stop, leaking buffers\n");
		memset(priv->rx_slots, 0, sizeof(priv->rx_slots));
		memset(priv->tx_slots, 0, sizeof(priv->tx_slots));
		priv->tx_cons = priv->tx_prod;
	}

	bcmbca_sp_tx_free(priv);
	bcmbca_sp_rx_free(priv);
}

static int bcmbca_sp_up(struct bcmbca_sp *priv)
{
	struct net_device *ndev = priv->ndev;
	int err;

	err = phylink_of_phy_connect(priv->phylink, priv->dev->of_node, 0);
	if (err) {
		netdev_err(ndev, "could not attach to PHY: %d\n", err);
		return err;
	}

	err = bcmbca_sp_rx_alloc(priv);
	if (err)
		goto err_disconnect;

	bcmbca_sp_umac_init(priv);
	bcmbca_sp_dma_init(priv);
	netdev_reset_queue(ndev);

	err = bcmbca_sp_dma_enable(priv);
	if (err)
		goto err_stop;

	napi_enable(&priv->napi);
	bcmbca_sp_irq_enable(priv);

	phylink_start(priv->phylink);
	netif_start_queue(ndev);
	priv->up = true;

	return 0;

err_stop:
	bcmbca_sp_dma_stop(priv);
err_disconnect:
	phylink_disconnect_phy(priv->phylink);
	return err;
}

static void bcmbca_sp_down(struct bcmbca_sp *priv)
{
	struct net_device *ndev = priv->ndev;

	if (!priv->up)
		return;
	priv->up = false;

	/* NAPI first, so that TX completion cannot wake the queue again */
	napi_disable(&priv->napi);
	netif_tx_disable(ndev);
	bcmbca_sp_irq_mask_all(priv);
	synchronize_irq(priv->rx_irq);
	synchronize_irq(priv->tx_irq);

	/* Before phylink_stop(), the MAC must still transmit for TX to drain */
	bcmbca_sp_dma_stop(priv);

	phylink_stop(priv->phylink);
	phylink_disconnect_phy(priv->phylink);
}

static int bcmbca_sp_open(struct net_device *ndev)
{
	return bcmbca_sp_up(netdev_priv(ndev));
}

static int bcmbca_sp_stop(struct net_device *ndev)
{
	bcmbca_sp_down(netdev_priv(ndev));

	return 0;
}

static void bcmbca_sp_reset_work(struct work_struct *work)
{
	struct bcmbca_sp *priv = container_of(work, struct bcmbca_sp,
						reset_work);
	int err;

	rtnl_lock();
	if (netif_running(priv->ndev) && priv->up) {
		bcmbca_sp_down(priv);
		err = bcmbca_sp_up(priv);
		if (err)
			netdev_err(priv->ndev, "restart failed: %d\n", err);
	}
	rtnl_unlock();
}

static void bcmbca_sp_tx_timeout(struct net_device *ndev,
				 unsigned int txqueue)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);

	netdev_warn(ndev, "TX timeout (TDMA %08x), restarting\n",
		    sp_read(priv, SP_TDMA + TDMA_PROD_CONS(0)));
	schedule_work(&priv->reset_work);
}

static int bcmbca_sp_set_mac_address(struct net_device *ndev, void *addr)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);
	int err;

	err = eth_mac_addr(ndev, addr);
	if (err)
		return err;

	if (netif_running(ndev))
		bcmbca_sp_set_umac_addr(priv, ndev->dev_addr);

	return 0;
}

static int bcmbca_sp_ioctl(struct net_device *ndev, struct ifreq *ifr,
			   int cmd)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);

	return phylink_mii_ioctl(priv->phylink, ifr, cmd);
}

static const struct net_device_ops bcmbca_sp_netdev_ops = {
	.ndo_open = bcmbca_sp_open,
	.ndo_stop = bcmbca_sp_stop,
	.ndo_start_xmit = bcmbca_sp_start_xmit,
	.ndo_tx_timeout = bcmbca_sp_tx_timeout,
	.ndo_set_mac_address = bcmbca_sp_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_eth_ioctl = bcmbca_sp_ioctl,
};

static int bcmbca_sp_get_link_ksettings(struct net_device *ndev,
					struct ethtool_link_ksettings *cmd)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);

	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
}

static int bcmbca_sp_set_link_ksettings(struct net_device *ndev,
					const struct ethtool_link_ksettings *cmd)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);

	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

static int bcmbca_sp_nway_reset(struct net_device *ndev)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);

	return phylink_ethtool_nway_reset(priv->phylink);
}

static void bcmbca_sp_get_pauseparam(struct net_device *ndev,
				     struct ethtool_pauseparam *pause)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);

	phylink_ethtool_get_pauseparam(priv->phylink, pause);
}

static int bcmbca_sp_set_pauseparam(struct net_device *ndev,
				    struct ethtool_pauseparam *pause)
{
	struct bcmbca_sp *priv = netdev_priv(ndev);

	return phylink_ethtool_set_pauseparam(priv->phylink, pause);
}

static const struct ethtool_ops bcmbca_sp_ethtool_ops = {
	.get_link = ethtool_op_get_link,
	.get_link_ksettings = bcmbca_sp_get_link_ksettings,
	.set_link_ksettings = bcmbca_sp_set_link_ksettings,
	.nway_reset = bcmbca_sp_nway_reset,
	.get_pauseparam = bcmbca_sp_get_pauseparam,
	.set_pauseparam = bcmbca_sp_set_pauseparam,
};

static struct bcmbca_sp *phylink_config_to_sp(struct phylink_config *config)
{
	return netdev_priv(to_net_dev(config->dev));
}

static void bcmbca_sp_mac_config(struct phylink_config *config,
				 unsigned int mode,
				   const struct phylink_link_state *state)
{
	/* The interface is selected by the misc block */
}

static void bcmbca_sp_mac_link_down(struct phylink_config *config,
				    unsigned int mode,
				      phy_interface_t interface)
{
	struct bcmbca_sp *priv = phylink_config_to_sp(config);

	/*
	 * The MAC stays enabled so that the TDMA never stalls with frames
	 * queued; bcmbca_sp_hw_stop() disables it once the DMA is idle.
	 */
	if (phy_interface_mode_is_rgmii(interface))
		bcm47622_misc_set_ib_status(priv->misc, false, SPEED_UNKNOWN,
					    DUPLEX_UNKNOWN);
}

static void bcmbca_sp_mac_link_up(struct phylink_config *config,
				  struct phy_device *phy, unsigned int mode,
				    phy_interface_t interface, int speed,
				    int duplex, bool tx_pause, bool rx_pause)
{
	struct bcmbca_sp *priv = phylink_config_to_sp(config);
	u32 cmd, spd, ipg;

	switch (speed) {
	case SPEED_1000:
		spd = CMD_SPEED_1000;
		ipg = 0x05;
		break;
	case SPEED_100:
		spd = CMD_SPEED_100;
		ipg = 0x0a;
		break;
	case SPEED_10:
		spd = CMD_SPEED_10;
		ipg = 0x0a;
		break;
	default:
		netdev_err(priv->ndev, "unsupported speed %d\n", speed);
		return;
	}

	/* Speed, duplex and pause may only be changed in reset */
	cmd = sp_read(priv, SP_UMAC + UMAC_CMD);
	cmd |= CMD_SW_RESET;
	sp_write(priv, SP_UMAC + UMAC_CMD, cmd);

	cmd &= ~(CMD_SPEED_MASK << CMD_SPEED_SHIFT | CMD_HD_EN |
		 CMD_RX_PAUSE_IGNORE | CMD_TX_PAUSE_IGNORE);
	cmd |= spd << CMD_SPEED_SHIFT | CMD_SW_OVERRIDE_TX | CMD_SW_OVERRIDE_RX;
	if (duplex == DUPLEX_HALF)
		cmd |= CMD_HD_EN;
	if (!rx_pause)
		cmd |= CMD_RX_PAUSE_IGNORE;
	if (!tx_pause)
		cmd |= CMD_TX_PAUSE_IGNORE;
	sp_write(priv, SP_UMAC + UMAC_CMD, cmd);

	sp_maskset(priv, SP_UMAC + UMAC_HD_BKP_CTRL,
		   IPG_CONFIG_RX_MASK << IPG_CONFIG_RX_SHIFT,
		   ipg << IPG_CONFIG_RX_SHIFT);

	/* The ASIC team asks for a longer reset at 10 Mbps */
	if (speed == SPEED_10)
		fsleep(1000);

	cmd &= ~CMD_SW_RESET;
	sp_write(priv, SP_UMAC + UMAC_CMD, cmd);

	if (phy_interface_mode_is_rgmii(interface))
		bcm47622_misc_set_ib_status(priv->misc, true, speed, duplex);

	sp_write(priv, SP_UMAC + UMAC_CMD, cmd | CMD_TX_EN | CMD_RX_EN);
}

static const struct phylink_mac_ops bcmbca_sp_phylink_ops = {
	.mac_config = bcmbca_sp_mac_config,
	.mac_link_down = bcmbca_sp_mac_link_down,
	.mac_link_up = bcmbca_sp_mac_link_up,
};

/* The misc block provides the MDIO bus, the GPHY and the RGMII crossbar */
static int bcmbca_sp_get_misc(struct bcmbca_sp *priv)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct device_link *link;

	np = of_find_compatible_node(NULL, NULL, "brcm,bcm6755-systemport-misc");
	if (!np || !of_device_is_available(np)) {
		of_node_put(np);
		return dev_err_probe(priv->dev, -ENODEV, "no sysport-misc\n");
	}

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return dev_err_probe(priv->dev, -EPROBE_DEFER,
				     "waiting for sysport-misc\n");

	priv->misc = platform_get_drvdata(pdev);
	link = priv->misc ? device_link_add(priv->dev, &pdev->dev,
					    DL_FLAG_AUTOREMOVE_CONSUMER) : NULL;
	put_device(&pdev->dev);
	if (!priv->misc)
		return dev_err_probe(priv->dev, -EPROBE_DEFER,
				     "waiting for sysport-misc\n");
	if (!link)
		return dev_err_probe(priv->dev, -EINVAL,
				     "failed to link to sysport-misc\n");

	return 0;
}

static int bcmbca_sp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	phy_interface_t phy_mode;
	struct bcmbca_sp *priv;
	struct net_device *ndev;
	const char *name;
	u32 rev;
	int err;

	ndev = devm_alloc_etherdev(dev, sizeof(*priv));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, dev);

	priv = netdev_priv(ndev);
	priv->dev = dev;
	priv->ndev = ndev;
	spin_lock_init(&priv->tx_lock);
	INIT_WORK(&priv->reset_work, bcmbca_sp_reset_work);
	platform_set_drvdata(pdev, priv);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	/* The SYSTEMPORT power domain is expected to be on (CFE / PMC) */
	rev = sp_read(priv, SP_TOPCTRL + TOPCTRL_REV);
	if (rev == ~0U || rev == 0xdeadbeef)
		return dev_err_probe(dev, -ENODEV,
				     "SYSTEMPORT revision reads %08x, block powered off?\n",
				     rev);

	err = of_get_phy_mode(np, &phy_mode);
	if (err)
		return dev_err_probe(dev, err, "invalid phy-mode\n");

	err = bcmbca_sp_get_misc(priv);
	if (err)
		return err;

	/* The bootloader may have left the DMA running */
	bcmbca_sp_irq_mask_all(priv);
	bcmbca_sp_hw_stop(priv);

	priv->rx_irq = platform_get_irq(pdev, SP_IRQ_RX);
	if (priv->rx_irq < 0)
		return priv->rx_irq;
	priv->tx_irq = platform_get_irq(pdev, SP_IRQ_TX);
	if (priv->tx_irq < 0)
		return priv->tx_irq;

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err)
		return dev_err_probe(dev, err, "failed to set DMA mask\n");

	err = of_get_ethdev_address(np, ndev);
	if (err == -EPROBE_DEFER)
		return err;
	if (err) {
		eth_hw_addr_random(ndev);
		dev_info(dev, "using random MAC address %pM\n", ndev->dev_addr);
	}

	ndev->netdev_ops = &bcmbca_sp_netdev_ops;
	ndev->ethtool_ops = &bcmbca_sp_ethtool_ops;
	ndev->pcpu_stat_type = NETDEV_PCPU_STAT_TSTATS;
	/* 2018, a DSA tag (e.g. 8 bytes for rtl8_4) is part of the MTU */
	ndev->max_mtu = SP_MAX_FRAME_LEN - ETH_HLEN - VLAN_HLEN - ETH_FCS_LEN;
	ndev->priv_flags |= IFF_LIVE_ADDR_CHANGE;

	priv->phylink_config.dev = &ndev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	priv->phylink_config.mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
						MAC_10 | MAC_100 | MAC_1000FD;
	__set_bit(PHY_INTERFACE_MODE_GMII,
		  priv->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_INTERNAL,
		  priv->phylink_config.supported_interfaces);
	phy_interface_set_rgmii(priv->phylink_config.supported_interfaces);

	priv->phylink = phylink_create(&priv->phylink_config,
				       of_fwnode_handle(np), phy_mode,
				       &bcmbca_sp_phylink_ops);
	if (IS_ERR(priv->phylink))
		return dev_err_probe(dev, PTR_ERR(priv->phylink),
				     "failed to create phylink\n");

	name = devm_kasprintf(dev, GFP_KERNEL, "%s-rx", dev_name(dev));
	err = name ? devm_request_irq(dev, priv->rx_irq, bcmbca_sp_isr, 0,
				      name, priv) : -ENOMEM;
	if (err)
		goto err_phylink;
	name = devm_kasprintf(dev, GFP_KERNEL, "%s-tx", dev_name(dev));
	err = name ? devm_request_irq(dev, priv->tx_irq, bcmbca_sp_isr, 0,
				      name, priv) : -ENOMEM;
	if (err)
		goto err_phylink;

	netif_napi_add(ndev, &priv->napi, bcmbca_sp_poll);

	err = register_netdev(ndev);
	if (err)
		goto err_napi;

	netdev_info(ndev, "SYSTEMPORT rev %08x, %s\n", rev, phy_modes(phy_mode));

	return 0;

err_napi:
	netif_napi_del(&priv->napi);
err_phylink:
	phylink_destroy(priv->phylink);
	return err;
}

static void bcmbca_sp_remove(struct platform_device *pdev)
{
	struct bcmbca_sp *priv = platform_get_drvdata(pdev);

	unregister_netdev(priv->ndev);
	cancel_work_sync(&priv->reset_work);
	netif_napi_del(&priv->napi);
	phylink_destroy(priv->phylink);
}

static const struct of_device_id bcmbca_sp_of_match[] = {
	{ .compatible = "brcm,bcm6755-systemport" },
	{}
};
MODULE_DEVICE_TABLE(of, bcmbca_sp_of_match);

static struct platform_driver bcmbca_sp_driver = {
	.driver = {
		.name = "bcmbca-sysport",
		.of_match_table = bcmbca_sp_of_match,
	},
	.probe = bcmbca_sp_probe,
	.remove = bcmbca_sp_remove,
};

static struct platform_driver * const bcmbca_sp_drivers[] = {
	&bcm47622_misc_driver,
	&bcmbca_sp_driver,
};

static int __init bcmbca_sp_init(void)
{
	return platform_register_drivers(bcmbca_sp_drivers,
					 ARRAY_SIZE(bcmbca_sp_drivers));
}
module_init(bcmbca_sp_init);

static void __exit bcmbca_sp_exit(void)
{
	platform_unregister_drivers(bcmbca_sp_drivers,
				    ARRAY_SIZE(bcmbca_sp_drivers));
}
module_exit(bcmbca_sp_exit);

MODULE_DESCRIPTION("Broadcom BCA SYSTEMPORT v2 Ethernet driver");
MODULE_LICENSE("GPL");
