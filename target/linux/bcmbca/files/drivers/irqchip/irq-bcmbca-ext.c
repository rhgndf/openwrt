// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom BCA external interrupt controller
 *
 * The peripheral block of BCA SoCs (e.g. BCM47622 / BCM6755) has a small
 * number of external interrupt lines, each wired to its own GIC SPI. A line
 * is connected to any GPIO pin through a mux, latches the selected edge or
 * level, and holds its SPI high until it is cleared.
 *
 * The domain hwirq is the GPIO number. A free line is routed to the pin when
 * an interrupt is allocated, so consumers use the GPIO number like with any
 * other GPIO interrupt controller and don't have to know about the lines.
 *
 * Register layout from the Broadcom SDK (bcm_bca_extintr.c, impl1), relative
 * to the ExtIrqCtrl register, i.e. 0x1c past the SDK's register base.
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

/* One bit per line in each field */
#define EXTINTR_CTRL			0x00
#define  EXTINTR_CTRL_SENSE(l)		BIT(8 + (l))	/* high / rising */
#define  EXTINTR_CTRL_BOTH_EDGE(l)	BIT(16 + (l))
#define  EXTINTR_CTRL_LEVEL(l)		BIT(24 + (l))
#define EXTINTR_STATUS			0x04
#define EXTINTR_CLEAR			0x0c
#define EXTINTR_MASK_SET		0x14	/* enables the line */
#define EXTINTR_MASK_CLEAR		0x18	/* disables the line */
/* 4 bits per line, the selected pin is SEL1 * 16 + SEL0 */
#define EXTINTR_MUX_SEL0		0x24
#define EXTINTR_MUX_SEL1		0x28
#define  EXTINTR_MUX_SHIFT(l)		((l) * 4)
#define  EXTINTR_MUX_MASK(l)		(GENMASK(3, 0) << EXTINTR_MUX_SHIFT(l))

#define EXTINTR_MAX_LINES		8
#define EXTINTR_MAX_PINS		256

struct bcmbca_ext_intc;

struct bcmbca_ext_line {
	struct bcmbca_ext_intc *intc;
	struct irq_fwspec parent;
	unsigned int index;
};

struct bcmbca_ext_intc {
	void __iomem *base;
	/* Protects the shared CTRL, CLEAR and MUX registers and @used */
	raw_spinlock_t lock;
	unsigned long used;
	unsigned int nr_lines;
	struct bcmbca_ext_line lines[] __counted_by(nr_lines);
};

/*
 * The SDK disagrees with itself about the clear register: its init code
 * writes all ones then zero, its ack writes the line's bit and then the
 * inverted bit, which would also clear (or hold in clear) all other lines.
 * Writing the bit and then zero is right for both readings of the register.
 */
static void bcmbca_ext_clear(struct bcmbca_ext_intc *intc, u32 lines)
{
	lockdep_assert_held(&intc->lock);

	writel(lines, intc->base + EXTINTR_CLEAR);
	writel(0, intc->base + EXTINTR_CLEAR);
}

static void bcmbca_ext_route(struct bcmbca_ext_intc *intc, unsigned int line,
			     unsigned int pin)
{
	unsigned int shift = EXTINTR_MUX_SHIFT(line);
	u32 mask = EXTINTR_MUX_MASK(line);
	u32 val;

	lockdep_assert_held(&intc->lock);

	val = readl(intc->base + EXTINTR_MUX_SEL0) & ~mask;
	writel(val | (((pin % 16) << shift) & mask), intc->base + EXTINTR_MUX_SEL0);
	val = readl(intc->base + EXTINTR_MUX_SEL1) & ~mask;
	writel(val | (((pin / 16) << shift) & mask), intc->base + EXTINTR_MUX_SEL1);
}

static void bcmbca_ext_ack(struct irq_data *d)
{
	struct bcmbca_ext_line *line = irq_data_get_irq_chip_data(d);
	struct bcmbca_ext_intc *intc = line->intc;

	guard(raw_spinlock_irqsave)(&intc->lock);
	bcmbca_ext_clear(intc, BIT(line->index));
}

static void bcmbca_ext_eoi(struct irq_data *d)
{
	/*
	 * Level interrupts are latched as well. Clear them once handled, the
	 * line latches again right away if the input is still asserted.
	 */
	if (irqd_is_level_type(d))
		bcmbca_ext_ack(d);

	irq_chip_eoi_parent(d);
}

static void bcmbca_ext_mask(struct irq_data *d)
{
	struct bcmbca_ext_line *line = irq_data_get_irq_chip_data(d);

	writel(BIT(line->index), line->intc->base + EXTINTR_MASK_CLEAR);
	irq_chip_mask_parent(d);
}

static void bcmbca_ext_unmask(struct irq_data *d)
{
	struct bcmbca_ext_line *line = irq_data_get_irq_chip_data(d);

	irq_chip_unmask_parent(d);
	writel(BIT(line->index), line->intc->base + EXTINTR_MASK_SET);
}

static int bcmbca_ext_set_type(struct irq_data *d, unsigned int type)
{
	struct bcmbca_ext_line *line = irq_data_get_irq_chip_data(d);
	struct bcmbca_ext_intc *intc = line->intc;
	unsigned int l = line->index;
	u32 mask = EXTINTR_CTRL_SENSE(l) | EXTINTR_CTRL_BOTH_EDGE(l) |
		   EXTINTR_CTRL_LEVEL(l);
	u32 val;

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		val = EXTINTR_CTRL_LEVEL(l) | EXTINTR_CTRL_SENSE(l);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		val = EXTINTR_CTRL_LEVEL(l);
		break;
	case IRQ_TYPE_EDGE_RISING:
		val = EXTINTR_CTRL_SENSE(l);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		val = 0;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		val = EXTINTR_CTRL_BOTH_EDGE(l);
		break;
	default:
		return -EINVAL;
	}

	scoped_guard(raw_spinlock_irqsave, &intc->lock) {
		val |= readl(intc->base + EXTINTR_CTRL) & ~mask;
		writel(val, intc->base + EXTINTR_CTRL);
		/* Drop what the old configuration latched */
		bcmbca_ext_clear(intc, BIT(l));
	}

	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(d, handle_fasteoi_irq);
	else
		irq_set_handler_locked(d, handle_fasteoi_ack_irq);

	return irq_chip_set_type_parent(d, IRQ_TYPE_LEVEL_HIGH);
}

static struct irq_chip bcmbca_ext_chip = {
	.name			= "bcmbca-ext",
	.irq_ack		= bcmbca_ext_ack,
	.irq_eoi		= bcmbca_ext_eoi,
	.irq_mask		= bcmbca_ext_mask,
	.irq_unmask		= bcmbca_ext_unmask,
	.irq_set_type		= bcmbca_ext_set_type,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.flags			= IRQCHIP_SET_TYPE_MASKED |
				  IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_MASK_ON_SUSPEND,
};

static void bcmbca_ext_release(struct bcmbca_ext_intc *intc, unsigned int line)
{
	guard(raw_spinlock_irqsave)(&intc->lock);

	writel(BIT(line), intc->base + EXTINTR_MASK_CLEAR);
	bcmbca_ext_route(intc, line, EXTINTR_MAX_PINS - 1);
	__clear_bit(line, &intc->used);
}

static int bcmbca_ext_alloc(struct irq_domain *domain, unsigned int virq,
			    unsigned int nr_irqs, void *arg)
{
	struct bcmbca_ext_intc *intc = domain->host_data;
	irq_hw_number_t pin;
	unsigned int type, line;
	int ret;

	if (WARN_ON(nr_irqs != 1))
		return -EINVAL;

	ret = irq_domain_translate_twocell(domain, arg, &pin, &type);
	if (ret)
		return ret;
	if (pin >= EXTINTR_MAX_PINS || type == IRQ_TYPE_NONE)
		return -EINVAL;

	scoped_guard(raw_spinlock_irqsave, &intc->lock) {
		line = find_first_zero_bit(&intc->used, intc->nr_lines);
		if (line >= intc->nr_lines)
			return -ENOSPC;
		__set_bit(line, &intc->used);
		bcmbca_ext_route(intc, line, pin);
	}

	irq_domain_set_hwirq_and_chip(domain, virq, pin, &bcmbca_ext_chip,
				      &intc->lines[line]);

	ret = irq_domain_alloc_irqs_parent(domain, virq, 1,
					   &intc->lines[line].parent);
	if (ret)
		bcmbca_ext_release(intc, line);

	return ret;
}

static void bcmbca_ext_free(struct irq_domain *domain, unsigned int virq,
			    unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct bcmbca_ext_line *line = irq_data_get_irq_chip_data(d);

	bcmbca_ext_release(line->intc, line->index);
	irq_domain_free_irqs_common(domain, virq, nr_irqs);
}

static const struct irq_domain_ops bcmbca_ext_domain_ops = {
	.translate	= irq_domain_translate_twocell,
	.alloc		= bcmbca_ext_alloc,
	.free		= bcmbca_ext_free,
};

static int bcmbca_ext_probe(struct platform_device *pdev,
			    struct device_node *parent)
{
	struct device_node *np = pdev->dev.of_node;
	struct irq_domain *parent_domain, *domain;
	struct bcmbca_ext_intc *intc;
	unsigned int i;
	int nr_lines;

	parent_domain = irq_find_host(parent);
	if (!parent_domain)
		return -EPROBE_DEFER;

	/* One parent interrupt per line */
	nr_lines = of_irq_count(np);
	if (nr_lines <= 0 || nr_lines > EXTINTR_MAX_LINES)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "need 1 to %u interrupts\n",
				     EXTINTR_MAX_LINES);

	intc = devm_kzalloc(&pdev->dev, struct_size(intc, lines, nr_lines),
			    GFP_KERNEL);
	if (!intc)
		return -ENOMEM;
	intc->nr_lines = nr_lines;
	raw_spin_lock_init(&intc->lock);

	intc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(intc->base))
		return PTR_ERR(intc->base);

	for (i = 0; i < intc->nr_lines; i++) {
		struct bcmbca_ext_line *line = &intc->lines[i];
		struct of_phandle_args args;
		int ret;

		ret = of_irq_parse_one(np, i, &args);
		if (ret)
			return ret;
		of_phandle_args_to_fwspec(args.np, args.args, args.args_count,
					  &line->parent);
		of_node_put(args.np);
		line->intc = intc;
		line->index = i;
	}

	/* Disable, clear and disconnect all lines, like the SDK does */
	writel(GENMASK(intc->nr_lines - 1, 0), intc->base + EXTINTR_MASK_CLEAR);
	scoped_guard(raw_spinlock_irqsave, &intc->lock) {
		bcmbca_ext_clear(intc, GENMASK(intc->nr_lines - 1, 0));
		for (i = 0; i < intc->nr_lines; i++)
			bcmbca_ext_route(intc, i, EXTINTR_MAX_PINS - 1);
	}

	domain = irq_domain_create_hierarchy(parent_domain, 0, EXTINTR_MAX_PINS,
					     of_fwnode_handle(np),
					     &bcmbca_ext_domain_ops, intc);
	if (!domain)
		return -ENOMEM;

	return 0;
}

IRQCHIP_PLATFORM_DRIVER_BEGIN(bcmbca_ext)
IRQCHIP_MATCH("brcm,bcmbca-ext-intc", bcmbca_ext_probe)
IRQCHIP_PLATFORM_DRIVER_END(bcmbca_ext)
MODULE_DESCRIPTION("Broadcom BCA external interrupt controller");
MODULE_LICENSE("GPL");
