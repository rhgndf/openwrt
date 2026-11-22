// SPDX-License-Identifier: GPL-2.0-only
/*
 * Broadcom BCA "CLED" LED controller
 *
 * This controller is found on BCM47622, BCM63178, BCM6756, BCM6855, BCM4912
 * and newer BCA SoCs. Its register layout is not compatible with the older
 * controller handled by leds-bcm63138: every LED has its own configuration
 * block holding the brightness and the flash (blink) rate, and a change to
 * it only takes effect after the LED is activated through CH_ACTIVATE.
 *
 * Only parallel LEDs under software control are supported. Serial LEDs,
 * LEDs driven by hardware (network) sources, the output crossbar of the
 * newer controller versions and the other per-LED effects are not.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/leds.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>

#define CLED_HW_EN			0x004	/* 1: LED follows its HW source */
#define CLED_SW_SET			0x010
#define CLED_PAR_POLARITY		0x018	/* 1: active high, 0: active low */
#define CLED_CH_ACTIVATE		0x01c
#define CLED_CH_CFG0(n)			(0x020 + (n) * 0x10)
#define  CLED_CFG0_FLASH		GENMASK(5, 3)
#define  CLED_CFG0_BRIGHT		GENMASK(13, 6)
#define CLED_SW_CLEAR			0x444
#define CLED_SW_STATUS			0x448

/* Full scale used by the vendor driver for the 8-bit brightness field */
#define CLED_MAX_BRIGHTNESS		128
#define CLED_DEFAULT_BLINK_MS		320
#define CLED_ACTIVATE_TIMEOUT_US	100

/**
 * struct bcmbca_cled_data - controller version specific data
 * @num_leds: number of LED channels
 * @has_sw_clear: SW_SET is write-1-to-set, with SW_CLEAR and SW_STATUS
 *                next to it. Without it SW_SET is a plain data register.
 */
struct bcmbca_cled_data {
	unsigned int num_leds;
	bool has_sw_clear;
};

struct bcmbca_cled {
	struct device *dev;
	void __iomem *base;
	const struct bcmbca_cled_data *data;
	/* Serialises read-modify-write accesses and channel activation */
	spinlock_t lock;
};

struct bcmbca_cled_led {
	struct led_classdev cdev;
	struct bcmbca_cled *cled;
	u32 num;
};

/*
 * Upper delay limit in ms for flash rate codes 1..7, as used by the vendor
 * driver. The vendor driver uses code 7 for any longer delay; the limit for
 * code 7 is extrapolated so that longer delays fall back to software blink.
 */
static const unsigned int bcmbca_cled_flash_max_ms[] = {
	60, 120, 240, 480, 960, 1920, 3840,
};

static inline struct bcmbca_cled_led *to_bcmbca_cled_led(struct led_classdev *cdev)
{
	return container_of(cdev, struct bcmbca_cled_led, cdev);
}

static void bcmbca_cled_rmw(struct bcmbca_cled *cled, unsigned int reg,
			    u32 mask, u32 val)
{
	writel((readl(cled->base + reg) & ~mask) | val, cled->base + reg);
}

static void bcmbca_cled_sw_set(struct bcmbca_cled_led *led, bool on)
{
	struct bcmbca_cled *cled = led->cled;
	u32 bit = BIT(led->num);

	if (cled->data->has_sw_clear)
		writel(bit, cled->base + (on ? CLED_SW_SET : CLED_SW_CLEAR));
	else
		bcmbca_cled_rmw(cled, CLED_SW_SET, bit, on ? bit : 0);
}

static bool bcmbca_cled_sw_get(struct bcmbca_cled_led *led)
{
	struct bcmbca_cled *cled = led->cled;
	unsigned int reg = cled->data->has_sw_clear ? CLED_SW_STATUS : CLED_SW_SET;

	return readl(cled->base + reg) & BIT(led->num);
}

static void bcmbca_cled_update_cfg0(struct bcmbca_cled_led *led, u32 mask,
				    u32 val)
{
	struct bcmbca_cled *cled = led->cled;
	u32 bit = BIT(led->num);
	u32 act;
	int ret;

	bcmbca_cled_rmw(cled, CLED_CH_CFG0(led->num), mask, val);

	/* Latch the new configuration, the bit clears once it is applied */
	writel(bit, cled->base + CLED_CH_ACTIVATE);
	ret = readl_poll_timeout_atomic(cled->base + CLED_CH_ACTIVATE, act,
					!(act & bit), 1,
					CLED_ACTIVATE_TIMEOUT_US);
	if (ret)
		dev_warn_once(cled->dev, "LED %u: activation timed out\n",
			      led->num);
}

static void bcmbca_cled_brightness_set(struct led_classdev *cdev,
				       enum led_brightness value)
{
	struct bcmbca_cled_led *led = to_bcmbca_cled_led(cdev);

	guard(spinlock_irqsave)(&led->cled->lock);

	if (value) {
		bcmbca_cled_update_cfg0(led, CLED_CFG0_BRIGHT,
					FIELD_PREP(CLED_CFG0_BRIGHT, value));
		bcmbca_cled_sw_set(led, true);
	} else {
		/* Switching off also stops hardware blinking */
		bcmbca_cled_sw_set(led, false);
		bcmbca_cled_update_cfg0(led, CLED_CFG0_FLASH, 0);
	}
}

static unsigned int bcmbca_cled_flash_rate(unsigned long delay)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(bcmbca_cled_flash_max_ms); i++)
		if (delay <= bcmbca_cled_flash_max_ms[i])
			return i + 1;

	return 0;
}

static int bcmbca_cled_blink_set(struct led_classdev *cdev,
				 unsigned long *delay_on,
				 unsigned long *delay_off)
{
	struct bcmbca_cled_led *led = to_bcmbca_cled_led(cdev);
	unsigned int brightness = cdev->brightness ?: cdev->max_brightness;
	unsigned int rate;

	if (!*delay_on && !*delay_off) {
		*delay_on = CLED_DEFAULT_BLINK_MS;
		*delay_off = CLED_DEFAULT_BLINK_MS;
	}

	/* The hardware only blinks with equal on and off times */
	if (!*delay_on || !*delay_off)
		return -EINVAL;

	rate = bcmbca_cled_flash_rate(*delay_on);
	if (!rate || rate != bcmbca_cled_flash_rate(*delay_off))
		return -EINVAL;

	guard(spinlock_irqsave)(&led->cled->lock);

	bcmbca_cled_update_cfg0(led, CLED_CFG0_FLASH | CLED_CFG0_BRIGHT,
				FIELD_PREP(CLED_CFG0_FLASH, rate) |
				FIELD_PREP(CLED_CFG0_BRIGHT, brightness));
	bcmbca_cled_sw_set(led, true);

	return 0;
}

static int bcmbca_cled_add_led(struct bcmbca_cled *cled,
			       struct fwnode_handle *fwnode)
{
	struct led_init_data init_data = { .fwnode = fwnode };
	struct device *dev = cled->dev;
	struct bcmbca_cled_led *led;
	struct pinctrl *pinctrl;
	bool active_low;
	u32 bit, cfg0;
	int ret;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	ret = fwnode_property_read_u32(fwnode, "reg", &led->num);
	if (ret)
		return dev_err_probe(dev, ret, "%pfw: missing reg\n", fwnode);

	if (led->num >= cled->data->num_leds)
		return dev_err_probe(dev, -EINVAL, "%pfw: invalid reg %u\n",
				     fwnode, led->num);

	led->cled = cled;
	led->cdev.max_brightness = CLED_MAX_BRIGHTNESS;
	led->cdev.brightness_set = bcmbca_cled_brightness_set;
	led->cdev.blink_set = bcmbca_cled_blink_set;

	active_low = fwnode_property_read_bool(fwnode, "active-low");
	bit = BIT(led->num);

	/*
	 * Only the registers and bits of this LED are touched, so LEDs not
	 * described in DT keep the configuration set up by the bootloader.
	 */
	scoped_guard(spinlock_irqsave, &cled->lock) {
		bcmbca_cled_rmw(cled, CLED_HW_EN, bit, 0);
		bcmbca_cled_rmw(cled, CLED_PAR_POLARITY, bit,
				active_low ? 0 : bit);

		switch (led_init_default_state_get(fwnode)) {
		case LEDS_DEFSTATE_KEEP:
			cfg0 = readl(cled->base + CLED_CH_CFG0(led->num));
			if (bcmbca_cled_sw_get(led))
				led->cdev.brightness =
					min(FIELD_GET(CLED_CFG0_BRIGHT, cfg0),
					    CLED_MAX_BRIGHTNESS);
			break;
		case LEDS_DEFSTATE_ON:
			led->cdev.brightness = CLED_MAX_BRIGHTNESS;
			fallthrough;
		default:
			bcmbca_cled_update_cfg0(led,
						CLED_CFG0_FLASH | CLED_CFG0_BRIGHT,
						FIELD_PREP(CLED_CFG0_BRIGHT,
							   led->cdev.brightness));
			bcmbca_cled_sw_set(led, led->cdev.brightness);
			break;
		}
	}

	ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
	if (ret)
		return dev_err_probe(dev, ret, "%pfw: failed to register\n",
				     fwnode);

	/* Mux the pin to the LED controller only once its state is set up */
	pinctrl = devm_pinctrl_get_select_default(led->cdev.dev);
	if (IS_ERR(pinctrl) && PTR_ERR(pinctrl) != -ENODEV)
		return dev_err_probe(dev, PTR_ERR(pinctrl),
				     "%pfw: failed to select pins\n", fwnode);

	return 0;
}

static int bcmbca_cled_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcmbca_cled *cled;
	int ret;

	cled = devm_kzalloc(dev, sizeof(*cled), GFP_KERNEL);
	if (!cled)
		return -ENOMEM;

	cled->dev = dev;
	cled->data = device_get_match_data(dev);
	spin_lock_init(&cled->lock);

	cled->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cled->base))
		return PTR_ERR(cled->base);

	device_for_each_child_node_scoped(dev, child) {
		ret = bcmbca_cled_add_led(cled, child);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct bcmbca_cled_data bcm47622_cled_data = {
	.num_leds = 32,
};

static const struct of_device_id bcmbca_cled_of_match[] = {
	{ .compatible = "brcm,bcm6755-leds", .data = &bcm47622_cled_data },
	{ }
};
MODULE_DEVICE_TABLE(of, bcmbca_cled_of_match);

static struct platform_driver bcmbca_cled_driver = {
	.probe = bcmbca_cled_probe,
	.driver = {
		.name = "leds-bcmbca-cled",
		.of_match_table = bcmbca_cled_of_match,
	},
};
module_platform_driver(bcmbca_cled_driver);

MODULE_AUTHOR("Ho Jie Feng <hjf3108@gmail.com>");
MODULE_DESCRIPTION("Broadcom BCA CLED LED controller driver");
MODULE_LICENSE("GPL");
