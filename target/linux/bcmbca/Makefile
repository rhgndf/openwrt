# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

BOARD:=bcmbca
BOARDNAME:=Broadcom BCMBCA
FEATURES:=squashfs nand gpio
SUBTARGETS:=cortexa7

KERNEL_PATCHVER:=6.18

define Target/Description
	Build firmware images for Broadcom BCA (Broadband Carrier Access)
	ARM SoC based routers. The subtargets follow the CPU cores of the
	SoCs, like ARCH_BCMBCA in the kernel.
endef

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES += \
	kmod-gpio-button-hotplug kmod-leds-gpio

$(eval $(call BuildTarget))
