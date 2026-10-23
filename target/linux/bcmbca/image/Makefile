# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

CFE_IMAGE := $(CURDIR)/bcmbca-cfe-image.py

DEVICE_VARS += CFE_RAM CFE_DTB_NAME CFE_WFI_CHIP_ID CFE_WFI_FLASH_TYPE

# Boot-file volume content: cferam.000, vmlinux.lz and the DTB.
# cferam.000 isn't shipped with OpenWrt, it has to be extracted from the
# stock firmware to CFE_RAM.
define Build/cfe-bootfs
	$(CFE_IMAGE) bootfs --kernel $@ --elf $(KDIR)/vmlinux.elf \
		--dtb $(KDIR)/image-$(DEVICE_DTS).dtb --dtb-name $(CFE_DTB_NAME) \
		--cferam $(CFE_RAM) -o $@
endef

# Raw pureUBI image of one firmware slot (uncommitted METADATA)
define Build/cfe-ubi
	$(CFE_IMAGE) ubi --bootfs $(IMAGE_KERNEL) --rootfs $(IMAGE_ROOTFS) \
		--peb-size $(BLOCKSIZE:%k=%KiB) --page-size $(PAGESIZE) -o $@
endef

# WFI tag for CFE rescue (TFTP and web) and the stock web UI
# (version 0x5732 is "normal", flag 0x2 "secure BootROM")
define Build/cfe-wfi-tag
	$(TOPDIR)/scripts/cfe-wfi-tag.py --little-endian \
		--input-file $@ --output-file $@.new \
		--version 0x5732 --chip-id $(CFE_WFI_CHIP_ID) \
		--flash-type $(CFE_WFI_FLASH_TYPE) --flags 0x2
	mv $@.new $@
endef

define Device/Default
  PROFILES := Default
  DEVICE_DTS_DIR := ../dts
  KERNEL_DEPENDS = $$(wildcard ../dts/$$(DEVICE_DTS).dts) $$(CFE_RAM) $(CFE_IMAGE)
  KERNEL_NAME := Image
  # 4 MiB dictionary, as in the vendor kernels
  KERNEL := kernel-bin | lzma -d22 | cfe-bootfs
  FILESYSTEMS := squashfs
  BLOCKSIZE := 128k
  PAGESIZE := 2048
  # NAND with 128 KiB blocks
  CFE_WFI_FLASH_TYPE := 3
  IMAGES := factory.bin sysupgrade.bin
  IMAGE/factory.bin := cfe-ubi | cfe-wfi-tag | check-size
  IMAGE/sysupgrade.bin := cfe-ubi | append-metadata | check-size
endef

include $(SUBTARGET).mk

$(eval $(call BuildImage))
