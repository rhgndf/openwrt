# SPDX-License-Identifier: GPL-2.0-only

define Device/bcm47622
  SOC := bcm47622
  # CFE tries "<boardId>.dtb" first and falls back to this
  CFE_DTB_NAME := 947622.dtb
  CFE_WFI_CHIP_ID := 0x47622
endef

define Device/asus_rt-ax55
  $(Device/bcm47622)
  DEVICE_VENDOR := ASUS
  DEVICE_MODEL := RT-AX55
  DEVICE_DTS := bcm6755-asus-rt-ax55
  CFE_RAM := $(DL_DIR)/asus-rt-ax55-cferam.000
  # Leaves CFE its 8 spare good blocks and room for bad ones in the 408
  # block firmware slot
  IMAGE_SIZE := 50432k
endef
TARGET_DEVICES += asus_rt-ax55
