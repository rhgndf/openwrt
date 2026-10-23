ARCH:=arm
SUBTARGET:=cortexa7
BOARDNAME:=ARMv7 Cortex-A7
CPU_TYPE:=cortex-a7
CPU_SUBTYPE:=neon-vfpv4
KERNELNAME:=Image

define Target/Description
	Build firmware images for Broadcom BCA routers with Cortex-A7 based
	SoCs (ARCH_BCMBCA_CORTEXA7), such as the BCM47622 / BCM6755.
endef
