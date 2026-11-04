#
# NXP i.MXRT 10xx Make file include
#

PLATFORM_SDK := mcux_rt
NXP_FAMILY := imxrt10xx

ifeq ($(TARGET_MCU),MIMXRT1064)
# A board .mk may set MCUX_CPU first for a different speed grade or package.
MCUX_CPU        ?= MIMXRT1064DVJ6B
MCUX_RT_SERIES  := RT1064
DEVICE_FLAGS    += -DMIMXRT1064 -DIMXRT10XX -DCPU_$(MCUX_CPU)
# 4 MB NOR die stacked in the package, on FlexSPI2.
MCU_FLASH_SIZE  ?= 4096
LD_SCRIPT       ?= $(LINKER_DIR)/imxrt1064_flexspi_nor.ld
else ifeq ($(TARGET_MCU),MIMXRT1062)
MCUX_CPU        ?= MIMXRT1062DVJ6B
# The vendor headers keep RT1061 and RT1062 under one RT1060 directory.
MCUX_RT_SERIES  := RT1060
DEVICE_FLAGS    += -DMIMXRT1062 -DIMXRT10XX -DCPU_$(MCUX_CPU)
# External NOR on FlexSPI1; the linker script follows the board's flash size.
MCU_FLASH_SIZE  ?= 8192
LD_SCRIPT       ?= $(LINKER_DIR)/imxrt1062_flexspi_nor_8m.ld
else
$(error Unknown MCU $(TARGET_MCU) for i.MXRT 10xx platform)
endif

include $(TARGET_PLATFORM_DIR)/mk/nxp_common.mk
