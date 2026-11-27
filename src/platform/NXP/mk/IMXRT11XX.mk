#
# NXP i.MXRT 11xx Make file include
#

PLATFORM_SDK := mcux_rt
NXP_FAMILY := imxrt11xx

ifeq ($(TARGET_MCU),MIMXRT1176)
# Builds for the CM7 core only; the CM4 is held in reset. A board .mk may set
# MCUX_CPU first for a different speed grade or package.
MCUX_CPU        ?= MIMXRT1176DVMAA_cm7
# The vendor headers keep the whole 117x range under one RT1170 directory.
MCUX_RT_SERIES  := RT1170
DEVICE_FLAGS    += -DMIMXRT1176 -DIMXRT11XX -DCPU_$(MCUX_CPU)
# External NOR on FlexSPI1; the linker script follows the board's flash size.
MCU_FLASH_SIZE  ?= 65536
LD_SCRIPT       ?= $(LINKER_DIR)/imxrt1176_flexspi_nor.ld
else
$(error Unknown MCU $(TARGET_MCU) for i.MXRT 11xx platform)
endif

include $(TARGET_PLATFORM_DIR)/mk/nxp_common.mk

MCU_COMMON_SRC += NXP/serial_flexio_nxp.c
