UF2_FAMILY_ID = 0x00
SDK_DIR = hw/mcu/infineon/mtb-xmclib-cat3

DEPS_SUBMODULES += ${SDK_DIR}

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

CFLAGS += \
  -flto \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_XMC4000

# mcu driver cause following warnings
#CFLAGS += -Wno-error=shadow -Wno-error=cast-align

SRC_C += \
	src/portable/synopsys/dwc2/dcd_dwc2.c \
	src/portable/synopsys/dwc2/hcd_dwc2.c \
	src/portable/synopsys/dwc2/dwc2_common.c \
	${SDK_DIR}/CMSIS/Infineon/COMPONENT_${MCU_VARIANT}/Source/system_${MCU_VARIANT}.c \
	${SDK_DIR}/Newlib/syscalls.c \
	${SDK_DIR}/XMCLib/src/xmc_gpio.c \
	${SDK_DIR}/XMCLib/src/xmc4_gpio.c \
	${SDK_DIR}/XMCLib/src/xmc4_scu.c \
	${SDK_DIR}/XMCLib/src/xmc_usic.c \
	${SDK_DIR}/XMCLib/src/xmc_uart.c

SRC_S += ${SDK_DIR}/CMSIS/Infineon/COMPONENT_${MCU_VARIANT}/Source/TOOLCHAIN_GCC_ARM/startup_${MCU_VARIANT}.S

INC += \
  $(TOP)/$(BOARD_PATH) \
	$(TOP)/${SDK_DIR}/CMSIS/Core/Include \
	$(TOP)/${SDK_DIR}/CMSIS/Infineon/COMPONENT_${MCU_VARIANT}/Include \
	$(TOP)/${SDK_DIR}/XMCLib/inc
