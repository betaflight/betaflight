UF2_FAMILY_ID = 0xADA52840

NRFX_DIR = hw/mcu/nordic/nrfx

include $(TOP)/$(BOARD_PATH)/board.mk

# nRF52 is cortex-m4, nRF53 is cortex-m33
CPU_CORE ?= cortex-m4

CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_NRF5X \
  -DCONFIG_GPIO_AS_PINRESET \
  -D__STARTUP_CLEAR_BSS

#CFLAGS += -nostdlib
#CFLAGS += -D__START=main

# suppress warning caused by vendor mcu driver
CFLAGS_GCC += \
  -flto \
  -Wno-error=undef \
  -Wno-error=unused-parameter \
  -Wno-error=unused-variable \
  -Wno-error=cast-align \
  -Wno-error=cast-qual \
  -Wno-error=redundant-decls \

LDFLAGS_GCC += \
  -nostartfiles \
  --specs=nosys.specs --specs=nano.specs \
  -L$(TOP)/${NRFX_DIR}/mdk

LDFLAGS_CLANG += \
  -L$(TOP)/${NRFX_DIR}/mdk \

SRC_C += \
  src/portable/nordic/nrf5x/dcd_nrf5x.c \
	${NRFX_DIR}/helpers/nrfx_flag32_allocator.c \
	${NRFX_DIR}/drivers/src/nrfx_gpiote.c \
  ${NRFX_DIR}/drivers/src/nrfx_power.c \
  ${NRFX_DIR}/drivers/src/nrfx_spim.c \
  ${NRFX_DIR}/drivers/src/nrfx_uarte.c \
  ${NRFX_DIR}/mdk/system_$(MCU_VARIANT).c \
  ${NRFX_DIR}/soc/nrfx_atomic.c

INC += \
  $(TOP)/$(BOARD_PATH) \
  $(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
  $(TOP)/${NRFX_DIR} \
  $(TOP)/${NRFX_DIR}/mdk \
  $(TOP)/${NRFX_DIR}/hal \
  $(TOP)/${NRFX_DIR}/drivers/include \
  $(TOP)/${NRFX_DIR}/drivers/src \

SRC_S += ${NRFX_DIR}/mdk/gcc_startup_$(MCU_VARIANT).S

ASFLAGS += -D__HEAP_SIZE=0

# For flash-jlink target
JLINK_DEVICE ?= $(MCU_VARIANT)_xxaa
