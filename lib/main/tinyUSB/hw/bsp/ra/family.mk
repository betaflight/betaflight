FSP_RA = hw/mcu/renesas/fsp/ra/fsp
include $(TOP)/$(BOARD_PATH)/board.mk

# Don't include options setting in .bin file since it create unnecessary large file due to padding
OBJCOPY_BIN_OPTION = --only-section .text --only-section .data --only-section .rodata --only-section .bss

# ----------------------
# Port & Speed Selection
# ----------------------
RHPORT_SPEED ?= OPT_MODE_FULL_SPEED OPT_MODE_HIGH_SPEED
RHPORT_DEVICE ?= 0
RHPORT_HOST ?= 0

# Determine RHPORT_DEVICE_SPEED if not defined
ifndef RHPORT_DEVICE_SPEED
ifeq ($(RHPORT_DEVICE), 0)
  RHPORT_DEVICE_SPEED = $(firstword $(RHPORT_SPEED))
else
  RHPORT_DEVICE_SPEED = $(lastword $(RHPORT_SPEED))
endif
endif

# Determine RHPORT_HOST_SPEED if not defined
ifndef RHPORT_HOST_SPEED
ifeq ($(RHPORT_HOST), 0)
  RHPORT_HOST_SPEED = $(firstword $(RHPORT_SPEED))
else
  RHPORT_HOST_SPEED = $(lastword $(RHPORT_SPEED))
endif
endif

# --------------
# Compiler Flags
# --------------
CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_RAXXX \
	-DBOARD_TUD_RHPORT=${RHPORT_DEVICE} \
	-DBOARD_TUD_MAX_SPEED=${RHPORT_DEVICE_SPEED} \
	-DBOARD_TUH_RHPORT=${RHPORT_HOST} \
	-DBOARD_TUH_MAX_SPEED=${RHPORT_HOST_SPEED}

CFLAGS_GCC += \
  -flto \
	-Wno-error=undef \
	-Wno-error=strict-prototypes \
	-Wno-error=cast-align \
	-Wno-error=cast-qual \
	-Wno-error=unused-but-set-variable \
	-Wno-error=unused-variable \
	-ffreestanding

LDFLAGS_GCC += \
	-nostartfiles -nostdlib \
  -specs=nosys.specs -specs=nano.specs

# -----------------
# Sources & Include
# -----------------
SRC_C += \
	src/portable/renesas/rusb2/dcd_rusb2.c \
	src/portable/renesas/rusb2/hcd_rusb2.c \
	src/portable/renesas/rusb2/rusb2_common.c \
	${BOARD_PATH}/ra_gen/common_data.c \
	${BOARD_PATH}/ra_gen/pin_data.c \
	$(FSP_RA)/src/bsp/cmsis/Device/RENESAS/Source/startup.c \
	$(FSP_RA)/src/bsp/cmsis/Device/RENESAS/Source/system.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_clocks.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_common.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_delay.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_group_irq.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_guard.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_io.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_irq.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_register_protection.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_rom_registers.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_sbrk.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_security.c \
	$(FSP_RA)/src/r_ioport/r_ioport.c \

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(BOARD_PATH)/ra_cfg/fsp_cfg \
	$(TOP)/$(BOARD_PATH)/ra_cfg/fsp_cfg/bsp \
	$(TOP)/$(BOARD_PATH)/ra_gen \
	$(TOP)/lib/CMSIS_6/CMSIS/Core/Include \
	$(TOP)/$(FSP_RA)/src/bsp/cmsis/Device/RENESAS/Include \
	$(TOP)/$(FSP_RA)/inc \
	$(TOP)/$(FSP_RA)/inc/api \
	$(TOP)/$(FSP_RA)/inc/instances \
  $(TOP)/$(FSP_RA)/src/bsp/mcu/all \
	$(TOP)/$(FSP_RA)/src/bsp/mcu/$(MCU_VARIANT) \

ifndef LD_FILE
LD_FILE = $(BOARD_PATH)/script/fsp.ld
endif

LDFLAGS += -L$(TOP)/$(BOARD_PATH)/script
LDFLAGS += -Wl,--defsym=end=__bss_end__

# For freeRTOS port source
# hack to use the port provided by renesas
FREERTOS_PORTABLE_SRC = $(FSP_RA)/src/rm_freertos_port
