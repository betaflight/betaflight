include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m0plus

CFLAGS += \
  -mthumb \
  -nostdlib -nostartfiles \
  -DCONF_DFLL_OVERWRITE_CALIBRATION=0 \
  -DOSC32K_OVERWRITE_CALIBRATION=0 \
  -DCFG_EXAMPLE_MSC_READONLY \
  -DCFG_EXAMPLE_VIDEO_READONLY \
  -DCFG_TUSB_MCU=OPT_MCU_SAMD11

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=redundant-decls

# SAM driver is flooded with -Wcast-qual which slow down complication significantly
CFLAGS_SKIP += -Wcast-qual

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

SRC_C += \
	src/portable/microchip/samd/dcd_samd.c \
	hw/mcu/microchip/samd11/gcc/system_samd11.c \
	hw/mcu/microchip/samd11/gcc/gcc/startup_samd11.c \
	hw/mcu/microchip/samd11/hal/src/hal_atomic.c \
	hw/mcu/microchip/samd11/hpl/gclk/hpl_gclk.c \
	hw/mcu/microchip/samd11/hpl/pm/hpl_pm.c \
	hw/mcu/microchip/samd11/hpl/sysctrl/hpl_sysctrl.c \

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/hw/mcu/microchip/samd11/ \
	$(TOP)/hw/mcu/microchip/samd11/config \
	$(TOP)/hw/mcu/microchip/samd11/include \
	$(TOP)/hw/mcu/microchip/samd11/hal/include \
	$(TOP)/hw/mcu/microchip/samd11/hal/utils/include \
	$(TOP)/hw/mcu/microchip/samd11/hpl/pm/ \
	$(TOP)/hw/mcu/microchip/samd11/hpl/port \
	$(TOP)/hw/mcu/microchip/samd11/hri \
	$(TOP)/hw/mcu/microchip/samd11/CMSIS/Include \
	$(TOP)/hw/mcu/microchip/samd11/CMSIS/Core/Include
