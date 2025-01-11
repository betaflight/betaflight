UF2_FAMILY_ID = 0x55114460

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

SDK_DIR = hw/mcu/microchip/${SAM_FAMILY}

CFLAGS += \
  -flto \
  -DCFG_TUSB_MCU=OPT_MCU_SAMD51

# SAM driver is flooded with -Wcast-qual which slow down complication significantly
CFLAGS_SKIP += -Wcast-qual

LDFLAGS_GCC += \
  -nostdlib -nostartfiles \
  --specs=nosys.specs --specs=nano.specs

SRC_C += \
	src/portable/microchip/samd/dcd_samd.c \
	${SDK_DIR}/gcc/gcc/startup_${SAM_FAMILY}.c \
	${SDK_DIR}/gcc/system_${SAM_FAMILY}.c \
	${SDK_DIR}/hpl/gclk/hpl_gclk.c \
	${SDK_DIR}/hpl/mclk/hpl_mclk.c \
	${SDK_DIR}/hpl/osc32kctrl/hpl_osc32kctrl.c \
	${SDK_DIR}/hpl/oscctrl/hpl_oscctrl.c \
	${SDK_DIR}/hal/src/hal_atomic.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/${SDK_DIR} \
	$(TOP)/${SDK_DIR}/config \
	$(TOP)/${SDK_DIR}/include \
	$(TOP)/${SDK_DIR}/hal/include \
	$(TOP)/${SDK_DIR}/hal/utils/include \
	$(TOP)/${SDK_DIR}/hpl/port \
	$(TOP)/${SDK_DIR}/hri \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \

# flash using bossac at least version 1.8
# can be found in arduino15/packages/arduino/tools/bossac/
# Add it to your PATH or change BOSSAC variable to match your installation
BOSSAC = bossac

flash-bossac: $(BUILD)/$(PROJECT).bin
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(BOSSAC) --port=$(SERIAL) -U -i --offset=0x4000 -e -w $^ -R

# flash using edbg from https://github.com/ataradov/edbg
flash-edbg: $(BUILD)/$(PROJECT).bin
	edbg --verbose -t $(MCU) -pv -f $<
