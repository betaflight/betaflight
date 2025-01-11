include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

SDK_DIR = hw/mcu/microchip/samg55

CFLAGS += \
  -flto \
  -DCFG_TUSB_MCU=OPT_MCU_SAMG

# suppress following warnings from mcu driver
CFLAGS += -Wno-error=undef -Wno-error=null-dereference -Wno-error=redundant-decls

# SAM driver is flooded with -Wcast-qual which slow down complication significantly
CFLAGS_SKIP += -Wcast-qual

LDFLAGS_GCC += \
  -nostdlib -nostartfiles \
  --specs=nosys.specs --specs=nano.specs \

SRC_C += \
	src/portable/microchip/samg/dcd_samg.c \
	${SDK_DIR}/samg55/gcc/gcc/startup_samg55.c \
	${SDK_DIR}/samg55/gcc/system_samg55.c \
	${SDK_DIR}/hal/src/hal_atomic.c \
	${SDK_DIR}/hpl/core/hpl_init.c \
	${SDK_DIR}/hpl/usart/hpl_usart.c \
	${SDK_DIR}/hpl/pmc/hpl_pmc.c \

INC += \
  $(TOP)/$(FAMILY_PATH) \
  $(TOP)/$(BOARD_PATH) \
	$(TOP)/${SDK_DIR} \
	$(TOP)/${SDK_DIR}/config \
	$(TOP)/${SDK_DIR}/samg55/include \
	$(TOP)/${SDK_DIR}/hal/include \
	$(TOP)/${SDK_DIR}/hal/utils/include \
	$(TOP)/${SDK_DIR}/hpl/core \
	$(TOP)/${SDK_DIR}/hpl/pio \
	$(TOP)/${SDK_DIR}/hpl/pmc \
	$(TOP)/${SDK_DIR}/hri \
	$(TOP)/${SDK_DIR}/CMSIS/Core/Include

# flash using edbg from https://github.com/ataradov/edbg
flash-edbg: $(BUILD)/$(PROJECT).bin
	edbg --verbose -t samg55 -pv -f $<
