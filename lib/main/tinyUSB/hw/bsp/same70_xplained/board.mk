DEPS_SUBMODULES += hw/mcu/microchip
ASF_DIR = hw/mcu/microchip/same70

CFLAGS += \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m7 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -nostdlib -nostartfiles \
  -D__SAME70Q21B__ \
  -DCFG_TUSB_MCU=OPT_MCU_SAMX7X

# suppress following warnings from mcu driver
CFLAGS += -Wno-error=unused-parameter -Wno-error=cast-align -Wno-error=redundant-decls

# SAM driver is flooded with -Wcast-qual which slow down complication significantly
CFLAGS_SKIP += -Wcast-qual

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

# All source paths should be relative to the top level.
LD_FILE = $(ASF_DIR)/same70b/gcc/gcc/same70q21b_flash.ld

SRC_C += \
	src/portable/microchip/samx7x/dcd_samx7x.c \
	$(ASF_DIR)/same70b/gcc/gcc/startup_same70q21b.c \
	$(ASF_DIR)/same70b/gcc/system_same70q21b.c \
	$(ASF_DIR)/hpl/core/hpl_init.c \
	$(ASF_DIR)/hpl/usart/hpl_usart.c \
	$(ASF_DIR)/hpl/pmc/hpl_pmc.c \
	$(ASF_DIR)/hal/src/hal_usart_async.c \
	$(ASF_DIR)/hal/src/hal_io.c \
	$(ASF_DIR)/hal/src/hal_atomic.c \
	$(ASF_DIR)/hal/utils/src/utils_ringbuffer.c

INC += \
  $(TOP)/hw/bsp/$(BOARD) \
	$(TOP)/$(ASF_DIR) \
	$(TOP)/$(ASF_DIR)/config \
	$(TOP)/$(ASF_DIR)/same70b/include \
	$(TOP)/$(ASF_DIR)/hal/include \
	$(TOP)/$(ASF_DIR)/hal/utils/include \
	$(TOP)/$(ASF_DIR)/hpl/core \
	$(TOP)/$(ASF_DIR)/hpl/pio \
	$(TOP)/$(ASF_DIR)/hpl/pmc \
	$(TOP)/$(ASF_DIR)/hri \
	$(TOP)/$(ASF_DIR)/CMSIS/Core/Include

# For freeRTOS port source
FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/ARM_CM7

# For flash-jlink target
JLINK_DEVICE = SAME70Q21B

# flash using edbg from https://github.com/ataradov/edbg
# Note: SAME70's GPNVM1 must be set to 1 to boot from flash with
# 	edbg -t same70 -F w0,1,1
flash: $(BUILD)/$(PROJECT).bin
	edbg --verbose -t same70 -pv -f $<
