CROSS_COMPILE = msp430-elf-
DEPS_SUBMODULES += hw/mcu/ti
SKIP_NANOLIB = 1

SDK_DIR = hw/mcu/ti/msp430/msp430-gcc-support-files/include

include $(TOP)/$(BOARD_PATH)/board.mk

CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_MSP430x5xx \
	-DCFG_EXAMPLE_MSC_READONLY \
	-DCFG_TUD_ENDPOINT0_SIZE=8

LDFLAGS += -L${TOP}/${SDK_DIR}

SRC_C += src/portable/ti/msp430x5xx/dcd_msp430x5xx.c

INC += \
	${TOP}/${SDK_DIR} \
	$(TOP)/$(BOARD_PATH)

# export for libmsp430.so to same installation
ifneq ($(OS),Windows_NT)
export LD_LIBRARY_PATH=$(dir $(shell which MSP430Flasher))
endif

# flash target using TI MSP430-Flasher
# http://www.ti.com/tool/MSP430-FLASHER
# Please add its installation dir to PATH
flash: $(BUILD)/$(PROJECT).hex
	MSP430Flasher -w $< -z [VCC]

# flash target using mspdebug.
flash-mspdebug: $(BUILD)/$(PROJECT).elf
	$(MSPDEBUG) tilib "prog $<" --allow-fw-update
