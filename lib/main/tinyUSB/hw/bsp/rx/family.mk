DEPS_SUBMODULES += hw/mcu/renesas/rx

# Cross Compiler for RX
CROSS_COMPILE = rx-elf-

include $(TOP)/$(BOARD_PATH)/board.mk

CFLAGS += \
  -nostartfiles \
  -ffunction-sections \
  -fdata-sections \
  -fshort-enums \
  -mlittle-endian-data \
  -DSSIZE_MAX=__INT_MAX__

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=redundant-decls

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

SRC_C += \
	src/portable/renesas/rusb2/dcd_rusb2.c \
	src/portable/renesas/rusb2/hcd_rusb2.c \
	$(MCU_DIR)/vects.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(MCU_DIR)

SRC_S += $(MCU_DIR)/start.S

$(BUILD)/$(PROJECT).mot: $(BUILD)/$(PROJECT).elf
	@echo CREATE $@
	$(OBJCOPY) -O srec -I elf32-rx-be-ns $^ $@

# flash using rfp-cli
flash-rfp: $(BUILD)/$(PROJECT).mot
	rfp-cli -device rx65x -tool e2l -if fine -fo id FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF -auth id FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF -auto $^
