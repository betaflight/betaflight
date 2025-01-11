MCU_DIR = hw/mcu/broadcom
include $(TOP)/$(BOARD_PATH)/board.mk

CFLAGS += \
	-Wall \
	-O0 \
	-ffreestanding \
	-nostdlib \
	-nostartfiles \
	--specs=nosys.specs \
	-mgeneral-regs-only \
	-std=c17

CROSS_COMPILE = aarch64-none-elf-

# mcu driver cause following warnings
CFLAGS += -Wno-error=cast-qual -Wno-error=redundant-decls

SRC_C += \
	src/portable/synopsys/dwc2/dcd_dwc2.c \
	src/portable/synopsys/dwc2/hcd_dwc2.c \
	src/portable/synopsys/dwc2/dwc2_common.c \
	$(MCU_DIR)/broadcom/gen/interrupt_handlers.c \
	$(MCU_DIR)/broadcom/gpio.c \
	$(MCU_DIR)/broadcom/interrupts.c \
	$(MCU_DIR)/broadcom/mmu.c \
	$(MCU_DIR)/broadcom/caches.c \
	$(MCU_DIR)/broadcom/vcmailbox.c

LD_FILE = $(MCU_DIR)/broadcom/link8.ld

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(MCU_DIR) \
	$(TOP)/lib/CMSIS_5/CMSIS/Core_A/Include

SRC_S += $(MCU_DIR)/broadcom/boot8.s

$(BUILD)/kernel8.img: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O binary $^ $@

# Copy to kernel to netboot drive or SD card
# Change destinaation to fit your need
flash: $(BUILD)/kernel8.img
	@$(CP) $< /home/$(USER)/Documents/code/pi_tinyusb/boot_cpy
