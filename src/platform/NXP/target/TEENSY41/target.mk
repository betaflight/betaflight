TARGET_MCU        := MIMXRT1062
TARGET_MCU_FAMILY := IMXRT10XX

# Winbond W25Q64JV (8 MB) on the Teensy 4.1 module, on FlexSPI1.
MCU_FLASH_SIZE  := 8192
LD_SCRIPT       := $(LINKER_DIR)/imxrt1062_flexspi_nor_8m.ld

include $(TARGET_PLATFORM_DIR)/target/common/target_imxrt.mk
