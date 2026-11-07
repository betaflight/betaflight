TARGET_MCU        := MIMXRT1062
TARGET_MCU_FAMILY := IMXRT10XX

# 8 MB default boot flash on FlexSPI1; a board config may override it.
MCU_FLASH_SIZE  ?= 8192

include $(TARGET_PLATFORM_DIR)/target/common/target_imxrt.mk
