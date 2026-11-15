TARGET_MCU        := MIMXRT1064
TARGET_MCU_FAMILY := IMXRT10XX

# Internal 4 MB NOR die on FlexSPI2. A per-board config.mk may override it.
MCU_FLASH_SIZE  ?= 4096

include $(TARGET_PLATFORM_DIR)/target/common/target_imxrt.mk
