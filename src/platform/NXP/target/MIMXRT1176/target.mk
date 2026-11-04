TARGET_MCU        := MIMXRT1176
TARGET_MCU_FAMILY := IMXRT11XX

# Macronix MX25UM51345G, 64 MB of octal NOR on FlexSPI1; a board config.mk may
# override it.
MCU_FLASH_SIZE  ?= 65536

include $(TARGET_PLATFORM_DIR)/target/common/target_imxrt.mk
