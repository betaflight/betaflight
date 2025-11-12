TARGET_MCU        := RP2350A
TARGET_MCU_FAMILY := RP2350

MCU_FLASH_SIZE  = 4096

DEVICE_FLAGS    += -DPICO_RP2350A=1

# For pico-sdk, define flash-related attributes
# 4194304 = 4 * 1024 * 1024
DEVICE_FLAGS    += \
                   -DPICO_FLASH_SPI_CLKDIV=2 \
                   -DPICO_FLASH_SIZE_BYTES=4194304 \
                   -DPICO_BOOT_STAGE2_CHOOSE_W25Q080=1
