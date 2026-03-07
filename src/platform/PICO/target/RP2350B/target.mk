TARGET_MCU        := RP2350B
TARGET_MCU_FAMILY := RP2350

MCU_FLASH_SIZE  = 8192

# In pico-sdk, PICO_RP2350A=0 means RP2350B family.
DEVICE_FLAGS    += -DPICO_RP2350A=0

# For pico-sdk, define flash-related attributes
# 8388608 = 8 * 1024 * 1024
DEVICE_FLAGS    += \
                   -DPICO_FLASH_SPI_CLKDIV=2 \
                   -DPICO_FLASH_SIZE_BYTES=8388608 \
                   -DPICO_BOOT_STAGE2_CHOOSE_W25Q080=1
