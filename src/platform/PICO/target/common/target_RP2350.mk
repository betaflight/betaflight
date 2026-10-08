# Default flash size (for Betaflight checks), can override in config.mk by setting TARGET_FLASH_SIZE
MCU_FLASH_SIZE  = 8192

# For pico-sdk, define flash-related attributes
# A definition in config.mk, if present, will take precendence
PICO_FLASH_DEFINES ?= \
                   -DPICO_FLASH_SPI_CLKDIV=2 \
                   -DPICO_FLASH_SIZE_BYTES=8388608 \
                   -DPICO_BOOT_STAGE2_CHOOSE_W25Q080=1

DEVICE_FLAGS    += $(PICO_FLASH_DEFINES)
