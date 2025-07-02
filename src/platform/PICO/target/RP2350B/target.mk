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

# Define some default pins, so that we can debug/trace using pico_stdio (uart, usb)
# or run some pico-examples programs.
# These ones are suitable for a Laurel board, with UART1 for stdio.
DEVICE_FLAGS    += \
                   -DPICO_DEFAULT_UART=0 \
                   -DPICO_DEFAULT_UART_TX_PIN=34 \
                   -DPICO_DEFAULT_UART_RX_PIN=35 \
                   -DPICO_DEFAULT_LED_PIN=6 \
                   -DPICO_DEFAULT_I2C=0 \
                   -DPICO_DEFAULT_I2C_SDA_PIN=44 \
                   -DPICO_DEFAULT_I2C_SCL_PIN=45 \
                   -DPICO_DEFAULT_SPI=0 \
                   -DPICO_DEFAULT_SPI_SCK_PIN=2 \
                   -DPICO_DEFAULT_SPI_TX_PIN=3 \
                   -DPICO_DEFAULT_SPI_RX_PIN=4 \
                   -DPICO_DEFAULT_SPI_CSN_PIN=1
