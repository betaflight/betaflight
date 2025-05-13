TARGET_MCU        := RP2350A
TARGET_MCU_FAMILY := PICO

MCU_FLASH_SIZE  = 4096

DEVICE_FLAGS    += -DPICO_RP2350A=1

# For pico-sdk, define flash-related attributes
# 4194304 = 4 * 1024 * 1024
DEVICE_FLAGS    += \
                   -DPICO_FLASH_SPI_CLKDIV=2 \
                   -DPICO_FLASH_SIZE_BYTES=4194304 \
                   -DPICO_BOOT_STAGE2_CHOOSE_W25Q080=1

# Define some default pins, so that we can debug/trace using pico_stdio (uart, usb)
# or run some pico-examples programs.
# These ones are suitable for a Pico2 board.
DEVICE_FLAGS    += \
                   -DPICO_DEFAULT_UART=0 \
                   -DPICO_DEFAULT_UART_TX_PIN=0 \
                   -DPICO_DEFAULT_UART_RX_PIN=1 \
                   -DPICO_DEFAULT_LED_PIN=25 \
                   -DPICO_DEFAULT_I2C=0 \
                   -DPICO_DEFAULT_I2C_SDA_PIN=4 \
                   -DPICO_DEFAULT_I2C_SCL_PIN=5 \
                   -DPICO_DEFAULT_SPI=0 \
                   -DPICO_DEFAULT_SPI_SCK_PIN=18 \
                   -DPICO_DEFAULT_SPI_TX_PIN=19 \
                   -DPICO_DEFAULT_SPI_RX_PIN=16 \
                   -DPICO_DEFAULT_SPI_CSN_PIN=17
