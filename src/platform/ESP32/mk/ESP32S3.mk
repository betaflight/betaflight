#
# ESP32-S3 Make file include
#
# The top level Makefile adds $(MCU_COMMON_SRC) and $(DEVICE_STDPERIPH_SRC) to SRC collection.
#

DEFAULT_OUTPUT := bin

# Auto-hydrate esp-idf submodule when building ESP32 targets
PLATFORM_SDK_STAMP := $(ESP_IDF_STAMP)

# ESP-IDF location (when submodule is hydrated)
ESP_IDF_DIR = $(LIB_MAIN_DIR)/esp-idf

# Override ARM toolchain with Xtensa ESP32-S3 toolchain
# Use local install path if available, otherwise fall back to PATH
ESP_TOOLS_BIN := $(firstword $(wildcard $(IDF_TOOLS_PATH)/tools/xtensa-esp32s3-elf/*/xtensa-esp32s3-elf/bin))
ifneq ($(ESP_TOOLS_BIN),)
  ARM_SDK_PREFIX := $(ESP_TOOLS_BIN)/xtensa-esp32s3-elf-
else
  ARM_SDK_PREFIX := xtensa-esp32s3-elf-
endif

INCLUDE_DIRS += \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/include

# Architecture flags for Xtensa LX7 (ESP32-S3)
ARCH_FLAGS = -mlongcalls

DEVICE_FLAGS += \
            -DESP32S3 \
            -DESP32

MCU_FLASH_SIZE := 8192

LD_SCRIPT = $(LINKER_DIR)/esp32s3.ld

STARTUP_SRC =

# Override default LD_FLAGS since the ARM-specific ones don't apply
LD_FLAGS = -lm \
              -nostartfiles \
              -lc \
              -lgcc \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -static \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -T$(LD_SCRIPT) \
              $(EXTRA_LD_FLAGS)

# Platform source files (stub implementations)
MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_i2c_utils.c \
            drivers/serial_pinconfig.c \
            drivers/usb_io.c \
            drivers/dshot.c \
            drivers/adc.c \
            ESP32/adc_esp32.c \
            ESP32/bus_i2c_esp32.c \
            ESP32/bus_spi_esp32.c \
            ESP32/config_flash.c \
            ESP32/debug_esp32.c \
            ESP32/dma_esp32.c \
            ESP32/dshot_esp32.c \
            ESP32/exti_esp32.c \
            ESP32/io_esp32.c \
            ESP32/persistent.c \
            ESP32/pwm_motor_esp32.c \
            ESP32/pwm_servo_esp32.c \
            ESP32/pwm_beeper_esp32.c \
            ESP32/serial_uart_esp32.c \
            ESP32/serial_usb_vcp_esp32.c \
            ESP32/system.c \
            ESP32/light_ws2811strip_esp32.c

# No vendor peripheral library sources yet (will add ESP-IDF components later)
DEVICE_STDPERIPH_SRC =

MCU_EXCLUDES =
