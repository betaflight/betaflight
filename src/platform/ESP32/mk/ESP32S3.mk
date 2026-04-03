#
# ESP32-S3 Make file include
#
# The top level Makefile adds $(MCU_COMMON_SRC) and $(DEVICE_STDPERIPH_SRC) to SRC collection.
#

DEFAULT_OUTPUT := bin

# Auto-hydrate esp-idf submodule when building ESP32 targets
PLATFORM_SDK := esp_idf
PLATFORM_SDK_STAMP := $(ESP_IDF_STAMP)

# ESP-IDF location (when submodule is hydrated)
ESP_IDF_DIR = $(LIB_MAIN_DIR)/esp-idf

# Override ARM toolchain with Xtensa ESP32-S3 toolchain
# ESP_TOOLS_BIN is resolved in tools.mk; reuse it here
ifneq ($(ESP_TOOLS_BIN),)
  ARM_SDK_PREFIX := $(ESP_TOOLS_BIN)/xtensa-esp32s3-elf-
else
  ARM_SDK_PREFIX := xtensa-esp32s3-elf-
endif

INCLUDE_DIRS += \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/include \
            $(ESP_IDF_DIR)/components/hal/include \
            $(ESP_IDF_DIR)/components/hal/esp32s3/include \
            $(ESP_IDF_DIR)/components/hal/platform_port/include \
            $(ESP_IDF_DIR)/components/soc/esp32s3/include \
            $(ESP_IDF_DIR)/components/soc/esp32s3/register \
            $(ESP_IDF_DIR)/components/soc/include \
            $(ESP_IDF_DIR)/components/esp_rom/include \
            $(ESP_IDF_DIR)/components/esp_rom/esp32s3 \
            $(ESP_IDF_DIR)/components/esp_common/include \
            $(ESP_IDF_DIR)/components/esp_hw_support/include \
            $(ESP_IDF_DIR)/components/esp_system/include \
            $(ESP_IDF_DIR)/components/log/include \
            $(ESP_IDF_DIR)/components/newlib/platform_include \
            $(ESP_IDF_DIR)/components/xtensa/include \
            $(ESP_IDF_DIR)/components/xtensa/esp32s3/include \
            $(ESP_IDF_DIR)/components/esp_timer/include \
            $(ESP_IDF_DIR)/components/freertos/config/include \
            $(ESP_IDF_DIR)/components/freertos/config/xtensa/include

# Architecture flags for Xtensa LX7 (ESP32-S3)
ARCH_FLAGS = -mlongcalls -mtext-section-literals

DEVICE_FLAGS += \
            -DESP32S3 \
            -DESP32

MCU_FLASH_SIZE := 8192

LD_SCRIPT = $(LINKER_DIR)/esp32s3.ld

STARTUP_SRC =

# ROM linker scripts that provide symbols for ROM functions (esp_rom_delay_us, etc.)
ESP_ROM_LD_DIR = $(ESP_IDF_DIR)/components/esp_rom/esp32s3/ld

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
              -T$(ESP_ROM_LD_DIR)/esp32s3.rom.ld \
              -T$(ESP_ROM_LD_DIR)/esp32s3.rom.api.ld \
              $(EXTRA_LD_FLAGS)

# Platform source files (stub implementations)
MCU_COMMON_SRC = \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/serial_pinconfig.c \
            drivers/usb_io.c \
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
            ESP32/light_ws2811strip_esp32.c \
            ESP32/periph_regs_esp32.c

# ESP-IDF SOC peripheral descriptor sources (provide GPIO, SYSTIMER, RMT, etc. symbols)
# Paths are relative to LIB_MAIN_DIR (lib/main) since that's in VPATH
DEVICE_STDPERIPH_SRC = \
            esp-idf/components/soc/esp32s3/gpio_periph.c \
            esp-idf/components/soc/esp32s3/i2c_periph.c \
            esp-idf/components/soc/esp32s3/uart_periph.c \
            esp-idf/components/soc/esp32s3/ledc_periph.c \
            esp-idf/components/soc/esp32s3/rmt_periph.c \
            esp-idf/components/soc/esp32s3/interrupts.c

MCU_EXCLUDES =
