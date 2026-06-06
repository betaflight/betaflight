#
# ESP32-P4 Make file include
#
# The top level Makefile adds $(MCU_COMMON_SRC) and $(DEVICE_STDPERIPH_SRC) to SRC collection.
#

DEFAULT_OUTPUT := bin

# Auto-hydrate esp-idf submodule when building ESP32 targets
PLATFORM_SDK := esp_idf
PLATFORM_SDK_STAMP := $(ESP_IDF_STAMP)

# ESP-IDF location (when submodule is hydrated)
ESP_IDF_DIR = $(LIB_MODULES_DIR)/esp-idf

# Override ARM toolchain with the RISC-V ESP toolchain
# ESP_TOOLS_RISCV_BIN is resolved in tools.mk
ifneq ($(ESP_TOOLS_RISCV_BIN),)
  ARM_SDK_PREFIX := $(ESP_TOOLS_RISCV_BIN)/riscv32-esp-elf-
else
  ARM_SDK_PREFIX := riscv32-esp-elf-
endif

INCLUDE_DIRS += \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/include \
            $(ESP_IDF_DIR)/components/hal/include \
            $(ESP_IDF_DIR)/components/hal/esp32p4/include \
            $(ESP_IDF_DIR)/components/hal/platform_port/include \
            $(ESP_IDF_DIR)/components/soc/esp32p4/include \
            $(ESP_IDF_DIR)/components/soc/esp32p4/register \
            $(ESP_IDF_DIR)/components/soc/esp32p4/register/hw_ver1 \
            $(ESP_IDF_DIR)/components/soc/include \
            $(ESP_IDF_DIR)/components/esp_rom/include \
            $(ESP_IDF_DIR)/components/esp_rom/esp32p4 \
            $(ESP_IDF_DIR)/components/esp_common/include \
            $(ESP_IDF_DIR)/components/esp_hw_support/include \
            $(ESP_IDF_DIR)/components/esp_system/include \
            $(ESP_IDF_DIR)/components/log/include \
            $(ESP_IDF_DIR)/components/newlib/platform_include \
            $(ESP_IDF_DIR)/components/riscv/include \
            $(ESP_IDF_DIR)/components/esp_timer/include \
            $(ESP_IDF_DIR)/components/freertos/config/include \
            $(ESP_IDF_DIR)/components/freertos/config/riscv/include

# Architecture flags for RISC-V ESP32-P4 (rv32imafc + Zicsr/Zifencei +
# Espressif loop/vector extensions, hard-float ilp32f ABI). Mirrors
# IDF's tools/cmake/toolchain-esp32p4.cmake.
ARCH_FLAGS = -march=rv32imafc_zicsr_zifencei_xesploop_xespv -mabi=ilp32f

DEVICE_FLAGS += \
            -DESP32P4 \
            -DESP32

MCU_FLASH_SIZE := 16384

LD_SCRIPT = $(LINKER_DIR)/esp32p4.ld

STARTUP_SRC =

ESP_ROM_LD_DIR = $(ESP_IDF_DIR)/components/esp_rom/esp32p4/ld

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
              -T$(ESP_ROM_LD_DIR)/esp32p4.rom.ld \
              -T$(ESP_ROM_LD_DIR)/esp32p4.rom.api.ld \
              $(EXTRA_LD_FLAGS)

# Platform source files. Skeleton scope: mirrors the ESP32 (WROOM)
# driver set. Chip-specific divergences (HSPI/VSPI signals, APB I2C
# clock, S3 peripheral struct addresses) are patched per-chip; runtime
# behaviour still targets S3-shaped peripherals until follow-up driver
# port work lands.
MCU_COMMON_SRC = \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/serial_pinconfig.c \
            drivers/adc.c \
            ESP32/adc_esp32.c \
            drivers/bus_spi_config.c \
            ESP32/bus_i2c_esp32.c \
            ESP32/bus_spi_esp32.c \
            ESP32/config_flash.c \
            ESP32/debug_esp32.c \
            ESP32/dma_stub_esp32.c \
            ESP32/dshot_esp32.c \
            ESP32/exti_esp32.c \
            ESP32/interrupt_esp32.c \
            ESP32/io_esp32.c \
            ESP32/persistent.c \
            ESP32/pwm_motor_esp32.c \
            ESP32/pwm_servo_esp32.c \
            ESP32/pwm_beeper_esp32.c \
            ESP32/serial_uart_esp32.c \
            ESP32/system.c \
            ESP32/light_ws2811strip_esp32.c \
            ESP32/timer_esp32.c \
            ESP32/periph_regs_esp32.c

DEVICE_STDPERIPH_SRC = \
            esp-idf/components/soc/esp32p4/gpio_periph.c \
            esp-idf/components/soc/esp32p4/i2c_periph.c \
            esp-idf/components/soc/esp32p4/uart_periph.c \
            esp-idf/components/soc/esp32p4/ledc_periph.c \
            esp-idf/components/soc/esp32p4/interrupts.c

MCU_EXCLUDES =
