# ESP32 Xtensa toolchain
# ESP-IDF v5.4 uses esp-idf toolchain managed via idf_tools.py

# Register SDK for CI caching and hydration
PLATFORM_SDKS += esp_idf
PLATFORM_SDK_esp_idf_SUBMODULE := lib/modules/esp-idf
PLATFORM_SDK_esp_idf_HYDRATE   := esp_sdk
PLATFORM_SDK_esp_idf_TOOLS     := esp_tools_install
PLATFORM_SDK_esp_idf_CC_INSTALL := esp_tools_install

ESP_IDF_PATH    ?= $(ROOT_DIR)/lib/modules/esp-idf
IDF_TOOLS_PATH  ?= $(TOOLS_DIR)/espressif
export IDF_TOOLS_PATH

# Resolve toolchain bin directories: prefer local install, fall back to PATH.
# The unified xtensa-esp-elf toolchain covers ESP32 / ESP32S3 (Xtensa cores);
# riscv32-esp-elf covers ESP32C5 / ESP32P4 (RISC-V cores). Each platform .mk
# selects the matching ESP_TOOLS_BIN.
ESP_TOOLS_XTENSA_BIN := $(firstword $(wildcard $(IDF_TOOLS_PATH)/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin))
ESP_TOOLS_RISCV_BIN  := $(firstword $(wildcard $(IDF_TOOLS_PATH)/tools/riscv32-esp-elf/*/riscv32-esp-elf/bin))

# Default ESP_TOOLS_BIN preserves the Xtensa path for the existing
# Xtensa-targeting platform .mk files; the RISC-V .mk files override it.
ESP_TOOLS_BIN := $(ESP_TOOLS_XTENSA_BIN)

ifneq ($(ESP_TOOLS_XTENSA_BIN),)
  PLATFORM_SDK_esp_idf_CC := $(ESP_TOOLS_XTENSA_BIN)/xtensa-esp32-elf-gcc
else
  PLATFORM_SDK_esp_idf_CC := xtensa-esp32-elf-gcc
endif

# esptool, from the IDF python environment, used to wrap the linked ELF into a
# bootable ESP-IDF application image (flashed at the app partition offset).
ESP_PYTHON      := $(firstword $(wildcard $(IDF_TOOLS_PATH)/python_env/*/bin/python))
ESP_ESPTOOL     := $(firstword $(wildcard $(IDF_TOOLS_PATH)/python_env/*/bin/esptool.py))

# Image parameters. ESP_CHIP and ESP_FLASH_SIZE are derived (deferred) from the
# per-target values, which are set later than this file is included.
ESP_FLASH_MODE  ?= dio
ESP_FLASH_FREQ  ?= 80m
ESP_CHIP         = $(shell echo $(TARGET_MCU_FAMILY) | tr '[:upper:]' '[:lower:]')
ESP_FLASH_SIZE   = $(shell expr $(MCU_FLASH_SIZE) / 1024)MB

# elf2image command used by the top-level $(TARGET_BIN) rule. Each ESP32 MCU .mk
# opts in by setting BIN_FROM_ELF_CMD = $(ESP_ELF2IMAGE); doing it there (not
# here) keeps it scoped to ESP32 builds, since this file is included for all.
ESP_ELF2IMAGE    = $(ESP_PYTHON) $(ESP_ESPTOOL) --chip $(ESP_CHIP) elf2image \
                   --flash_mode $(ESP_FLASH_MODE) --flash_freq $(ESP_FLASH_FREQ) \
                   --flash_size $(ESP_FLASH_SIZE) --output $@ $<

# Stamp file indicating esp-idf has been hydrated
ESP_IDF_STAMP := $(ESP_IDF_PATH)/.git

## esp_sdk            : Hydrate ESP-IDF submodule
.PHONY: esp_sdk
esp_sdk: $(ESP_IDF_STAMP)

# Auto-hydrate esp-idf when needed as a build dependency
$(ESP_IDF_STAMP):
	@echo "Hydrating esp-idf submodule"
	$(V1) git submodule update --init --checkout -- lib/modules/esp-idf || { echo "Failed to update esp-idf"; exit 1; }
	@echo "esp-idf ready"

## esp_tools_install  : Install ESP32 toolchain via esp-idf
.PHONY: esp_tools_install
esp_tools_install: | $(ESP_IDF_STAMP)
	@ldconfig -p 2>/dev/null | grep -q libusb-1.0 || \
		echo "WARNING: libusb-1.0 not found. Install it (e.g. 'sudo apt-get install libusb-1.0-0') or esp32 tools may fail."
	@python3 -c 'import venv' 2>/dev/null || \
		echo "WARNING: python3-venv not found. Install it (e.g. 'sudo apt-get install python3-venv') or esp32 tools may fail."
	@echo "Installing ESP32 tools to $(IDF_TOOLS_PATH)"
	$(V1) cd $(ESP_IDF_PATH) && IDF_TOOLS_PATH=$(IDF_TOOLS_PATH) ./install.sh esp32,esp32s3 || { echo "Failed to install ESP32 tools"; exit 1; }
	@echo "ESP32 tools installed. Source export.sh before building:"
	@echo "  IDF_TOOLS_PATH=$(IDF_TOOLS_PATH) . $(ESP_IDF_PATH)/export.sh"

## esp_tools_clean    : Remove ESP32 toolchain
.PHONY: esp_tools_clean
esp_tools_clean:
	@echo " CLEAN        ESP32 tools"
	$(V1) [ ! -d "$(IDF_TOOLS_PATH)" ] || $(RM) -rf $(IDF_TOOLS_PATH)
