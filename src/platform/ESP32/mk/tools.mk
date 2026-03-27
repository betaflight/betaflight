# ESP32 Xtensa toolchain
# ESP-IDF v5.4 uses esp-idf toolchain managed via idf_tools.py

# Register SDK for CI caching and hydration
PLATFORM_SDKS += esp_idf
PLATFORM_SDK_esp_idf_SUBMODULE := lib/main/esp-idf
PLATFORM_SDK_esp_idf_HYDRATE   := esp_sdk
PLATFORM_SDK_esp_idf_TOOLS     := esp_tools_install
PLATFORM_SDK_esp_idf_CC_INSTALL := esp_tools_install

ESP_IDF_PATH    ?= $(ROOT_DIR)/lib/main/esp-idf
IDF_TOOLS_PATH  ?= $(TOOLS_DIR)/espressif
export IDF_TOOLS_PATH

# Resolve Xtensa compiler: prefer local install, fall back to PATH
ESP_TOOLS_BIN := $(firstword $(wildcard $(IDF_TOOLS_PATH)/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin))
ifneq ($(ESP_TOOLS_BIN),)
  PLATFORM_SDK_esp_idf_CC := $(ESP_TOOLS_BIN)/xtensa-esp32s3-elf-gcc
else
  PLATFORM_SDK_esp_idf_CC := xtensa-esp32s3-elf-gcc
endif

# Stamp file indicating esp-idf has been hydrated
ESP_IDF_STAMP := $(ESP_IDF_PATH)/.git

## esp_sdk            : Hydrate ESP-IDF submodule
.PHONY: esp_sdk
esp_sdk: $(ESP_IDF_STAMP)

# Auto-hydrate esp-idf when needed as a build dependency
$(ESP_IDF_STAMP):
	@echo "Hydrating esp-idf submodule"
	$(V1) git submodule update --init --checkout -- lib/main/esp-idf || { echo "Failed to update esp-idf"; exit 1; }
	@echo "esp-idf ready"

## esp_tools_install  : Install ESP32 toolchain via esp-idf
.PHONY: esp_tools_install
esp_tools_install: | $(ESP_IDF_STAMP)
	@echo "Installing ESP32 tools to $(IDF_TOOLS_PATH)"
	$(V1) cd $(ESP_IDF_PATH) && IDF_TOOLS_PATH=$(IDF_TOOLS_PATH) ./install.sh esp32s3 || { echo "Failed to install ESP32 tools"; exit 1; }
	@echo "ESP32 tools installed. Source export.sh before building:"
	@echo "  IDF_TOOLS_PATH=$(IDF_TOOLS_PATH) . $(ESP_IDF_PATH)/export.sh"

## esp_tools_clean    : Remove ESP32 toolchain
.PHONY: esp_tools_clean
esp_tools_clean:
	@echo " CLEAN        ESP32 tools"
	$(V1) [ ! -d "$(IDF_TOOLS_PATH)" ] || $(RM) -rf $(IDF_TOOLS_PATH)
