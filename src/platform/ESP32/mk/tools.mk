# ESP32 Xtensa toolchain
# ESP-IDF v5.4 uses esp-idf toolchain managed via idf_tools.py
# For standalone use, the Xtensa GCC toolchain can be installed separately.
ESP_IDF_PATH ?= $(ROOT_DIR)/lib/main/esp-idf

## esp_sdk            : Hydrate ESP-IDF submodule
.PHONY: esp_sdk
esp_sdk:
	@echo "Updating esp-idf"
	$(V1) git submodule update --init --recursive -- lib/main/esp-idf || { echo "Failed to update esp-idf"; exit 1; }
	@echo "esp-idf updated"

## esp_tools_install  : Install ESP32 toolchain via esp-idf
.PHONY: esp_tools_install
esp_tools_install: esp_sdk
	@echo "Installing ESP32 tools via idf_tools.py"
	$(V1) cd $(ESP_IDF_PATH) && ./install.sh esp32s3 || { echo "Failed to install ESP32 tools"; exit 1; }
	@echo "ESP32 tools installed. Source export.sh before building:"
	@echo "  . $(ESP_IDF_PATH)/export.sh"

## esp_tools_clean    : Remove ESP32 toolchain
.PHONY: esp_tools_clean
esp_tools_clean:
	@echo " CLEAN        ESP32 tools"
	$(V1) [ ! -d "$(HOME)/.espressif" ] || $(RM) -rf $(HOME)/.espressif
