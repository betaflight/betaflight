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

# Variant for MCUs that merge into a single bootable $(TARGET_BIN) (e.g. ESP32S3):
# emit the app image to a temporary file ($@ with .bin -> _tmp.bin) that
# ESP_FLASH_IMAGE_CMD consumes and then deletes, leaving the merged image as the
# final .bin. Kept separate from ESP_ELF2IMAGE so app-image-only MCUs are unaffected.
ESP_APP_TMP_BIN     = $(@:.bin=_tmp.bin)
ESP_ELF2IMAGE_TMP   = $(ESP_PYTHON) $(ESP_ESPTOOL) --chip $(ESP_CHIP) elf2image \
                      --flash_mode $(ESP_FLASH_MODE) --flash_freq $(ESP_FLASH_FREQ) \
                      --flash_size $(ESP_FLASH_SIZE) --output $(ESP_APP_TMP_BIN) $<

# Second-stage bootloader. By default it is built from source via a minimal
# in-tree IDF project (src/platform/ESP32/bootloader) using `idf.py bootloader`;
# set ESP_BOOTLOADER_FROM_SOURCE=no to use the committed prebuilt blob instead
# (handy where CMake/Ninja or the full IDF aren't available). The from-source
# build is guarded on the output existing, so it runs once per clean tree and
# then incremental app builds skip it; `make clean` (or removing the build dir)
# forces a rebuild.
ESP_BOOTLOADER_FROM_SOURCE ?= yes
ESP_BOOTLOADER_PROJ  = $(TARGET_PLATFORM_DIR)/bootloader
ESP_PYTHON_ENV_DIR   = $(patsubst %/bin/python,%,$(ESP_PYTHON))

ifeq ($(ESP_BOOTLOADER_FROM_SOURCE),yes)
# Absolute paths: idf.py resolves -B against the CWD but -DSDKCONFIG against the
# project dir, so a relative path would split them apart. The build dir is keyed
# by flash size as well as chip so two boards on the same chip with different
# flash sizes (e.g. 8 MB ESP32S3 vs 4 MB CUSTS3AIO) don't share a bootloader
# built/cached with the wrong header.
ESP_BOOTLOADER_BUILD_DIR = $(abspath $(OBJECT_DIR))/esp-bootloader-$(ESP_CHIP)-$(ESP_FLASH_SIZE)
ESP_BOOTLOADER_BIN       = $(ESP_BOOTLOADER_BUILD_DIR)/bootloader/bootloader.bin
# Inject the flash size from MCU_FLASH_SIZE as an extra sdkconfig default layered
# on top of the project's sdkconfig.defaults, so the bootloader header matches
# the target's module. Generated into the (size-keyed) build dir, not the source
# tree. esptool still patches the size into the merged image at merge_bin time.
ESP_BUILD_BOOTLOADER = [ -f $(ESP_BOOTLOADER_BIN) ] || \
    ( mkdir -p $(ESP_BOOTLOADER_BUILD_DIR) && \
      echo 'CONFIG_ESPTOOLPY_FLASHSIZE_$(ESP_FLASH_SIZE)=y' > $(ESP_BOOTLOADER_BUILD_DIR)/sdkconfig.flashsize && \
      env IDF_PATH=$(ESP_IDF_PATH) IDF_TOOLS_PATH=$(IDF_TOOLS_PATH) \
        IDF_PYTHON_ENV_PATH=$(ESP_PYTHON_ENV_DIR) PATH="$(ESP_TOOLS_BIN):$$PATH" \
        $(ESP_PYTHON) $(ESP_IDF_PATH)/tools/idf.py -C $(abspath $(ESP_BOOTLOADER_PROJ)) \
            -B $(ESP_BOOTLOADER_BUILD_DIR) -DIDF_TARGET=$(ESP_CHIP) \
            -DSDKCONFIG_DEFAULTS="$(abspath $(ESP_BOOTLOADER_PROJ))/sdkconfig.defaults;$(ESP_BOOTLOADER_BUILD_DIR)/sdkconfig.flashsize" \
            -DSDKCONFIG=$(ESP_BOOTLOADER_BUILD_DIR)/sdkconfig bootloader )
else
ESP_BOOTLOADER_BIN   = $(TARGET_PLATFORM_DIR)/bin/bootloader_$(ESP_CHIP).bin
ESP_BUILD_BOOTLOADER = true
endif

# Single bootable image assembly: ensure the bootloader exists, generate the
# partition table from the in-tree CSV (IDF gen_esp32part.py), then merge
# bootloader (0x0) + table (0x8000) + app (the _tmp.bin from elf2image, at
# ESP_APP_OFFSET) into the final $(TARGET_BIN) - flashable in a single esptool
# write_flash at 0x0 - and delete the temporary app image. A per-MCU .mk opts in
# by chaining ESP_FLASH_IMAGE_CMD after ESP_ELF2IMAGE_TMP in BIN_FROM_ELF_CMD. All
# vars are expanded at recipe time, so values set in the per-MCU .mk are available.
ESP_GEN_ESP32PART   = $(ESP_PYTHON) $(ESP_IDF_PATH)/components/partition_table/gen_esp32part.py
ESP_PARTITIONS_CSV ?= $(TARGET_PLATFORM_DIR)/partitions.csv
ESP_PARTITION_BIN   = $(OBJECT_DIR)/partition-table_$(TARGET_NAME).bin
ESP_APP_OFFSET     ?= 0x10000
# Secondary bootloader flash offset, matching IDF CONFIG_BOOTLOADER_OFFSET_IN_FLASH:
# 0x1000 on ESP32/S2, 0x2000 on P4/C5, 0x0 elsewhere (S3, C2/C3/C6/H2).
ESP_BOOTLOADER_OFFSET = $(if $(filter esp32 esp32s2,$(ESP_CHIP)),0x1000,$(if $(filter esp32p4 esp32c5,$(ESP_CHIP)),0x2000,0x0))

ESP_FLASH_IMAGE_CMD = ( $(ESP_BUILD_BOOTLOADER) ) && \
                      $(ESP_GEN_ESP32PART) $(ESP_PARTITIONS_CSV) $(ESP_PARTITION_BIN) && \
                      $(ESP_PYTHON) $(ESP_ESPTOOL) --chip $(ESP_CHIP) merge_bin --output $@ \
                        --flash_mode $(ESP_FLASH_MODE) --flash_freq $(ESP_FLASH_FREQ) --flash_size $(ESP_FLASH_SIZE) \
                        $(ESP_BOOTLOADER_OFFSET) $(ESP_BOOTLOADER_BIN) 0x8000 $(ESP_PARTITION_BIN) $(ESP_APP_OFFSET) $(ESP_APP_TMP_BIN) && \
                      rm -f $(ESP_APP_TMP_BIN)

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
