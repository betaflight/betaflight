# CH32 RISC-V WCH toolchain
# Register SDK for CI caching and hydration
PLATFORM_SDKS += ch32_sdk
PLATFORM_SDK_ch32_sdk_SUBMODULE :=
PLATFORM_SDK_ch32_sdk_HYDRATE   :=
PLATFORM_SDK_ch32_sdk_TOOLS     := ch32_sdk_install
PLATFORM_SDK_ch32_sdk_CC_INSTALL := ch32_sdk_install

# Toolchain version and URL
CH32_SDK_VERSION := 12.2.0
CH32_SDK_URL     := https://github.com/TianpeiLee/riscv-gun-toolchain-12.2.0-x86_64-linux-riscv-wch-elf/archive/refs/tags/12.2.0.tar.gz
CH32_SDK_FILE    := $(notdir $(CH32_SDK_URL))
CH32_SDK_DIR     := $(TOOLS_DIR)/riscv-wch-elf-$(CH32_SDK_VERSION)
CH32_SDK_BIN     := $(CH32_SDK_DIR)/bin

PLATFORM_SDK_ch32_sdk_CC         := $(CH32_SDK_BIN)/riscv-wch-elf-gcc
PLATFORM_SDK_ch32_sdk_CC_VERSION := $(CH32_SDK_VERSION)

# Resolve WCH RISC-V compiler: prefer local install, fall back to PATH
ifneq ($(wildcard $(CH32_SDK_BIN)/riscv-wch-elf-gcc),)
  CH32_SDK_PREFIX := $(CH32_SDK_BIN)/riscv-wch-elf-
else
  CH32_SDK_PREFIX := riscv-wch-elf-
endif

## ch32_sdk_install    : Install WCH RISC-V toolchain
.PHONY: ch32_sdk_install
ch32_sdk_install: | $(DL_DIR) $(TOOLS_DIR)
ch32_sdk_install: $(CH32_SDK_DIR)/.installed

$(CH32_SDK_DIR)/.installed: $(DL_DIR)/$(CH32_SDK_FILE)
	@echo " EXTRACT      $(CH32_SDK_FILE)"
	$(V1) mkdir -p $(CH32_SDK_DIR)
	$(V1) tar -C $(CH32_SDK_DIR) --strip-components=1 -xzf "$<"
	$(V1) touch $@
	@echo "CH32 WCH toolchain installed: $(CH32_SDK_DIR)"

$(DL_DIR)/$(CH32_SDK_FILE):
	@echo " DOWNLOAD     $(CH32_SDK_URL)"
	$(V1) curl -L -k -o "$@" $(if $(wildcard $@),-z "$@",) "$(CH32_SDK_URL)"

## ch32_sdk_clean      : Remove WCH RISC-V toolchain
.PHONY: ch32_sdk_clean
ch32_sdk_clean:
	@echo " CLEAN        $(CH32_SDK_DIR)"
	$(V1) [ ! -d "$(CH32_SDK_DIR)" ] || $(RM) -rf $(CH32_SDK_DIR)
	$(V1) $(RM) -f $(DL_DIR)/$(CH32_SDK_FILE)
