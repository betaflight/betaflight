# CH32 RISC-V WCH toolchain and CH32H41x vendor SDK submodule
# Register SDK for CI caching and hydration
PLATFORM_SDKS += ch32_sdk
PLATFORM_SDK_ch32_sdk_SUBMODULE := lib/modules/CH32H41x
PLATFORM_SDK_ch32_sdk_HYDRATE   := ch32_sdk_submodule
PLATFORM_SDK_ch32_sdk_TOOLS     := ch32_sdk_install
PLATFORM_SDK_ch32_sdk_CC_INSTALL := ch32_sdk_install

CH32_LIB_PATH ?= $(ROOT_DIR)/lib/modules/CH32H41x

# Stamp file: use the CH32H41x submodule root as proof it's populated
CH32_SDK_STAMP := $(CH32_LIB_PATH)/.git

## ch32_sdk_submodule  : Hydrate CH32H41x submodule
.PHONY: ch32_sdk_submodule
ch32_sdk_submodule: $(CH32_SDK_STAMP)

# Auto-hydrate CH32H41x when needed as a build dependency
$(CH32_SDK_STAMP):
	@echo "Hydrating CH32H41x submodule"
	$(V1) git submodule update --init --checkout -- lib/modules/CH32H41x || { echo "Failed to update CH32H41x"; exit 1; }
	@echo "CH32H41x ready"

# Toolchain version and URL
CH32_SDK_VERSION := 12.2.0
CH32_SDK_URL     := https://github.com/TianpeiLee/riscv-gun-toolchain-12.2.0-x86_64-linux-riscv-wch-elf/archive/refs/tags/12.2.0.tar.gz
CH32_SDK_FILE    := riscv-wch-elf-gcc-12.2.0-linux-x86_64.tar.gz
CH32_SDK_SHA256  := 8b9c27b9631c24e91fd51ca496ccd08f66832191d400848112ee4b9e0f8c4c39
CH32_SDK_DIR     := $(TOOLS_DIR)/riscv-wch-elf-$(CH32_SDK_VERSION)
CH32_SDK_BIN     := $(CH32_SDK_DIR)/bin

# Resolve WCH RISC-V compiler: prefer local install, fall back to PATH
ifneq ($(wildcard $(CH32_SDK_BIN)/riscv-wch-elf-gcc),)
  CH32_SDK_PREFIX := $(CH32_SDK_BIN)/riscv-wch-elf-
else
  CH32_SDK_PREFIX := riscv-wch-elf-
endif

PLATFORM_SDK_ch32_sdk_CC         := $(CH32_SDK_PREFIX)gcc
PLATFORM_SDK_ch32_sdk_CC_VERSION := $(CH32_SDK_VERSION)

## ch32_sdk_install    : Install WCH RISC-V toolchain
.PHONY: ch32_sdk_install
ch32_sdk_install: | $(DL_DIR) $(TOOLS_DIR)
ch32_sdk_install: $(CH32_SDK_DIR)/.installed

$(CH32_SDK_DIR)/.installed: $(DL_DIR)/$(CH32_SDK_FILE)
	@echo " EXTRACT      $(CH32_SDK_FILE)"
	$(V1) if command -v sha256sum >/dev/null 2>&1; then \
	    echo "$(CH32_SDK_SHA256)  $<" | sha256sum -c -; \
	else \
	    echo "$(CH32_SDK_SHA256)  $<" | shasum -a 256 -c -; \
	fi
	$(V1) mkdir -p $(CH32_SDK_DIR)
	$(V1) tar -C $(CH32_SDK_DIR) --strip-components=1 -xzf "$<"
	$(V1) touch $@
	@echo "CH32 WCH toolchain installed: $(CH32_SDK_DIR)"

$(DL_DIR)/$(CH32_SDK_FILE):
ifneq ($(OSFAMILY)-$(ARCHFAMILY),linux-x86_64)
	$(error Automated CH32 WCH toolchain install currently supports linux-x86_64 only (detected $(OSFAMILY)-$(ARCHFAMILY)); install riscv-wch-elf-gcc in PATH)
endif
	@echo " DOWNLOAD     $(CH32_SDK_URL)"
	$(V1) curl --fail -L -o "$@" $(if $(wildcard $@),-z "$@",) "$(CH32_SDK_URL)"

## ch32_sdk_clean      : Remove WCH RISC-V toolchain
.PHONY: ch32_sdk_clean
ch32_sdk_clean:
	@echo " CLEAN        $(CH32_SDK_DIR)"
	$(V1) [ ! -d "$(CH32_SDK_DIR)" ] || $(RM) -rf $(CH32_SDK_DIR)
	$(V1) $(RM) -f $(DL_DIR)/$(CH32_SDK_FILE)
