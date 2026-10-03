# HPMicro vendor SDK (hpm_sdk submodule) + RISC-V toolchain
# Register SDK for CI caching and hydration
PLATFORM_SDKS += hpm_sdk
PLATFORM_SDK_hpm_sdk_SUBMODULE := lib/modules/hpm_sdk
PLATFORM_SDK_hpm_sdk_HYDRATE   := hpm_sdk
PLATFORM_SDK_hpm_sdk_TOOLS     := riscv_sdk_install
PLATFORM_SDK_hpm_sdk_CC_INSTALL := riscv_sdk_install

# HPM_SDK_BASE may come from the environment or the command line; anything else
# (including mk/local.mk defaulting it to the submodule) is internal.
HPM_EXTERNAL_SDK := $(filter environment environment override command line,$(origin HPM_SDK_BASE))
ifeq ($(HPM_EXTERNAL_SDK),)
# Default to submodule path
HPM_SDK_PATH    ?= $(ROOT_DIR)/lib/modules/hpm_sdk
else
# External SDK path specified - skip submodule hydration
HPM_SDK_PATH := $(HPM_SDK_BASE)
endif

# RISC-V toolchain install (mirrors arm_sdk_install in mk/tools.mk)
# Source: https://github.com/hpmicro/riscv-gnu-toolchain (release 2023.10.18, gcc 13.2.0)
ifeq ($(OSFAMILY)-$(ARCHFAMILY), linux-x86_64)
  RISCV_SDK_URL := https://github.com/hpmicro/riscv-gnu-toolchain/releases/download/2023.10.18/rv32imac_zicsr_zifencei_multilib_b_ext-linux.tar.gz
  RISCV_DL_SHA256 = 550e867c86d14e0a1fddfde4d9316f7485a396474311c5d8ab61ebff5492ce17
else ifeq ($(OSFAMILY), macosx)
  # Intel build, runs under Rosetta on arm64
  RISCV_SDK_URL := https://github.com/hpmicro/riscv-gnu-toolchain/releases/download/2023.10.18/riscv32-unknown-elf-newlib-multilib-2023.10.18-macos-intel.tar.gz
  RISCV_DL_SHA256 = cceec9c0a8fd2fc1a730b9d1bb61089e19bcb039fda619d87b983530bbb0bf18
else ifeq ($(OSFAMILY), windows)
  RISCV_SDK_URL := https://github.com/hpmicro/riscv-gnu-toolchain/releases/download/2023.10.18/rv32imac_zicsr_zifencei_multilib_b_ext-win.zip
  RISCV_DL_SHA256 = 794ae8a337db01372fffb5b4abee082ad73c6ef9b665430145ba34b167b7f3f4
else
  $(error No RISC-V toolchain URL defined for $(OSFAMILY)-$(ARCHFAMILY))
endif

RISCV_SDK_FILE := $(notdir $(RISCV_SDK_URL))
RISCV_SDK_DIR := $(TOOLS_DIR)/$(patsubst %.zip,%,$(patsubst %.tar.gz,%,$(RISCV_SDK_FILE)))
RISCV_INSTALL_MARKER := $(RISCV_SDK_DIR)/.installed

# RISC-V toolchain resolution, shared by the family .mk (ARM_SDK_PREFIX
# override) and the CI compiler metadata below.
# Priority: toolchain installed in tools/ > RISCV_SDK_PREFIX env var > bare name (must be in PATH)
ifeq ($(shell [ -d "$(RISCV_SDK_DIR)" ] && echo "exists"), exists)
HPM_RISCV_PREFIX := $(RISCV_SDK_DIR)/bin/riscv32-unknown-elf-
else ifneq ($(RISCV_SDK_PREFIX),)
HPM_RISCV_PREFIX := $(RISCV_SDK_PREFIX)
else
HPM_RISCV_PREFIX := riscv32-unknown-elf-
endif

PLATFORM_SDK_hpm_sdk_CC         := $(HPM_RISCV_PREFIX)gcc

.PHONY: riscv_sdk_install riscv_sdk_clean
riscv_sdk_install: | $(TOOLS_DIR)
riscv_sdk_install: riscv_sdk_download $(RISCV_INSTALL_MARKER)

$(RISCV_INSTALL_MARKER): $(DL_DIR)/$(RISCV_SDK_FILE)
        # Verify the pinned SHA-256 before extraction. macOS provides shasum;
        # GNU environments (Linux and MSYS2) provide sha256sum.
	@checksum=$$( (command -v sha256sum >/dev/null 2>&1 && sha256sum "$<" || shasum -a 256 "$<") | awk '{print $$1}'); \
	if [ "$$checksum" != "$(RISCV_DL_SHA256)" ]; then \
		echo "$@ SHA-256 mismatch! Expected $(RISCV_DL_SHA256), got $$checksum."; \
		exit 1; \
	fi
ifeq ($(OSFAMILY), windows)
	$(V1) unzip -q -d $(TOOLS_DIR) "$<"
else
	$(V1) tar -C $(TOOLS_DIR) -xf "$<"
endif
	$(V1) touch $(RISCV_INSTALL_MARKER)

.PHONY: riscv_sdk_download
riscv_sdk_download: | $(DL_DIR)
riscv_sdk_download: $(DL_DIR)/$(RISCV_SDK_FILE)
$(DL_DIR)/$(RISCV_SDK_FILE):
        # download the source only if it's newer than what we already have
	$(V1) curl --fail --location -o "$@" $(if $(wildcard $@), -z "$@",) "$(RISCV_SDK_URL)"

## riscv_sdk_clean  : Uninstall RISC-V SDK
riscv_sdk_clean:
	$(V1) [ ! -d "$(RISCV_SDK_DIR)" ] || $(RM) -r $(RISCV_SDK_DIR)
	$(V1) [ ! -f "$(DL_DIR)/$(RISCV_SDK_FILE)" ] || $(RM) -f "$(DL_DIR)/$(RISCV_SDK_FILE)"

# Stamp file: use the hpm_sdk submodule root as proof it's populated.  Never
# write into an externally supplied SDK tree.
ifneq ($(HPM_EXTERNAL_SDK),)
HPM_SDK_STAMP := $(OBJECT_DIR)/hpm_sdk_external.stamp
else
HPM_SDK_STAMP := $(HPM_SDK_PATH)/.git
endif

## hpm_sdk         : Hydrate hpm_sdk submodule
.PHONY: hpm_sdk
hpm_sdk: $(HPM_SDK_STAMP)

# Auto-hydrate hpm_sdk when needed as a build dependency
$(HPM_SDK_STAMP):
ifneq ($(HPM_EXTERNAL_SDK),)
	@echo "Using external HPM SDK: $(HPM_SDK_PATH)"
	@if [ ! -d "$(HPM_SDK_PATH)" ]; then \
		echo "Error: External HPM SDK path does not exist: $(HPM_SDK_PATH)"; \
		exit 1; \
	fi
	@mkdir -p $(dir $@)
	@touch $(HPM_SDK_STAMP)
else
	@echo "Hydrating hpm_sdk submodule"
	$(V1) git submodule update --init --checkout -- lib/modules/hpm_sdk || { echo "Failed to update hpm_sdk"; exit 1; }
	@echo "hpm_sdk ready"
endif
