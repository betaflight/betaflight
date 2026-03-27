# Raspberry Pi Pico tools
# Register SDK for CI caching and hydration
PLATFORM_SDKS += pico_sdk
PLATFORM_SDK_pico_sdk_SUBMODULE := lib/main/pico-sdk
PLATFORM_SDK_pico_sdk_HYDRATE := pico_sdk
PLATFORM_SDK_pico_sdk_TOOLS      := arm_sdk_install picotool_install
PLATFORM_SDK_pico_sdk_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_pico_sdk_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_pico_sdk_CC_INSTALL := arm_sdk_install

PICOTOOL_REPO   := https://github.com/raspberrypi/picotool.git
PICOTOOL_DL_DIR := $(DL_DIR)/picotool
PICOTOOL_BUILD_DIR := $(PICOTOOL_DL_DIR)/build
PICOTOOL_DIR    := $(TOOLS_DIR)/picotool
PICO_SDK_PATH   ?= $(ROOT_DIR)/lib/main/pico-sdk
PICOTOOL        ?= $(PICOTOOL_DIR)/picotool

# Stamp file indicating pico-sdk has been hydrated
PICO_SDK_STAMP  := $(PICO_SDK_PATH)/.git

# Resolve picotool from local install or PATH
ifneq ($(filter uf2,$(MAKECMDGOALS)),)
    ifeq (,$(wildcard $(PICOTOOL)))
        ifneq (,$(shell which picotool 2>/dev/null))
            PICOTOOL := picotool
        else
            $(error picotool not found. Run 'make picotool_install' to install it in the tools folder)
        endif
    endif
endif

## pico_sdk           : Hydrate Pico SDK submodule
.PHONY: pico_sdk
pico_sdk: $(PICO_SDK_STAMP)

# Auto-hydrate pico-sdk when needed as a build dependency
$(PICO_SDK_STAMP):
	@echo "Hydrating pico-sdk submodule"
	$(V1) git submodule update --init --checkout --recursive -- lib/main/pico-sdk || { echo "Failed to update pico-sdk"; exit 1; }
	@echo "pico-sdk ready"

.PHONY: picotool_install
picotool_install: | $(PICO_SDK_STAMP) $(DL_DIR) $(TOOLS_DIR)
	@if [ -x "$(PICOTOOL_DIR)/picotool" ]; then \
		echo "picotool already installed:"; \
		$(PICOTOOL_DIR)/picotool version; \
	else \
		echo "\n CLONE     $(PICOTOOL_REPO)"; \
		$(RM) -rf "$(PICOTOOL_DL_DIR)"; \
		git clone --depth 1 $(PICOTOOL_REPO) "$(PICOTOOL_DL_DIR)" || { echo "Failed to clone picotool repository"; exit 1; }; \
		echo "\n BUILD      $(PICOTOOL_BUILD_DIR)"; \
		mkdir -p "$(PICOTOOL_DIR)" "$(PICOTOOL_BUILD_DIR)"; \
		cmake -S $(PICOTOOL_DL_DIR) -B $(PICOTOOL_BUILD_DIR) -D PICO_SDK_PATH=$(PICO_SDK_PATH) || { echo "CMake configuration failed"; exit 1; }; \
		$(MAKE) -C $(PICOTOOL_BUILD_DIR) || { echo "picotool build failed"; exit 1; }; \
		cp $(PICOTOOL_BUILD_DIR)/picotool $(PICOTOOL_DIR)/picotool || { echo "Failed to install picotool binary"; exit 1; }; \
		echo "\n VERSION:"; \
		$(PICOTOOL_DIR)/picotool version; \
	fi

.PHONY: picotool_clean
picotool_clean:
	@echo " CLEAN        $(PICOTOOL_DIR)"
	$(V1) [ ! -d "$(PICOTOOL_DIR)" ] || $(RM) -rf $(PICOTOOL_DIR)
	@echo " CLEAN        $(PICOTOOL_DL_DIR)"
	$(V1) [ ! -d "$(PICOTOOL_DL_DIR)" ] || $(RM) -rf $(PICOTOOL_DL_DIR)
