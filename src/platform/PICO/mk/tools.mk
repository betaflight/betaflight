# Raspberry Pi Pico tools
PICOTOOL_REPO   := https://github.com/raspberrypi/picotool.git
PICOTOOL_DL_DIR := $(DL_DIR)/picotool
PICOTOOL_BUILD_DIR := $(PICOTOOL_DL_DIR)/build
PICOTOOL_DIR    := $(TOOLS_DIR)/picotool
PICO_SDK_PATH   ?= $(ROOT_DIR)/lib/main/pico-sdk
PICOTOOL        ?= $(PICOTOOL_DIR)/picotool

# Stamp file indicating pico-sdk has been hydrated
PICO_SDK_STAMP  := $(PICO_SDK_PATH)/.git

ifneq ($(filter picotool_install uf2,$(MAKECMDGOALS)),)
    ifeq ($(wildcard $(PICO_SDK_PATH)/CMakeLists.txt),)
        $(error "PICO_SDK_PATH ($(PICO_SDK_PATH)) does not point to a valid Pico SDK. Please 'make pico_sdk' to hydrate the Pico SDK.")
    endif
endif

ifneq ($(filter uf2,$(MAKECMDGOALS)),)
    ifeq (,$(wildcard $(PICOTOOL)))
        ifeq (,$(shell which picotool 2>/dev/null))
            $(error "picotool not in the PATH or configured. Run 'make picotool_install' to install in the tools folder.")
        else
            PICOTOOL := picotool
        endif
    endif
endif

## pico_sdk           : Hydrate Pico SDK submodule
.PHONY: pico_sdk
pico_sdk: $(PICO_SDK_STAMP)

# Auto-hydrate pico-sdk when needed as a build dependency
$(PICO_SDK_STAMP):
	@echo "Hydrating pico-sdk submodule"
	$(V1) git submodule update --init --recursive -- lib/main/pico-sdk || { echo "Failed to update pico-sdk"; exit 1; }
	@echo "pico-sdk ready"

.PHONY: picotool_install
picotool_install: | $(DL_DIR) $(TOOLS_DIR)
picotool_install: picotool_clean
	@echo "\n CLONE     $(PICOTOOL_REPO)"
	$(V1) git clone --depth 1 $(PICOTOOL_REPO) "$(PICOTOOL_DL_DIR)" || { echo "Failed to clone picotool repository"; exit 1; }
	@echo "\n BUILD      $(PICOTOOL_BUILD_DIR)"
	$(V1) [ -d "$(PICOTOOL_DIR)" ] || mkdir -p $(PICOTOOL_DIR)
	$(V1) [ -d "$(PICOTOOL_BUILD_DIR)" ] || mkdir -p $(PICOTOOL_BUILD_DIR)
	$(V1) cmake -S $(PICOTOOL_DL_DIR) -B $(PICOTOOL_BUILD_DIR) -D PICO_SDK_PATH=$(PICO_SDK_PATH) || { echo "CMake configuration failed"; exit 1; }
	$(V1) $(MAKE) -C $(PICOTOOL_BUILD_DIR) || { echo "picotool build failed"; exit 1; }
	$(V1) cp $(PICOTOOL_BUILD_DIR)/picotool $(PICOTOOL_DIR)/picotool || { echo "Failed to install picotool binary"; exit 1; }
	@echo "\n VERSION:"
	$(V1) $(PICOTOOL_DIR)/picotool version

.PHONY: picotool_clean
picotool_clean:
	@echo " CLEAN        $(PICOTOOL_DIR)"
	$(V1) [ ! -d "$(PICOTOOL_DIR)" ] || $(RM) -rf $(PICOTOOL_DIR)
	@echo " CLEAN        $(PICOTOOL_DL_DIR)"
	$(V1) [ ! -d "$(PICOTOOL_DL_DIR)" ] || $(RM) -rf $(PICOTOOL_DL_DIR)
