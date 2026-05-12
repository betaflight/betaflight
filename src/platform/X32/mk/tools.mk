# X32M7 vendor Lib (X32M7_Std_SDK submodule)
# Register SDK for CI caching and hydration
PLATFORM_SDKS += x32m7
PLATFORM_SDK_x32m7_SUBMODULE := lib/main/X32M7
PLATFORM_SDK_x32m7_HYDRATE := x32m7_sdk
PLATFORM_SDK_x32m7_TOOLS      := arm_sdk_install
PLATFORM_SDK_x32m7_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_x32m7_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_x32m7_CC_INSTALL := arm_sdk_install

X32M7_LIB_PATH ?= $(ROOT_DIR)/lib/main/X32M7

# Stamp file: use the X32M7 submodule root as proof it's populated
X32M7_SDK_STAMP := $(X32M7_LIB_PATH)/.git

## x32m7_sdk      : Hydrate X32M7 submodule
.PHONY: x32m7_sdk
x32m7_sdk: $(X32M7_SDK_STAMP)

# Auto-hydrate X32M7 when needed as a build dependency
$(X32M7_SDK_STAMP):
	@echo "Hydrating X32M7 submodule"
	$(V1) git submodule update --init --checkout -- lib/main/X32M7 || { echo "Failed to update X32M7"; exit 1; }
	@echo "X32M7 ready"
