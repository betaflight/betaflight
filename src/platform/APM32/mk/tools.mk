# APM32F4 vendor HAL (APM32F4xx_DAL_SDK submodule)
# Register SDK for CI caching and hydration
PLATFORM_SDKS += apm32f4
PLATFORM_SDK_apm32f4_SUBMODULE := lib/main/APM32F4
PLATFORM_SDK_apm32f4_HYDRATE := apm32f4_sdk
PLATFORM_SDK_apm32f4_TOOLS      := arm_sdk_install
PLATFORM_SDK_apm32f4_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_apm32f4_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_apm32f4_CC_INSTALL := arm_sdk_install

APM32F4_LIB_PATH ?= $(ROOT_DIR)/lib/main/APM32F4

# Stamp file: use the APM32F4 submodule root as proof it's populated
APM32F4_SDK_STAMP := $(APM32F4_LIB_PATH)/.git

## apm32f4_sdk      : Hydrate APM32F4 submodule
.PHONY: apm32f4_sdk
apm32f4_sdk: $(APM32F4_SDK_STAMP)

# Auto-hydrate APM32F4 when needed as a build dependency
$(APM32F4_SDK_STAMP):
	@echo "Hydrating APM32F4 submodule"
	$(V1) git submodule update --init --checkout -- lib/main/APM32F4 || { echo "Failed to update APM32F4"; exit 1; }
	@echo "APM32F4 ready"
