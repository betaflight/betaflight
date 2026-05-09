# GD32H7 firmware library (GD32H7xx_Firmware submodule)
# Register SDK for CI caching and hydration
PLATFORM_SDKS += gd32h7
PLATFORM_SDK_gd32h7_SUBMODULE := lib/main/GD32H7
PLATFORM_SDK_gd32h7_HYDRATE   := gd32h7_sdk
PLATFORM_SDK_gd32h7_TOOLS     := arm_sdk_install
PLATFORM_SDK_gd32h7_CC        := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_gd32h7_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_gd32h7_CC_INSTALL := arm_sdk_install

GD32H7_LIB_PATH ?= $(ROOT_DIR)/lib/main/GD32H7

# Stamp file: use the standard peripheral driver directory as proof the submodule is populated
GD32H7_SDK_STAMP := $(GD32H7_LIB_PATH)/Firmware/GD32H7xx_standard_peripheral

## gd32h7_sdk        : Hydrate GD32H7xx_Firmware submodule
.PHONY: gd32h7_sdk
gd32h7_sdk: $(GD32H7_SDK_STAMP)

# Auto-hydrate GD32H7 firmware library when needed as a build dependency
$(GD32H7_SDK_STAMP):
	@echo "Hydrating GD32H7xx_Firmware submodule"
	$(V1) git submodule update --init --checkout -- lib/main/GD32H7 || { echo "Failed to update GD32H7xx_Firmware"; exit 1; }
	@echo "GD32H7xx_Firmware ready"
