# STM32N6 vendor HAL (STM32CubeN6 submodule)
# Register SDK for CI caching and hydration
PLATFORM_SDKS += stm32n6
PLATFORM_SDK_stm32n6_SUBMODULE := lib/main/STM32N6
PLATFORM_SDK_stm32n6_HYDRATE := stm32n6_sdk
PLATFORM_SDK_stm32n6_TOOLS      := arm_sdk_install
PLATFORM_SDK_stm32n6_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_stm32n6_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_stm32n6_CC_INSTALL := arm_sdk_install

STM32N6_LIB_PATH ?= $(ROOT_DIR)/lib/main/STM32N6

# Stamp file: use the HAL driver submodule as proof the nested submodules are populated
STM32N6_SDK_STAMP := $(STM32N6_LIB_PATH)/Drivers/STM32N6xx_HAL_Driver/.git

## stm32n6_sdk       : Hydrate STM32CubeN6 submodule and required nested submodules
.PHONY: stm32n6_sdk
stm32n6_sdk: $(STM32N6_SDK_STAMP)

# Auto-hydrate STM32CubeN6 and its nested driver submodules when needed
$(STM32N6_SDK_STAMP):
	@echo "Hydrating STM32CubeN6 submodule"
	$(V1) git submodule update --init --checkout -- lib/main/STM32N6 || { echo "Failed to update STM32CubeN6"; exit 1; }
	@echo "Hydrating STM32CubeN6 nested driver submodules"
	$(V1) cd $(STM32N6_LIB_PATH) && git submodule update --init --checkout -- \
		Drivers/STM32N6xx_HAL_Driver \
		Drivers/CMSIS/Device/ST/STM32N6xx \
		|| { echo "Failed to update STM32CubeN6 nested submodules"; exit 1; }
	@echo "STM32CubeN6 ready"
