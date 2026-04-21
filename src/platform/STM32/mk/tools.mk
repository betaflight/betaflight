# STM32H5 vendor HAL (STM32CubeH5 submodule)
# Register SDK for CI caching and hydration
PLATFORM_SDKS += stm32h5
PLATFORM_SDK_stm32h5_SUBMODULE := lib/main/STM32H5
PLATFORM_SDK_stm32h5_HYDRATE := stm32h5_sdk
PLATFORM_SDK_stm32h5_TOOLS      := arm_sdk_install
PLATFORM_SDK_stm32h5_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_stm32h5_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_stm32h5_CC_INSTALL := arm_sdk_install

# STM32C5 vendor HAL (STM32CubeC5 submodule)
# Register SDK for CI caching and hydration
PLATFORM_SDKS += stm32c5
PLATFORM_SDK_stm32c5_SUBMODULE := lib/main/STM32C5
PLATFORM_SDK_stm32c5_HYDRATE := stm32c5_sdk
PLATFORM_SDK_stm32c5_TOOLS      := arm_sdk_install
PLATFORM_SDK_stm32c5_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_stm32c5_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_stm32c5_CC_INSTALL := arm_sdk_install
# C5 silicon supports FDCAN — advertise the same cross-platform lib deps
# as the plain arm SDK so the CI tools cache covers libcanard for C5 too.
PLATFORM_SDK_stm32c5_LIB_SUBMODULES := $(DRONECAN_LIB_DIR)

# STM32N6 vendor HAL (STM32CubeN6 submodule)
# Register SDK for CI caching and hydration
PLATFORM_SDKS += stm32n6
PLATFORM_SDK_stm32n6_SUBMODULE := lib/main/STM32N6
PLATFORM_SDK_stm32n6_HYDRATE := stm32n6_sdk
PLATFORM_SDK_stm32n6_TOOLS      := arm_sdk_install
PLATFORM_SDK_stm32n6_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_stm32n6_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_stm32n6_CC_INSTALL := arm_sdk_install

STM32H5_LIB_PATH ?= $(ROOT_DIR)/lib/main/STM32H5
STM32N6_LIB_PATH ?= $(ROOT_DIR)/lib/main/STM32N6
STM32C5_LIB_PATH ?= $(ROOT_DIR)/lib/main/STM32C5

# Stamp file: use the HAL driver submodule as proof the nested submodules are populated
STM32H5_SDK_STAMP := $(STM32H5_LIB_PATH)/Drivers/STM32H5xx_HAL_Driver/.git

## stm32h5_sdk       : Hydrate STM32CubeH5 submodule and required nested submodules
.PHONY: stm32h5_sdk
stm32h5_sdk: $(STM32H5_SDK_STAMP)

# Auto-hydrate STM32CubeH5 and its nested driver submodules when needed
$(STM32H5_SDK_STAMP):
	@echo "Hydrating STM32CubeH5 submodule"
	$(V1) git submodule update --init --checkout -- lib/main/STM32H5 || { echo "Failed to update STM32CubeH5"; exit 1; }
	@echo "Hydrating STM32CubeH5 nested driver submodules"
	$(V1) cd $(STM32H5_LIB_PATH) && git submodule update --init --checkout -- \
		Drivers/STM32H5xx_HAL_Driver \
		Drivers/CMSIS/Device/ST/STM32H5xx \
		|| { echo "Failed to update STM32CubeH5 nested submodules"; exit 1; }
	@echo "STM32CubeH5 ready"

# Stamp file: use the drivers submodule as proof the nested submodules are populated
# STM32CubeC5 uses a new "Cube 2.0" layout with stm32c5xx_drivers and stm32c5xx_dfp submodules
STM32C5_SDK_STAMP := $(STM32C5_LIB_PATH)/stm32c5xx_drivers/.git

## stm32c5_sdk       : Hydrate STM32CubeC5 submodule and required nested submodules
.PHONY: stm32c5_sdk
stm32c5_sdk: $(STM32C5_SDK_STAMP)

# Auto-hydrate STM32CubeC5 and its nested driver submodules when needed
$(STM32C5_SDK_STAMP):
	@echo "Hydrating STM32CubeC5 submodule"
	$(V1) git submodule update --init --checkout -- lib/main/STM32C5 || { echo "Failed to update STM32CubeC5"; exit 1; }
	@echo "Hydrating STM32CubeC5 nested submodules"
	$(V1) cd $(STM32C5_LIB_PATH) && git submodule update --init --checkout -- \
		stm32c5xx_drivers \
		stm32c5xx_dfp \
		arch/cmsis \
		|| { echo "Failed to update STM32CubeC5 nested submodules"; exit 1; }
	@echo "STM32CubeC5 ready"

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
