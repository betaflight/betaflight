# STM32N6 vendor HAL (STM32CubeN6 submodule)
STM32N6_LIB_PATH ?= $(ROOT_DIR)/lib/main/STM32N6

# Stamp file indicating STM32CubeN6 has been hydrated
STM32N6_SDK_STAMP := $(STM32N6_LIB_PATH)/.git

## stm32n6_sdk       : Hydrate STM32CubeN6 submodule
.PHONY: stm32n6_sdk
stm32n6_sdk: $(STM32N6_SDK_STAMP)

# Auto-hydrate STM32CubeN6 when needed as a build dependency
$(STM32N6_SDK_STAMP):
	@echo "Hydrating STM32CubeN6 submodule"
	$(V1) git submodule update --init --checkout -- lib/main/STM32N6 || { echo "Failed to update STM32CubeN6"; exit 1; }
	@echo "STM32CubeN6 ready"
