# UM324xF vendor SDK submodule and toolchain registration.
PLATFORM_SDKS += um324xf
PLATFORM_SDK_um324xf_SUBMODULE := lib/modules/UM324xF
PLATFORM_SDK_um324xf_HYDRATE := um324xf_sdk
PLATFORM_SDK_um324xf_TOOLS := arm_sdk_install
PLATFORM_SDK_um324xf_CC := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_um324xf_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_um324xf_CC_INSTALL := arm_sdk_install

UM324XF_LIB_PATH ?= $(ROOT_DIR)/lib/modules/UM324xF
UM324XF_SDK_STAMP := $(UM324XF_LIB_PATH)/.git

## um324xf_sdk      : Hydrate the UM324xF SDK submodule
.PHONY: um324xf_sdk
um324xf_sdk: $(UM324XF_SDK_STAMP)

$(UM324XF_SDK_STAMP):
	@echo "Hydrating UM324xF SDK submodule"
	$(V1) git submodule update --init --checkout -- lib/modules/UM324xF || { echo "Failed to update UM324xF SDK"; exit 1; }
	@echo "UM324xF SDK ready"
