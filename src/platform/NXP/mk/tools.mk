# NXP i.MXRT tools: the arm-none-eabi toolchain plus the mcux-devices-rt and
# TinyUSB submodules, or mcux-devices-imx, OpenAMP and libmetal for the i.MX95.

# Register the SDK so CI can cache it and the build can download it
PLATFORM_SDKS += mcux_rt
PLATFORM_SDK_mcux_rt_SUBMODULE  := lib/modules/mcux-devices-rt lib/modules/tinyusb
PLATFORM_SDK_mcux_rt_HYDRATE    := mcux_rt_sdk
PLATFORM_SDK_mcux_rt_TOOLS      := arm_sdk_install
PLATFORM_SDK_mcux_rt_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_mcux_rt_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_mcux_rt_CC_INSTALL := arm_sdk_install

MCUX_RT_PATH  ?= $(ROOT_DIR)/lib/modules/mcux-devices-rt
TINYUSB_PATH  ?= $(ROOT_DIR)/lib/modules/tinyusb

# Stamp files that show each submodule has been checked out
MCUX_RT_STAMP := $(MCUX_RT_PATH)/.git
TINYUSB_STAMP := $(TINYUSB_PATH)/.git

## mcux_rt_sdk        : Download the NXP i.MXRT device header and TinyUSB submodules
.PHONY: mcux_rt_sdk
mcux_rt_sdk: $(MCUX_RT_STAMP) $(TINYUSB_STAMP)

# Download mcux-devices-rt when a build needs it
$(MCUX_RT_STAMP):
	@echo "Downloading mcux-devices-rt submodule"
	$(V1) git submodule update --init --checkout -- lib/modules/mcux-devices-rt || \
		{ echo "Failed to update mcux-devices-rt"; exit 1; }
	@echo "mcux-devices-rt ready"

# Download tinyusb when a build needs it; its own submodules are not needed.
$(TINYUSB_STAMP):
	@echo "Downloading tinyusb submodule"
	$(V1) git submodule update --init --checkout -- lib/modules/tinyusb || \
		{ echo "Failed to update tinyusb"; exit 1; }
	@echo "tinyusb ready"

# The i.MX95 M7 needs the i.MX device headers and OpenAMP with libmetal for
# the rpmsg link to Linux.
PLATFORM_SDKS += mcux_imx
PLATFORM_SDK_mcux_imx_SUBMODULE  := lib/modules/mcux-devices-imx lib/modules/open-amp lib/modules/libmetal
PLATFORM_SDK_mcux_imx_HYDRATE    := mcux_imx_sdk
PLATFORM_SDK_mcux_imx_TOOLS      := arm_sdk_install
PLATFORM_SDK_mcux_imx_CC         := $(PLATFORM_SDK_arm_CC)
PLATFORM_SDK_mcux_imx_CC_VERSION := $(PLATFORM_SDK_arm_CC_VERSION)
PLATFORM_SDK_mcux_imx_CC_INSTALL := arm_sdk_install

MCUX_IMX_PATH  ?= $(ROOT_DIR)/lib/modules/mcux-devices-imx
OPENAMP_PATH   ?= $(ROOT_DIR)/lib/modules/open-amp
LIBMETAL_PATH  ?= $(ROOT_DIR)/lib/modules/libmetal
MCUX_IMX_STAMP := $(MCUX_IMX_PATH)/.git
OPENAMP_STAMP  := $(OPENAMP_PATH)/.git
LIBMETAL_STAMP := $(LIBMETAL_PATH)/.git

## mcux_imx_sdk       : Download the NXP i.MX device header, OpenAMP and libmetal submodules
.PHONY: mcux_imx_sdk
mcux_imx_sdk: $(MCUX_IMX_STAMP) $(OPENAMP_STAMP) $(LIBMETAL_STAMP)

$(MCUX_IMX_STAMP) $(OPENAMP_STAMP) $(LIBMETAL_STAMP):
	@echo "Downloading $(patsubst $(ROOT_DIR)/%/.git,%,$@) submodule"
	$(V1) git submodule update --init --checkout -- $(patsubst $(ROOT_DIR)/%/.git,%,$@) || \
		{ echo "Failed to update $(patsubst $(ROOT_DIR)/%/.git,%,$@)"; exit 1; }
