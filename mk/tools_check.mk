# Platform toolchain validation
#
# Included after the MCU family .mk so PLATFORM_SDK is known.
# Each platform SDK registers in its tools.mk:
#   PLATFORM_SDK_<name>_CC          - compiler binary to validate
#   PLATFORM_SDK_<name>_CC_VERSION  - required version (checked via -dumpversion, optional)
#   PLATFORM_SDK_<name>_CC_INSTALL  - make target that installs the toolchain
#
# Platforms with no _CC (e.g. none/SITL) skip validation.
# Utility goals that don't compile also skip validation.

_SKIP_TOOLCHECK_GOALS := %_sdk %_install test% clean% %-print checks help configs platform-%

ifneq ($(filter-out $(_SKIP_TOOLCHECK_GOALS), $(MAKECMDGOALS)),)
ifdef PLATFORM_SDK

_SDK_CC         := $(PLATFORM_SDK_$(PLATFORM_SDK)_CC)
_SDK_CC_VERSION := $(PLATFORM_SDK_$(PLATFORM_SDK)_CC_VERSION)
_SDK_CC_INSTALL := $(PLATFORM_SDK_$(PLATFORM_SDK)_CC_INSTALL)

ifneq ($(_SDK_CC),)
# Check compiler is available
_FOUND_VERSION := $(shell $(_SDK_CC) -dumpversion 2>/dev/null)
ifeq ($(_FOUND_VERSION),)
  $(error **ERROR** $(_SDK_CC) not found. Run 'make $(_SDK_CC_INSTALL)' to install the toolchain)
endif

# Check version matches if a specific version is required
ifneq ($(_SDK_CC_VERSION),)
ifneq ($(_FOUND_VERSION),$(_SDK_CC_VERSION))
  $(error **ERROR** $(_SDK_CC) version '$(_FOUND_VERSION)' found but '$(_SDK_CC_VERSION)' is required. Override with GCC_REQUIRED_VERSION in mk/local.mk or run 'make $(_SDK_CC_INSTALL)' to install the correct version)
endif
endif

# ARM toolchain found in PATH — set prefix so the rest of the build uses it
ifeq ($(ARM_SDK_PREFIX),)
ifneq ($(filter arm_sdk_install,$(PLATFORM_SDK_$(PLATFORM_SDK)_TOOLS)),)
  ARM_SDK_PREFIX := arm-none-eabi-
endif
endif

endif # _SDK_CC
endif # PLATFORM_SDK
endif # goals filter
