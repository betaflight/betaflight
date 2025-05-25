
CONFIGS_REPO_URL ?= https://github.com/betaflight/config
# handle only this directory as config submodule
CONFIGS_SUBMODULE_DIR = src/config
BASE_CONFIGS      = $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(CONFIG_DIR)/configs/*/config.h)))))

ifneq ($(filter-out %_install test% %_clean clean% %-print %.hex %.h hex checks help configs $(BASE_TARGETS) $(BASE_CONFIGS),$(MAKECMDGOALS)),)
ifeq ($(wildcard $(CONFIG_DIR)/configs/),)
$(error `$(CONFIG_DIR)` not found. Have you hydrated configuration using: 'make configs'?)
endif
endif

ifneq ($(CONFIG),)

ifneq ($(TARGET),)
$(error TARGET or CONFIG should be specified. Not both.)
endif

CONFIG_HEADER_FILE  = $(CONFIG_DIR)/configs/$(CONFIG)/config.h
CONFIG_SOURCE_FILE  = $(CONFIG_DIR)/configs/$(CONFIG)/config.c
INCLUDE_DIRS       += $(CONFIG_DIR)/configs/$(CONFIG)

ifneq ($(wildcard $(CONFIG_HEADER_FILE)),)

CONFIG_SRC :=
ifneq ($(wildcard $(CONFIG_SOURCE_FILE)),)
CONFIG_SRC += $(CONFIG_SOURCE_FILE)
TARGET_FLAGS += -DUSE_CONFIG_SOURCE
endif

CONFIG_REVISION := norevision
ifeq ($(shell git -C $(CONFIG_DIR) diff --shortstat),)
CONFIG_REVISION := $(shell git -C $(CONFIG_DIR) log -1 --format="%h")
CONFIG_REVISION_DEFINE := -D'__CONFIG_REVISION__="$(CONFIG_REVISION)"'
endif

HSE_VALUE_MHZ := $(shell sed -E -n "/^[[:space:]]*\#[[:space:]]*define[[:space:]]+SYSTEM_HSE_MHZ[[:space:]]+([0-9]+).*/s//\1/p" $(CONFIG_HEADER_FILE))
ifneq ($(HSE_VALUE_MHZ),)
HSE_VALUE     := $(shell echo $$(( $(HSE_VALUE_MHZ) * 1000000 )) )
endif

TARGET        := $(shell sed -E -n "/^[[:space:]]*\#[[:space:]]*define[[:space:]]+FC_TARGET_MCU[[:space:]]+([[:alnum:]_]+).*/s//\1/p" $(CONFIG_HEADER_FILE))
ifeq ($(TARGET),)
$(error No TARGET identified. Is the $(CONFIG_HEADER_FILE) valid for $(CONFIG)?)
endif

ifneq ($(filter $(TARGET),$(EXE_TARGETS)),)
	OUTPUT_TYPE := exe
else ifneq ($(filter $(subst _rev,,$@),$(UF2_TARGETS)),)
	OUTPUT_TYPE := uf2
else
	OUTPUT_TYPE := hex
endif


EXST_ADJUST_VMA := $(shell sed -E -n "/^[[:space:]]*\#[[:space:]]*define[[:space:]]+FC_VMA_ADDRESS[[:space:]]+((0[xX])?[[:xdigit:]]+).*/s//\1/p" $(CONFIG_HEADER_FILE))
ifneq ($(EXST_ADJUST_VMA),)
EXST = yes
endif

else #exists
$(error `$(CONFIG_HEADER_FILE)` not found. Have you hydrated configuration using: 'make configs'?)
endif #CONFIG_HEADER_FILE exists
endif #config

.PHONY: configs
configs:
ifeq ($(shell realpath $(CONFIG_DIR)),$(shell realpath $(CONFIGS_SUBMODULE_DIR)))
	@echo "Updating config submodule: $(CONFIGS_SUBMODULE_DIR)"
	$(V0) git submodule update --init -- $(CONFIGS_SUBMODULE_DIR) || { echo "Config submodule update failed. Please check your git configuration."; exit 1; }
	@echo "Submodule update succeeded."
else
ifeq ($(wildcard $(CONFIG_DIR)),)
	@echo "Hydrating clone for configs: $(CONFIG_DIR)"
	$(V0) git clone $(CONFIGS_REPO_URL) $(CONFIG_DIR)
else
	$(V0) git -C $(CONFIG_DIR) pull origin
endif
endif

$(BASE_CONFIGS):
	@echo "Building target config $@"
	$(V0) $(MAKE) $(MAKE_PARALLEL) $(OUTPUT_TYPE) CONFIG=$@
	@echo "Building target config $@ succeeded."

## <CONFIG>_rev    : build configured target and add revision to filename
$(addsuffix _rev,$(BASE_CONFIGS)):
	$(V0) $(MAKE) $(MAKE_PARALLEL) $(OUTPUT_TYPE) CONFIG=$(subst _rev,,$@) REV=yes
