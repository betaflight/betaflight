
API_URL           := https://build.betaflight.com/api/config
BASE_CONFIGS_URL  := $(API_URL)/$(FC_VER)/targets
BASE_CONFIGS_FILE := $(CONFIG_DIR)/$(FC_VER)/targets.lst

BASE_CONFIGS_DATE  = $(if $(wildcard $(BASE_CONFIGS_FILE)),-z "$(BASE_CONFIGS_FILE)",)
BASE_CONFIGS       = $(if $(wildcard $(BASE_CONFIGS_FILE)),$(sort $(shell cat $(BASE_CONFIGS_FILE))),)

CONFIG_FILES      = $(addsuffix /config.h, $(addprefix $(CONFIG_DIR)/$(FC_VER)/,$(BASE_CONFIGS)))

CONFIGS_CLEAN 	  = $(addsuffix _clean,$(BASE_CONFIGS))


ifneq ($(CONFIG),)

ifneq ($(TARGET),)
$(error TARGET or CONFIG should be specified. Not both.)
endif

CONFIG_FILE      = $(CONFIG_DIR)/$(FC_VER)/$(CONFIG)/config.h
INCLUDE_DIRS    += $(CONFIG_DIR)/$(FC_VER)/$(CONFIG)

ifneq ($(wildcard $(CONFIG_FILE)),)

TARGET        := $(shell grep " FC_TARGET_MCU" $(CONFIG_FILE) | awk '{print $$3}' )
HSE_VALUE_MHZ := $(shell grep " SYSTEM_HSE_MHZ" $(CONFIG_FILE) | awk '{print $$3}' )
ifneq ($(HSE_VALUE_MHZ),)
HSE_VALUE     := $(shell echo $$(( $(HSE_VALUE_MHZ) * 1000000 )) )
endif

GYRO_DEFINE   := $(shell grep " USE_GYRO_" $(CONFIG_FILE) | awk '{print $$2}' )

ifeq ($(TARGET),)
$(error No TARGET identified. Is the $(CONFIG_FILE) valid for $(CONFIG)?)
endif

EXST_ADJUST_VMA := $(shell grep " FC_VMA_ADDRESS" $(CONFIG_FILE) | awk '{print $$3}' )
ifneq ($(EXST_ADJUST_VMA),)
EXST = yes
endif

endif #config_file exists
endif #config

$(CONFIG_FILES):
	@mkdir -p $(dir $@)
	$(V0) curl -L -k -o "$@" $(if $(wildcard $@),-z "$@") "$(API_URL)/$(FC_VER)/$(lastword $(subst /, ,$(dir $@)))/header"

$(BASE_CONFIGS_FILE):
	@mkdir -p $(dir $@)
	$(V0) curl -L -k -o "$@" $(if $(wildcard $@),-z "$@") "$(BASE_CONFIGS_URL)"

.PHONY: configs_clean
configs_clean:
	@echo "Cleaning configuration files for $(FC_VER)"
	@rm -rf $(CONFIG_DIR)/$(FC_VER)/*

.PHONY: configs
configs: $(BASE_CONFIGS_FILE)
	@echo "Valid configs: $(BASE_CONFIGS)"

.SECONDEXPANSION:
$(BASE_CONFIGS): $$(CONFIG_DIR)/$$(FC_VER)/$$@/config.h
	$(V0) @echo "Building config $@" && \
	$(MAKE) hex CONFIG=$@ && \
	echo "Building config $@ succeeded."

## <CONFIG>_clean    : clean up one specific config (alias for above)
$(CONFIGS_CLEAN): $(BASE_CONFIGS_FILE)
	$(V0) $(MAKE) -j CONFIG=$(subst _clean,,$@) clean
