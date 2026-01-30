###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the betaflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#
###############################################################################


# Things that the user might override on the commandline
#

# The target or config to build
TARGET    ?=
CONFIG    ?=

# Compile-time options
OPTIONS   ?=

# compile for External Storage Bootloader support
EXST      ?= no

# compile for target loaded into RAM
RAM_BASED ?= no

# reserve space for custom defaults
CUSTOM_DEFAULTS_EXTENDED ?= no

# Debugger optons:
#   empty - ordinary build with all optimizations enabled
#   INFO - ordinary build with debug symbols and all optimizations enabled. Only builds touched files.
#   GDB - debug build with minimum number of optimizations
DEBUG     ?=

# Insert the debugging hardfault debugger
# releases should not be built with this flag as it does not disable pwm output
DEBUG_HARDFAULTS ?=

# Serial port/Device for flashing
SERIAL_DEVICE   ?= $(firstword $(wildcard /dev/ttyACM*) $(firstword $(wildcard /dev/ttyUSB*) no-port-found))

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

# Disabled build flags
CFLAGS_DISABLED         ?=

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME      = betaflight

# Working directories
# ROOT_DIR is the full path to the directory containing this Makefile
ROOT_DIR        := $(realpath $(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
# ROOT is the relative path to the directory containing this Makefile
ROOT            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

PLATFORM_DIR	:= $(ROOT)/src/platform
SRC_DIR         := $(ROOT)/src/main
LIB_MAIN_DIR    := $(ROOT)/lib/main
OBJECT_DIR      := $(ROOT)/obj/main
BIN_DIR         := $(ROOT)/obj
CMSIS_DIR       := $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS    := $(SRC_DIR)

MAKE_SCRIPT_DIR := $(ROOT)/mk

## V                 : Set verbosity level based on the V= parameter
##                     V=0 Low
##                     V=1 High
include $(MAKE_SCRIPT_DIR)/build_verbosity.mk

# Build tools, so we all share the same versions
# import macros common to all supported build systems
include $(MAKE_SCRIPT_DIR)/system-id.mk

# developer preferences, edit these at will, they'll be gitignored
ifneq ($(wildcard $(MAKE_SCRIPT_DIR)/local.mk),)
include $(MAKE_SCRIPT_DIR)/local.mk
endif

# some targets use parallel build by default
# MAKEFLAGS is valid only inside target, do not use this at parse phase
DEFAULT_PARALLEL_JOBS 	:=    # all jobs in parallel (for backward compatibility)
MAKE_PARALLEL 		     = $(if $(filter -j%, $(MAKEFLAGS)),$(EMPTY),-j$(DEFAULT_PARALLEL_JOBS))

# pre-build sanity checks
include $(MAKE_SCRIPT_DIR)/checks.mk

# basic target list
PLATFORMS        := $(sort $(notdir $(patsubst /%,%, $(wildcard $(PLATFORM_DIR)/*))))
BASE_TARGETS     := $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(PLATFORM_DIR)/*/target/*/target.mk)))))

# configure some directories that are relative to wherever ROOT_DIR is located
TOOLS_DIR  ?= $(ROOT)/tools
DL_DIR     := $(ROOT)/downloads
CONFIG_DIR ?= $(BETAFLIGHT_CONFIG)
ifeq ($(CONFIG_DIR),)
CONFIG_DIR := $(ROOT)/src/config
endif
DIRECTORIES := $(DL_DIR) $(TOOLS_DIR)

export RM := rm

# import macros that are OS specific
include $(MAKE_SCRIPT_DIR)/$(OSFAMILY).mk

# include the tools makefile
include $(MAKE_SCRIPT_DIR)/tools.mk

# -----------------------------------------------------------------------------
# Tool discovery (need CROSS_CC available before config handling)
# -----------------------------------------------------------------------------

# Find out if ccache is installed on the system
CCACHE := ccache
RESULT = $(shell (which $(CCACHE) > /dev/null 2>&1; echo $$?) )
ifneq ($(RESULT),0)
CCACHE :=
endif

# Tool names (defer prefix resolution for per-platform overrides like SITL)
CROSS_CC     = $(CCACHE) $(ARM_SDK_PREFIX)gcc
CROSS_CXX    = $(CCACHE) $(ARM_SDK_PREFIX)g++
CROSS_GDB    = $(ARM_SDK_PREFIX)gdb
OBJCOPY      = $(ARM_SDK_PREFIX)objcopy
OBJDUMP      = $(ARM_SDK_PREFIX)objdump
READELF      = $(ARM_SDK_PREFIX)readelf
SIZE         = $(ARM_SDK_PREFIX)size
DFUSE-PACK  := src/utils/dfuse-pack.py

# Preprocessor helpers (generic .h parsing)
include $(MAKE_SCRIPT_DIR)/preprocess.mk

# Search path for sources
VPATH           := $(SRC_DIR):$(LIB_MAIN_DIR):$(PLATFORM_DIR)
FATFS_DIR        = $(ROOT)/lib/main/FatFS
FATFS_SRC        = $(notdir $(wildcard $(FATFS_DIR)/*.c))
CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

# import config handling (must occur after tool discovery and hydration)
include $(MAKE_SCRIPT_DIR)/config.mk

# default xtal value
HSE_VALUE       ?= 8000000

CI_EXCLUDED_TARGETS := $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(PLATFORM_DIR)/*/target/*/.exclude)))))
CI_COMMON_TARGETS   := STM32F4DISCOVERY CRAZYBEEF4SX1280 CRAZYBEEF4FR MATEKF405TE AIRBOTG4AIO TBS_LUCID_FC IFLIGHT_BLITZ_F722 NUCLEOF446 SPRACINGH7EXTREME SPRACINGH7RF
CI_TARGETS          := $(filter-out $(CI_EXCLUDED_TARGETS), $(BASE_TARGETS) $(filter $(CI_COMMON_TARGETS), $(BASE_CONFIGS)))
PREVIEW_TARGETS     := MATEKF411 AIKONF4V2 AIRBOTG4AIO ZEEZF7V3 FOXEERF745V4_AIO KAKUTEH7 TBS_LUCID_FC SITL SPRACINGH7EXTREME SPRACINGH7RF

TARGET_PLATFORM     := $(notdir $(patsubst %/,%,$(subst target/$(TARGET)/,, $(dir $(wildcard $(PLATFORM_DIR)/*/target/$(TARGET)/target.mk)))))
TARGET_PLATFORM_DIR := $(PLATFORM_DIR)/$(TARGET_PLATFORM)
LINKER_DIR          := $(TARGET_PLATFORM_DIR)/link

ifneq ($(TARGET),)
TARGET_DIR     = $(TARGET_PLATFORM_DIR)/target/$(TARGET)
include $(TARGET_DIR)/target.mk
endif

REVISION := norevision
ifeq ($(shell git diff --shortstat),)
REVISION := $(shell git rev-parse --short=9 HEAD)
endif

LD_FLAGS        :=
EXTRA_LD_FLAGS  :=

#
# Default Tool options - can be overridden in {mcu}.mk files.
#
DEBUG_MIXED = no

ifeq ($(DEBUG),INFO)
DEBUG_MIXED = yes
endif
ifeq ($(DEBUG),GDB)
DEBUG_MIXED = yes
endif

ifeq ($(DEBUG),GDB)
OPTIMISE_DEFAULT      := -Og

LTO_FLAGS             := $(OPTIMISE_DEFAULT)
DEBUG_FLAGS            = -ggdb2 -gdwarf-5 -DDEBUG
else
ifeq ($(DEBUG),INFO)
DEBUG_FLAGS            = -ggdb2
endif
OPTIMISATION_BASE     := -flto=auto -fuse-linker-plugin -ffast-math -fmerge-all-constants
OPTIMISE_DEFAULT      := -O2
OPTIMISE_SPEED        := -Ofast
OPTIMISE_SIZE         := -Os

LTO_FLAGS             := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif

VPATH 			:= $(VPATH):$(MAKE_SCRIPT_DIR)

ifneq ($(TARGET),)

# start specific includes
ifeq ($(TARGET_MCU),)
$(error No TARGET_MCU specified. Is the target.mk valid for $(TARGET)?)
endif

ifeq ($(TARGET_MCU_FAMILY),)
$(error No TARGET_MCU_FAMILY specified. Is the target.mk valid for $(TARGET)?)
endif

TARGET_FLAGS := -D$(TARGET) -D$(TARGET_PLATFORM) -D$(TARGET_MCU_FAMILY) $(TARGET_FLAGS)

ifneq ($(CONFIG),)
TARGET_FLAGS := $(TARGET_FLAGS) -DUSE_CONFIG
endif

SPEED_OPTIMISED_SRC :=
SIZE_OPTIMISED_SRC  :=

include $(TARGET_PLATFORM_DIR)/mk/$(TARGET_MCU_FAMILY).mk

# Configure default flash sizes for the targets (largest size specified gets hit first) if flash not specified already.
ifeq ($(TARGET_FLASH_SIZE),)
ifneq ($(MCU_FLASH_SIZE),)
TARGET_FLASH_SIZE := $(MCU_FLASH_SIZE)
else
$(error MCU_FLASH_SIZE not configured for target $(TARGET))
endif
endif

DEVICE_FLAGS  := $(DEVICE_FLAGS) -DTARGET_FLASH_SIZE=$(TARGET_FLASH_SIZE)

ifneq ($(HSE_VALUE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DHSE_VALUE=$(HSE_VALUE)
endif

endif # TARGET specified

ifeq ($(or $(CONFIG),$(TARGET)),)
.DEFAULT_GOAL := all
else
.DEFAULT_GOAL := fwo
endif

# openocd specific includes
include $(MAKE_SCRIPT_DIR)/openocd.mk

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(ROOT)/lib/main/MAVLink

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_DIR)

VPATH           := $(VPATH):$(TARGET_DIR)

include $(MAKE_SCRIPT_DIR)/source.mk

ifneq ($(TARGET),)
ifneq ($(filter-out $(SRC),$(SPEED_OPTIMISED_SRC)),)
$(error Speed optimised sources not valid: $(strip $(filter-out $(SRC),$(SPEED_OPTIMISED_SRC))))
endif

ifneq ($(filter-out $(SRC),$(SIZE_OPTIMISED_SRC)),)
$(error Size optimised sources not valid: $(strip $(filter-out $(SRC),$(SIZE_OPTIMISED_SRC))))
endif
endif

###############################################################################
# Things that might need changing to use different tools
#

#
# Tool options.
#
CC_DEBUG_OPTIMISATION   := $(OPTIMISE_DEFAULT)
CC_DEFAULT_OPTIMISATION := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
CC_SPEED_OPTIMISATION   := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
CC_SIZE_OPTIMISATION    := $(OPTIMISATION_BASE) $(OPTIMISE_SIZE)
CC_NO_OPTIMISATION      :=

CC_DEBUG_OPTIMISATION   := $(filter-out $(CFLAGS_DISABLED), $(CC_DEBUG_OPTIMISATION))
CC_DEFAULT_OPTIMISATION := $(filter-out $(CFLAGS_DISABLED), $(CC_DEFAULT_OPTIMISATION))
CC_SPEED_OPTIMISATION   := $(filter-out $(CFLAGS_DISABLED), $(CC_SPEED_OPTIMISATION))
CC_SIZE_OPTIMISATION    := $(filter-out $(CFLAGS_DISABLED), $(CC_SIZE_OPTIMISATION))
CC_NO_OPTIMISATION      := $(filter-out $(CFLAGS_DISABLED), $(CC_NO_OPTIMISATION))


# Extract version from the version header
# Expand FC_VERSION_STRING via the preprocessor; compute before finalizing CFLAGS
FC_VER           := $(call pp_def_value_str,src/main/build/version.h,FC_VERSION_STRING)

#
# Added after GCC version update, remove once the warnings have been fixed
#
TEMPORARY_FLAGS :=

EXTRA_WARNING_FLAGS := -Wold-style-definition

CFLAGS     += $(ARCH_FLAGS) \
              $(addprefix -D,$(OPTIONS)) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              $(addprefix -isystem,$(SYS_INCLUDE_DIRS)) \
              $(DEBUG_FLAGS) \
              -std=gnu17 \
              -Wall -Wextra -Werror -Wunsafe-loop-optimizations -Wdouble-promotion \
              $(EXTRA_WARNING_FLAGS) \
              -ffunction-sections \
              -fdata-sections \
              -fno-common \
              $(TEMPORARY_FLAGS) \
              $(DEVICE_FLAGS) \
              -D_GNU_SOURCE \
              -D$(TARGET) \
              $(TARGET_FLAGS) \
              -D'__FORKNAME__="$(FORKNAME)"' \
              -D'__TARGET__="$(TARGET)"' \
              -D'__REVISION__="$(REVISION)"' \
              -D'__FC_VERSION__="$(FC_VER)"' \
              $(CONFIG_REVISION_DEFINE) \
              -pipe \
              -MMD -MP \
              $(EXTRA_FLAGS)

CFLAGS     := $(filter-out $(CFLAGS_DISABLED), $(CFLAGS))

ASFLAGS     = $(ARCH_FLAGS) \
              $(DEBUG_FLAGS) \
              -x assembler-with-cpp \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              $(addprefix -isystem,$(SYS_INCLUDE_DIRS)) \
              -MMD -MP

ifeq ($(LD_FLAGS),)
LD_FLAGS     = -lm \
              -nostartfiles \
              --specs=nano.specs \
              -lc \
              -lnosys \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -static \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -Wl,--no-wchar-size-warning \
              -Wl,--print-memory-usage \
              -T$(LD_SCRIPT) \
               $(EXTRA_LD_FLAGS)
endif

LTO_FLAGS               := $(filter-out $(CFLAGS_DISABLED), $(LTO_FLAGS))

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK        = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
                  --std=c99 --inline-suppr --quiet --force \
                  $(addprefix -I,$(INCLUDE_DIRS)) \
                  $(addprefix -isystem,$(SYS_INCLUDE_DIRS)) \
                  -I/usr/include -I/usr/include/linux

ifneq ($(filter fwo hex uf2 bin elf zip, $(MAKECMDGOALS)),)
    ifeq ($(TARGET),)
        $(error "You must specify a target to build.")
    endif
endif

TARGET_NAME := $(TARGET)

ifneq ($(CONFIG),)
TARGET_NAME := $(TARGET_NAME)_$(CONFIG)
endif

TARGET_NAME_CLEAN := $(TARGET_NAME)

ifeq ($(REV),yes)
TARGET_NAME := $(TARGET_NAME)_$(REVISION)
endif

TARGET_FULLNAME = $(FORKNAME)_$(FC_VER)_$(TARGET_NAME)
#
# Things we will build
#
TARGET_BIN      := $(BIN_DIR)/$(TARGET_FULLNAME).bin
TARGET_HEX      := $(BIN_DIR)/$(TARGET_FULLNAME).hex
TARGET_UF2      := $(BIN_DIR)/$(TARGET_FULLNAME).uf2
TARGET_EXE      := $(BIN_DIR)/$(TARGET_FULLNAME)
TARGET_DFU      := $(BIN_DIR)/$(TARGET_FULLNAME).dfu
TARGET_ZIP      := $(BIN_DIR)/$(TARGET_FULLNAME).zip
TARGET_OBJ_DIR  := $(OBJECT_DIR)/$(TARGET_NAME)
TARGET_ELF      := $(OBJECT_DIR)/$(FORKNAME)_$(TARGET_NAME).elf
TARGET_EXST_ELF := $(OBJECT_DIR)/$(FORKNAME)_$(TARGET_NAME)_EXST.elf
TARGET_UNPATCHED_BIN := $(OBJECT_DIR)/$(FORKNAME)_$(TARGET_NAME)_UNPATCHED.bin
TARGET_LST      := $(OBJECT_DIR)/$(FORKNAME)_$(TARGET_NAME).lst
TARGET_OBJS     := $(addsuffix .o,$(addprefix $(TARGET_OBJ_DIR)/,$(basename $(SRC))))
TARGET_DEPS     := $(addsuffix .d,$(addprefix $(TARGET_OBJ_DIR)/,$(basename $(SRC))))
TARGET_MAP      := $(OBJECT_DIR)/$(FORKNAME)_$(TARGET_NAME).map

TARGET_EXST_HASH_SECTION_FILE := $(TARGET_OBJ_DIR)/exst_hash_section.bin

ifeq ($(DEBUG_MIXED),yes)
TARGET_EF_HASH      := $(shell echo -n -- "$(EXTRA_FLAGS)" "$(OPTIONS)" "$(DEVICE_FLAGS)" "$(TARGET_FLAGS)"  | openssl dgst -md5 -r | awk '{print $$1;}')
else
TARGET_EF_HASH      := $(shell echo -n -- "$(EXTRA_FLAGS)" "$(OPTIONS)" "$(DEBUG_FLAGS)" "$(DEVICE_FLAGS)" "$(TARGET_FLAGS)"  | openssl dgst -md5 -r | awk '{print $$1;}')
endif

TARGET_EF_HASH_FILE := $(TARGET_OBJ_DIR)/.efhash_$(TARGET_EF_HASH)

CLEAN_ARTIFACTS := $(TARGET_ELF) $(TARGET_EXST_ELF) $(TARGET_MAP)
CLEAN_ARTIFACTS += $(wildcard $(BIN_DIR)/*$(TARGET_NAME_CLEAN)*)

# Make sure build date and revision is updated on every incremental build
$(TARGET_OBJ_DIR)/build/version.o : $(SRC)

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_LST): $(TARGET_ELF)
	$(V0) $(OBJDUMP) -S --disassemble $< > $@

ifeq ($(EXST),no)
$(TARGET_BIN): $(TARGET_ELF)
	@echo "Creating BIN $(TARGET_BIN)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O binary $< $@

$(TARGET_HEX): $(TARGET_ELF)
	@echo "Creating HEX $(TARGET_HEX)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_DFU): $(TARGET_HEX)
	@echo "Creating DFU $(TARGET_DFU)" "$(STDOUT)"
	$(V1) $(PYTHON) $(DFUSE-PACK) -i $< $@

else
$(TARGET_UNPATCHED_BIN): $(TARGET_ELF)
	@echo "Creating BIN (without checksum) $(TARGET_UNPATCHED_BIN)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O binary $< $@

$(TARGET_BIN): $(TARGET_UNPATCHED_BIN)
	@echo "Creating EXST $(TARGET_BIN)" "$(STDOUT)"
# Linker script should allow .bin generation from a .elf which results in a file that is the same length as the FIRMWARE_SIZE.
# These 'dd' commands will pad a short binary to length FIRMWARE_SIZE.
	$(V1) dd if=/dev/zero ibs=1k count=$(FIRMWARE_SIZE) of=$(TARGET_BIN)
	$(V1) dd if=$(TARGET_UNPATCHED_BIN) of=$(TARGET_BIN) conv=notrunc

	@echo "Generating MD5 hash of binary" "$(STDOUT)"
	$(V1) openssl dgst -md5 $(TARGET_BIN) > $(TARGET_UNPATCHED_BIN).md5

	@echo "Patching MD5 hash into binary" "$(STDOUT)"
	$(V1) cat $(TARGET_UNPATCHED_BIN).md5 | awk '{printf("%08x: %s",(1024*$(FIRMWARE_SIZE))-16,$$2);}' | xxd -r - $(TARGET_BIN)
	$(V1) echo $(FIRMWARE_SIZE) | awk '{printf("-s 0x%08x -l 16 -c 16 %s",(1024*$$1)-16,"$(TARGET_BIN)");}' | xargs xxd

# Note: From the objcopy manual "If you do not specify outfile, objcopy creates a temporary file and destructively renames the result with the name of infile"
# Due to this a temporary file must be created and removed, even though we're only extracting data from the input file.
# If this temporary file is NOT used the $(TARGET_ELF) is modified, and running make a second time will result in
# a) regeneration of $(TARGET_BIN), and
# b) the results of $(TARGET_BIN) will not be as expected.
	@echo "Extracting HASH section from unpatched EXST elf $(TARGET_ELF)" "$(STDOUT)"
	$(OBJCOPY) $(TARGET_ELF) $(TARGET_EXST_ELF).tmp --dump-section .exst_hash=$(TARGET_EXST_HASH_SECTION_FILE) -j .exst_hash
	rm $(TARGET_EXST_ELF).tmp

	@echo "Patching MD5 hash into HASH section" "$(STDOUT)"
	$(V1) cat $(TARGET_UNPATCHED_BIN).md5 | awk '{printf("%08x: %s",64-16,$$2);}' | xxd -r - $(TARGET_EXST_HASH_SECTION_FILE)

	$(V1) @echo "Patching updated HASH section into $(TARGET_EXST_ELF)" "$(STDOUT)"
	$(OBJCOPY) $(TARGET_ELF) $(TARGET_EXST_ELF) --update-section .exst_hash=$(TARGET_EXST_HASH_SECTION_FILE)

	$(V1) $(READELF) -S $(TARGET_EXST_ELF)
	$(V1) $(READELF) -l $(TARGET_EXST_ELF)

$(TARGET_HEX): $(TARGET_BIN)
	$(if $(EXST_ADJUST_VMA),,$(error "EXST_ADJUST_VMA not specified"))

	@echo "Creating EXST HEX from patched EXST BIN $(TARGET_BIN), VMA Adjust $(EXST_ADJUST_VMA)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -I binary -O ihex --adjust-vma=$(EXST_ADJUST_VMA) $(TARGET_BIN) $@

endif

$(TARGET_ELF): $(TARGET_OBJS) $(LD_SCRIPT) $(LD_SCRIPTS)
	@echo "Linking $(TARGET_NAME)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -o $@ $(filter-out %.ld,$^) $(LD_FLAGS)
	$(V1) $(SIZE) $(TARGET_ELF)

$(TARGET_UF2): $(TARGET_ELF)
	@echo "Creating UF2 $(TARGET_UF2)" "$(STDOUT)"
	$(V1) $(PICOTOOL) uf2 convert $< $@ || { echo "Failed to convert ELF to UF2 format"; exit 1; }

$(TARGET_EXE): $(TARGET_ELF)
	@echo "Creating exe - copying $< to $@" "$(STDOUT)"
	$(V1) cp $< $@

# Compile

## compile_file takes two arguments: (1) optimisation description string and (2) optimisation compiler flag
define compile_file
	echo "%% ($(1)) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(2) $<
endef

## `paths` is a list of paths that will be replaced for checking of speed, and size optimised sources
paths := $(SRC_DIR)/ $(LIB_MAIN_DIR)/ $(PLATFORM_DIR)/
subst_paths_for = $(foreach path,$(paths),$(filter-out $(1),$(subst $(path),,$(1))))
subst_paths = $(strip $(if $(call subst_paths_for,$(1)), $(call subst_paths_for,$(1)), $(1)))

ifeq ($(DEBUG),GDB)
$(TARGET_OBJ_DIR)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $<,$(NOT_OPTIMISED_SRC)), \
		$(call compile_file,not optimised, $(CC_NO_OPTIMISATION)) \
	, \
		$(call compile_file,debug,$(CC_DEBUG_OPTIMISATION)) \
	)
else
$(TARGET_OBJ_DIR)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $<,$(NOT_OPTIMISED_SRC)), \
		$(call compile_file,not optimised,$(CC_NO_OPTIMISATION)) \
	, \
		$(if $(findstring $(call subst_paths,$<),$(SPEED_OPTIMISED_SRC)), \
			$(call compile_file,speed optimised,$(CC_SPEED_OPTIMISATION)) \
		, \
			$(if $(findstring $(call subst_paths,$<),$(SIZE_OPTIMISED_SRC)), \
				$(call compile_file,size optimised,$(CC_SIZE_OPTIMISATION)) \
			, \
				$(call compile_file,optimised,$(CC_DEFAULT_OPTIMISATION)) \
			) \
		) \
	)
endif

# Assemble
$(TARGET_OBJ_DIR)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	@echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(TARGET_OBJ_DIR)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	@echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

## all               : Build all currently built targets
all: $(CI_TARGETS)

.PHONY: $(BASE_TARGETS)
$(BASE_TARGETS):
	$(MAKE) fwo TARGET=$@

TARGETS_CLEAN = $(addsuffix _clean,$(BASE_TARGETS))

CONFIGS_CLEAN = $(addsuffix _clean,$(BASE_CONFIGS))

## clean             : clean up temporary / machine-generated files
clean:
ifeq ($(strip $(TARGET_NAME)),)
	@echo "No TARGET/CONFIG name specified. Performing full cleanup of $(BIN_DIR)"
	@if [ -d "$(BIN_DIR)" ]; then \
		echo "Removing directory: $(BIN_DIR)"; \
		rm -rf "$(BIN_DIR)"; \
	fi
	@echo "Cleaning all succeeded."
else
	@echo "Cleaning $(TARGET_NAME)"
	$(eval FILES_TO_REMOVE := $(wildcard $(CLEAN_ARTIFACTS)))
	@if [ -n "$(FILES_TO_REMOVE)" ]; then \
		echo "Removing artifacts:"; \
		echo $(FILES_TO_REMOVE) | tr ' ' '\n'; \
		rm -f $(FILES_TO_REMOVE); \
	fi
	@if [ -d "$(TARGET_OBJ_DIR)" ]; then \
		echo "Removing object directory:"; \
		echo $(TARGET_OBJ_DIR); \
		rm -rf $(TARGET_OBJ_DIR); \
	fi
	@echo "Cleaning $(TARGET_NAME) succeeded."
endif

## test_clean        : clean up temporary / machine-generated files (tests)
test-%_clean:
	$(MAKE) test_clean

test_clean:
	$(V0) cd src/test && $(MAKE) clean || true

## <TARGET>_clean    : clean up one specific target (alias for above)
$(TARGETS_CLEAN):
	$(V0) $(MAKE) $(MAKE_PARALLEL) TARGET=$(subst _clean,,$@) clean

## <CONFIG>_clean    : clean up one specific target (alias for above)
$(CONFIGS_CLEAN):
	$(V0) $(MAKE) $(MAKE_PARALLEL) CONFIG=$(subst _clean,,$@) clean

## clean_all         : clean all targets
clean_all: $(TARGETS_CLEAN) test_clean

## preview           : build one target for each platform and execute make test
preview: $(PREVIEW_TARGETS) test

## all_configs       : Build all configs
all_configs: $(BASE_CONFIGS)

TARGETS_FLASH = $(addsuffix _flash,$(BASE_TARGETS))

## <TARGET>_flash    : build and flash a target
$(TARGETS_FLASH):
	$(V0) $(MAKE) hex TARGET=$(subst _flash,,$@)
ifneq (,$(findstring /dev/ttyUSB,$(SERIAL_DEVICE)))
	$(V0) $(MAKE) tty_flash TARGET=$(subst _flash,,$@)
else
	$(V0) $(MAKE) dfu_flash TARGET=$(subst _flash,,$@)
endif

## tty_flash         : flash firmware (.hex) onto flight controller via a serial port
tty_flash:
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) echo -n 'R' > $(SERIAL_DEVICE)
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## dfu_flash         : flash firmware (.bin) onto flight controller via a DFU mode
dfu_flash:
ifneq (no-port-found,$(SERIAL_DEVICE))
	# potentially this is because the MCU already is in DFU mode, try anyway
	$(V0) echo -n 'R' > $(SERIAL_DEVICE)
	$(V0) sleep 1
endif
	$(V0) $(MAKE) $(TARGET_DFU)
	$(V0) dfu-util -a 0 -D $(TARGET_DFU) -s :leave

st-flash_$(TARGET): $(TARGET_BIN)
	$(V0) st-flash --reset write $< 0x08000000

## st-flash          : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

ifneq ($(OPENOCD_COMMAND),)
openocd-gdb: $(TARGET_ELF)
	$(V0) $(OPENOCD_COMMAND) & $(CROSS_GDB) $(TARGET_ELF) -ex "target remote localhost:3333" -ex "load"
endif

TARGETS_ZIP = $(addsuffix _zip,$(BASE_TARGETS))

## <TARGET>_zip    : build target and zip it (useful for posting to GitHub)
.PHONY: $(TARGETS_ZIP)
$(TARGETS_ZIP):
	$(V1) $(MAKE) $(MAKE_PARALLEL) zip TARGET=$(subst _zip,,$@)

.PHONY: zip
zip: $(TARGET_HEX)
	$(V1) zip $(TARGET_ZIP) $(TARGET_HEX)

.PHONY: binary
binary:
	$(V1) $(MAKE) $(MAKE_PARALLEL) $(TARGET_BIN)

.PHONY: hex
hex:
	$(V1) $(MAKE) $(MAKE_PARALLEL) $(TARGET_HEX)

.PHONY: uf2
uf2:
	$(V1) $(MAKE) $(MAKE_PARALLEL) $(TARGET_UF2)

.PHONY: exe
exe: $(TARGET_EXE)

# FWO (Firmware Output) is the default output for building the firmware
.PHONY: fwo
fwo:
ifeq ($(DEFAULT_OUTPUT),exe)
	$(V1) $(MAKE) exe
else ifeq ($(DEFAULT_OUTPUT),uf2)
	$(V1) $(MAKE) uf2
else
	$(V1) $(MAKE) hex
endif

TARGETS_REVISION = $(addsuffix _rev, $(BASE_TARGETS))
## <TARGET>_rev    : build target and add revision to filename
.PHONY: $(TARGETS_REVISION)
$(TARGETS_REVISION):
	$(V1) $(MAKE) fwo REV=yes TARGET=$(subst _rev,,$@)

.PHONY: all_rev
all_rev: $(addsuffix _rev, $(CI_TARGETS))

.PHONY: unbrick_$(TARGET)
unbrick_$(TARGET): $(TARGET_HEX)
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick           : unbrick flight controller
.PHONY: unbrick
unbrick: unbrick_$(TARGET)

## cppcheck          : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(V0) $(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(V0) $(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

# mkdirs
$(DIRECTORIES):
	mkdir -p $@

## version           : print firmware version
.PHONY: version
version:
	@echo '$(FC_VER)'

## help              : print this help message and exit
help: Makefile mk/tools.mk
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [V=<verbosity>] [TARGET=<target>] [OPTIONS=\"<options>\"] [EXTRA_FLAGS=\"<extra_flags>\"]"
	@echo "Or:"
	@echo "        make <target> [V=<verbosity>] [OPTIONS=\"<options>\"] [EXTRA_FLAGS=\"<extra_flags>\"]"
	@echo "Or:"
	@echo "        make <config-target> [V=<verbosity>] [OPTIONS=\"<options>\"] [EXTRA_FLAGS=\"<extra_flags>\"]"
	@echo ""
	@echo "To populate configuration targets:"
	@echo "        make configs"
	@echo ""
	@echo "Valid TARGET values are: $(BASE_TARGETS)"
	@echo ""
	@sed -n 's/^## //p' $?

## targets           : print a list of all valid target platforms (for consumption by scripts)
targets:
	@echo "Platforms:           $(PLATFORMS)"
	@echo "Valid targets:       $(BASE_TARGETS)"
	@echo "Built targets:       $(CI_TARGETS)"
	@echo "Default target:      $(TARGET)"
	@echo "CI common targets:   $(CI_COMMON_TARGETS)"
	@echo "CI excluded targets: $(CI_EXCLUDED_TARGETS)"
	@echo "Preview targets:     $(PREVIEW_TARGETS)"

targets-ci-print:
	@echo $(CI_TARGETS)

## target-mcu        : print the MCU type of the target
target-mcu:
	@echo "$(TARGET_MCU_FAMILY) : $(TARGET_MCU)"

## targets-by-mcu    : make all targets that have a MCU_TYPE mcu
targets-by-mcu:
	$(V1) for target in $${TARGETS}; do \
		TARGET_MCU_TYPE=$$($(MAKE) -s TARGET=$${target} target-mcu); \
		if [ "$${TARGET_MCU_TYPE}" = "$${MCU_TYPE}" ]; then \
			if [ "$${DO_BUILD}" = 1 ]; then \
				echo "Building target $${target}..."; \
				$(MAKE) TARGET=$${target}; \
				if [ $$? -ne 0 ]; then \
					echo "Building target $${target} failed, aborting."; \
					exit 1; \
				fi; \
			else \
				echo -n "$${target} "; \
			fi; \
		fi; \
	done
	@echo

## test              : run the Betaflight test suite
## junittest         : run the Betaflight test suite, producing Junit XML result files.
## test-representative: run a representative subset of the Betaflight test suite (i.e. run all tests, but run each expanded test only for one target)
## test-all: run the Betaflight test suite including all per-target expanded tests
test junittest test-all test-representative:
	$(V0) cd src/test && $(MAKE) $@

## test_help         : print the help message for the test suite (including a list of the available tests)
test_help:
	$(V0) cd src/test && $(MAKE) help

## test_versions         : print the compiler versions used for the test suite
test_versions:
	$(V0) cd src/test && $(MAKE) versions

## test_%            : run test 'test_%' from the test suite
test_%:
	$(V0) cd src/test && $(MAKE) $@

$(TARGET_EF_HASH_FILE):
	$(V1) mkdir -p $(dir $@)
	$(V1) rm -f $(TARGET_OBJ_DIR)/.efhash_*
	@echo "EF HASH -> $(TARGET_EF_HASH_FILE)"
	$(V1) touch $(TARGET_EF_HASH_FILE)

# rebuild everything when makefile changes or the extra flags have changed
$(TARGET_OBJS): $(TARGET_EF_HASH_FILE) Makefile $(TARGET_DIR)/target.mk $(wildcard make/*) $(CONFIG_FILE)

# include auto-generated dependencies
-include $(TARGET_DEPS)
