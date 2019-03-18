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

# The target to build, see VALID_TARGETS below
TARGET    ?= BETAFLIGHTF3

# Compile-time options
OPTIONS   ?=

# compile for OpenPilot BootLoader support
OPBL      ?= no

# Debugger optons:
#   empty           - ordinary build with all optimizations enabled
#   RELWITHDEBINFO  - ordinary build with debug symbols and all optimizations enabled
#   GDB             - debug build with minimum number of optimizations
DEBUG     ?=

# Insert the debugging hardfault debugger
# releases should not be built with this flag as it does not disable pwm output
DEBUG_HARDFAULTS ?=

# Serial port/Device for flashing
SERIAL_DEVICE   ?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=


###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME      = betaflight

# Working directories
ROOT            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR         := $(ROOT)/src/main
OBJECT_DIR      := $(ROOT)/obj/main
BIN_DIR         := $(ROOT)/obj
CMSIS_DIR       := $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS    := $(SRC_DIR) \
                   $(ROOT)/src/main/target \
                   $(ROOT)/src/main/startup
LINKER_DIR      := $(ROOT)/src/main/target/link

## V                 : Set verbosity level based on the V= parameter
##                     V=0 Low
##                     V=1 High
include $(ROOT)/make/build_verbosity.mk

# Build tools, so we all share the same versions
# import macros common to all supported build systems
include $(ROOT)/make/system-id.mk

# developer preferences, edit these at will, they'll be gitignored
-include $(ROOT)/make/local.mk

# configure some directories that are relative to wherever ROOT_DIR is located
ifndef TOOLS_DIR
TOOLS_DIR := $(ROOT)/tools
endif
BUILD_DIR := $(ROOT)/build
DL_DIR    := $(ROOT)/downloads

export RM := rm

# import macros that are OS specific
include $(ROOT)/make/$(OSFAMILY).mk

# include the tools makefile
include $(ROOT)/make/tools.mk

# default xtal value for F4 targets
HSE_VALUE       ?= 8000000

# used for turning on features like VCP and SDCARD
FEATURES        =

# used to disable features based on flash space shortage (larger number => more features disabled)
FEATURE_CUT_LEVEL_SUPPLIED := $(FEATURE_CUT_LEVEL)
FEATURE_CUT_LEVEL =

# The list of targets to build for 'pre-push'
PRE_PUSH_TARGET_LIST ?= BETAFLIGHTF3 OMNIBUSF4 SPRACINGF7DUAL SITL test-representative

include $(ROOT)/make/targets.mk

REVISION := $(shell git log -1 --format="%h")

FC_VER_MAJOR := $(shell grep " FC_VERSION_MAJOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_MINOR := $(shell grep " FC_VERSION_MINOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_PATCH := $(shell grep " FC_VERSION_PATCH" src/main/build/version.h | awk '{print $$3}' )

FC_VER := $(FC_VER_MAJOR).$(FC_VER_MINOR).$(FC_VER_PATCH)

# Search path for sources
VPATH           := $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR       = $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC   = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))
FATFS_DIR       = $(ROOT)/lib/main/FatFS
FATFS_SRC       = $(notdir $(wildcard $(FATFS_DIR)/*.c))

CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

LD_FLAGS         :=

#
# Default Tool options - can be overridden in {mcu}.mk files.
#
ifeq ($(DEBUG),GDB)
OPTIMISE_DEFAULT      := -Og

LTO_FLAGS             := $(OPTIMISE_DEFAULT)
DEBUG_FLAGS            = -ggdb3 -DDEBUG
else
ifeq ($(DEBUG),INFO)
DEBUG_FLAGS            = -ggdb3
endif
OPTIMISATION_BASE     := -flto -fuse-linker-plugin -ffast-math
OPTIMISE_DEFAULT      := -O2
OPTIMISE_SPEED        := -Ofast
OPTIMISE_SIZE         := -Os

LTO_FLAGS             := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif

VPATH 			:= $(VPATH):$(ROOT)/make/mcu
VPATH 			:= $(VPATH):$(ROOT)/make

# start specific includes
include $(ROOT)/make/mcu/$(TARGET_MCU).mk

# openocd specific includes
include $(ROOT)/make/openocd.mk

# Configure default flash sizes for the targets (largest size specified gets hit first) if flash not specified already.
ifeq ($(FLASH_SIZE),)
ifneq ($(TARGET_FLASH),)
FLASH_SIZE := $(TARGET_FLASH)
else
$(error FLASH_SIZE not configured for target $(TARGET))
endif
endif

DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)

ifneq ($(HSE_VALUE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DHSE_VALUE=$(HSE_VALUE)
endif

ifneq ($(FEATURE_CUT_LEVEL_SUPPLIED),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFEATURE_CUT_LEVEL=$(FEATURE_CUT_LEVEL_SUPPLIED)
else ifneq ($(FEATURE_CUT_LEVEL),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFEATURE_CUT_LEVEL=$(FEATURE_CUT_LEVEL)
endif

TARGET_DIR     = $(ROOT)/src/main/target/$(BASE_TARGET)
TARGET_DIR_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

ifeq ($(OPBL),yes)
TARGET_FLAGS := -DOPBL $(TARGET_FLAGS)
.DEFAULT_GOAL := binary
else
.DEFAULT_GOAL := hex
endif

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(ROOT)/lib/main/MAVLink

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_DIR)

VPATH           := $(VPATH):$(TARGET_DIR)

include $(ROOT)/make/source.mk

###############################################################################
# Things that might need changing to use different tools
#

# Find out if ccache is installed on the system
CCACHE := ccache
RESULT = $(shell (which $(CCACHE) > /dev/null 2>&1; echo $$?) )
ifneq ($(RESULT),0)
CCACHE :=
endif

# Tool names
CROSS_CC    := $(CCACHE) $(ARM_SDK_PREFIX)gcc
CROSS_CXX   := $(CCACHE) $(ARM_SDK_PREFIX)g++
CROSS_GDB   := $(ARM_SDK_PREFIX)gdb
OBJCOPY     := $(ARM_SDK_PREFIX)objcopy
OBJDUMP     := $(ARM_SDK_PREFIX)objdump
SIZE        := $(ARM_SDK_PREFIX)size

#
# Tool options.
#
CC_DEBUG_OPTIMISATION   := $(OPTIMISE_DEFAULT)
CC_DEFAULT_OPTIMISATION := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
CC_SPEED_OPTIMISATION   := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
CC_SIZE_OPTIMISATION    := $(OPTIMISATION_BASE) $(OPTIMISE_SIZE)
CC_NO_OPTIMISATION      := 

CFLAGS     += $(ARCH_FLAGS) \
              $(addprefix -D,$(OPTIONS)) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              $(DEBUG_FLAGS) \
              -std=gnu11 \
              -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
              -ffunction-sections \
              -fdata-sections \
              -fno-common \
              -pedantic \
              $(DEVICE_FLAGS) \
              -D_GNU_SOURCE \
              -DUSE_STDPERIPH_DRIVER \
              -D$(TARGET) \
              $(TARGET_FLAGS) \
              -D'__FORKNAME__="$(FORKNAME)"' \
              -D'__TARGET__="$(TARGET)"' \
              -D'__REVISION__="$(REVISION)"' \
              -save-temps=obj \
              -MMD -MP \
              $(EXTRA_FLAGS)

ASFLAGS     = $(ARCH_FLAGS) \
              $(DEBUG_FLAGS) \
              -x assembler-with-cpp \
              $(addprefix -I,$(INCLUDE_DIRS)) \
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
              -T$(LD_SCRIPT)
endif

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK        = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
                  --std=c99 --inline-suppr --quiet --force \
                  $(addprefix -I,$(INCLUDE_DIRS)) \
                  -I/usr/include -I/usr/include/linux

#
# Things we will build
#
TARGET_BIN      = $(BIN_DIR)/$(FORKNAME)_$(FC_VER)_$(TARGET).bin
TARGET_HEX      = $(BIN_DIR)/$(FORKNAME)_$(FC_VER)_$(TARGET).hex
TARGET_ELF      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_LST      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).lst
TARGET_OBJS     = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC))))
TARGET_DEPS     = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC))))
TARGET_MAP      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map


CLEAN_ARTIFACTS := $(TARGET_BIN)
CLEAN_ARTIFACTS += $(TARGET_HEX)
CLEAN_ARTIFACTS += $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
CLEAN_ARTIFACTS += $(TARGET_LST)

# Make sure build date and revision is updated on every incremental build
$(OBJECT_DIR)/$(TARGET)/build/version.o : $(SRC)

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_LST): $(TARGET_ELF)
	$(V0) $(OBJDUMP) -S --disassemble $< > $@

$(TARGET_HEX): $(TARGET_ELF)
	@echo "Creating HEX $(TARGET_HEX)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	@echo "Creating BIN $(TARGET_BIN)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O binary $< $@

$(TARGET_ELF): $(TARGET_OBJS) $(LD_SCRIPT)
	@echo "Linking $(TARGET)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -o $@ $(filter-out %.ld,$^) $(LD_FLAGS)
	$(V1) $(SIZE) $(TARGET_ELF)

# Compile

## compile_file takes two arguments: (1) optimisation description string and (2) optimisation compiler flag
define compile_file
	echo "%% ($(1)) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(2) $<
endef

ifeq ($(DEBUG),GDB)
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $<,$(NOT_OPTIMISED_SRC)), \
		$(call compile_file,not optimised, $(CC_NO_OPTIMISATION)) \
	, \
		$(call compile_file,debug,$(CC_DEBUG_OPTIMISATION)) \
	)
else
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $<,$(NOT_OPTIMISED_SRC)), \
		$(call compile_file,not optimised,$(CC_NO_OPTIMISATION)) \
	, \
		$(if $(findstring $(subst ./src/main/,,$<),$(SPEED_OPTIMISED_SRC)), \
			$(call compile_file,speed optimised,$(CC_SPEED_OPTIMISATION)) \
		, \
			$(if $(findstring $(subst ./src/main/,,$<),$(SIZE_OPTIMISED_SRC)), \
				$(call compile_file,size optimised,$(CC_SIZE_OPTIMISATION)) \
			, \
				$(call compile_file,optimised,$(CC_DEFAULT_OPTIMISATION)) \
			) \
		) \
	)
endif

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	@echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	@echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<


## all               : Build all targets (excluding unsupported)
all: $(SUPPORTED_TARGETS)

## all_with_unsupported : Build all targets (including unsupported)
all_with_unsupported: $(VALID_TARGETS)

## unsupported : Build unsupported targets
unsupported: $(UNSUPPORTED_TARGETS)

## pre-push : The minimum verification that should be run before pushing, to check if CI has a chance of succeeding
pre-push:
	$(MAKE) $(addprefix clean_,$(PRE_PUSH_TARGET_LIST)) $(PRE_PUSH_TARGET_LIST) EXTRA_FLAGS=-Werror

## official          : Build all official (travis) targets
official: $(OFFICIAL_TARGETS)

## targets-group-1   : build some targets
targets-group-1: $(GROUP_1_TARGETS)

## targets-group-2   : build some targets
targets-group-2: $(GROUP_2_TARGETS)

## targets-group-3   : build some targets
targets-group-3: $(GROUP_3_TARGETS)

## targets-group-3   : build some targets
targets-group-4: $(GROUP_4_TARGETS)

## targets-group-rest: build the rest of the targets (not listed in group 1, 2 or 3)
targets-group-rest: $(GROUP_OTHER_TARGETS)

$(VALID_TARGETS):
	$(V0) @echo "Building $@" && \
	$(MAKE) binary hex TARGET=$@ && \
	echo "Building $@ succeeded."

$(NOBUILD_TARGETS):
	$(MAKE) TARGET=$@

CLEAN_TARGETS = $(addprefix clean_,$(VALID_TARGETS) )
TARGETS_CLEAN = $(addsuffix _clean,$(VALID_TARGETS) )

## clean             : clean up temporary / machine-generated files
clean:
	@echo "Cleaning $(TARGET)"
	$(V0) rm -f $(CLEAN_ARTIFACTS)
	$(V0) rm -rf $(OBJECT_DIR)/$(TARGET)
	@echo "Cleaning $(TARGET) succeeded."

## clean_test        : clean up temporary / machine-generated files (tests)
clean_test-%:
	$(MAKE) clean_test

clean_test:
	$(V0) cd src/test && $(MAKE) clean || true

## clean_<TARGET>    : clean up one specific target
$(CLEAN_TARGETS):
	$(V0) $(MAKE) -j TARGET=$(subst clean_,,$@) clean

## <TARGET>_clean    : clean up one specific target (alias for above)
$(TARGETS_CLEAN):
	$(V0) $(MAKE) -j TARGET=$(subst _clean,,$@) clean

## clean_all         : clean all valid targets
clean_all: $(CLEAN_TARGETS)

## all_clean         : clean all valid targets (alias for above)
all_clean: $(TARGETS_CLEAN)


flash_$(TARGET): $(TARGET_HEX)
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) echo -n 'R' >$(SERIAL_DEVICE)
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## flash             : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

st-flash_$(TARGET): $(TARGET_BIN)
	$(V0) st-flash --reset write $< 0x08000000

## st-flash          : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

ifneq ($(OPENOCD_COMMAND),)
openocd-gdb: $(TARGET_ELF)
	$(V0) $(OPENOCD_COMMAND) & $(CROSS_GDB) $(TARGET_ELF) -ex "target remote localhost:3333" -ex "load"
endif

binary:
	$(V0) $(MAKE) -j $(TARGET_BIN)

hex:
	$(V0) $(MAKE) -j $(TARGET_HEX)

unbrick_$(TARGET): $(TARGET_HEX)
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick           : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck          : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(V0) $(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(V0) $(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

# mkdirs
$(DL_DIR):
	mkdir -p $@

$(TOOLS_DIR):
	mkdir -p $@

$(BUILD_DIR):
	mkdir -p $@

## version           : print firmware version
version:
	@echo $(FC_VER)

## help              : print this help message and exit
help: Makefile make/tools.mk
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [V=<verbosity>] [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo "Or:"
	@echo "        make <target> [V=<verbosity>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
	@sed -n 's/^## //p' $?

## targets           : print a list of all valid target platforms (for consumption by scripts)
targets:
	@echo "Valid targets:       $(VALID_TARGETS)"
	@echo "Supported targets:   $(SUPPORTED_TARGETS)"
	@echo "Unsupported targets: $(UNSUPPORTED_TARGETS)"
	@echo "Target:              $(TARGET)"
	@echo "Base target:         $(BASE_TARGET)"
	@echo "targets-group-1:     $(GROUP_1_TARGETS)"
	@echo "targets-group-2:     $(GROUP_2_TARGETS)"
	@echo "targets-group-3:     $(GROUP_3_TARGETS)"
	@echo "targets-group-4:     $(GROUP_4_TARGETS)"
	@echo "targets-group-rest:  $(GROUP_OTHER_TARGETS)"

	@echo "targets-group-1:     $(words $(GROUP_1_TARGETS)) targets"
	@echo "targets-group-2:     $(words $(GROUP_2_TARGETS)) targets"
	@echo "targets-group-3:     $(words $(GROUP_3_TARGETS)) targets"
	@echo "targets-group-4:     $(words $(GROUP_4_TARGETS)) targets"
	@echo "targets-group-rest:  $(words $(GROUP_OTHER_TARGETS)) targets"
	@echo "total in all groups  $(words $(SUPPORTED_TARGETS)) targets"

## target-mcu        : print the MCU type of the target
target-mcu:
	@echo $(TARGET_MCU)

## targets-by-mcu    : make all targets that have a MCU_TYPE mcu
targets-by-mcu:
	$(V1) for target in $(VALID_TARGETS); do \
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

## targets-f3        : make all F3 targets
targets-f3:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F3 DO_BUILD=1

targets-f3-print:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F3

## targets-f4        : make all F4 targets
targets-f4:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F4 DO_BUILD=1

targets-f4-print:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F4

## targets-f7        : make all F7 targets
targets-f7:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F7 DO_BUILD=1

targets-f7-print:
	$(V1) $(MAKE) -s targets-by-mcu MCU_TYPE=STM32F7

## test              : run the Betaflight test suite
## junittest         : run the Betaflight test suite, producing Junit XML result files.
## test-representative: run a representative subset of the Betaflight test suite (i.e. run all tests, but run each expanded test only for one target)
## test-all: run the Betaflight test suite including all per-target expanded tests
test junittest test-all test-representative:
	$(V0) cd src/test && $(MAKE) $@

## test_help         : print the help message for the test suite (including a list of the available tests)
test_help:
	$(V0) cd src/test && $(MAKE) help

## test_%            : run test 'test_%' from the test suite
test_%:
	$(V0) cd src/test && $(MAKE) $@


check-target-independence:
	$(V1) for test_target in $(VALID_TARGETS); do \
		FOUND=$$(grep -rE "\W$${test_target}(\W.*)?$$" src/main | grep -vE "(//)|(/\*).*\W$${test_target}(\W.*)?$$" | grep -vE "^src/main/target"); \
		if [ "$${FOUND}" != "" ]; then \
			echo "Target dependencies for target '$${test_target}' found:"; \
			echo "$${FOUND}"; \
			exit 1; \
		fi; \
	done

check-fastram-usage-correctness:
	$(V1) NON_TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM_ZERO_INIT\W.*=.*" src/main/ | grep -Ev "=\s*(false|NULL|0(\.0*f?)?)\s*[,;]"); \
	if [ "$${NON_TRIVIALLY_INITIALIZED}" != "" ]; then \
		echo "Non-trivially initialized FAST_RAM_ZERO_INIT variables found, use FAST_RAM instead:"; \
		echo "$${NON_TRIVIALLY_INITIALIZED}"; \
		exit 1; \
	fi; \
	TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM\W.*;" src/main/ | grep -v "="); \
	EXPLICITLY_TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM\W.*;" src/main/ | grep -E "=\s*(false|NULL|0(\.0*f?)?)\s*[,;]"); \
	if [ "$${TRIVIALLY_INITIALIZED}$${EXPLICITLY_TRIVIALLY_INITIALIZED}" != "" ]; then \
		echo "Trivially initialized FAST_RAM variables found, use FAST_RAM_ZERO_INIT instead to save FLASH:"; \
		echo "$${TRIVIALLY_INITIALIZED}\n$${EXPLICITLY_TRIVIALLY_INITIALIZED}"; \
		exit 1; \
	fi

check-platform-included:
	$(V1) PLATFORM_NOT_INCLUDED=$$(find src/main -type d -name target -prune -o -type f -name \*.c -exec grep -L "^#include \"platform.h\"" {} \;); \
	if [ "$$(echo $${PLATFORM_NOT_INCLUDED} | grep -v -e '^$$' | wc -l)" -ne 0 ]; then \
		echo "The following compilation units do not include the required target specific configuration provided by 'platform.h':"; \
		echo "$${PLATFORM_NOT_INCLUDED}"; \
		exit 1; \
	fi

# rebuild everything when makefile changes
$(TARGET_OBJS): Makefile $(TARGET_DIR)/target.mk $(wildcard make/*)


# include auto-generated dependencies
-include $(TARGET_DEPS)
