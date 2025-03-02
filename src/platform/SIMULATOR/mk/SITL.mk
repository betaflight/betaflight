
INCLUDE_DIRS := \
        $(INCLUDE_DIRS) \
        $(TARGET_PLATFORM_DIR) \
        $(TARGET_PLATFORM_DIR)/include \
        $(LIB_MAIN_DIR)/dyad

MCU_COMMON_SRC  := \
        $(LIB_MAIN_DIR)/dyad/dyad.c \
        SIMULATOR/sitl.c \
        SIMULATOR/udplink.c

#Flags
ARCH_FLAGS      =
DEVICE_FLAGS    =
LD_SCRIPT       = $(LINKER_DIR)/sitl.ld
STARTUP_SRC     =

MCU_FLASH_SIZE  := 2048

ARM_SDK_PREFIX  =

MCU_EXCLUDES = \
        drivers/rx/rx_xn297.c \
        drivers/display_ug2864hsweg01.c \
        telemetry/crsf.c \
        telemetry/ghst.c \
        telemetry/srxl.c \
        io/displayport_oled.c

TARGET_MAP  = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

LD_FLAGS    := \
            -lm \
            -lpthread \
            -lc \
            -lrt \
            $(ARCH_FLAGS) \
            $(LTO_FLAGS) \
            $(DEBUG_FLAGS) \
            -Wl,-gc-sections,-Map,$(TARGET_MAP) \
            -Wl,-L$(LINKER_DIR) \
            -Wl,--cref \
            -T$(LD_SCRIPT)

ifneq ($(filter SITL_STATIC,$(OPTIONS)),)
LD_FLAGS     += \
              -static \
              -static-libgcc
endif

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Ofast
OPTIMISE_SPEED      := -Ofast
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif

ifeq ($(OSFAMILY),macosx)
  ifneq ($(findstring arm,$(ARCHFAMILY)),)
    override OPTIMISATION_BASE := $(filter-out -fuse-linker-plugin,$(OPTIMISATION_BASE))
    override WARNING_FLAGS := $(filter-out -Werror -Wunsafe-loop-optimizations,$(WARNING_FLAGS))
    override LD_FLAGS := \
            -lm \
            -lpthread \
            -lc \
            $(ARCH_FLAGS) \
            $(LTO_FLAGS) \
            $(DEBUG_FLAGS) \
            -Wl,-map,$(TARGET_MAP)
    OBJCOPY_EXISTS := $(shell command -v $(OBJCOPY) 2>/dev/null)
    ifeq ($(OBJCOPY_EXISTS),)
      BREW_LLVM_PREFIX := $(shell brew --prefix llvm 2>/dev/null)
      ifneq ($(BREW_LLVM_PREFIX),)
        override OBJCOPY := $(BREW_LLVM_PREFIX)/bin/llvm-objcopy
      else
        $(error OBJCOPY is not found, please install llvm and set PATH or install llvm through homebrew)
      endif
    endif
  endif
endif
