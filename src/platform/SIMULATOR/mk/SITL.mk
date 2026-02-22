# SITL Makefile for the simulator platform

# Default output is an exe file
DEFAULT_OUTPUT := exe

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

LIBS        = -lm -lpthread -lc -lrt

# overriden on Apple silicon macosx
LD_FLAGS    := \
            $(LIBS) \
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

ifneq ($(filter macosx-arm% macosx-x86_64%,$(OSFAMILY)-$(ARCHFAMILY)),)

    CFLAGS_DISABLED := -Werror -Wunsafe-loop-optimizations -fuse-linker-plugin

    ifneq ($(filter SITL_STATIC,$(OPTIONS)),)
        $(error Static builds are not supported on MacOS)
    endif

    # This removes the linker script for MacOS apple silicon builds and may cause issues with PG.
    LD_FLAGS := \
            $(filter-out -lrt, $(LIBS)) \
            $(ARCH_FLAGS) \
            $(LTO_FLAGS) \
            $(DEBUG_FLAGS) \
            -Wl,-map,$(TARGET_MAP)
endif
