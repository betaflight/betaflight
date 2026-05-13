# Betaflight N6 OpenBootloader build.
#
# Architecture:
#   - Skeleton sources here: main.c (boot decision), system_stm32n6xx_obl.c,
#     stm32n6xx_hal_msp.c, stm32n6xx_it.c, startup, linker, flash driver.
#   - The OBL DFU stack (CubeN6 Middlewares/ST/OpenBootloader + USB Device
#     Library + the Application/OpenBootloader/USB_Device glue) is pulled
#     in via VPATH from the submodule, NOT modified or copied — so re-
#     pulling CubeN6 keeps us up to date. Currently disabled (dfu_stub.c
#     stands in) until task #6 wires it up.
#
# Usage:
#   make                     produce obl.elf and obl.bin (skeleton only)
#   make signed              re-sign into prebuilt/obl_signed.stm32
#   make OBL_FLASH_DRIVER=mx66uw1g45g  pick driver source (default)
#   make clean

ROOT          := $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/../../../..)
TOOLCHAIN_BIN := $(ROOT)/tools/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin
PREFIX        := $(if $(wildcard $(TOOLCHAIN_BIN)/arm-none-eabi-gcc),$(TOOLCHAIN_BIN)/,)arm-none-eabi-

CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
LD      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

OBL_FLASH_DRIVER ?= mx66uw1g45g

HAL_DIR     := $(ROOT)/lib/main/STM32N6/Drivers/STM32N6xx_HAL_Driver
CMSIS_ST    := $(ROOT)/lib/main/STM32N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include
CMSIS       := $(ROOT)/lib/main/STM32N6/Drivers/CMSIS/Include
OBL_MW      := $(ROOT)/lib/main/STM32N6/Middlewares/ST/OpenBootloader
OBL_USBLIB  := $(OBL_MW)/STM32_USB_Device_Library
OBL_PROJ    := $(ROOT)/lib/main/STM32N6/Projects/STM32N6570-DK/Applications/OpenBootloader

INCLUDES := \
    -I. \
    -Iflash \
    -I$(ROOT)/src/platform/STM32/include \
    -I$(HAL_DIR)/Inc \
    -I$(CMSIS_ST) \
    -I$(CMSIS) \
    -I$(OBL_MW)/Core \
    -I$(OBL_MW)/Util \
    -I$(OBL_MW)/Modules/Mem \
    -I$(OBL_MW)/Modules/USB \
    -I$(OBL_MW)/Modules/USART \
    -I$(OBL_USBLIB)/Core/Inc \
    -I$(OBL_USBLIB)/Class/DFU/Inc \
    -I$(OBL_PROJ)/Core/Inc \
    -I$(OBL_PROJ)/OpenBootloader/App \
    -I$(OBL_PROJ)/OpenBootloader/Target \
    -I$(OBL_PROJ)/USB_Device/App \
    -I$(OBL_PROJ)/USB_Device/Target

# Match betaflight main build: NS aliases (no -mcmse, no CPU_IN_SECURE_STATE).
DEFINES := \
    -DSTM32N657xx \
    -DUSE_HAL_DRIVER \
    -DHSE_VALUE=48000000

# Bring-up override: make OBL_FORCE_DFU=1 to build an OBL that always
# enters DFU regardless of nor0 contents.
ifdef OBL_FORCE_DFU
DEFINES += -DOBL_FORCE_DFU
endif

ARCH_FLAGS := -mthumb -mcpu=cortex-m55 -mfloat-abi=hard -mfpu=fpv5-d16

CFLAGS := \
    $(ARCH_FLAGS) \
    -Os -g3 \
    -ffunction-sections -fdata-sections \
    -Wall -Wextra \
    -Wno-unused-parameter \
    -std=gnu11 \
    $(INCLUDES) $(DEFINES)

ASFLAGS := $(ARCH_FLAGS) -g3 $(INCLUDES) $(DEFINES)

LDSCRIPT := STM32N657XX_OBL.ld
LDFLAGS  := \
    $(ARCH_FLAGS) \
    -T$(LDSCRIPT) \
    -nostartfiles \
    -specs=nano.specs -specs=nosys.specs \
    -Wl,--gc-sections \
    -Wl,-Map=obl.map \
    -Wl,--print-memory-usage

LOCAL_C_SRCS := \
    main.c \
    obl_app.c \
    app_openbootloader.c \
    stm32n6xx_hal_msp.c \
    stm32n6xx_it.c \
    system_stm32n6xx_obl.c \
    external_memory_interface.c \
    otp_interface.c \
    usbd_conf.c \
    usbd_desc.c \
    usbd_dfu.c \
    usbd_dfu_if.c \
    flash/$(OBL_FLASH_DRIVER).c

HAL_C_SRCS := \
    stm32n6xx_hal.c \
    stm32n6xx_hal_cortex.c \
    stm32n6xx_hal_rcc.c \
    stm32n6xx_hal_rcc_ex.c \
    stm32n6xx_hal_pwr.c \
    stm32n6xx_hal_pwr_ex.c \
    stm32n6xx_hal_gpio.c \
    stm32n6xx_hal_xspi.c \
    stm32n6xx_hal_dma.c \
    stm32n6xx_hal_dma_ex.c \
    stm32n6xx_hal_iwdg.c \
    stm32n6xx_hal_pcd.c \
    stm32n6xx_hal_pcd_ex.c \
    stm32n6xx_ll_usb.c

# OBL middleware (protocol-agnostic core, memory mgr, USB transport bindings,
# utilities). Pulled in unchanged from the CubeN6 submodule.
OBL_MW_C_SRCS := \
    Core/openbl_core.c \
    Util/openbl_util.c \
    Modules/Mem/openbl_mem.c \
    Modules/USB/openbl_usb_cmd.c

# USB Device Library (DFU class + core). Note: usbd_dfu.c is overridden
# locally (see LOCAL_C_SRCS) — the Betaflight copy patches the per-alt
# iInterface descriptor lookup so all alts use DfuInterface->pStrDesc.
OBL_USB_C_SRCS := \
    Core/Src/usbd_core.c \
    Core/Src/usbd_ctlreq.c \
    Core/Src/usbd_ioreq.c

# Application-level glue from the reference OBL project. We provide our
# own external_memory_interface.c (above), so that submodule file is NOT
# listed here.
OBL_APP_C_SRCS := \
    OpenBootloader/Target/common_interface.c \
    OpenBootloader/Target/iwdg_interface.c \
    OpenBootloader/Target/ram_interface.c \
    OpenBootloader/Target/usb_interface.c \
    USB_Device/App/usb_device.c

LOCAL_OBJS := \
    main.o \
    obl_app.o \
    app_openbootloader.o \
    stm32n6xx_hal_msp.o \
    stm32n6xx_it.o \
    system_stm32n6xx_obl.o \
    external_memory_interface.o \
    otp_interface.o \
    usbd_conf.o \
    usbd_desc.o \
    usbd_dfu.o \
    usbd_dfu_if.o \
    flash/$(OBL_FLASH_DRIVER).o \
    startup_stm32n657xx_obl.o
HAL_OBJS    := $(HAL_C_SRCS:.c=.hal.o)
OBL_MW_OBJS := $(notdir $(OBL_MW_C_SRCS:.c=.mw.o))
OBL_USB_OBJS:= $(notdir $(OBL_USB_C_SRCS:.c=.usbd.o))
OBL_APP_OBJS:= $(notdir $(OBL_APP_C_SRCS:.c=.app.o))
OBJS        := $(LOCAL_OBJS) $(HAL_OBJS) $(OBL_MW_OBJS) $(OBL_USB_OBJS) $(OBL_APP_OBJS)

ELF := obl.elf
BIN := obl.bin

SIGNING_TOOL ?= STM32_SigningTool_CLI
SIGNED_BLOB  := prebuilt/obl_$(OBL_FLASH_DRIVER)_signed.stm32

# Boot ROM @FSBL DFU alt cap empirically ~485 KiB. Build fails loudly if
# the OBL output exceeds the safe budget.
OBL_SIZE_LIMIT := 491520     # 480 KiB, matches linker LENGTH

.PHONY: all signed size_check clean
all: $(BIN) size_check

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	$(SIZE) $<

$(ELF): $(OBJS) $(LDSCRIPT)
	$(LD) $(LDFLAGS) $(OBJS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(AS) $(ASFLAGS) -c $< -o $@

flash/%.o: flash/%.c
	$(CC) $(CFLAGS) -c $< -o $@

%.hal.o: $(HAL_DIR)/Src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Pattern rules for the OBL middleware / USB Device Library / application
# glue. Each rule strips the suffix used to disambiguate object files
# (.mw.o, .usbd.o, .app.o) and finds the .c by leaf-name search across
# the upstream subdirectories. notdir + foreach keeps the rule chain
# manageable without listing every nested directory in VPATH.

OBL_MW_DIRS := \
    $(OBL_MW)/Core \
    $(OBL_MW)/Util \
    $(OBL_MW)/Modules/Mem \
    $(OBL_MW)/Modules/USB \
    $(OBL_MW)/Modules/USART

OBL_USB_DIRS := \
    $(OBL_USBLIB)/Core/Src \
    $(OBL_USBLIB)/Class/DFU/Src

OBL_APP_DIRS := \
    $(OBL_PROJ)/OpenBootloader/App \
    $(OBL_PROJ)/OpenBootloader/Target \
    $(OBL_PROJ)/USB_Device/App \
    $(OBL_PROJ)/USB_Device/Target

vpath %.c $(OBL_MW_DIRS) $(OBL_USB_DIRS) $(OBL_APP_DIRS)

%.mw.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.usbd.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.app.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Hard size cap — keeps OBL fit-for-@FSBL-DFU even as middleware grows.
size_check: $(BIN)
	@bin_size=$$(stat -c %s $(BIN)); \
	if [ $$bin_size -gt $(OBL_SIZE_LIMIT) ]; then \
	    echo "ERROR: $(BIN) is $$bin_size bytes, exceeds $(OBL_SIZE_LIMIT) byte cap (boot ROM @FSBL alt limit)"; \
	    exit 1; \
	else \
	    echo "OBL size OK: $$bin_size / $(OBL_SIZE_LIMIT) bytes"; \
	fi

signed: $(SIGNED_BLOB)
$(SIGNED_BLOB): $(BIN) | prebuilt
	$(SIGNING_TOOL) -bin $< -nk -of 0x80000000 -t ssbl -hv 2.3 -align -o $@ -s

prebuilt:
	mkdir -p $@

clean:
	rm -f $(LOCAL_OBJS) $(HAL_OBJS) $(ELF) $(BIN) obl.map
