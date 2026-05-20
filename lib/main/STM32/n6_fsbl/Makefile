# STM32N6 FSBL stub — standalone build. Not invoked by betaflight's main
# build; the signed blob in prebuilt/ is what gets bundled into the combined
# image.
#
# Usage:
#   make             # produce n6_fsbl.elf and n6_fsbl.bin
#   make signed      # also re-sign the .bin into prebuilt/
#   make combined    # rebuild prebuilt/combined.bin = signed FSBL padded
#                    # to 1 MiB + RAM_ONLY betaflight .bin (read by the
#                    # FSBL stub from 0x70100000 and shadow-copied into
#                    # AXISRAM1 at 0x24000000)
#   make clean
#
# `combined` requires CONFIG and CONFIG_DIR for the betaflight build:
#   make combined CONFIG=STM32N657DK_DSK320 CONFIG_DIR=$$HOME/src/config

ROOT          := $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/../../../..)
TOOLCHAIN_BIN := $(ROOT)/tools/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin
PREFIX        := $(if $(wildcard $(TOOLCHAIN_BIN)/arm-none-eabi-gcc),$(TOOLCHAIN_BIN)/,)arm-none-eabi-

CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
LD      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

HAL_DIR  := $(ROOT)/lib/main/STM32N6/Drivers/STM32N6xx_HAL_Driver
CMSIS_ST := $(ROOT)/lib/main/STM32N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include
CMSIS    := $(ROOT)/lib/main/STM32N6/Drivers/CMSIS/Include

INCLUDES := \
    -I. \
    -I$(HAL_DIR)/Inc \
    -I$(CMSIS_ST) \
    -I$(CMSIS)

# Match the betaflight main build: no -mcmse and no CPU_IN_SECURE_STATE, so
# peripheral macros resolve to the NS (0x4xxxxxxx) aliases that the boot
# ROM's RIFSC defaults allow. Defining CPU_IN_SECURE_STATE pulls in
# HAL_MPU_*_NS variants that need -mcmse.
DEFINES := \
    -DSTM32N657xx \
    -DUSE_HAL_DRIVER \
    -DHSE_VALUE=48000000

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

LDSCRIPT := STM32N657XX_FSBL.ld
LDFLAGS  := \
    $(ARCH_FLAGS) \
    -T$(LDSCRIPT) \
    -nostartfiles \
    -specs=nano.specs -specs=nosys.specs \
    -Wl,--gc-sections \
    -Wl,-Map=n6_fsbl.map \
    -Wl,--print-memory-usage

LOCAL_C_SRCS := \
    main.c \
    stm32n6xx_hal_msp.c \
    stm32n6xx_it.c \
    system_stm32n6xx_fsbl.c

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
    stm32n6xx_hal_dma_ex.c

LOCAL_OBJS := $(LOCAL_C_SRCS:.c=.o) startup_stm32n657xx_fsbl.o
HAL_OBJS   := $(HAL_C_SRCS:.c=.hal.o)
OBJS       := $(LOCAL_OBJS) $(HAL_OBJS)

ELF := n6_fsbl.elf
BIN := n6_fsbl.bin

# Override SIGNING_TOOL on the make command line if STM32CubeProgrammer's bin
# directory isn't on $PATH, e.g.:
#   make signed SIGNING_TOOL=/path/to/STM32_SigningTool_CLI
SIGNING_TOOL ?= STM32_SigningTool_CLI
SIGNED_BLOB  := prebuilt/n6_fsbl_signed.stm32

# Combined-image inputs. The signed FSBL stub + XIP-linked betaflight
# image (linked at 0x70100000, runs in-place from XSPI). The stub brings
# XSPI memory-mapped + jumps to 0x70100000 without copying or deinit'ing
# the peripheral.
CONFIG       ?= STM32N657DK_DSK320
COMBINED     := prebuilt/combined.bin
FSBL_SLOT    := 1048576    # 1 MiB — XSPI offset where the BF XIP slot starts
BF_OBJ       := $(ROOT)/obj
BF_BIN       := $(BF_OBJ)/betaflight_2026.6.0-alpha_STM32N657_$(CONFIG).bin
BF_HEX       := $(BF_OBJ)/betaflight_2026.6.0-alpha_STM32N657_$(CONFIG).hex

.PHONY: all signed combined bf-ram bf-ram-elf clean
all: $(BIN)

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	$(SIZE) $<

$(ELF): $(OBJS) $(LDSCRIPT)
	$(LD) $(LDFLAGS) $(OBJS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(AS) $(ASFLAGS) -c $< -o $@

%.hal.o: $(HAL_DIR)/Src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Re-sign and overwrite the committed blob. Run only after verifying the
# rebuilt stub on hardware. -t ssbl produces Binary type 0x0 in the v2.3
# header, which is what the N6 boot ROM accepts; -t fsbl produces type
# 0x10 and is rejected.
signed: $(SIGNED_BLOB)
$(SIGNED_BLOB): $(BIN) | prebuilt
	$(SIGNING_TOOL) -bin $< -nk -of 0x80000000 -t ssbl -hv 2.3 -align -o $@ -s

prebuilt:
	mkdir -p $@

# Build the betaflight XIP ELF/BIN via the main build. bf-xip-elf is phony
# so the recursive make is always invoked (it handles its own incremental
# dependency tracking).
bf-xip: $(BF_BIN)

bf-xip-elf:
	$(MAKE) -C $(ROOT) CONFIG_DIR=$(CONFIG_DIR) CONFIG=$(CONFIG) XIP=1 binary

$(BF_BIN): bf-xip-elf

# Combined flash image: signed FSBL padded to FSBL_SLOT, then the XIP
# betaflight image at offset 0x100000. cp --no-preserve=mode strips the
# read-only bit on the signed source so truncate can grow it.
combined: $(COMBINED)
$(COMBINED): $(SIGNED_BLOB) $(BF_BIN) | prebuilt
	cp --no-preserve=mode $(SIGNED_BLOB) $@.tmp
	truncate -s $(FSBL_SLOT) $@.tmp
	cat $(BF_BIN) >> $@.tmp
	mv $@.tmp $@

# BF-as-FSBL debug variant: build the entire betaflight image with
# BF_AS_FSBL=1 (link via STM32N657XX_FSBL_FULL.ld), sign the resulting .bin
# with the same SSBL recipe used for the stub, and write it directly to
# nor0 0x0 via the OpenBootloader-staged TSV. Replaces the
# stub→XSPI-memory-map→memcpy→jump handoff for bring-up debug.
BF_FSBL_BIN     := $(BF_OBJ)/../betaflight_2026.6.0-alpha_STM32N657_$(CONFIG).bin
BF_FSBL_SIGNED  := prebuilt/bf_as_fsbl.stm32
BF_FSBL_TSV     := FlashLayout_BF_AS_FSBL.tsv

bf-as-fsbl: $(BF_FSBL_SIGNED)

bf-as-fsbl-elf:
	$(MAKE) -C $(ROOT) CONFIG=$(CONFIG) BF_AS_FSBL=1 binary

$(BF_FSBL_BIN): bf-as-fsbl-elf

$(BF_FSBL_SIGNED): $(BF_FSBL_BIN) | prebuilt
	$(SIGNING_TOOL) -bin $< -nk -of 0x80000000 -t ssbl -hv 2.3 -align -o $@ -s

bf-as-fsbl-flash: $(BF_FSBL_SIGNED)
	STM32_Programmer_CLI -c port=USB1 mode=HotPlug -d $(BF_FSBL_TSV)

.PHONY: bf-as-fsbl bf-as-fsbl-elf bf-as-fsbl-flash

clean:
	rm -f $(OBJS) $(ELF) $(BIN) n6_fsbl.map
