#
# HPMicro RISC-V MCU family makefile
#

# Auto-hydrate hpm_sdk submodule when building HPMicro targets
PLATFORM_SDK := hpm_sdk
PLATFORM_SDK_STAMP := $(HPM_SDK_STAMP)

# ---- Toolchain ----
# Override default ARM toolchain with the RISC-V toolchain
# (HPM_RISCV_PREFIX is resolved in mk/tools.mk).
ARM_SDK_PREFIX := $(HPM_RISCV_PREFIX)

# ---- Architecture flags ----
# RV32GC with explicit Zicsr/Zifencei extensions and the ILP32D ABI:
# I=Integer, M=Multiply, A=Atomic, F/D=single/double-precision FPU,
# C=Compressed.
ARCH_FLAGS     = -march=rv32gc_zicsr_zifencei -mabi=ilp32d \
                 -mcmodel=medany -msmall-data-limit=8 \
                 -mno-save-restore -fsingle-precision-constant

EXTRA_FLAGS += -g


# ---- Include paths ----
# HPM SDK headers
# If HPM_SDK_BASE is set externally (environment or command line), use it;
# otherwise default to the submodule path. External path validation and the
# hydration stamp are handled in mk/tools.mk.
HPM_SDK_BASE   ?= $(LIB_MODULES_DIR)/hpm_sdk

# Map TARGET_MCU to SoC series directory and default board
ifeq ($(TARGET_MCU),HPM6750)
HPM_SOC_SERIES := HPM6700
HPM_PORT_BOARD := hpmpilotbf67
else ifeq ($(TARGET_MCU),HPM6360)
HPM_SOC_SERIES := HPM6300
HPM_PORT_BOARD := hpmpilotbf63
endif

# Allow CONFIG to override the port board (e.g. CONFIG=SK_FLYCONTROL_BF uses sk_flycontrol_bf board)
ifneq ($(CONFIG),)
# Derive port board name from CONFIG (lowercase) only when the matching
# platform board directory exists; otherwise retain the target default.
HPM_CONFIG_BOARD := $(shell echo $(CONFIG) | tr '[:upper:]' '[:lower:]')
ifneq ($(wildcard $(ROOT)/src/platform/HPMicro/boards/$(HPM_CONFIG_BOARD)),)
HPM_PORT_BOARD := $(HPM_CONFIG_BOARD)
endif
endif

HPM_SOC_DIR    := $(HPM_SDK_BASE)/soc/$(HPM_SOC_SERIES)/$(TARGET_MCU)
HPM_SOC_IP_DIR := $(HPM_SDK_BASE)/soc/$(HPM_SOC_SERIES)/ip
# Boot header source is SoC-specific: soc/<series>/<mcu>/boot/hpm_bootheader.c
HPM_SOC_BOOT_DIR := $(HPM_SOC_DIR)/boot
HPM_BOARD_DIR  := $(ROOT)/src/platform/HPMicro/boards/$(HPM_PORT_BOARD)

# CherryUSB base directory
CHERRYUSB_DIR  := $(HPM_SDK_BASE)/middleware/cherryusb

INCLUDE_DIRS   := $(INCLUDE_DIRS) \
                   $(ROOT)/src/platform/HPMicro/include \
                   $(ROOT)/src/main/drivers \
                   $(ROOT)/src/platform/HPMicro \
                   $(HPM_SOC_DIR) \
                   $(HPM_SOC_IP_DIR) \
                   $(HPM_SOC_BOOT_DIR) \
                   $(HPM_SDK_BASE)/drivers/inc \
                   $(HPM_SDK_BASE)/drivers/src \
                   $(HPM_SDK_BASE)/arch \
                   $(HPM_SDK_BASE)/arch/riscv/intc \
                   $(HPM_SDK_BASE)/arch/riscv/l1c \
                   $(HPM_SDK_BASE)/components \
                   $(HPM_SDK_BASE)/components/spi \
                   $(HPM_SDK_BASE)/components/usb \
                   $(HPM_SDK_BASE)/components/usb/device \
                   $(HPM_SDK_BASE)/components/debug_console \
                   $(CHERRYUSB_DIR) \
                   $(CHERRYUSB_DIR)/core \
                   $(CHERRYUSB_DIR)/common \
                   $(CHERRYUSB_DIR)/class/cdc \
                   $(CHERRYUSB_DIR)/class/msc \
                   $(CHERRYUSB_DIR)/port/hpmicro \
                   $(CHERRYUSB_DIR)/osal \
                   $(CHERRYUSB_DIR)/platform/fatfs \
                   $(HPM_SDK_BASE)/middleware/hpm_sdmmc \
                   $(HPM_SDK_BASE)/middleware/hpm_sdmmc/lib \
                   $(HPM_SDK_BASE)/middleware/hpm_sdmmc/port \
                   $(ROOT)/src/platform/HPMicro \
                   $(ROOT)/src/platform/HPMicro/config \
                   $(ROOT)/src/platform/HPMicro/usb \
                   $(HPM_BOARD_DIR)

VPATH           := $(VPATH):$(HPM_BOARD_DIR):$(ROOT)/src/platform/HPMicro/startup:$(ROOT)/src/platform/HPMicro:$(ROOT)/src/platform/HPMicro/target/$(TARGET_MCU)

# Betaflight common source files needed specifically for HPM
BF_HPM_EXTRA_SRC = \
            drivers/usb_msc_common.c \
            msc/emfat.c \
            msc/emfat_file.c \

# HPM needs the common DShot shared code from the STM32 platform

# HPM6750 does not provide a DAC peripheral/driver, uses the legacy
# pllctl driver instead of pllctlv2, and uses the ADC12 driver instead of ADC16.
ifeq ($(TARGET_MCU),HPM6750)
HPM_DAC_DRV_SRC =
HPM_PLLCTL_DRV_SRC = hpm_pllctl_drv.c
HPM_ADC_DRV_SRC = hpm_adc12_drv.c
else
HPM_DAC_DRV_SRC = hpm_dac_drv.c
HPM_PLLCTL_DRV_SRC = hpm_pllctlv2_drv.c
HPM_ADC_DRV_SRC = hpm_adc16_drv.c
endif

# ---- HPM SDK driver source files ----
# Common drivers from hpm_sdk/drivers/src/
HPM_SDK_DRV_SRC = \
            hpm_gpio_drv.c \
            hpm_gptmr_drv.c \
            hpm_i2c_drv.c \
            hpm_spi_drv.c \
            hpm_uart_drv.c \
            hpm_dma_drv.c \
            $(HPM_ADC_DRV_SRC) \
            hpm_sdxc_drv.c \
            hpm_pwm_drv.c \
            $(HPM_PLLCTL_DRV_SRC) \
            hpm_pmp_drv.c \
            hpm_pcfg_drv.c \
            hpm_enet_drv.c \
            hpm_femc_drv.c \
            hpm_can_drv.c \
            $(HPM_DAC_DRV_SRC) \
            hpm_acmp_drv.c \
            hpm_qei_drv.c \
            hpm_usb_drv.c

# SoC-specific driver sources from hpm_sdk/soc/<series>/<mcu>/
HPM_SDK_SOC_SRC = \
            hpm_clock_drv.c \
            hpm_sysctl_drv.c \
            hpm_otp_drv.c \
            system.c

# SoC-specific boot header from hpm_sdk/soc/<series>/<mcu>/boot/
# (defines .boot_header/.fw_info_table consumed by the linker script)
HPM_SDK_BOOT_SRC = \
            hpm_bootheader.c

# Arch-specific sources from hpm_sdk/arch/riscv/l1c/
HPM_SDK_ARCH_SRC = \
            hpm_l1c_drv.c

# SoC toolchain-specific sources (reset handler, trap handler)
HPM_SDK_TOOLCHAIN_SRC = \
            reset.c \
            trap.c

# SDMMC middleware sources (from hpm_sdk/middleware/hpm_sdmmc/lib/)
HPM_SDK_SDMMC_SRC = \
            hpm_sdmmc_common.c \
            hpm_sdmmc_emmc.c \
            hpm_sdmmc_host.c \
            hpm_sdmmc_osal.c \
            hpm_sdmmc_port.c \
            hpm_sdmmc_sd.c \
            hpm_sdmmc_sdio.c

# HPM SPI component (from hpm_sdk/components/spi/)
HPM_COMPONENT_SPI_SRC = \
            hpm_spi.c

# HPM USB device component (from hpm_sdk/components/usb/device/)
HPM_COMPONENT_USB_SRC = \
            hpm_usb_device.c

# HPM debug console component (from hpm_sdk/components/debug_console/)
HPM_COMPONENT_DBG_SRC = \
            hpm_debug_console.c

# Additional HPM SDK driver sources (that exist in drivers/src/)
HPM_SDK_DRV_EXTRA_SRC = \
            hpm_mchtmr_drv.c

# CherryUSB core/class sources (from hpm_sdk/middleware/cherryusb/)
CHERRYUSB_SRC = \
            usbd_core.c \
            usbd_cdc_acm.c \
            usbd_msc.c \
            usb_dc_hpm.c \
            usb_glue_hpm.c

# Keep Betaflight sources under the project's normal warning policy.  The
# vendor SDK and CherryUSB are imported sources and need a small compatibility
# allowance for GCC extensions and warnings emitted by their pinned version.
HPM_SDK_WARNING_FLAGS = -Wno-pedantic -Wno-error=format -Wno-error=incompatible-pointer-types -Wno-error=sign-compare -Wno-error=override-init -Wno-error=overflow -Wno-error=aggressive-loop-optimizations -Wno-error=double-promotion -Wno-error=return-type -Wno-missing-braces -Wno-error=type-limits -Wno-error=unused-parameter
HPM_SDK_ALL_SRC = $(HPM_SDK_DRV_SRC) $(HPM_SDK_DRV_EXTRA_SRC) $(HPM_SDK_SOC_SRC) $(HPM_SDK_BOOT_SRC) $(HPM_SDK_ARCH_SRC) $(HPM_SDK_TOOLCHAIN_SRC) $(HPM_SDK_SDMMC_SRC) $(HPM_COMPONENT_SPI_SRC) $(HPM_COMPONENT_USB_SRC) $(HPM_COMPONENT_DBG_SRC) $(CHERRYUSB_SRC)
$(foreach src,$(HPM_SDK_ALL_SRC),$(eval SRC_CFLAGS_$(src) += $(HPM_SDK_WARNING_FLAGS)))

# ---- Device compile definitions (SDK config) ----
# These mirror the CMakeLists.txt CONFIG_* settings
# NOTE: TARGET_FLASH_SIZE is defined in Makefile line 210
# NOTE: __FPU_PRESENT is defined in platform_mcu.h (detected from __riscv_flen)
DEVICE_FLAGS_HPM := -DFLASH_XIP \
                    -DDEFIO_PORT_PINS=32 \
                    -DCONFIG_FATFS=1 \
                    -DCONFIG_HPM_SPI=1 \
                    -DCONFIG_CHERRYUSB=1 \
                    -DCONFIG_USB_DEVICE=1 \
                    -DCONFIG_USB_DEVICE_CDC_ACM=1 \
                    -DCONFIG_SDMMC=1 \
                    -DCONFIG_USB_DEVICE_MSC=1 \
                    -DDISABLE_IRQ_PREEMPTIVE=0 \
                    -DHPM_SDMMC_HOST_ENABLE_IRQ=1 \
                    -DHPM_SPI_DRV_DEFAULT_RETRY_COUNT=50000 \
                    -DHPMSOC_HAS_HPMSDK_PWM \
                    -DHPMSOC_HAS_HPMSDK_DMA \
                    -DUSE_USB_MSC \
                    -DUSE_VCP

# ---- Chip-specific configuration ----
ifeq ($(TARGET_MCU),HPM6750)
DEVICE_FLAGS    = -DHPM6750 $(DEVICE_FLAGS_HPM)
HPM_LD_SCRIPT_BASE := hpm_flash_hpm6750
STARTUP_SRC     = start.S
MCU_FLASH_SIZE  := 4096

else ifeq ($(TARGET_MCU),HPM6360)
DEVICE_FLAGS    = -DHPM6360 $(DEVICE_FLAGS_HPM)
HPM_LD_SCRIPT_BASE := hpm_flash_hpm6360
STARTUP_SRC     = start.S
MCU_FLASH_SIZE  := 4096

else
$(error Unknown MCU for HPMicro target: $(TARGET_MCU))
endif

ifeq ($(strip $(DEBUG)),)
HPM_USE_ILM_VARIANT := yes
LD_SCRIPT := $(LINKER_DIR)/$(HPM_LD_SCRIPT_BASE)_ilm.ld
HPM_ILM_CODE_SCRIPT := $(LINKER_DIR)/hpm_ilm_code.ld
HPM_BUILD_NAME := $(TARGET)$(if $(CONFIG),_$(CONFIG))$(if $(filter yes,$(REV)),_$(REVISION))
HPM_ILM_GENERATED_DIR := $(OBJECT_DIR)/$(HPM_BUILD_NAME)
HPM_ILM_EXCLUDE_SCRIPT := $(HPM_ILM_GENERATED_DIR)/hpm_ilm_exclude.ld
LD_SCRIPTS += $(HPM_ILM_CODE_SCRIPT) $(HPM_ILM_EXCLUDE_SCRIPT)
HPM_ILM_LD_SEARCH_FLAG := -Wl,-L$(HPM_ILM_GENERATED_DIR)
else
HPM_USE_ILM_VARIANT := no
LD_SCRIPT := $(LINKER_DIR)/$(HPM_LD_SCRIPT_BASE).ld
endif

DEVICE_FLAGS   += -DHPMicro

# ---- Linker flags ----
# Override ARM defaults: RISC-V uses nosys.specs instead of nano.specs,
# and removes ARM-specific --no-wchar-size-warning
# Keep LD_FLAGS recursive so it resolves TARGET_MAP after the common Makefile
# finalizes it (otherwise the linker writes betaflight_.map instead of
# matching the final ELF filename).
LD_FLAGS        = -lm \
                   -nostartfiles \
                   --specs=nosys.specs \
                   -lc \
                   -lnosys \
                   $(ARCH_FLAGS) \
                   $(LTO_FLAGS) \
                   $(DEBUG_FLAGS) \
                   -static \
                   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
                   -Wl,-L$(LINKER_DIR) \
                   $(HPM_ILM_LD_SEARCH_FLAG) \
                   -Wl,--cref \
                   -Wl,--print-memory-usage \
                   -T$(LD_SCRIPT) \
                    $(EXTRA_LD_FLAGS)

# ---- Exclude ARM/STM32-specific sources from common lists ----
# These files use STM32 HAL / CMSIS headers not available on RISC-V.
# HPM equivalents are provided in the hpmicro driver directory.
MCU_EXCLUDES = \
            drivers/bus_i2c_hal.c \
            bus_bst_stm32f30x.c \
            drivers/bus_i2c_hal_init.c \
            drivers/bus_spi_ll.c \
            drivers/exti.c \
            drivers/rcc.c \
            drivers/serial_escserial.c \
            drivers/serial_usb_vcp.c \
            drivers/timer.c \
            i2c_bst.c \
            fc/hardfaults.c

# ---- MCU driver source files ----
# hpmicro drivers + board files + HPM SDK driver sources
# SDK driver .c files are in hpm_sdk/drivers/src/ (added to VPATH below)
MCU_COMMON_SRC = \
            $(BF_HPM_EXTRA_SRC) \
            $(HPM_SDK_DRV_SRC) \
            $(HPM_SDK_DRV_EXTRA_SRC) \
            $(HPM_SDK_SOC_SRC) \
            $(HPM_SDK_BOOT_SRC) \
            $(HPM_SDK_ARCH_SRC) \
            $(HPM_SDK_TOOLCHAIN_SRC) \
            $(HPM_SDK_SDMMC_SRC) \
            $(HPM_COMPONENT_SPI_SRC) \
            $(HPM_COMPONENT_USB_SRC) \
            $(HPM_COMPONENT_DBG_SRC) \
            $(CHERRYUSB_SRC) \
            adc_hpmicro.c \
            drivers/inverter.c \
            drivers/serial_pinconfig.c \
            drivers/adc.c \
            bus_i2c_hpmicro.c \
            bus_spi_pinconfig.c \
            bus_spi_hpmicro.c \
            pwm_output_hpmicro.c \
            pwm_output_dshot.c \
            pwm_output_dshot_shared.c \
            dshot_dpwm.c \
            trgm_dshot_resource.c \
            timer_hw_ext.c \
            drivers/bus_spi_config.c \
            usb/cdc_acm.c \
            dma_hpmicro.c \
            dma_reqmap_mcu.c \
            exti_hpmicro.c \
            io_hpmicro.c \
            usb/msc_sdcard.c \
            usb/usb_descriptor_hpm.c \
            persistent.c \
            sdio_hpmicro.c \
            serial_uart_hpmicro.c \
            usb/serial_usb_vcp.c \
            system_hpmicro.c \
            config_flash.c \
            timer_hpmicro.c \
            usb/usb_msc_hpm.c \

# SoC-specific timer tables and extension data.
ifeq ($(TARGET_MCU),HPM6750)
MCU_COMMON_SRC := $(MCU_COMMON_SRC) \
            timer_hpm6750.c
else ifeq ($(TARGET_MCU),HPM6360)
MCU_COMMON_SRC := $(MCU_COMMON_SRC) \
            timer_hpm6360.c
endif

ifeq ($(HPM_USE_ILM_VARIANT),yes)
# hpm_ilm_code.ld is the single source of truth for ILM placement.  Extract its
# exact object selectors and map them back to source files, avoiding a second
# list here that could drift when hot-path sources are added or renamed.
HPM_ILM_OBJECT_PATTERNS := $(shell sed -n 's/^[[:space:]]*\([^[:space:]]*\.o\)(\.text \.text\.\*)[[:space:]]*$$/\1/p' $(HPM_ILM_CODE_SCRIPT))
HPM_ILM_OBJECT_SUFFIXES := $(subst */,,$(HPM_ILM_OBJECT_PATTERNS))
HPM_ILM_ROOT_OBJECTS := $(foreach obj,$(HPM_ILM_OBJECT_SUFFIXES),$(if $(findstring /,$(obj)),,$(obj)))

HPM_PLATFORM_C_SRC := $(shell find $(TARGET_PLATFORM_DIR) -name '*.c')
# Normalize each main source against SRC_DIR so absolute and relative CSOURCES
# entries produce the same selector-relative object path.
HPM_MAIN_OBJECT_FROM_SOURCE = $(patsubst $(abspath $(SRC_DIR))/%.c,%.o,$(abspath $(1)))
HPM_ILM_MAIN_SRC := $(foreach src,$(CSOURCES),$(if $(filter $(call HPM_MAIN_OBJECT_FROM_SOURCE,$(src)),$(HPM_ILM_OBJECT_SUFFIXES)),$(src)))
HPM_ILM_PLATFORM_SRC := $(foreach src,$(HPM_PLATFORM_C_SRC),$(if $(filter $(patsubst %.c,%.o,$(notdir $(src))),$(HPM_ILM_ROOT_OBJECTS)),$(src)))
HPM_NOT_OPTIMISED_SRC := $(HPM_ILM_PLATFORM_SRC) $(HPM_ILM_MAIN_SRC)
HPM_ILM_MAPPED_OBJECTS := $(foreach src,$(HPM_ILM_PLATFORM_SRC),$(patsubst %.c,%.o,$(notdir $(src)))) \
                          $(foreach src,$(HPM_ILM_MAIN_SRC),$(call HPM_MAIN_OBJECT_FROM_SOURCE,$(src)))
HPM_ILM_UNMAPPED_OBJECTS := $(filter-out $(HPM_ILM_MAPPED_OBJECTS),$(HPM_ILM_OBJECT_SUFFIXES))
ifneq ($(HPM_ILM_UNMAPPED_OBJECTS),)
$(error ILM object selectors do not map to HPMicro or Betaflight sources: $(HPM_ILM_UNMAPPED_OBJECTS))
endif

NOT_OPTIMISED_SRC := $(NOT_OPTIMISED_SRC) $(HPM_NOT_OPTIMISED_SRC)

# NOT_OPTIMISED_SRC omits the normal optimization flags (including LTO), while
# these sources still need -O2 for execution from ILM.  Derive the per-file
# overrides from the same list so source additions and renames stay in sync.
$(foreach src,$(HPM_NOT_OPTIMISED_SRC),$(eval SRC_CFLAGS_$(notdir $(src)) = -O2))

# The flash .text section must exclude exactly the same objects consumed by
# hpm_ilm_code.ld.  Generate both EXCLUDE_FILE clauses from that canonical list.
$(HPM_ILM_EXCLUDE_SCRIPT): $(HPM_ILM_CODE_SCRIPT) $(TARGET_PLATFORM_DIR)/mk/HPMicro.mk
	$(V1) mkdir -p $(dir $@)
	$(V1) printf '%s\n' \
		'*(EXCLUDE_FILE ($(HPM_ILM_OBJECT_PATTERNS)) .text)' \
		'*(EXCLUDE_FILE ($(HPM_ILM_OBJECT_PATTERNS)) .text*)' > $@
endif

# Add SDK driver src, SoC, and arch directories to VPATH
VPATH           := $(VPATH):$(HPM_SDK_BASE)/drivers/src
VPATH           := $(VPATH):$(HPM_SOC_DIR)
VPATH           := $(VPATH):$(HPM_SOC_BOOT_DIR)
VPATH           := $(VPATH):$(HPM_SOC_DIR)/toolchains:$(HPM_SOC_DIR)/toolchains/gcc
VPATH           := $(VPATH):$(HPM_SDK_BASE)/arch/riscv/l1c
VPATH           := $(VPATH):$(HPM_SDK_BASE)/middleware/hpm_sdmmc/lib
VPATH           := $(VPATH):$(HPM_SDK_BASE)/middleware/hpm_sdmmc/port
VPATH           := $(VPATH):$(HPM_SDK_BASE)/components/spi
VPATH           := $(VPATH):$(HPM_SDK_BASE)/components/usb/device
VPATH           := $(VPATH):$(HPM_SDK_BASE)/components/debug_console
VPATH           := $(VPATH):$(CHERRYUSB_DIR)/core
VPATH           := $(VPATH):$(CHERRYUSB_DIR)/class/cdc
VPATH           := $(VPATH):$(CHERRYUSB_DIR)/class/msc
VPATH           := $(VPATH):$(CHERRYUSB_DIR)/port/hpmicro

# GCC diagnoses the legacy compass alignment enum/uint8_t API mismatch on
# RISC-V.  Keep this exception limited to the one affected common source.
SRC_CFLAGS_compass.c += -Wno-error=incompatible-pointer-types

# Imported Open Location Code sources use double literals with float inputs.
SRC_CFLAGS_olc.c += -Wno-error=double-promotion
