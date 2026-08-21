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
LD_SCRIPT       = $(LINKER_DIR)/hpm_flash_hpm6750_ilm.ld
STARTUP_SRC     = start.S
MCU_FLASH_SIZE  := 4096

else ifeq ($(TARGET_MCU),HPM6360)
DEVICE_FLAGS    = -DHPM6360 $(DEVICE_FLAGS_HPM)
LD_SCRIPT       = $(LINKER_DIR)/hpm_flash_hpm6360_ilm.ld
STARTUP_SRC     = start.S
MCU_FLASH_SIZE  := 4096

else
$(error Unknown MCU for HPMicro target: $(TARGET_MCU))
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

# HPM DShot driver files need to be compiled without LTO to preserve
# symbol visibility for vtable function pointers used by the common DShot framework.
NOT_OPTIMISED_SRC := $(NOT_OPTIMISED_SRC) \
    ./src/platform/HPMicro/io_hpmicro.c \
    ./src/platform/HPMicro/system_hpmicro.c \
    ./src/platform/HPMicro/timer_hpmicro.c \
    ./src/platform/HPMicro/timer_hw_ext.c \
    ./src/platform/HPMicro/dma_hpmicro.c \
    ./src/platform/HPMicro/bus_spi_hpmicro.c \
    ./src/platform/HPMicro/exti_hpmicro.c \
    ./src/platform/HPMicro/adc_hpmicro.c \
    ./src/platform/HPMicro/pwm_output_hpmicro.c \
    ./src/platform/HPMicro/pwm_output_dshot.c \
    ./src/platform/HPMicro/pwm_output_dshot_shared.c \
    ./src/platform/HPMicro/dshot_dpwm.c \
    ./src/platform/HPMicro/trgm_dshot_resource.c \
    ./src/platform/HPMicro/dma_reqmap_mcu.c \
    ./src/platform/HPMicro/serial_uart_hpmicro.c \
    ./src/platform/HPMicro/target/HPM6360/timer_hpm6360.c \
    ./src/platform/HPMicro/target/HPM6750/timer_hpm6750.c \
    ./src/main/flight/pid.c \
    ./src/main/flight/mixer.c \
    ./src/main/flight/imu.c \
    ./src/main/flight/rpm_filter.c \
    ./src/main/flight/dyn_notch_filter.c \
    ./src/main/flight/failsafe.c \
    ./src/main/flight/interpolated_setpoint.c \
    ./src/main/flight/feedforward.c \
    ./src/main/flight/rc_smoothing.c \
    ./src/main/flight/crash_recovery.c \
    ./src/main/flight/launch_control.c \
    ./src/main/common/filter.c \
    ./src/main/common/sdft.c \
    ./src/main/scheduler/scheduler.c \
    ./src/main/rx/rx.c \
    ./src/main/sensors/gyro.c \
    ./src/main/fc/core.c \
    ./src/main/fc/rc.c \
    ./src/main/fc/tasks.c \
    ./src/main/fc/rc_adjustments.c \
    ./src/main/fc/rc_controls.c \
    ./src/main/fc/rc_modes.c \
    ./src/main/fc/dispatch.c \
    ./src/main/common/maths.c \
    ./src/main/common/vector.c \
    ./src/main/common/explog_approx.c \
    ./src/main/common/sensor_alignment.c \
    ./src/main/common/time.c \
    ./src/main/common/typeconversion.c \
    ./src/main/common/bitarray.c \
    ./src/main/sensors/acceleration.c \
    ./src/main/sensors/gyro_filter_impl.c \
    ./src/main/sensors/sensors.c \
    ./src/main/rx/mavlink.c \
    ./src/main/telemetry/mavlink.c \
    ./src/main/flight/autopilot_multirotor.c \
    ./src/main/flight/position.c \
    ./src/main/flight/position_estimator.c \
    ./src/main/flight/alt_hold_multirotor.c \
    ./src/main/sensors/compass.c \
    ./src/main/drivers/dshot.c \
    ./src/main/drivers/dshot_command.c \
    ./src/main/drivers/motor.c \
    ./src/main/drivers/pwm_output.c

# Restore -O2 optimization for NOT_OPTIMISED_SRC platform files (LTO is disabled
# so the linker script can redirect them to ILM, but we still want speed).
SRC_CFLAGS_io_hpmicro.c = -O2
SRC_CFLAGS_system_hpmicro.c = -O2
SRC_CFLAGS_timer_hpmicro.c = -O2
SRC_CFLAGS_timer_hw_ext.c = -O2
SRC_CFLAGS_dma_hpmicro.c = -O2
SRC_CFLAGS_bus_spi_hpmicro.c = -O2
SRC_CFLAGS_exti_hpmicro.c = -O2
SRC_CFLAGS_adc_hpmicro.c = -O2
SRC_CFLAGS_pwm_output_hpmicro.c = -O2
SRC_CFLAGS_pwm_output_dshot.c = -O2
SRC_CFLAGS_pwm_output_dshot_shared.c = -O2
SRC_CFLAGS_dshot_dpwm.c = -O2
SRC_CFLAGS_trgm_dshot_resource.c = -O2
SRC_CFLAGS_dma_reqmap_mcu.c = -O2
SRC_CFLAGS_serial_uart_hpmicro.c = -O2
SRC_CFLAGS_timer_hpm6360.c = -O2
SRC_CFLAGS_timer_hpm6750.c = -O2
SRC_CFLAGS_pid.c = -O2
SRC_CFLAGS_mixer.c = -O2
SRC_CFLAGS_imu.c = -O2
SRC_CFLAGS_rpm_filter.c = -O2
SRC_CFLAGS_dyn_notch_filter.c = -O2
SRC_CFLAGS_failsafe.c = -O2
SRC_CFLAGS_interpolated_setpoint.c = -O2
SRC_CFLAGS_feedforward.c = -O2
SRC_CFLAGS_rc_smoothing.c = -O2
SRC_CFLAGS_crash_recovery.c = -O2
SRC_CFLAGS_launch_control.c = -O2
SRC_CFLAGS_filter.c = -O2
SRC_CFLAGS_sdft.c = -O2
SRC_CFLAGS_scheduler.c = -O2
SRC_CFLAGS_rx.c = -O2
SRC_CFLAGS_gyro.c = -O2
SRC_CFLAGS_core.c = -O2
SRC_CFLAGS_rc.c = -O2
SRC_CFLAGS_tasks.c = -O2
SRC_CFLAGS_rc_adjustments.c = -O2
SRC_CFLAGS_rc_controls.c = -O2
SRC_CFLAGS_rc_modes.c = -O2
SRC_CFLAGS_dispatch.c = -O2
SRC_CFLAGS_maths.c = -O2
SRC_CFLAGS_vector.c = -O2
SRC_CFLAGS_explog_approx.c = -O2
SRC_CFLAGS_sensor_alignment.c = -O2
SRC_CFLAGS_time.c = -O2
SRC_CFLAGS_typeconversion.c = -O2
SRC_CFLAGS_bitarray.c = -O2
SRC_CFLAGS_acceleration.c = -O2
SRC_CFLAGS_gyro_filter_impl.c = -O2
SRC_CFLAGS_sensors.c = -O2
SRC_CFLAGS_mavlink.c = -O2
SRC_CFLAGS_autopilot_multirotor.c = -O2
SRC_CFLAGS_position.c = -O2
SRC_CFLAGS_position_estimator.c = -O2
SRC_CFLAGS_alt_hold_multirotor.c = -O2
SRC_CFLAGS_compass.c = -O2
SRC_CFLAGS_dshot.c = -O2
SRC_CFLAGS_dshot_command.c = -O2
SRC_CFLAGS_motor.c = -O2
SRC_CFLAGS_pwm_output.c = -O2

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

# HPM driver files that need API porting — suppress errors temporarily
SRC_CFLAGS_serial_4way.c = -Wno-error
SRC_CFLAGS_gyro_sync.c = -Wno-error
SRC_CFLAGS_bus_spi.c = -Wno-error=unused-parameter -Wno-error=type-limits
# GCC diagnoses the legacy compass alignment enum/uint8_t API mismatch on
# RISC-V.  Keep this exception limited to the one affected common source.
SRC_CFLAGS_compass.c += -Wno-error=incompatible-pointer-types
# This source is compiled with zero I2C buses for the HPM targets, which makes
# its generic reset loop trigger GCC's type-limits diagnostic.
SRC_CFLAGS_bus_i2c.c += -Wno-error=type-limits
# Imported Open Location Code sources use double literals with float inputs.
SRC_CFLAGS_olc.c += -Wno-error=double-promotion
