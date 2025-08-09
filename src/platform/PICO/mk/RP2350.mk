#
# Raspberry PICO Make file include
#
# The top level Makefile adds $(MCU_COMMON_SRC) and $(DEVICE_STDPERIPH_SRC) to SRC collection.
#

# PICO_TRACE = 1
DEFAULT_OUTPUT := uf2

# Run from SRAM. To disable, set environment variable RUN_FROM_RAM=0
ifeq ($(RUN_FROM_RAM),)
RUN_FROM_RAM = 1
endif

PICO_LIB_OPTIMISATION      := -O2 -fuse-linker-plugin -ffast-math -fmerge-all-constants

# This file PICO.mk is $(TARGET_PLATFORM_DIR)/mk/$(TARGET_MCU_FAMILY).mk
PICO_MK_DIR = $(TARGET_PLATFORM_DIR)/mk

ifneq ($(PICO_TRACE),)
include $(PICO_MK_DIR)/PICO_trace.mk
endif

RP2350_TARGETS = RP2350A RP2350B
ifneq ($(filter $(TARGET_MCU), $(RP2350_TARGETS)),)
RP2350_TARGET = $(TARGET_MCU)
endif

ifeq ($(DEBUG_HARDFAULTS),PICO)
CFLAGS          += -DDEBUG_HARDFAULTS
endif

SDK_DIR         = $(LIB_MAIN_DIR)/pico-sdk/src

#CMSIS
CMSIS_DIR      := $(SDK_DIR)/rp2_common/cmsis/stub/CMSIS

#STDPERIPH
STDPERIPH_DIR  := $(SDK_DIR)

PICO_LIB_SRC = \
            rp2_common/pico_crt0/crt0.S \
            rp2_common/hardware_sync_spin_lock/sync_spin_lock.c \
            rp2_common/hardware_gpio/gpio.c \
            rp2_common/hardware_uart/uart.c \
            rp2_common/hardware_irq/irq.c \
            rp2_common/hardware_irq/irq_handler_chain.S \
            rp2_common/hardware_timer/timer.c \
            rp2_common/hardware_clocks/clocks.c \
            rp2_common/hardware_pll/pll.c \
            rp2_common/hardware_dma/dma.c \
            rp2_common/hardware_spi/spi.c \
            rp2_common/hardware_i2c/i2c.c \
            rp2_common/hardware_adc/adc.c \
            rp2_common/hardware_pio/pio.c \
            rp2_common/hardware_watchdog/watchdog.c \
            rp2_common/hardware_flash/flash.c \
            rp2_common/pico_unique_id/unique_id.c \
            rp2_common/pico_platform_panic/panic.c \
            rp2_common/pico_multicore/multicore.c \
            common/pico_sync/mutex.c \
            common/pico_time/time.c \
            common/pico_sync/lock_core.c \
            common/hardware_claim/claim.c \
            common/pico_sync/critical_section.c \
            rp2_common/hardware_sync/sync.c \
            rp2_common/pico_bootrom/bootrom.c \
            rp2_common/pico_runtime_init/runtime_init.c \
            rp2_common/pico_runtime_init/runtime_init_clocks.c \
            rp2_common/pico_runtime_init/runtime_init_stack_guard.c \
            rp2_common/pico_runtime/runtime.c \
            rp2_common/hardware_ticks/ticks.c \
            rp2_common/hardware_xosc/xosc.c \
            common/pico_sync/sem.c \
            common/pico_time/timeout_helper.c \
            common/pico_util/datetime.c \
            common/pico_util/pheap.c \
            common/pico_util/queue.c \
            rp2350/pico_platform/platform.c \
            rp2_common/pico_atomic/atomic.c \
            rp2_common/pico_bootrom/bootrom.c \
            rp2_common/pico_bootrom/bootrom_lock.c \
            rp2_common/pico_divider/divider_compiler.c \
            rp2_common/pico_double/double_math.c \
            rp2_common/pico_flash/flash.c \
            rp2_common/pico_float/float_math.c \
            rp2_common/hardware_divider/divider.c \
            rp2_common/hardware_vreg/vreg.c \
            rp2_common/hardware_xip_cache/xip_cache.c \
            rp2_common/pico_standard_binary_info/standard_binary_info.c \
            rp2_common/pico_clib_interface/newlib_interface.c \
            rp2_common/pico_malloc/malloc.c \
            rp2_common/pico_stdlib/stdlib.c

TINY_USB_SRC_DIR = $(LIB_MAIN_DIR)/pico-sdk/lib/tinyusb/src
TINYUSB_SRC := \
            $(TINY_USB_SRC_DIR)/tusb.c \
            $(TINY_USB_SRC_DIR)/class/cdc/cdc_device.c \
            $(TINY_USB_SRC_DIR)/common/tusb_fifo.c \
            $(TINY_USB_SRC_DIR)/device/usbd.c \
            $(TINY_USB_SRC_DIR)/device/usbd_control.c \
            $(TINY_USB_SRC_DIR)/portable/raspberrypi/rp2040/dcd_rp2040.c \
            $(TINY_USB_SRC_DIR)/portable/raspberrypi/rp2040/rp2040_usb.c

# TODO which of these do we need?
TINYUSB_SRC += \
            $(TINY_USB_SRC_DIR)/class/vendor/vendor_device.c \
            $(TINY_USB_SRC_DIR)/class/net/ecm_rndis_device.c \
            $(TINY_USB_SRC_DIR)/class/net/ncm_device.c \
            $(TINY_USB_SRC_DIR)/class/dfu/dfu_rt_device.c \
            $(TINY_USB_SRC_DIR)/class/dfu/dfu_device.c \
            $(TINY_USB_SRC_DIR)/class/msc/msc_device.c \
            $(TINY_USB_SRC_DIR)/class/midi/midi_device.c \
            $(TINY_USB_SRC_DIR)/class/video/video_device.c \
            $(TINY_USB_SRC_DIR)/class/hid/hid_device.c \
            $(TINY_USB_SRC_DIR)/class/usbtmc/usbtmc_device.c \
            $(TINY_USB_SRC_DIR)/class/audio/audio_device.c


VPATH := $(VPATH):$(STDPERIPH_DIR)

ifdef RP2350_TARGET
TARGET_MCU_LIB_LOWER = rp2350
TARGET_MCU_LIB_UPPER = RP2350
endif

#CMSIS
VPATH       := $(VPATH):$(CMSIS_DIR)/Core/Include:$(CMSIS_DIR)/Device/$(TARGET_MCU_LIB_UPPER)/Include
CMSIS_SRC   :=

INCLUDE_DIRS += \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/include \
            $(TARGET_PLATFORM_DIR)/usb \
            $(TARGET_PLATFORM_DIR)/startup

SYS_INCLUDE_DIRS = \
            $(SDK_DIR)/common/pico_bit_ops_headers/include \
            $(SDK_DIR)/common/pico_base_headers/include \
            $(SDK_DIR)/common/boot_picoboot_headers/include \
            $(SDK_DIR)/common/pico_usb_reset_interface_headers/include \
            $(SDK_DIR)/common/pico_time/include \
            $(SDK_DIR)/common/boot_uf2_headers/include \
            $(SDK_DIR)/common/pico_divider_headers/include \
            $(SDK_DIR)/common/boot_picobin_headers/include \
            $(SDK_DIR)/common/pico_util/include \
            $(SDK_DIR)/common/pico_stdlib_headers/include \
            $(SDK_DIR)/common/hardware_claim/include \
            $(SDK_DIR)/common/pico_binary_info/include \
            $(SDK_DIR)/common/pico_sync/include \
            $(SDK_DIR)/rp2_common/pico_stdio_uart/include \
            $(SDK_DIR)/rp2_common/pico_stdio_usb/include \
            $(SDK_DIR)/rp2_common/pico_stdio_rtt/include \
            $(SDK_DIR)/rp2_common/tinyusb/include \
            $(SDK_DIR)/rp2_common/hardware_rtc/include \
            $(SDK_DIR)/rp2_common/hardware_boot_lock/include \
            $(SDK_DIR)/rp2_common/pico_mem_ops/include \
            $(SDK_DIR)/rp2_common/hardware_exception/include \
            $(SDK_DIR)/rp2_common/hardware_sync_spin_lock/include \
            $(SDK_DIR)/rp2_common/pico_runtime_init/include \
            $(SDK_DIR)/rp2_common/pico_standard_link/include \
            $(SDK_DIR)/rp2_common/hardware_pio/include \
            $(SDK_DIR)/rp2_common/pico_platform_compiler/include \
            $(SDK_DIR)/rp2_common/hardware_divider/include \
            $(SDK_DIR)/rp2_common/pico_bootsel_via_double_reset/include \
            $(SDK_DIR)/rp2_common/hardware_powman/include \
            $(SDK_DIR)/rp2_common/pico_btstack/include \
            $(SDK_DIR)/rp2_common/pico_cyw43_driver/include \
            $(SDK_DIR)/rp2_common/hardware_flash/include \
            $(SDK_DIR)/rp2_common/hardware_ticks/include \
            $(SDK_DIR)/rp2_common/hardware_dma/include \
            $(SDK_DIR)/rp2_common/pico_bit_ops/include \
            $(SDK_DIR)/rp2_common/hardware_clocks/include \
            $(SDK_DIR)/rp2_common/pico_unique_id/include \
            $(SDK_DIR)/rp2_common/hardware_dcp/include \
            $(SDK_DIR)/rp2_common/hardware_watchdog/include \
            $(SDK_DIR)/rp2_common/pico_rand/include \
            $(SDK_DIR)/rp2_common/hardware_hazard3/include \
            $(SDK_DIR)/rp2_common/hardware_uart/include \
            $(SDK_DIR)/rp2_common/hardware_interp/include \
            $(SDK_DIR)/rp2_common/pico_printf/include \
            $(SDK_DIR)/rp2_common/pico_aon_timer/include \
            $(SDK_DIR)/rp2_common/hardware_riscv_platform_timer/include \
            $(SDK_DIR)/rp2_common/pico_double/include \
            $(SDK_DIR)/rp2_common/pico_cyw43_arch/include \
            $(SDK_DIR)/rp2_common/hardware_vreg/include \
            $(SDK_DIR)/rp2_common/pico_mbedtls/include \
            $(SDK_DIR)/rp2_common/hardware_spi/include \
            $(SDK_DIR)/rp2_common/hardware_rcp/include \
            $(SDK_DIR)/rp2_common/hardware_riscv/include \
            $(SDK_DIR)/rp2_common/pico_standard_binary_info/include \
            $(SDK_DIR)/rp2_common/pico_i2c_slave/include \
            $(SDK_DIR)/rp2_common/pico_int64_ops/include \
            $(SDK_DIR)/rp2_common/pico_sha256/include \
            $(SDK_DIR)/rp2_common/hardware_irq/include \
            $(SDK_DIR)/rp2_common/pico_divider/include \
            $(SDK_DIR)/rp2_common/pico_flash/include \
            $(SDK_DIR)/rp2_common/hardware_sync/include \
            $(SDK_DIR)/rp2_common/pico_bootrom/include \
            $(SDK_DIR)/rp2_common/pico_crt0/include \
            $(SDK_DIR)/rp2_common/pico_clib_interface/include \
            $(SDK_DIR)/rp2_common/pico_stdio/include \
            $(SDK_DIR)/rp2_common/pico_runtime/include \
            $(SDK_DIR)/rp2_common/pico_time_adapter/include \
            $(SDK_DIR)/rp2_common/pico_platform_panic/include \
            $(SDK_DIR)/rp2_common/hardware_adc/include \
            $(SDK_DIR)/rp2_common/cmsis/include \
            $(SDK_DIR)/rp2_common/hardware_pll/include \
            $(SDK_DIR)/rp2_common/pico_platform_sections/include \
            $(SDK_DIR)/rp2_common/boot_bootrom_headers/include \
            $(SDK_DIR)/rp2_common/pico_fix/include \
            $(SDK_DIR)/rp2_common/pico_lwip/include \
            $(SDK_DIR)/rp2_common/hardware_base/include \
            $(SDK_DIR)/rp2_common/hardware_xosc/include \
            $(SDK_DIR)/rp2_common/pico_async_context/include \
            $(SDK_DIR)/rp2_common/hardware_pwm/include \
            $(SDK_DIR)/rp2_common/pico_stdio_semihosting/include \
            $(SDK_DIR)/rp2_common/pico_float/include \
            $(SDK_DIR)/rp2_common/hardware_resets/include \
            $(SDK_DIR)/rp2_common/pico_cxx_options/include \
            $(SDK_DIR)/rp2_common/pico_stdlib/include \
            $(SDK_DIR)/rp2_common/hardware_sha256/include \
            $(SDK_DIR)/rp2_common/hardware_i2c/include \
            $(SDK_DIR)/rp2_common/pico_atomic/include \
            $(SDK_DIR)/rp2_common/pico_multicore/include \
            $(SDK_DIR)/rp2_common/hardware_gpio/include \
            $(SDK_DIR)/rp2_common/pico_malloc/include \
            $(SDK_DIR)/rp2_common/hardware_timer/include \
            $(SDK_DIR)/rp2_common/hardware_xip_cache/include \
            $(CMSIS_DIR)/Core/Include \
            $(CMSIS_DIR)/Device/$(TARGET_MCU_LIB_UPPER)/Include \
            $(SDK_DIR)/$(TARGET_MCU_LIB_LOWER)/pico_platform/include \
            $(SDK_DIR)/$(TARGET_MCU_LIB_LOWER)/hardware_regs/include \
            $(SDK_DIR)/$(TARGET_MCU_LIB_LOWER)/hardware_structs/include \
            $(LIB_MAIN_DIR)/pico-sdk/lib/tinyusb/src

SYS_INCLUDE_DIRS += \
            $(SDK_DIR)/rp2350/boot_stage2/include

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m33 -march=armv8-m.main+fp+dsp -mcmse -mfloat-abi=softfp
ARCH_FLAGS      += -DPICO_COPY_TO_RAM=$(RUN_FROM_RAM)

# Automatically treating constants as single-precision breaks pico-sdk (-Werror=double-promotion)
# We should go through BF code and explicitly declare constants as single rather than double as required,
# rather than relying on this flag.
# ARCH_FLAGS      += -fsingle-precision-constant

PICO_STDIO_USB_FLAGS = \
            -DLIB_PICO_PRINTF=1 \
            -DLIB_PICO_PRINTF_PICO=1  \
            -DLIB_PICO_STDIO=1  \
            -DLIB_PICO_STDIO_USB=1 \
            -DCFG_TUSB_DEBUG=0  \
            -DCFG_TUSB_MCU=OPT_MCU_RP2040  \
            -DCFG_TUSB_OS=OPT_OS_NONE  \
            -DLIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1 \
            -DPICO_RP2040_USB_DEVICE_UFRAME_FIX=1  \
            -DPICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000 \
            -DLIB_PICO_UNIQUEID=1

PICO_STDIO_LD_FLAGS  = \
            -Wl,--wrap=sprintf \
            -Wl,--wrap=snprintf \
            -Wl,--wrap=vsnprintf \
            -Wl,--wrap=printf \
            -Wl,--wrap=vprintf \
            -Wl,--wrap=puts \
            -Wl,--wrap=putchar \
            -Wl,--wrap=getchar

EXTRA_LD_FLAGS += $(PICO_STDIO_LD_FLAGS) $(PICO_TRACE_LD_FLAGS)

ifdef RP2350_TARGET

# Q. do we need LIB_BOOT_STAGE_2_HEADERS?
# TODO review LIB_PICO options
DEVICE_FLAGS    += \
            -D$(RP2350_TARGET) \
            -DPICO_RP2350_A2_SUPPORTED=1 \
            -DLIB_BOOT_STAGE2_HEADERS=1 \
            -DLIB_PICO_ATOMIC=1 \
            -DLIB_PICO_BIT_OPS=1 \
            -DLIB_PICO_BIT_OPS_PICO=1 \
            -DLIB_PICO_CLIB_INTERFACE=1 \
            -DLIB_PICO_CRT0=1 \
            -DLIB_PICO_CXX_OPTIONS=1 \
            -DLIB_PICO_DIVIDER=1 \
            -DLIB_PICO_DIVIDER_COMPILER=1 \
            -DLIB_PICO_DOUBLE=1 \
            -DLIB_PICO_DOUBLE_PICO=1 \
            -DLIB_PICO_FLOAT=1 \
            -DLIB_PICO_FLOAT_PICO=1 \
            -DLIB_PICO_FLOAT_PICO_VFP=1 \
            -DLIB_PICO_INT64_OPS=1 \
            -DLIB_PICO_INT64_OPS_COMPILER=1 \
            -DLIB_PICO_MALLOC=1 \
            -DLIB_PICO_MEM_OPS=1 \
            -DLIB_PICO_MEM_OPS_COMPILER=1 \
            -DLIB_PICO_NEWLIB_INTERFACE=1 \
            -DLIB_PICO_PLATFORM=1 \
            -DLIB_PICO_PLATFORM_COMPILER=1 \
            -DLIB_PICO_PLATFORM_PANIC=1 \
            -DLIB_PICO_PLATFORM_SECTIONS=1 \
            -DLIB_PICO_PRINTF=1 \
            -DLIB_PICO_PRINTF_PICO=1 \
            -DLIB_PICO_RUNTIME=1 \
            -DLIB_PICO_RUNTIME_INIT=1 \
            -DLIB_PICO_STANDARD_BINARY_INFO=1 \
            -DLIB_PICO_STANDARD_LINK=1 \
            -DLIB_PICO_STDIO=1 \
            -DLIB_PICO_STDIO_UART=1 \
            -DLIB_PICO_STDLIB=1 \
            -DLIB_PICO_SYNC=1 \
            -DLIB_PICO_SYNC_CRITICAL_SECTION=1 \
            -DLIB_PICO_SYNC_MUTEX=1 \
            -DLIB_PICO_SYNC_SEM=1 \
            -DLIB_PICO_TIME=1 \
            -DLIB_PICO_TIME_ADAPTER=1 \
            -DLIB_PICO_UTIL=1 \
            -DPICO_32BIT=1 \
            -DPICO_BUILD=1 \
            -DPICO_CXX_ENABLE_EXCEPTIONS=0 \
            -DPICO_NO_FLASH=0 \
            -DPICO_NO_HARDWARE=0 \
            -DPICO_ON_DEVICE=1 \
            -DPICO_RP2350=1 \
            -DPICO_USE_BLOCKED_RAM=0

ifeq ($(RUN_FROM_RAM),1)
LD_SCRIPT       = $(LINKER_DIR)/pico_rp2350_RunFromRAM.ld
else
LD_SCRIPT       = $(LINKER_DIR)/pico_rp2350_RunFromFLASH.ld
endif

STARTUP_SRC     = PICO/startup/bs2_default_padded_checksummed.S

# Override the OPTIMISE_SPEED compiler setting to save flash space on these 512KB targets.
# Performance is only slightly affected but around 50 kB of flash are saved.
OPTIMISE_SPEED  = -O2

# TODO tidy up -
# we might lose some/all of pico_stdio_*, pico_printf if not using PICO_TRACE
PICO_LIB_SRC += \
            rp2_common/pico_stdio/stdio.c \
            rp2_common/pico_printf/printf.c

ifneq ($(PICO_TRACE),)
PICO_LIB_SRC += \
            rp2_common/pico_stdio_uart/stdio_uart.c
endif

# TODO review
# rp2_common/pico_stdio_usb/stdio_usb.c \

PICO_STDIO_USB_SRC = \
            rp2_common/pico_stdio_usb/reset_interface.c \
            rp2_common/pico_fix/rp2040_usb_device_enumeration/rp2040_usb_device_enumeration.c \
            rp2_common/pico_bit_ops/bit_ops_aeabi.S

# TODO check
#    rp2_common/pico_stdio_usb/stdio_usb_descriptors.c \
# vs PICO/usb/usb_descriptors.c

PICO_LIB_SRC += $(PICO_STDIO_USB_SRC)

# Work around https://github.com/raspberrypi/pico-sdk/issues/2451
# by using system headers, which are more tolerant of macro redefinitions.

SYS_INCLUDE_DIRS += \
            $(SDK_DIR)/rp2_common/pico_fix/rp2040_usb_device_enumeration/include

# TODO use system_RP2350.c instead of writing into PICO/system.c
# MCU_COMMON_SRC += \
#             system_RP2350.c

else
$(error Unknown MCU for Raspberry PICO target)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DPICO

DEVICE_FLAGS    += $(PICO_STDIO_USB_FLAGS)

MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_i2c_utils.c \
            drivers/serial_pinconfig.c \
            drivers/usb_io.c \
            drivers/dshot.c \
            drivers/adc.c \
            PICO/adc_pico.c \
            PICO/bus_i2c_pico.c \
            PICO/bus_spi_pico.c \
            PICO/bus_quadspi_pico.c \
            PICO/config_flash.c \
            PICO/debug_pico.c \
            PICO/dma_pico.c \
            PICO/dshot_pico.c \
            PICO/exti_pico.c \
            PICO/io_pico.c \
            PICO/persistent.c \
            PICO/pwm_pico.c \
            PICO/pwm_beeper_pico.c \
            PICO/serial_uart_pico.c \
            PICO/serial_usb_vcp_pico.c \
            PICO/system.c \
            PICO/usb/usb_cdc.c \
            PICO/usb/usb_descriptors.c \
            PICO/usb/usb_msc_pico.c \
            PICO/multicore.c \
            PICO/debug_pin.c \
            PICO/light_ws2811strip_pico.c

# USB MSC support sources (TinyUSB backend on PICO)
MSC_SRC = \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c

DEVICE_STDPERIPH_SRC := \
            $(PICO_LIB_SRC) \
            $(STDPERIPH_SRC) \
            $(TINYUSB_SRC) \
            $(PICO_TRACE_SRC)

# Add a target-specific definition for PICO_LIB_TARGETS in order
# to remove -flto=auto for pico-sdk file compilation
# (to avoid problem of interaction with wrapping -Wl,wrap=...).
# Place this at the end because we require PICO_LIB_TARGETS to be expanded before setting the target

PICO_LIB_OBJS = $(addsuffix .o, $(basename $(PICO_LIB_SRC)))
PICO_LIB_OBJS += $(addsuffix .o, $(basename $(PICO_TRACE_SRC)))
PICO_LIB_TARGETS := $(foreach pobj, $(PICO_LIB_OBJS), %/$(pobj))
$(PICO_LIB_TARGETS): CC_DEFAULT_OPTIMISATION := $(PICO_LIB_OPTIMISATION)
