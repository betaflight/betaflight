#
# Raspberry PICO Make file include
#

ifeq ($(DEBUG_HARDFAULTS),PICO)
CFLAGS          += -DDEBUG_HARDFAULTS
endif

SDK_DIR         = $(LIB_MAIN_DIR)/pico-sdk

#CMSIS
CMSIS_DIR      := $(SDK_DIR)/rp2_common/cmsis/stub/CMSIS

#STDPERIPH
STDPERIPH_DIR  := $(SDK_DIR)/rp2_common
STDPERIPH_SRC  := \
            pico_clib_interface/newlib_interface.c \
            hardware_sync_spin_lock/sync_spin_lock.c \
            hardware_gpio/gpio.c \
            pico_stdio/stdio.c \
            hardware_uart/uart.c \
            hardware_irq/irq.c \
            hardware_timer/timer.c \
            hardware_clocks/clocks.c \
            hardware_pll/pll.c \
            hardware_spi/spi.c \
            hardware_i2c/i2c.c \
            hardware_adc/adc.c \
            hardware_pio/pio.c \
            hardware_watchdog/watchdog.c \
            hardware_flash/flash.c

VPATH := $(VPATH):$(STDPERIPH_DIR)

DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC) \
            $(USBHID_SRC) \
            $(USBMSC_SRC)

ifeq ($(TARGET_MCU),RP2350B)
TARGET_MCU_LIB_LOWER = rp2350
TARGET_MCU_LIB_UPPER = RP2350
endif

#CMSIS
VPATH       := $(VPATH):$(CMSIS_DIR)/Core/Include:$(CMSIS_DIR)/Device/$(TARGET_MCU_LIB_UPPER)/Include
CMSIS_SRC   := \


INCLUDE_DIRS += \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/startup \
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
            $(CMSIS_DIR)/Core/Include \
            $(CMSIS_DIR)/Device/$(TARGET_MCU_LIB_UPPER)/Include \
            $(SDK_DIR)/$(TARGET_MCU_LIB_LOWER)/pico_platform/include \
            $(SDK_DIR)/$(TARGET_MCU_LIB_LOWER)/hardware_regs/include \
            $(SDK_DIR)/$(TARGET_MCU_LIB_LOWER)/hardware_structs/include

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m33 -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -mcmse

DEVICE_FLAGS    =

ifeq ($(TARGET_MCU),RP2350B)
DEVICE_FLAGS    += -DRP2350B -DLIB_BOOT_STAGE2_HEADERS=1 -DLIB_PICO_ATOMIC=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_CLIB_INTERFACE=1 -DLIB_PICO_CRT0=1 -DLIB_PICO_CXX_OPTIONS=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_COMPILER=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_FLOAT_PICO_VFP=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_COMPILER=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_COMPILER=1 -DLIB_PICO_NEWLIB_INTERFACE=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PLATFORM_COMPILER=1 -DLIB_PICO_PLATFORM_PANIC=1 -DLIB_PICO_PLATFORM_SECTIONS=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_RUNTIME_INIT=1 -DLIB_PICO_STANDARD_BINARY_INFO=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_TIME_ADAPTER=1 -DLIB_PICO_UTIL=1 -DPICO_32BIT=1 -DPICO_BUILD=1 -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_RP2350=1 -DPICO_USE_BLOCKED_RAM=0
LD_SCRIPT       = $(LINKER_DIR)/pico_rp2350.ld
STARTUP_SRC     = PICO/startup/bs2_default_padded_checksummed.S
MCU_FLASH_SIZE  = 4096
# Override the OPTIMISE_SPEED compiler setting to save flash space on these 512KB targets.
# Performance is only slightly affected but around 50 kB of flash are saved.
OPTIMISE_SPEED  = -O2

STDPERIPH_SRC += \
            common/RP2350/pico_platform/platform.c

MCU_SRC += \
            system_RP2350.c

else
$(error Unknown MCU for Raspberry PICO target)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DPICO

MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/bus_spi.c \
            PICO/system.c \
            PICO/io_pico.c \
            PICO/bus_spi_pico.c \
            PICO/serial_uart_pico.c \
            PICO/config_flash.c

DEVICE_FLAGS +=
