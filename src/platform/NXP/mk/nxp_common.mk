#
# NXP shared make file include, included by each family .mk after it
# sets PLATFORM_SDK, NXP_FAMILY and the per part settings.
#

DEFAULT_OUTPUT := hex

PLATFORM_SDK_STAMP ?= $(MCUX_RT_STAMP) $(TINYUSB_STAMP)

# Cortex-M7 with the double precision FPU.
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard

# The flash writer runs while flash is unreadable and the clock code re-locks the PLL that
# feeds it, so neither may call memcpy or memset; the MPU setup is held to the same rule.
NXP_NO_LIBCALL_TARGETS := %/clocks_$(NXP_FAMILY).o %/mpu_$(NXP_FAMILY).o %/flexspi_nor_imxrt.o %/flexspi_nor_imx95.o
$(NXP_NO_LIBCALL_TARGETS): CFLAGS += -fno-tree-loop-distribute-patterns \
                                       -fno-builtin-memcpy -fno-builtin-memset

# LTO moves functions between translation units and breaks the per file ITCM
# and RAM section placement.
LTO             := no

# Section layout shared by every board; LD_SCRIPT holds only the MEMORY block.
LD_SCRIPTS       = $(LINKER_DIR)/nxp_itcm256k_sections.ld

# Vendor CMSIS device headers, from lib/modules/mcux-devices-rt unless the
# family .mk sets MCUX_RT_DIR first. This is the only place their paths are
# recorded; MCUX_RT_SERIES names the series directory, which does not always
# match the part number.
MCUX_RT_DIR         ?= $(LIB_MODULES_DIR)/mcux-devices-rt
MCUX_RT_FAMILY_DIR  := $(MCUX_RT_DIR)/$(MCUX_RT_SERIES)
MCUX_RT_DEVICE_DIR  := $(MCUX_RT_FAMILY_DIR)/$(TARGET_MCU)

CMSIS_DIR           := $(LIB_MAIN_DIR)/CMSIS
CMSIS_SRC           :=

INCLUDE_DIRS += \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/include \
            $(TARGET_PLATFORM_DIR)/startup \
            $(TARGET_PLATFORM_DIR)/usb

# Vendor and CMSIS headers go in through -isystem so -Werror skips them.
SYS_INCLUDE_DIRS += \
            $(CMSIS_DIR)/Core/Include \
            $(MCUX_RT_DEVICE_DIR) \
            $(MCUX_RT_DEVICE_DIR)/drivers \
            $(MCUX_RT_FAMILY_DIR)/periph

# TinyUSB device stack with the CDC and MSC classes on the ChipIdea USB1 core.
# Its headers must not go through -isystem, or -MMD drops tusb_config.h from
# the dependency files.
TINYUSB_DIR := $(LIB_MODULES_DIR)/tinyusb/src

INCLUDE_DIRS += \
            $(TINYUSB_DIR)

TINYUSB_SRC := \
            $(TINYUSB_DIR)/tusb.c \
            $(TINYUSB_DIR)/common/tusb_fifo.c \
            $(TINYUSB_DIR)/device/usbd.c \
            $(TINYUSB_DIR)/device/usbd_control.c \
            $(TINYUSB_DIR)/class/cdc/cdc_device.c \
            $(TINYUSB_DIR)/class/msc/msc_device.c \
            $(TINYUSB_DIR)/portable/chipidea/ci_hs/dcd_ci_hs.c

# DroneCAN for the FlexCAN controllers; mk/dronecan.mk does the rest.
LIB_SUBMODULES   += $(DRONECAN_LIB_DIR)

# Platform sources built for every NXP target.
MCU_COMMON_SRC = \
            common/stm32/can_pinconfig.c \
            drivers/adc.c \
            drivers/bus_spi_config.c \
            drivers/inverter.c \
            drivers/serial_pinconfig.c \
            NXP/bus_i2c_nxp.c \
            NXP/bus_spi_nxp.c \
            NXP/can_nxp.c \
            NXP/config_flash.c \
            NXP/debug.c \
            NXP/dma_imxrt.c \
            NXP/dshot_flexio_nxp.c \
            NXP/dshot_flexio_decode.c \
            NXP/exti_imxrt.c \
            NXP/fault_handlers.c \
            NXP/flexpwm_imxrt.c \
            NXP/flexpwm_pins_imxrt.c \
            NXP/flexspi_nor_imxrt.c \
            NXP/io_imxrt.c \
            NXP/iomux_nxp.c \
            NXP/light_ws2811strip_imxrt.c \
            NXP/persistent.c \
            NXP/pwm_beeper_imxrt.c \
            NXP/pwm_motor_imxrt.c \
            NXP/pwm_servo_imxrt.c \
            NXP/romapi_imxrt.c \
            NXP/sdio_usdhc.c \
            NXP/serial_uart_nxp.c \
            NXP/serial_usb_vcp_imxrt.c \
            NXP/system.c \
            NXP/usb_io_imxrt.c \
            NXP/usb/usb_cdc.c \
            NXP/usb/usb_descriptors.c \
            NXP/usb/usb_msc_imxrt.c \
            NXP/xip/image_vector_table.c

# Mass storage over the SD card, served by sdio_usdhc.c.
MSC_SRC = \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_sdio.c

# The MPU window is sized from TARGET_FLASH_SIZE, so nxp_itcm256k_sections.ld checks it
# against the flash the linker script describes.
EXTRA_LD_FLAGS += -Wl,--defsym=__target_flash_size=$(or $(TARGET_FLASH_SIZE),$(MCU_FLASH_SIZE))

# The core temperature driver is built where the family has one.
MCU_COMMON_SRC += \
            NXP/adc_$(NXP_FAMILY).c \
            $(if $(wildcard $(TARGET_PLATFORM_DIR)/coretemp_$(NXP_FAMILY).c),NXP/coretemp_$(NXP_FAMILY).c) \
            NXP/startup/clocks_$(NXP_FAMILY).c \
            NXP/startup/mpu_$(NXP_FAMILY).c

STARTUP_SRC = NXP/startup/startup_$(NXP_FAMILY).S \
              NXP/startup/startup_nxp_common.S

DEVICE_STDPERIPH_SRC := $(TINYUSB_SRC)

MCU_EXCLUDES =

# After each link, print ITCM and DTCM use and fail on a stale rule in nxp_itcm256k_sections.ld.
ELF_LINK_CMD += && { command -v $(PYTHON) >/dev/null 2>&1 || { echo "check_placement.py needs $(PYTHON)" >&2; false; }; } \
    && $(PYTHON) $(LINKER_DIR)/check_placement.py $(TARGET_MAP) $(LD_SCRIPTS) \
    || { rm -f $@; exit 1; }
