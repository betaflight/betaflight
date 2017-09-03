###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the iNav firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#
###############################################################################


# Things that the user might override on the commandline
#

# The target to build, see VALID_TARGETS below
TARGET    ?= SPRACINGF3

# Compile-time options
OPTIONS   ?=

# compile for OpenPilot BootLoader support
OPBL      ?= no

# Debugger optons, must be empty or GDB
DEBUG     ?=

# Insert the debugging hardfault debugger
# releases should not be built with this flag as it does not disable pwm output
DEBUG_HARDFAULTS ?=

# Serial port/Device for flashing
SERIAL_DEVICE   ?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

## V                 : Set verbosity level based on the V= parameter
##                     V=0 Low
##                     V=1 High
export AT := @

ifndef V
export V0    :=
export V1    := $(AT)
export STDOUT   :=
else ifeq ($(V), 0)
export V0    := $(AT)
export V1    := $(AT)
export STDOUT:= "> /dev/null"
export MAKE  := $(MAKE) --no-print-directory
else ifeq ($(V), 1)
export V0    :=
export V1    :=
export STDOUT   :=
endif

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME      = inav

# Working directories
ROOT            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR         = $(ROOT)/src/main
OBJECT_DIR      = $(ROOT)/obj/main
BIN_DIR         = $(ROOT)/obj
CMSIS_DIR       = $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS    = $(SRC_DIR) \
                  $(ROOT)/src/main/target
LINKER_DIR      = $(ROOT)/src/main/target/link

# default xtal value for F4 targets
HSE_VALUE       = 8000000

# used for turning on features like VCP and SDCARD
FEATURES        =

ALT_TARGETS     = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.mk)))))
OPBL_TARGETS    = $(filter %_OPBL, $(ALT_TARGETS))

#VALID_TARGETS  = $(F1_TARGETS) $(F3_TARGETS) $(F4_TARGETS)
VALID_TARGETS   = $(dir $(wildcard $(ROOT)/src/main/target/*/target.mk))
VALID_TARGETS  := $(subst /,, $(subst ./src/main/target/,, $(VALID_TARGETS)))
VALID_TARGETS  := $(VALID_TARGETS) $(ALT_TARGETS)
VALID_TARGETS  := $(sort $(VALID_TARGETS))

CLEAN_TARGETS = $(addprefix clean_,$(VALID_TARGETS) )
TARGETS_CLEAN = $(addsuffix _clean,$(VALID_TARGETS) )

ifeq ($(filter $(TARGET),$(ALT_TARGETS)), $(TARGET))
BASE_TARGET    := $(firstword $(subst /,, $(subst ./src/main/target/,, $(dir $(wildcard $(ROOT)/src/main/target/*/$(TARGET).mk)))))
-include $(ROOT)/src/main/target/$(BASE_TARGET)/$(TARGET).mk
else
BASE_TARGET    := $(TARGET)
endif

ifeq ($(filter $(TARGET),$(OPBL_TARGETS)), $(TARGET))
OPBL            = yes
endif

# silently ignore if the file is not present. Allows for target specific.
-include $(ROOT)/src/main/target/$(BASE_TARGET)/target.mk

F4_TARGETS      = $(F405_TARGETS) $(F411_TARGETS) $(F427_TARGETS)
F7_TARGETS      = $(F7X2RE_TARGETS) $(F7X5XE_TARGETS) $(F7X5XG_TARGETS) $(F7X5XI_TARGETS) $(F7X6XG_TARGETS)

ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS). Have you prepared a valid target.mk?)
endif

ifeq ($(filter $(TARGET),$(F1_TARGETS) $(F3_TARGETS) $(F4_TARGETS) $(F7_TARGETS)),)
$(error Target '$(TARGET)' has not specified a valid STM group, must be one of F1, F3, F405, F411, F427 or F7x. Have you prepared a valid target.mk?)
endif

128K_TARGETS  = $(F1_TARGETS)
256K_TARGETS  = $(F3_TARGETS)
512K_TARGETS  = $(F411_TARGETS) $(F7X2RE_TARGETS) $(F7X5XE_TARGETS)
1024K_TARGETS = $(F405_TARGETS) $(F7X5XG_TARGETS) $(F7X6XG_TARGETS)
2048K_TARGETS = $(F427_TARGETS) $(F7X5XI_TARGETS)

# Configure default flash sizes for the targets (largest size specified gets hit first) if flash not specified already.
ifeq ($(FLASH_SIZE),)
ifeq ($(TARGET),$(filter $(TARGET),$(2048K_TARGETS)))
FLASH_SIZE = 2048
else ifeq ($(TARGET),$(filter $(TARGET),$(1024K_TARGETS)))
FLASH_SIZE = 1024
else ifeq ($(TARGET),$(filter $(TARGET),$(512K_TARGETS)))
FLASH_SIZE = 512
else ifeq ($(TARGET),$(filter $(TARGET),$(256K_TARGETS)))
FLASH_SIZE = 256
else ifeq ($(TARGET),$(filter $(TARGET),$(128K_TARGETS)))
FLASH_SIZE = 128
else
$(error FLASH_SIZE not configured for target $(TARGET))
endif
endif

# note that there is no hardfault debugging startup file assembly handler for other platforms
ifeq ($(DEBUG_HARDFAULTS),F3)
CFLAGS               += -DDEBUG_HARDFAULTS
STM32F30x_COMMON_SRC  = startup_stm32f3_debug_hardfault_handler.S
else
STM32F30x_COMMON_SRC  = startup_stm32f30x_md_gcc.S
endif

ifeq ($(DEBUG_HARDFAULTS),F7)
CFLAGS               += -DDEBUG_HARDFAULTS
endif

REVISION = $(shell git log -1 --format="%h")

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

ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
# F3 TARGETS

STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f30x_crc.c \
                  stm32f30x_can.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))
DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

VPATH           := $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC       = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
                  $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/CM1/CoreSupport \
                   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x

ifneq ($(filter VCP, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(USBFS_DIR)/inc \
                   $(ROOT)/src/main/vcp

VPATH           := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
                        $(USBPERIPH_SRC)
endif

ifneq ($(filter SDCARD, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR) \

VPATH           := $(VPATH):$(FATFS_DIR)
endif

LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS    = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS    = -D$(TARGET)
# End F3 targets
#
# Start F4 targets
else ifeq ($(TARGET),$(filter $(TARGET), $(F4_TARGETS)))

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F4xx_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f4xx_crc.c \
                  stm32f4xx_can.c \
                  stm32f4xx_fmc.c \
                  stm32f4xx_sai.c \
                  stm32f4xx_cec.c \
                  stm32f4xx_dsi.c \
                  stm32f4xx_flash_ramfunc.c \
                  stm32f4xx_fmpi2c.c \
                  stm32f4xx_lptim.c \
                  stm32f4xx_qspi.c \
                  stm32f4xx_spdifrx.c \
                  stm32f4xx_cryp.c \
                  stm32f4xx_cryp_aes.c \
                  stm32f4xx_hash_md5.c \
                  stm32f4xx_cryp_des.c \
                  stm32f4xx_rtc.c \
                  stm32f4xx_hash.c \
                  stm32f4xx_dbgmcu.c \
                  stm32f4xx_cryp_tdes.c \
                  stm32f4xx_hash_sha1.c

ifeq ($(TARGET),$(filter $(TARGET), $(F411_TARGETS)))
EXCLUDES += stm32f4xx_fsmc.c
endif

ifeq ($(TARGET),$(filter $(TARGET), $(F427_TARGETS)))
EXCLUDES += stm32f4xx_fsmc.c
endif

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/src/*.c))
USBOTG_DIR  = $(ROOT)/lib/main/STM32_USB_OTG_Driver
USBOTG_SRC  = $(notdir $(wildcard $(USBOTG_DIR)/src/*.c))
EXCLUDES    = usb_bsp_template.c \
              usb_conf_template.c \
              usb_hcd_int.c \
              usb_hcd.c \
              usb_otg.c

USBOTG_SRC  := $(filter-out ${EXCLUDES}, $(USBOTG_SRC))
USBCDC_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/cdc
USBCDC_SRC  = $(notdir $(wildcard $(USBCDC_DIR)/src/*.c))
EXCLUDES    = usbd_cdc_if_template.c
USBCDC_SRC  := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))
VPATH       := $(VPATH):$(USBOTG_DIR)/src:$(USBCORE_DIR)/src:$(USBCDC_DIR)/src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBOTG_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/CM4/CoreSupport:$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx
CMSIS_SRC       = $(notdir $(wildcard $(CMSIS_DIR)/CM4/CoreSupport/*.c \
                  $(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx/*.c))
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(USBOTG_DIR)/inc \
                   $(USBCORE_DIR)/inc \
                   $(USBCDC_DIR)/inc \
                   $(USBFS_DIR)/inc \
                   $(CMSIS_DIR)/CM4/CoreSupport \
                   $(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx \
                   $(ROOT)/src/main/vcpf4

ifneq ($(filter SDCARD,$(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR)
VPATH           := $(VPATH):$(FATFS_DIR)
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion

ifeq ($(TARGET),$(filter $(TARGET),$(F411_TARGETS)))
DEVICE_FLAGS    = -DSTM32F411xE
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f411.ld
else ifeq ($(TARGET),$(filter $(TARGET),$(F405_TARGETS)))
DEVICE_FLAGS    = -DSTM32F40_41xxx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f405.ld
else ifeq ($(TARGET),$(filter $(TARGET),$(F427_TARGETS)))
DEVICE_FLAGS    = -DSTM32F427_437xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f427.ld
else
$(error Unknown MCU for F4 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

TARGET_FLAGS = -D$(TARGET)
# End F4 targets
#
# Start F7 targets
else ifeq ($(TARGET),$(filter $(TARGET), $(F7_TARGETS)))

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F7xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        = stm32f7xx_hal_can.c \
                  stm32f7xx_hal_cec.c \
                  stm32f7xx_hal_crc.c \
                  stm32f7xx_hal_crc_ex.c \
                  stm32f7xx_hal_cryp.c \
                  stm32f7xx_hal_cryp_ex.c \
                  stm32f7xx_hal_dac.c \
                  stm32f7xx_hal_dac_ex.c \
                  stm32f7xx_hal_dcmi.c \
                  stm32f7xx_hal_dcmi_ex.c \
                  stm32f7xx_hal_dfsdm.c \
                  stm32f7xx_hal_dma2d.c \
                  stm32f7xx_hal_dsi.c \
                  stm32f7xx_hal_eth.c \
                  stm32f7xx_hal_hash.c \
                  stm32f7xx_hal_hash_ex.c \
                  stm32f7xx_hal_hcd.c \
                  stm32f7xx_hal_i2s.c \
                  stm32f7xx_hal_irda.c \
                  stm32f7xx_hal_iwdg.c \
                  stm32f7xx_hal_jpeg.c \
                  stm32f7xx_hal_lptim.c \
                  stm32f7xx_hal_ltdc.c \
                  stm32f7xx_hal_ltdc_ex.c \
                  stm32f7xx_hal_mdios.c \
                  stm32f7xx_hal_mmc.c \
                  stm32f7xx_hal_msp_template.c \
                  stm32f7xx_hal_nand.c \
                  stm32f7xx_hal_nor.c \
                  stm32f7xx_hal_qspi.c \
                  stm32f7xx_hal_rng.c \
                  stm32f7xx_hal_rtc.c \
                  stm32f7xx_hal_rtc_ex.c \
                  stm32f7xx_hal_sai.c \
                  stm32f7xx_hal_sai_ex.c \
                  stm32f7xx_hal_sd.c \
                  stm32f7xx_hal_sdram.c \
                  stm32f7xx_hal_smartcard.c \
                  stm32f7xx_hal_smartcard_ex.c \
                  stm32f7xx_hal_smbus.c \
                  stm32f7xx_hal_spdifrx.c \
                  stm32f7xx_hal_sram.c \
                  stm32f7xx_hal_timebase_rtc_alarm_template.c \
                  stm32f7xx_hal_timebase_rtc_wakeup_template.c \
                  stm32f7xx_hal_timebase_tim_template.c \
                  stm32f7xx_hal_wwdg.c \
                  stm32f7xx_ll_adc.c \
                  stm32f7xx_ll_crc.c \
                  stm32f7xx_ll_dac.c \
                  stm32f7xx_ll_dma.c \
                  stm32f7xx_ll_dma2d.c \
                  stm32f7xx_ll_exti.c \
                  stm32f7xx_ll_fmc.c \
                  stm32f7xx_ll_gpio.c \
                  stm32f7xx_ll_i2c.c \
                  stm32f7xx_ll_lptim.c \
                  stm32f7xx_ll_pwr.c \
                  stm32f7xx_ll_rcc.c \
                  stm32f7xx_ll_rng.c \
                  stm32f7xx_ll_rtc.c \
                  stm32f7xx_ll_sdmmc.c \
                  stm32f7xx_ll_spi.c \
                  stm32f7xx_ll_tim.c \
                  stm32f7xx_ll_usart.c \
                  stm32f7xx_ll_utils.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

#USB
USBCORE_DIR = $(ROOT)/lib/main/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/CM7/Include:$(CMSIS_DIR)/CM7/Device/ST/STM32F7xx
VPATH           := $(VPATH):$(STDPERIPH_DIR)/Src
CMSIS_SRC       = $(notdir $(wildcard $(CMSIS_DIR)/CM7/Include/*.c \
                  $(CMSIS_DIR)/CM7/Device/ST/STM32F7xx/*.c))
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(CMSIS_DIR)/CM7/Include \
                   $(CMSIS_DIR)/CM7/Device/ST/STM32F7xx/Include \
                   $(ROOT)/src/main/vcp_hal

ifneq ($(filter SDCARD,$(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR)
VPATH           := $(VPATH):$(FATFS_DIR)
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsingle-precision-constant -Wdouble-promotion

ifeq ($(TARGET),$(filter $(TARGET),$(F7X5XG_TARGETS)))
DEVICE_FLAGS    = -DSTM32F745xx -DUSE_HAL_DRIVER -D__FPU_PRESENT
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f745.ld
STARTUP_SRC     = startup_stm32f745xx.s
else ifeq ($(TARGET),$(filter $(TARGET),$(F7X6XG_TARGETS)))
DEVICE_FLAGS    = -DSTM32F746xx -DUSE_HAL_DRIVER -D__FPU_PRESENT
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f746.ld
STARTUP_SRC     = startup_stm32f746xx.s
else ifeq ($(TARGET),$(filter $(TARGET),$(F7X2RE_TARGETS)))
DEVICE_FLAGS    = -DSTM32F722xx -DUSE_HAL_DRIVER -D__FPU_PRESENT
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f722.ld
STARTUP_SRC     = startup_stm32f722xx.s
else
$(error Unknown MCU for F7 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

TARGET_FLAGS = -D$(TARGET)

# End F7 targets
#
# Start F1 targets
else

STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f10x_crc.c \
                  stm32f10x_cec.c \
                  stm32f10x_can.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH           := $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC       = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
                  $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/CM3/CoreSupport \
                   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

ifneq ($(filter VCP, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(USBFS_DIR)/inc \
                   $(ROOT)/src/main/vcp

VPATH           := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC) \
                        $(USBPERIPH_SRC)

endif

LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld
ARCH_FLAGS      = -mthumb -mcpu=cortex-m3
TARGET_FLAGS   := -D$(TARGET) -pedantic $(TARGET_FLAGS)

ifeq ($(DEVICE_FLAGS),)
DEVICE_FLAGS    = -DSTM32F10X_MD
endif
DEVICE_FLAGS   += -DSTM32F10X

endif
#
# End F1 targets
#
ifneq ($(BASE_TARGET), $(TARGET))
TARGET_FLAGS  := $(TARGET_FLAGS) -D$(BASE_TARGET)
endif

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

ifneq ($(HSE_VALUE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DHSE_VALUE=$(HSE_VALUE)
endif

TARGET_DIR     = $(ROOT)/src/main/target/$(BASE_TARGET)
TARGET_DIR_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

ifeq ($(OPBL),yes)
TARGET_FLAGS := -DOPBL $(TARGET_FLAGS)
ifeq ($(TARGET), $(filter $(TARGET),$(F405_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f405_opbl.ld
else ifeq ($(TARGET), $(filter $(TARGET),$(F411_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f411_opbl.ld
else ifeq ($(TARGET), $(filter $(TARGET),$(F3_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k_opbl.ld
else ifeq ($(TARGET), $(filter $(TARGET),$(F1_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k_opbl.ld
endif
.DEFAULT_GOAL := binary
else
.DEFAULT_GOAL := hex
endif

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(ROOT)/lib/main/MAVLink

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_DIR)

VPATH           := $(VPATH):$(TARGET_DIR)

COMMON_SRC = \
            $(TARGET_DIR_SRC) \
            main.c \
            build/assert.c \
            build/build_config.c \
            build/debug.c \
            build/version.c \
            common/bitarray.c \
            common/crc.c \
            common/encoding.c \
            common/filter.c \
            common/maths.c \
            common/printf.c \
            common/streambuf.c \
            common/typeconversion.c \
            common/string_light.c \
            config/config_eeprom.c \
            config/config_streamer.c \
            config/feature.c \
            config/parameter_group.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_spi.c \
            drivers/bus_spi_soft.c \
            drivers/display.c \
            drivers/exti.c \
            drivers/gps_i2cnav.c \
            drivers/gyro_sync.c \
            drivers/io.c \
            drivers/io_pca9685.c \
            drivers/light_led.c \
            drivers/logging.c \
            drivers/rx_nrf24l01.c \
            drivers/rx_spi.c \
            drivers/rx_xn297.c \
            drivers/pitotmeter_adc.c \
            drivers/pwm_esc_detect.c \
            drivers/pwm_mapping.c \
            drivers/pwm_output.c \
            drivers/rcc.c \
            drivers/rx_pwm.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/sound_beeper.c \
            drivers/stack_check.c \
            drivers/system.c \
            drivers/timer.c \
            fc/cli.c \
            fc/config.c \
            fc/controlrate_profile.c \
            fc/fc_core.c \
            fc/fc_init.c \
            fc/fc_tasks.c \
            fc/fc_hardfaults.c \
            fc/fc_msp.c \
            fc/rc_adjustments.c \
            fc/rc_controls.c \
            fc/rc_curves.c \
            fc/rc_modes.c \
            fc/runtime_config.c \
            fc/settings.c \
            fc/stats.c \
            flight/failsafe.c \
            flight/hil.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            flight/pid_autotune.c \
            flight/servos.c \
            io/beeper.c \
            io/pwmdriver_i2c.c \
            io/serial.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            io/statusindicator.c \
            io/rcsplit.c \
            msp/msp_serial.c \
            rx/ibus.c \
            rx/jetiexbus.c \
            rx/msp.c \
            rx/nrf24_cx10.c \
            rx/nrf24_inav.c \
            rx/nrf24_h8_3d.c \
            rx/nrf24_syma.c \
            rx/nrf24_v202.c \
            rx/pwm.c \
            rx/rx.c \
            rx/rx_spi.c \
            rx/crsf.c \
            rx/sbus.c \
            rx/spektrum.c \
            rx/sumd.c \
            rx/sumh.c \
            rx/xbus.c \
            rx/eleres.c \
            scheduler/scheduler.c \
            sensors/acceleration.c \
            sensors/battery.c \
            sensors/boardalignment.c \
            sensors/compass.c \
            sensors/diagnostics.c \
            sensors/gyro.c \
            sensors/initialisation.c \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

HIGHEND_SRC = \
            blackbox/blackbox.c \
            blackbox/blackbox_encoding.c \
            blackbox/blackbox_io.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_builtin.c \
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            common/colorconversion.c \
            common/gps_conversion.c \
            drivers/display_ug2864hsweg01.c \
            drivers/rangefinder_hcsr04.c \
            drivers/rangefinder_hcsr04_i2c.c \
            drivers/rangefinder_srf10.c \
            drivers/rangefinder_vl53l0x.c \
            io/dashboard.c \
            io/displayport_max7456.c \
            io/displayport_msp.c \
            io/displayport_oled.c \
            io/gps.c \
            io/gps_ublox.c \
            io/gps_nmea.c \
            io/gps_naza.c \
            io/gps_i2cnav.c \
            io/ledstrip.c \
            io/osd.c \
            navigation/navigation.c \
            navigation/navigation_fixedwing.c \
            navigation/navigation_fw_launch.c \
            navigation/navigation_geo.c \
            navigation/navigation_multicopter.c \
            navigation/navigation_pos_estimator.c \
            sensors/barometer.c \
            sensors/pitotmeter.c \
            sensors/rangefinder.c \
            telemetry/crsf.c \
            telemetry/frsky.c \
            telemetry/hott.c \
	    telemetry/ibus_shared.c \
            telemetry/ibus.c \
            telemetry/ltm.c \
            telemetry/mavlink.c \
            telemetry/smartport.c \
            telemetry/telemetry.c

ifeq ($(TARGET),$(filter $(TARGET),$(F4_TARGETS)))
VCP_SRC = \
            vcpf4/stm32f4xx_it.c \
            vcpf4/usb_bsp.c \
            vcpf4/usbd_desc.c \
            vcpf4/usbd_usr.c \
            vcpf4/usbd_cdc_vcp.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c
else ifeq ($(TARGET),$(filter $(TARGET),$(F7_TARGETS)))
VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf.c \
            vcp_hal/usbd_cdc_interface.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c
else
VCP_SRC = \
            vcp/hw_config.c \
            vcp/stm32_it.c \
            vcp/usb_desc.c \
            vcp/usb_endp.c \
            vcp/usb_istr.c \
            vcp/usb_prop.c \
            vcp/usb_pwr.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c
endif

STM32F10x_COMMON_SRC = \
            startup_stm32f10x_md_gcc.S \
            drivers/adc_stm32f10x.c \
            drivers/bus_i2c_stm32f10x.c \
            drivers/dma.c \
            drivers/gpio_stm32f10x.c \
            drivers/inverter.c \
            drivers/serial_softserial.c \
            drivers/serial_uart_stm32f10x.c \
            drivers/system_stm32f10x.c \
            drivers/timer_stm32f10x.c

STM32F30x_COMMON_SRC = \
            startup_stm32f30x_md_gcc.S \
            target/system_stm32f30x.c \
            drivers/adc_stm32f30x.c \
            drivers/bus_i2c_stm32f30x.c \
            drivers/dma.c \
            drivers/gpio_stm32f30x.c \
            drivers/serial_uart_stm32f30x.c \
            drivers/system_stm32f30x.c \
            drivers/timer_stm32f30x.c

STM32F4xx_COMMON_SRC = \
            startup_stm32f40xx.s \
            target/system_stm32f4xx.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/adc_stm32f4xx.c \
            drivers/adc_stm32f4xx.c \
            drivers/bus_i2c_stm32f10x.c \
            drivers/gpio_stm32f4xx.c \
            drivers/inverter.c \
            drivers/serial_softserial.c \
            drivers/serial_uart_stm32f4xx.c \
            drivers/system_stm32f4xx.c \
            drivers/timer_stm32f4xx.c \
            drivers/dma_stm32f4xx.c

STM32F7xx_COMMON_SRC = \
            target/system_stm32f7xx.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/adc_stm32f7xx.c \
            drivers/bus_i2c_hal.c \
            drivers/dma_stm32f7xx.c \
            drivers/gpio_stm32f7xx.c \
            drivers/inverter.c \
            drivers/bus_spi_hal.c \
            drivers/timer_hal.c \
            drivers/timer_stm32f7xx.c \
            drivers/system_stm32f7xx.c \
            drivers/serial_uart_stm32f7xx.c \
            drivers/serial_uart_hal.c

F7EXCLUDES = drivers/bus_spi.c \
            drivers/bus_i2c.c \
            drivers/timer.c \
            drivers/serial_uart.c

# check if target.mk supplied
ifeq ($(TARGET),$(filter $(TARGET),$(F4_TARGETS)))
TARGET_SRC := $(STM32F4xx_COMMON_SRC) $(TARGET_SRC)
else ifeq ($(TARGET),$(filter $(TARGET),$(F7_TARGETS)))
TARGET_SRC := $(STARTUP_SRC) $(STM32F7xx_COMMON_SRC) $(TARGET_SRC)
else ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
TARGET_SRC := $(STM32F30x_COMMON_SRC) $(TARGET_SRC)
else ifeq ($(TARGET),$(filter $(TARGET),$(F1_TARGETS)))
TARGET_SRC := $(STM32F10x_COMMON_SRC) $(TARGET_SRC)
endif

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
TARGET_SRC += \
            drivers/flash_m25p16.c \
            io/flashfs.c
endif

ifeq ($(TARGET),$(filter $(TARGET),$(F7_TARGETS) $(F4_TARGETS) $(F3_TARGETS)))
TARGET_SRC += $(HIGHEND_SRC)
else ifneq ($(filter HIGHEND,$(FEATURES)),)
TARGET_SRC += $(HIGHEND_SRC)
endif

TARGET_SRC += $(COMMON_SRC)
#excludes
ifeq ($(TARGET),$(filter $(TARGET),$(F7_TARGETS)))
TARGET_SRC   := $(filter-out ${F7EXCLUDES}, $(TARGET_SRC))
endif

ifneq ($(filter SDCARD,$(FEATURES)),)
TARGET_SRC += \
            drivers/sdcard.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c
endif

ifneq ($(filter VCP,$(FEATURES)),)
TARGET_SRC += $(VCP_SRC)
endif
# end target specific make file checks

# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
ifneq ($(TOOLCHAINPATH),)
CROSS_CC    = $(TOOLCHAINPATH)/arm-none-eabi-gcc
OBJCOPY     = $(TOOLCHAINPATH)/arm-none-eabi-objcopy
SIZE        = $(TOOLCHAINPATH)/arm-none-eabi-size
else
CROSS_CC    = arm-none-eabi-gcc
OBJCOPY     = arm-none-eabi-objcopy
SIZE        = arm-none-eabi-size
endif

#
# Tool options.
#

ifeq ($(DEBUG),GDB)
OPTIMIZE    = -O0
LTO_FLAGS   = $(OPTIMIZE)
else
OPTIMIZE    = -Os
LTO_FLAGS   = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS = -ggdb3 -DDEBUG

CFLAGS      += $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(addprefix -D,$(OPTIONS)) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              $(DEBUG_FLAGS) \
              -std=gnu99 \
              -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
              -ffunction-sections \
              -fdata-sections \
              $(DEVICE_FLAGS) \
              -DUSE_STDPERIPH_DRIVER \
              $(TARGET_FLAGS) \
              -D'__FORKNAME__="$(FORKNAME)"' \
              -D'__TARGET__="$(TARGET)"' \
              -D'__REVISION__="$(REVISION)"' \
              -save-temps=obj \
              -MMD -MP

ASFLAGS     = $(ARCH_FLAGS) \
              -x assembler-with-cpp \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              -D$(TARGET) \
              -MMD -MP

LDFLAGS     = -lm \
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
              -T$(LD_SCRIPT)

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
TARGET_OBJS     = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(TARGET_SRC))))
TARGET_DEPS     = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(TARGET_SRC))))
TARGET_MAP      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map


CLEAN_ARTIFACTS := $(TARGET_BIN)
CLEAN_ARTIFACTS += $(TARGET_HEX)
CLEAN_ARTIFACTS += $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)

# Make sure build date and revision is updated on every incremental build
$(OBJECT_DIR)/$(TARGET)/build/version.o : $(TARGET_SRC)

# Settings generator
.PHONY: settings clean-settings
UTILS_DIR			= $(ROOT)/src/utils
SETTINGS_GENERATOR	= $(UTILS_DIR)/settings.rb

GENERATED_SETTINGS	= $(SRC_DIR)/fc/settings_generated.h $(SRC_DIR)/fc/settings_generated.c
SETTINGS_FILE 		= $(SRC_DIR)/fc/settings.yaml
$(GENERATED_SETTINGS): $(SETTINGS_GENERATOR) $(SETTINGS_FILE)

# Use a pattern rule, since they're different than normal rules.
# See https://www.gnu.org/software/make/manual/make.html#Pattern-Examples
%generated.h %generated.c:
	$(V1) echo "settings.yaml -> settings_generated.h, settings_generated.c" "$(STDOUT)"
	$(V1) CFLAGS="$(CFLAGS)" ruby $(SETTINGS_GENERATOR) . $(SETTINGS_FILE)

settings: $(GENERATED_SETTINGS)
clean-settings:
	$(V1) $(RM) $(GENERATED_SETTINGS)

# Files that depend on the generated settings
$(OBJECT_DIR)/$(TARGET)/fc/cli.o: settings
$(OBJECT_DIR)/$(TARGET)/fc/settings.o: settings

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(V0) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(V0) $(OBJCOPY) -O binary $< $@

$(TARGET_ELF): clean-settings $(TARGET_OBJS)
	$(V1) echo Linking $(TARGET)
	$(V1) $(CROSS_CC) -o $@ $(filter %.o, $^) $(LDFLAGS)
	$(V0) $(SIZE) $(TARGET_ELF)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) echo %% $(notdir $<) "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	$(V1) echo %% $(notdir $<) "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	$(V1) echo %% $(notdir $<) "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<


## all               : Build all valid targets
all: $(VALID_TARGETS)

$(VALID_TARGETS):
	$(V0) echo "" && \
	echo "Building $@" && \
	$(MAKE) -j 4 TARGET=$@ && \
	echo "Building $@ succeeded."

## clean             : clean up all temporary / machine-generated files
clean:
	$(V0) echo "Cleaning $(TARGET)"
	$(V0) rm -f $(CLEAN_ARTIFACTS)
	$(V0) rm -rf $(OBJECT_DIR)/$(TARGET)
	$(V0) rm -f $(GENERATED_SETTINGS)
	$(V0) echo "Cleaning $(TARGET) succeeded."

## clean_test        : clean up all temporary / machine-generated files (tests)
clean_test:
	$(V0) cd src/test && $(MAKE) clean || true

## clean_<TARGET>    : clean up one specific target
$(CLEAN_TARGETS) :
	$(V0) $(MAKE) -j 4 TARGET=$(subst clean_,,$@) clean

## <TARGET>_clean    : clean up one specific target (alias for above)
$(TARGETS_CLEAN) :
	$(V0) $(MAKE) -j 4 TARGET=$(subst _clean,,$@) clean

## clean_all         : clean all valid targets
clean_all:$(CLEAN_TARGETS)

## all_clean         : clean all valid targets (alias for above)
all_clean:$(TARGETS_CLEAN)

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

binary: $(TARGET_BIN)
hex:    $(TARGET_HEX)

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

## help              : print this help message and exit
help: Makefile
	$(V0) @echo ""
	$(V0) @echo "Makefile for the $(FORKNAME) firmware"
	$(V0) @echo ""
	$(V0) @echo "Usage:"
	$(V0) @echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	$(V0) @echo "Or:"
	$(V0) @echo "        make <target> [OPTIONS=\"<options>\"]"
	$(V0) @echo ""
	$(V0) @echo "Valid TARGET values are: $(VALID_TARGETS)"
	$(V0) @echo ""
	$(V0) @sed -n 's/^## //p' $<

## targets           : print a list of all valid target platforms (for consumption by scripts)
targets:
	$(V0) @echo "Valid targets: $(VALID_TARGETS)"
	$(V0) @echo "Target:        $(TARGET)"
	$(V0) @echo "Base target:   $(BASE_TARGET)"

## test              : run the cleanflight test suite
test:
	$(V0) cd src/test && $(MAKE) test || true

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)
