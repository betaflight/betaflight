###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the betaflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#
###############################################################################


# Things that the user might override on the commandline
#

# The target to build, see VALID_TARGETS below
TARGET    ?= NAZE

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

FORKNAME      = betaflight

# Working directories
ROOT            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR         = $(ROOT)/src/main
OBJECT_DIR      = $(ROOT)/obj/main
BIN_DIR         = $(ROOT)/obj
CMSIS_DIR       = $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS    = $(SRC_DIR) \
                  $(ROOT)/src/main/target
LINKER_DIR      = $(ROOT)/src/main/target/link

# Build tools, so we all share the same versions
# import macros common to all supported build systems
include $(ROOT)/make/system-id.mk

# developer preferences, edit these at will, they'll be gitignored
-include $(ROOT)/make/local.mk

# configure some directories that are relative to wherever ROOT_DIR is located
TOOLS_DIR := $(ROOT)/tools
BUILD_DIR := $(ROOT)/build
DL_DIR := $(ROOT)/downloads

export RM := rm

# import macros that are OS specific
include $(ROOT)/make/$(OSFAMILY).mk

# include the tools makefile
include $(ROOT)/make/tools.mk

# default xtal value for F4 targets
HSE_VALUE       = 8000000

# used for turning on features like VCP and SDCARD
FEATURES        =

SAMPLE_TARGETS  = ALIENFLIGHTF3 ALIENFLIGHTF4 ANYFCF7 BETAFLIGHTF3 BLUEJAYF4 CC3D FURYF4 NAZE REVO SIRINFPV SPARKY SPRACINGF3 SPRACINGF3EVO STM32F3DISCOVERY
ALT_TARGETS     = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.mk)))))
OPBL_TARGETS    = $(filter %_OPBL, $(ALT_TARGETS))

#VALID_TARGETS  = $(F1_TARGETS) $(F3_TARGETS) $(F4_TARGETS)
VALID_TARGETS   = $(dir $(wildcard $(ROOT)/src/main/target/*/target.mk))
VALID_TARGETS  := $(subst /,, $(subst ./src/main/target/,, $(VALID_TARGETS)))
VALID_TARGETS  := $(VALID_TARGETS) $(ALT_TARGETS)
VALID_TARGETS  := $(sort $(VALID_TARGETS))

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

F4_TARGETS      = $(F405_TARGETS) $(F411_TARGETS)
F7_TARGETS      = $(F7X5XE_TARGETS) $(F7X5XG_TARGETS) $(F7X5XI_TARGETS) $(F7X6XG_TARGETS)

ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS). Have you prepared a valid target.mk?)
endif

ifeq ($(filter $(TARGET),$(F1_TARGETS) $(F3_TARGETS) $(F4_TARGETS) $(F7_TARGETS)),)
$(error Target '$(TARGET)' has not specified a valid STM group, must be one of F1, F3, F405, F411 or F7x5. Have you prepared a valid target.mk?)
endif

128K_TARGETS  = $(F1_TARGETS)
256K_TARGETS  = $(F3_TARGETS)
512K_TARGETS  = $(F411_TARGETS) $(F7X5XE_TARGETS)
1024K_TARGETS = $(F405_TARGETS) $(F7X5XG_TARGETS) $(F7X6XG_TARGETS)
2048K_TARGETS = $(F7X5XI_TARGETS)

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

REVISION := $(shell git log -1 --format="%h")

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
else
$(error Unknown MCU for F4 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

# End F4 targets
#
# Start F7 targets
else ifeq ($(TARGET),$(filter $(TARGET), $(F7_TARGETS)))

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F7xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        = stm32f7xx_hal_timebase_rtc_wakeup_template.c \
				  stm32f7xx_hal_timebase_rtc_alarm_template.c \
				  stm32f7xx_hal_timebase_tim_template.c

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
else ifeq ($(TARGET),$(filter $(TARGET),$(F7X6XG_TARGETS)))
DEVICE_FLAGS    = -DSTM32F746xx -DUSE_HAL_DRIVER -D__FPU_PRESENT
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f746.ld
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
            build/build_config.c \
            build/debug.c \
            build/version.c \
            $(TARGET_DIR_SRC) \
            main.c \
            common/encoding.c \
            common/filter.c \
            common/maths.c \
            common/printf.c \
            common/streambuf.c \
            common/typeconversion.c \
            config/config_eeprom.c \
            config/feature.c \
            config/parameter_group.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_spi.c \
            drivers/bus_spi_soft.c \
            drivers/display.c \
            drivers/exti.c \
            drivers/gyro_sync.c \
            drivers/io.c \
            drivers/light_led.c \
            drivers/resource.c \
            drivers/rx_nrf24l01.c \
            drivers/rx_spi.c \
            drivers/rx_xn297.c \
            drivers/pwm_esc_detect.c \
            drivers/pwm_output.c \
            drivers/rcc.c \
            drivers/rx_pwm.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/sound_beeper.c \
            drivers/stack_check.c \
            drivers/system.c \
            drivers/timer.c \
            fc/config.c \
            fc/fc_tasks.c \
            fc/fc_msp.c \
            fc/mw.c \
            fc/rc_controls.c \
            fc/rc_curves.c \
            fc/runtime_config.c \
            flight/altitudehold.c \
            flight/failsafe.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            flight/servos.c \
            io/beeper.c \
            io/serial.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            io/serial_cli.c \
            io/statusindicator.c \
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
            scheduler/scheduler.c \
            sensors/acceleration.c \
            sensors/battery.c \
            sensors/boardalignment.c \
            sensors/compass.c \
            sensors/gyro.c \
            sensors/initialisation.c \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

HIGHEND_SRC = \
            blackbox/blackbox.c \
            blackbox/blackbox_io.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_builtin.c \
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_vtx.c \
            common/colorconversion.c \
            drivers/display_ug2864hsweg01.c \
            drivers/light_ws2811strip.c \
            drivers/serial_escserial.c \
            drivers/serial_softserial.c \
            drivers/sonar_hcsr04.c \
            flight/gtune.c \
            flight/navigation.c \
            flight/gps_conversion.c \
            io/dashboard.c \
            io/displayport_max7456.c \
            io/displayport_msp.c \
            io/displayport_oled.c \
            io/gps.c \
            io/ledstrip.c \
            io/osd.c \
            sensors/sonar.c \
            sensors/barometer.c \
            telemetry/telemetry.c \
            telemetry/crsf.c \
            telemetry/srxl.c \
            telemetry/frsky.c \
            telemetry/hott.c \
            telemetry/smartport.c \
            telemetry/ltm.c \
            telemetry/mavlink.c \
            sensors/esc_sensor.c \

SPEED_OPTIMISED_SRC := ""
SIZE_OPTIMISED_SRC  := ""

ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
            common/encoding.c \
            common/filter.c \
            common/maths.c \
            common/typeconversion.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_spi.c \
            drivers/bus_spi_soft.c \
            drivers/exti.c \
            drivers/gyro_sync.c \
            drivers/io.c \
            drivers/light_led.c \
            drivers/resource.c \
            drivers/rx_nrf24l01.c \
            drivers/rx_spi.c \
            drivers/rx_xn297.c \
            drivers/pwm_output.c \
            drivers/rcc.c \
            drivers/rx_pwm.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/sound_beeper.c \
            drivers/stack_check.c \
            drivers/system.c \
            drivers/timer.c \
            fc/fc_tasks.c \
            fc/mw.c \
            fc/rc_controls.c \
            fc/rc_curves.c \
            fc/runtime_config.c \
            flight/altitudehold.c \
            flight/failsafe.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            flight/servos.c \
            io/beeper.c \
            io/serial.c \
            io/statusindicator.c \
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
            scheduler/scheduler.c \
            sensors/acceleration.c \
            sensors/boardalignment.c \
            sensors/gyro.c \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC) \
            blackbox/blackbox.c \
            blackbox/blackbox_io.c \
            drivers/display_ug2864hsweg01.c \
            drivers/light_ws2811strip.c \
            drivers/serial_softserial.c \
            io/dashboard.c \
            io/displayport_max7456.c \
            io/displayport_msp.c \
            io/displayport_oled.c \
            io/ledstrip.c \
            io/osd.c \
            telemetry/telemetry.c \
            telemetry/crsf.c \
            telemetry/frsky.c \
            telemetry/hott.c \
            telemetry/smartport.c \
            telemetry/ltm.c \
            telemetry/mavlink.c \
            telemetry/esc_telemetry.c \

SIZE_OPTIMISED_SRC := $(SIZE_OPTIMISED_SRC) \
            drivers/serial_escserial.c \
            io/serial_cli.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            msp/msp_serial.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_builtin.c \
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_vtx.c
endif #F3

ifeq ($(TARGET),$(filter $(TARGET),$(F4_TARGETS)))
VCP_SRC = \
            vcpf4/stm32f4xx_it.c \
            vcpf4/usb_bsp.c \
            vcpf4/usbd_desc.c \
            vcpf4/usbd_usr.c \
            vcpf4/usbd_cdc_vcp.c \
            drivers/serial_usb_vcp.c
else ifeq ($(TARGET),$(filter $(TARGET),$(F7_TARGETS)))
VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf.c \
            vcp_hal/usbd_cdc_interface.c \
            drivers/serial_usb_vcp.c
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
            drivers/light_ws2811strip_stm32f10x.c \
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
            drivers/light_ws2811strip_stm32f30x.c \
            drivers/pwm_output_stm32f3xx.c \
            drivers/serial_uart_stm32f30x.c \
            drivers/system_stm32f30x.c \
            drivers/timer_stm32f30x.c

STM32F4xx_COMMON_SRC = \
            startup_stm32f40xx.s \
            target/system_stm32f4xx.c \
            drivers/accgyro_mpu.c \
            drivers/adc_stm32f4xx.c \
            drivers/bus_i2c_stm32f10x.c \
            drivers/dma_stm32f4xx.c \
            drivers/gpio_stm32f4xx.c \
            drivers/inverter.c \
            drivers/light_ws2811strip_stm32f4xx.c \
            drivers/pwm_output_stm32f4xx.c \
            drivers/serial_uart_stm32f4xx.c \
            drivers/system_stm32f4xx.c \
            drivers/timer_stm32f4xx.c

STM32F7xx_COMMON_SRC = \
            startup_stm32f745xx.s \
            target/system_stm32f7xx.c \
            drivers/accgyro_mpu.c \
            drivers/adc_stm32f7xx.c \
            drivers/bus_i2c_hal.c \
            drivers/dma_stm32f7xx.c \
            drivers/gpio_stm32f7xx.c \
            drivers/inverter.c \
            drivers/bus_spi_hal.c \
            drivers/pwm_output_stm32f7xx.c \
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
TARGET_SRC := $(STM32F7xx_COMMON_SRC) $(TARGET_SRC)
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

# Find out if ccache is installed on the system
CCACHE := ccache
RESULT = $(shell (which $(CCACHE) > /dev/null 2>&1; echo $$?) )
ifneq ($(RESULT),0)
CCACHE :=
endif

# Tool names
CROSS_CC    := $(CCACHE) $(ARM_SDK_PREFIX)gcc
CROSS_CXX   := $(CCACHE) $(ARM_SDK_PREFIX)g++
OBJCOPY     := $(ARM_SDK_PREFIX)objcopy
SIZE        := $(ARM_SDK_PREFIX)size

#
# Tool options.
#

ifneq ($(DEBUG),GDB)
OPTIMISATION_BASE   := -flto -fuse-linker-plugin -ffast-math
OPTIMISE_SPEED      := ""
OPTIMISE_SIZE       := ""

ifeq ($(TARGET),$(filter $(TARGET),$(F1_TARGETS)))
OPTIMISE_DEFAULT    := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)

else ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
OPTIMISE_DEFAULT    := -O2
OPTIMISE_SPEED      := -Ofast
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)

else
OPTIMISE_DEFAULT    := -Ofast

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)

endif #TARGETS

CC_DEFAULT_OPTIMISATION := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
CC_SPEED_OPTIMISATION   := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
CC_SIZE_OPTIMISATION    := $(OPTIMISATION_BASE) $(OPTIMISE_SIZE)

else #DEBUG
OPTIMISE_DEFAULT    := -O0

CC_DEBUG_OPTIMISATION := $(OPTIMISE_DEFAULT)

LTO_FLAGS           := $(OPTIMISE_DEFAULT)
endif #DEBUG

DEBUG_FLAGS = -ggdb3 -DDEBUG

CFLAGS      += $(ARCH_FLAGS) \
              $(addprefix -D,$(OPTIONS)) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              $(DEBUG_FLAGS) \
              -std=gnu99 \
              -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
              -ffunction-sections \
              -fdata-sections \
              -pedantic \
              $(DEVICE_FLAGS) \
              -DUSE_STDPERIPH_DRIVER \
              -D$(TARGET) \
              $(TARGET_FLAGS) \
              -D'__FORKNAME__="$(FORKNAME)"' \
              -D'__TARGET__="$(TARGET)"' \
              -D'__REVISION__="$(REVISION)"' \
              -save-temps=obj \
              -MMD -MP

ASFLAGS     = $(ARCH_FLAGS) \
              -x assembler-with-cpp \
              $(addprefix -I,$(INCLUDE_DIRS)) \
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

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(V0) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(V0) $(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(V1) echo Linking $(TARGET)
	$(V1) $(CROSS_CC) -o $@ $^ $(LDFLAGS)
	$(V0) $(SIZE) $(TARGET_ELF)

# Compile
ifneq ($(DEBUG),GDB)
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $(subst ./src/main/,,$<),$(SPEED_OPTIMISED_SRC)), \
	echo "%% (speed optimised) $(notdir $<)" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_SPEED_OPTIMISATION) $<, \
	$(if $(findstring $(subst ./src/main/,,$<),$(SIZE_OPTIMISED_SRC)), \
	echo "%% (size optimised) $(notdir $<)" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_SIZE_OPTIMISATION) $<, \
	echo "%% $(notdir $<)" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_DEFAULT_OPTIMISATION) $<))
else
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% $(notdir $<)" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_DEBUG_OPTIMISATION) $<
endif

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

## sample            : Build all sample (travis) targets
sample: $(SAMPLE_TARGETS)

## all               : Build all valid targets
all: $(VALID_TARGETS)

$(VALID_TARGETS):
		$(V0) echo "" && \
		echo "Building $@" && \
		$(MAKE) binary hex TARGET=$@ && \
		echo "Building $@ succeeded."



CLEAN_TARGETS = $(addprefix clean_,$(VALID_TARGETS) )
TARGETS_CLEAN = $(addsuffix _clean,$(VALID_TARGETS) )

## clean             : clean up temporary / machine-generated files
clean:
	$(V0) echo "Cleaning $(TARGET)"
	$(V0) rm -f $(CLEAN_ARTIFACTS)
	$(V0) rm -rf $(OBJECT_DIR)/$(TARGET)
	$(V0) echo "Cleaning $(TARGET) succeeded."

## clean_test        : clean up temporary / machine-generated files (tests)
clean_test:
	$(V0) cd src/test && $(MAKE) clean || true

## clean_<TARGET>    : clean up one specific target
$(CLEAN_TARGETS) :
	$(V0) $(MAKE) -j TARGET=$(subst clean_,,$@) clean

## <TARGET>_clean    : clean up one specific target (alias for above)
$(TARGETS_CLEAN) :
	$(V0) $(MAKE) -j TARGET=$(subst _clean,,$@) clean

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

binary:
	$(V0) $(MAKE) -j $(TARGET_BIN)

hex:
	$(V0) $(MAKE) -j $(TARGET_HEX)

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

# mkdirs
$(DL_DIR):
	mkdir -p $@

$(TOOLS_DIR):
	mkdir -p $@

$(BUILD_DIR):
	mkdir -p $@

## help              : print this help message and exit
help: Makefile make/tools.mk
	$(V0) @echo ""
	$(V0) @echo "Makefile for the $(FORKNAME) firmware"
	$(V0) @echo ""
	$(V0) @echo "Usage:"
	$(V0) @echo "        make [V=<verbosity>] [TARGET=<target>] [OPTIONS=\"<options>\"]"
	$(V0) @echo "Or:"
	$(V0) @echo "        make <target> [V=<verbosity>] [OPTIONS=\"<options>\"]"
	$(V0) @echo ""
	$(V0) @echo "Valid TARGET values are: $(VALID_TARGETS)"
	$(V0) @echo ""
	$(V0) @sed -n 's/^## //p' $?

## targets           : print a list of all valid target platforms (for consumption by scripts)
targets:
	$(V0) @echo "Valid targets: $(VALID_TARGETS)"
	$(V0) @echo "Target:        $(TARGET)"
	$(V0) @echo "Base target:   $(BASE_TARGET)"

## test              : run the cleanflight test suite
## junittest         : run the cleanflight test suite, producing Junit XML result files.
test junittest:
	$(V0) cd src/test && $(MAKE) $@

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)
