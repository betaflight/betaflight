###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the cleanflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#

###############################################################################
# Things that the user might override on the commandline
#

# The target to build, see VALID_TARGETS below
TARGET		?= NAZE

# Compile-time options
OPTIONS		?=
export OPTIONS

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME			 = cleanflight

64K_TARGETS  = CJMCU
128K_TARGETS = ALIENFLIGHTF1 CC3D NAZE OLIMEXINO RMDO SPRACINGF1OSD
256K_TARGETS = ALIENFLIGHTF3 CHEBUZZF3 COLIBRI_RACE EUSTM32F103RC IRCFUSIONF3 LUX_RACE MOTOLAB PORT103R RCEXPLORERF3 SPARKY SPRACINGF3 SPRACINGF3EVO SPRACINGF3MINI STM32F3DISCOVERY SPRACINGF3OSD

F3_TARGETS = ALIENFLIGHTF3 CHEBUZZF3 COLIBRI_RACE IRCFUSIONF3 LUX_RACE MOTOLAB RCEXPLORERF3 RMDO SPARKY SPRACINGF3 SPRACINGF3EVO SPRACINGF3MINI STM32F3DISCOVERY SPRACINGF3OSD

VALID_TARGETS = $(64K_TARGETS) $(128K_TARGETS) $(256K_TARGETS)

VCP_TARGETS = CC3D ALIENFLIGHTF3 CHEBUZZF3 COLIBRI_RACE LUX_RACE MOTOLAB RCEXPLORERF3 SPARKY SPRACINGF3EVO SPRACINGF3MINI STM32F3DISCOVERY SPRACINGF1OSD SPRACINGF3OSD
OSD_TARGETS = SPRACINGF1OSD SPRACINGF3OSD

# Configure default flash sizes for the targets
ifeq ($(FLASH_SIZE),)
ifeq ($(TARGET),$(filter $(TARGET),$(64K_TARGETS)))
FLASH_SIZE = 64
else ifeq ($(TARGET),$(filter $(TARGET),$(128K_TARGETS)))
FLASH_SIZE = 128
else ifeq ($(TARGET),$(filter $(TARGET),$(256K_TARGETS)))
FLASH_SIZE = 256
else
$(error FLASH_SIZE not configured for target $(TARGET))
endif
endif

REVISION := $(shell git log -1 --format="%h")

# Working directories
ROOT		 := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR		 = $(ROOT)/src/main
OBJECT_DIR	 = $(ROOT)/obj/main
BIN_DIR		 = $(ROOT)/obj
CMSIS_DIR	 = $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS	 = $(SRC_DIR) \
				$(ROOT)/src/main/target
LINKER_DIR	 = $(ROOT)/src/main/target

# Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR	= $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))

CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
# F3 TARGETS

STDPERIPH_DIR	= $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f30x_crc.c \
		stm32f30x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

DEVICE_STDPERIPH_SRC = \
		$(STDPERIPH_SRC)


VPATH		:= $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM1/CoreSupport \
		   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x

ifeq ($(TARGET),$(filter $(TARGET),$(VCP_TARGETS)))
INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(USBFS_DIR)/inc \
		   $(ROOT)/src/main/vcp

VPATH := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
		   $(USBPERIPH_SRC)

endif

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS = -D$(TARGET)

else ifeq ($(TARGET),$(filter $(TARGET),EUSTM32F103RC PORT103R))
# TARGETS: EUSTM32F103RC PORT103R


STDPERIPH_DIR	 = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f10x_crc.c \
		stm32f10x_cec.c \
		stm32f10x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
TARGET_FLAGS = -D$(TARGET) -pedantic
DEVICE_FLAGS = -DSTM32F10X_HD -DSTM32F10X

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

else
# F1 TARGETS

STDPERIPH_DIR	 = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f10x_crc.c \
		stm32f10x_cec.c \
		stm32f10x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

ifeq ($(TARGET),$(filter $(TARGET),$(VCP_TARGETS)))
INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(USBFS_DIR)/inc \
		   $(ROOT)/src/main/vcp

VPATH := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC) \
		   $(USBPERIPH_SRC)

endif

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
TARGET_FLAGS = -D$(TARGET) -pedantic
DEVICE_FLAGS = -DSTM32F10X_MD -DSTM32F10X

endif #TARGETS

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

TARGET_DIR = $(ROOT)/src/main/target/$(TARGET)
TARGET_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

# VARIANTS
ifeq ($(TARGET),ALIENFLIGHTF1)
# ALIENFLIGHTF1 is a VARIANT of NAZE
TARGET_FLAGS := $(TARGET_FLAGS) -DNAZE -DALIENFLIGHT
TARGET_DIR = $(ROOT)/src/main/target/NAZE
endif
ifeq ($(TARGET),CHEBUZZF3)
# CHEBUZZ is a VARIANT of STM32F3DISCOVERY
TARGET_FLAGS := $(TARGET_FLAGS) -DSTM32F3DISCOVERY
endif
ifeq ($(TARGET),$(filter $(TARGET),RMDO IRCFUSIONF3))
# RMDO and IRCFUSIONF3 are a VARIANT of SPRACINGF3
TARGET_FLAGS := $(TARGET_FLAGS) -DSPRACINGF3
endif

# OSDs
ifeq ($(TARGET),$(filter $(TARGET),$(OSD_TARGETS)))
TARGET_FLAGS := $(TARGET_FLAGS) -DOSD
endif



INCLUDE_DIRS := $(INCLUDE_DIRS) \
		    $(TARGET_DIR)

VPATH		:= $(VPATH):$(TARGET_DIR)

SYSTEM_SRC = \
		   build/build_config.c \
		   build/debug.c \
		   build/version.c \
		   config/config_streamer.c \
		   config/parameter_group.c \
		   config/config_eeprom.c \
		   common/encoding.c \
		   common/filter.c \
		   common/maths.c \
		   common/printf.c \
		   common/streambuf.c \
		   common/typeconversion.c \
		   drivers/buf_writer.c \
		   drivers/dma.c \
		   drivers/serial.c \
		   drivers/system.c \
		   scheduler/scheduler.c \
		   io/serial.c \
		   io/statusindicator.c \
		   msp/msp.c \
		   msp/msp_serial.c \
		   $(TARGET_SRC) \
		   $(CMSIS_SRC) \
		   $(DEVICE_STDPERIPH_SRC)
 
FC_COMMON_SRC = \
		   config/feature.c \
		   config/profile.c \
		   fc/boot.c \
		   fc/cleanflight_fc.c \
		   fc/fc_tasks.c \
		   fc/rate_profile.c \
		   fc/rc_adjustments.c \
		   fc/rc_controls.c \
		   fc/rc_curves.c \
		   fc/fc_serial.c \
		   fc/config.c \
		   fc/runtime_config.c \
		   fc/msp_server_fc.c \
		   flight/altitudehold.c \
		   flight/failsafe.c \
		   flight/pid.c \
		   flight/pid_luxfloat.c \
		   flight/pid_mwrewrite.c \
		   flight/pid_mw23.c \
		   flight/imu.c \
		   flight/mixer.c \
		   flight/servos.c \
		   drivers/bus_i2c_soft.c \
		   drivers/sound_beeper.c \
		   drivers/gyro_sync.c \
		   io/beeper.c \
		   io/gimbal.c \
		   io/motor_and_servo.c \
		   io/serial_4way.c \
		   io/serial_4way_avrootloader.c \
		   io/serial_4way_stk500v2.c \
		   io/serial_cli.c \
		   io/statusindicator.c \
		   rx/rx.c \
		   rx/pwm.c \
		   rx/msp.c \
		   rx/sbus.c \
		   rx/sumd.c \
		   rx/sumh.c \
		   rx/spektrum.c \
		   rx/xbus.c \
		   rx/ibus.c \
		   sensors/sensors.c \
		   sensors/acceleration.c \
		   sensors/battery.c \
		   sensors/boardalignment.c \
		   sensors/compass.c \
		   sensors/gyro.c \
		   sensors/initialisation.c

OSD_COMMON_SRC = \
		   osd/boot.c \
		   osd/cleanflight_osd.c \
		   osd/fc_state.c \
		   osd/config.c \
		   osd/osd.c \
		   osd/osd_serial.c \
		   osd/msp_server_osd.c \
		   osd/msp_client_osd.c \
		   osd/osd_tasks.c \
		   sensors/battery.c \
		   io/beeper.c

HIGHEND_SRC = \
		   flight/gtune.c \
		   flight/navigation.c \
		   flight/gps_conversion.c \
		   common/colorconversion.c \
		   io/gps.c \
		   io/ledstrip.c \
		   io/display.c \
		   telemetry/telemetry.c \
		   telemetry/frsky.c \
		   telemetry/hott.c \
		   telemetry/smartport.c \
		   telemetry/ltm.c \
		   telemetry/mavlink.c \
		   sensors/sonar.c \
		   sensors/barometer.c \
		   blackbox/blackbox.c \
		   blackbox/blackbox_io.c

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

STM32F10x_COMMON_SRC = \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/system_stm32f10x.c

NAZE_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   $(STM32F10x_COMMON_SRC) \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/barometer_bmp280.c \
		   drivers/bus_spi.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/inverter.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   io/flashfs.c \
		   hardware_revision.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

ALIENFLIGHTF1_SRC = $(NAZE_SRC)

EUSTM32F103RC_SRC = \
		   startup_stm32f10x_hd_gcc.S \
		   $(STM32F10x_COMMON_SRC) \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_spi.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/flash_m25p16.c \
		   drivers/inverter.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

PORT103R_SRC = $(EUSTM32F103RC_SRC)

OLIMEXINO_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   $(STM32F10x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_bmp085.c \
		   drivers/bus_spi.c \
		   drivers/compass_hmc5883l.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

CJMCU_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   $(STM32F10x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/compass_hmc5883l.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   hardware_revision.c \
		   flight/gtune.c \
		   blackbox/blackbox.c \
		   blackbox/blackbox_io.c \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

CC3D_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   $(STM32F10x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_spi.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/flash_m25p16.c \
		   drivers/inverter.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)

STM32F30x_COMMON_SRC = \
		   startup_stm32f30x_md_gcc.S \
		   target/system_stm32f30x.c \
		   drivers/adc.c \
		   drivers/adc_stm32f30x.c \
		   drivers/bus_i2c_stm32f30x.c \
		   drivers/bus_spi.c \
		   drivers/gpio_stm32f30x.c \
		   drivers/light_led_stm32f30x.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f30x.c \
		   drivers/sound_beeper_stm32f30x.c \
		   drivers/system_stm32f30x.c

STM32F30x_FC_COMMON_SRC = \
		   drivers/display_ug2864hsweg01.h \
		   drivers/timer.c \
		   drivers/timer_stm32f30x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c

STM32F3DISCOVERY_COMMON_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_lsm303dlhc.c \
		   drivers/compass_hmc5883l.c \
		   $(VCP_SRC)

STM32F3DISCOVERY_SRC = \
		   $(STM32F3DISCOVERY_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/sdcard.c \
		   drivers/sdcard_standard.c \
		   io/asyncfatfs/asyncfatfs.c \
		   io/asyncfatfs/fat_standard.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

CHEBUZZF3_SRC = \
		   $(STM32F3DISCOVERY_SRC) \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

COLIBRI_RACE_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8963.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)
		   
LUX_RACE_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)

SPARKY_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/display_ug2864hsweg01.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   drivers/sonar_hcsr04.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)

ALIENFLIGHTF3_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/display_ug2864hsweg01.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/compass_ak8963.c \
		   drivers/serial_usb_vcp.c \
		   drivers/sonar_hcsr04.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)
		  
RCEXPLORERF3_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_hmc5883l.c \
		   drivers/compass_ak8975.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/serial_usb_vcp.c \
		   drivers/flash_m25p16.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/sonar_hcsr04.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)

RMDO_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_bmp280.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

SPRACINGF3_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

SPRACINGF3EVO_SRC	 = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/barometer_bmp280.c \
		   drivers/compass_ak8963.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   drivers/sdcard.c \
		   drivers/sdcard_standard.c \
		   drivers/transponder_ir.c \
		   drivers/transponder_ir_stm32f30x.c \
		   io/asyncfatfs/asyncfatfs.c \
		   io/asyncfatfs/fat_standard.c \
		   io/transponder_ir.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)

MOTOLAB_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   drivers/flash_m25p16.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)

SPRACINGF3MINI_SRC	 = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/barometer_bmp280.c \
		   drivers/compass_ak8963.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_softserial.c \
		   drivers/serial_usb_vcp.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sdcard.c \
		   drivers/sdcard_standard.c \
		   drivers/transponder_ir.c \
		   drivers/transponder_ir_stm32f30x.c \
		   io/asyncfatfs/asyncfatfs.c \
		   io/asyncfatfs/fat_standard.c \
		   io/transponder_ir.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)
		   
IRCFUSIONF3_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(STM32F30x_FC_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_bmp085.c \
		   drivers/flash_m25p16.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(FC_COMMON_SRC) \
		   $(SYSTEM_SRC)

SPRACINGF1OSD_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   $(STM32F10x_COMMON_SRC) \
		   drivers/bus_spi.c \
		   drivers/video_max7456.c \
		   drivers/flash_m25p16.c \
		   io/flashfs.c \
		   osd/fonts/font_max7456_12x18.c \
		   osd/osd_max7456.c \
		   $(OSD_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)

SPRACINGF3OSD_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/video_max7456.c \
		   drivers/flash_m25p16.c \
		   io/flashfs.c \
		   osd/fonts/font_max7456_12x18.c \
		   osd/osd_max7456.c \
		   $(OSD_COMMON_SRC) \
		   $(SYSTEM_SRC) \
		   $(VCP_SRC)

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 = arm-none-eabi-gcc
OBJCOPY		 = arm-none-eabi-objcopy
SIZE		 = arm-none-eabi-size

#
# Tool options.
#

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -O0
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -Os
LTO_FLAGS	 =  -flto -fuse-linker-plugin $(OPTIMIZE)
endif

ifneq ($(filter $(OPTIONS),FAIL_ON_WARNINGS),)
WARN_FLAGS      += -Werror
endif

DEBUG_FLAGS	 = -ggdb3 -DDEBUG

CFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(WARN_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		   -std=gnu99 \
		   -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef \
		   -ffunction-sections \
		   -fdata-sections \
		   $(DEVICE_FLAGS) \
		   -DUSE_STDPERIPH_DRIVER \
		   $(TARGET_FLAGS) \
		   -D'__FORKNAME__="$(FORKNAME)"' \
		   -D'__TARGET__="$(TARGET)"' \
		   -D'__REVISION__="$(REVISION)"' \
		   -fverbose-asm -ffat-lto-objects \
		   -save-temps=obj \
		   -MMD -MP

ASFLAGS		 = $(ARCH_FLAGS) \
		   $(WARN_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		  -MMD -MP

LDFLAGS		 = -lm \
		   -nostartfiles \
		   --specs=nano.specs \
		   -lc \
		   -lnosys \
		   $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(WARN_FLAGS) \
		   $(DEBUG_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -Wl,-L$(LINKER_DIR) \
		   -Wl,--cref \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK         = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
		   --std=c99 --inline-suppr --quiet --force \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -I/usr/include -I/usr/include/linux

#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

TARGET_BIN	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).bin
TARGET_HEX	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).hex
TARGET_ELF	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_DEPS	 = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map


## Default make goal:
## hex         : Make filetype hex only
.DEFAULT_GOAL := hex

## Optional make goals:
## all         : Make all filetypes, binary and hex
all: hex bin

## binary      : Make binary filetype
## bin         : Alias of 'binary'
## hex         : Make hex filetype
bin:    $(TARGET_BIN)
binary: $(TARGET_BIN)
hex:    $(TARGET_HEX)

# rule to reinvoke make with TARGET= parameter
# rules that should be handled in toplevel Makefile, not dependent on TARGET
GLOBAL_GOALS	= all_targets cppcheck test

.PHONY: $(VALID_TARGETS)
$(VALID_TARGETS):
	$(MAKE) TARGET=$@ $(filter-out $(VALID_TARGETS) $(GLOBAL_GOALS), $(MAKECMDGOALS))

## all_targets : Make all TARGETs
.PHONY: all_targets
all_targets : $(VALID_TARGETS)

## clean       : clean up all temporary / machine-generated files
clean:
	rm -f $(TARGET_BIN) $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)
	cd src/test && $(MAKE) clean || true

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## flash       : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

st-flash_$(TARGET): $(TARGET_BIN)
	st-flash --reset write $< 0x08000000

## st-flash    : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick     : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck    : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

## help        : print this help message and exit
help: Makefile
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [goal] [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
	@sed -n 's/^## //p' $<

## test        : run the cleanflight test suite
## junittest   : run the cleanflight test suite, producing Junit XML result files.
test junittest:
	cd src/test && $(MAKE) $@

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<



# include auto-generated dependencies
-include $(TARGET_DEPS)
