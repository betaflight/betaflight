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

# compile for OpenPilot BootLoader support
OPBL ?=no

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME			 = cleanflight

VALID_TARGETS	 = NAZE NAZE32PRO OLIMEXINO STM32F3DISCOVERY CHEBUZZF3 CC3D CJMCU EUSTM32F103RC SPRACINGF3 PORT103R SPARKY ALIENWIIF1 ALIENWIIF3

# Valid targets for OP BootLoader support
OPBL_VALID_TARGETS = CC3D

# Configure default flash sizes for the targets
ifeq ($(FLASH_SIZE),)
ifeq ($(TARGET),$(filter $(TARGET),CJMCU))
FLASH_SIZE = 64
else ifeq ($(TARGET),$(filter $(TARGET),NAZE CC3D ALIENWIIF1 SPRACINGF3 OLIMEXINO))
FLASH_SIZE = 128
else ifeq ($(TARGET),$(filter $(TARGET),EUSTM32F103RC PORT103R STM32F3DISCOVERY CHEBUZZF3 NAZE32PRO SPARKY ALIENWIIF3))
FLASH_SIZE = 256
else
$(error FLASH_SIZE not configured for target)
endif
endif

REVISION = $(shell git log -1 --format="%h")

# Working directories
ROOT		 := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR		 = $(ROOT)/src/main
OBJECT_DIR	 = $(ROOT)/obj/main
BIN_DIR		 = $(ROOT)/obj
CMSIS_DIR	 = $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS	 = $(SRC_DIR)
LINKER_DIR	 = $(ROOT)/src/main/target

# Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR	= $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))

ifeq ($(TARGET),$(filter $(TARGET),STM32F3DISCOVERY CHEBUZZF3 NAZE32PRO SPRACINGF3 SPARKY ALIENWIIF3))

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

ifneq ($(TARGET),SPRACINGF3)
INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(USBFS_DIR)/inc \
		   $(ROOT)/src/main/vcp

VPATH := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
		   $(USBPERIPH_SRC) 

endif

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS = -D$(TARGET)
ifeq ($(TARGET),CHEBUZZF3)
# CHEBUZZ is a VARIANT of STM32F3DISCOVERY
TARGET_FLAGS := $(TARGET_FLAGS) -DSTM32F3DISCOVERY
endif

else ifeq ($(TARGET),$(filter $(TARGET),EUSTM32F103RC PORT103R))


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

ifeq ($(TARGET),CC3D)
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

endif

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

TARGET_DIR = $(ROOT)/src/main/target/$(TARGET)
TARGET_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

ifeq ($(TARGET),ALIENWIIF1)
# ALIENWIIF1 is a VARIANT of NAZE
TARGET_FLAGS := $(TARGET_FLAGS) -DNAZE -DALIENWII32
TARGET_DIR = $(ROOT)/src/main/target/NAZE
endif

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		    $(TARGET_DIR)

VPATH		:= $(VPATH):$(TARGET_DIR)

COMMON_SRC	 = build_config.c \
		   debug.c \
		   version.c \
		   $(TARGET_SRC) \
		   config/config.c \
		   config/runtime_config.c \
		   common/maths.c \
		   common/printf.c \
		   common/typeconversion.c \
		   common/encoding.c \
		   main.c \
		   mw.c \
		   flight/altitudehold.c \
		   flight/failsafe.c \
		   flight/pid.c \
		   flight/imu.c \
		   flight/mixer.c \
		   flight/lowpass.c \
		   drivers/bus_i2c_soft.c \
		   drivers/serial.c \
		   drivers/sound_beeper.c \
		   drivers/system.c \
		   io/beeper.c \
		   io/rc_controls.c \
		   io/rc_curves.c \
		   io/serial.c \
		   io/serial_cli.c \
		   io/serial_msp.c \
		   io/statusindicator.c \
		   rx/rx.c \
		   rx/pwm.c \
		   rx/msp.c \
		   rx/sbus.c \
		   rx/sumd.c \
		   rx/sumh.c \
		   rx/spektrum.c \
		   rx/xbus.c \
		   sensors/acceleration.c \
		   sensors/battery.c \
		   sensors/boardalignment.c \
		   sensors/compass.c \
		   sensors/gyro.c \
		   sensors/initialisation.c \
		   $(CMSIS_SRC) \
		   $(DEVICE_STDPERIPH_SRC)

HIGHEND_SRC  = flight/autotune.c \
		   flight/navigation.c \
		   flight/gps_conversion.c \
		   common/colorconversion.c \
		   io/gps.c \
		   io/ledstrip.c \
		   io/display.c \
		   telemetry/telemetry.c \
		   telemetry/frsky.c \
		   telemetry/hott.c \
		   telemetry/msp.c \
		   telemetry/smartport.c \
		   sensors/sonar.c \
		   sensors/barometer.c \
		   blackbox/blackbox.c \
		   blackbox/blackbox_io.c

VCP_SRC	 = \
		   vcp/hw_config.c \
		   vcp/stm32_it.c \
		   vcp/usb_desc.c \
		   vcp/usb_endp.c \
		   vcp/usb_istr.c \
		   vcp/usb_prop.c \
		   vcp/usb_pwr.c \
		   drivers/serial_usb_vcp.c

NAZE_SRC	 = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_spi.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   drivers/flash_m25p16.c \
		   io/flashfs.c \
		   hardware_revision.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

ALIENWIIF1_SRC	 = $(NAZE_SRC)

EUSTM32F103RC_SRC	 = startup_stm32f10x_hd_gcc.S \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

PORT103R_SRC = $(EUSTM32F103RC_SRC)

OLIMEXINO_SRC	 = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_mpu6050.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/compass_hmc5883l.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

ifeq ($(OPBL),yes)
ifneq ($(filter $(TARGET),$(OPBL_VALID_TARGETS)),)
TARGET_FLAGS := -DOPBL $(TARGET_FLAGS)
LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k_opbl.ld
.DEFAULT_GOAL := binary
else
$(error OPBL specified with a unsupported target)
endif
endif

CJMCU_SRC	 = \
		   startup_stm32f10x_md_gcc.S \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   hardware_revision.c \
		   blackbox/blackbox.c \
		   blackbox/blackbox_io.c \
		   $(COMMON_SRC)

CC3D_SRC	 = \
		   startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_spi.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   drivers/flash_m25p16.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

STM32F30x_COMMON_SRC	 = \
		   startup_stm32f30x_md_gcc.S \
		   drivers/adc.c \
		   drivers/adc_stm32f30x.c \
		   drivers/bus_i2c_stm32f30x.c \
		   drivers/bus_spi.c \
		   drivers/gpio_stm32f30x.c \
		   drivers/light_led_stm32f30x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f30x.c \
		   drivers/sound_beeper_stm32f30x.c \
		   drivers/system_stm32f30x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f30x.c

NAZE32PRO_SRC	 = \
		   $(STM32F30x_COMMON_SRC) \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

STM32F3DISCOVERY_COMMON_SRC	 = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_lsm303dlhc.c \
		   drivers/compass_hmc5883l.c \
		   $(VCP_SRC)

STM32F3DISCOVERY_SRC	 = \
		   $(STM32F3DISCOVERY_COMMON_SRC) \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

CHEBUZZF3_SRC	 = \
		   $(STM32F3DISCOVERY_SRC) \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

SPARKY_SRC	 = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/display_ug2864hsweg01.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/serial_usb_vcp.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

ALIENWIIF3_SRC	 = $(SPARKY_SRC)

SPRACINGF3_SRC	 = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/sonar_hcsr04.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

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
LTO_FLAGS	 = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -ggdb3

CFLAGS		 = $(ARCH_FLAGS) \
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

ASFLAGS		 = $(ARCH_FLAGS) \
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
		   $(DEBUG_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

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

clean:
	rm -f $(TARGET_BIN) $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

flash: flash_$(TARGET)

binary: $(TARGET_BIN)

unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

unbrick: unbrick_$(TARGET)

help:
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)
