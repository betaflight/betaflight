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

# The target to build, must be one of NAZE, FY90Q, OLIMEXINO or STM32F3DISCOVERY
TARGET		?= NAZE

# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME			 = cleanflight

VALID_TARGETS	 = NAZE FY90Q OLIMEXINO STM32F3DISCOVERY CHEBUZZF3

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)/src
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj
CMSIS_DIR	 = $(ROOT)/lib/CMSIS
INCLUDE_DIRS = $(SRC_DIR)

# Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup

ifeq ($(TARGET),$(filter $(TARGET),STM32F3DISCOVERY CHEBUZZF3))

STDPERIPH_DIR	 = $(ROOT)/lib/STM32F30x_StdPeriph_Driver

VPATH		:= $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM1/CoreSupport \
		   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x \

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4
DEVICE_FLAGS = -DSTM32F303xC
TARGET_FLAGS = -D$(TARGET)
ifeq ($(TARGET),CHEBUZZF3)
# CHEBUZZ is a VARIANT of STM32F3DISCOVERY
TARGET_FLAGS := $(TARGET_FLAGS) -DSTM32F3DISCOVERY
endif


else

STDPERIPH_DIR	 = $(ROOT)/lib/STM32F10x_StdPeriph_Driver

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
TARGET_FLAGS = -D$(TARGET)
DEVICE_FLAGS = -DSTM32F10X_MD

endif

COMMON_SRC	 = build_config.c \
		   battery.c \
		   boardalignment.c \
		   buzzer.c \
		   config.c \
		   common/maths.c \
		   common/printf.c \
		   common/typeconversion.c \
		   failsafe.c \
		   main.c \
		   mw.c \
		   sensors_acceleration.c \
		   sensors_barometer.c \
		   sensors_compass.c \
		   sensors_gyro.c \
		   sensors_initialisation.c \
		   sensors_sonar.c \
		   drivers/bus_i2c_soft.c \
		   drivers/serial_common.c \
		   drivers/sound_beeper.c \
		   drivers/system_common.c \
		   flight_common.c \
		   flight_imu.c \
		   flight_mixer.c \
		   gps_common.c \
		   runtime_config.c \
		   rc_controls.c \
		   rc_curves.c \
		   rx_common.c \
		   rx_msp.c \
		   rx_pwm.c \
		   rx_sbus.c \
		   rx_sumd.c \
		   rx_spektrum.c \
		   telemetry_common.c \
		   telemetry_frsky.c \
		   telemetry_hott.c \
		   serial_common.c \
		   serial_cli.c \
		   serial_msp.c \
		   statusindicator.c \
		   $(CMSIS_SRC) \
		   $(STDPERIPH_SRC)

NAZE_SRC	 = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/adc_common.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_spi.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/light_ledring.c \
		   drivers/sonar_hcsr04.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart_common.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/timer_common.c \
		   $(COMMON_SRC)

FY90Q_SRC	 = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_fy90q.c \
		   drivers/adc_fy90q.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/pwm_fy90q.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/serial_uart_common.c \
		   drivers/serial_uart_stm32f10x.c \
		   $(COMMON_SRC)

OLIMEXINO_SRC	 = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/adc_common.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart_common.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/timer_common.c \
		   $(COMMON_SRC)

STM32F3DISCOVERY_COMMON_SRC	 = startup_stm32f30x_md_gcc.S \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_lsm303dlhc.c \
		   drivers/adc_common.c \
		   drivers/bus_i2c_stm32f30x.c \
		   drivers/bus_spi.c \
		   drivers/gpio_stm32f30x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_uart_common.c \
		   drivers/serial_uart_stm32f30x.c \
		   drivers/serial_softserial.c \
		   drivers/timer_common.c

STM32F3DISCOVERY_SRC	 = $(STM32F3DISCOVERY_COMMON_SRC) \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_l3g4200d.c \
		   $(COMMON_SRC)

CHEBUZZF3_SRC	 = $(STM32F3DISCOVERY_COMMON_SRC) \
		   $(COMMON_SRC)

# In some cases, %.s regarded as intermediate file, which is actually not.
# This will prevent accidental deletion of startup code.
.PRECIOUS: %.s

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 = arm-none-eabi-gcc
OBJCOPY		 = arm-none-eabi-objcopy

#
# Tool options.
#

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
BASE_CFLAGS	 = $(ARCH_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -Wall \
		   -ffunction-sections \
		   -fdata-sections \
		   $(DEVICE_FLAGS) \
		   -DUSE_STDPERIPH_DRIVER \
		   $(TARGET_FLAGS) \
		   -D'__FORKNAME__="$(FORKNAME)"'

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LD_SCRIPT	 = $(ROOT)/stm32_flash.ld
LDFLAGS		 = -lm \
		   $(ARCH_FLAGS) \
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

ifeq ($(DEBUG),GDB)
CFLAGS = $(BASE_CFLAGS) \
	-ggdb \
	-O0
else
CFLAGS = $(BASE_CFLAGS) \
	-Os
endif


TARGET_HEX	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP   = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

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
$(OBJECT_DIR)/$(TARGET)/%.o): %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

clean:
	rm -f $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

flash: flash_$(TARGET)


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
