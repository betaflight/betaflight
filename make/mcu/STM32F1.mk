#
# F1 Make file include
#

ifeq ($(OPBL),yes)
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k_opbl.ld
endif

TARGET_FLASH   := 128
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F1/Drivers/STM32F10x_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f10x_crc.c \
                  stm32f10x_cec.c \
                  stm32f10x_can.c
STARTUP_SRC     = startup_stm32f10x_md_gcc.S
STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH           := $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC       = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
                  $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/CM3/CoreSupport \
                   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x

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

MCU_COMMON_SRC = \
            drivers/adc_stm32f10x.c \
            drivers/bus_i2c_stm32f10x.c \
            drivers/bus_spi_stdperiph.c \
            drivers/dma.c \
            drivers/inverter.c \
            drivers/light_ws2811strip_stdperiph.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_stm32f10x.c \
            drivers/system_stm32f10x.c \
            drivers/timer_stm32f10x.c

DSP_LIB :=

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      :=
OPTIMISE_SIZE       :=

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif
