CMSIS_DIR         := $(ROOT)/lib/main/AT32F43x/cmsis
TARGET_MCU        := AT32F435
MCU_FLASH_SIZE    := 4032
DEVICE_FLAGS       = -DAT32F435ZMT7
TARGET_MCU_FAMILY := AT32F4

AT_LIB_DIR      = $(ROOT)/lib/main/AT32F43x
STDPERIPH_DIR   = $(AT_LIB_DIR)/drivers

STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES        = at32f435_437_dvp.c \
				  at32f435_437_can.c \
				  at32f435_437_xmc.c \
				  at32f435_437_emac

STARTUP_SRC     = at32/startup_at32f435_437.s
STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

VPATH           := $(VPATH):$(AT_LIB_DIR)/cmsis/cm4/core_support:$(STDPERIPH_DIR)/inc:$(SRC_DIR)/startup/at32:$(STDPERIPH_DIR)/src

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(SRC_DIR)/startup/at32 \
                   $(STDPERIPH_DIR)/inc \
                   $(AT_LIB_DIR)/cmsis/cm4/core_support \
                   $(AT_LIB_DIR)/cmsis/cm4 \
                   $(AT_LIB_DIR)/middlewares/i2c_application_library \
                   $(AT_LIB_DIR)/middlewares/usb_drivers/inc \
                   $(AT_LIB_DIR)/middlewares/usbd_class/cdc

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

TARGET_SRC		:= \
            $(AT_LIB_DIR)/middlewares/i2c_application_library/i2c_application.c

LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xM.ld

ARCH_FLAGS      = -std=c99  -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS   += -DUSE_ATBSP_DRIVER -DAT32F43x -DHSE_VALUE=$(HSE_VALUE) -DAT32 -DUSE_OTG_HOST_MODE

MCU_COMMON_SRC = \
    $(addprefix startup/at32/,$(notdir $(wildcard $(SRC_DIR)/startup/at32/*.c))) \
    $(addprefix drivers/at32/,$(notdir $(wildcard $(SRC_DIR)/drivers/at32/*.c))) \
    drivers/bus_i2c_timing.c \
    drivers/pwm_output_dshot_shared.c

MCU_EXCLUDES =

VCP_SRC = \
        $(addprefix $(AT_LIB_DIR)/middlewares/usbd_class/cdc/,$(notdir $(wildcard $(AT_LIB_DIR)/middlewares/usbd_class/cdc/*.c))) \
        $(addprefix $(AT_LIB_DIR)/middlewares/usb_drivers/src/,$(notdir $(wildcard $(AT_LIB_DIR)/middlewares/usb_drivers/src/*.c))) \
        drivers/usb_io.c
