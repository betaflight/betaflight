CMSIS_DIR       := $(ROOT)/lib/main/AT32F43x/cmsis
TARGET_MCU      := AT32F4
MCU_FLASH_SIZE  := 4032
DEVICE_FLAGS    = -DAT32F435ZMT7

STDPERIPH_DIR   = $(ROOT)/lib/main/AT32F43x/drivers
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = at32f435_437_dvp.c \
				  at32f435_437_can.c \
				  at32f435_437_xmc.c \
				  at32f435_437_emac

STARTUP_SRC     = at32/startup_at32f435_437.s
STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

VPATH           := $(VPATH):$(ROOT)/lib/main/AT32F43x/cmsis/cm4/core_support:$(STDPERIPH_DIR)/inc:$(SRC_DIR)/startup/at32

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(SRC_DIR)/startup/at32 \
                   $(STDPERIPH_DIR)/inc \
                   $(ROOT)/lib/main/AT32F43x/cmsis/cm4/core_support \
                   $(ROOT)/lib/main/AT32F43x/cmsis/cm4

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xM.ld

ARCH_FLAGS      = -std=c99  -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS   += -DUSE_ATBSP_DRIVER -DAT32F43x -DHSE_VALUE=$(HSE_VALUE)

MCU_COMMON_SRC = \
            $(addprefix startup/at32/,$(notdir $(wildcard $(SRC_DIR)/startup/at32/*.c))) \
            $(addprefix drivers/at32/,$(notdir $(wildcard $(SRC_DIR)/drivers/at32/*.c)))
