SDK_DIR = hw/mcu/allwinner/f1c100s

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= arm926ej-s

#CFLAGS += \
#  -march=armv5te \
#  -mtune=arm926ej-s \
#  -mfloat-abi=soft \
#  -marm \

CFLAGS += \
  -ffreestanding \
  -std=gnu99 \
  -mno-thumb-interwork \
  -D__ARM32_ARCH__=5 \
  -D__ARM926EJS__ \
  -Wno-float-equal \
  -Wno-unused-parameter \
  -DCFG_TUSB_MCU=OPT_MCU_F1C100S \
  -Wno-error=array-bounds \

LD_FILE = ${SDK_DIR}/f1c100s.ld

# TODO may skip nanolib
LDFLAGS += \
  -nostdlib -lgcc \
  --specs=nosys.specs --specs=nano.specs \

SRC_C += \
	src/portable/sunxi/dcd_sunxi_musb.c \
	${SDK_DIR}/machine/sys-uart.c \
	${SDK_DIR}/machine/exception.c \
	${SDK_DIR}/machine/sys-clock.c \
	${SDK_DIR}/machine/sys-copyself.c \
	${SDK_DIR}/machine/sys-dram.c \
	${SDK_DIR}/machine/sys-mmu.c \
	${SDK_DIR}/machine/sys-spi-flash.c \
	${SDK_DIR}/machine/f1c100s-intc.c \
	${SDK_DIR}/lib/malloc.c \
	${SDK_DIR}/lib/printf.c

SRC_S += \
  ${SDK_DIR}/machine/start.S \
	${SDK_DIR}/lib/memcpy.S \
	${SDK_DIR}/lib/memset.S

INC += \
	$(TOP)/${SDK_DIR}/include \
	$(TOP)/$(BOARD_PATH)

# flash target using xfel
flash: flash-xfel

exec: $(BUILD)/$(PROJECT).bin
	xfel ddr
	xfel write 0x80000000 $<
	xfel exec 0x80000000
