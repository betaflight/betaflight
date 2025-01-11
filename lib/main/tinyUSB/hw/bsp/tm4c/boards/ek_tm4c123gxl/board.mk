MCU_SUB_VARIANT = 123

CFLAGS += -DTM4C123GH6PM

LD_FILE = $(BOARD_PATH)/tm4c123.ld

# For flash-jlink target
JLINK_DEVICE = TM4C123GH6PM

# flash using openocd
OPENOCD_OPTION = -f board/ti_ek-tm4c123gxl.cfg

UNIFLASH_OPTION = -c ${TOP}/${BOARD_PATH}/${BOARD}.ccxml -r 1

flash: flash-openocd
