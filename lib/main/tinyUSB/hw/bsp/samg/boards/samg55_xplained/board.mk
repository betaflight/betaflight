CFLAGS += -D__SAMG55J19__

JLINK_DEVICE = ATSAMG55J19

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/samg55j19_flash.ld

OPENOCD_OPTION = -f board/atmel_samg55_xplained_pro.cfg

flash: flash-openocd
