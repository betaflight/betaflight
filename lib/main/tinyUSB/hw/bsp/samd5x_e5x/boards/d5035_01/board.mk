SAM_FAMILY = same51

HWREV ?= 1

CFLAGS += \
  -D__SAME51J19A__ \
  -DCONF_CPU_FREQUENCY=80000000 \
  -DCONF_GCLK_USB_FREQUENCY=48000000 \
  -DD5035_01=1 \
  -DBOARD_NAME="\"D5035-01\"" \
  -DSVC_Handler=SVCall_Handler \
  -DHWREV=$(HWREV)

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/same51j19a_flash.ld

# For flash-jlink target
JLINK_DEVICE = ATSAME51J19

# flash using jlink
flash: flash-jlink
