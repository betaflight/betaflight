MCU_VARIANT = nrf52840
CFLAGS += -DNRF52840_XXAA

LD_FILE = $(BOARD_PATH)/$(BOARD).ld

# flash using bossac (as part of Nano33 BSP tools)
# can be found in arduino15/packages/arduino/tools/bossac/
# Add it to your PATH or change BOSSAC variable to match your installation
BOSSAC = bossac

flash: $(BUILD)/$(PROJECT).bin
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(BOSSAC) --port=$(SERIAL) -U -i -e -w $^ -R
