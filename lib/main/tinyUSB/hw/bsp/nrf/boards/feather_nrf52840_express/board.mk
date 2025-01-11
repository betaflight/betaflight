MCU_VARIANT = nrf52840
CFLAGS += -DNRF52840_XXAA

# enable max3421 host driver for this board
MAX3421_HOST = 1

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/nrf/linker/nrf52840_s140_v6.ld

$(BUILD)/$(PROJECT).zip: $(BUILD)/$(PROJECT).hex
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0xFFFE --application $^ $@

# flash using adafruit-nrfutil dfu
flash: $(BUILD)/$(PROJECT).zip
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	adafruit-nrfutil --verbose dfu serial --package $^ -p $(SERIAL) -b 115200 --singlebank --touch 1200
