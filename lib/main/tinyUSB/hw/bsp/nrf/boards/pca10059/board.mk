MCU_VARIANT = nrf52840
CFLAGS += -DNRF52840_XXAA

LD_FILE = $(BOARD_PATH)/$(BOARD).ld

# flash using Nordic nrfutil (pip2 install nrfutil)
# 	make BOARD=pca10059 SERIAL=/dev/ttyACM0 all flash
NRFUTIL = nrfutil

$(BUILD)/$(PROJECT).zip: $(BUILD)/$(PROJECT).hex
	$(NRFUTIL) pkg generate --hw-version 52 --sd-req 0x0000 --debug-mode --application $^ $@

flash: $(BUILD)/$(PROJECT).zip
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(NRFUTIL) dfu usb-serial --package $^ -p $(SERIAL) -b 115200
