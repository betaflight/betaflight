CFLAGS += -DCPU_MIMXRT1062DVL6A
MCU_VARIANT = MIMXRT1062

# For flash-jlink target
JLINK_DEVICE = MIMXRT1062xxx6A

# flash by using teensy_loader_cli https://github.com/PaulStoffregen/teensy_loader_cli
# Make sure it is in your PATH
flash: $(BUILD)/$(PROJECT).hex
	teensy_loader_cli --mcu=imxrt1062 -v -w $<
