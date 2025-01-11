MCU_VARIANT = nrf52840
CFLAGS += -DNRF52840_XXAA

LD_FILE = hw/mcu/nordic/nrfx/mdk/nrf52840_xxaa.ld

# flash using jlink
flash: flash-jlink
