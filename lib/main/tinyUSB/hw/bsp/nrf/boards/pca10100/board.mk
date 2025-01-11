MCU_VARIANT = nrf52833
CFLAGS += -DNRF52833_XXAA

LD_FILE = hw/mcu/nordic/nrfx/mdk/nrf52833_xxaa.ld

# flash using jlink
flash: flash-jlink
