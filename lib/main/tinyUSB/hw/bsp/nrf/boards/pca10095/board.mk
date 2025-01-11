CPU_CORE = cortex-m33
MCU_VARIANT = nrf5340_application
CFLAGS += -DNRF5340_XXAA -DNRF5340_XXAA_APPLICATION

# enable max3421 host driver for this board
MAX3421_HOST = 1

LD_FILE = hw/mcu/nordic/nrfx/mdk/nrf5340_xxaa_application.ld

SRC_C += hw/mcu/nordic/nrfx/drivers/src/nrfx_usbreg.c

# caused by void SystemStoreFICRNS() (without void) in system_nrf5340_application.c
CFLAGS += -Wno-error=strict-prototypes

# flash using jlink
JLINK_DEVICE = nrf5340_xxaa_app
flash: flash-jlink
