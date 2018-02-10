F7X2RE_TARGETS  += $(TARGET)
FEATURES        += VCP  ONBOARDFLASH
TARGET_SRC = \
            drivers/accgyro/accgyro_spi_icm20689.c\
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/max7456.c
