F405_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD_SPI

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/max7456.c