F7X2RE_TARGETS += $(TARGET)
FEATURES    = VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/compass/compass_fake.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/max7456.c
