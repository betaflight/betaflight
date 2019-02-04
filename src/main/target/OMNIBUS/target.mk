F3_TARGETS  += $(TARGET)

FEATURES    = VCP SDCARD_SPI

FEATURE_CUT_LEVEL = 9

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/max7456.c 
