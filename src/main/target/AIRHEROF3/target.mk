F3_TARGETS  += $(TARGET)
HSE_VALUE = 12000000

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_spi_bmp280.c
