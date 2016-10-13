F3_TARGETS  += $(TARGET)
FEATURES    = ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/barometer_bmp280.c

