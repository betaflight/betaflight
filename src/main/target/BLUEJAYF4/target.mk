F405_TARGETS   += $(TARGET)
OPTIONS        += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro_spi_mpu6500.c \
            drivers/accgyro_mpu6500.c \
            drivers/barometer_ms5611.c 
