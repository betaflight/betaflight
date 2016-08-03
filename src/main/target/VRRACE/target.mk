F405_TARGETS   += $(TARGET)
FEATURES       += VCP

TARGET_SRC = \
            drivers/accgyro_spi_mpu6000.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/accgyro_mpu6500.c
