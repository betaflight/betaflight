F405_TARGETS    += $(TARGET)
FEATURES        += VCP
HSE_VALUE       = 25000000

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/compass/compass_ak8963.c
