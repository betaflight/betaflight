F405_TARGETS    += $(TARGET)
FEATURES        += VCP
HSE_VALUE       = 25000000

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/compass_ak8963.c
