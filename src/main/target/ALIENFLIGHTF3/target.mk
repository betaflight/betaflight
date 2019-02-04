F3_TARGETS  += $(TARGET)

FEATURES    = VCP

FEATURE_CUT_LEVEL = 0

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/compass/compass_ak8963.c
