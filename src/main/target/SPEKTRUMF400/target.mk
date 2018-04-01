F446_TARGETS  += $(TARGET)

#FEATURES    = VCP ONBOARDFLASH
FEATURES    = VCP

HSE_VALUE      = 12000000

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c
