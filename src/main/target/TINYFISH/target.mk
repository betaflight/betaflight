F3_TARGETS  += $(TARGET)
FEATURES    = VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/flash_m25p16.c \
            drivers/accgyro/accgyro_spi_mpu6000.c
