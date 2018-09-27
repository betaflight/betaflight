F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD_SPI VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705_soft_spi.c
