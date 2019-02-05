F405_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD_SDIO

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_lps.c \
            drivers/max7456.c