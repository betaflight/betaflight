F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705_soft_spi.c

