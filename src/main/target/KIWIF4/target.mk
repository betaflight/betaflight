F405_TARGETS    += $(TARGET)
FEATURES        += VCP ONBOARDFLASH SDCARD

TARGET_SRC = \
            drivers/accgyro_spi_mpu6000.c \
            drivers/max7456.c \
            io/osd.c

