F405_TARGETS   += $(TARGET)
FEATURES       += VCP SDCARD

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/max7456.c \
            io/osd.c 