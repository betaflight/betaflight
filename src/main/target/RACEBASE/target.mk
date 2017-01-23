F3_TARGETS  += $(TARGET)

FEATURES    = ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/flash_m25p16.c \
            drivers/max7456.c \
            io/osd.c
