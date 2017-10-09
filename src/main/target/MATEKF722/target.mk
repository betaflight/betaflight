F7X2RE_TARGETS += $(TARGET)
FEATURES       += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/max7456.c
