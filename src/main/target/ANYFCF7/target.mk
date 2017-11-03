F7X5XG_TARGETS += $(TARGET)
FEATURES       += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/max7456.c
