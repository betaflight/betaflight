F7X5XG_TARGETS += $(TARGET)
FEATURES       += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c

