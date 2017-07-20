F7X5XG_TARGETS += $(TARGET)
FEATURES       += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_spi_bmp280.c \
            drivers/barometer/barometer_ms56xx.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_mag3110.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/max7456.c

