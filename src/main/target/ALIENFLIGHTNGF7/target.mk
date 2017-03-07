F7X2RE_TARGETS  += $(TARGET)
FEATURES        += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_spi_bmp280.c \
            drivers/barometer_ms5611.c \
            drivers/barometer_spi_ms5611.c \
            drivers/compass_ak8963.c \
            drivers/compass_hmc5883l.c \
            drivers/compass_spi_hmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c
