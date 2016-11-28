F405_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = \
			drivers/accgyro_spi_mpu6000.c \
            drivers/barometer_bmp085.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_ms5611.c \
            drivers/compass_ak8963.c \
            drivers/compass_ak8975.c \
            drivers/compass_hmc5883l.c \
            drivers/compass_mag3110.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f4xx.c \
            drivers/max7456.c
