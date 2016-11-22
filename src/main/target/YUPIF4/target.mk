F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

TARGET_SRC = \
            drivers/accgyro_spi_mpu6500.c \
            drivers/accgyro_mpu6500.c \
	    drivers/barometer_bmp280.c \
	    drivers/compass_hmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f4xx.c

