F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP ONBOARDFLASH

TARGET_SRC = \
				drivers/accgyro_spi_mpu6500.c \
				drivers/accgyro_mpu6500.c \
				drivers/barometer_ms5611.c \
				drivers/compass_hmc5883l.c \
				drivers/compass_ak8975.c \
				drivers/compass_ist8310.c \
				drivers/compass_ak8963.c \
				drivers/compass_mag3110.c \
				drivers/light_ws2811strip.c \
				drivers/light_ws2811strip_stm32f4xx.c\
				drivers/max7456.c
