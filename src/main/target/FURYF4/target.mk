F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_spi_mpu6000.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/accgyro_mpu6500.c \
            drivers/barometer_ms5611.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f4xx.c

