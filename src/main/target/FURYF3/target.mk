F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/barometer_ms5611.c \
            drivers/barometer_bmp280.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/accgyro_spi_icm20689.c \
            drivers/pwm_output_stm32f3xx.c

