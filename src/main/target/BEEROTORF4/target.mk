F405_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD HIGHEND

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/barometer_bmp280.c \
            drivers/max7456.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f4xx.c
#            drivers/accgyro_spi_icm20689.c \
#            drivers/transponder_ir.c \
#            io/transponder_ir.c
