F3_TARGETS  += $(TARGET)
FEATURES    = VCP 

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/barometer_ms5611.c \
            drivers/barometer_bmp280.c \
            drivers/compass_ak8975.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f30x.c

