F3_TARGETS  += $(TARGET)
FEATURES    = VCP 

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/accgyro_mpu6500.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f30x.c

