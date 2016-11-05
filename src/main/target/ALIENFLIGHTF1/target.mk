F1_TARGETS  += $(TARGET)
FEATURES    = HIGHEND 

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f10x.c
