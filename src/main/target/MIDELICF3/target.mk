F3_TARGETS  += $(TARGET)

FEATURES  = VCP SDCARD

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/cc2500.c \
            rx/frsky_d.c
