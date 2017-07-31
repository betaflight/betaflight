F3_TARGETS  += $(TARGET)
FEATURES  = ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
			blackbox/blackbox.c \
            blackbox/blackbox_io.c \
			drivers/cc2500.c \
			rx/frsky_d.c
