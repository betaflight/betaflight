F3_TARGETS   += $(TARGET)
FEATURES     = VCP ONBOARDFLASH
TARGET_FLAGS = -DSPRACINGF3

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/barometer/barometer_bmp085.c
