F3_TARGETS  += $(TARGET)

FEATURES    = VCP

FEATURE_CUT_LEVEL = 0

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8975.c
