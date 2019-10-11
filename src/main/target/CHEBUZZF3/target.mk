F3_TARGETS  += $(TARGET)

FEATURES    = VCP SDCARD_SPI

FEATURE_CUT_LEVEL = 6

TARGET_SRC = \
            drivers/compass/compass_hmc5883l.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu3050.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8975.c
