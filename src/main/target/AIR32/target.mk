F3_TARGETS  += $(TARGET)

FEATURES    = VCP ONBOARDFLASH

FEATURE_CUT_LEVEL = 1

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/inverter.c
