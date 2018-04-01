F7X2RE_TARGETS  += $(TARGET)
FEATURES        += SDCARD VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c
