F405_TARGETS    += $(TARGET)
FEATURES        += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_spi_mpu9250.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_ms5611.c \
            drivers/compass_ak8963.c \
            drivers/compass_hmc5883l.c

