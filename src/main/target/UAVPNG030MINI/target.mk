F405_TARGETS   += $(TARGET)
FEATURES        += VCP
TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c
