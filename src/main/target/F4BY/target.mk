F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

TARGET_SRC = \
             drivers/accgyro_spi_mpu6000.c \
             drivers/barometer_ms5611.c \
             drivers/compass_hmc5883l.c

