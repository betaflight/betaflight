F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

TARGET_SRC = \
             drivers/accgyro/accgyro_spi_mpu6000.c \
             drivers/barometer/barometer_ms56xx.c \
             drivers/compass/compass_hmc5883l.c \
             drivers/compass/compass_qmc5883l.c \
             drivers/rangefinder_hcsr04.c \
             drivers/max7456.c