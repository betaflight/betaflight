F405_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

TARGET_SRC = \
             drivers/accgyro_spi_mpu6000.c \
             drivers/barometer_ms56xx.c \
             drivers/compass_hmc5883l.c \
             drivers/sonar_hcsr04.c \
             drivers/max7456.c