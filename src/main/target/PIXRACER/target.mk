F427_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

HSE_VALUE = 24000000

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_ms56xx.c \
            drivers/barometer/barometer_spi_ms56xx.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_mag3110.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/serial_usb_vcp.c
