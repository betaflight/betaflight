F427_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

HSE_VALUE = 24000000

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/barometer_ms56xx.c \
            drivers/barometer_spi_ms56xx.c \
            drivers/compass_hmc5883l.c \
            drivers/compass_mag3110.c \
            drivers/compass_ak8963.c \
            drivers/serial_usb_vcp.c
