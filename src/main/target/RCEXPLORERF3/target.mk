F3_TARGETS  += $(TARGET)
FEATURES    = VCP

TARGET_SRC = \
           drivers/accgyro/accgyro_mpu.c \
           drivers/accgyro/accgyro_spi_mpu6000.c \
           drivers/barometer/barometer_ms5611.c \
           drivers/compass/compass_hmc5883l.c \
           drivers/compass/compass_ak8975.c \
           drivers/display_ug2864hsweg01.c \
           drivers/serial_usb_vcp.c \
           drivers/flash_m25p16.c
