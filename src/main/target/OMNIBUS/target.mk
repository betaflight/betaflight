F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_spi_bmp280.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_ms56xx.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_mag3110.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/pitotmeter_ms4525.c \
            drivers/pitotmeter_adc.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stdperiph.c \
            drivers/serial_usb_vcp.c \
            drivers/max7456.c
