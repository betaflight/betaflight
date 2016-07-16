F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_spi_bmp280.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f30x.c \
            drivers/serial_usb_vcp.c \
            drivers/transponder_ir.c \
            drivers/transponder_ir_stm32f30x.c \
            io/transponder_ir.c \
            drivers/max7456.c \
            io/osd.c
