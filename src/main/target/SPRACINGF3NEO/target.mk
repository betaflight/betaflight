F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_ms5611.c \
            drivers/compass_ak8975.c \
            drivers/compass_hmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f30x.c \
            drivers/serial_usb_vcp.c \
            drivers/transponder_ir.c \
            drivers/transponder_ir_stm32f30x.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705.c \
            io/osd.c \
            io/transponder_ir.c \
            io/vtx.c
