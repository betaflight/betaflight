F405_TARGETS    += $(TARGET)
FEATURES        += VCP SDCARD HIGHEND

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_mag3110.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/max7456.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stdperiph.c
#            drivers/accgyro/accgyro_spi_icm20689.c \
#            drivers/transponder_ir.c \
#            io/transponder_ir.c
