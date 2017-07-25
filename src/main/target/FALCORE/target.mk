F3_TARGETS  += $(TARGET)
HSE_VALUE   = 12000000
FEATURES    = VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_mag3110.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/barometer/barometer_ms56xx.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stdperiph.c

