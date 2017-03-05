F1_TARGETS  += $(TARGET)
FEATURES     = ONBOARDFLASH HIGHEND

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_ms56xx.c \
            drivers/compass_hmc5883l.c \
            drivers/flash_m25p16.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f10x.c \
            io/flashfs.c \
            hardware_revision.c
