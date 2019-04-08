ifeq ($(TARGET), KAKUTEF7V2)
F7X5XI_TARGETS += $(TARGET)
else
F7X5XG_TARGETS += $(TARGET)
endif

ifeq ($(TARGET), KAKUTEF7MINI)
FEATURES       += VCP ONBOARDFLASH
else
FEATURES       += SDCARD_SPI VCP
endif

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/max7456.c
