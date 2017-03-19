F405_TARGETS   += $(TARGET)

ifeq ($(TARGET), CL_RACINGF4)
FEATURES       = VCP SDCARD
else
FEATURES       += VCP ONBOARDFLASH
endif

TARGET_SRC = \
            drivers/accgyro_spi_mpu6000.c \
            drivers/barometer_ms5611.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_spi_bmp280.c \
            drivers/compass_hmc5883l.c \
            drivers/max7456.c
