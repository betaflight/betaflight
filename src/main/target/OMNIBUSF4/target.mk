F405_TARGETS   += $(TARGET)

ifeq ($(TARGET), CL_RACINGF4)
FEATURES       = VCP SDCARD
else
FEATURES       += VCP ONBOARDFLASH
endif

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_spi_bmp280.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/max7456.c
